/*
 * SHARP Brain keyboard matrix
 *
 * Wiring from keybd_EDNA2.dll (MAIN NK VA 0xc0872cc0 keymap):
 *
 *   columns (driven, outputs): GPIO4 pins 0,1,2,3,4,6,7
 *   rows    (read,   inputs):  GPIO2 pins 16,17,18,19,20,21 (rows 0..5)
 *                              GPIO4 pin 8                    (row 6)
 *
 * Scanning drives one column LOW at a time and reads the 7 row inputs.
 * A pressed key ties its row to the driven column (row reads LOW).
 *
 * ICOLL 63 is the MCU doorbell (guest pulses INTR63.SOFTIRQ).  Do not
 * drive that line from the matrix.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/arm/mxs.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "ui/console.h"
#include "ui/input.h"
#include "system/runstate.h"

#define BRAIN_KBD_COLS  8
#define BRAIN_KBD_ROWS  7

/* DLL scan order 0,1,2,3,4,6,7; pin 5 last so Q/J/A stay on pin 6. */
static const int brain_kbd_col_pins[BRAIN_KBD_COLS] = { 0, 1, 2, 3, 4, 6, 7, 5 };
#define BRAIN_KBD_ROW0_GPIO2_PIN   16
#define BRAIN_KBD_ROW6_GPIO4_PIN   8

/*
 * Guest walks GPIO4 columns from a poll thread.  A one-timeslice host
 * tap is gone before the walk.  2.5 s virtual hold is the S22-verified
 * value: with it `sendkey y` reaches the WCEPRJ [Y] dialog answer on
 * repaired4.  (A 30 s hold keeps the row low for too long and the
 * guest sees an endless key repeat instead of a press/release pair.)
 */
/*
 * PS/2 (hw/input/ps2.c) applies make/break immediately.  The matrix
 * has no scancode queue, so a same-tick HMP sendkey (down+up before
 * the guest scans) would vanish.  Hold the row only until the next
 * 1 ms virtual slice — the same order as i8042's kbd-throttle
 * (pckbd.c: 1000 us) — then release.  Real SDL up events after the
 * guest has had time to poll are applied at once, like PS/2.
 */
#define BRAIN_KBD_MIN_HOLD_NS  (1000 * 1000LL)

/*
 * EDNA2 MCU touchkey report (mailbox +0x404, consumed by VMCopy.dll's
 * touchkey reader / sx8650_touchscreen.dll's touchkey handler):
 *
 *   bit 0x10     = touchkey data valid/ready (always set by the MCU)
 *   bits 0x2, 0x4, 0x8, 0x20, 0x40, 0x100, 0x200, 0x400, 0x800
 *                = per-key PRESSED state (1 = pressed)
 *
 * On the real Brain the EDNA2 MCU scans the touchkeys and posts this
 * word into the mailbox continuously; the host wake path is the MCU
 * attention IRQ (ICOL 33, the same `edna2_int` line).  Map the nine
 * slots onto the QEMU key events that make sense for the menu
 * navigation keys.  The exact physical assignment is not documented,
 * so the mapping is provisional and calibrated empirically against
 * the APPMAIN menu behaviour.
 */
#define BRAIN_TOUCHKEY_VALID    0x10u
/*
 * User HID dump (VK + PS/2 Set 2): ホーム=F1, 国語漢字=F2, 英和和英=F3,
 * My辞書=F7, 履歴=F8, 一覧から選ぶ=Home, 音声=F11.
 */
static const uint32_t brain_touchkey_bits[] = {
    0x2, 0x4, 0x8, 0x20, 0x40,
    0x100, 0x200, 0x400, 0x800,
};

static const QKeyCode brain_touchkey_qcodes[] = {
    Q_KEY_CODE_F1,            /* ホーム VK 0x70 Set2 0x05 */
    Q_KEY_CODE_F2,            /* 国語 漢字 VK 0x71 Set2 0x06 */
    Q_KEY_CODE_F3,            /* 英和 和英 VK 0x72 Set2 0x04 */
    Q_KEY_CODE_F7,            /* My 辞書 VK 0x76 Set2 0x83 */
    Q_KEY_CODE_F8,            /* 履歴 VK 0x77 Set2 0x0A */
    Q_KEY_CODE_F11,           /* 音声 VK 0x7A Set2 0x78 */
    Q_KEY_CODE_PGUP,          /* 音量+ VK 0x21 */
    Q_KEY_CODE_PGDN,          /* 音量- VK 0x22 */
    Q_KEY_CODE_HOME,          /* 一覧から選ぶ VK 0x24 Set2 0x6C */
};

typedef struct BrainKbdState {
    SysBusDevice parent_obj;

    DeviceState *pinctrl;
    uint8_t want[BRAIN_KBD_COLS];
    uint8_t state[BRAIN_KBD_COLS];
    QEMUTimer *hold_timer;
    int64_t hold_until_ns;
    QEMUTimer *edna2_pulse_timer;
    qemu_irq edna2_int;
    bool power;   /* power-key sense line (GPIO0 pin 16) held high */
    uint32_t *touchkey_state;   /* points at bms->edna2_touchkey */
    uint32_t *touchkey_mb;      /* points at mailbox +0x404 word */
} BrainKbdState;

OBJECT_DECLARE_SIMPLE_TYPE(BrainKbdState, BRAIN_KBD)

static bool brain_kbd_debug(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("BRAIN_KBD_DEBUG");

        on = e && *e && *e != '0';
    }
    return on;
}

static void brain_kbd_touchkey_update(BrainKbdState *s, QKeyCode qcode,
                                      bool down);

/*
 * DLL table is AT Set-1 (Esc=0x01 A=0x1e J=0x24 Enter=0x1c).
 * HID dump is Set-2 (Q=0x15 = Set-1 Y).  Use XT/Set-1 from
 * qemu_input_key_value_to_number (physical, JIS-safe).
 * Columns: GPIO4 0,1,2,3,4,6,7 then extra pin 5.
 */
static const uint8_t brain_keymap[BRAIN_KBD_COLS][BRAIN_KBD_ROWS] = {
    { 0x16, 0x08, 0x19, 0x25, 0x03, 0x04, 0x4d },
    { 0x0d, 0x0b, 0x21, 0x27, 0x2a, 0x06, 0x05 },
    { 0x02, 0x0a, 0x15, 0x28, 0x31, 0x18, 0x50 },
    { 0x0e, 0x09, 0x2e, 0x26, 0x2b, 0x11, 0x1d },
    { 0x2c, 0x07, 0x14, 0x29, 0x2d, 0x01, 0x1c },
    { 0x10, 0x0f, 0x22, 0x24, 0x1a, 0x17, 0x1e },
    { 0x1b, 0x0c, 0x12, 0x2f, 0x30, 0x13, 0x1f },
    { 0x20, 0x23, 0x32, 0x39, 0x4b, 0x48, 0x47 },
};

static uint8_t brain_host_set1(const KeyValue *kv)
{
    int n = qemu_input_key_value_to_number(kv);
    QKeyCode q = qemu_input_key_value_to_qcode(kv);

    switch (q) {
    case Q_KEY_CODE_1: case Q_KEY_CODE_KP_1: return 0x10;
    case Q_KEY_CODE_2: case Q_KEY_CODE_KP_2: return 0x11;
    case Q_KEY_CODE_3: case Q_KEY_CODE_KP_3: return 0x12;
    case Q_KEY_CODE_4: case Q_KEY_CODE_KP_4: return 0x13;
    case Q_KEY_CODE_5: case Q_KEY_CODE_KP_5: return 0x14;
    case Q_KEY_CODE_6: case Q_KEY_CODE_KP_6: return 0x15;
    case Q_KEY_CODE_7: case Q_KEY_CODE_KP_7: return 0x16;
    case Q_KEY_CODE_8: case Q_KEY_CODE_KP_8: return 0x17;
    case Q_KEY_CODE_9: case Q_KEY_CODE_KP_9: return 0x18;
    case Q_KEY_CODE_0: case Q_KEY_CODE_KP_0: return 0x19;
    case Q_KEY_CODE_F12:
    case Q_KEY_CODE_GRAVE_ACCENT:
    case Q_KEY_CODE_MUHENKAN:
    case Q_KEY_CODE_INSERT:
        return 0x29;
    case Q_KEY_CODE_HENKAN:
        return 0x39;
    default:
        break;
    }
    if (n <= 0) {
        return 0;
    }
    return n & 0x7f;
}

static bool brain_set1_to_cell(uint8_t sc, int *col, int *row)
{
    int c, r;

    if (!sc) {
        return false;
    }
    for (c = 0; c < BRAIN_KBD_COLS; c++) {
        for (r = 0; r < BRAIN_KBD_ROWS; r++) {
            if (brain_keymap[c][r] == sc) {
                *col = c;
                *row = r;
                return true;
            }
        }
    }
    return false;
}

static void brain_kbd_refresh(void *opaque);

/*
 * EDNA2 MCU attention pulse.  On real hardware the EDNA2 MCU scans
 * the keyboard matrix as well and raises its interrupt line (ICOLL
 * 33) on any key activity; the WinCE keybd_EDNA2 ISTs (ISRMainProc /
 * ISRKeyIoProc) are kicked by that interrupt.  The matrix state that
 * brain_kbd drives on the GPIO pins is consumed by the poll scan
 * (0xc0878664), but the interrupt-driven path (hevKey/ThreadProc /
 * SetDirectKey) only advances when ICOLL 33 fires.
 *
 * The line is HELD asserted for the full key-hold (not a short blip):
 * the WinCE keybd_EDNA2 IST can be woken from a deep idle (SRAM
 * clock-switch WFI at 0x327c) only if the ICOLL 33 level is still up
 * when the CPU gets around to reading the vector.  A 100 us auto-clear
 * can expire before the guest re-schedules, silently dropping the key
 * (key tap registered in QEMU but the date dialog never reacts).  The
 * interrupt is therefore deasserted together with the key release in
 * the hold timer, matching the real MCU holding its attention line
 * until the scan confirms the key was read.
 */
static void brain_kbd_edna2_pulse_tick(void *opaque)
{
    BrainKbdState *s = opaque;

    qemu_set_irq(s->edna2_int, 0);
}

static void brain_kbd_edna2_pulse(BrainKbdState *s)
{
    qemu_set_irq(s->edna2_int, 1);
}

/*
 * Timed attention pulse used by the touch path (brain_kbd_edna2_pulse_ext),
 * which has no matrix key to release the line.  A touch wakes the panel
 * PDD; a short pulse is enough and the line is auto-cleared so it does
 * not stay asserted forever.
 */
static void brain_kbd_edna2_timed_pulse(BrainKbdState *s)
{
    qemu_set_irq(s->edna2_int, 1);
    timer_mod(s->edna2_pulse_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000);
}

/*
 * The EDNA2 MCU watches the touch panel detect line in parallel with
 * the host (on the real Brain it owns the panel wake path).  A touch
 * therefore raises the MCU attention line (ICOL 33) just like a key
 * press does, waking the host even when the touch PDD is in its OFF
 * power state.
 */
void brain_kbd_edna2_pulse_ext(DeviceState *kbd)
{
    BrainKbdState *s = BRAIN_KBD(kbd);

    brain_kbd_edna2_timed_pulse(s);
}

static void brain_kbd_panel_wake(void *opaque, int n, int level)
{
    if (level) {
        brain_kbd_edna2_timed_pulse(BRAIN_KBD(opaque));
    }
}

/*
 * Power-key sense line: the keybd_EDNA2 key processor reads DIN0 bit 16
 * (0xc08787e4: tst r3,#0x10000 -> keydata byte 0x1c) and treats a SET
 * bit as "power key pressed".  On the real Brain the EDNA2 MCU holds
 * this line low at rest and raises it when the power key is used; the
 * QEMU pinctrl default (ext=all-ones) would read as permanently
 * pressed, keeping ISRMainProc's debounce loop spinning forever (r7=1)
 * so real matrix keys are never delivered.  brain_kbd therefore drives
 * the line: 0 at rest, 1 while the POWER key is held.
 */
#define BRAIN_KBD_POWER_PIN  16   /* GPIO0 pin 16 */

static void brain_kbd_hold_tick(void *opaque)
{
    BrainKbdState *s = opaque;
    bool changed = false;
    int c;

    for (c = 0; c < BRAIN_KBD_COLS; c++) {
        uint8_t drop = s->state[c] & ~s->want[c];

        if (drop) {
            s->state[c] &= ~drop;
            changed = true;
        }
    }
    if (changed) {
        brain_kbd_refresh(s);
        /* The key was released: drop the EDNA2 attention line.  It was
         * held high for the whole key-hold so a deep-idle guest always
         * has time to service the ICOLL 33 wake. */
        qemu_set_irq(s->edna2_int, 0);
    }
}

static void brain_kbd_refresh(void *opaque)
{
    BrainKbdState *s = opaque;
    uint32_t doe4, dout4;
    uint32_t din2 = 0xffffffffu, din4 = 0xffffffffu;
    uint32_t driven_cols = 0;
    int c, r;

    if (!s->pinctrl) {
        return;
    }

    doe4  = mxs_pinctrl_get_doe(s->pinctrl, 4);
    dout4 = mxs_pinctrl_get_dout(s->pinctrl, 4);

    for (c = 0; c < BRAIN_KBD_COLS; c++) {
        int pin = brain_kbd_col_pins[c];
        if (((doe4 >> pin) & 1) && !((dout4 >> pin) & 1)) {
            driven_cols |= 1u << c;
        }
    }

    for (c = 0; c < BRAIN_KBD_COLS; c++) {
        if (!(driven_cols & (1u << c))) {
            continue;
        }
        for (r = 0; r < BRAIN_KBD_ROWS; r++) {
            if (!(s->state[c] & (1u << r))) {
                continue;
            }
            if (r < 6) {
                din2 &= ~(1u << (BRAIN_KBD_ROW0_GPIO2_PIN + r));
            } else {
                din4 &= ~(1u << BRAIN_KBD_ROW6_GPIO4_PIN);
            }
        }
    }

    /*
     * Row-pin DIN update with full IRQ latching.
     *
     * The keybd_EDNA2 driver registers InterruptInitialize(33, ...):
     * the OAL's 28-byte sysintr table entry for SYSINTR 33 lists the
     * seven keyboard ROW pins (GPIO2 pins 16-21 = 0xd0-0xd5, GPIO4
     * pin 8 = 0x108 in the 128+bank*32+pin encoding), and its enable
     * path arms PIN2IRQ2 bits 16-21 / PIN2IRQ4 bit 8 plus ICOLL 125 /
     * 123 (verified at runtime: PIN2IRQ2=0x003f0000, PIN2IRQ4=
     * 0x00000100, IRQLEVEL=level, IRQPOL=0 active-low).  The
     * OEMInterruptHandler maps the GPIO bit to a row-pin sysintr
     * (0xd0-0xd5 / 0x108) and OALIntrTranslateIrq maps those to the
     * logical IRQ 33, whose registered event wakes ISRKeyIoProc.
     *
     * The old poll-only path (set_din_poll) was an S22 workaround
     * for the era when InterruptInitialize(33) never ran (driver
     * init failure) so PIN2IRQ was 0 and any IRQSTAT latch produced
     * an "undefined IRQ" storm.  With the init fixed, the pins are
     * properly armed and the level-sensitive latch is the correct
     * hardware behaviour: pressing a key pulls the row pin low
     * (level pins mirror the active input level in IRQSTAT), which
     * raises the GPIO bank IRQ until the guest clears IRQSTAT.
     */
    mxs_pinctrl_set_din(s->pinctrl, 2, din2);
    mxs_pinctrl_set_din(s->pinctrl, 4, din4);
    mxs_pinctrl_set_din(s->pinctrl, 0,
                        s->power ? 0xffffffffu
                                 : (0xffffffffu & ~(1u << BRAIN_KBD_POWER_PIN)));

    if (brain_kbd_debug()) {
        fprintf(stderr, "[brain-kbd] refresh driven_cols=%02x doe4=%08x "
                "dout4=%08x din2=%08x din4=%08x\n",
                driven_cols, doe4, dout4, din2, din4);
    }
}

static void brain_kbd_event(DeviceState *dev, QemuConsole *src,
                            InputEvent *evt)
{
    BrainKbdState *s = BRAIN_KBD(dev);
    InputKeyEvent *key;
    QKeyCode qcode;
    int c, r;

    if (evt->type != INPUT_EVENT_KIND_KEY) {
        return;
    }
    key = evt->u.key.data;
    qcode = qemu_input_key_value_to_qcode(key->key);

    /*
     * Brain 電源 is GPIO0.16, not a matrix scancode and not host Power
     * (that suspends the PC).  Pause/Break is on JIS and US boards;
     * AC_Bookmarks is not a keycap on a JIS 106.
     */
    /* 電源 dump unknown.  F12 is 記号 (VK 0x7B).  Not host Power. */
    if (qcode == Q_KEY_CODE_PAUSE || qcode == Q_KEY_CODE_STOP) {
        s->power = key->down;
        brain_kbd_refresh(s);
        qemu_set_irq(s->edna2_int, key->down ? 1 : 0);
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
        return;
    }

    if (!brain_set1_to_cell(brain_host_set1(key->key), &c, &r)) {
        brain_kbd_touchkey_update(s, qcode, key->down);
        return;
    }

    if (brain_kbd_debug()) {
        fprintf(stderr, "[brain-kbd] event qcode=%d set1=0x%02x cell=%d,%d down=%d\n",
                qcode, brain_host_set1(key->key), c, r, key->down);
    }

    qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);

    if (key->down) {
        s->want[c] |= 1u << r;
        s->state[c] |= 1u << r;
        brain_kbd_refresh(s);
        brain_kbd_edna2_pulse(s);
        s->hold_until_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                           BRAIN_KBD_MIN_HOLD_NS;
    } else {
        s->want[c] &= ~(1u << r);
        if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= s->hold_until_ns) {
            brain_kbd_hold_tick(s);
            return;
        }
    }
    timer_mod(s->hold_timer, s->hold_until_ns);
}

static const QemuInputHandler brain_kbd_handler = {
    .name  = "SHARP Brain keyboard",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = brain_kbd_event,
};

void brain_kbd_set_touchkey_state(DeviceState *kbd, uint32_t *state_ptr,
                                 uint32_t *mb_word)
{
    BrainKbdState *s = BRAIN_KBD(kbd);

    s->touchkey_state = state_ptr;
    s->touchkey_mb = mb_word;
}

static void brain_kbd_touchkey_update(BrainKbdState *s, QKeyCode qcode,
                                      bool down)
{
    int i;

    if (!s->touchkey_state) {
        return;
    }
    for (i = 0; i < ARRAY_SIZE(brain_touchkey_qcodes); i++) {
        if (brain_touchkey_qcodes[i] != qcode) {
            continue;
        }
        if (down) {
            *s->touchkey_state |= brain_touchkey_bits[i];
        } else {
            *s->touchkey_state &= ~brain_touchkey_bits[i];
        }
        /*
         * The real EDNA2 MCU keeps the mailbox word current while the
         * host polls it through VMCopy.dll (0xc05f2d24 ff.).  Mirror
         * the state into the mailbox with the 0x10 valid bit.
         */
        if (s->touchkey_mb) {
            *s->touchkey_mb = BRAIN_TOUCHKEY_VALID | *s->touchkey_state;
        }
        return;
    }
}

void brain_kbd_set_pinctrl(DeviceState *kbd, DeviceState *pinctrl)
{
    BrainKbdState *s = BRAIN_KBD(kbd);

    s->pinctrl = pinctrl;
    mxs_pinctrl_set_notify(pinctrl, brain_kbd_refresh, s);
    brain_kbd_refresh(s);
}

static void brain_kbd_realize(DeviceState *dev, Error **errp)
{
    BrainKbdState *s = BRAIN_KBD(dev);

    s->hold_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, brain_kbd_hold_tick, s);
    s->edna2_pulse_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        brain_kbd_edna2_pulse_tick, s);
    /* power-key sense line: low at rest, before the guest ever scans */
    s->power = false;
    mxs_pinctrl_set_din(s->pinctrl, 0,
                        0xffffffffu & ~(1u << BRAIN_KBD_POWER_PIN));
    if (s->pinctrl) {
        mxs_pinctrl_set_notify(s->pinctrl, brain_kbd_refresh, s);
        brain_kbd_refresh(s);
    }
    qemu_input_handler_register(dev, &brain_kbd_handler);
}

static void brain_kbd_reset(DeviceState *dev)
{
    BrainKbdState *s = BRAIN_KBD(dev);

    memset(s->want, 0, sizeof(s->want));
    memset(s->state, 0, sizeof(s->state));
    if (s->hold_timer) {
        timer_del(s->hold_timer);
    }
    if (s->edna2_pulse_timer) {
        timer_del(s->edna2_pulse_timer);
    }
}

static void brain_kbd_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);

    object_property_add_link(obj, "pinctrl", TYPE_DEVICE,
                             (Object **)&BRAIN_KBD(obj)->pinctrl,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    qdev_init_gpio_out_named(dev, &BRAIN_KBD(obj)->edna2_int, "edna2-int", 1);
    qdev_init_gpio_in_named(dev, brain_kbd_panel_wake, "panel-wake", 1);
}

static const VMStateDescription vmstate_brain_kbd = {
    .name = "brain-keyboard",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(want, BrainKbdState, BRAIN_KBD_COLS),
        VMSTATE_UINT8_ARRAY(state, BrainKbdState, BRAIN_KBD_COLS),
        VMSTATE_END_OF_LIST()
    }
};

static void brain_kbd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = brain_kbd_realize;
    device_class_set_legacy_reset(dc, brain_kbd_reset);
    dc->vmsd = &vmstate_brain_kbd;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo brain_kbd_types[] = {
    {
        .name           = TYPE_BRAIN_KBD,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(BrainKbdState),
        .instance_init  = brain_kbd_init,
        .class_init     = brain_kbd_class_init,
    },
};

DEFINE_TYPES(brain_kbd_types)
