/*
 * SHARP Brain keyboard matrix
 *
 * Wiring, decoded from keybd_EDNA2.dll (MAIN NK XIP ROM):
 *
 *   columns (driven, outputs): GPIO4 pins 0,1,2,3,4,6,7
 *   rows    (read,   inputs):  GPIO2 pins 16,17,18,19,20,21 (rows 0..5)
 *                              GPIO4 pin 8                    (row 6)
 *
 * The scan routine (VA 0xc0878664) walks r7 = 1,2,4,...,0x40 and calls
 * VA 0xc087847c, which ORs the remap table at VA 0xc0872ca4
 * { 0x01,0x02,0x04,0x08,0x10,0x40,0x80 } into HW_PINCTRL_DOUT4_CLR
 * (+0x748) / DOE4_SET (+0xb44); DOE4_CLR (+0xb48) = 0xDF releases the
 * columns again.  So DLL column 0..6 = GPIO4 pin 0,1,2,3,4,6,7 and GPIO4
 * pin 5 is *not* a matrix line.  Each column read builds a 7-bit row
 * mask (VA 0xc0878708):
 *
 *   mask bit i (i=0..5) = ~DIN2 bit (16+i)     (row i, active low)
 *   mask bit 6          = ~DIN4 bit 8          (row 6, active low)
 *
 * A pressed key ties its row to the driven (low) column, so a row that
 * reads LOW while its column is driven means "pressed".
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

#define BRAIN_KBD_COLS  7
#define BRAIN_KBD_ROWS  7

/*
 * The driver scans these seven GPIO4 columns in this order (remap table
 * at VA 0xc0872ca4, driven through DOUT4_CLR/DOE4_SET).  GPIO4 pin 5 is
 * driven low by another driver but is not part of the key matrix.
 */
static const int brain_kbd_col_pins[BRAIN_KBD_COLS] = { 0, 1, 2, 3, 4, 6, 7 };
#define BRAIN_KBD_ROW0_GPIO2_PIN   16
#define BRAIN_KBD_ROW6_GPIO4_PIN   8

/*
 * MRSensor (edna2_MRSensor.dll) lines on GPIO0 pins 6 and 7.
 *
 * Reverse-engineering evidence (S97-B, nk_full.bin + boot trace):
 *   - edna2_MRSensor.dll imports only DDKGpioClearIntrPin/Config/
 *     ReadDataPin from cspddk.dll (no I2C / LRADC / EDNA2 mailbox).
 *   - HKLM\Drivers\BuiltIn\MRSensor has only Order/Index/Prefix/Dll
 *     (no GPIO/IRQ values) -- the pins are hard-coded in the driver.
 *   - Boot trace (MXS_TRACE=pinctrl + BRAIN_BWATCH on the cspddk GPIO
 *     workers): between the "BSPBacklightInitialize()" and the
 *     "MRS_Init() status1 = 1, status2 = 1" prints the MRSensor driver
 *     (caller context pointers r4/r5/r1 all inside 0xc0893xxx = the
 *     DLL's own data) clears DOE0 for bits 0x40/0x80, arms
 *     IRQEN0/IRQLEVEL0/IRQPOL0 for bits 6 and 7, reads DIN0 (0x900)
 *     twice and the kernel then sets PIN2IRQ0 bits 6 and 7.  The two
 *     configured lines match the two "status" results MRS_Init prints.
 *
 * So the device is a two-line sensor read through GPIO0 pins 6/7 with a
 * level IRQ.  On the real board the sensor part is not identifiable
 * from public material (no DT node, no teardown data); this model
 * drives the two GPIO lines and lets the guest's level-IRQ path fire.
 * Lines are high at rest (matches the boot DIN0 read 0xed00ffe0).
 */
#define BRAIN_MRS_GPIO0_PINS   ((1u << 6) | (1u << 7))

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
 *   bit 0x80     = scan-in-progress flag (clear while the pads are idle)
 *   bits 0x2, 0x4, 0x8, 0x20, 0x40, 0x100, 0x200, 0x400, 0x800
 *                = per-key PRESSED state, ACTIVE-LOW (bit clear = pressed)
 *
 * On the real Brain the EDNA2 MCU scans the touchkeys on its own cycle
 * and posts the sampled state into the mailbox; between two scans the
 * word simply keeps the last sample.  The host wake path is the MCU
 * attention IRQ (ICOL 33, the same `edna2_int` line), asserted by the
 * scan that first sees a pad and dropped by the scan that finds the pads
 * released.  With no pad touched the word is 0xf7e: the 0x10 valid bit
 * and all nine pad bits set, the 0x80 scan bit clear.  A pressed pad
 * clears its bit.  The word encoding is shared with the mailbox seed in
 * hw/arm/mxs.c, so the constants live in include/hw/arm/mxs.h.
 */
/*
 * The exact physical assignment of the nine pads is not documented, so
 * the mapping is provisional and calibrated empirically against the
 * APPMAIN menu behaviour (see the strip pad ordering in mxs_lradc.c).
 * Bit layout re-checked against a real unit where possible.
 */
static const uint32_t brain_touchkey_bits[] = {
    0x2, 0x4, 0x8, 0x20, 0x40,
    0x100, 0x200, 0x400, 0x800,
};

/*
 * Host keys that double as touchkeys for scripted navigation (kept for
 * the QMP/send-key harnesses).  Only the first three pads have a fixed
 * host binding; the remaining six are reached through the strip.
 */
static const QKeyCode brain_touchkey_qcodes[] = {
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
    QEMUTimer *attn_timer;
    qemu_irq edna2_int;
    bool power;   /* power-key sense line (GPIO0 pin 16) held high */
    bool attn;    /* MCU attention: row 0 briefly pulled low */
    bool mod_ctrl; /* host Ctrl held (chord prefix, never forwarded) */
    bool mod_alt;  /* host Alt held (chord prefix, never forwarded) */
    uint32_t *touchkey_state;   /* points at bms->edna2_touchkey */
    uint32_t *touchkey_mb;      /* points at mailbox +0x404 word */
    uint64_t *touchkey_mb_reads; /* mailbox reads, all of them             */
    uint64_t *touchkey_tk_reads; /* reads that hit the +0x404 report word   */
    uint32_t touchkey_want;     /* pads the finger is actually on          */
    uint32_t touchkey_pub;      /* what the MCU's last finished scan saw   */
    QEMUTimer *touchkey_scan_timer; /* the MCU key scan cycle              */
    bool touchkey_scan_busy;    /* a conversion is running (report 0x80)   */
    uint32_t mrs_low;           /* MRSensor lines pulled low on GPIO0
                                 * (only BRAIN_MRS_GPIO0_PINS bits used) */
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
static void brain_kbd_touchkey_publish(BrainKbdState *s, const char *why);
static void brain_kbd_touchkey_release_irq(BrainKbdState *s);

/*
 * The MCU does not watch the pads continuously: it scans them.  SCAN_NS is
 * the interval between two scans and BUSY_NS how long one conversion takes;
 * while a conversion runs the report word carries the "scan in progress"
 * flag (bit 0x80), which is what that bit is for on the part.  A touch is
 * therefore visible for whole scan cycles around the finger's own press and
 * release, and a tap shorter than one cycle is not reported at all -- the
 * same behaviour the hardware has, and the reason the model deliberately
 * keeps no "hold until the guest looked" latch: such a latch turns a
 * position the guest never touched into a key press.
 */
#define BRAIN_KBD_TOUCHKEY_SCAN_NS  (10 * 1000 * 1000LL)
#define BRAIN_KBD_TOUCHKEY_BUSY_NS  (1 * 1000 * 1000LL)

/*
 * Host key -> matrix cell.
 *
 * keybd_EDNA2.dll decodes the matrix in TWO stages, and both must be
 * modelled to get the right key:
 *
 *   1. VA 0xc0878604 walks the pressed cells and emits, per cell,
 *      byte table49[col*7 + row] from the 49-byte table at VA 0xc0872cc0.
 *      Those bytes are NOT PS/2 Set-1 scancodes: they are 1..49 key
 *      indices (the set {1..49} minus 28/32/35, 46 values for the 46
 *      populated cells).
 *   2. The polling routine (VA 0xc087311c) then does
 *          VK = keytable[layout * 50 + index]
 *      (VA 0xc087329c: `and r3,r2,#0x7f; mla r3,r1,sl,r3; ldr r1,[sb,r3,lsl#2]`,
 *      sl = 50) against the 3 x 50-entry table at VA 0xc087a148, whose
 *      entries are PS/2 Set-2 codes.
 *
 * So cell -> index -> Set-2.  Applying both stages to all 46 populated
 * cells yields exactly one complete keyboard with no duplicates:
 * A-Z, '-', Enter, Backspace, Space, Esc, CapsLock, the four arrows,
 * Home, PgUp, PgDn and F1/F2/F3/F7/F8/F11/F12.  Treating stage 1 as
 * Set-1 instead (the old map) scrambles every key: e.g. the old
 * J -> (5,3) cell holds index 36 = Set-2 0x5a = Enter, and the old
 * A -> (5,6) cell holds index 30 = Set-2 0x06 = F2.
 *
 * Independent cross-check: the user HID dump lists ホーム=F1 (Set2 0x05),
 * 国語漢字=F2 (0x06), 英和和英=F3 (0x04), My辞書=F7 (0x83), 履歴=F8 (0x0A),
 * 音声=F11 (0x78), 一覧から選ぶ=Home (E0 6C) - all seven land on exactly
 * the cells this table assigns them.
 *
 * Keys with no Brain keycap (digits, punctuation, Tab, Ctrl, Alt,
 * F4/F5/F6/F9/F10) are deliberately left unmapped: on the real
 * machine digits are a symbol-layer combination produced through the
 * EDNA2 touchkey path, not a matrix cell, and inventing a cell for them
 * would bypass the driver.
 */
typedef struct BrainKbdCell {
    QKeyCode key;
    int      col;
    int      row;
} BrainKbdCell;

static const BrainKbdCell brain_kbd_keymap[] = {
    { Q_KEY_CODE_V,            0, 0 },  /* code 22 = Set2 0x002a */
    { Q_KEY_CODE_H,            0, 1 },  /* code  8 = Set2 0x0033 */
    { Q_KEY_CODE_Y,            0, 2 },  /* code 25 = Set2 0x0035 */
    { Q_KEY_CODE_ESC,          0, 3 },  /* code 37 = Set2 0x0076 */
    { Q_KEY_CODE_C,            0, 4 },  /* code  3 = Set2 0x0021 */
    { Q_KEY_CODE_D,            0, 5 },  /* code  4 = Set2 0x0023 */
    { Q_KEY_CODE_M,            1, 0 },  /* code 13 = Set2 0x003a */
    { Q_KEY_CODE_K,            1, 1 },  /* code 11 = Set2 0x0042 */
    { Q_KEY_CODE_DOWN,         1, 3 },  /* code 39 = Set2 0xe072 */
    { Q_KEY_CODE_PGUP,         1, 4 },  /* code 42 = Set2 0xe07d */
    { Q_KEY_CODE_F,            1, 5 },  /* code  6 = Set2 0x002b */
    { Q_KEY_CODE_E,            1, 6 },  /* code  5 = Set2 0x0024 */
    { Q_KEY_CODE_B,            2, 0 },  /* code  2 = Set2 0x0032 */
    { Q_KEY_CODE_J,            2, 1 },  /* code 10 = Set2 0x003b */
    { Q_KEY_CODE_U,            2, 2 },  /* code 21 = Set2 0x003c */
    { Q_KEY_CODE_LEFT,         2, 3 },  /* code 40 = Set2 0xe06b */
    { Q_KEY_CODE_X,            2, 5 },  /* code 24 = Set2 0x0022 */
    { Q_KEY_CODE_N,            3, 0 },  /* code 14 = Set2 0x0031 */
    { Q_KEY_CODE_I,            3, 1 },  /* code  9 = Set2 0x0043 */
    { Q_KEY_CODE_UP,           3, 3 },  /* code 38 = Set2 0xe075 */
    { Q_KEY_CODE_PGDN,         3, 4 },  /* code 43 = Set2 0xe07a */
    { Q_KEY_CODE_Q,            3, 5 },  /* code 17 = Set2 0x0015 */
    { Q_KEY_CODE_BACKSPACE,    4, 0 },  /* code 44 = Set2 0x0066 */
    /* The keycap of this cell reads 削除/クリア (Delete/Clear), so the PC
     * Delete key is the same physical key as Backspace. */
    { Q_KEY_CODE_DELETE,       4, 0 },  /* code 44 = Set2 0x0066 */
    { Q_KEY_CODE_G,            4, 1 },  /* code  7 = Set2 0x0034 */
    { Q_KEY_CODE_T,            4, 2 },  /* code 20 = Set2 0x002c */
    { Q_KEY_CODE_RIGHT,        4, 3 },  /* code 41 = Set2 0xe074 */
    /* The keycap of this cell is the physical [シフト] key; the driver
     * reports it as the layer-toggle (Set-2 0x58).  Bind the PC Shift and
     * CapsLock keys to it so the guest's shift layer is reachable. */
    { Q_KEY_CODE_CAPS_LOCK,    4, 4 },  /* code 45 = Set2 0x0058 */
    { Q_KEY_CODE_SHIFT,        4, 4 },  /* code 45 = Set2 0x0058 (シフト) */
    { Q_KEY_CODE_SHIFT_R,      4, 4 },  /* code 45 = Set2 0x0058 (シフト) */
    { Q_KEY_CODE_A,            4, 5 },  /* code  1 = Set2 0x001c */
    { Q_KEY_CODE_P,            5, 0 },  /* code 16 = Set2 0x004d */
    { Q_KEY_CODE_O,            5, 1 },  /* code 15 = Set2 0x0044 */
    { Q_KEY_CODE_HOME,         5, 2 },  /* code 34 = Set2 0xe06c */
    { Q_KEY_CODE_RET,          5, 3 },  /* code 36 = Set2 0x005a */
    { Q_KEY_CODE_KP_ENTER,     5, 3 },  /* code 36 = Set2 0x005a */
    { Q_KEY_CODE_Z,            5, 4 },  /* code 26 = Set2 0x001a */
    { Q_KEY_CODE_W,            5, 5 },  /* code 23 = Set2 0x001d */
    { Q_KEY_CODE_MINUS,        6, 0 },  /* code 27 = Set2 0x004e */
    { Q_KEY_CODE_L,            6, 1 },  /* code 12 = Set2 0x004b */
    { Q_KEY_CODE_R,            6, 2 },  /* code 18 = Set2 0x002d */
    { Q_KEY_CODE_SPC,          6, 3 },  /* code 47 = Set2 0x0029 */
    { Q_KEY_CODE_S,            6, 5 },  /* code 19 = Set2 0x001b */
};

/*
 * The Brain top-row / touch keys (ホーム..記号, 音声) used to sit on
 * single-press PC F-keys, but F1-F12 are live host keys (help, rename,
 * refresh, devtools, Alt+F4, ...), so stealing them gets in the way of
 * PC use.  Serve them as Ctrl+Alt+letter chords instead: Ctrl and Alt
 * have no Brain keycaps and are never forwarded to the guest, and
 * Ctrl+Alt+letter is neither reserved by the host OS nor eaten by the
 * QEMU UI (GTK/SDL bind only Ctrl+Alt+{g,f,m} plus the digits for
 * grab/full-screen/menu-bar/virtual-console switching).  Letters are
 * mnemonic: H=ホーム K=国語漢字 E=英和和英 J=辞書 R=履歴 V=voice S=記号.
 */
static const BrainKbdCell brain_kbd_chords[] = {
    { Q_KEY_CODE_H,            3, 6 },  /* ホーム     = index 29 Set2 0x05 */
    { Q_KEY_CODE_K,            5, 6 },  /* 国語漢字   = index 30 Set2 0x06 */
    { Q_KEY_CODE_E,            6, 6 },  /* 英和和英   = index 31 Set2 0x04 */
    { Q_KEY_CODE_J,            3, 2 },  /* My辞書     = index 46 Set2 0x83 */
    { Q_KEY_CODE_R,            1, 2 },  /* 履歴       = index 33 Set2 0x0a */
    { Q_KEY_CODE_V,            6, 4 },  /* 音声       = index 48 Set2 0x78 */
    { Q_KEY_CODE_S,            2, 4 },  /* 記号       = index 49 Set2 0x07 */
};

static bool brain_qcode_to_cell(QKeyCode q, int *col, int *row)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(brain_kbd_keymap); i++) {
        if (brain_kbd_keymap[i].key == q) {
            *col = brain_kbd_keymap[i].col;
            *row = brain_kbd_keymap[i].row;
            return true;
        }
    }
    return false;
}

static bool brain_qcode_chord_or_cell(BrainKbdState *s, QKeyCode q,
                                      int *col, int *row)
{
    int i;

    if (s->mod_ctrl && s->mod_alt) {
        for (i = 0; i < ARRAY_SIZE(brain_kbd_chords); i++) {
            if (brain_kbd_chords[i].key == q) {
                *col = brain_kbd_chords[i].col;
                *row = brain_kbd_chords[i].row;
                return true;
            }
        }
    }
    return brain_qcode_to_cell(q, col, row);
}

static void brain_kbd_refresh(void *opaque);

/*
 * Power-key attention pulse.  The keybd_EDNA2 poller thread only runs
 * when woken through the SYSINTR-33 event, whose OAL sysintr table arms
 * the seven matrix ROW pins (PIN2IRQ2 bits 16-21 / PIN2IRQ4 bit 8);
 * nothing else wakes it.  On the real Brain the EDNA2 MCU mirrors the
 * power key onto GPIO0 pin 16 AND asserts its attention by pulling a
 * row line, so a lone power press still produces a row-pin IRQ and the
 * poller's raw reader (0xc08787b4) samples the power bit.  Model that
 * MCU attention as a brief (3 ms) row-0 pull-down on every power edge.
 * The raw reader takes two snapshots 5 ms apart, so both snapshots can
 * never fall inside the 3 ms window: a snapshot pair straddling the
 * release mismatches and is retried, and the stable pair that gets
 * decoded sees only the power bit - never a phantom matrix key.
 */
#define BRAIN_KBD_ATTN_HOLD_NS  (3000 * 1000LL)

static void brain_kbd_attn_tick(void *opaque)
{
    BrainKbdState *s = opaque;

    s->attn = false;
    brain_kbd_refresh(s);
}

static void brain_kbd_attn_pulse(BrainKbdState *s)
{
    s->attn = true;
    brain_kbd_refresh(s);
    timer_mod(s->attn_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + BRAIN_KBD_ATTN_HOLD_NS);
}

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

    /* MCU attention: row 0 pulled low for the brief pulse window. */
    if (s->attn) {
        din2 &= ~(1u << BRAIN_KBD_ROW0_GPIO2_PIN);
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
                        ((s->power ? 0xffffffffu
                                   : (0xffffffffu & ~(1u << BRAIN_KBD_POWER_PIN)))
                         & ~(s->mrs_low & BRAIN_MRS_GPIO0_PINS)));

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

    /* Ctrl/Alt are chord prefixes only; the Brain has no such keycaps
     * and the codes must never reach the guest on their own. */
    switch (qcode) {
    case Q_KEY_CODE_CTRL:
    case Q_KEY_CODE_CTRL_R:
        s->mod_ctrl = key->down;
        return;
    case Q_KEY_CODE_ALT:
    case Q_KEY_CODE_ALT_R:
        s->mod_alt = key->down;
        return;
    default:
        break;
    }

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
        /* MCU attention edge: wake the poller via a row-pin IRQ so the
         * raw reader samples the power sense bit (see attn comment). */
        brain_kbd_attn_pulse(s);
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
        return;
    }

    if (!brain_qcode_chord_or_cell(s, qcode, &c, &r)) {
        brain_kbd_touchkey_update(s, qcode, key->down);
        return;
    }

    if (brain_kbd_debug()) {
        fprintf(stderr, "[brain-kbd] event qcode=%d cell=%d,%d down=%d\n",
                qcode, c, r, key->down);
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

/*
 * MRSensor GPIO lines.  `levels` is a 2-bit mask: bit 0 = GPIO0 pin 6,
 * bit 1 = GPIO0 pin 7, 1 = line high, 0 = line low.  A change is pushed
 * to the pinctrl DIN inputs, where the guest's level/edge IRQ detectors
 * (armed by edna2_MRSensor's ISR setup) latch IRQSTAT and raise the
 * GPIO0 bank interrupt.
 */
void brain_kbd_set_mrs(DeviceState *kbd, uint32_t levels)
{
    BrainKbdState *s = BRAIN_KBD(kbd);

    s->mrs_low = (~levels & 0x3) << 6;
    if (s->pinctrl) {
        brain_kbd_refresh(s);
    }
}

uint32_t brain_kbd_get_mrs(DeviceState *kbd)
{
    BrainKbdState *s = BRAIN_KBD(kbd);

    return (~(s->mrs_low >> 6)) & 0x3;
}

/*
 * Write the report the MCU owns: what the last finished scan sampled, plus
 * the "scan in progress" flag while a conversion is running.  The pad bits
 * are active-low (see BRAIN_TOUCHKEY_IDLE above), so the pressed set is the
 * cleared bits of the idle word with the 0x10 valid flag set.  An unchanged
 * report is not rewritten and not logged: the word changes when a scan has
 * sampled something, not when the host happens to look.  The mailbox word is
 * compared against what it actually holds, because the MCU command executor
 * seeds that word too (see brain_edna2_mcu_execute() in hw/arm/mxs.c).
 */
static void brain_kbd_touchkey_publish(BrainKbdState *s, const char *why)
{
    uint32_t pads = s->touchkey_pub;
    uint32_t word = (BRAIN_TOUCHKEY_IDLE & ~pads) |
                    (s->touchkey_scan_busy ? BRAIN_TOUCHKEY_SCAN : 0);

    if (!s->touchkey_state) {
        return;
    }
    if (pads == *s->touchkey_state &&
        (!s->touchkey_mb || word == *s->touchkey_mb)) {
        return;
    }
    *s->touchkey_state = pads;
    if (s->touchkey_mb) {
        *s->touchkey_mb = word;
        fprintf(stderr, "[touchkey] %lld %s pads=0x%03x -> "
                "+0x404=0x%08x mb=%" PRIu64 "/%" PRIu64 "\n",
                (long long)g_get_real_time(), why, pads, word,
                s->touchkey_mb_reads ? *s->touchkey_mb_reads : 0,
                s->touchkey_tk_reads ? *s->touchkey_tk_reads : 0);
    }
}

/*
 * One edge of the MCU's key-scan cycle: open a conversion, and BUSY_NS later
 * sample the pad level and make it the report.  The pads are a level the
 * scanner reads, so a press reaches the guest within one scan cycle and a
 * release leaves the report one cycle after the finger lifted -- and nothing
 * is ever posted that the finger did not actually cover.
 */
static void brain_kbd_touchkey_scan(void *opaque)
{
    BrainKbdState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t prev;
    bool fresh;

    if (!s->touchkey_scan_busy) {
        s->touchkey_scan_busy = true;
        brain_kbd_touchkey_publish(s, "scan start");
        timer_mod(s->touchkey_scan_timer, now + BRAIN_KBD_TOUCHKEY_BUSY_NS);
        return;
    }

    prev = s->touchkey_pub;
    fresh = s->touchkey_want & ~prev;
    s->touchkey_scan_busy = false;
    s->touchkey_pub = s->touchkey_want;
    brain_kbd_touchkey_publish(s, "scan end");

    /*
     * The strip is handled by the EDNA2 MCU, not by the resistive plate, so
     * the plate's panel-wake line never fires.  The scan that first sees a
     * pad is what raises ICOLL 33 (the `edna2_int` line) and asks for a
     * system wake so the keybd_EDNA2 IST (ISRMainProc) wakes and samples the
     * touchkey mailbox word through VMC1: ioctl 0x8000208b.  The line is a
     * level for as long as a pad is held down, so a pending auto-clear from
     * the matrix path must not undo it; the scan that finds the pads
     * released drops it again.
     */
    if (fresh) {
        timer_del(s->edna2_pulse_timer);
        brain_kbd_edna2_pulse(s);
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
    }
    if (!s->touchkey_pub) {
        brain_kbd_touchkey_release_irq(s);
    }

    timer_mod(s->touchkey_scan_timer, now + BRAIN_KBD_TOUCHKEY_SCAN_NS);
}

/* Nothing posted any more: the MCU may drop its attention line. */
static void brain_kbd_touchkey_release_irq(BrainKbdState *s)
{
    if (s->touchkey_state && !*s->touchkey_state) {
        qemu_set_irq(s->edna2_int, 0);
    }
}

/*
 * A finger only ever makes and breaks a level on one pad; that is all the
 * input side of the model does.  The report follows at the next scan.
 */
static void brain_kbd_touchkey_drive(BrainKbdState *s, int index, bool down,
                                     const char *why)
{
    uint32_t bit = brain_touchkey_bits[index];

    if (down) {
        s->touchkey_want |= bit;
    } else {
        s->touchkey_want &= ~bit;
    }
    if (brain_kbd_debug()) {
        fprintf(stderr, "[brain-kbd] %s touchkey %d %s, level 0x%03x\n",
                why, index, down ? "down" : "up", s->touchkey_want);
    }
}

static void brain_kbd_touchkey_press(BrainKbdState *s, int index, bool down)
{
    if (index < 0 || index >= ARRAY_SIZE(brain_touchkey_bits)) {
        return;
    }
    brain_kbd_touchkey_drive(s, index, down, "host");
}

static void brain_kbd_touchkey_update(BrainKbdState *s, QKeyCode qcode,
                                      bool down)
{
    int i;

    if (!s->touchkey_state) {
        return;
    }
    for (i = 0; i < ARRAY_SIZE(brain_touchkey_qcodes); i++) {
        if (brain_touchkey_qcodes[i] == qcode) {
            brain_kbd_touchkey_press(s, i, down);
            return;
        }
    }
}

void brain_kbd_touchkey_strip(DeviceState *kbd, int index, bool down)
{
    BrainKbdState *s = BRAIN_KBD(kbd);

    if (!s->touchkey_state || index < 0 ||
        index >= ARRAY_SIZE(brain_touchkey_bits)) {
        return;
    }
    brain_kbd_touchkey_drive(s, index, down, "strip");
}

void brain_kbd_touchkey_set_mb_counters(DeviceState *kbd, uint64_t *mb,
                                        uint64_t *tk)
{
    BrainKbdState *s;

    if (!kbd) {
        return;
    }
    s = BRAIN_KBD(kbd);
    s->touchkey_mb_reads = mb;
    s->touchkey_tk_reads = tk;
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
    s->attn_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, brain_kbd_attn_tick, s);
    /*
     * The MCU's touchkey scan runs from power-on: the report word is only
     * ever a sampled copy of the pad level, so the scan has to be going
     * whether or not a finger is on the glass.
     */
    s->touchkey_scan_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          brain_kbd_touchkey_scan, s);
    s->touchkey_scan_busy = false;
    timer_mod(s->touchkey_scan_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              BRAIN_KBD_TOUCHKEY_SCAN_NS);
    /* power-key sense line: low at rest, before the guest ever scans */
    s->power = false;
    s->attn = false;
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
    if (s->attn_timer) {
        timer_del(s->attn_timer);
    }
    if (s->touchkey_scan_timer) {
        timer_del(s->touchkey_scan_timer);
    }
    s->touchkey_want = 0;
    s->touchkey_pub = 0;
    s->touchkey_scan_busy = false;
    if (s->touchkey_state) {
        *s->touchkey_state = 0;
    }
    if (s->touchkey_scan_timer) {
        timer_mod(s->touchkey_scan_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  BRAIN_KBD_TOUCHKEY_SCAN_NS);
    }
    s->attn = false;
    s->mrs_low = 0;             /* MRSensor lines high at rest */
    if (s->pinctrl) {
        brain_kbd_refresh(s);
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

static void brain_kbd_post_load(void *opaque, int version_id)
{
    BrainKbdState *s = opaque;

    /*
     * The scan cycle is a free-running timer and timers are not migrated,
     * so restart it; the report itself was migrated with the mailbox page
     * and needs no resync.
     */
    s->touchkey_scan_busy = false;
    if (s->touchkey_scan_timer) {
        timer_mod(s->touchkey_scan_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  BRAIN_KBD_TOUCHKEY_SCAN_NS);
    }
}

static const VMStateDescription vmstate_brain_kbd = {
    .name = "brain-keyboard",
    .version_id = 5,
    .minimum_version_id = 4,
    .post_load = brain_kbd_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(want, BrainKbdState, BRAIN_KBD_COLS),
        VMSTATE_UINT8_ARRAY(state, BrainKbdState, BRAIN_KBD_COLS),
        VMSTATE_UINT32(mrs_low, BrainKbdState),
        VMSTATE_UINT32_V(touchkey_want, BrainKbdState, 5),
        VMSTATE_UINT32_V(touchkey_pub, BrainKbdState, 5),
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
