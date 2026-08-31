/*
 * Freescale i.MX28 (MXS) Low Resolution ADC
 *
 * Conversions complete instantly.  The block also carries the resistive
 * touch screen controller of the SHARP Brain, which is wired up to the
 * QEMU absolute pointer input.  Per the i.MX28 reference manual the
 * touch plates are physical channels 2 (X+/XPUL), 3 (Y+/YPLL),
 * 4 (X-/XNUL) and 5 (Y-/YNLR); the WinCE TPDriver reads 2/3/5 through
 * the LDC1: driver with the identity CTRL4 channel map.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "ui/console.h"
#include "ui/input.h"

/* register indices (offset >> 4) */
#define LRADC_CTRL0         0x00
#define LRADC_CTRL1         0x01
#define LRADC_CTRL2         0x02
#define LRADC_CTRL3         0x03
#define LRADC_STATUS        0x04
#define LRADC_CH0           0x05    /* .. 0x0c */
#define LRADC_DELAY0        0x0d    /* .. 0x10 */
#define LRADC_DEBUG0        0x11
#define LRADC_DEBUG1        0x12
#define LRADC_CONVERSION    0x13
#define LRADC_CTRL4         0x14
#define LRADC_THRESHOLD0    0x15
#define LRADC_THRESHOLD1    0x16
#define LRADC_VERSION       0x17
#define LRADC_NREGS         0x20

#define CTRL0_SFTRST                (1u << 31)
#define CTRL0_CLKGATE               (1u << 30)
#define CTRL0_TOUCH_DETECT_ENABLE   (1u << 23)
#define CTRL0_SCHEDULE_MASK         0xff

#define CTRL1_TOUCH_DETECT_IRQ_EN   (1u << 24)
#define CTRL1_TOUCH_DETECT_IRQ      (1u << 8)
#define CTRL1_IRQ_EN_SHIFT          16
#define CTRL1_IRQ_MASK              0xff

#define STATUS_TOUCH_DETECT_RAW     (1u << 0)

#define DELAY_TRIGGER_LRADCS_SHIFT  24
#define DELAY_KICK                  (1u << 20)
#define DELAY_LOOP_COUNT_SHIFT      11
#define DELAY_LOOP_COUNT_MASK       0x1f
#define DELAY_DELAY_MASK            0x7ff

#define LRADC_NCHANNELS     8

/*
 * Calibrated plate span, from the boot registry CalibrationData
 * (HARDWARE\DEVICEMAP\TOUCH, "1931,1961 889,2993 888,906 2961,920
 * 2958,3039"): raw X runs 888..2961 over the 800 px display width and raw Y
 * 906..3039 over the 480 px height.  Centre raw (1931,1961) therefore lands
 * on display (400,240), exactly as measured on hardware.
 */
#define BRAIN_RAW_X_LO      888
#define BRAIN_RAW_X_HI      2961
#define BRAIN_RAW_Y_LO      906
#define BRAIN_RAW_Y_HI      3039

typedef struct MXSLradcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq_touch;             /* ICOLL "touch screen" line */
    qemu_irq irq_ch[LRADC_NCHANNELS];
    qemu_irq panel_wake;            /* EDNA2 attention, like a key press */

    uint32_t regs[LRADC_NREGS];
    QEMUTimer *delay_timer[4];

    /* touch screen state */
    QemuInputHandlerState *input;
    int touch_x;                    /* 0 .. 0x7fff from the UI */
    int touch_y;
    bool touch_down;
    int last_x, last_y;             /* transitional pen-up sample */

    /*
     * A DELAY kick arms LOOP_COUNT further conversions; on silicon the
     * sequence is free-running once started.  Latch the plate state for the
     * whole run so a UI motion landing in the middle of a burst can not make
     * the driver's three-sample jitter filter reject the read.
     */
    int run_x, run_y;
    bool run_down;
    bool run_valid;

    uint32_t loop_count[4];         /* conversions left per DELAY channel */
    bool touch_up_pending;          /* deliver one transitional sample */

    uint32_t batt_value;
    uint32_t vddio_value;
} MXSLradcState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSLradcState, MXS_LRADC)

static int clamp32(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static bool brain_touch_debug(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("BRAIN_TOUCH_DEBUG");

        on = e && *e && *e != '0';
    }
    return on;
}

static void mxs_lradc_update_irq(MXSLradcState *s)
{
    uint32_t c1 = s->regs[LRADC_CTRL1];
    bool touch;
    int i;

    touch = (c1 & CTRL1_TOUCH_DETECT_IRQ) && (c1 & CTRL1_TOUCH_DETECT_IRQ_EN);
    qemu_set_irq(s->irq_touch, touch);

    for (i = 0; i < LRADC_NCHANNELS; i++) {
        bool level = (c1 & (1u << i)) &&
                     (c1 & (1u << (CTRL1_IRQ_EN_SHIFT + i)));

        qemu_set_irq(s->irq_ch[i], level);
    }
}

/*
 * Factory raw ranges from the boot registry CalibrationData
 * (nk_main.bin file 0x1be9f8d, HARDWARE\DEVICEMAP\TOUCH):
 *
 *   "1931,1961 889,2993 888,906 2961,920 2958,3039"
 *
 * centre = raw (1931,1961), UL = (888,906), LR = (2958,3039):
 * raw X spans 888..2961 across 800 px, raw Y 906..3039 across 480 px
 * (both non-inverted).
 *
 * The touchraw.dll sampler (ioctl 0x8000202f worker 0xc06c1ef4) runs
 * three DELAY-triggered phases per sample and averages:
 *
 *   ReadXY  : trigger CH2|CH5 (0x24) -> CH2 = X-coord, CH5 = Y-coord
 *   phase A : trigger CH2|CH3 (0x0c) -> CH3 read  -> averaged into x
 *   phase B : trigger CH2     (0x04) -> CH2 read  -> averaged into y
 *
 * (verified with brain_wwatch on the fetch's x_global 0xc07c71c0 /
 * y_global 0xc07c71c4 in runs/tch12: the guest received exactly the
 * model's CH3 value as x and the CH2 value as y).  The plate value
 * therefore depends on which trigger group the conversion belongs
 * to: CH3 under trigger CH2|CH3 must report the X coordinate and
 * CH2 under trigger CH2 alone must report the Y coordinate.
 */
static uint32_t mxs_lradc_sample(MXSLradcState *s, int ch, uint32_t trigger)
{
    uint32_t x, y;
    int ax, ay;
    int sx, sy;
    bool down;

    /*
     * Report the state latched for the current conversion burst (see
     * mxs_lradc_run_start()): the driver's three-sample jitter filter must
     * not be tripped by a UI motion that lands inside a burst, which is what
     * makes a slide "stop halfway" while a static tap works.
     *
     * A pen-up transition is sampled while the panel is already released: on
     * silicon the last conversion still carries the release position, so the
     * driver can tell "the pen lifted" from "no pen was ever down".  Feed the
     * transitional coordinates through the burst exactly once.
     */
    sx = s->run_valid ? s->run_x : s->touch_x;
    sy = s->run_valid ? s->run_y : s->touch_y;
    down = s->run_valid ? s->run_down : s->touch_down;

    /*
     * The UI delivers normalised absolute coordinates over the whole panel;
     * the calibrated plate span is what a finger on the glass produces, so
     * map [0, 0x7fff] -> [LEFT, RIGHT] with round-to-nearest.  Truncating
     * instead costs a systematic half-to-one pixel and pushes the extreme
     * column outside the driver's rectangle test.
     */
    ax = DIV_ROUND_UP(sx * (BRAIN_RAW_X_HI - BRAIN_RAW_X_LO), 0x7fff);
    ay = DIV_ROUND_UP(sy * (BRAIN_RAW_Y_HI - BRAIN_RAW_Y_LO), 0x7fff);
    /* 12-bit converter: a plate reading can never exceed 0xfff */
    x = MIN(BRAIN_RAW_X_LO + ax, 0xfff);
    y = MIN(BRAIN_RAW_Y_LO + ay, 0xfff);

    if (!down) {
        x = 0;
        y = 0;
    }

    /*
     * Physical plate semantics (i.MX28 4-wire, verified against
     * touchraw.dll's ReadXY 0xc06c1948 / GetX 0xc06c18e0):
     *
     *   GetX: plate C (XNNSW|YNNSW) drives X-/Y-; CH2 (XPUL) reads
     *         the X position.
     *   ReadXY: plate A (XPPSW|XNNSW) drives the X axis; CH3 (YPLL)
     *           is the wiper and reads the X position (potentiometer).
     *           The driver computes Z = CH3*R/4096 as a pressure
     *           estimate: Z == 0 (i.e. CH3 == 0) makes ReadXY fail,
     *           which is how the release is detected.
     *           Plate B then drives Y; CH2 reads the Y position.
     *
     * The worker averages the CH3-under-A samples into its X slot
     * and the CH2-under-B samples into its Y slot (tch12 wwatch on
     * x_global 0xc07c71c0 / y_global 0xc07c71c4).  CH3 under the
     * plate-A trigger must return the X coordinate; the S70 constant-
     * pressure experiment broke the X slot and produced px=0
     * (runs/tch29/tch30), while tch20 with this model delivered
     * px=1604/py=968 = screen (401,242) for the centre tap.
     */
    switch (ch) {
    case 2:     /* XPUL */
        if ((trigger & ~(1u << 2)) == 0) {
            /* plate B (CH2 alone): Y position */
            return y;
        }
        /* GetX / ReadXY: X position */
        return x;
    case 3:     /* YPLL: X coordinate under plate A, pressure else */
        if (trigger & (1u << 3)) {
            return x;
        }
        return down ? 0x200 : 0;
    case 4:     /* XNUL (unread by the touch stack) */
        return down ? 0x400 : 0;
    case 5:     /* YNLR: Y probe (read by GetX as X5) */
        return y;
    case 6:     /* VDDIO */
        return s->vddio_value;
    case 7:     /* battery */
        return s->batt_value;
    default:
        return 0x400;
    }
}

/*
 * Latch the touch state for the duration of one conversion burst.  Called
 * with @force on a fresh DELAY kick (the sequence starts there), and without
 * it from the auto-rearm path, which must keep reporting the same sample the
 * driver armed the loop for.
 */
static void mxs_lradc_run_start(MXSLradcState *s, bool force)
{
    if (s->run_valid && !force) {
        return;
    }
    s->run_x = s->touch_x;
    s->run_y = s->touch_y;
    s->run_down = s->touch_down;
    if (!s->touch_down && s->touch_up_pending) {
        /* the pen lifted since the last sample: report the release position */
        s->run_x = s->last_x;
        s->run_y = s->last_y;
        s->run_down = true;
        s->touch_up_pending = false;
    }
    s->run_valid = true;
}

static void mxs_lradc_run_end(MXSLradcState *s)
{
    s->run_valid = false;
}

static void mxs_lradc_convert(MXSLradcState *s, uint32_t channels)
{
    int i;

    for (i = 0; i < LRADC_NCHANNELS; i++) {
        int phys;
        uint32_t sample;

        if (!(channels & (1u << i))) {
            continue;
        }
        /* HW_LRADC_CTRL4 maps the virtual channel onto a physical one */
        phys = (s->regs[LRADC_CTRL4] >> (i * 4)) & 0xf;
        sample = mxs_lradc_sample(s, phys, channels);
        s->regs[LRADC_CH0 + i] =
            (s->regs[LRADC_CH0 + i] & 0xfffc0000) | sample;
        s->regs[LRADC_CTRL1] |= 1u << i;
        if (brain_touch_debug()) {
            fprintf(stderr, "[brain] lradc convert: vch%d phys%d -> 0x%x "
                    "(touch_down=%d x=%d y=%d ctrl4=0x%08x)\n",
                    i, phys, sample, s->touch_down, s->touch_x, s->touch_y,
                    s->regs[LRADC_CTRL4]);
        }
    }
    mxs_lradc_update_irq(s);
}

/*
 * HW_LRADC_DELAYn counts LOOP_COUNT conversions and then stops; DELAY.KICK is
 * self-clearing on silicon.  Both matter far beyond fidelity.
 *
 * The WinCE touch PDD ends a sampling sequence by clearing KICK
 * (0xc06731e8).  A loop that ignored KICK kept re-arming a 1 ms conversion
 * for the rest of the session: every round re-latched the channel IRQ status,
 * which the guest's write-1-to-clear can never win against, so the collector
 * spun in "InterruptHandle() already gated" (~20k ICOLL transactions per tap
 * measured in runs/fix02), the touch IST thread starved, and the result
 * registers were overwritten with the release value before the driver read
 * them.  That is the "tap does nothing", "slide stops halfway" and
 * boot-time input-lag symptom in one bug.
 */
static void mxs_lradc_delay_stop(MXSLradcState *s, int n)
{
    s->loop_count[n] = 0;
    timer_del(s->delay_timer[n]);
    mxs_lradc_run_end(s);
}

static void mxs_lradc_delay_schedule(MXSLradcState *s, int n)
{
    uint32_t d = s->regs[LRADC_DELAY0 + n];
    uint32_t delay = d & DELAY_DELAY_MASK;
    int64_t period = (int64_t)(delay ? delay : 1) * 500000; /* ~2kHz */

    if ((d & DELAY_KICK) && s->loop_count[n] > 1) {
        s->loop_count[n]--;
        timer_mod(s->delay_timer[n],
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period);
    } else {
        mxs_lradc_delay_stop(s, n);
    }
}

static void mxs_lradc_delay_tick(void *opaque, int n)
{
    MXSLradcState *s = opaque;
    uint32_t d = s->regs[LRADC_DELAY0 + n];

    if (!(d & DELAY_KICK)) {
        mxs_lradc_delay_stop(s, n);
        return;
    }
    mxs_lradc_run_start(s, false);
    mxs_lradc_convert(s, d >> DELAY_TRIGGER_LRADCS_SHIFT);
    mxs_lradc_delay_schedule(s, n);
}

static void mxs_lradc_delay0(void *o) { mxs_lradc_delay_tick(o, 0); }
static void mxs_lradc_delay1(void *o) { mxs_lradc_delay_tick(o, 1); }
static void mxs_lradc_delay2(void *o) { mxs_lradc_delay_tick(o, 2); }
static void mxs_lradc_delay3(void *o) { mxs_lradc_delay_tick(o, 3); }

static uint64_t mxs_lradc_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSLradcState *s = MXS_LRADC(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= LRADC_NREGS) {
        return 0;
    }

    switch (idx) {
    case LRADC_VERSION:
        val = 0x02000000;
        break;
    case LRADC_STATUS:
        val = s->regs[idx] & ~STATUS_TOUCH_DETECT_RAW;
        if (s->touch_down) {
            val |= STATUS_TOUCH_DETECT_RAW;
        }
        break;
    default:
        val = s->regs[idx];
        break;
    }

    return mxs_bank_extract(offset, size, val);
}

static void mxs_lradc_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    MXSLradcState *s = MXS_LRADC(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= LRADC_NREGS || idx == LRADC_VERSION) {
        return;
    }

    val = mxs_bank_apply(s->regs[idx], offset, value, size);

    switch (idx) {
    case LRADC_CTRL0: {
        uint32_t sched = val & CTRL0_SCHEDULE_MASK;

        val = mxs_bank_sftrst(s->regs[idx], val);
        s->regs[idx] = val & ~CTRL0_SCHEDULE_MASK;
        if (sched && !(val & (CTRL0_SFTRST | CTRL0_CLKGATE))) {
            mxs_lradc_run_start(s, true);
            mxs_lradc_convert(s, sched);
            mxs_lradc_run_end(s);
        }
        break;
    }

    case LRADC_CTRL1:
        /*
         * The IRQ status bits [15:8] are write-1-to-clear on real
         * silicon: a BSP that does the usual
         *   HW_LRADC_CTRL1_CLR = TOUCH_DETECT_IRQ;
         * sequence expects the bit to actually go back to zero.
         * Without this the kernel sees the touch IRQ staying
         * asserted across re-arming of the IST and complains
         * with "InterruptHandle() already gated".
         */
        s->regs[idx] = (s->regs[idx] & ~(val & 0x0000ff00u)) |
                       (val & ~0x0000ff00u);
        mxs_lradc_update_irq(s);
        break;

    case LRADC_DELAY0:
    case LRADC_DELAY0 + 1:
    case LRADC_DELAY0 + 2:
    case LRADC_DELAY0 + 3: {
        int n = idx - LRADC_DELAY0;

        s->regs[idx] = val;
        if (val & DELAY_KICK) {
            uint32_t loops = (val >> DELAY_LOOP_COUNT_SHIFT) &
                             DELAY_LOOP_COUNT_MASK;

            /*
             * The WinCE BSP waits out the conversion with a CPU
             * spin (StallExecution in touchraw.dll / lradc.dll),
             * during which the virtual clock does not advance.  A
             * timer-based conversion therefore completes only after
             * the reader has already moved on and the touch plate
             * values are read back as zero (verified: 14 s touch
             * hold in runs/tch3/tch6, R +0x070 -> 0).  Convert
             * synchronously on the kick -- the guest spin provides
             * the settling delay -- and keep the timer only for the
             * loop re-arms.
             */
            /* consume KICK: the burst is armed for 1 + LOOP_COUNT reads */
            s->regs[idx] = val & ~DELAY_KICK;
            s->loop_count[n] = 1 + loops;
            mxs_lradc_run_start(s, true);
            mxs_lradc_convert(s, val >> DELAY_TRIGGER_LRADCS_SHIFT);
            mxs_lradc_delay_schedule(s, n);
        } else {
            /* KICK cleared by the driver: the burst is over */
            mxs_lradc_delay_stop(s, n);
        }
        break;
    }

    default:
        s->regs[idx] = val;
        break;
    }
}

MXS_TRACE_WRAP(mxs_lradc, "lradc")

static const MemoryRegionOps mxs_lradc_ops = {
    .read = mxs_lradc_read_tr,
    .write = mxs_lradc_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* ------------------------------------------------------------------ */
/* touch screen                                                        */
/* ------------------------------------------------------------------ */

/*
 * Shared entry point for touch state changes: the QEMU input layer
 * (SDL mouse etc.) and the headless HMP `brain_touch` command both
 * funnel through here so that the model state is identical in both
 * cases.
 */
void mxs_lradc_set_touch(DeviceState *dev, int x, int y, bool down)
{
    MXSLradcState *s = MXS_LRADC(dev);

    /*
     * A release transition must show up as ONE transitional sample.
     * The touchraw worker (0xc06c1ef4) takes three samples per read
     * and fails the whole read when the samples disagree by more
     * than 0x300 or the total distance exceeds 0x9200 (jitter
     * rejection).  On real hardware the pen lift lands in the
     * middle of the three samples, so the worker returns status 0
     * and the fetch reports the pen-up flag.  Emulate the same
     * edge: keep the pre-release coordinates for the first
     * conversion after the release.
     */
    if (s->touch_down && !down) {
        s->last_x = s->touch_x;
        s->last_y = s->touch_y;
        s->touch_up_pending = true;
    }
    bool down_edge = !s->touch_down && down;

    /*
     * Clamp, never mask.  "& 0x7fff" folded an over-range value back onto
     * the opposite side of the panel: the HMP helper's int(px * 0x8000 / 800)
     * returns exactly 0x8000 for the last pixel, so the rightmost column was
     * reported as the leftmost one -- "the right edge cannot be captured".
     */
    s->touch_x = clamp32(x, 0, 0x7fff);
    s->touch_y = clamp32(y, 0, 0x7fff);
    s->touch_down = down;

    if (brain_touch_debug()) {
        fprintf(stderr, "[brain] lradc SETTOUCH x=%d y=%d down=%d "
                "CTRL0=0x%08x CTRL1=0x%08x vnow=%lld\n",
                s->touch_x, s->touch_y, down,
                s->regs[LRADC_CTRL0], s->regs[LRADC_CTRL1],
                (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }

    if (!down) {
        return;
    }

    /*
     * Only the down *edge* latches TOUCH_DETECT_IRQ.  ABS motion events
     * while the pen is already down used to re-set the bit on every
     * mouse-move, which re-entered the IST, kicked LCDIF, and made the
     * screen flash / the guest stall for the duration of the tap.
     */
    if (!down_edge) {
        return;
    }

    /*
     * Latch the touch-detect status whenever the analog detector is
     * enabled (CTRL0.TOUCH_DETECT_ENABLE), exactly like real silicon:
     * the comparator that flags a touch is independent of whether the
     * CPU currently has the interrupt unmasked.
     */
    s->regs[LRADC_CTRL1] |= CTRL1_TOUCH_DETECT_IRQ;
    /* Drop then raise so ICOLL sees a new edge even if the previous
     * tap's raw bit was still set. */
    qemu_set_irq(s->irq_touch, 0);
    mxs_lradc_update_irq(s);
    /*
     * The EDNA2 MCU owns the panel wake line on the real Brain: it watches
     * the touch panel in parallel with the host LRADC, so a press rings it
     * whatever the entry point was.  Pulsing it here (rather than from the
     * board's HMP helper) is what makes the SDL/gtk pointer wake a PDD that
     * powered itself down, exactly like brain_touch does.
     */
    qemu_irq_pulse(s->panel_wake);
}

static void mxs_lradc_touch_event(DeviceState *dev, QemuConsole *src,
                                  InputEvent *evt)
{
    MXSLradcState *s = MXS_LRADC(dev);
    InputMoveEvent *move;
    InputBtnEvent *btn;

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        move = evt->u.abs.data;
        if (move->axis == INPUT_AXIS_X) {
            mxs_lradc_set_touch(dev, move->value, s->touch_y, s->touch_down);
        } else {
            mxs_lradc_set_touch(dev, s->touch_x, move->value, s->touch_down);
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        btn = evt->u.btn.data;
        if (btn->button == INPUT_BUTTON_LEFT) {
            mxs_lradc_set_touch(dev, s->touch_x, s->touch_y, btn->down);
        }
        break;
    default:
        break;
    }
}

static const QemuInputHandler mxs_lradc_input_handler = {
    .name  = "MXS LRADC touchscreen",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = mxs_lradc_touch_event,
};

/* ------------------------------------------------------------------ */

static void mxs_lradc_reset(DeviceState *dev)
{
    MXSLradcState *s = MXS_LRADC(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[LRADC_CTRL0] = CTRL0_SFTRST | CTRL0_CLKGATE;
    /* identity mapping of the virtual channels */
    s->regs[LRADC_CTRL4] = 0x76543210;
    for (i = 0; i < 4; i++) {
        timer_del(s->delay_timer[i]);
        s->loop_count[i] = 0;
    }
    s->touch_down = false;
    s->touch_up_pending = false;
    s->run_valid = false;
}

static void mxs_lradc_realize(DeviceState *dev, Error **errp)
{
    MXSLradcState *s = MXS_LRADC(dev);

    s->input = qemu_input_handler_register(dev, &mxs_lradc_input_handler);
}

static void mxs_lradc_init(Object *obj)
{
    mxs_lradc_trace = mxs_trace_enabled("lradc");

    MXSLradcState *s = MXS_LRADC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &mxs_lradc_ops, s, "mxs-lradc",
                          0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq_touch);
    for (i = 0; i < LRADC_NCHANNELS; i++) {
        sysbus_init_irq(sbd, &s->irq_ch[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), &s->panel_wake, "panel-wake", 1);

    s->delay_timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL, mxs_lradc_delay0, s);
    s->delay_timer[1] = timer_new_ns(QEMU_CLOCK_VIRTUAL, mxs_lradc_delay1, s);
    s->delay_timer[2] = timer_new_ns(QEMU_CLOCK_VIRTUAL, mxs_lradc_delay2, s);
    s->delay_timer[3] = timer_new_ns(QEMU_CLOCK_VIRTUAL, mxs_lradc_delay3, s);
}

static const Property mxs_lradc_properties[] = {
    DEFINE_PROP_UINT32("battery-value", MXSLradcState, batt_value, 0xb00),
    DEFINE_PROP_UINT32("vddio-value", MXSLradcState, vddio_value, 0x900),
};

static const VMStateDescription vmstate_mxs_lradc = {
    .name = "mxs-lradc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSLradcState, LRADC_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_lradc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mxs_lradc_realize;
    device_class_set_legacy_reset(dc, mxs_lradc_reset);
    device_class_set_props(dc, mxs_lradc_properties);
    dc->vmsd = &vmstate_mxs_lradc;
}

static const TypeInfo mxs_lradc_types[] = {
    {
        .name           = TYPE_MXS_LRADC,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSLradcState),
        .instance_init  = mxs_lradc_init,
        .class_init     = mxs_lradc_class_init,
    },
};

DEFINE_TYPES(mxs_lradc_types)
