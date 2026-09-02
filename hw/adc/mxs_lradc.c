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
#include <stdlib.h>
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "trace.h"
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

/* the 12-bit converter's full scale */
#define LRADC_MAX_VALUE     4095

/*
 * A pen release is reported as "still down at the release position" for
 * this many conversion bursts before the plate goes open.  The touchraw
 * worker (0xc06c1ef4) takes three DELAY-triggered phases per sample, so
 * three bursts cover one full read cycle; four leave headroom for a
 * release that lands in the middle of the burst the driver armed.
 */
#define LRADC_LIFT_BURSTS   4

/*
 * Plate counts the PW-SH6 resistive digitiser presents to the LRADC.
 *
 * These are hardware constants of the panel, measured off the unit's own
 * factory calibration record, which the boot registry carries as
 * HARDWARE\DEVICEMAP\TOUCH "CalibrationData":
 *
 *   "1931,1961  889,2993  888,906  2961,920  2958,3039"
 *    centre      top-left    bottom-left  bottom-right  top-right
 *
 * (order established below, not assumed: it is the only reading under which
 * the four corner records are mutually consistent, see the note there).  A
 * 5-point calibration taps inset targets, so these raws are *not* the counts
 * at the glass edges, they are the counts at the targets, and the panel's law
 * is the straight line through them:
 *
 *   X: raw 888.5 (left pair) and 2959.5 (right pair); the driver's transform
 *      x' = (raw*633 - 36*2811... /2811 puts those at logical x = 164 and
 *      631, i.e. the classic ~20% inset, so raw = 160 + 4.4347 * x'.
 *   Y: raw 3016 (upper pair, y' = 119..124) and 913 (lower pair, y' =
 *      389..391) -> raw = 3968 - 7.83 * y'.  The Y plate count therefore
 *      *decreases* as the finger moves down: the axis is inverted at the
 *      panel level, which is what the driver's negative Y coefficient
 *      compensates for.
 *
 * Two independent checks agree with that line, which is why it is trusted:
 * the driver's own acceptance rectangle is  x' - 0x50 > 0x2cf  (rejects
 * x' < 80 and x' > 799, i.e. it expects a calibrated 0..799 logical range),
 * and its live BootArgs coefficient block at PA 0x47f9007c is
 * {633, 2811, -36, 320, -2502, 507} -- the exact inverse of the fit above,
 * mapping raw 160..3704 onto 0..799 and raw 3960..219 onto 0..479.
 *
 * so the counts at the *edges of the glass* -- what the plates show when a
 * finger is on the outermost row or column of the panel -- are the four
 * constants below.  They are stated in the frame the finger arrives in, which
 * is the guest's picture: fx is picture pixels from the left edge of the driven
 * picture, fy from its top row.  The span is the calibrated area, 800 pixels
 * across and 480 down, i.e. 3552 counts over 800 (4.44/pixel) and 3753 counts
 * over 480 (7.82/pixel).
 *
 * The panel's own array runs transversally to that frame: a picture column is
 * an array row, and a picture *row* is an array *column counted from the far
 * end* (array column 479 is the top picture row, because the LCDIF scan order
 * and the mounting compose to one transpose with one mirror).  These constants
 * used to be named AT_ROW0 / AT_COL479 after the array; that invited exactly
 * one of the two laws to be fed its endpoints in array order, and a Y law
 * handed its endpoints the wrong way round is a vertical mirror -- the driver
 * then reports y = 480 - clicked, which the UI reads as "taps do nothing"
 * because every finger lands on the mirrored row.  Naming them after the frame
 * they are evaluated in makes the two axes read alike.
 *
 * Both stay inside the converter's 0..4095 rails, as they must on real
 * silicon.  Nothing here clamps to a "usable window": clamping the plates to
 * 888..2961 is the bug that hid 42 % of the glass and ran the Y axis the
 * wrong way.  A finger outside the calibrated span does drive the plate past
 * it, and it is the *driver* that discards such a sample (its rectangle test
 * is x' - 0x50 > 0x2cf); the model reproduces the panel, never the driver's
 * rejection.
 */
#define BRAIN_PLATE_X_AT_PIC0   160     /* X wiper at the left edge */
#define BRAIN_PLATE_X_AT_PIC800 3712    /* and one pixel past the right edge */
#define BRAIN_PLATE_Y_AT_PIC0   3964    /* Y wiper at the top row */
#define BRAIN_PLATE_Y_AT_PIC480 211     /* and one pixel past the bottom edge */
#define BRAIN_PLATE_X_SPAN      800      /* picture columns the X law spans */
#define BRAIN_PLATE_Y_SPAN      480      /* picture rows the Y law spans */

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

    /*
     * Pen state as the converter sees it.  On silicon a conversion that nobody
     * reads leaves its value standing in the channel data register, so a press
     * that arrives while the driver is in its OFF power state is not lost: the
     * next burst -- the one the wake causes -- samples the plate while the
     * finger is still on it.  Reporting pen-up to a burst armed after the
     * release is what made a suspended guest swallow taps: measured in
     * runs/acc10, a 12 s hold produced 5822 conversions and every single one
     * reported touch_down=0, i.e. the panel looked open for the whole time the
     * finger was on it, so the driver had nothing to deliver.
     */
    enum {
        MXS_LRADC_PEN_UP = 0,       /* plate open */
        MXS_LRADC_PEN_DOWN,         /* finger on the glass, re-latch position */
        MXS_LRADC_PEN_LIFTING       /* report the release position, then open */
    } pen_state;

    /*
     * Conversion bursts still to report the release position as "down"
     * while in PEN_LIFTING.  See mxs_lradc_run_start(): a press whose
     * down edge lands between two of the driver's sample bursts must
     * still be observable -- real silicon latches the press in the
     * channel data registers -- or a quick tap is swallowed.
     */
    int lift_bursts;

    uint32_t loop_count[4];         /* conversions left per DELAY channel */
    int64_t due_ns[4];              /* conversion due time, 0 = none */

    /*
     * The display module this digitizer is bonded to.  Only used to turn the
     * front end's normalised axis into a position on the panel array; with no
     * panel attached the axis range is taken as the calibrated span.
     */
    DeviceState *panel;

    uint32_t batt_value;
    uint32_t vddio_value;
} MXSLradcState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSLradcState, MXS_LRADC)

static int clamp32(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/*
 * Round to nearest: truncating here costs a systematic half pixel on every
 * sample.  Clamp only to the converter's rails, because a finger outside the
 * calibrated span really does drive the plate past it and the *driver* is the
 * thing that rejects such a sample (its rectangle test is x' - 0x50 > 0x2cf).
 */
static int mxs_lradc_plate_count(int pos, int span, int at_first, int at_last)
{
    int64_t sgn = (int64_t)at_last - at_first;
    int64_t v = at_first + (sgn * pos + span / 2) / span;

    return clamp32((int)v, 0, LRADC_MAX_VALUE);
}

/*
 * Where is the finger?  The digitizer is bonded to the panel, so its
 * coordinate frame is the panel's array: ask the LCDIF which array pixel the
 * front end's normalised axis points at, relative to the corner where the
 * guest's own picture starts.  With no panel modelled (or before the console
 * exists) the whole axis range is the calibrated span, which is the same
 * mapping in the degenerate case.
 */
static void mxs_lradc_finger(MXSLradcState *s, int ax, int ay, int *px, int *py)
{
    if (mxs_lcdif_touch_position(s->panel, ax, ay, px, py)) {
        return;
    }
    *px = (int)((int64_t)ax * BRAIN_PLATE_X_SPAN / INPUT_EVENT_ABS_MAX);
    *py = (int)((int64_t)ay * BRAIN_PLATE_Y_SPAN / INPUT_EVENT_ABS_MAX);
}

static void mxs_lradc_update_irq(MXSLradcState *s)
{
    uint32_t c1 = s->regs[LRADC_CTRL1];
    bool touch;
    int i;

    /*
     * TOUCH_DETECT is a *level* on silicon: the comparator keeps the line
     * asserted for as long as the plate is pressed, and it stays high through
     * the release until the last conversion has been read.  Only latching the
     * status bit on the press edge made a held finger look like a single
     * event: a slide then produced exactly one delivered point (runs/acc11
     * drag: 9 motion positions -> one pen-down sample), because the driver only
     * re-enters its sampling path while the line is high.
     */
    touch = ((c1 & CTRL1_TOUCH_DETECT_IRQ) ||
             s->pen_state != MXS_LRADC_PEN_UP) &&
            (c1 & CTRL1_TOUCH_DETECT_IRQ_EN);
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
    int sx, sy, fx, fy;
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
     * The latch is per *burst*: every channel converted in one burst sees the
     * same finger position, so the three samples one read takes are consistent
     * (the worker's jitter filter rejects a read whose samples disagree).  It
     * is re-armed at the next burst, so a held finger is re-sampled where it is
     * now rather than frozen at its press position.  The latch is released by
     * mxs_lradc_convert() once the burst has converted all its channels --
     * clearing it here, before the remaining channels were read, made the
     * first channel of a burst report the press while the rest reported the
     * open plate, which the jitter filter then rejected outright.
     */

    /*
     * The UI delivers the finger as a normalised position over the whole
     * surface; the panel law turns that into the counts the plates produce.
     * Nothing here knows about the driver's calibration: the round trip to a
     * pixel is a property of a calibrated panel, not something to force.
     */
    mxs_lradc_finger(s, sx, sy, &fx, &fy);
    x = mxs_lradc_plate_count(fx, BRAIN_PLATE_X_SPAN,
                              BRAIN_PLATE_X_AT_PIC0,
                              BRAIN_PLATE_X_AT_PIC800);
    y = mxs_lradc_plate_count(fy, BRAIN_PLATE_Y_SPAN,
                              BRAIN_PLATE_Y_AT_PIC0,
                              BRAIN_PLATE_Y_AT_PIC480);

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
    switch (s->pen_state) {
    case MXS_LRADC_PEN_DOWN:
        /* finger still on the glass: report where it is pressing now */
        s->run_x = s->touch_x;
        s->run_y = s->touch_y;
        s->run_down = true;
        break;
    case MXS_LRADC_PEN_LIFTING:
        /*
         * The pen lifted before (or while) the driver was sampling.  Hand out
         * the release position as the transitional sample the worker's jitter
         * filter expects, and keep handing it out -- still flagged down -- for
         * LRADC_LIFT_BURSTS bursts, then return to the open-plate state.  A
         * quick tap's down edge otherwise falls between two of the driver's
         * bursts and the read that runs after the release sees an open plate,
         * i.e. the press is swallowed entirely (measured: instant tap -> 0
         * conversions, held tap -> hundreds).
         */
        s->run_x = s->last_x;
        s->run_y = s->last_y;
        s->run_down = true;
        if (s->lift_bursts > 0 && --s->lift_bursts == 0) {
            s->pen_state = MXS_LRADC_PEN_UP;
        }
        break;
    default:
        s->run_x = s->touch_x;
        s->run_y = s->touch_y;
        s->run_down = false;
        break;
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
        trace_mxs_lradc_convert(i, phys, sample, s->touch_down,
                                s->touch_x, s->touch_y,
                                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    /* the burst has converted every channel it was going to: release the
     * latch so the next burst samples the finger where it is now */
    s->run_valid = false;
    mxs_lradc_update_irq(s);
}

/*
 * HW_LRADC_DELAYn counts LOOP_COUNT conversions and then stops; the KICK bit is
 * self-clearing on silicon and the driver's write-1-to-clear aborts a
 * burst that has not run yet.  Both matter far beyond fidelity:
 *
 * The WinCE touch PDD ends a sampling sequence by clearing KICK (0xc06731e8).
 * A loop that ignored KICK kept re-arming a 1 ms conversion for the rest of the
 * session: every round re-latched the channel IRQ status, which the guest's
 * write-1-to-clear can never win against, so the collector spun in
 * "InterruptHandle() already gated" (~20k ICOLL transactions per tap measured
 * in runs/fix02), the touch IST thread starved, and the result registers were
 * overwritten with the release value before the driver read them.  That is the
 * "tap does nothing", "slide stops halfway" and boot-time input-lag symptom in
 * one bug, and it is why mxs_lradc_delay_stop() exists.
 */
/*
 * One conversion burst: latch the plate state, convert the channels the delay
 * loop triggers, and leave the loop finished.  Both the timer and a reader that
 * arrived late (see mxs_lradc_poll_due()) go through here, so the value the
 * guest sees never depends on which of them got there first.
 */
static void mxs_lradc_burst(MXSLradcState *s, int n)
{
    uint32_t d = s->regs[LRADC_DELAY0 + n];

    s->due_ns[n] = 0;
    s->loop_count[n] = 0;
    mxs_lradc_run_start(s, true);
    mxs_lradc_convert(s, d >> DELAY_TRIGGER_LRADCS_SHIFT);
}

static void mxs_lradc_delay_stop(MXSLradcState *s, int n)
{
    s->loop_count[n] = 0;
    s->due_ns[n] = 0;
    timer_del(s->delay_timer[n]);
    mxs_lradc_run_end(s);
}

static void mxs_lradc_delay_tick(void *opaque, int n)
{
    MXSLradcState *s = opaque;

    if (!s->due_ns[n]) {
        return;
    }
    if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->due_ns[n]) {
        timer_mod(s->delay_timer[n], s->due_ns[n]);
        return;
    }
    mxs_lradc_burst(s, n);
}

/*
 * A reader may arrive after a conversion became due while no timer callback
 * could run -- the WinCE BSP waits with StallExecution, and the main loop
 * cannot be entered from inside a running vCPU.  Complete anything whose
 * deadline has passed at the moment the guest looks, which is exactly when
 * hardware would have presented it.
 */
static void mxs_lradc_poll_due(MXSLradcState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int n;

    for (n = 0; n < 4; n++) {
        if (s->due_ns[n] && now >= s->due_ns[n]) {
            mxs_lradc_burst(s, n);
        }
    }
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

    mxs_lradc_poll_due(s);
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
            int64_t delay = (int64_t)(val & DELAY_DELAY_MASK) * 500000;

            /*
             * KICK starts LOOP_COUNT + 1 conversions and the bit is
             * self-clearing on silicon; the driver's own write-1-to-clear
             * aborts a burst it no longer wants (measured at 0xc06731e8: it
             * writes HW_LRADC_DELAYn_CLR = 0xffffffff straight after reading
             * the samples, which is what mxs_lradc_delay_stop() honours).
             *
             * The first conversion is completed here, synchronously.  That is
             * what the reader on real hardware observes, and it is measured,
             * not assumed: the driver programs DELAY1 with trigger CH2|CH5,
             * DELAY = 2 and KICK, then takes its samples within about thirty
             * MMIO accesses (W +0xe4 set 0x0c000000 / set 0x2 / set 0x100000,
             * then R +0x70, R +0x80, R +0xa0 -- runs/s89/probe_kick.py).
             * Deferring the conversion to a timer makes it unobservable,
             * because no timer callback runs while the vCPU holds the BQL: the
             * driver reads a channel that was never written, its pressure test
             * (Z = CH3 * R / 4096, zero meaning "no pen") fails, and every tap
             * is dropped while the input events still reach the device (122 of
             * them against 0 conversions, measured the same way).
             *
             * The remaining LOOP_COUNT conversions are armed for the timer,
             * which is what keeps a *held* finger re-sampling for as long as
             * the driver lets the loop run.  The plate cannot move within one
             * burst, so a burst converts the state it was started with.
             *
             * The programmed DELAY field is scaled by the ~2 kHz figure the
             * BSP's own re-arm spacing implies -- an inherited estimate, not a
             * datasheet value, and nothing in the touch read path depends on
             * it: the sample the driver takes comes from the synchronous
             * conversion above.
             */
            s->regs[idx] = val & ~DELAY_KICK;
            mxs_lradc_burst(s, n);
            if (loops) {
                s->loop_count[n] = loops;
                s->due_ns[n] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay;
                timer_mod(s->delay_timer[n], s->due_ns[n]);
            }
            trace_mxs_lradc_kick(n, loops, (int)delay / 1000,
                                 val >> DELAY_TRIGGER_LRADCS_SHIFT);
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

static const MemoryRegionOps mxs_lradc_ops = {
    .read = mxs_lradc_read,
    .write = mxs_lradc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* ------------------------------------------------------------------ */
/* touch screen                                                        */
/* ------------------------------------------------------------------ */

/*
 * Shared entry point for touch state changes: everything that can move a
 * finger on this panel -- the QEMU input layer for the SDL/VNC window, and
 * QMP input-send-event -- arrives through the device's input handler, so the
 * model state is the same whichever front end was used.
 */
static void mxs_lradc_set_touch(DeviceState *dev, int x, int y, bool down)
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
        /* deliver the transitional sample from the *next* burst, whenever
         * that is -- including one armed long after the release -- and keep
         * it flagged down for LRADC_LIFT_BURSTS bursts so a quick tap is
         * not lost between the driver's sampling windows */
        s->pen_state = MXS_LRADC_PEN_LIFTING;
        s->lift_bursts = LRADC_LIFT_BURSTS;
    } else if (down) {
        s->pen_state = MXS_LRADC_PEN_DOWN;
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

    trace_mxs_lradc_set_touch(s->touch_x, s->touch_y, down,
                              s->regs[LRADC_CTRL0], s->regs[LRADC_CTRL1],
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));

    if (!down) {
        /*
         * The pen-up edge drops the latched detect status so the level-
         * triggered ICOLL input falls again.  (A status bit the driver clears
         * itself would be strictly more like silicon, but with that behaviour
         * the guest leaves the line asserted between taps and this model's
         * idle path changes shape; kept as the committed, measured-working
         * edge.  Verified both ways: see runs/s89/probe notes.)
         */
        s->regs[LRADC_CTRL1] &= ~CTRL1_TOUCH_DETECT_IRQ;
        mxs_lradc_update_irq(s);
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
     * whatever the entry point was -- that is what wakes a PDD that had
     * powered itself down.
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
    s->pen_state = MXS_LRADC_PEN_UP;
    s->lift_bursts = 0;
    s->run_valid = false;
}


static void mxs_lradc_realize(DeviceState *dev, Error **errp)
{
    MXSLradcState *s = MXS_LRADC(dev);

    s->input = qemu_input_handler_register(dev, &mxs_lradc_input_handler);
}


static void mxs_lradc_init(Object *obj)
{

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
    DEFINE_PROP_LINK("panel", MXSLradcState, panel, TYPE_MXS_LCDIF,
                     DeviceState *),
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
