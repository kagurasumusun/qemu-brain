/*
 * Freescale i.MX28 (MXS) timers and rotary decoder (TIMROT)
 *
 * The i.MX28 flavour of the block ("v2") gives every timer a 32 bit
 * RUNNING_COUNT register which always counts *down* at the selected rate.
 * Two modes exist:
 *
 *   - fixed count mode: the counter is loaded from FIXED_COUNT and an
 *     interrupt is raised when it reaches zero.  With RELOAD set it starts
 *     over, otherwise it stops.
 *
 *   - match mode (TIMCTRLn.MATCH_MODE): the counter free runs and wraps
 *     through the whole 32 bit range; an interrupt is raised whenever it
 *     equals MATCH_COUNT.
 *
 * Windows CE's OAL uses match mode for both the system tick and the
 * performance counter, so the free running counter has to be readable at
 * any time - it is therefore derived from the virtual clock instead of
 * being stepped by a ptimer.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "brain_stats.h"

#define MXS_NUM_TIMERS  4

/* register indices (offset / 0x10) */
#define TIMROT_ROTCTRL      0x0
#define TIMROT_ROTCOUNT     0x1
#define TIMROT_TIMCTRL(n)   (0x2 + (n) * 4)
#define TIMROT_RUNNING(n)   (0x3 + (n) * 4)
#define TIMROT_FIXED(n)     (0x4 + (n) * 4)
#define TIMROT_MATCH(n)     (0x5 + (n) * 4)
#define TIMROT_VERSION      0x12

#define TIMCTRL_IRQ         (1u << 15)
#define TIMCTRL_IRQ_EN      (1u << 14)
#define TIMCTRL_MATCH_MODE  (1u << 11)
#define TIMCTRL_POLARITY    (1u << 8)
#define TIMCTRL_UPDATE      (1u << 7)
#define TIMCTRL_RELOAD      (1u << 6)
#define TIMCTRL_PRESCALE_SHIFT  4
#define TIMCTRL_PRESCALE_MASK   3
#define TIMCTRL_SELECT_MASK     0xf

/*
 * Rate of the "TICK_ALWAYS" source.  The block is fed from the 24 MHz
 * crystal on the i.MX28; the slower selections are the divided down 32 kHz
 * derivatives.
 */
#define MXS_TIMROT_XTAL_HZ  24000000

typedef struct MXSTimrotState MXSTimrotState;

typedef struct MXSTimer {
    MXSTimrotState *parent;
    QEMUTimer *qtimer;
    qemu_irq irq;
    int index;

    uint32_t ctrl;
    uint32_t fixed;
    uint32_t match;

    /*
     * Match value the timer last fired on.  In match mode hardware raises
     * the interrupt when the down counter passes *through* MATCH; once it
     * has fired, the same equality can only recur after a full 32 bit
     * wrap.  Remembering the value lets us model that "fire once per
     * programming" behaviour instead of re-firing immediately.
     */
    uint32_t fired_match;
    bool fired_match_valid;

    /* free running counter: count == base_count at time base_ns */
    int64_t  base_ns;
    uint32_t base_count;
    uint32_t freq;
    bool running;
} MXSTimer;

struct MXSTimrotState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t rotctrl;
    MXSTimer timers[MXS_NUM_TIMERS];
};

OBJECT_DECLARE_SIMPLE_TYPE(MXSTimrotState, MXS_TIMROT)

static unsigned mxs_timer_freq(uint32_t ctrl)
{
    unsigned freq;

    switch (ctrl & TIMCTRL_SELECT_MASK) {
    case 0x0:   /* NEVER_TICK */
        return 0;
    case 0xb:   /* 32 kHz xtal */
        freq = 32768;
        break;
    case 0xc:   /* 8 kHz */
        freq = 8192;
        break;
    case 0xd:   /* 4 kHz */
        freq = 4096;
        break;
    case 0xe:   /* 1 kHz */
        freq = 1024;
        break;
    case 0xf:   /* TICK_ALWAYS */
        freq = MXS_TIMROT_XTAL_HZ;
        break;
    default:    /* PWM / rotary sources: approximate with the 32 kHz xtal */
        freq = 32768;
        break;
    }

    return freq >> ((ctrl >> TIMCTRL_PRESCALE_SHIFT) & TIMCTRL_PRESCALE_MASK);
}

static void mxs_timer_update_irq(MXSTimer *t)
{
    bool level = (t->ctrl & TIMCTRL_IRQ) && (t->ctrl & TIMCTRL_IRQ_EN);
    static int irq_budget = 40;

    if (irq_budget > 0) {
        fprintf(stderr, "timrot-irq: t%d level=%d ctrl=%04x irq_en=%d\n",
                t->index, level, t->ctrl, !!(t->ctrl & TIMCTRL_IRQ_EN));
        irq_budget--;
    }
    qemu_set_irq(t->irq, level);
}

/* number of ticks that elapsed between @from and @now */
static uint64_t mxs_timer_ticks(MXSTimer *t, int64_t now)
{
    if (!t->freq || now <= t->base_ns) {
        return 0;
    }
    return muldiv64(now - t->base_ns, t->freq, NANOSECONDS_PER_SECOND);
}

static int64_t mxs_timer_ns(MXSTimer *t, uint64_t ticks)
{
    return muldiv64(ticks, NANOSECONDS_PER_SECOND, t->freq);
}

/* current value of RUNNING_COUNT */
static uint32_t mxs_timer_count(MXSTimer *t, int64_t now)
{
    uint64_t elapsed;

    if (!t->running || !t->freq) {
        return t->base_count;
    }

    elapsed = mxs_timer_ticks(t, now);

    if (t->ctrl & TIMCTRL_MATCH_MODE) {
        /* free running, wraps through the full 32 bit range */
        return (uint32_t)(t->base_count - elapsed);
    }

    if (elapsed >= t->base_count) {
        if (!(t->ctrl & TIMCTRL_RELOAD) || !t->fixed) {
            return 0;
        }
        /* periodic: fixed .. 0 inclusive is fixed + 1 ticks */
        elapsed = (elapsed - t->base_count) % ((uint64_t)t->fixed + 1);
        return t->fixed - (uint32_t)elapsed;
    }
    return t->base_count - (uint32_t)elapsed;
}

/* fold the elapsed time into base_count/base_ns to keep the maths small */
static void mxs_timer_sync(MXSTimer *t, int64_t now)
{
    uint64_t elapsed;

    if (!t->running || !t->freq) {
        return;
    }
    elapsed = mxs_timer_ticks(t, now);
    t->base_count = mxs_timer_count(t, now);
    t->base_ns += mxs_timer_ns(t, elapsed);
}

static void mxs_timer_resched(MXSTimer *t)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t delta;
    uint32_t cur;

    if (!t->running || !t->freq) {
        timer_del(t->qtimer);
        return;
    }

    mxs_timer_sync(t, now);
    cur = t->base_count;

    if (t->ctrl & TIMCTRL_MATCH_MODE) {
        delta = (uint32_t)(cur - t->match);
        if (delta == 0) {
            delta = 1ull << 32;         /* just fired: next wrap */
        } else if ((int32_t)delta < 0) {
            if (t->fired_match_valid && t->fired_match == t->match) {
                /*
                 * We already fired for exactly this deadline and the
                 * guest has not programmed a new one since.  Hardware
                 * equality can only recur after the counter wraps the
                 * full 32 bits, and @delta already holds precisely that
                 * modular wrap distance, so keep it.  Re-firing at once
                 * instead would spin the virtual timer at full host speed
                 * (~400k expiries/s observed) while the guest believes
                 * no interrupt should be pending at all.
                 */
            } else {
                /*
                 * The match lies "in the past" (mod 2^32): the down
                 * counter has already counted through it.  On real
                 * hardware the guest never runs into this (re-arm
                 * latency is bounded in real time) but under emulation
                 * the ISR can easily be late relative to the virtual
                 * counter, and an equality-only match would then only be
                 * reached after a full ~358 second wrap -- which is
                 * exactly the multi-minute stall the WinCE tick was
                 * exhibiting.  Treat it like hardware that reports
                 * "deadline already crossed": fire as soon as possible.
                 */
                delta = 1;
            }
        }
        {
            static int rs_budget = 400;
            int64_t dns = mxs_timer_ns(t, delta);

            if (rs_budget > 0 && t->index == 0) {
                fprintf(stderr,
                        "timrot-resched: t%d cur=%08x match=%08x "
                        "delta=%u ticks freq=%u dns=%lldms base_ns=%lld\n",
                        t->index, cur, t->match, (unsigned)delta, t->freq,
                        (long long)(dns / 1000000),
                        (long long)t->base_ns);
                rs_budget--;
            }
        }
    } else {
        if (cur) {
            delta = cur;
        } else if ((t->ctrl & TIMCTRL_RELOAD) && t->fixed) {
            delta = (uint64_t)t->fixed + 1;
        } else {
            timer_del(t->qtimer);
            return;
        }
    }

    timer_mod_ns(t->qtimer, t->base_ns + mxs_timer_ns(t, delta));
}

static void mxs_timer_diag_clocks(void)
{
    fprintf(stderr,
            "timrot-clocks: VIRTUAL=%lld REALTIME=%lld HOST=%lld\n",
            (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
            (long long)qemu_clock_get_ns(QEMU_CLOCK_REALTIME),
            (long long)qemu_clock_get_ns(QEMU_CLOCK_HOST));
}

static void mxs_timer_expire(void *opaque)
{
    MXSTimer *t = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t wall = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    static int64_t last_fire[4] = { -1, -1, -1, -1 };
    static uint64_t nfire[4];
    int64_t vdelta = last_fire[t->index] < 0 ? 0 : now - last_fire[t->index];

    nfire[t->index]++;
    if (getenv("BRAIN_TIMTRACE") && nfire[t->index] <= 40) {
        fprintf(stderr,
                "timrot-expire: t%d #%llu vnow=%lld wall=%lld dy=%lldms "
                "ctrl=%04x freq=%u match=%08x\n",
                t->index, (unsigned long long)nfire[t->index],
                (long long)now, (long long)wall,
                (long long)(vdelta / 1000000), t->ctrl, t->freq, t->match);
    }
    last_fire[t->index] = now;

    brain_stat_inc((BrainStat)(BST_TIMROTn_EXPIRE0 + t->index));
    t->ctrl |= TIMCTRL_IRQ;
    t->fired_match = t->match;
    t->fired_match_valid = true;
    mxs_timer_update_irq(t);
    mxs_timer_resched(t);
}

/* (re)start the counter from @count */
static void mxs_timer_restart(MXSTimer *t, uint32_t count)
{
    t->freq = mxs_timer_freq(t->ctrl);
    t->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    t->base_count = count;
    t->running = t->freq != 0;
    t->fired_match_valid = false;
    mxs_timer_resched(t);
}

/* the control register changed: keep the counter, just re-evaluate */
static void mxs_timer_retune(MXSTimer *t)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned newfreq = mxs_timer_freq(t->ctrl);

    mxs_timer_sync(t, now);
    t->freq = newfreq;
    t->base_ns = now;
    t->running = newfreq != 0;
    t->fired_match_valid = false;
    mxs_timer_resched(t);
}

static uint64_t mxs_timrot_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSTimrotState *s = MXS_TIMROT(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t val = 0;
    int i;

    if (idx == TIMROT_ROTCTRL) {
        val = s->rotctrl | (0x1fu << 25);   /* ROTARY / TIMn_PRESENT */
    } else if (idx == TIMROT_VERSION) {
        val = 0x02000000;
    } else {
        for (i = 0; i < MXS_NUM_TIMERS; i++) {
            MXSTimer *t = &s->timers[i];

            if (idx == TIMROT_TIMCTRL(i)) {
                val = t->ctrl;
            } else if (idx == TIMROT_RUNNING(i)) {
                static int run_budget = 60;

                val = mxs_timer_count(t, now);
                if (run_budget > 0 && i == 0) {
                    mxs_timer_diag_clocks();
                    fprintf(stderr,
                            "timrot-run0: count=%08x wall=%lld vnow=%lld "
                            "base_ns=%lld base=%08x freq=%u run=%d\n",
                            val, (long long)qemu_clock_get_ns(QEMU_CLOCK_REALTIME),
                            (long long)now, (long long)t->base_ns,
                            t->base_count, t->freq, t->running);
                    run_budget--;
                }
            } else if (idx == TIMROT_FIXED(i)) {
                val = t->fixed;
            } else if (idx == TIMROT_MATCH(i)) {
                val = t->match;
            }
        }
    }

    return mxs_bank_extract(offset, size, val);
}

static void mxs_timrot_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    MXSTimrotState *s = MXS_TIMROT(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    int i;

    if (idx == TIMROT_ROTCTRL) {
        s->rotctrl = mxs_bank_sftrst(s->rotctrl,
                                     mxs_bank_apply(s->rotctrl, offset,
                                                    value, size));
        return;
    }

    for (i = 0; i < MXS_NUM_TIMERS; i++) {
        MXSTimer *t = &s->timers[i];

        if (idx == TIMROT_TIMCTRL(i)) {
            uint32_t old = t->ctrl;
            uint32_t written = mxs_bank_shift(offset, value) &
                               mxs_bank_mask(offset, size);
            uint32_t new_ctrl = mxs_bank_apply(old, offset, value, size);
            uint32_t changed;

            /*
             * TIMCTRLn.IRQ (bit 15) is a write-one-to-clear status bit on
             * hardware; a plain mxs_bank_apply() would instead store a
             * written 1 back and latch the interrupt line forever (this
             * was observed live: TIMCTRL0 stuck at 0xc81f with the timer
             * IRQ permanently asserted into the ICOLL and the guest
             * eventually giving up with "InterruptHandle() already
             * gated").  Model the W1C semantics:
             *   - plain write:  the bit keeps its value unless written 1
             *   - set/toggle:   a written 1 clears it
             *   - clear alias:  already handled by mxs_bank_apply()
             */
            switch (MXS_BANK_OP(offset)) {
            case MXS_OP_WRITE:
                new_ctrl = (new_ctrl & ~TIMCTRL_IRQ) |
                           (old & TIMCTRL_IRQ & ~written);
                break;
            case MXS_OP_SET:
            case MXS_OP_TOG:
                if (written & TIMCTRL_IRQ) {
                    new_ctrl &= ~TIMCTRL_IRQ;
                }
                break;
            default:
                break;
            }
            changed = old ^ new_ctrl;

            brain_log_event(BSTAG('T', 'C', 'T', '0' + i), (uint32_t)new_ctrl,
                            (uint32_t)old, written, t->match);
            t->ctrl = new_ctrl;
            if (changed & (TIMCTRL_SELECT_MASK |
                           (TIMCTRL_PRESCALE_MASK << TIMCTRL_PRESCALE_SHIFT) |
                           TIMCTRL_MATCH_MODE | TIMCTRL_RELOAD)) {
                if (!(old & TIMCTRL_SELECT_MASK) &&
                    (new_ctrl & TIMCTRL_SELECT_MASK)) {
                    /* going from NEVER_TICK to a real source */
                    mxs_timer_restart(t, (new_ctrl & TIMCTRL_MATCH_MODE) ?
                                      t->base_count : t->fixed);
                } else {
                    mxs_timer_retune(t);
                }
            }
            mxs_timer_update_irq(t);
            return;
        }
        if (idx == TIMROT_FIXED(i)) {
            t->fixed = mxs_bank_apply(t->fixed, offset, value, size);
            /*
             * Writing FIXED_COUNT (re)loads the down counter unless the
             * timer free runs in match mode.
             */
            if (!(t->ctrl & TIMCTRL_MATCH_MODE)) {
                mxs_timer_restart(t, t->fixed);
            }
            return;
        }
        if (idx == TIMROT_MATCH(i)) {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            uint32_t cur = mxs_timer_count(t, now);
            uint32_t ndelta = (uint32_t)(cur - t->match);

            t->match = mxs_bank_apply(t->match, offset, value, size);
            t->fired_match_valid = false;
            brain_log_event(BSTAG('T', 'M', 'T', '0' + i), t->match, cur,
                            ndelta, t->ctrl);
            /*
             * BRAIN-DEBUG: any re-arm whose deadline is more than ~2s
             * away (at 12 MHz, 24M ticks) is anomalous for the WinCE
             * dynamic tick and worth attributing to a guest PC.
             */
            if (ndelta > 24000000 && getenv("BRAIN_TIMTRACE")) {
                fprintf(stderr,
                        "timrot-tmt: t%d match=%08x cur=%08x delta=%u (%dms) "
                        "ctrl=%04x pc=0x%08x\n",
                        i, t->match, cur, ndelta, (int)(ndelta / 12000),
                        t->ctrl, (unsigned)mxs_trace_guest_pc());
            }
            mxs_timer_resched(t);
            return;
        }
        if (idx == TIMROT_RUNNING(i)) {
            return;     /* read only */
        }
    }
}

MXS_TRACE_WRAP(mxs_timrot, "timrot")

static const MemoryRegionOps mxs_timrot_ops = {
    .read = mxs_timrot_read_tr,
    .write = mxs_timrot_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_timrot_reset(DeviceState *dev)
{
    MXSTimrotState *s = MXS_TIMROT(dev);
    int i;

    s->rotctrl = MXS_SFTRST_BIT | MXS_CLKGATE_BIT;
    for (i = 0; i < MXS_NUM_TIMERS; i++) {
        MXSTimer *t = &s->timers[i];

        timer_del(t->qtimer);
        t->ctrl = 0;
        t->fixed = 0;
        t->match = 0;
        t->fired_match = 0;
        t->fired_match_valid = false;
        t->base_count = 0;
        t->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        t->freq = 0;
        t->running = false;
        qemu_set_irq(t->irq, 0);
    }
}

static void mxs_timrot_init(Object *obj)
{
    mxs_timrot_trace = mxs_trace_enabled("timrot");

    MXSTimrotState *s = MXS_TIMROT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &mxs_timrot_ops, s, "mxs-timrot",
                          0x2000);
    sysbus_init_mmio(sbd, &s->iomem);

    for (i = 0; i < MXS_NUM_TIMERS; i++) {
        MXSTimer *t = &s->timers[i];

        t->parent = s;
        t->index = i;
        t->qtimer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mxs_timer_expire, t);
        sysbus_init_irq(sbd, &t->irq);
    }
}

static const VMStateDescription vmstate_mxs_timer = {
    .name = "mxs-timer",
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, MXSTimer),
        VMSTATE_UINT32(fixed, MXSTimer),
        VMSTATE_UINT32(match, MXSTimer),
        VMSTATE_UINT32(fired_match, MXSTimer),
        VMSTATE_BOOL(fired_match_valid, MXSTimer),
        VMSTATE_INT64(base_ns, MXSTimer),
        VMSTATE_UINT32(base_count, MXSTimer),
        VMSTATE_UINT32(freq, MXSTimer),
        VMSTATE_BOOL(running, MXSTimer),
        VMSTATE_TIMER_PTR(qtimer, MXSTimer),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_mxs_timrot = {
    .name = "mxs-timrot",
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(rotctrl, MXSTimrotState),
        VMSTATE_STRUCT_ARRAY(timers, MXSTimrotState, MXS_NUM_TIMERS, 3,
                             vmstate_mxs_timer, MXSTimer),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_timrot_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mxs_timrot_reset);
    dc->vmsd = &vmstate_mxs_timrot;
}

static const TypeInfo mxs_timrot_types[] = {
    {
        .name           = TYPE_MXS_TIMROT,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSTimrotState),
        .instance_init  = mxs_timrot_init,
        .class_init     = mxs_timrot_class_init,
    },
};

DEFINE_TYPES(mxs_timrot_types)
