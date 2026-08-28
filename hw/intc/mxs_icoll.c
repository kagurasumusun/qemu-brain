/*
 * Freescale i.MX28 (MXS) Interrupt Collector (ICOLL)
 *
 * The i.MX28 collector has one 32 bit control register per interrupt
 * source at HW_ICOLL_INTERRUPTn = 0x120 + n * 0x10 (n = 0..127), each with
 * the usual set/clear/toggle aliases.  Every source has a two bit priority;
 * the collector presents the highest priority pending source and supports
 * nesting via the priority levels.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/arm/mxs.h"
#include "hw/core/cpu.h"

#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "brain_stats.h"

/* register indices (offset / 0x10) */
#define ICOLL_VECTOR        0x00
#define ICOLL_LEVELACK      0x01
#define ICOLL_CTRL          0x02
#define ICOLL_VBASE         0x04
#define ICOLL_STAT          0x07
#define ICOLL_RAW0          0x0a
#define ICOLL_RAW1          0x0b
#define ICOLL_RAW2          0x0c
#define ICOLL_RAW3          0x0d
#define ICOLL_INTERRUPT0    0x12        /* 0x120, one register every 0x10 */
#define ICOLL_UNDEF_VECTOR  0xf2        /* 0xf20 */
#define ICOLL_VERSION       0xf3        /* 0xf30 */

#define ICOLL_INTR_PRIORITY 0x3
#define ICOLL_INTR_ENABLE   (1u << 2)
#define ICOLL_INTR_SOFTIRQ  (1u << 3)
#define ICOLL_INTR_ENFIQ    (1u << 4)

#define ICOLL_CTRL_SFTRST           (1u << 31)
#define ICOLL_CTRL_CLKGATE          (1u << 30)
#define ICOLL_CTRL_NO_NESTING       (1u << 19)
#define ICOLL_CTRL_ARM_RSE_MODE     (1u << 18)
#define ICOLL_CTRL_FIQ_FINAL_ENABLE (1u << 17)
#define ICOLL_CTRL_IRQ_FINAL_ENABLE (1u << 16)

#define ICOLL_NUM_LEVELS    4

typedef struct MXSIcollState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq fiq;

    uint32_t raw[MXS_NUM_IRQS / 32];
    uint32_t intr[MXS_NUM_IRQS];
    uint32_t ctrl;
    uint32_t vbase;
    uint32_t undef_vector;

    /* one bit per priority level currently being serviced */
    uint32_t serving;
    /* which source entered each in-service priority level (-1 = none) */
    int32_t serving_src[ICOLL_NUM_LEVELS];
    int current;            /* interrupt the collector currently presents */
    uint8_t irq_out;        /* last level driven on s->irq (edge stats) */
} MXSIcollState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSIcollState, MXS_ICOLL)

static bool brain_pin_debug(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("BRAIN_PIN_DEBUG");

        on = e && *e && *e != '0';
    }
    return on;
}

static bool mxs_icoll_asserted(MXSIcollState *s, int n)
{
    if (!(s->intr[n] & ICOLL_INTR_ENABLE)) {
        return false;
    }
    if (s->intr[n] & ICOLL_INTR_SOFTIRQ) {
        return true;
    }
    return (s->raw[n / 32] >> (n % 32)) & 1;
}

/* highest priority level that is currently being serviced, -1 if idle */
static int mxs_icoll_serving_level(MXSIcollState *s)
{
    int i;

    for (i = ICOLL_NUM_LEVELS - 1; i >= 0; i--) {
        if (s->serving & (1u << i)) {
            return i;
        }
    }
    return -1;
}

/*
 * Find the pending source with the highest priority.  Priority 3 is the
 * most urgent one; sources of equal priority are resolved by index.
 */
static int mxs_icoll_pending(MXSIcollState *s, bool fiq)
{
    int best = -1;
    int best_prio = -1;
    int i;

    for (i = 0; i < MXS_NUM_IRQS; i++) {
        int prio;

        if (!mxs_icoll_asserted(s, i)) {
            continue;
        }
        if (!!(s->intr[i] & ICOLL_INTR_ENFIQ) != fiq) {
            continue;
        }
        prio = s->intr[i] & ICOLL_INTR_PRIORITY;
        if (prio > best_prio) {
            best_prio = prio;
            best = i;
        }
    }
    return best;
}

static void mxs_icoll_update(MXSIcollState *s)
{
    int l;

    /*
     * Robustness: an in-service level whose originating source has
     * gone away (deasserted or disabled in the meantime, e.g. after
     * an aborted interrupt service) must not block the collector
     * forever.  Hardware ties the in-service state to the signalled
     * source, so release it here.
     */
    for (l = 0; l < ICOLL_NUM_LEVELS; l++) {
        if ((s->serving & (1u << l)) && s->serving_src[l] >= 0 &&
            !mxs_icoll_asserted(s, s->serving_src[l]) &&
            !(s->intr[s->serving_src[l]] & ICOLL_INTR_SOFTIRQ)) {
            brain_stat_inc(BST_ICOLL_AUTORELEASE);
            brain_log_event(BSTAG('A', 'R', 'L', 'S'), l,
                            s->serving_src[l], s->serving, s->ctrl);
            s->serving &= ~(1u << l);
            s->serving_src[l] = -1;
        }
    }

    int irq = mxs_icoll_pending(s, false);
    int fiq = mxs_icoll_pending(s, true);
    int serving = mxs_icoll_serving_level(s);
    bool assert_irq = false;

    s->current = irq;

    if (irq >= 0 && (s->ctrl & ICOLL_CTRL_IRQ_FINAL_ENABLE)) {
        int prio = s->intr[irq] & ICOLL_INTR_PRIORITY;

        if (serving < 0) {
            assert_irq = true;
        } else if (!(s->ctrl & ICOLL_CTRL_NO_NESTING) && prio > serving) {
            assert_irq = true;
        }
    }

    if (irq >= 0 && !assert_irq) {
        /* pending source masked by the in-service level (nesting off) */
        brain_stat_inc(BST_ICOLL_SUPPRESS_NEW);
        brain_log_event(BSTAG('S', 'U', 'P', 'R'), irq,
                        s->intr[irq] & ICOLL_INTR_PRIORITY, serving,
                        s->ctrl);
    }

    if (!!assert_irq != !!s->irq_out) {
        s->irq_out = assert_irq;
        brain_stat_inc(assert_irq ? BST_ICOLL_IRQ_ASSERT
                                  : BST_ICOLL_IRQ_DEASSERT);
    }

    qemu_set_irq(s->irq, assert_irq);
    qemu_set_irq(s->fiq,
                 fiq >= 0 && (s->ctrl & ICOLL_CTRL_FIQ_FINAL_ENABLE));
}

static void mxs_icoll_set_irq(void *opaque, int n, int level)
{
    MXSIcollState *s = MXS_ICOLL(opaque);

    if (n < 0 || n >= MXS_NUM_IRQS) {
        return;
    }
    if (n == 63 && brain_pin_debug()) {
        fprintf(stderr, "[brain] irq63 line=%d pc=0x%08x intr63=0x%x\n",
                level, mxs_trace_guest_pc(), s->intr[63]);
    }
    if (n == 48 && brain_pin_debug()) {
        static int dbg48 = 3000;
        if (dbg48 > 0) {
            dbg48--;
            fprintf(stderr, "[brain] icoll-irq48 line=%d raw0=%08x vnow=%lld\n",
                    level, s->raw[0],
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
    }
    if ((n == 10 || n == 18) && brain_pin_debug()) {
        fprintf(stderr, "[brain] icoll-irq%-2d line=%d raw%d=%08x intr%d=%08x "
                "pc=0x%08x vnow=%lld\n", n, level, n / 32, s->raw[n / 32],
                n, s->intr[n], mxs_trace_guest_pc(),
                (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    if (n == 33 && brain_pin_debug()) {
        fprintf(stderr, "[brain] icoll-irq33 line=%d raw0=%08x intr33=%08x "
                "pc=0x%08x vnow=%lld\n", level, s->raw[0],
                s->intr[33], mxs_trace_guest_pc(),
                (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    if (n == 125 && brain_pin_debug()) {
        fprintf(stderr, "[brain] icoll-irq125 line=%d raw3=%08x "
                "intr125=%08x pc=0x%08x vnow=%lld\n", level, s->raw[3],
                s->intr[125], mxs_trace_guest_pc(),
                (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    if (n == 61 && brain_pin_debug()) {
        fprintf(stderr, "[brain] icoll-irq61 line=%d raw1=%08x intr61=%08x "
                "pc=0x%08x vnow=%lld\n", level, s->raw[1], s->intr[61],
                mxs_trace_guest_pc(),
                (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    if (level) {
        s->raw[n / 32] |= 1u << (n % 32);
    } else {
        s->raw[n / 32] &= ~(1u << (n % 32));
    }
    mxs_icoll_update(s);
}

static uint64_t mxs_icoll_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSIcollState *s = MXS_ICOLL(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val = 0;

    if (idx >= ICOLL_INTERRUPT0 && idx < ICOLL_INTERRUPT0 + MXS_NUM_IRQS) {
        return mxs_bank_extract(offset, size, s->intr[idx - ICOLL_INTERRUPT0]);
    }

    switch (idx) {
    case ICOLL_VECTOR:
        /*
         * In ARM read-side-effect mode, reading the vector register
         * acknowledges the interrupt: the collector enters the priority
         * level of the source so that only higher levels can preempt it.
         */
        if (s->current >= 0) {
            val = s->vbase + ((uint32_t)s->current << 2);
            if (s->ctrl & ICOLL_CTRL_ARM_RSE_MODE) {
                unsigned p = s->intr[s->current] & ICOLL_INTR_PRIORITY;

                if ((s->current == 63 || s->current == 125 ||
                     s->current == 61) && brain_pin_debug()) {
                    fprintf(stderr, "[brain] VECTOR read -> irq%d (prio %u) "
                            "intr125=%08x raw3=%08x pc=0x%08x vnow=%lld\n",
                            s->current, p, s->intr[125], s->raw[3],
                            mxs_trace_guest_pc(),
                            (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
                }
                brain_stat_inc(BST_ICOLL_VECTOR_RSE);
                brain_log_event(BSTAG('V', 'E', 'C', 'R'), s->current, p,
                                s->serving | (1u << p), s->ctrl);
                s->serving |= 1u << p;
                s->serving_src[p] = s->current;
                mxs_icoll_update(s);
            }
        } else {
            val = s->undef_vector ? s->undef_vector : s->vbase;
        }
        break;
    case ICOLL_LEVELACK:
        val = 0;
        break;
    case ICOLL_CTRL:
        val = s->ctrl;
        break;
    case ICOLL_VBASE:
        val = s->vbase;
        break;
    case ICOLL_STAT:
        val = s->current >= 0 ? s->current : 0x7f;
        break;
    case ICOLL_RAW0:
    case ICOLL_RAW1:
    case ICOLL_RAW2:
    case ICOLL_RAW3:
        val = s->raw[idx - ICOLL_RAW0];
        break;
    case ICOLL_UNDEF_VECTOR:
        val = s->undef_vector;
        break;
    case ICOLL_VERSION:
        val = 0x02000000;
        break;
    default:
        break;
    }

    return mxs_bank_extract(offset, size, val);
}

static void mxs_icoll_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    MXSIcollState *s = MXS_ICOLL(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);

    if (idx >= ICOLL_INTERRUPT0 && idx < ICOLL_INTERRUPT0 + MXS_NUM_IRQS) {
        unsigned n = idx - ICOLL_INTERRUPT0;

        if (getenv("BRAIN_PIN_DEBUG")) {
            fprintf(stderr, "[brain] W INTR%-3d off=0x%03x val=0x%08x "
                    "pc=0x%08x (was 0x%x)\n", n, (unsigned)offset,
                    (uint32_t)value, mxs_trace_guest_pc(), s->intr[n]);
        }
        s->intr[n] = mxs_bank_apply(s->intr[n], offset, value, size);
        mxs_icoll_update(s);
        return;
    }

    switch (idx) {
    case ICOLL_VECTOR:
        /*
         * Writing the vector register signals the end of the interrupt
         * service routine and pops the current priority level.
         */
        if (s->serving) {
            int lvl = mxs_icoll_serving_level(s);

            if (lvl >= 0) {
                s->serving &= ~(1u << lvl);
                s->serving_src[lvl] = -1;
            }
        }
        mxs_icoll_update(s);
        break;
    case ICOLL_LEVELACK:
        /* one bit per level; acknowledging leaves that level */
        {
            uint32_t clr = s->serving & ((uint32_t)value & 0xf);
            int l;

            brain_stat_inc(BST_ICOLL_LEVELACK);
            brain_log_event(BSTAG('L', 'A', 'C', 'K'), (uint32_t)value,
                            s->serving, s->serving & ~(value & 0xf) & 0xf,
                            s->ctrl);
            for (l = 0; l < ICOLL_NUM_LEVELS; l++) {
                if (clr & (1u << l)) {
                    s->serving_src[l] = -1;
                }
            }
        }
        s->serving &= ~((uint32_t)value & 0xf);
        mxs_icoll_update(s);
        break;
    case ICOLL_CTRL: {
        uint32_t old = s->ctrl;

        if ((uint32_t)value != old && brain_pin_debug()) {
            fprintf(stderr,
                    "[brain] icoll CTRL wr: off=%03x val=%08x old=%08x "
                    "new=%08x pc=%08x vnow=%lld\n",
                    (unsigned)offset, (uint32_t)value, old,
                    (uint32_t)(mxs_bank_apply(old, offset, value, size)),
                    mxs_trace_guest_pc(),
                    (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        s->ctrl = mxs_bank_sftrst(old,
                                  mxs_bank_apply(old, offset, value, size));
        if ((s->ctrl & ICOLL_CTRL_SFTRST) && !(old & ICOLL_CTRL_SFTRST)) {
            memset(s->intr, 0, sizeof(s->intr));
            s->serving = 0;
            memset(s->serving_src, -1, sizeof(s->serving_src));
        }
        mxs_icoll_update(s);
        break;
    }
    case ICOLL_VBASE:
        s->vbase = mxs_bank_apply(s->vbase, offset, value, size);
        break;
    case ICOLL_UNDEF_VECTOR:
        s->undef_vector = mxs_bank_apply(s->undef_vector, offset, value, size);
        break;
    default:
        break;
    }
}

MXS_TRACE_WRAP(mxs_icoll, "icoll")

static const MemoryRegionOps mxs_icoll_ops = {
    .read = mxs_icoll_read_tr,
    .write = mxs_icoll_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_icoll_reset(DeviceState *dev)
{
    MXSIcollState *s = MXS_ICOLL(dev);

    memset(s->raw, 0, sizeof(s->raw));
    memset(s->intr, 0, sizeof(s->intr));
    s->ctrl = ICOLL_CTRL_SFTRST | ICOLL_CTRL_CLKGATE;
    s->vbase = 0;
    s->undef_vector = 0;
    s->serving = 0;
    memset(s->serving_src, -1, sizeof(s->serving_src));
    s->current = -1;
    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->fiq, 0);
}

static void mxs_icoll_init(Object *obj)
{
    mxs_icoll_trace = mxs_trace_enabled("icoll");

    MXSIcollState *s = MXS_ICOLL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &mxs_icoll_ops, s, "mxs-icoll",
                          0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(DEVICE(obj), mxs_icoll_set_irq, MXS_NUM_IRQS);
}

static const VMStateDescription vmstate_mxs_icoll = {
    .name = "mxs-icoll",
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(raw, MXSIcollState, MXS_NUM_IRQS / 32),
        VMSTATE_UINT32_ARRAY(intr, MXSIcollState, MXS_NUM_IRQS),
        VMSTATE_UINT32(ctrl, MXSIcollState),
        VMSTATE_UINT32(vbase, MXSIcollState),
        VMSTATE_UINT32(undef_vector, MXSIcollState),
        VMSTATE_UINT32(serving, MXSIcollState),
        VMSTATE_INT32_ARRAY(serving_src, MXSIcollState, ICOLL_NUM_LEVELS),
        VMSTATE_INT32(current, MXSIcollState),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_icoll_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mxs_icoll_reset);
    dc->vmsd = &vmstate_mxs_icoll;
}

static const TypeInfo mxs_icoll_types[] = {
    {
        .name           = TYPE_MXS_ICOLL,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSIcollState),
        .instance_init  = mxs_icoll_init,
        .class_init     = mxs_icoll_class_init,
    },
};

DEFINE_TYPES(mxs_icoll_types)
