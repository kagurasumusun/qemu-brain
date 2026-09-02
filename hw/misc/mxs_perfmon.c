/*
 * Freescale i.MX28 Performance Monitor (PERFMON)
 *
 * The PERFMON watches AXI traffic and reports active cycles, transfer
 * count, total and maximum latency and transferred data count, with an
 * optional address trap and a latency threshold, both of which can
 * raise an interrupt.  It is a real i.MX28 block (MCIMX28RM Rev 2
 * chapter 21, mapped at 0x80006000) and is modelled here rather than
 * left as an access-swallowing placeholder.
 *
 * Modelled:
 *   - the register bank with the MXS set/clr/tog aliases and the
 *     chapter's reset values (CTRL C000_0000, DEBUG 0000_0001,
 *     VERSION 0100_0000)
 *   - RUN gating of the statistics collection
 *   - ACTIVE_CYCLE counting real elapsed virtual clock cycles while RUN
 *     is set, lazily accumulated so no timer is needed
 *   - SNAP: latches the live counters into the shadow registers that
 *     reads return, then self-clears (as the manual specifies)
 *   - CLR: zeroes the statistics registers, then self-clears
 *   - the trap range, latency threshold, per-master enables and the
 *     three interrupt enables/status bits, all readable and writable
 *
 * Not modelled: actually detecting AXI transactions.  QEMU has no AXI
 * interconnect to observe, so TRANSFER_COUNT, TOTAL_LATENCY, DATA_COUNT
 * and MAX_LATENCY stay at zero and TRAP_IRQ / LATENCY_IRQ are never
 * raised by hardware.  Software that only configures and reads the
 * block -- which is all the WinCE image's OAL does -- sees correct
 * reset values, correct RUN/SNAP/CLR behaviour and a live cycle count.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/mxs_bank.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_MXS_PERFMON "mxs-perfmon"
OBJECT_DECLARE_SIMPLE_TYPE(MXSPERFMONState, MXS_PERFMON)

#define PM_CTRL             0x000
#define PM_MASTER_EN        0x010
#define PM_TRAP_ADDR_LOW    0x020
#define PM_TRAP_ADDR_HIGH   0x030
#define PM_LAT_THRESHOLD    0x040
#define PM_ACTIVE_CYCLE     0x050
#define PM_TRANSFER_COUNT   0x060
#define PM_TOTAL_LATENCY    0x070
#define PM_DATA_COUNT       0x080
#define PM_MAX_LATENCY      0x090
#define PM_DEBUG            0x0A0
#define PM_VERSION          0x0B0
#define PM_NREGS            (0x0C0 >> 4)

/* indices of the five statistics registers, kept contiguous for CLR */
#define PM_STAT_FIRST       (PM_ACTIVE_CYCLE >> 4)
#define PM_STAT_LAST        (PM_MAX_LATENCY >> 4)

/* HW_PERFMON_CTRL */
#define PM_CTRL_SFTRST          (1u << 31)
#define PM_CTRL_CLKGATE         (1u << 30)
#define PM_CTRL_IRQ_MID_SHIFT   16
#define PM_CTRL_BUS_ERR_IRQ     (1u << 12)
#define PM_CTRL_LATENCY_IRQ     (1u << 11)
#define PM_CTRL_TRAP_IRQ        (1u << 10)
#define PM_CTRL_BUS_ERR_IRQ_EN  (1u << 9)
#define PM_CTRL_LATENCY_IRQ_EN  (1u << 8)
#define PM_CTRL_TRAP_IRQ_EN     (1u << 7)
#define PM_CTRL_LATENCY_ENABLE  (1u << 6)
#define PM_CTRL_TRAP_IN_RANGE   (1u << 5)
#define PM_CTRL_TRAP_ENABLE     (1u << 4)
#define PM_CTRL_READ_EN         (1u << 3)
#define PM_CTRL_CLR             (1u << 2)
#define PM_CTRL_SNAP            (1u << 1)
#define PM_CTRL_RUN             (1u << 0)

struct MXSPERFMONState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[PM_NREGS];

    /*
     * Shadow registers that reads return.  SNAP copies the live values
     * here; until the first snapshot the shadow mirrors the live
     * counters so a read is never stale garbage.
     */
    uint32_t shadow[PM_STAT_LAST - PM_STAT_FIRST + 1];

    /* virtual-clock time at which the current RUN period started, and
     * the cycle count accumulated by previous RUN periods */
    int64_t run_start_ns;
    uint64_t cycles_banked;
    bool running;
};

/*
 * Fold the elapsed virtual time of the current RUN period into the
 * counter.  Called before anything reads or stops the count so the
 * register always reflects "now" without needing a periodic timer.
 */
static void pm_accumulate(MXSPERFMONState *s)
{
    if (!s->running) {
        return;
    }
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed = now - s->run_start_ns;

    if (elapsed > 0) {
        /*
         * The counter is an AXI clock count.  The block runs off the
         * 24 MHz reference in the absence of a modelled AXI clock, so
         * convert nanoseconds at that rate.
         */
        s->cycles_banked += (uint64_t)elapsed * 24ull / 1000ull;
        s->run_start_ns = now;
    }
    s->regs[PM_ACTIVE_CYCLE >> 4] = (uint32_t)s->cycles_banked;
}

static void pm_start(MXSPERFMONState *s)
{
    if (!s->running) {
        s->running = true;
        s->run_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static void pm_stop(MXSPERFMONState *s)
{
    if (s->running) {
        pm_accumulate(s);
        s->running = false;
    }
}

static uint64_t mxs_perfmon_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSPERFMONState *s = opaque;
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t v;

    if (idx >= PM_STAT_FIRST && idx <= PM_STAT_LAST) {
        pm_accumulate(s);
        /* statistics reads come from the shadow registers */
        v = s->shadow[idx - PM_STAT_FIRST];
    } else {
        v = s->regs[idx];
    }
    return mxs_bank_extract(offset, size, v);
}

static void mxs_perfmon_write(void *opaque, hwaddr offset, uint64_t val,
                              unsigned size)
{
    MXSPERFMONState *s = opaque;
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t old = s->regs[idx];
    uint32_t v = mxs_bank_apply(old, offset, val, size);

    /* Statistics registers are read-only. */
    if (idx >= PM_STAT_FIRST && idx <= PM_STAT_LAST) {
        return;
    }
    if (idx == PM_VERSION >> 4) {
        return;
    }
    /* SFTRST/CLKGATE only exist in CTRL; do not apply the edge rule to
     * every register or a write with bit 31 set would be corrupted. */
    if ((offset & ~0xfu) == PM_CTRL) {
        v = mxs_bank_sftrst(old, v);
    }
    s->regs[idx] = v;

    if ((offset & ~0xfu) == PM_CTRL) {
        if (v & PM_CTRL_SFTRST) {
            /* Coming out of reset returns the block to its reset state. */
            memset(s->regs, 0, sizeof(s->regs));
            memset(s->shadow, 0, sizeof(s->shadow));
            s->cycles_banked = 0;
            s->running = false;
            s->regs[PM_CTRL >> 4] = PM_CTRL_SFTRST | PM_CTRL_CLKGATE;
            s->regs[PM_DEBUG >> 4] = 1;
            s->regs[PM_VERSION >> 4] = 0x01000000;
            qemu_irq_lower(s->irq);
            return;
        }

        /* SNAP: copy live counters into the readable shadow, self-clear */
        if (v & PM_CTRL_SNAP) {
            pm_accumulate(s);
            for (unsigned i = PM_STAT_FIRST; i <= PM_STAT_LAST; i++) {
                s->shadow[i - PM_STAT_FIRST] = s->regs[i];
            }
            v &= ~PM_CTRL_SNAP;
            s->regs[idx] = v;
        }
        /* CLR: zero the statistics, self-clear */
        if (v & PM_CTRL_CLR) {
            pm_stop(s);
            for (unsigned i = PM_STAT_FIRST; i <= PM_STAT_LAST; i++) {
                s->regs[i] = 0;
                s->shadow[i - PM_STAT_FIRST] = 0;
            }
            s->cycles_banked = 0;
            v &= ~PM_CTRL_CLR;
            s->regs[idx] = v;
            if (v & PM_CTRL_RUN) {
                pm_start(s);
            }
        }

        if (v & PM_CTRL_RUN) {
            pm_start(s);
        } else {
            pm_stop(s);
        }

        /*
         * The three status bits are cleared by writing one to their SET
         * clear address, which mxs_bank_apply() has already folded into
         * v; re-derive the interrupt line from what is still pending.
         */
        if (v & (PM_CTRL_BUS_ERR_IRQ | PM_CTRL_LATENCY_IRQ |
                 PM_CTRL_TRAP_IRQ)) {
            qemu_irq_raise(s->irq);
        } else {
            qemu_irq_lower(s->irq);
        }
    }
}

static const MemoryRegionOps mxs_perfmon_ops = {
    .read = mxs_perfmon_read,
    .write = mxs_perfmon_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void mxs_perfmon_reset(DeviceState *dev)
{
    MXSPERFMONState *s = MXS_PERFMON(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->shadow, 0, sizeof(s->shadow));
    s->cycles_banked = 0;
    s->running = false;
    s->run_start_ns = 0;

    /* chapter 21 reset values */
    s->regs[PM_CTRL >> 4] = PM_CTRL_SFTRST | PM_CTRL_CLKGATE;
    s->regs[PM_DEBUG >> 4] = 0x00000001;
    s->regs[PM_VERSION >> 4] = 0x01000000;
    qemu_irq_lower(s->irq);
}

static void mxs_perfmon_realize(DeviceState *dev, Error **errp)
{
    MXSPERFMONState *s = MXS_PERFMON(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mxs_perfmon_ops, s,
                          TYPE_MXS_PERFMON, 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_mxs_perfmon = {
    .name = TYPE_MXS_PERFMON,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSPERFMONState, PM_NREGS),
        VMSTATE_UINT32_ARRAY(shadow, MXSPERFMONState,
                             PM_STAT_LAST - PM_STAT_FIRST + 1),
        VMSTATE_INT64(run_start_ns, MXSPERFMONState),
        VMSTATE_UINT64(cycles_banked, MXSPERFMONState),
        VMSTATE_BOOL(running, MXSPERFMONState),
        VMSTATE_END_OF_LIST()
    },
};

static void mxs_perfmon_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mxs_perfmon_realize;
    device_class_set_legacy_reset(dc, mxs_perfmon_reset);
    dc->vmsd = &vmstate_mxs_perfmon;
    dc->desc = "Freescale i.MX28 Performance Monitor";
}

static const TypeInfo mxs_perfmon_info = {
    .name = TYPE_MXS_PERFMON,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MXSPERFMONState),
    .class_init = mxs_perfmon_class_init,
};

static void mxs_perfmon_register_types(void)
{
    type_register_static(&mxs_perfmon_info);
}

type_init(mxs_perfmon_register_types)
