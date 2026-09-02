/*
 * Freescale i.MX28 Default First-Level Page Table (DFLPT).
 *
 * RM chapter 3.  A 16-Kbyte AHB slave at 0x800C_0000-0x800C_3FFF that
 * implements the ARM926EJ-S MMU's level-1 page table in hardware: 4096
 * 32-bit section descriptors, of which 16 are "movable" (MPTE0-15) and
 * one (entry 2048) is a semi-programmable fixed descriptor covering the
 * 1-Mbyte PIO/register space (0x80000000-0x800FFFFF, virtual == real).
 *
 * The 16 MPTEs are fully programmable in value and location:
 *   - the *value* (the 32-bit section descriptor) is programmed by a
 *     write to the bound PTE location 0x800C_0000 + (LOC << 2);
 *   - the *location* (LOC/SPAN/DIS) is programmed through the
 *     HW_DIGCTL_MPTEn_LOC registers in the DIGCTL block, which this
 *     model reads through mxs_digctl_mpte_loc().
 *
 * Behaviour (RM 3.2):
 *   - a read of an unbound PTE returns 0 (a section translation fault
 *     if the access is a page-table walk);
 *   - a write to an unbound PTE returns a slave error, which the AHB
 *     turns into an ARM data abort.  This model returns MEMTX_ERROR so
 *     QEMU's bus-error policy (mxs machine: ignore_memory_transaction_
 *     failures, default off) delivers the abort like real silicon;
 *   - each MPTE has a reset value of 0 and HW_DIGCTL_MPTEn_LOC resets
 *     to n, so at reset entries 0..15 are bound one-to-one to MPTE0-15;
 *   - a spanned MPTE returns "value + ((index - LOC) << 20)": the base
 *     address grows linearly across the span (RM figure 3-3);
 *   - entry 2048 resets to 0x80000C12 (POINTER=0x80000, AP=3, DOMAIN=0,
 *     C=0, B=0, first-level=2).  Only AP (11:10), DOMAIN (8:5) and
 *     BUFFERABLE (2) are writable; everything else reads back fixed.
 *
 * What is intentionally not modelled: the DFLPT is an MMU-walk
 * accelerator, so its AHB read/write ports are the whole interface; it
 * has no interrupt, no clock gating and no side effects beyond the
 * descriptor store.  The documented fixed 1-cycle wait state and burst
 * acceptance have no functional effect in QEMU.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/mxs_bank.h"
#include "hw/misc/mxs_dflpt.h"
#include "hw/misc/mxs_syscon.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define DFLPT_PTE_COUNT     4096
#define DFLPT_PTE_BYTES     (DFLPT_PTE_COUNT * 4)
#define DFLPT_FIXED_ENTRY   2048

/* writable fields of the fixed PIO descriptor: AP(11:10) DOMAIN(8:5) B(2) */
#define DFLPT_FIXED_RW_MASK 0x00000fe4u
#define DFLPT_FIXED_RESET   0x80000c12u

#define DFLPT_MPTE_DIS      (1u << 31)
#define DFLPT_MPTE_SPAN_SH  24
#define DFLPT_MPTE_SPAN_MSK (7u << DFLPT_MPTE_SPAN_SH)
#define DFLPT_MPTE_LOC_MSK  0x00000fffu

typedef struct MXSDFLPTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t mpte[16];      /* the 16 movable PTE descriptor values */
    uint32_t fixed;         /* PTE 2048, the fixed PIO descriptor */
    DeviceState *digctl;    /* source of HW_DIGCTL_MPTEn_LOC */
    char *name;
    bool trace;
} MXSDFLPTState;

#define TYPE_MXS_DFLPT "mxs-dflpt"

OBJECT_DECLARE_SIMPLE_TYPE(MXSDFLPTState, MXS_DFLPT)

/*
 * Find which MPTE (0..15) binds a PTE index, or -1 if the entry is
 * unbound.  Overlapping/duplicate LOC programming is documented as
 * non-deterministic on real silicon; we deterministically return the
 * lowest-numbered enabled MPTE whose span covers the index.
 */
static int mxs_dflpt_binding(MXSDFLPTState *s, unsigned idx)
{
    int n;

    for (n = 0; n < 16; n++) {
        uint32_t w = mxs_digctl_mpte_loc(s->digctl, n);
        unsigned loc, span;

        if (w & DFLPT_MPTE_DIS) {
            continue;
        }
        loc = w & DFLPT_MPTE_LOC_MSK;
        span = 1u << ((w & DFLPT_MPTE_SPAN_MSK) >> DFLPT_MPTE_SPAN_SH);
        if (idx >= loc && idx - loc < span) {
            return n;
        }
    }
    return -1;
}

static MemTxResult mxs_dflpt_read(void *opaque, hwaddr offset, uint64_t *data,
                                  unsigned size, MemTxAttrs attrs)
{
    MXSDFLPTState *s = MXS_DFLPT(opaque);
    uint32_t val = 0;

    if (offset < DFLPT_PTE_BYTES) {
        unsigned idx = offset >> 2;

        if (idx == DFLPT_FIXED_ENTRY) {
            val = s->fixed;
        } else {
            int n = mxs_dflpt_binding(s, idx);

            if (n >= 0) {
                uint32_t w = mxs_digctl_mpte_loc(s->digctl, n);
                unsigned loc = w & DFLPT_MPTE_LOC_MSK;

                /* spanned entries get a linearly grown base address */
                val = s->mpte[n] + ((idx - loc) << 20);
            } else {
                val = 0;        /* unbound: section fault on a walk */
            }
        }
        if (size < 4) {
            val = (val >> ((offset & 3) * 8)) & ((1u << (size * 8)) - 1);
        }
    }
    /* offsets beyond the 16-Kbyte PTE space are reserved: read zero */
    *data = val;
    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access_word(s->name, false, offset, val);
    }
    return MEMTX_OK;
}

static MemTxResult mxs_dflpt_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size, MemTxAttrs attrs)
{
    MXSDFLPTState *s = MXS_DFLPT(opaque);

    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access_word(s->name, true, offset, (uint32_t)value);
    }

    /* reserved space above the 16-Kbyte PTE area */
    if (offset >= DFLPT_PTE_BYTES) {
        return MEMTX_OK;
    }

    {
        unsigned idx = offset >> 2;

        if (idx == DFLPT_FIXED_ENTRY) {
            s->fixed = (s->fixed & ~DFLPT_FIXED_RW_MASK) |
                       ((uint32_t)value & DFLPT_FIXED_RW_MASK);
            return MEMTX_OK;
        }

        {
            int n = mxs_dflpt_binding(s, idx);

            if (n >= 0) {
                s->mpte[n] = (uint32_t)value;
                return MEMTX_OK;
            }
        }
    }

    /* write to an unbound PTE: slave error -> ARM data abort */
    return MEMTX_ERROR;
}

static const MemoryRegionOps mxs_dflpt_ops = {
    .read_with_attrs = mxs_dflpt_read,
    .write_with_attrs = mxs_dflpt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void mxs_dflpt_reset(DeviceState *dev)
{
    MXSDFLPTState *s = MXS_DFLPT(dev);

    memset(s->mpte, 0, sizeof(s->mpte));
    s->fixed = DFLPT_FIXED_RESET;
}

static void mxs_dflpt_realize(DeviceState *dev, Error **errp)
{
    MXSDFLPTState *s = MXS_DFLPT(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->trace = mxs_trace_enabled(s->name ? s->name : TYPE_MXS_DFLPT);

    /* the DIGCTL link is optional: without it every MPTE reads unbound */
    if (!s->digctl) {
        warn_report("mxs-dflpt: no DIGCTL link; all MPTEs treated unbound");
    }

    memory_region_init_io(&s->iomem, OBJECT(dev), &mxs_dflpt_ops, s,
                          TYPE_MXS_DFLPT, 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_mxs_dflpt = {
    .name = TYPE_MXS_DFLPT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(mpte, MXSDFLPTState, 16),
        VMSTATE_UINT32(fixed, MXSDFLPTState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property mxs_dflpt_properties[] = {
    DEFINE_PROP_STRING("name", MXSDFLPTState, name),
};

static void mxs_dflpt_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mxs_dflpt_realize;
    device_class_set_legacy_reset(dc, mxs_dflpt_reset);
    dc->vmsd = &vmstate_mxs_dflpt;
    device_class_set_props(dc, mxs_dflpt_properties);
}

static const TypeInfo mxs_dflpt_type = {
    .name = TYPE_MXS_DFLPT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MXSDFLPTState),
    .class_init = mxs_dflpt_class_init,
};

static void mxs_dflpt_register_types(void)
{
    type_register_static(&mxs_dflpt_type);
}
type_init(mxs_dflpt_register_types)

void mxs_dflpt_set_digctl(DeviceState *dflpt, DeviceState *digctl)
{
    MXSDFLPTState *s = MXS_DFLPT(dflpt);

    s->digctl = digctl;
}
