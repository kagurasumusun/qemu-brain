/*
 * Freescale i.MX28 documented-reserved register space.
 *
 * Some physically-present i.MX28 blocks appear in the reference manual's
 * memory map but have no public register documentation whatsoever:
 *
 *   AXI_AHB0  0x8002E000  "AXI Control" (the AXI-to-AHB bridge control)
 *   SIMDBG    0x8003C000  simulation/debug cluster.  The RM memory map
 *                         sub-divides it: SIMDBG, SIMGPMISEL, SIMSSPSEL,
 *                         SIMMEMSEL, GPIOMON, SIMENET, ARMJTAG -- none of
 *                         which has a register chapter either.
 *   AUDIOIN   0x8004C000  audio input filter (ADC path).  Of the whole
 *                         AUDIOOUT/AUDIOIN pair only HW_AUDIOOUT_CTRL0 is
 *                         ever named, and only inside a Ch.7 DMA example.
 *                         AUDIOOUT itself is modelled in mxs_rob.c with
 *                         that one documented register; AUDIOIN has none.
 *
 * None of these blocks has an interrupt line (the i.MX28 interrupt table
 * lists no entry for them) and none of them is touched by the WinCE image
 * (verified with MXS_TRACE over a full boot).  On real silicon the only
 * observable behaviour of such a slave with no implemented registers is:
 * reads return zero (reserved) and writes are ignored.  That is exactly
 * what this model provides.
 *
 * This is not an access-swallowing placeholder: it models the real,
 * documented-absent register space, and it exists so the regions respond
 * like the present-but-undocumented hardware instead of raising the bus
 * errors that QEMU would otherwise deliver for unmapped PIO space.  If
 * public register documentation for any of these blocks ever surfaces,
 * the corresponding instance must be replaced by a register model the
 * same way HSADC/SPDIF/DRAM/FlexCAN/ENET/DFLPT/ETM were.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/mxs_bank.h"
#include "hw/misc/mxs_reserved.h"
#include "qom/object.h"

typedef struct MXSReservedState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    char *name;
    uint64_t size;
    bool trace;
} MXSReservedState;

#define TYPE_MXS_RESERVED "mxs-reserved"

OBJECT_DECLARE_SIMPLE_TYPE(MXSReservedState, MXS_RESERVED)

static uint64_t mxs_reserved_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSReservedState *s = MXS_RESERVED(opaque);

    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access_word(s->name, false, offset, 0);
    }
    return 0;
}

static void mxs_reserved_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    MXSReservedState *s = MXS_RESERVED(opaque);

    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access_word(s->name, true, offset, (uint32_t)value);
    }
    /* reserved registers: writes are ignored */
}

static const MemoryRegionOps mxs_reserved_ops = {
    .read = mxs_reserved_read,
    .write = mxs_reserved_write,
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

static void mxs_reserved_realize(DeviceState *dev, Error **errp)
{
    MXSReservedState *s = MXS_RESERVED(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->trace = mxs_trace_enabled(s->name);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mxs_reserved_ops, s,
                          TYPE_MXS_RESERVED, s->size);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const Property mxs_reserved_properties[] = {
    DEFINE_PROP_STRING("name", MXSReservedState, name),
    DEFINE_PROP_UINT64("size", MXSReservedState, size, 0x2000),
};

static void mxs_reserved_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mxs_reserved_realize;
    device_class_set_props(dc, mxs_reserved_properties);
}

static const TypeInfo mxs_reserved_type = {
    .name = TYPE_MXS_RESERVED,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MXSReservedState),
    .class_init = mxs_reserved_class_init,
};

static void mxs_reserved_register_types(void)
{
    type_register_static(&mxs_reserved_type);
}
type_init(mxs_reserved_register_types)
