/*
 * Freescale i.MX28 Embedded Trace Macrocell (ARM CoreSight ETM9CSSingle).
 *
 * RM 2.4: the i.MX28 includes a stand-alone ARM CoreSight Embedded Trace
 * Macrocell, "ETM9CSSingle", which provides instruction and data trace for
 * the ARM926EJ-S core; the reader is pointed at the CoreSight ETM9 TRM.
 * The block lives at 0x80022000-0x80023FFF (8KB).  It has no interrupt
 * line (it is a trace *source*; its output is the trace port), and the
 * WinCE image never touches it (verified with MXS_TRACE over a full boot).
 * The ETM is on JTAG scan chain 6; the RM also documents that a JTAG
 * reset can be issued by writing 0xDEADC0DE to "ETM address 0x70".
 *
 * Register map: the ETMv1.3 programmer's model (ARM IHI 0014 "Embedded
 * Trace Macrocell Architecture Specification", and the ETM9 TRM
 * DDI 0157G), with CoreSight identification registers.  Register N sits
 * at byte offset 4*N:
 *
 *   0x000  ETMCR      Main Control                 RW
 *   0x004  ETMCCR     Configuration Code           RO   (impl-defined, 0)
 *   0x008  ETMTRIGGER Trigger Event                WO   (reads 0)
 *   0x010  ETMSR      Status                       RO
 *   0x014  ETMSCR     System Configuration         RW
 *   0x020  ETMTEEVR   TraceEnable Event            RW
 *   0x024  ETMTECR1   TraceEnable Control 1        RW
 *   0x028  ETMFFRR    FIFOFULL Region              RW
 *   0x200  ETMTEEVR2                               RW
 *   0x204  ETMTEEVR3                               RW
 *   0xFB0  ETMIDR     ID Register                  RO   (impl-defined, 0)
 *   0xFB4  ETMCCER    Configuration Code Extension RO   (impl-defined, 0)
 *   0xFC8  ETMDEVTYPE                              RO   0x13 (trace source)
 *   0xFD0-0xFDC PeripheralID4..7                   RO   (impl-defined, 0)
 *   0xFE0-0xFEC PeripheralID0..3                   RO   (impl-defined, 0)
 *   0xFF0-0xFFC ComponentID0..3                    RO   0xB105900D
 *
 * Reset values: the control registers reset to zero (tracing disabled).
 * ETMCCR / ETMIDR / ETMCCER and the CoreSight Peripheral ID encode the
 * exact ETM9 instance configuration (numbers of comparators, counters,
 * FIFO size, ...); that configuration is not published for the i.MX28, so
 * rather than invent a configuration code this model reads those fields
 * as 0.  ETMDEVTYPE and the CoreSight Component ID are architecturally
 * fixed values.
 *
 * Not modelled (documented, not faked): the trace engine itself.  There
 * is no instruction/data trace generation, no trace FIFO and no trace
 * port; ETMSR never reports overflow, and the ETMCR.Programming gating of
 * register writes is not enforced.  None of this is observable without
 * attaching a trace port analyser, which qemu-brain does not do.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/mxs_bank.h"
#include "hw/misc/mxs_etm.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define ETM_CR          0x000
#define ETM_CCR         0x004
#define ETM_TRIGGER     0x008
#define ETM_SR          0x010
#define ETM_SCR         0x014
#define ETM_TEEVR       0x020
#define ETM_TECR1       0x024
#define ETM_FFRR        0x028
#define ETM_TEEVR2      0x200
#define ETM_TEEVR3      0x204
#define ETM_IDR         0xFB0
#define ETM_CCER        0xFB4
#define ETM_DEVTYPE     0xFC8
#define ETM_CIDR0       0xFF0
#define ETM_CIDR1       0xFF4
#define ETM_CIDR2       0xFF8
#define ETM_CIDR3       0xFFC

typedef struct MXSETMState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t cr, scr, teevr, tecr1, ffrr, teevr2, teevr3;
    char *name;
    uint64_t size;
    bool trace;
} MXSETMState;

#define TYPE_MXS_ETM "mxs-etm"

OBJECT_DECLARE_SIMPLE_TYPE(MXSETMState, MXS_ETM)

static uint32_t mxs_etm_extract(uint32_t v, hwaddr offset, unsigned size)
{
    unsigned byte = offset & 3;

    if (size >= 4) {
        return v;
    }
    return (v >> (byte * 8)) & ((1u << (size * 8)) - 1);
}

static uint32_t mxs_etm_apply(uint32_t old, hwaddr offset, uint64_t value,
                              unsigned size)
{
    unsigned byte = offset & 3;
    uint32_t mask = (size >= 4) ? 0xffffffffu
                                : ((1u << (size * 8)) - 1) << (byte * 8);

    return (old & ~mask) | (((uint32_t)value << (byte * 8)) & mask);
}

static uint64_t mxs_etm_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSETMState *s = MXS_ETM(opaque);
    uint32_t v = 0;

    switch (offset & ~3u) {
    case ETM_CR:     v = s->cr;     break;
    case ETM_CCR:    v = 0;         break;  /* impl-defined config code */
    case ETM_TRIGGER: v = 0;        break;  /* write-only */
    case ETM_SR:     v = 0;         break;  /* no overflow, not programming */
    case ETM_SCR:    v = s->scr;    break;
    case ETM_TEEVR:  v = s->teevr;  break;
    case ETM_TECR1:  v = s->tecr1;  break;
    case ETM_FFRR:   v = s->ffrr;   break;
    case ETM_TEEVR2: v = s->teevr2; break;
    case ETM_TEEVR3: v = s->teevr3; break;
    case ETM_IDR:    v = 0;         break;  /* impl-defined */
    case ETM_CCER:   v = 0;         break;  /* impl-defined */
    case ETM_DEVTYPE: v = 0x13;     break;  /* trace source */
    case ETM_CIDR0:  v = 0x0d;      break;
    case ETM_CIDR1:  v = 0x90;      break;
    case ETM_CIDR2:  v = 0x05;      break;
    case ETM_CIDR3:  v = 0xb1;      break;
    default:         v = 0;         break;  /* reserved */
    }

    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access_word(s->name, false, offset, v);
    }
    return mxs_etm_extract(v, offset, size);
}

static void mxs_etm_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    MXSETMState *s = MXS_ETM(opaque);

    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access_word(s->name, true, offset, (uint32_t)value);
    }

    switch (offset & ~3u) {
    case ETM_CR:     s->cr     = mxs_etm_apply(s->cr, offset, value, size); break;
    case ETM_SCR:    s->scr    = mxs_etm_apply(s->scr, offset, value, size); break;
    case ETM_TEEVR:  s->teevr  = mxs_etm_apply(s->teevr, offset, value, size); break;
    case ETM_TECR1:  s->tecr1  = mxs_etm_apply(s->tecr1, offset, value, size); break;
    case ETM_FFRR:   s->ffrr   = mxs_etm_apply(s->ffrr, offset, value, size); break;
    case ETM_TEEVR2: s->teevr2 = mxs_etm_apply(s->teevr2, offset, value, size); break;
    case ETM_TEEVR3: s->teevr3 = mxs_etm_apply(s->teevr3, offset, value, size); break;
    default:         break;  /* write-only trigger + reserved: dropped */
    }
}

static const MemoryRegionOps mxs_etm_ops = {
    .read = mxs_etm_read,
    .write = mxs_etm_write,
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

static void mxs_etm_reset(DeviceState *dev)
{
    MXSETMState *s = MXS_ETM(dev);

    s->cr = s->scr = s->teevr = s->tecr1 = s->ffrr = 0;
    s->teevr2 = s->teevr3 = 0;
}

static void mxs_etm_realize(DeviceState *dev, Error **errp)
{
    MXSETMState *s = MXS_ETM(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->trace = mxs_trace_enabled(s->name ? s->name : TYPE_MXS_ETM);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mxs_etm_ops, s,
                          TYPE_MXS_ETM, s->size ? s->size : 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_mxs_etm = {
    .name = TYPE_MXS_ETM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr, MXSETMState),
        VMSTATE_UINT32(scr, MXSETMState),
        VMSTATE_UINT32(teevr, MXSETMState),
        VMSTATE_UINT32(tecr1, MXSETMState),
        VMSTATE_UINT32(ffrr, MXSETMState),
        VMSTATE_UINT32(teevr2, MXSETMState),
        VMSTATE_UINT32(teevr3, MXSETMState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property mxs_etm_properties[] = {
    DEFINE_PROP_STRING("name", MXSETMState, name),
    DEFINE_PROP_UINT64("size", MXSETMState, size, 0x2000),
};

static void mxs_etm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mxs_etm_realize;
    device_class_set_legacy_reset(dc, mxs_etm_reset);
    dc->vmsd = &vmstate_mxs_etm;
    device_class_set_props(dc, mxs_etm_properties);
}

static const TypeInfo mxs_etm_type = {
    .name = TYPE_MXS_ETM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MXSETMState),
    .class_init = mxs_etm_class_init,
};

static void mxs_etm_register_types(void)
{
    type_register_static(&mxs_etm_type);
}
type_init(mxs_etm_register_types)
