/*
 * Freescale i.MX28 BCH (Bose-Chaudhuri-Hocquenghem) ECC engine.
 *
 * Register layout from the Linux BSP header (drivers/mtd/nand/gpmi-nand/
 * bch-regs.h).  The engine performs real ECC generation using the kernel
 * lib/bch algorithm (ported as mxs_bchlib.c): on an encode request the
 * data at HW_BCH_DATAPTR/ENCODEPTR is fed through encode_bch() and the
 * parity is written back to the guest buffer at HW_BCH_METAPTR, then
 * STATUS0/COMPLETE_IRQ are raised per the CTRL enable bits.
 *
 * ECC strength follows the i.MX28 BCH LAYOUT encoding
 * (0 -> 0, 1 -> 2, 2 -> 4, 3 -> 8, 4 -> 12, else 16 bits).
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/misc/mxs_bchlib.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/dma.h"

#define BCH_CTRL            0x00
#define BCH_STATUS0         0x10
#define BCH_MODE            0x20
#define BCH_ENCODEPTR       0x30
#define BCH_DATAPTR         0x40
#define BCH_METAPTR         0x50
#define BCH_LAYOUTSELECT    0x70
#define BCH_FLASH0LAYOUT0   0x80
#define BCH_FLASH0LAYOUT1   0x90
#define BCH_FLASH1LAYOUT0   0xa0
#define BCH_FLASH1LAYOUT1   0xb0
#define BCH_VERSION         0xc0

#define CTRL_COMPLETE_IRQ_EN   (1u << 8)
#define CTRL_COMPLETE_IRQ      (1u << 0)

#define MXS_MAX_REGS 64

typedef struct MXSBchState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[MXS_MAX_REGS];
    char *name;
    uint64_t size;
    bool trace;

    struct bch_control *bch[6];   /* one per strength slot */
} MXSBchState;

#define TYPE_MXS_BCH "mxs-bch"
OBJECT_DECLARE_SIMPLE_TYPE(MXSBchState, MXS_BCH)

static const int bch_strength[6] = { 0, 2, 4, 8, 12, 16 };

static struct bch_control *bch_for_strength(MXSBchState *s, unsigned enc)
{
    unsigned idx = enc < 6 ? enc : 5;

    if (!s->bch[idx]) {
        s->bch[idx] = init_bch(13, bch_strength[idx] ? bch_strength[idx] : 1, 0);
    }
    return s->bch[idx];
}

static void bch_update_irq(MXSBchState *s)
{
    uint32_t c = s->regs[BCH_CTRL >> 4];

    qemu_set_irq(s->irq,
                 (c & CTRL_COMPLETE_IRQ) && (c & CTRL_COMPLETE_IRQ_EN));
}

/* encode one layout-described page from guest memory */
static void bch_do_encode(MXSBchState *s, uint32_t layout0, uint32_t layout1,
                          uint32_t data_ptr, uint32_t meta_ptr)
{
    unsigned nb = (layout0 >> 24) & 0xff;
    unsigned meta = (layout0 >> 16) & 0xff;
    unsigned ecc0 = (layout0 >> 12) & 0xf;
    unsigned eccn = (layout1 >> 12) & 0xf;
    unsigned datan = layout1 & 0xfff;
    struct bch_control *bch;
    uint8_t buf[2048];
    uint8_t ecc[32];
    unsigned i;

    /* metadata is protected with the ECC0 strength */
    if (meta && ecc0) {
        bch = bch_for_strength(s, ecc0);
        if (bch && meta <= sizeof(buf)) {
            dma_memory_read(&address_space_memory, data_ptr, buf, meta,
                            MEMTXATTRS_UNSPECIFIED);
            memset(ecc, 0, sizeof(ecc));
            encode_bch(bch, buf, meta, ecc);
            dma_memory_write(&address_space_memory, meta_ptr, ecc,
                             bch->ecc_bytes, MEMTXATTRS_UNSPECIFIED);
        }
    }
    /* data chunks with ECCN strength */
    for (i = 0; i < nb; i++) {
        unsigned len = datan ? datan * 4 : 0;   /* DATAx_SIZE in 32-bit words? RM: bytes/4 */

        if (!len || !eccn || len > sizeof(buf)) {
            continue;
        }
        bch = bch_for_strength(s, eccn);
        if (!bch) {
            continue;
        }
        dma_memory_read(&address_space_memory,
                        data_ptr + (meta ? meta : 0) + i * len,
                        buf, len, MEMTXATTRS_UNSPECIFIED);
        memset(ecc, 0, sizeof(ecc));
        encode_bch(bch, buf, len, ecc);
        dma_memory_write(&address_space_memory,
                         meta_ptr + bch->ecc_bytes * (i + 1), ecc,
                         bch->ecc_bytes, MEMTXATTRS_UNSPECIFIED);
    }
    s->regs[BCH_CTRL >> 4] |= CTRL_COMPLETE_IRQ;
    s->regs[BCH_STATUS0 >> 4] = 0;   /* no uncorrectable errors: clean encode */
    bch_update_irq(s);
}

static uint64_t mxs_bch_read(void *opaque, hwaddr off, unsigned size)
{
    MXSBchState *s = MXS_BCH(opaque);
    unsigned idx = MXS_BANK_INDEX(off);
    uint32_t v = 0;

    if (idx < MXS_MAX_REGS) {
        v = s->regs[idx];
    }
    if (idx == (BCH_VERSION >> 4)) {
        v = 0x00010000;
    }
    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, false, off, v);
    }
    return mxs_bank_extract(off, size, v);
}

static void mxs_bch_write(void *opaque, hwaddr off, uint64_t value,
                          unsigned size)
{
    MXSBchState *s = MXS_BCH(opaque);
    unsigned idx = MXS_BANK_INDEX(off);

    if (idx >= MXS_MAX_REGS) {
        return;
    }
    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, true, off, (uint32_t)value);
    }
    if (idx == (BCH_CTRL >> 4)) {
        uint32_t old = s->regs[idx];
        uint32_t val = mxs_bank_apply(old, off, value, size);

        val = mxs_bank_sftrst(old, val);
        /* COMPLETE_IRQ is write-1-to-clear */
        if (((off >> 2) & 3) == 1) {
            val &= ~CTRL_COMPLETE_IRQ;
        }
        s->regs[idx] = val;
        bch_update_irq(s);
        return;
    }
    if (idx == (BCH_ENCODEPTR >> 4)) {
        uint32_t val = mxs_bank_apply(s->regs[idx], off, value, size);
        uint32_t sel = s->regs[BCH_LAYOUTSELECT >> 4] & 1;
        uint32_t l0 = s->regs[(sel ? BCH_FLASH1LAYOUT0 : BCH_FLASH0LAYOUT0) >> 4];
        uint32_t l1 = s->regs[(sel ? BCH_FLASH1LAYOUT1 : BCH_FLASH0LAYOUT1) >> 4];

        s->regs[idx] = val;
        bch_do_encode(s, l0, l1, val, s->regs[BCH_METAPTR >> 4]);
        return;
    }
    s->regs[idx] = mxs_bank_apply(s->regs[idx], off, value, size);
}

MXS_TRACE_WRAP(mxs_bch, "bch")

static const MemoryRegionOps mxs_bch_ops = {
    .read = mxs_bch_read_tr, .write = mxs_bch_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_bch_reset(DeviceState *d)
{
    MXSBchState *s = MXS_BCH(d);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_set_irq(s->irq, 0);
}

static void mxs_bch_realize(DeviceState *d, Error **e)
{
    MXSBchState *s = MXS_BCH(d);
    SysBusDevice *sbd = SYS_BUS_DEVICE(d);
    g_autofree char *n = g_strdup_printf("mxs-%s", s->name ? s->name : "bch");

    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_bch_ops, s, n,
                          s->size ? s->size : 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->trace = mxs_trace_enabled(s->name ? s->name : "bch");
}

static const Property mxs_bch_props[] = {
    DEFINE_PROP_STRING("name", MXSBchState, name),
    DEFINE_PROP_UINT64("size", MXSBchState, size, 0x2000),
};

static const VMStateDescription vmstate_mxs_bch = {
    .name = "mxs-bch", .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSBchState, MXS_MAX_REGS),
        VMSTATE_END_OF_LIST()
    },
};

static void mxs_bch_class_init(ObjectClass *k, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(k);

    dc->realize = mxs_bch_realize;
    device_class_set_legacy_reset(dc, mxs_bch_reset);
    dc->vmsd = &vmstate_mxs_bch;
    device_class_set_props(dc, mxs_bch_props);
}

static const TypeInfo mxs_bch_info[] = {
{
    .name = TYPE_MXS_BCH, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MXSBchState),
    .class_init = mxs_bch_class_init,
}
};
DEFINE_TYPES(mxs_bch_info)
