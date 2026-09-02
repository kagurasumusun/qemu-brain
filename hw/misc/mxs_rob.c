/*
 * Freescale i.MX28 "register-only" peripherals.
 *
 * These i.MX28 blocks are present on the die and have real register
 * maps in the reference manual, but the WinCE image that qemu-brain runs
 * has no driver for them and never enables their datapaths.  The project
 * rule is to model real hardware with its real registers rather than as
 * access-swallowing placeholders, so each block below carries the full
 * reset state of its register file and honours its access rules:
 *
 *   HSADC   - High-Speed ADC (RM ch 37, 0x80002000)        bank style
 *   SPDIF   - SPDIF transmitter (RM ch 36, 0x80054000)     bank style
 *   DRAM    - External Memory Interface (RM ch 14)         word style
 *   FlexCAN - CAN0/CAN1 (RM ch 25)                         word style
 *   ENET    - Ethernet MAC0/MAC1 (RM ch 26, FEC)           word style
 *   ENET-SWI- 3-port Ethernet switch (RM ch 29)            word style
 *
 * Two register addressing styles exist on this SoC:
 *
 *   bank  - the MXS convention: registers are 0x10 bytes apart with
 *           SET/CLR/TOG aliases at +0x4/+0x8/+0xc, and a soft reset
 *           protocol (SFTRST/CLKGATE) in the first register.
 *   word  - plain 4-byte registers (third-party IP: FlexCAN, FEC and
 *           the DRAM controller have no SET/CLR/TOG aliases).
 *
 * What is NOT modelled here (documented per block rather than faked):
 *   - HSADC: no actual ADC conversion; FIFO stays empty, the status and
 *     interrupt bits never assert and the interrupt line stays low.
 *   - SPDIF: no frame generation or FIFO; HW_SPDIF_DATA accepts writes
 *     but nothing is serialised out, the FIFO error/IRQ bits never set.
 *   - DRAM: no DDR command sequencing or AXI traffic accounting; the
 *     debug/status registers read their reset values.
 *   - FlexCAN: no CAN frame tx/rx; the message-buffer and RXIMR areas
 *     (0x080-0x87f, 0x880-0x97f) are modelled as RAM.  On real silicon
 *     RXIMR is RAM too ("not affected by reset"); a QEMU machine reset
 *     zeroes it, which matches power-on undefined content well enough
 *     for a block the guest never touches.
 *   - ENET/switch: no packet DMA, MII management or frame forwarding;
 *     the RMON/IEEE counters read their reset values.
 *
 * Because the WinCE guest performs no accesses to any of these blocks
 * (verified by MXS_TRACE), the register file semantics above are exactly
 * what the guest can observe, and the reset values are the only values
 * it could ever read back.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/mxs_bank.h"
#include "hw/misc/mxs_rob.h"
#include "migration/vmstate.h"
#include "qom/object.h"

typedef struct MXSRobState MXSRobState;

typedef struct MXSRobDesc {
    const char *type;
    const char *desc;
    bool word;      /* plain 4-byte registers instead of MXS 0x10 banks */
    bool sftrst;    /* first register carries SFTRST/CLKGATE (bank style) */
    uint32_t size;  /* MMIO region size */
    uint32_t nwords;
    const uint32_t *reset;
    const uint32_t *ro;
} MXSRobDesc;

enum {
    MXS_ROB_HSADC = 0,
    MXS_ROB_SPDIF,
    MXS_ROB_DRAM,
    MXS_ROB_CAN,
    MXS_ROB_ENET,
    MXS_ROB_SWI,
    MXS_ROB_AUDIOOUT,
    MXS_ROB_NUM,
};


/* Auto-derived from MCIMX28RM Rev.2 register maps (see gen_rob3.py). */
/* Freescale i.MX28 High-Speed ADC */
static const uint32_t hsadc_reset[12] = {
    [0] = 0xc0000040,
    [1] = 0xf0000020,
    [2] = 0x0000238e,
    [11] = 0x00010000,
};
static const uint32_t hsadc_ro[12] = {
    [5] = 0xffffffff,
    [6] = 0xffffffff,
    [7] = 0xffffffff,
    [8] = 0xffffffff,
    [11] = 0xffffffff,
};

/* Freescale i.MX28 SPDIF transmitter */
static const uint32_t spdif_reset[7] = {
    [0] = 0xc0000020,
    [1] = 0x80000000,
    [2] = 0x00020000,
    [3] = 0x10000000,
    [4] = 0x00000001,
    [6] = 0x01010000,
};
static const uint32_t spdif_ro[7] = {
    [1] = 0xffffffff,
    [4] = 0xffffffff,
    [6] = 0xffffffff,
};

/* Freescale i.MX28 External Memory Interface */
static const uint32_t dram_reset[190] = {
    [8] = 0x00000010,
    [30] = 0x00040f0c,
    [66] = 0x00040000,
    [86] = 0x00002040,
};
static const uint32_t dram_ro[190] = {
    [8] = 0xffffffff,
    [9] = 0xffffffff,
    [10] = 0xffffffff,
    [11] = 0xffffffff,
    [12] = 0xffffffff,
    [13] = 0xffffffff,
    [14] = 0xffffffff,
    [15] = 0xffffffff,
    [30] = 0xffffffff,
    [59] = 0xffffffff,
    [60] = 0xffffffff,
    [61] = 0xffffffff,
    [62] = 0xffffffff,
    [63] = 0xffffffff,
    [64] = 0xffffffff,
    [65] = 0xffffffff,
    [86] = 0xffffffff,
    [95] = 0xffffffff,
    [96] = 0xffffffff,
    [97] = 0xffffffff,
    [98] = 0xffffffff,
    [99] = 0xffffffff,
    [100] = 0xffffffff,
    [101] = 0xffffffff,
    [102] = 0xffffffff,
    [103] = 0xffffffff,
    [104] = 0xffffffff,
    [105] = 0xffffffff,
    [106] = 0xffffffff,
    [107] = 0xffffffff,
    [108] = 0xffffffff,
    [109] = 0xffffffff,
    [110] = 0xffffffff,
    [111] = 0xffffffff,
    [112] = 0xffffffff,
    [113] = 0xffffffff,
    [114] = 0xffffffff,
    [115] = 0xffffffff,
    [116] = 0xffffffff,
    [117] = 0xffffffff,
    [118] = 0xffffffff,
    [119] = 0xffffffff,
    [120] = 0xffffffff,
    [121] = 0xffffffff,
    [122] = 0xffffffff,
    [123] = 0xffffffff,
    [124] = 0xffffffff,
    [125] = 0xffffffff,
    [126] = 0xffffffff,
    [127] = 0xffffffff,
    [128] = 0xffffffff,
    [129] = 0xffffffff,
    [130] = 0xffffffff,
    [131] = 0xffffffff,
    [132] = 0xffffffff,
    [133] = 0xffffffff,
    [134] = 0xffffffff,
    [135] = 0xffffffff,
    [136] = 0xffffffff,
    [137] = 0xffffffff,
    [138] = 0xffffffff,
    [139] = 0xffffffff,
    [140] = 0xffffffff,
    [141] = 0xffffffff,
    [142] = 0xffffffff,
    [143] = 0xffffffff,
    [144] = 0xffffffff,
    [145] = 0xffffffff,
    [146] = 0xffffffff,
    [147] = 0xffffffff,
    [148] = 0xffffffff,
    [149] = 0xffffffff,
    [150] = 0xffffffff,
    [151] = 0xffffffff,
    [152] = 0xffffffff,
    [153] = 0xffffffff,
    [154] = 0xffffffff,
    [155] = 0xffffffff,
    [156] = 0xffffffff,
    [157] = 0xffffffff,
    [158] = 0xffffffff,
    [159] = 0xffffffff,
    [160] = 0xffffffff,
    [161] = 0xffffffff,
};

/* Freescale i.MX28 FlexCAN */
static const uint32_t can_reset[608] = {
    [0] = 0x5890000f,
    [4] = 0xffffffff,
    [5] = 0xffffffff,
    [6] = 0xffffffff,
    [13] = 0x0000007f,
};
static const uint32_t can_ro[608] = { 0 };

/* Freescale i.MX28 Ethernet MAC (FEC) */
static const uint32_t enet_reset[418] = {
    [9] = 0xf0000000,
    [25] = 0xc0000000,
    [33] = 0x05ee0001,
    [58] = 0x00008808,
    [59] = 0x00010000,
    [83] = 0x00000600,
    [84] = 0x00000500,
    [102] = 0x00000004,
    [103] = 0x00000004,
    [105] = 0x00000004,
    [106] = 0x00000008,
    [107] = 0x0000000c,
    [108] = 0x000007ff,
    [259] = 0x3b9aca00,
};
static const uint32_t enet_ro[418] = {
    [128] = 0xffffffff,
    [129] = 0xffffffff,
    [130] = 0xffffffff,
    [131] = 0xffffffff,
    [132] = 0xffffffff,
    [133] = 0xffffffff,
    [134] = 0xffffffff,
    [135] = 0xffffffff,
    [136] = 0xffffffff,
    [137] = 0xffffffff,
    [138] = 0xffffffff,
    [139] = 0xffffffff,
    [140] = 0xffffffff,
    [141] = 0xffffffff,
    [142] = 0xffffffff,
    [143] = 0xffffffff,
    [144] = 0xffffffff,
    [145] = 0xffffffff,
    [146] = 0xffffffff,
    [147] = 0xffffffff,
    [148] = 0xffffffff,
    [149] = 0xffffffff,
    [150] = 0xffffffff,
    [151] = 0xffffffff,
    [152] = 0xffffffff,
    [153] = 0xffffffff,
    [154] = 0xffffffff,
    [155] = 0xffffffff,
    [156] = 0xffffffff,
    [157] = 0xffffffff,
    [161] = 0xffffffff,
    [162] = 0xffffffff,
    [163] = 0xffffffff,
    [164] = 0xffffffff,
    [165] = 0xffffffff,
    [166] = 0xffffffff,
    [167] = 0xffffffff,
    [168] = 0xffffffff,
    [170] = 0xffffffff,
    [171] = 0xffffffff,
    [172] = 0xffffffff,
    [173] = 0xffffffff,
    [174] = 0xffffffff,
    [175] = 0xffffffff,
    [176] = 0xffffffff,
    [177] = 0xffffffff,
    [178] = 0xffffffff,
    [179] = 0xffffffff,
    [180] = 0xffffffff,
    [181] = 0xffffffff,
    [182] = 0xffffffff,
    [183] = 0xffffffff,
    [184] = 0xffffffff,
    [262] = 0xffffffff,
    [400] = 0xffffffff,
    [401] = 0xffffffff,
    [402] = 0xffffffff,
    [403] = 0xffffffff,
};

/* Freescale i.MX28 Ethernet switch */
static const uint32_t swi_reset[8192] = {
    [13] = 0x00008100,
    [32] = 0x0060004a,
    [33] = 0x00000009,
    [36] = 0x00000007,
    [39] = 0x00000009,
    [256] = 0x00000020,
};
static const uint32_t swi_ro[8192] = {
    [35] = 0xffffffff,
    [36] = 0xffffffff,
    [192] = 0xffffffff,
    [193] = 0xffffffff,
    [194] = 0xffffffff,
    [195] = 0xffffffff,
    [196] = 0xffffffff,
    [197] = 0xffffffff,
    [198] = 0xffffffff,
    [199] = 0xffffffff,
    [200] = 0xffffffff,
    [201] = 0xffffffff,
    [202] = 0xffffffff,
    [203] = 0xffffffff,
    [204] = 0xffffffff,
    [205] = 0xffffffff,
    [206] = 0xffffffff,
    [207] = 0xffffffff,
    [320] = 0xffffffff,
    [321] = 0xffffffff,
    [322] = 0xffffffff,
};

/* Freescale i.MX28 audio output filter (DAC path) */
static const uint32_t audioout_reset[1] = { 0 };
static const uint32_t audioout_ro[1] = { 0 };

static const MXSRobDesc mxs_rob_descs[MXS_ROB_NUM] = {
    [MXS_ROB_HSADC] = { .type = "mxs-hsadc", .desc = "Freescale i.MX28 High-Speed ADC",
        .word = false, .sftrst = true, .size = 0x2000, .nwords = 12,
        .reset = hsadc_reset, .ro = hsadc_ro },
    [MXS_ROB_SPDIF] = { .type = "mxs-spdif", .desc = "Freescale i.MX28 SPDIF transmitter",
        .word = false, .sftrst = true, .size = 0x2000, .nwords = 7,
        .reset = spdif_reset, .ro = spdif_ro },
    [MXS_ROB_DRAM] = { .type = "mxs-dram", .desc = "Freescale i.MX28 External Memory Interface",
        .word = true, .sftrst = false, .size = 0x10000, .nwords = 190,
        .reset = dram_reset, .ro = dram_ro },
    [MXS_ROB_CAN] = { .type = "mxs-flexcan", .desc = "Freescale i.MX28 FlexCAN controller",
        .word = true, .sftrst = false, .size = 0x2000, .nwords = 608,
        .reset = can_reset, .ro = can_ro },
    [MXS_ROB_ENET] = { .type = "mxs-enet", .desc = "Freescale i.MX28 Ethernet MAC (FEC)",
        .word = true, .sftrst = false, .size = 0x4000, .nwords = 418,
        .reset = enet_reset, .ro = enet_ro },
    [MXS_ROB_SWI] = { .type = "mxs-enet-swi", .desc = "Freescale i.MX28 Ethernet switch",
        .word = true, .sftrst = false, .size = 0x8000, .nwords = 8192,
        .reset = swi_reset, .ro = swi_ro },
    [MXS_ROB_AUDIOOUT] = { .type = "mxs-audioout", .desc = "Freescale i.MX28 audio output filter (DAC path)",
        .word = true, .sftrst = false, .size = 0x4000, .nwords = 1,
        .reset = audioout_reset, .ro = audioout_ro },
};

typedef struct MXSRobClass {
    SysBusDeviceClass parent_class;
    const MXSRobDesc *desc;
} MXSRobClass;

#define TYPE_MXS_ROB "mxs-rob"

OBJECT_DECLARE_TYPE(MXSRobState, MXSRobClass, MXS_ROB)

struct MXSRobState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;      /* main block interrupt (never raised: no datapath) */
    qemu_irq irq1588;  /* ENET IEEE-1588 timer interrupt (never raised) */
    const MXSRobDesc *d;
    uint32_t *regs;
    uint32_t nwords;
    uint64_t size;      /* optional size property override */
    char *name;
    bool trace;
};

static void mxs_rob_reset_state(MXSRobState *s)
{
    const MXSRobDesc *d = s->d;

    if (d->reset) {
        memcpy(s->regs, d->reset, d->nwords * sizeof(uint32_t));
    } else {
        memset(s->regs, 0, d->nwords * sizeof(uint32_t));
    }
    qemu_irq_lower(s->irq);
    qemu_irq_lower(s->irq1588);
}

static uint64_t mxs_rob_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSRobState *s = MXS_ROB(opaque);
    const MXSRobDesc *d = s->d;
    unsigned idx = d->word ? offset >> 2 : MXS_BANK_INDEX(offset);
    uint32_t v;

    if (idx >= d->nwords) {
        return 0;
    }
    v = s->regs[idx];
    if (unlikely(s->trace)) {
        if (d->word) {
            mxs_trace_access_word(s->name ? s->name : d->type, false,
                                  offset, v);
        } else {
            mxs_trace_access(s->name ? s->name : d->type, false, offset, v);
        }
    }
    if (d->word) {
        if (size < 4) {
            unsigned byte = offset & 3;

            v = (v >> (byte * 8)) & ((1u << (size * 8)) - 1);
        }
        return v;
    }
    return mxs_bank_extract(offset, size, v);
}

static void mxs_rob_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    MXSRobState *s = MXS_ROB(opaque);
    const MXSRobDesc *d = s->d;
    unsigned idx = d->word ? offset >> 2 : MXS_BANK_INDEX(offset);
    uint32_t old, v;

    if (idx >= d->nwords) {
        return;
    }
    old = s->regs[idx];
    if (d->word) {
        unsigned byte = offset & 3;
        uint32_t mask = (size >= 4) ? 0xffffffffu
                                    : ((1u << (size * 8)) - 1) << (byte * 8);

        v = (old & ~mask) | (((uint32_t)value << (byte * 8)) & mask);
    } else {
        v = mxs_bank_apply(old, offset, value, size);
        if (d->sftrst && (offset & ~0xfu) == 0) {
            v = mxs_bank_sftrst(old, v);
        }
    }
    if (d->ro) {
        v = (v & ~d->ro[idx]) | (old & d->ro[idx]);
    }
    s->regs[idx] = v;
    if (unlikely(s->trace)) {
        if (d->word) {
            mxs_trace_access_word(s->name ? s->name : d->type, true,
                                  offset, (uint32_t)value);
        } else {
            mxs_trace_access(s->name ? s->name : d->type, true, offset,
                             (uint32_t)value);
        }
    }

    /* SFTRST (bank style): force the block back to its reset state. */
    if (d->sftrst && !d->word && idx == 0 && (v & MXS_SFTRST_BIT)) {
        mxs_rob_reset_state(s);
    }
}

static const MemoryRegionOps mxs_rob_ops = {
    .read = mxs_rob_read,
    .write = mxs_rob_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void mxs_rob_realize(DeviceState *dev, Error **errp)
{
    MXSRobState *s = MXS_ROB(dev);
    MXSRobClass *k = MXS_ROB_GET_CLASS(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->d = k->desc;
    s->nwords = s->d->nwords;
    s->regs = g_malloc0(s->nwords * sizeof(uint32_t));
    s->trace = mxs_trace_enabled(s->name ? s->name : s->d->type);
    mxs_rob_reset_state(s);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mxs_rob_ops, s,
                          s->d->type, s->size ? s->size : s->d->size);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->irq1588);
}

static void mxs_rob_reset(DeviceState *dev)
{
    MXSRobState *s = MXS_ROB(dev);

    mxs_rob_reset_state(s);
}

static const VMStateDescription vmstate_mxs_rob = {
    .name = TYPE_MXS_ROB,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_VARRAY_UINT32(regs, MXSRobState, nwords, 0,
                              vmstate_info_uint32, uint32_t),
        VMSTATE_END_OF_LIST()
    },
};

static const Property mxs_rob_properties[] = {
    DEFINE_PROP_STRING("name", MXSRobState, name),
    DEFINE_PROP_UINT64("size", MXSRobState, size, 0),
};

static void mxs_rob_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    MXSRobClass *k = MXS_ROB_CLASS(oc);

    k->desc = data;
    dc->realize = mxs_rob_realize;
    device_class_set_legacy_reset(dc, mxs_rob_reset);
    dc->vmsd = &vmstate_mxs_rob;
    device_class_set_props(dc, mxs_rob_properties);
}

static const TypeInfo mxs_rob_type = {
    .name = TYPE_MXS_ROB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .abstract = true,
    .instance_size = sizeof(MXSRobState),
    .class_size = sizeof(MXSRobClass),
};

static const TypeInfo mxs_rob_concrete_types[] = {
    { .name = "mxs-hsadc", .parent = TYPE_MXS_ROB, .instance_size = sizeof(MXSRobState),
      .class_init = mxs_rob_class_init, .class_data = &mxs_rob_descs[MXS_ROB_HSADC] },
    { .name = "mxs-spdif", .parent = TYPE_MXS_ROB, .instance_size = sizeof(MXSRobState),
      .class_init = mxs_rob_class_init, .class_data = &mxs_rob_descs[MXS_ROB_SPDIF] },
    { .name = "mxs-dram", .parent = TYPE_MXS_ROB, .instance_size = sizeof(MXSRobState),
      .class_init = mxs_rob_class_init, .class_data = &mxs_rob_descs[MXS_ROB_DRAM] },
    { .name = "mxs-flexcan", .parent = TYPE_MXS_ROB, .instance_size = sizeof(MXSRobState),
      .class_init = mxs_rob_class_init, .class_data = &mxs_rob_descs[MXS_ROB_CAN] },
    { .name = "mxs-enet", .parent = TYPE_MXS_ROB, .instance_size = sizeof(MXSRobState),
      .class_init = mxs_rob_class_init, .class_data = &mxs_rob_descs[MXS_ROB_ENET] },
    { .name = "mxs-enet-swi", .parent = TYPE_MXS_ROB, .instance_size = sizeof(MXSRobState),
      .class_init = mxs_rob_class_init, .class_data = &mxs_rob_descs[MXS_ROB_SWI] },
    { .name = "mxs-audioout", .parent = TYPE_MXS_ROB, .instance_size = sizeof(MXSRobState),
      .class_init = mxs_rob_class_init, .class_data = &mxs_rob_descs[MXS_ROB_AUDIOOUT] }
};

DEFINE_TYPES(mxs_rob_concrete_types)

static void mxs_rob_register_types(void)
{
    type_register_static(&mxs_rob_type);
}
type_init(mxs_rob_register_types)
