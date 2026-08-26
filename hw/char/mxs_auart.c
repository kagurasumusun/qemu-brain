/*
 * Freescale i.MX28 (MXS) Application UART
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define AUART_CTRL0     0x0
#define AUART_CTRL1     0x1
#define AUART_CTRL2     0x2
#define AUART_LINECTRL  0x3
#define AUART_LINECTRL2 0x4
#define AUART_INTR      0x5
#define AUART_DATA      0x6
#define AUART_STAT      0x7
#define AUART_DEBUG     0x8
#define AUART_VERSION   0x9
#define AUART_AUTOBAUD  0xa
#define AUART_NREGS     0x10

#define CTRL0_SFTRST    (1u << 31)
#define CTRL0_CLKGATE   (1u << 30)

#define CTRL2_RXE       (1u << 9)
#define CTRL2_TXE       (1u << 8)
#define CTRL2_UARTEN    (1u << 0)

#define INTR_RTIEN      (1u << 22)
#define INTR_TXIEN      (1u << 21)
#define INTR_RXIEN      (1u << 20)
#define INTR_RTIS       (1u << 6)
#define INTR_TXIS       (1u << 5)
#define INTR_RXIS       (1u << 4)

#define STAT_BUSY       (1u << 29)
#define STAT_CTS        (1u << 28)
#define STAT_TXFE       (1u << 27)
#define STAT_RXFF       (1u << 26)
#define STAT_TXFF       (1u << 25)
#define STAT_RXFE       (1u << 24)

#define AUART_FIFO_DEPTH 16

typedef struct MXSAuartState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    CharFrontend chr;

    uint32_t regs[AUART_NREGS];

    uint8_t rx_fifo[AUART_FIFO_DEPTH];
    int rx_len;
} MXSAuartState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSAuartState, MXS_AUART)

static void mxs_auart_update_irq(MXSAuartState *s)
{
    uint32_t intr = s->regs[AUART_INTR];
    bool level;

    if (s->rx_len) {
        intr |= INTR_RXIS | INTR_RTIS;
    } else {
        intr &= ~(INTR_RXIS | INTR_RTIS);
    }
    /* TX always drains immediately */
    intr |= INTR_TXIS;
    s->regs[AUART_INTR] = intr;

    level = !!(((intr >> 16) & (intr & 0xffff)) & 0x7f) ||
            (((intr & INTR_RXIEN) && (intr & INTR_RXIS)) ||
             ((intr & INTR_TXIEN) && (intr & INTR_TXIS)) ||
             ((intr & INTR_RTIEN) && (intr & INTR_RTIS)));
    qemu_set_irq(s->irq, level);
}

static int mxs_auart_can_receive(void *opaque)
{
    MXSAuartState *s = opaque;

    if (!(s->regs[AUART_CTRL2] & CTRL2_RXE)) {
        return 0;
    }
    return AUART_FIFO_DEPTH - s->rx_len;
}

static void mxs_auart_receive(void *opaque, const uint8_t *buf, int size)
{
    MXSAuartState *s = opaque;
    int i;

    for (i = 0; i < size && s->rx_len < AUART_FIFO_DEPTH; i++) {
        s->rx_fifo[s->rx_len++] = buf[i];
    }
    mxs_auart_update_irq(s);
}

static uint64_t mxs_auart_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSAuartState *s = MXS_AUART(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= AUART_NREGS) {
        return 0;
    }

    switch (idx) {
    case AUART_DATA: {
        int i;

        val = 0;
        for (i = 0; i < (int)(size >= 4 ? 4 : size); i++) {
            if (s->rx_len) {
                val |= (uint32_t)s->rx_fifo[0] << (i * 8);
                memmove(s->rx_fifo, s->rx_fifo + 1, --s->rx_len);
            }
        }
        mxs_auart_update_irq(s);
        return val;
    }
    case AUART_STAT:
        val = STAT_TXFE | STAT_CTS;
        if (!s->rx_len) {
            val |= STAT_RXFE;
        } else {
            val |= s->rx_len;
            if (s->rx_len >= AUART_FIFO_DEPTH) {
                val |= STAT_RXFF;
            }
        }
        break;
    case AUART_VERSION:
        val = 0x03000000;
        break;
    default:
        val = s->regs[idx];
        break;
    }

    return mxs_bank_extract(offset, size, val);
}

static void mxs_auart_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    MXSAuartState *s = MXS_AUART(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= AUART_NREGS) {
        return;
    }

    if (idx == AUART_DATA) {
        uint8_t ch[4];
        int n = size >= 4 ? 1 : size;   /* 32 bit write pushes one char */

        ch[0] = value & 0xff;
        if (size < 4) {
            n = size;
            ch[0] = value & 0xff;
            ch[1] = (value >> 8) & 0xff;
        }
        qemu_chr_fe_write_all(&s->chr, ch, n);
        mxs_auart_update_irq(s);
        return;
    }

    val = mxs_bank_apply(s->regs[idx], offset, value, size);

    switch (idx) {
    case AUART_CTRL0:
        val = mxs_bank_sftrst(s->regs[idx], val);
        s->regs[idx] = val;
        break;
    case AUART_INTR:
        s->regs[idx] = val;
        mxs_auart_update_irq(s);
        break;
    case AUART_STAT:
    case AUART_VERSION:
        break;
    default:
        s->regs[idx] = val;
        break;
    }
}

MXS_TRACE_WRAP(mxs_auart, "auart")

static const MemoryRegionOps mxs_auart_ops = {
    .read = mxs_auart_read_tr,
    .write = mxs_auart_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_auart_reset(DeviceState *dev)
{
    MXSAuartState *s = MXS_AUART(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[AUART_CTRL0] = CTRL0_SFTRST | CTRL0_CLKGATE;
    s->rx_len = 0;
}

static void mxs_auart_realize(DeviceState *dev, Error **errp)
{
    MXSAuartState *s = MXS_AUART(dev);

    qemu_chr_fe_set_handlers(&s->chr, mxs_auart_can_receive,
                             mxs_auart_receive, NULL, NULL, s, NULL, true);
}

static void mxs_auart_init(Object *obj)
{
    mxs_auart_trace = mxs_trace_enabled("auart");

    MXSAuartState *s = MXS_AUART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &mxs_auart_ops, s, "mxs-auart",
                          0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property mxs_auart_properties[] = {
    DEFINE_PROP_CHR("chardev", MXSAuartState, chr),
};

static const VMStateDescription vmstate_mxs_auart = {
    .name = "mxs-auart",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSAuartState, AUART_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_auart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mxs_auart_realize;
    device_class_set_legacy_reset(dc, mxs_auart_reset);
    device_class_set_props(dc, mxs_auart_properties);
    dc->vmsd = &vmstate_mxs_auart;
}

static const TypeInfo mxs_auart_types[] = {
    {
        .name           = TYPE_MXS_AUART,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSAuartState),
        .instance_init  = mxs_auart_init,
        .class_init     = mxs_auart_class_init,
    },
};

DEFINE_TYPES(mxs_auart_types)
