/*
 * Freescale i.MX28 (MXS) AHB-to-APBX bridge DMA controller
 *
 * The APBH DMA engine walks a chain of command descriptors ("CCW").  Each
 * descriptor can carry a couple of PIO words which are written into the
 * peripheral attached to the channel before the data phase is executed.
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
#include "system/dma.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define MXS_APBX_CHANNELS   16
#define MXS_APBX_PIO_WORDS  16

#define APBX_CTRL0          0x0
#define APBX_CTRL1          0x1
#define APBX_CTRL2          0x2
#define APBX_CHANNEL_CTRL   0x3
#define APBX_DEVSEL         0x4
#define APBX_BURST_SIZE     0x5
#define APBX_DEBUG          0x6
#define APBX_CH_BASE        0x10        /* 0x100 / 0x10 */
#define APBX_CH_STRIDE      7           /* 0x70 / 0x10 */

#define CCW_COMMAND_MASK    0x3
#define CCW_CMD_NO_XFER     0
#define CCW_CMD_WRITE       1           /* device -> memory */
#define CCW_CMD_READ        2           /* memory -> device */
#define CCW_CMD_SENSE       3
#define CCW_CHAIN           (1 << 2)
#define CCW_IRQONCMPLT      (1 << 3)
#define CCW_NANDLOCK        (1 << 4)
#define CCW_NANDWAIT4READY  (1 << 5)
#define CCW_SEMAPHORE       (1 << 6)
#define CCW_WAIT4ENDCMD     (1 << 7)
#define CCW_HALTONTERMINATE (1 << 8)
#define CCW_PIO_WORDS_SHIFT 12
#define CCW_PIO_WORDS_MASK  0xf

typedef struct MXSApbhChannel {
    uint32_t curcmdar;
    uint32_t nxtcmdar;
    uint32_t cmd;
    uint32_t bar;
    uint32_t sema;      /* semaphore counter */
    uint32_t sema_reg;
    const MXSDmaOps *ops;
    void *opaque;
} MXSApbhChannel;

typedef struct MXSApbhState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[MXS_APBX_CHANNELS];

    uint32_t ctrl0;
    uint32_t ctrl1;
    uint32_t ctrl2;
    uint32_t channel_ctrl;
    uint32_t devsel;
    uint32_t burst_size;
    MXSApbhChannel ch[MXS_APBX_CHANNELS];
} MXSApbhState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSApbhState, MXS_APBX)

/* ---- lightweight debug tracing, enabled with MXS_DEBUG=1 in the env ---- */
static bool mxs_dbg_on(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("MXS_DEBUG");
        on = e && *e != '0';
    }
    return on;
}
#define MXSDBG(fmt, ...) \
    do { if (mxs_dbg_on()) { fprintf(stderr, fmt "\n", ## __VA_ARGS__); } } while (0)

void mxs_apbx_attach(DeviceState *dev, int channel,
                     const MXSDmaOps *ops, void *opaque)
{
    MXSApbhState *s = MXS_APBX(dev);

    assert(channel >= 0 && channel < MXS_APBX_CHANNELS);
    s->ch[channel].ops = ops;
    s->ch[channel].opaque = opaque;
}

static void mxs_apbx_update_irq(MXSApbhState *s)
{
    int i;

    for (i = 0; i < MXS_APBX_CHANNELS; i++) {
        bool level = (s->ctrl1 & (1u << i)) && (s->ctrl1 & (1u << (16 + i)));

        qemu_set_irq(s->irq[i], level);
    }
}

static void mxs_apbx_run(MXSApbhState *s, int n)
{
    MXSApbhChannel *ch = &s->ch[n];
    AddressSpace *as = &address_space_memory;
    int guard = 0;

    while (ch->sema > 0 && guard++ < 1024) {
        uint32_t desc[4 + MXS_APBX_PIO_WORDS];
        uint32_t cmdaddr = ch->nxtcmdar;
        uint32_t flags, xfer_bytes, bufaddr, npio;

        dma_memory_read(as, cmdaddr, desc, sizeof(desc), MEMTXATTRS_UNSPECIFIED);

        ch->curcmdar = cmdaddr;
        ch->cmd = desc[1];
        ch->bar = desc[2];

        flags = desc[1] & 0xffff;
        xfer_bytes = desc[1] >> 16;
        bufaddr = desc[2];
        npio = (flags >> CCW_PIO_WORDS_SHIFT) & CCW_PIO_WORDS_MASK;

        MXSDBG("apbh%d: ccw@%08x next=%08x bits=%04x bytes=%u buf=%08x npio=%u",
               n, cmdaddr, desc[0], flags, xfer_bytes, bufaddr, npio);

        if (ch->ops && ch->ops->pio && npio) {
            ch->ops->pio(ch->opaque, &desc[3], npio);
        }

        switch (flags & CCW_COMMAND_MASK) {
        case CCW_CMD_WRITE:     /* peripheral -> memory */
            if (ch->ops && ch->ops->xfer && xfer_bytes) {
                g_autofree uint8_t *buf = g_malloc0(xfer_bytes);

                ch->ops->xfer(ch->opaque, buf, xfer_bytes, false);
                dma_memory_write(as, bufaddr, buf, xfer_bytes,
                                 MEMTXATTRS_UNSPECIFIED);
            }
            break;
        case CCW_CMD_READ:      /* memory -> peripheral */
            if (ch->ops && ch->ops->xfer && xfer_bytes) {
                g_autofree uint8_t *buf = g_malloc0(xfer_bytes);

                dma_memory_read(as, bufaddr, buf, xfer_bytes,
                                MEMTXATTRS_UNSPECIFIED);
                ch->ops->xfer(ch->opaque, buf, xfer_bytes, true);
            }
            break;
        default:
            break;
        }

        if (ch->ops && ch->ops->complete) {
            ch->ops->complete(ch->opaque);
        }

        if (flags & CCW_SEMAPHORE) {
            ch->sema--;
        }
        if (flags & CCW_IRQONCMPLT) {
            s->ctrl1 |= 1u << n;
        }
        if (flags & CCW_CHAIN) {
            ch->nxtcmdar = desc[0];
        } else {
            ch->sema = 0;
            break;
        }
    }

    mxs_apbx_update_irq(s);
}

static uint64_t mxs_apbx_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSApbhState *s = MXS_APBX(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val = 0;

    switch (idx) {
    case APBX_CTRL0:
        val = s->ctrl0;
        break;
    case APBX_CTRL1:
        val = s->ctrl1;
        break;
    case APBX_CTRL2:
        val = s->ctrl2;
        break;
    case APBX_CHANNEL_CTRL:
        val = s->channel_ctrl;
        break;
    case APBX_DEVSEL:
        val = s->devsel;
        break;
    case APBX_BURST_SIZE:
        val = s->burst_size;
        break;
    default:
        if (idx >= APBX_CH_BASE) {
            unsigned n = (idx - APBX_CH_BASE) / APBX_CH_STRIDE;
            unsigned reg = (idx - APBX_CH_BASE) % APBX_CH_STRIDE;

            if (n < MXS_APBX_CHANNELS) {
                switch (reg) {
                case 0:
                    val = s->ch[n].curcmdar;
                    break;
                case 1:
                    val = s->ch[n].nxtcmdar;
                    break;
                case 2:
                    val = s->ch[n].cmd;
                    break;
                case 3:
                    val = s->ch[n].bar;
                    break;
                case 4:
                    val = (s->ch[n].sema & 0xff) << 16;
                    break;
                default:
                    val = 0;
                    break;
                }
            }
        }
        break;
    }

    return mxs_bank_extract(offset, size, val);
}

static void mxs_apbx_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    MXSApbhState *s = MXS_APBX(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);

    switch (idx) {
    case APBX_CTRL0:
        s->ctrl0 = mxs_bank_apply(s->ctrl0, offset, value, size);
        if (s->ctrl0 & (1u << 31)) {
            s->ctrl0 |= (1u << 30);
        }
        break;
    case APBX_CTRL1:
        s->ctrl1 = mxs_bank_apply(s->ctrl1, offset, value, size);
        mxs_apbx_update_irq(s);
        break;
    case APBX_CTRL2:
        s->ctrl2 = mxs_bank_apply(s->ctrl2, offset, value, size);
        break;
    case APBX_CHANNEL_CTRL:
        s->channel_ctrl = mxs_bank_apply(s->channel_ctrl, offset, value, size);
        /* RESET_CHANNEL bits are self clearing */
        if (s->channel_ctrl & 0xffff0000u) {
            int i;

            for (i = 0; i < MXS_APBX_CHANNELS; i++) {
                if (s->channel_ctrl & (1u << (16 + i))) {
                    s->ch[i].sema = 0;
                    s->ch[i].curcmdar = 0;
                    s->ch[i].nxtcmdar = 0;
                }
            }
            s->channel_ctrl &= 0x0000ffffu;
        }
        break;
    case APBX_DEVSEL:
        s->devsel = mxs_bank_apply(s->devsel, offset, value, size);
        break;
    case APBX_BURST_SIZE:
        s->burst_size = mxs_bank_apply(s->burst_size, offset, value, size);
        break;
    default:
        if (idx >= APBX_CH_BASE) {
            unsigned n = (idx - APBX_CH_BASE) / APBX_CH_STRIDE;
            unsigned reg = (idx - APBX_CH_BASE) % APBX_CH_STRIDE;

            if (n >= MXS_APBX_CHANNELS) {
                break;
            }
            switch (reg) {
            case 0:
                s->ch[n].curcmdar = mxs_bank_apply(s->ch[n].curcmdar, offset,
                                                   value, size);
                break;
            case 1:
                s->ch[n].nxtcmdar = mxs_bank_apply(s->ch[n].nxtcmdar, offset,
                                                   value, size);
                break;
            case 2:
                s->ch[n].cmd = mxs_bank_apply(s->ch[n].cmd, offset, value, size);
                break;
            case 3:
                s->ch[n].bar = mxs_bank_apply(s->ch[n].bar, offset, value, size);
                break;
            case 4: {
                uint32_t old = s->ch[n].sema_reg;
                uint32_t val = mxs_bank_apply(old, offset, value, size);

                s->ch[n].sema_reg = val;
                s->ch[n].sema += val & 0xff;
                s->ch[n].sema_reg = 0;
                if (s->ch[n].sema) {
                    mxs_apbx_run(s, n);
                }
                break;
            }
            default:
                break;
            }
        }
        break;
    }
}

MXS_TRACE_WRAP(mxs_apbx, "apbh")

static const MemoryRegionOps mxs_apbx_ops = {
    .read = mxs_apbx_read_tr,
    .write = mxs_apbx_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_apbx_reset(DeviceState *dev)
{
    MXSApbhState *s = MXS_APBX(dev);
    int i;

    s->ctrl0 = 0;
    s->ctrl1 = 0;
    s->ctrl2 = 0;
    s->channel_ctrl = 0;
    s->devsel = 0;
    s->burst_size = 0;
    for (i = 0; i < MXS_APBX_CHANNELS; i++) {
        s->ch[i].curcmdar = 0;
        s->ch[i].nxtcmdar = 0;
        s->ch[i].cmd = 0;
        s->ch[i].bar = 0;
        s->ch[i].sema = 0;
        s->ch[i].sema_reg = 0;
        qemu_set_irq(s->irq[i], 0);
    }
}

static void mxs_apbx_init(Object *obj)
{
    mxs_apbx_trace = mxs_trace_enabled("apbh");

    MXSApbhState *s = MXS_APBX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &mxs_apbx_ops, s, "mxs-apbx", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (i = 0; i < MXS_APBX_CHANNELS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static void mxs_apbx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mxs_apbx_reset);
}

static const TypeInfo mxs_apbx_types[] = {
    {
        .name           = TYPE_MXS_APBX,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSApbhState),
        .instance_init  = mxs_apbx_init,
        .class_init     = mxs_apbx_class_init,
    },
};

DEFINE_TYPES(mxs_apbx_types)
