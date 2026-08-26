/*
 * Freescale i.MX28 (MXS) USB 2.0 OTG controller (ChipIdea/ARC core)
 *
 * The SHARP Brain BSP brings both controllers up during driver load: it
 * resets the PHY, then writes USBCMD.RST and waits for the bit to clear
 * before configuring USBMODE and the endpoint list.  Nothing on the
 * emulated machine is plugged into either port, so all this model has to
 * do is behave like an idle, correctly resetting controller: the identity
 * and capability registers report a single port core, the self clearing
 * command bits clear, and the port never reports an attached device.
 *
 * There is deliberately no data path here - the model exists so that the
 * guest's USB stack finishes initialising instead of spinning forever.
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
#include "migration/vmstate.h"
#include "qom/object.h"

/* identity / hardware parameter block (read only) */
#define USB_ID              0x000
#define USB_HWGENERAL       0x004
#define USB_HWHOST          0x008
#define USB_HWDEVICE        0x00c
#define USB_HWTXBUF         0x010
#define USB_HWRXBUF         0x014

#define USB_GPTIMER0LD      0x080
#define USB_GPTIMER0CTRL    0x084
#define USB_GPTIMER1LD      0x088
#define USB_GPTIMER1CTRL    0x08c
#define USB_SBUSCFG         0x090

/* EHCI capability registers */
#define USB_CAPLENGTH       0x100
#define USB_HCSPARAMS       0x104
#define USB_HCCPARAMS       0x108
#define USB_DCIVERSION      0x120
#define USB_DCCPARAMS       0x124

/* operational registers */
#define USB_USBCMD          0x140
#define USB_USBSTS          0x144
#define USB_USBINTR         0x148
#define USB_FRINDEX         0x14c
#define USB_DEVICEADDR      0x154   /* PERIODICLISTBASE in host mode */
#define USB_ENDPTLISTADDR   0x158   /* ASYNCLISTADDR in host mode     */
#define USB_TTCTRL          0x15c
#define USB_BURSTSIZE       0x160
#define USB_TXFILLTUNING    0x164
#define USB_ULPI_VIEWPORT   0x170
#define USB_ENDPTNAK        0x178
#define USB_ENDPTNAKEN      0x17c
#define USB_CONFIGFLAG      0x180
#define USB_PORTSC1         0x184
#define USB_OTGSC           0x1a4
#define USB_USBMODE         0x1a8
#define USB_ENDPTSETUPSTAT  0x1ac
#define USB_ENDPTPRIME      0x1b0
#define USB_ENDPTFLUSH      0x1b4
#define USB_ENDPTSTAT       0x1b8
#define USB_ENDPTCOMPLETE   0x1bc
#define USB_ENDPTCTRL0      0x1c0
#define USB_NUM_ENDPOINTS   6

#define USBCMD_RS           (1u << 0)
#define USBCMD_RST          (1u << 1)
#define USBCMD_IAA          (1u << 6)
#define USBCMD_LR           (1u << 7)   /* light host controller reset */
#define USBCMD_ATDTW        (1u << 14)
#define USBCMD_SUTW         (1u << 15)

#define USBSTS_UI           (1u << 0)
#define USBSTS_UEI          (1u << 1)
#define USBSTS_PCI          (1u << 2)
#define USBSTS_FRI          (1u << 3)
#define USBSTS_SEI          (1u << 4)
#define USBSTS_AAI          (1u << 5)
#define USBSTS_URI          (1u << 6)
#define USBSTS_SRI          (1u << 7)
#define USBSTS_SLI          (1u << 8)
#define USBSTS_ULPII        (1u << 10)
#define USBSTS_HCH          (1u << 12)
#define USBSTS_RCL          (1u << 13)
#define USBSTS_PS           (1u << 14)
#define USBSTS_AS           (1u << 15)
#define USBSTS_NAKI         (1u << 16)
#define USBSTS_TI0          (1u << 24)
#define USBSTS_TI1          (1u << 25)

/* the write-one-to-clear part of USBSTS */
#define USBSTS_W1C          (USBSTS_UI | USBSTS_UEI | USBSTS_PCI | \
                             USBSTS_FRI | USBSTS_SEI | USBSTS_AAI | \
                             USBSTS_URI | USBSTS_SRI | USBSTS_SLI | \
                             USBSTS_ULPII | USBSTS_NAKI | \
                             USBSTS_TI0 | USBSTS_TI1)

/* OTGSC: interrupt status bits are write-one-to-clear, ID reads 1 (no cable) */
#define OTGSC_ID            (1u << 8)
#define OTGSC_AVV           (1u << 9)
#define OTGSC_ASV           (1u << 10)
#define OTGSC_BSV           (1u << 11)
#define OTGSC_BSE           (1u << 12)
#define OTGSC_STATUS_MASK   0x0000ff00u
#define OTGSC_W1C           0x007f0000u

#define ULPI_VIEWPORT_RUN   (1u << 30)
#define ULPI_VIEWPORT_WU    (1u << 31)

typedef struct MXSUsbCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    char *name;
    bool trace;

    uint32_t usbcmd;
    uint32_t usbsts;
    uint32_t usbintr;
    uint32_t frindex;
    uint32_t deviceaddr;
    uint32_t endptlistaddr;
    uint32_t ttctrl;
    uint32_t burstsize;
    uint32_t txfilltuning;
    uint32_t ulpi_viewport;
    uint32_t endptnak;
    uint32_t endptnaken;
    uint32_t configflag;
    uint32_t portsc;
    uint32_t otgsc;
    uint32_t usbmode;
    uint32_t endptsetupstat;
    uint32_t endptcomplete;
    uint32_t endptctrl[USB_NUM_ENDPOINTS];
    uint32_t gptimer_ld[2];
    uint32_t gptimer_ctrl[2];
    uint32_t sbuscfg;
} MXSUsbCtrlState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSUsbCtrlState, MXS_USBCTRL)

static void mxs_usbctrl_update_irq(MXSUsbCtrlState *s)
{
    bool level = (s->usbsts & s->usbintr & 0x03ff03ff) != 0;

    /* the OTG interrupts have their enables in the upper half of OTGSC */
    if ((s->otgsc >> 16) & (s->otgsc >> 24) & 0x7f) {
        level = true;
    }
    qemu_set_irq(s->irq, level);
}

/* bring the operational registers back to their post reset state */
static void mxs_usbctrl_core_reset(MXSUsbCtrlState *s)
{
    int i;

    /* ITC = 8 micro frames, everything else stopped */
    s->usbcmd = 0x00080000;
    s->usbsts = USBSTS_HCH;
    s->usbintr = 0;
    s->frindex = 0;
    s->deviceaddr = 0;
    s->endptlistaddr = 0;
    s->ttctrl = 0;
    s->burstsize = 0x00001010;
    s->txfilltuning = 0;
    s->ulpi_viewport = 0;
    s->endptnak = 0;
    s->endptnaken = 0;
    s->configflag = 0;
    /*
     * Nothing is connected: CCS/CSC clear, PP set (port power available),
     * line state SE0.  Bit 24 (PHY type = UTMI) reads as 0.
     */
    s->portsc = 0x00001000;
    /*
     * ID = 1: no A plug in the receptacle, so the core is a B device with
     * no session (VBUS below the session valid threshold).
     */
    s->otgsc = OTGSC_ID | OTGSC_BSE;
    s->usbmode = 0;
    s->endptsetupstat = 0;
    s->endptcomplete = 0;
    for (i = 0; i < USB_NUM_ENDPOINTS; i++) {
        s->endptctrl[i] = 0;
    }
    mxs_usbctrl_update_irq(s);
}

static uint64_t mxs_usbctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSUsbCtrlState *s = MXS_USBCTRL(opaque);
    uint32_t val = 0;

    switch (offset) {
    case USB_ID:
        /* NID[13:8] = ~ID[5:0], ID = 0x05, revision 2 */
        val = 0x0022fa05;
        break;
    case USB_HWGENERAL:
        val = 0x00000035;
        break;
    case USB_HWHOST:
        val = 0x10020001;       /* one port, host capable */
        break;
    case USB_HWDEVICE:
        val = 0x0000000b;       /* device capable, 5 endpoints */
        break;
    case USB_HWTXBUF:
        val = 0x80060a10;
        break;
    case USB_HWRXBUF:
        val = 0x00000a10;
        break;
    case USB_GPTIMER0LD:
        val = s->gptimer_ld[0];
        break;
    case USB_GPTIMER0CTRL:
        val = s->gptimer_ctrl[0];
        break;
    case USB_GPTIMER1LD:
        val = s->gptimer_ld[1];
        break;
    case USB_GPTIMER1CTRL:
        val = s->gptimer_ctrl[1];
        break;
    case USB_SBUSCFG:
        val = s->sbuscfg;
        break;
    case USB_CAPLENGTH:
        /* CAPLENGTH = 0x40 (operational regs at +0x140), HCIVERSION 1.00 */
        val = 0x01000040;
        break;
    case USB_HCSPARAMS:
        val = 0x00010011;       /* 1 port, 1 companion, port power control */
        break;
    case USB_HCCPARAMS:
        val = 0x00000006;       /* programmable frame list, async park */
        break;
    case USB_DCIVERSION:
        val = 0x00000001;
        break;
    case USB_DCCPARAMS:
        val = 0x00000185;       /* host + device capable, 5 endpoints */
        break;
    case USB_USBCMD:
        val = s->usbcmd;
        break;
    case USB_USBSTS:
        val = s->usbsts;
        break;
    case USB_USBINTR:
        val = s->usbintr;
        break;
    case USB_FRINDEX:
        val = s->frindex;
        break;
    case USB_DEVICEADDR:
        val = s->deviceaddr;
        break;
    case USB_ENDPTLISTADDR:
        val = s->endptlistaddr;
        break;
    case USB_TTCTRL:
        val = s->ttctrl;
        break;
    case USB_BURSTSIZE:
        val = s->burstsize;
        break;
    case USB_TXFILLTUNING:
        val = s->txfilltuning;
        break;
    case USB_ULPI_VIEWPORT:
        val = s->ulpi_viewport;
        break;
    case USB_ENDPTNAK:
        val = s->endptnak;
        break;
    case USB_ENDPTNAKEN:
        val = s->endptnaken;
        break;
    case USB_CONFIGFLAG:
        val = s->configflag;
        break;
    case USB_PORTSC1:
        val = s->portsc;
        break;
    case USB_OTGSC:
        val = s->otgsc;
        break;
    case USB_USBMODE:
        val = s->usbmode;
        break;
    case USB_ENDPTSETUPSTAT:
        val = s->endptsetupstat;
        break;
    case USB_ENDPTPRIME:
    case USB_ENDPTFLUSH:
    case USB_ENDPTSTAT:
        val = 0;                /* nothing is ever primed */
        break;
    case USB_ENDPTCOMPLETE:
        val = s->endptcomplete;
        break;
    case USB_ENDPTCTRL0 ... USB_ENDPTCTRL0 + USB_NUM_ENDPOINTS * 4 - 1:
        val = s->endptctrl[(offset - USB_ENDPTCTRL0) / 4];
        break;
    default:
        val = 0;
        break;
    }

    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, false, offset, val);
    }
    return val;
}

static void mxs_usbctrl_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    MXSUsbCtrlState *s = MXS_USBCTRL(opaque);
    uint32_t val = value;

    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, true, offset, val);
    }

    switch (offset) {
    case USB_GPTIMER0LD:
        s->gptimer_ld[0] = val;
        break;
    case USB_GPTIMER0CTRL:
        s->gptimer_ctrl[0] = val & ~(1u << 31);     /* never runs */
        break;
    case USB_GPTIMER1LD:
        s->gptimer_ld[1] = val;
        break;
    case USB_GPTIMER1CTRL:
        s->gptimer_ctrl[1] = val & ~(1u << 31);
        break;
    case USB_SBUSCFG:
        s->sbuscfg = val;
        break;

    case USB_USBCMD:
        if (val & USBCMD_RST) {
            /*
             * Controller reset.  Real hardware needs a couple of PHY
             * clocks; the guest polls the bit, so simply complete it
             * here - the register must read back with RST clear.
             */
            mxs_usbctrl_core_reset(s);
            break;
        }
        /* the "attach done" and "setup tripwire" bits are software owned */
        s->usbcmd = val & ~USBCMD_RST;
        if (val & USBCMD_LR) {
            s->usbcmd &= ~USBCMD_LR;    /* light reset also self clears */
            s->portsc &= ~1u;
        }
        if (val & USBCMD_IAA) {
            s->usbcmd &= ~USBCMD_IAA;
            s->usbsts |= USBSTS_AAI;
        }
        if (val & USBCMD_RS) {
            s->usbsts &= ~USBSTS_HCH;
        } else {
            s->usbsts |= USBSTS_HCH;
        }
        mxs_usbctrl_update_irq(s);
        break;

    case USB_USBSTS:
        s->usbsts &= ~(val & USBSTS_W1C);
        mxs_usbctrl_update_irq(s);
        break;
    case USB_USBINTR:
        s->usbintr = val;
        mxs_usbctrl_update_irq(s);
        break;
    case USB_FRINDEX:
        s->frindex = val;
        break;
    case USB_DEVICEADDR:
        /* USBADRA (bit 24) makes the address take effect immediately */
        s->deviceaddr = val & ~(1u << 24);
        break;
    case USB_ENDPTLISTADDR:
        s->endptlistaddr = val;
        break;
    case USB_TTCTRL:
        s->ttctrl = val;
        break;
    case USB_BURSTSIZE:
        s->burstsize = val;
        break;
    case USB_TXFILLTUNING:
        s->txfilltuning = val;
        break;
    case USB_ULPI_VIEWPORT:
        /* no ULPI PHY: complete the access straight away */
        s->ulpi_viewport = val & ~(ULPI_VIEWPORT_RUN | ULPI_VIEWPORT_WU);
        break;
    case USB_ENDPTNAK:
        s->endptnak &= ~val;
        break;
    case USB_ENDPTNAKEN:
        s->endptnaken = val;
        break;
    case USB_CONFIGFLAG:
        s->configflag = val;
        break;
    case USB_PORTSC1:
        /*
         * Keep the read only status bits, take the control bits and clear
         * the write-one-to-clear change bits.  PORT RESET (bit 8) finishes
         * instantly because there is no device to reset.
         */
        s->portsc = (s->portsc & ~0x1f80003eu) |
                    (val & 0x1f80003eu & ~0x0000012au);
        s->portsc &= ~(val & 0x0000002au);      /* CSC/PEC/OCC */
        s->portsc &= ~0x00000100u;              /* PR completes at once */
        break;
    case USB_OTGSC:
        s->otgsc = (s->otgsc & (OTGSC_STATUS_MASK | (val ? 0 : 0))) |
                   (val & ~(OTGSC_STATUS_MASK | OTGSC_W1C));
        s->otgsc &= ~(val & OTGSC_W1C);
        s->otgsc |= OTGSC_ID | OTGSC_BSE;       /* still nothing plugged in */
        mxs_usbctrl_update_irq(s);
        break;
    case USB_USBMODE:
        s->usbmode = val;
        break;
    case USB_ENDPTSETUPSTAT:
        s->endptsetupstat &= ~val;
        break;
    case USB_ENDPTPRIME:
        /*
         * Priming completes immediately (and nothing ever transfers), so
         * ENDPTPRIME/ENDPTSTAT read back as zero.
         */
        break;
    case USB_ENDPTFLUSH:
        break;
    case USB_ENDPTCOMPLETE:
        s->endptcomplete &= ~val;
        mxs_usbctrl_update_irq(s);
        break;
    case USB_ENDPTCTRL0 ... USB_ENDPTCTRL0 + USB_NUM_ENDPOINTS * 4 - 1:
        s->endptctrl[(offset - USB_ENDPTCTRL0) / 4] = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps mxs_usbctrl_ops = {
    .read = mxs_usbctrl_read,
    .write = mxs_usbctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_usbctrl_reset(DeviceState *dev)
{
    mxs_usbctrl_core_reset(MXS_USBCTRL(dev));
}

static void mxs_usbctrl_realize(DeviceState *dev, Error **errp)
{
    MXSUsbCtrlState *s = MXS_USBCTRL(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->trace = mxs_trace_enabled(s->name);
    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_usbctrl_ops, s,
                          s->name ? s->name : "mxs-usbctrl", 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property mxs_usbctrl_properties[] = {
    DEFINE_PROP_STRING("name", MXSUsbCtrlState, name),
};

static const VMStateDescription vmstate_mxs_usbctrl = {
    .name = "mxs-usbctrl",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(usbcmd, MXSUsbCtrlState),
        VMSTATE_UINT32(usbsts, MXSUsbCtrlState),
        VMSTATE_UINT32(usbintr, MXSUsbCtrlState),
        VMSTATE_UINT32(frindex, MXSUsbCtrlState),
        VMSTATE_UINT32(deviceaddr, MXSUsbCtrlState),
        VMSTATE_UINT32(endptlistaddr, MXSUsbCtrlState),
        VMSTATE_UINT32(portsc, MXSUsbCtrlState),
        VMSTATE_UINT32(otgsc, MXSUsbCtrlState),
        VMSTATE_UINT32(usbmode, MXSUsbCtrlState),
        VMSTATE_UINT32_ARRAY(endptctrl, MXSUsbCtrlState, USB_NUM_ENDPOINTS),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_usbctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mxs_usbctrl_realize;
    device_class_set_legacy_reset(dc, mxs_usbctrl_reset);
    dc->vmsd = &vmstate_mxs_usbctrl;
    device_class_set_props(dc, mxs_usbctrl_properties);
}

static const TypeInfo mxs_usbctrl_types[] = {
    {
        .name           = TYPE_MXS_USBCTRL,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSUsbCtrlState),
        .class_init     = mxs_usbctrl_class_init,
    },
};

DEFINE_TYPES(mxs_usbctrl_types)
