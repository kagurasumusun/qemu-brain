/*
 * Freescale i.MX28 (MXS) PINCTRL / GPIO block
 *
 * Register map — verified by disassembling the SHARP Brain's WinCE BSP
 * (cspddk.dll DDKGpioConfig/ReadDataPin/WriteDataPin/ClearIntrPin and the
 * EBOOT pin-setup code).  Each function register is spaced 0x10 bytes per
 * bank, with SET/CLR/TOG aliases at +0x4/+0x8/+0xc (MXS_BANK format):
 *
 *   CTRL      0x000
 *   MUXSEL    0x300 + bank*0x10   (4 bits per pin, ALT number)
 *   DRIVE     0x600 + bank*0x10   (2 bits per pin)
 *   DOUT      0x700 + bank*0x10   (output value, 1 bit per pin)
 *   DIN       0x900 + bank*0x10   (input value, read only)
 *   DOE       0xB00 + bank*0x10   (output enable, 1 bit per pin)
 *   IRQEN     0x1000 + bank*0x10
 *   PIN2IRQ   0x1100 + bank*0x10
 *   IRQLEVEL  0x1200 + bank*0x10
 *   IRQPOL    0x1300 + bank*0x10
 *   IRQSTAT   0x1400 + bank*0x10
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
#include "migration/vmstate.h"
#include "qom/object.h"
#include "brain_stats.h"

#define MXS_PINCTRL_BANKS   5
#define MXS_PINCTRL_REGS    512

/* Register indices (idx = byte offset >> 4; banks are +1 per 0x10) */
#define PIN_CTRL_IDX       0x00    /* 0x000 */
#define PIN_MUXSEL_IDX     0x30    /* 0x300 + bank*0x10 */
#define PIN_DRIVE_IDX      0x60    /* 0x600 + bank*0x10 */
#define PIN_DOUT_IDX       0x70    /* 0x700 + bank*0x10 */
#define PIN_DIN_IDX        0x90    /* 0x900 + bank*0x10 */
#define PIN_DOE_IDX        0xB0    /* 0xB00 + bank*0x10 */
#define PIN_IRQEN_IDX      0x100   /* 0x1000 + bank*0x10 */
#define PIN_PIN2IRQ_IDX    0x110   /* 0x1100 + bank*0x10 */
#define PIN_IRQLEVEL_IDX   0x120   /* 0x1200 + bank*0x10 */
#define PIN_IRQPOL_IDX      0x130   /* 0x1300 + bank*0x10 */
#define PIN_IRQSTAT_IDX    0x140   /* 0x1400 + bank*0x10 */

typedef struct MXSPinctrlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[MXS_PINCTRL_BANKS];

    uint32_t regs[MXS_PINCTRL_REGS];
    /* value driven onto the pins from outside the SoC */
    uint32_t ext[MXS_PINCTRL_BANKS];

    void (*notify)(void *opaque);
    void *notify_opaque;
} MXSPinctrlState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSPinctrlState, MXS_PINCTRL)

static void mxs_pinctrl_update_irq(MXSPinctrlState *s)
{
    int b;

    for (b = 0; b < MXS_PINCTRL_BANKS; b++) {
        /*
         * i.MX28: the pin detector is selected by PIN2IRQ, armed by
         * IRQEN, and latched in IRQSTAT.  OEMInterruptHandler for
         * GPIO2 (ICOLL 125) does IRQSTAT & PIN2IRQ; if PIN2IRQ is 0
         * it prints "undefined IRQ" and never clears STAT.  Live
         * repaired4: IRQEN2=0x003f0000, PIN2IRQ2=0, so bank 2 must
         * not raise ICOLL 125 (measured STAT=0x7d + 61-storm when
         * we raised on IRQSTAT&IRQEN alone).
         */
        uint32_t stat = s->regs[PIN_IRQSTAT_IDX + b] &
                        s->regs[PIN_IRQEN_IDX + b] &
                        s->regs[PIN_PIN2IRQ_IDX + b];

        qemu_set_irq(s->irq[b], stat ? 1 : 0);
    }
}

static uint32_t mxs_pinctrl_din(MXSPinctrlState *s, int bank)
{
    uint32_t doe = s->regs[PIN_DOE_IDX + bank];
    uint32_t dout = s->regs[PIN_DOUT_IDX + bank];

    return (dout & doe) | (s->ext[bank] & ~doe);
}

void mxs_pinctrl_set_din(DeviceState *dev, int bank, uint32_t value)
{
    MXSPinctrlState *s = MXS_PINCTRL(dev);
    uint32_t old_din, new_din, changed, inputs, sel, lvl, pol, dir_match;

    if (bank < 0 || bank >= MXS_PINCTRL_BANKS) {
        return;
    }

    old_din = mxs_pinctrl_din(s, bank);
    s->ext[bank] = value;
    new_din = mxs_pinctrl_din(s, bank);
    s->regs[PIN_DIN_IDX + bank] = new_din;

    /*
     * An external device toggled input pins: model the pin-IRQ detectors
     * that latch HW_PINCTRL_IRQSTATn.  Without this, edge-triggered GPIO
     * interrupts never fire and the WinCE key matrix / touchkey drivers
     * wait forever on their GPIO bank interrupt.
     */
    inputs  = ~s->regs[PIN_DOE_IDX + bank];
    changed = (old_din ^ new_din) & inputs;
    if (changed && getenv("BRAIN_PIN_DEBUG")) {
        fprintf(stderr, "[pin-debug] set_din bank=%d old=%08x new=%08x "
                "changed=%08x doe=%08x pin2irq=%08x irqen=%08x "
                "irqlvl=%08x irqpol=%08x pc=0x%08x\n",
                bank, old_din, new_din, changed,
                s->regs[PIN_DOE_IDX + bank],
                s->regs[PIN_PIN2IRQ_IDX + bank],
                s->regs[PIN_IRQEN_IDX + bank],
                s->regs[PIN_IRQLEVEL_IDX + bank],
                s->regs[PIN_IRQPOL_IDX + bank],
                (unsigned)mxs_trace_guest_pc());
    }
    if (changed) {
        sel = s->regs[PIN_PIN2IRQ_IDX + bank];
        lvl = s->regs[PIN_IRQLEVEL_IDX + bank];
        pol = s->regs[PIN_IRQPOL_IDX + bank];
        /* per-pin "transition went in the sensitive direction" */
        dir_match = (new_din & pol) | (~new_din & ~pol);

        /* edge-sensitive pins latch IRQSTAT on the detected edge */
        s->regs[PIN_IRQSTAT_IDX + bank] |= changed & ~lvl & sel & dir_match;
        /* level-sensitive pins mirror the (active) input level */
        s->regs[PIN_IRQSTAT_IDX + bank] =
            (s->regs[PIN_IRQSTAT_IDX + bank] & ~(lvl & sel & inputs)) |
            (dir_match & lvl & sel & inputs);

        if (s->regs[PIN_IRQSTAT_IDX + bank] & s->regs[PIN_IRQEN_IDX + bank]) {
            brain_log_event(BSTAG('P', 'I', 'R', 'Q'), bank, changed,
                            s->regs[PIN_IRQSTAT_IDX + bank],
                            s->regs[PIN_IRQEN_IDX + bank]);
        }
        mxs_pinctrl_update_irq(s);
    }
}

void mxs_pinctrl_set_din_poll(DeviceState *dev, int bank, uint32_t value)
{
    MXSPinctrlState *s = MXS_PINCTRL(dev);

    if (bank < 0 || bank >= MXS_PINCTRL_BANKS) {
        return;
    }
    s->ext[bank] = value;
    s->regs[PIN_DIN_IDX + bank] = mxs_pinctrl_din(s, bank);
}

uint32_t mxs_pinctrl_get_ext(DeviceState *dev, int bank)
{
    MXSPinctrlState *s = MXS_PINCTRL(dev);

    return (bank >= 0 && bank < MXS_PINCTRL_BANKS) ? s->ext[bank] : 0;
}

uint32_t mxs_pinctrl_get_dout(DeviceState *dev, int bank)
{
    MXSPinctrlState *s = MXS_PINCTRL(dev);

    return (bank >= 0 && bank < MXS_PINCTRL_BANKS) ?
           s->regs[PIN_DOUT_IDX + bank] : 0;
}

uint32_t mxs_pinctrl_get_doe(DeviceState *dev, int bank)
{
    MXSPinctrlState *s = MXS_PINCTRL(dev);

    return (bank >= 0 && bank < MXS_PINCTRL_BANKS) ?
           s->regs[PIN_DOE_IDX + bank] : 0;
}

void mxs_pinctrl_set_notify(DeviceState *dev, void (*cb)(void *), void *opaque)
{
    MXSPinctrlState *s = MXS_PINCTRL(dev);

    s->notify = cb;
    s->notify_opaque = opaque;
}

static uint64_t mxs_pinctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSPinctrlState *s = MXS_PINCTRL(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= MXS_PINCTRL_REGS) {
        return 0;
    }

    if (idx >= PIN_DIN_IDX && idx < PIN_DIN_IDX + MXS_PINCTRL_BANKS) {
        val = mxs_pinctrl_din(s, idx - PIN_DIN_IDX);
    } else {
        val = s->regs[idx];
    }

    return mxs_bank_extract(offset, size, val);
}

static void mxs_pinctrl_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    MXSPinctrlState *s = MXS_PINCTRL(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= MXS_PINCTRL_REGS) {
        return;
    }
    if (idx >= PIN_DIN_IDX && idx < PIN_DIN_IDX + MXS_PINCTRL_BANKS) {
        return;
    }

    val = mxs_bank_apply(s->regs[idx], offset, value, size);
    if (idx == PIN_CTRL_IDX && (val & (1u << 31))) {
        val |= (1u << 30);
    }
    s->regs[idx] = val;

    if ((idx >= PIN_DOUT_IDX && idx < PIN_DOUT_IDX + MXS_PINCTRL_BANKS) ||
        (idx >= PIN_DOE_IDX && idx < PIN_DOE_IDX + MXS_PINCTRL_BANKS)) {
        if (s->notify) {
            s->notify(s->notify_opaque);
        }
    }
    if (idx >= PIN_IRQSTAT_IDX && idx < PIN_IRQSTAT_IDX + MXS_PINCTRL_BANKS) {
        mxs_pinctrl_update_irq(s);
    }
    if (idx >= PIN_IRQEN_IDX && idx < PIN_IRQEN_IDX + MXS_PINCTRL_BANKS) {
        mxs_pinctrl_update_irq(s);
    }
    /*
     * PIN2IRQ selects which pins feed the IRQ detectors; toggling it
     * re-arms/disarms the bank interrupt.  Without this the GPIO IRQ
     * line stays asserted after the guest clears PIN2IRQ (observed on
     * repaired4: keybd_EDNA2/cspddk clears PIN2IRQ2 while servicing,
     * ICOLL 125 stayed raw and OEMInterruptHandler re-entered in an
     * endless "undefined IRQ (61)" loop).
     */
    if (idx >= PIN_PIN2IRQ_IDX && idx < PIN_PIN2IRQ_IDX + MXS_PINCTRL_BANKS) {
        mxs_pinctrl_update_irq(s);
    }
}

MXS_TRACE_WRAP(mxs_pinctrl, "pinctrl")

static const MemoryRegionOps mxs_pinctrl_ops = {
    .read = mxs_pinctrl_read_tr,
    .write = mxs_pinctrl_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_pinctrl_reset(DeviceState *dev)
{
    MXSPinctrlState *s = MXS_PINCTRL(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    /*
     * After reset every pin is muxed to its ALT0 function (GPIO) and
     * pulled up: MUXSEL fields are 0.  The BSP reconfigures the pins it
     * needs via MUXSEL/DRIVE/PULL/DOE/IRQEN during OAL init.
     */
    for (i = 0; i < MXS_PINCTRL_BANKS; i++) {
        s->ext[i] = 0xffffffff;     /* pulled up by default */
    }
}

static void mxs_pinctrl_init(Object *obj)
{
    mxs_pinctrl_trace = mxs_trace_enabled("pinctrl");

    MXSPinctrlState *s = MXS_PINCTRL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &mxs_pinctrl_ops, s, "mxs-pinctrl",
                          0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (i = 0; i < MXS_PINCTRL_BANKS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static const VMStateDescription vmstate_mxs_pinctrl = {
    .name = "mxs-pinctrl",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSPinctrlState, MXS_PINCTRL_REGS),
        VMSTATE_UINT32_ARRAY(ext, MXSPinctrlState, MXS_PINCTRL_BANKS),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_pinctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mxs_pinctrl_reset);
    dc->vmsd = &vmstate_mxs_pinctrl;
}

static const TypeInfo mxs_pinctrl_types[] = {
    {
        .name           = TYPE_MXS_PINCTRL,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSPinctrlState),
        .instance_init  = mxs_pinctrl_init,
        .class_init     = mxs_pinctrl_class_init,
    },
};

DEFINE_TYPES(mxs_pinctrl_types)
