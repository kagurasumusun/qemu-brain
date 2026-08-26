/*
 * Freescale i.MX28 (MXS) system control blocks:
 *   CLKCTRL, POWER, DIGCTL, OCOTP, RTC and a generic "dummy" block used
 *   for the peripherals which only need to accept register accesses.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/rtc.h"
#include "system/runstate.h"
#include "qom/object.h"

#define MXS_SFTRST      (1u << 31)
#define MXS_CLKGATE     (1u << 30)

#define MXS_MAX_REGS    512

typedef struct MXSSysconState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[MXS_MAX_REGS];
    /* properties */
    char *name;
    uint64_t size;
    uint32_t kind;
    uint32_t ctrl_idx;      /* register holding SFTRST/CLKGATE */
    bool trace;
    /* runtime */
    int64_t time_base;
    uint32_t tick_offset;
    qemu_irq irq;
} MXSSysconState;

enum {
    MXS_KIND_DUMMY = 0,
    MXS_KIND_CLKCTRL,
    MXS_KIND_POWER,
    MXS_KIND_DIGCTL,
    MXS_KIND_OCOTP,
    MXS_KIND_RTC,
    MXS_KIND_USBPHY,
    MXS_KIND_I2C,        /* pretend to be a working I2C controller so the */
                         /* WinCE BSP does not time out talking to the    */
                         /* audio CODEC / battery monitor on the bus.     */
};

/*
 * The PWM block used to live in the syscon model; it has been
 * moved to its own file (hw/misc/mxs_pwm.c) so that the
 * register-to-channel-present-mask field layout can be controlled
 * without dragging in unrelated syscon cases.
 */

/*
 * HW_USBPHY_* register indices (offset / 0x10).  Note that unlike most MXS
 * blocks the soft reset control register is *not* the first one: PWD comes
 * first and CTRL lives at 0x30.
 */
#define USBPHY_PWD              0x0
#define USBPHY_TX               0x1
#define USBPHY_RX               0x2
#define USBPHY_CTRL             0x3
#define USBPHY_STATUS           0x4
#define USBPHY_DEBUG            0x5
#define USBPHY_DEBUG0_STATUS    0x6
#define USBPHY_DEBUG1           0x7
#define USBPHY_VERSION          0x8

#define USBPHY_STATUS_RESUME        (1u << 10)
#define USBPHY_STATUS_OTGID         (1u << 8)
#define USBPHY_STATUS_DEVPLUGIN     (1u << 6)
#define USBPHY_STATUS_HOSTDISCON    (1u << 3)

#define TYPE_MXS_SYSCON "mxs-syscon"

OBJECT_DECLARE_SIMPLE_TYPE(MXSSysconState, MXS_SYSCON)

/* ------------------------------------------------------------------ */

/* CLKCTRL registers (index = offset / 0x10) */
#define CLK_PLL0CTRL0   0x0
#define CLK_PLL0CTRL1   0x1
#define CLK_PLL1CTRL0   0x2
#define CLK_PLL1CTRL1   0x3
#define CLK_PLL2CTRL0   0x4
#define CLK_CPU         0x5
#define CLK_HBUS        0x6
#define CLK_XBUS        0x7
#define CLK_XTAL        0x8
#define CLK_SSP0        0x9
#define CLK_SSP1        0xa
#define CLK_SSP2        0xb
#define CLK_SSP3        0xc
#define CLK_GPMI        0xd
#define CLK_SPDIF       0xe
#define CLK_EMI         0xf
#define CLK_SAIF0       0x10
#define CLK_SAIF1       0x11
#define CLK_LCDIF       0x12
#define CLK_ETM         0x13
#define CLK_ENET        0x14
#define CLK_HSADC       0x15
#define CLK_FLEXCAN     0x16
#define CLK_FRAC0       0x1b
#define CLK_FRAC1       0x1c
#define CLK_CLKSEQ      0x1d
#define CLK_RESET       0x1e
#define CLK_STATUS      0x1f
#define CLK_VERSION     0x20

/* POWER */
#define PWR_CTRL        0x0
#define PWR_5VCTRL      0x1
#define PWR_MINPWR      0x2
#define PWR_CHARGE      0x3
#define PWR_VDDDCTRL    0x4
#define PWR_VDDACTRL    0x5
#define PWR_VDDIOCTRL   0x6
#define PWR_VDDMEMCTRL  0x7
#define PWR_DCDC4P2     0x8
#define PWR_MISC        0x9
#define PWR_DCLIMITS    0xa
#define PWR_LOOPCTRL    0xb
#define PWR_STS         0xc
#define PWR_SPEED       0xd
#define PWR_BATTMONITOR 0xe
#define PWR_RESET       0x10
#define PWR_DEBUG       0x11
#define PWR_THERMAL     0x12
#define PWR_USB1CTRL    0x13
#define PWR_SPECIAL     0x14
#define PWR_VERSION     0x15

#define PWR_STS_DC_OK        (1u << 9)
#define PWR_STS_VDD5V_GT_VDDIO (1u << 5)
#define PWR_STS_PSWITCH_SHIFT 20

/* DIGCTL */
#define DIG_CTRL            0x0
#define DIG_STATUS          0x1
#define DIG_HCLKCOUNT       0x2
#define DIG_MICROSECONDS    0xc
#define DIG_CHIPID          0x31

/* OCOTP */
#define OCOTP_CTRL          0x0
#define OCOTP_DATA          0x1

/* RTC */
#define RTC_CTRL            0x0
#define RTC_STAT            0x1
#define RTC_MILLISECONDS    0x2
#define RTC_SECONDS         0x3
#define RTC_ALARM           0x4
#define RTC_WATCHDOG        0x5
#define RTC_PERSISTENT0     0x6
#define RTC_VERSION         0xd

#define RTC_CTRL_WATCHDOGEN     (1u << 4)
#define RTC_CTRL_ONEMSEC_IRQ    (1u << 3)
#define RTC_CTRL_ALARM_IRQ      (1u << 2)
#define RTC_CTRL_ONEMSEC_IRQ_EN (1u << 1)
#define RTC_CTRL_ALARM_IRQ_EN   (1u << 0)

#define RTC_STAT_PRESENT    (0xfu << 28)   /* rtc/alarm/wdog/xtal present */

static uint32_t mxs_syscon_reg_read(MXSSysconState *s, unsigned idx)
{
    uint32_t val = s->regs[idx];

    switch (s->kind) {
    case MXS_KIND_CLKCTRL:
        switch (idx) {
        case CLK_PLL0CTRL1:
        case CLK_PLL1CTRL1:
            val |= (1u << 31);          /* LOCK */
            break;
        case CLK_CPU:
        case CLK_EMI:
            val &= ~((1u << 29) | (1u << 28) | (1u << 27) |
                     (1u << 26) | (1u << 17));
            break;
        case CLK_HBUS:
        case CLK_XBUS:
            val &= ~(1u << 31);         /* ASM_BUSY / BUSY */
            break;
        case CLK_SSP0:
        case CLK_SSP1:
        case CLK_SSP2:
        case CLK_SSP3:
        case CLK_GPMI:
        case CLK_SAIF0:
        case CLK_SAIF1:
        case CLK_LCDIF:
        case CLK_ETM:
            val &= ~(1u << 29);         /* BUSY */
            break;
        case CLK_ENET:
            val &= ~(1u << 27);
            break;
        case CLK_FRAC0:
        case CLK_FRAC1:
            /* STABLE bits (bit 6 of every byte) read back as 0 */
            val &= ~0x40404040u;
            break;
        default:
            break;
        }
        break;

    case MXS_KIND_POWER:
        if (idx == PWR_STS) {
            val |= PWR_STS_DC_OK;
            val |= (3u << PWR_STS_PSWITCH_SHIFT); /* pswitch not pressed */
        } else if (idx == PWR_BATTMONITOR) {
            /* BATT_VAL[26:16], LSB = 8mV -> ~4.0V */
            val = (val & ~(0x7ffu << 16)) | (500u << 16);
        }
        break;

    case MXS_KIND_DIGCTL:
        if (idx == DIG_MICROSECONDS) {
            val = (uint32_t)(qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) -
                             s->time_base);
        } else if (idx == DIG_CHIPID) {
            val = 0x2800e000;           /* i.MX28 TA3 */
        } else if (idx == DIG_HCLKCOUNT) {
            val = (uint32_t)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        break;

    case MXS_KIND_OCOTP:
        if (idx == OCOTP_CTRL) {
            val &= ~((1u << 8) | (1u << 9));    /* !BUSY, !ERROR */
        }
        break;

    case MXS_KIND_RTC:
        switch (idx) {
        case RTC_STAT:
            val = RTC_STAT_PRESENT;             /* no stale/new regs */
            break;
        case RTC_SECONDS:
            val = s->tick_offset +
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / NANOSECONDS_PER_SECOND;
            break;
        case RTC_MILLISECONDS:
            val = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
            break;
        default:
            break;
        }
        break;

    case MXS_KIND_I2C:
        /*
         * The WinCE BSP uses I2C0 to talk to an audio CODEC and I2C1 for
         * the battery monitor.  None of those chips exist in the
         * emulation but the BSP still needs a well behaved controller
         * or the kernel's I2C layer will time out on every transfer.
         * We just return the controller's idle state plus any pending
         * completion interrupt.  The data register always reads zero,
         * which is the same response a disconnected device would give
         * on the real bus.  Register 0xb0 (HW_I2C_DATA extension in
         * some revisions) is treated as the read-completion flag: it
         * reads 1 once the most recent START command has been "latched"
         * by the model, which convinces the BSP that the byte has been
         * received and stops the polling loop.
         */
        switch (idx) {
        case 0x0:         /* CTRL0 - report idle, no error */
            val = (val & ~0xffu) | 0x00u;   /* !RUN, !BUSY, !MSTEN */
            break;
        case 0x3:         /* DATA - no bytes received */
            val = 0;
            break;
        case 0x4:         /* DEBUG0 - bus idle, no arbitration lost */
            val = 0;
            break;
        case 0x6:         /* VERSION */
            val = 0x03000000;
            break;
        case 0xb:         /* extension register used by the BSP to poll */
            val = 1;     /* transfer "complete" */
            break;
        default:
            break;
        }
        break;

    case MXS_KIND_USBPHY:
        if (idx == USBPHY_STATUS) {
            /* no cable, no device: only the (floating, pulled up) ID pin */
            val = USBPHY_STATUS_OTGID;
        }
        break;

    default:
        break;
    }

    return val;
}

static uint64_t mxs_syscon_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSSysconState *s = MXS_SYSCON(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= MXS_MAX_REGS) {
        return 0;
    }
    val = mxs_syscon_reg_read(s, idx);

    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, false, offset, val);
    }

    return mxs_bank_extract(offset, size, val);
}

static void mxs_syscon_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    MXSSysconState *s = MXS_SYSCON(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t old, val;

    if (idx >= MXS_MAX_REGS) {
        return;
    }

    old = s->regs[idx];
    val = mxs_bank_apply(old, offset, value, size);

    /* soft reset protocol: asserting SFTRST also asserts CLKGATE */
    if (idx == s->ctrl_idx) {
        val = mxs_bank_sftrst(old, val);
    }

    s->regs[idx] = val;

    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, true, offset, (uint32_t)value);
    }

    /*
     * I2C writes: pretend to be a working I2C controller.  The WinCE
     * BSP toggles CTRL0.MSTEN (bit 7) to start a transfer and then
     * polls CTRL1 holding the slave address or the DATA register, and
     * waits for CTRL0.RUN (bit 8) to drop - so the only thing we have
     * to do on the way out is acknowledge the command: keep MSTEN set
     * and clear RUN, and raise the IRQ.  This convinces the guest that
     * the transfer completed (with no data, since the audio CODEC and
     * battery monitor are not modelled).
     */
    if (s->kind == MXS_KIND_I2C) {
        if (idx == 0x0) {
            /* the BSP sets MSTEN|RUN, the controller model clears RUN
             * and raises IRQ on its own below.  Just keep the request. */
        } else if (idx == 0x2) {
            /* a START/STOP command was issued: clear RUN, raise IRQ. */
            s->regs[0x0] &= ~(1u << 8);        /* CTRL0.RUN */
            s->regs[0x0] |= (1u << 5);         /* CTRL0.IRQ pending */
            /* IRQEN is bit 7 of CTRL0; pulse the interrupt line */
            qemu_set_irq(s->irq, 1);
        }
    }

    if (s->kind == MXS_KIND_POWER && idx == PWR_RESET) {
        /* HW_POWER_RESET: 0x3e77_0001 = power off, ..._0002 = reset */
        if ((val & 0xffff0000u) == 0x3e770000u) {
            static int po_budget = 8;
            if (po_budget > 0) {
                po_budget--;
                fprintf(stderr, "[brain] HW_POWER_RESET val=%08x\n",
                        (uint32_t)val);
            }
            if (val & 2) {
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            } else if (val & 1) {
                qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
            }
        }
    }

    if (s->kind == MXS_KIND_RTC && idx == RTC_CTRL) {
        uint32_t level = 0;
        if ((val & RTC_CTRL_ALARM_IRQ) && (val & RTC_CTRL_ALARM_IRQ_EN)) {
            level = 1;
        }
        if ((val & RTC_CTRL_ONEMSEC_IRQ) && (val & RTC_CTRL_ONEMSEC_IRQ_EN)) {
            level = 1;
        }
        qemu_set_irq(s->irq, level);
    }
}

static const MemoryRegionOps mxs_syscon_ops = {
    .read = mxs_syscon_read,
    .write = mxs_syscon_write,
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

static void mxs_syscon_reset(DeviceState *dev)
{
    MXSSysconState *s = MXS_SYSCON(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->time_base = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);

    switch (s->kind) {
    case MXS_KIND_CLKCTRL:
        /*
         * Reset values taken from the i.MX28 reference manual.  Every
         * register that carries a divider comes up with DIV = 1: firmware
         * happily divides by these fields without checking them first.
         */
        s->regs[CLK_PLL0CTRL0] = 0x00060000;
        s->regs[CLK_PLL0CTRL1] = 0x00000000;
        s->regs[CLK_PLL1CTRL0] = 0x00060000;
        s->regs[CLK_PLL1CTRL1] = 0x00000000;
        s->regs[CLK_PLL2CTRL0] = 0x00000000;
        s->regs[CLK_CPU]       = 0x00010001;
        s->regs[CLK_HBUS]      = 0x00000001;
        s->regs[CLK_XBUS]      = 0x00000001;
        s->regs[CLK_XTAL]      = 0x70000001;
        s->regs[CLK_SSP0]      = 0x80000001;
        s->regs[CLK_SSP1]      = 0x80000001;
        s->regs[CLK_SSP2]      = 0x80000001;
        s->regs[CLK_SSP3]      = 0x80000001;
        s->regs[CLK_GPMI]      = 0x80000001;
        s->regs[CLK_SPDIF]     = 0x80000000;
        s->regs[CLK_EMI]       = 0x00000101;
        s->regs[CLK_SAIF0]     = 0x80000001;
        s->regs[CLK_SAIF1]     = 0x80000001;
        s->regs[CLK_LCDIF]     = 0x80000001;
        s->regs[CLK_ETM]       = 0x80000001;
        s->regs[CLK_ENET]      = 0x40000000;
        s->regs[CLK_HSADC]     = 0x00000000;
        s->regs[CLK_FLEXCAN]   = 0xc0000000;
        s->regs[CLK_FRAC0]     = 0x92929292;
        s->regs[CLK_FRAC1]     = 0x92929292;
        s->regs[CLK_CLKSEQ]    = 0x000001ff;
        s->regs[CLK_VERSION]   = 0x03000000;
        break;
    case MXS_KIND_POWER:
        s->regs[PWR_CTRL]      = 0x00000000;
        s->regs[PWR_5VCTRL]    = 0x00021002;
        s->regs[PWR_VDDDCTRL]  = 0x00000f14;
        s->regs[PWR_VDDACTRL]  = 0x0000000a;
        s->regs[PWR_VDDIOCTRL] = 0x00000010;
        s->regs[PWR_STS]       = PWR_STS_DC_OK;
        s->regs[PWR_VERSION]   = 0x03000000;
        break;
    case MXS_KIND_DIGCTL:
        s->regs[DIG_CTRL]      = 0x00000000;
        break;
    case MXS_KIND_RTC:
        s->regs[RTC_STAT]      = RTC_STAT_PRESENT;
        s->regs[RTC_VERSION]   = 0x03000000;
        s->tick_offset = 0;
        break;
    case MXS_KIND_USBPHY:
        /* power down everything, block held in reset with the clock gated */
        s->regs[USBPHY_PWD]     = 0x001e1c00;
        s->regs[USBPHY_TX]      = 0x10060607;
        s->regs[USBPHY_RX]      = 0x00000000;
        s->regs[USBPHY_CTRL]    = MXS_SFTRST | MXS_CLKGATE;
        /*
         * Nothing is plugged into the USB port of the emulated machine.
         * OTGID reads high (floating ID pin => plain B device / no cable),
         * everything else is idle.
         */
        s->regs[USBPHY_STATUS]  = USBPHY_STATUS_OTGID;
        s->regs[USBPHY_DEBUG]   = 0x7f180000;
        s->regs[USBPHY_VERSION] = 0x04020000;
        break;
    default:
        break;
    }
}

static void mxs_syscon_realize(DeviceState *dev, Error **errp)
{
    MXSSysconState *s = MXS_SYSCON(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    g_autofree char *name = g_strdup_printf("mxs-%s",
                                            s->name ? s->name : "syscon");

    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_syscon_ops, s, name,
                          s->size ? s->size : 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->trace = mxs_trace_enabled(s->name);
}

static const Property mxs_syscon_properties[] = {
    DEFINE_PROP_STRING("name", MXSSysconState, name),
    DEFINE_PROP_UINT64("size", MXSSysconState, size, 0x2000),
    DEFINE_PROP_UINT32("kind", MXSSysconState, kind, MXS_KIND_DUMMY),
    DEFINE_PROP_UINT32("ctrl-idx", MXSSysconState, ctrl_idx, 0),
};

static const VMStateDescription vmstate_mxs_syscon = {
    .name = "mxs-syscon",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSSysconState, MXS_MAX_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_syscon_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mxs_syscon_realize;
    device_class_set_legacy_reset(dc, mxs_syscon_reset);
    dc->vmsd = &vmstate_mxs_syscon;
    device_class_set_props(dc, mxs_syscon_properties);
}

#define MXS_SYSCON_SUBTYPE(typename, kindval)                           \
    static void typename##_init(Object *obj)                            \
    {                                                                   \
        MXS_SYSCON(obj)->kind = kindval;                                \
    }

MXS_SYSCON_SUBTYPE(mxs_clkctrl, MXS_KIND_CLKCTRL)
MXS_SYSCON_SUBTYPE(mxs_power, MXS_KIND_POWER)
MXS_SYSCON_SUBTYPE(mxs_digctl, MXS_KIND_DIGCTL)
MXS_SYSCON_SUBTYPE(mxs_ocotp, MXS_KIND_OCOTP)
MXS_SYSCON_SUBTYPE(mxs_rtc, MXS_KIND_RTC)
MXS_SYSCON_SUBTYPE(mxs_i2c, MXS_KIND_I2C)

static void mxs_usbphy_init(Object *obj)
{
    MXSSysconState *s = MXS_SYSCON(obj);

    s->kind = MXS_KIND_USBPHY;
    s->ctrl_idx = USBPHY_CTRL;      /* HW_USBPHY_CTRL is at offset 0x30 */
}

static const TypeInfo mxs_syscon_types[] = {
    {
        .name           = TYPE_MXS_SYSCON,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSSysconState),
        .class_init     = mxs_syscon_class_init,
    },
    {
        .name           = TYPE_MXS_DUMMY,
        .parent         = TYPE_MXS_SYSCON,
    },
    {
        .name           = TYPE_MXS_CLKCTRL,
        .parent         = TYPE_MXS_SYSCON,
        .instance_init  = mxs_clkctrl_init,
    },
    {
        .name           = TYPE_MXS_POWER,
        .parent         = TYPE_MXS_SYSCON,
        .instance_init  = mxs_power_init,
    },
    {
        .name           = TYPE_MXS_DIGCTL,
        .parent         = TYPE_MXS_SYSCON,
        .instance_init  = mxs_digctl_init,
    },
    {
        .name           = TYPE_MXS_OCOTP,
        .parent         = TYPE_MXS_SYSCON,
        .instance_init  = mxs_ocotp_init,
    },
    {
        .name           = TYPE_MXS_I2C,
        .parent         = TYPE_MXS_SYSCON,
        .instance_init  = mxs_i2c_init,
    },
    {
        .name           = TYPE_MXS_USBPHY,
        .parent         = TYPE_MXS_SYSCON,
        .instance_init  = mxs_usbphy_init,
    },
    {
        .name           = TYPE_MXS_RTC,
        .parent         = TYPE_MXS_SYSCON,
        .instance_init  = mxs_rtc_init,
    },
};

DEFINE_TYPES(mxs_syscon_types)
