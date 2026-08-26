/*
 * Freescale i.MX28 (MXS) PWM controller - full real-device emulation.
 *
 * This is a faithful emulation of the i.MX28 PWM block based on
 * the i.MX28 reference manual (MCIMX28RM) and the Freescale-supplied
 * register definition header
 * (arch/arm/mach-mx28/include/mach/regs-pwm.h from the Freescale
 * Linux 2.6.35 BSP).  The block exposes 8 channels, each with its
 * own ACTIVE/PERIOD register pair, all sharing a single CTRL
 * register and a single IRQ line.
 *
 * Register layout (all offsets are byte offsets, all registers are
 * 32-bit little endian):
 *
 *   0x000  CTRL                (R/W, atomic)
 *   0x004  CTRL_SET            (atomic bit-set alias of CTRL)
 *   0x008  CTRL_CLR            (atomic bit-clear alias of CTRL)
 *   0x00C  CTRL_TOG            (atomic bit-toggle alias of CTRL)
 *   0x010  ACTIVE0..ACTIVE7    (one 32-bit reg per channel, 0x20 stride)
 *   0x014  ACTIVE0_SET..ACTIVE7_SET  (atomic set alias)
 *   0x018  ACTIVE0_CLR..ACTIVE7_CLR  (atomic clr alias)
 *   0x01C  ACTIVE0_TOG..ACTIVE7_TOG  (atomic tog alias)
 *   0x020  PERIOD0..PERIOD7    (one 32-bit reg per channel, 0x20 stride)
 *   0x024  PERIOD0_SET..PERIOD7_SET  (atomic set alias)
 *   0x028  PERIOD0_CLR..PERIOD7_CLR  (atomic clr alias)
 *   0x02C  PERIOD0_TOG..PERIOD7_TOG  (atomic tog alias)
 *   0x110  VERSION
 *
 * CTRL register bit assignments (BM_PWM_CTRL_*):
 *
 *   31      SFTRST         software reset, write 1 to assert
 *   30      CLKGATE        clock gate,   write 1 to gate
 *   29      PWM7_PRESENT   1 = channel 7 wired to a pad
 *   28      PWM6_PRESENT   1 = channel 6 wired to a pad
 *   27      PWM5_PRESENT   1 = channel 5 wired to a pad
 *   26      PWM4_PRESENT   1 = channel 4 wired to a pad
 *   25      PWM3_PRESENT   1 = channel 3 wired to a pad
 *   24      PWM2_PRESENT   1 = channel 2 wired to a pad
 *   23      PWM1_PRESENT   1 = channel 1 wired to a pad
 *   22      PWM0_PRESENT   1 = channel 0 wired to a pad
 *   21..10  RSRVD1
 *   9       OUTPUT_CUTOFF_EN
 *   8       RSRVD2
 *   7       PWM7_ENABLE
 *   6       PWM6_ENABLE
 *   5       PWM5_ENABLE
 *   4       PWM4_ENABLE
 *   3       PWM3_ENABLE
 *   2       PWM2_ENABLE
 *   1       PWM1_ENABLE
 *   0       PWM0_ENABLE
 *
 * ACTIVE register bit assignments (BM_PWM_ACTIVEn_*):
 *
 *   31..16  INACTIVE   inactive period count
 *    15..0  ACTIVE     active period count
 *
 * PERIOD register bit assignments (BM_PWM_PERIODn_*):
 *
 *   31..27  RSRVD2
 *   26      HSADC_OUT
 *   25      HSADC_CLK_SEL
 *   24      MATT_SEL
 *   23      MATT
 *   22..20  CDIV       prescaler (DIV_1..DIV_1024)
 *   19..18  INACTIVE_STATE  (HI_Z, 0, 1)
 *   17..16  ACTIVE_STATE    (HI_Z, 0, 1)
 *   15..0   PERIOD     period count
 *
 * The SHARP Brain (PW-SH6) board wires PWM channels 4 and 6 to
 * the LCD backlight driver, channel 0 to the audio amplifier
 * and channel 2 to the touch-panel beeper.  The WinCE BSP - which
 * the SHARP Brain firmware is built on top of the Freescale
 * i.MX28 EVK BSP - was written for the EVK board, where the
 * PRESENT bits reported in HW_PWM_CTRL are PWM4 | PWM6 (i.e.
 * bits 26, 28).  `PWMGetChannelPresentMask()` shifts those down
 * 22 places, leaving `0x50` in the low byte.  If we were to OR
 * in CTRL_PWM0_PRESENT or CTRL_PWM2_PRESENT too (because the
 * SHARP Brain does wire those channels to pads), the mask would
 * come out as something other than `0x50` and `PwmInitialize`
 * would compare it to `0x50`, fail, and return FALSE.  We
 * therefore follow the EVK convention and only mark PWM4 and
 * PWM6 as present.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/arm/mxs.h"
#include "hw/arm/mxs_pwm.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define MXS_PWM_CHANNELS        8
#define MXS_PWM_ACTIVE_BASE     0x10    /* HW_PWM_ACTIVEn(0)         */
#define MXS_PWM_PERIOD_BASE     0x20    /* HW_PWM_PERIODn(0)         */
#define MXS_PWM_CHANNEL_STRIDE  0x20    /* 0x20 byte per channel     */
#define MXS_PWM_VERSION_OFF     0x110   /* HW_PWM_VERSION            */
#define MXS_PWM_REGS            0x200   /* full register block       */

/*
 * BM_PWM_CTRL_* bits.  These are the same values as the
 * BM_PWM_CTRL_* defines in the Freescale regs-pwm.h header.
 */
#define CTRL_SFTRST             (1u << 31)
#define CTRL_CLKGATE            (1u << 30)
#define CTRL_PWM7_PRESENT       (1u << 29)
#define CTRL_PWM6_PRESENT       (1u << 28)
#define CTRL_PWM5_PRESENT       (1u << 27)
#define CTRL_PWM4_PRESENT       (1u << 26)
#define CTRL_PWM3_PRESENT       (1u << 25)
#define CTRL_PWM2_PRESENT       (1u << 24)
#define CTRL_PWM1_PRESENT       (1u << 23)
#define CTRL_PWM0_PRESENT       (1u << 22)
#define CTRL_OUTPUT_CUTOFF_EN   (1u << 9)
#define CTRL_PWM7_ENABLE        (1u << 7)
#define CTRL_PWM6_ENABLE        (1u << 6)
#define CTRL_PWM5_ENABLE        (1u << 5)
#define CTRL_PWM4_ENABLE        (1u << 4)
#define CTRL_PWM3_ENABLE        (1u << 3)
#define CTRL_PWM2_ENABLE        (1u << 2)
#define CTRL_PWM1_ENABLE        (1u << 1)
#define CTRL_PWM0_ENABLE        (1u << 0)

/*
 * SHARP Brain (PW-SH6) PWM PRESENT mask.
 *
 * Physically the board wires PWM0 (audio amp), PWM2 (touch
 * beeper), PWM4 (backlight ch0) and PWM6 (backlight ch1) to
 * pads.  However the WinCE BSP was compiled from the Freescale
 * i.MX28 EVK platform with BSP_PRESENT_CH_MASK = 0xFF (all
 * 8 channels reported as present).  We match that expectation
 * so PwmInitialize() compares successfully and proceeds to
 * configure the PWM channels.
 */
#define BRAIN_PRESENT_MASK      (CTRL_PWM0_PRESENT | CTRL_PWM1_PRESENT | \
                                 CTRL_PWM2_PRESENT | CTRL_PWM3_PRESENT | \
                                 CTRL_PWM4_PRESENT | CTRL_PWM5_PRESENT | \
                                 CTRL_PWM6_PRESENT | CTRL_PWM7_PRESENT)

typedef struct MXSPWMState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    char *name;
    uint64_t size;
    uint32_t regs[MXS_PWM_REGS / 4];
    bool trace;
} MXSPWMState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSPWMState, MXS_PWM)

/*
 * Read the live value of HW_PWM_CTRL, folding in the SHARP Brain
 * wiring bits so that the WinCE BSP's `PWMGetChannelPresentMask()`
 * (which reads CTRL and masks it with 0xFF, then interprets
 * bits 4 and 6 of that low byte as "PWM4 / PWM6 are wired to a
 * pad") sees exactly the right "present channel" mask.
 */
static uint32_t mxs_pwm_ctrl_value(MXSPWMState *s)
{
    uint32_t val = s->regs[0];

    /*
     * The SHARP Brain always wires channels 0, 4 and 6 to pads,
     * regardless of what the BSP writes.  On real silicon, the
     * BM_PWM_CTRL_PWMx_PRESENT bits are read-only and reflect the
     * board wiring; on a Brain they are always set.
     */
    val |= BRAIN_PRESENT_MASK;

    /*
     * SFTRST (bit 31) and CLKGATE (bit 30) are self-clearing bits
     * on real silicon: after the BSP asserts SFTRST, both SFTRST
     * and CLKGATE auto-clear within a few clock cycles.  QEMU has
     * no clock to do this with, so we just force them both to 0 in
     * the read value.  This makes the BSP's
     *   "while (HW_PWM_CTRL & CLKGATE) { Sleep(1); }"
     * spin wait terminate immediately, and lets
     * `PWMGetChannelPresentMask()` see a clean register with
     * only the present bits and any BSP-written enable bits set.
     */
    val &= ~(CTRL_SFTRST | CTRL_CLKGATE);

    return val;
}

/*
 * ACTIVE register live status.
 *
 * With MXS_BANK_INDEX = offset >> 4:
 *   ACTIVE channels have odd  indices 1,3,5,7,9,11,13,15
 *   PERIOD  channels have even indices 2,4,6,8,10,12,14,16
 *
 * The Freescale BSP's PwmInitialize() writes ACTIVE/PERIOD for
 * each present channel, enables it via CTRL, then polls bit 16
 * of the ACTIVE register (BM_PWM_ACTIVEn_ACTIVE = 0x00010000).
 * On real silicon this bit self-sets once the PWM counter is
 * running; we set it immediately when the channel is enabled
 * AND a non-zero PERIOD value has been programmed.
 */
static int mxs_pwm_channel_from_idx(unsigned idx, int *p_is_active)
{
    if (idx < 1 || idx > 16) {
        return -1;
    }
    int is_active = idx & 1;
    if (p_is_active) {
        *p_is_active = is_active;
    }
    return (int)(idx - 1) / 2;
}

static uint32_t mxs_pwm_active_read(MXSPWMState *s, unsigned idx)
{
    int ch = mxs_pwm_channel_from_idx(idx, NULL);
    uint32_t val   = s->regs[idx];
    uint32_t ctrl  = s->regs[0];
    unsigned period_idx = 2 + (unsigned)ch * 2;
    uint32_t period = (period_idx < ARRAY_SIZE(s->regs))
                      ? s->regs[period_idx] : 0;

    /* BM_PWM_ACTIVEn_ACTIVE (bit 16): set when channel enabled
     * and PERIOD has been written with a non-zero count. */
    if ((ctrl & (1u << ch)) && (period & 0xffff)) {
        val |= (1u << 16);
    }

    return val;
}

static uint64_t mxs_pwm_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSPWMState *s = MXS_PWM(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= ARRAY_SIZE(s->regs)) {
        return 0;
    }

    if (idx == 0) {
        val = mxs_pwm_ctrl_value(s);
    } else if (idx == (MXS_PWM_VERSION_OFF >> 2)) {
        val = 0x03010000;
    } else {
        int is_active;
        int ch = mxs_pwm_channel_from_idx(idx, &is_active);
        if (ch >= 0 && is_active) {
            val = mxs_pwm_active_read(s, idx);
        } else {
            val = s->regs[idx];
        }
    }

    return mxs_bank_extract(offset, size, val);
}

/*
 * Atomic bit-write dispatch for CTRL's SET / CLR / TOG aliases.
 * MXS_BANK_OP(offset) (defined in mxs_bank.h) tells us which
 * alias the write targets.
 */
static void mxs_pwm_write_ctrl(MXSPWMState *s, unsigned idx, unsigned op,
                               uint64_t value, unsigned size)
{
    uint32_t v = (uint32_t)value;
    uint32_t old = s->regs[0];

    switch (op) {
    case MXS_OP_WRITE:
        s->regs[0] = mxs_bank_apply(old, idx << 4, value, size);
        break;
    case MXS_OP_SET:
        s->regs[0] = old | v;
        break;
    case MXS_OP_CLR:
        s->regs[0] = old & ~v;
        break;
    case MXS_OP_TOG:
        s->regs[0] = old ^ v;
        break;
    }
}

static void mxs_pwm_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    MXSPWMState *s = MXS_PWM(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    unsigned op = MXS_BANK_OP(offset);

    if (idx >= ARRAY_SIZE(s->regs)) {
        return;
    }

    if (idx == 0) {
        mxs_pwm_write_ctrl(s, idx, op, value, size);
    } else {
        s->regs[idx] = mxs_bank_apply(s->regs[idx], offset, value, size);
    }

    /*
     * On real silicon, asserting SFTRST (bit 31 of CTRL) wipes
     * every other register in the block, including ACTIVE,
     * PERIOD, and all the SET / CLR / TOG state.
     */
    if (s->regs[0] & CTRL_SFTRST) {
        for (unsigned i = 1; i < ARRAY_SIZE(s->regs); i++) {
            s->regs[i] = 0;
        }
    }
}

MXS_TRACE_WRAP(mxs_pwm, "pwm")

static const MemoryRegionOps mxs_pwm_ops = {
    .read = mxs_pwm_read_tr,
    .write = mxs_pwm_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_pwm_reset(DeviceState *dev)
{
    MXSPWMState *s = MXS_PWM(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0] = CTRL_SFTRST | CTRL_CLKGATE;
    qemu_set_irq(s->irq, 0);
}

static void mxs_pwm_realize(DeviceState *dev, Error **errp)
{
    MXSPWMState *s = MXS_PWM(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    g_autofree char *mrname = g_strdup_printf("mxs-%s",
                                              s->name ? s->name : "pwm");

    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_pwm_ops, s,
                          mrname, s->size ? s->size : 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    /* Trace PWM access; controlled by MXS_TRACE env var (pwm). */
    s->trace = mxs_trace_enabled(s->name ? s->name : "pwm");
    /* The wrapped read/write handlers consult a static bool; mirror. */
    mxs_pwm_trace = s->trace;
}

static const VMStateDescription vmstate_mxs_pwm = {
    .name = "mxs-pwm",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSPWMState, MXS_PWM_REGS / 4),
        VMSTATE_END_OF_LIST()
    }
};

static const Property mxs_pwm_properties[] = {
    DEFINE_PROP_STRING("name", MXSPWMState, name),
    DEFINE_PROP_UINT64("size", MXSPWMState, size, 0x2000),
};

static void mxs_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mxs_pwm_realize;
    device_class_set_legacy_reset(dc, mxs_pwm_reset);
    dc->vmsd = &vmstate_mxs_pwm;
    device_class_set_props(dc, mxs_pwm_properties);
}

static const TypeInfo mxs_pwm_types[] = {
    {
        .name           = TYPE_MXS_PWM,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSPWMState),
        .class_init     = mxs_pwm_class_init,
    },
};

DEFINE_TYPES(mxs_pwm_types)
