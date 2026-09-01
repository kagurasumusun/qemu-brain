/*
 * Freescale i.MX28 (MXS) SoC definitions
 *
 * Used by the SHARP Brain (PW-xx / WinCE 6.0) machine model.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#ifndef HW_ARM_MXS_H
#define HW_ARM_MXS_H

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"

/* --------------------------------------------------------------------- */
/* Memory map                                                            */
/* --------------------------------------------------------------------- */
#define MXS_OCRAM_BASE      0x00000000
#define MXS_OCRAM_SIZE      0x00020000      /* 128 KiB on chip SRAM      */
#define MXS_DRAM_BASE       0x40000000
#define MXS_ROM_BASE        0xC0000000      /* on chip boot ROM          */
#define MXS_ROM_SIZE        0x00010000

#define MXS_ICOLL_BASE      0x80000000
#define MXS_HSADC_BASE      0x80002000
#define MXS_APBH_BASE       0x80004000
#define MXS_PERFMON_BASE    0x80006000
#define MXS_BCH_BASE        0x8000A000
#define MXS_GPMI_BASE       0x8000C000
#define MXS_SSP0_BASE       0x80010000
#define MXS_SSP1_BASE       0x80012000
#define MXS_SSP2_BASE       0x80014000
#define MXS_SSP3_BASE       0x80016000
#define MXS_PINCTRL_BASE    0x80018000
#define MXS_DIGCTL_BASE     0x8001C000
#define MXS_ETM_BASE        0x80022000
#define MXS_APBX_BASE       0x80024000
#define MXS_DCP_BASE        0x80028000
#define MXS_PXP_BASE        0x8002A000
#define MXS_OCOTP_BASE      0x8002C000
#define MXS_AXI_AHB0_BASE   0x8002E000
#define MXS_LCDIF_BASE      0x80030000
#define MXS_CAN0_BASE       0x80032000
#define MXS_CAN1_BASE       0x80034000
#define MXS_SIMDBG_BASE     0x8003C000
#define MXS_CLKCTRL_BASE    0x80040000
#define MXS_SAIF0_BASE      0x80042000
#define MXS_POWER_BASE      0x80044000
#define MXS_SAIF1_BASE      0x80046000
#define MXS_AUDIOOUT_BASE   0x80048000
#define MXS_AUDIOIN_BASE    0x8004C000
#define MXS_LRADC_BASE      0x80050000
#define MXS_SPDIF_BASE      0x80054000
#define MXS_RTC_BASE        0x80056000
#define MXS_I2C0_BASE       0x80058000
#define MXS_I2C1_BASE       0x8005A000
#define MXS_PWM_BASE        0x80064000
#define MXS_TIMROT_BASE     0x80068000
#define MXS_AUART0_BASE     0x8006A000
#define MXS_AUART1_BASE     0x8006C000
#define MXS_AUART2_BASE     0x8006E000
#define MXS_AUART3_BASE     0x80070000
#define MXS_AUART4_BASE     0x80072000
#define MXS_DUART_BASE      0x80074000
#define MXS_USBPHY0_BASE    0x8007C000
#define MXS_USBPHY1_BASE    0x8007E000
#define MXS_USBCTRL0_BASE   0x80080000
#define MXS_USBCTRL1_BASE   0x80090000
#define MXS_DFLPT_BASE      0x800C0000
#define MXS_EMI_BASE        0x800E0000
#define MXS_ENET_BASE       0x800F0000

/* --------------------------------------------------------------------- */
/* Interrupt numbers (ICOLL inputs)                                      */
/* --------------------------------------------------------------------- */
#define MXS_NUM_IRQS        128

#define MXS_IRQ_PXP         39
#define MXS_IRQ_LCDIF       38
#define MXS_IRQ_DUART       47
#define MXS_IRQ_TIMER0      48
#define MXS_IRQ_TIMER1      49
#define MXS_IRQ_TIMER2      50
#define MXS_IRQ_TIMER3      51
#define MXS_IRQ_RTC_1MSEC   28
#define MXS_IRQ_RTC_ALARM   29
#define MXS_IRQ_DIGCTL      89
#define MXS_IRQ_LRADC_TOUCH 10
#define MXS_IRQ_LRADC_CH0   16
#define MXS_IRQ_APBH_DMA0   82   /* SSP0 dma channel */
#define MXS_IRQ_APBH_DMA1   83
#define MXS_IRQ_APBH_DMA2   84
#define MXS_IRQ_APBH_DMA3   85
#define MXS_IRQ_USB1        92
#define MXS_IRQ_USB0        93
#define MXS_IRQ_USB1_WAKE   94
#define MXS_IRQ_USB0_WAKE   95
#define MXS_IRQ_SSP0        96
#define MXS_IRQ_SSP1        97
#define MXS_IRQ_SSP2        98
#define MXS_IRQ_SSP3        99
#define MXS_IRQ_I2C1        110
#define MXS_IRQ_I2C0        111
#define MXS_IRQ_AUART0      112
#define MXS_IRQ_GPIO4       123
#define MXS_IRQ_GPIO3       124
#define MXS_IRQ_GPIO2       125
#define MXS_IRQ_GPIO1       126
#define MXS_IRQ_GPIO0       127

/* --------------------------------------------------------------------- */
/* Device type names                                                     */
/* --------------------------------------------------------------------- */
#define TYPE_MXS_ICOLL      "mxs-icoll"
#define TYPE_MXS_TIMROT     "mxs-timrot"
#define TYPE_MXS_CLKCTRL    "mxs-clkctrl"
#define TYPE_MXS_POWER      "mxs-power"
#define TYPE_MXS_DIGCTL     "mxs-digctl"
#define TYPE_MXS_OCOTP      "mxs-ocotp"
#define TYPE_MXS_RTC        "mxs-rtc"
#define TYPE_MXS_I2C        "mxs-i2c"
#define TYPE_MXS_I2C_REAL   "mxs-i2c-real"
#define TYPE_MXS_USBPHY     "mxs-usbphy"
#define TYPE_MXS_USBCTRL    "mxs-usbctrl"
#define TYPE_MXS_DUMMY      "mxs-dummy"
#define TYPE_MXS_PINCTRL    "mxs-pinctrl"
#define TYPE_MXS_APBH       "mxs-apbh"
#define TYPE_MXS_APBX       "mxs-apbx"
#define TYPE_MXS_BCH        "mxs-bch"
#define TYPE_MXS_GPMI       "mxs-gpmi"
#define TYPE_MXS_SSP        "mxs-ssp"
#define TYPE_MXS_LCDIF      "mxs-lcdif"
int mxs_lcdif_dump_fb(DeviceState *dev, const char *path);
int mxs_lcdif_dump_fb_opt(DeviceState *dev, const char *path,
                          uint32_t base, uint32_t w, uint32_t h, int bpp);
#define TYPE_MXS_PXP        "mxs-pxp"
#define TYPE_MXS_LRADC      "mxs-lradc"

/* touch screen input (also used by the headless HMP brain_touch) */
void mxs_lradc_set_touch(DeviceState *dev, int x, int y, bool down);
#define TYPE_MXS_AUART      "mxs-auart"
#define TYPE_BRAIN_KBD      "brain-keyboard"

/* --------------------------------------------------------------------- */
/* APBH DMA <-> peripheral glue                                          */
/* --------------------------------------------------------------------- */
typedef struct MXSDmaOps {
    /* PIO words carried by a DMA descriptor, written to the peripheral */
    void (*pio)(void *opaque, const uint32_t *words, int nwords);
    /*
     * Data transfer. @to_device is true when data flows from memory to the
     * peripheral (DMA_READ command), false when the peripheral supplies
     * data (DMA_WRITE command).  Returns the number of bytes transferred.
     */
    int (*xfer)(void *opaque, uint8_t *buf, int len, bool to_device);
    /* Called when the descriptor chain of the channel has completed */
    void (*complete)(void *opaque);
    /*
     * BRAIN fault-zone experiment aid: called before the engine executes
     * a descriptor carrying PIO words (a would-be peripheral command).
     * If the peripheral wants the descriptor held (e.g. a virtual-time
     * read delay), return the absolute QEMU_CLOCK_VIRTUAL deadline (ns)
     * at which the channel should retry; the descriptor then executes
     * unchanged.  Return 0 to run it now.  NULL means never hold.
     */
    int64_t (*hold_until)(void *opaque, uint32_t cmd, uint32_t arg,
                          uint32_t ctrl0);
} MXSDmaOps;

void mxs_apbh_attach(DeviceState *dma, int channel,
                     const MXSDmaOps *ops, void *opaque);
void mxs_apbx_attach(DeviceState *dma, int channel,
                     const MXSDmaOps *ops, void *opaque);
const MXSDmaOps *mxs_saif_get_dma_ops(void);
void mxs_apbh_set_dmareq(DeviceState *dma, int channel, bool level);

/* pinctrl: raw access to the GPIO input state (used by the key matrix) */
void mxs_pinctrl_set_din(DeviceState *pinctrl, int bank, uint32_t value);
/* Update DIN/ext only.  No IRQSTAT latch, no ICOLL.  Used by the
 * matrix keyboard: live PIN2IRQ2=0 so the guest polls DIN. */
void mxs_pinctrl_set_din_poll(DeviceState *pinctrl, int bank, uint32_t value);
uint32_t mxs_pinctrl_get_ext(DeviceState *pinctrl, int bank);
uint32_t mxs_pinctrl_get_dout(DeviceState *pinctrl, int bank);
uint32_t mxs_pinctrl_get_doe(DeviceState *pinctrl, int bank);
void mxs_pinctrl_set_notify(DeviceState *pinctrl, void (*cb)(void *),
                            void *opaque);

/* SSP: DMA glue used to wire the controller to an APBH channel */
const MXSDmaOps *mxs_ssp_get_dma_ops(void);

/* keyboard: needs to know which pinctrl it is wired to */
void brain_kbd_set_pinctrl(DeviceState *kbd, DeviceState *pinctrl);

#endif /* HW_ARM_MXS_H */

/* EDNA2 MCU touchkey report word (mailbox +0x404) */
void brain_kbd_set_touchkey_state(DeviceState *kbd, uint32_t *state_ptr,
                                 uint32_t *mb_word);
void brain_kbd_edna2_pulse_ext(DeviceState *kbd);
