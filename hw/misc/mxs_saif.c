/*
 * Freescale i.MX28 (MXS) SAIF (Serial Audio Interface).
 *
 * Full register-level model of the SAIF block as described by the
 * i.MX28 reference manual and the Linux BSP header (sound/soc/mxs/
 * mxs-saif.h): HW_SAIF_CTRL / HW_SAIF_STAT / HW_SAIF_DATA /
 * HW_SAIF_VERSION with the MXS set/clear/toggle register bank.
 *
 * Behaviour model
 * ---------------
 * The block has one 8 entry x 32 bit FIFO which is used either as a
 * transmit FIFO (CTRL.READ_MODE = 0: the CPU/DMA writes sample pairs to
 * HW_SAIF_DATA and the serial engine drains them) or as a receive FIFO
 * (READ_MODE = 1: the engine fills it and the CPU/DMA reads it back).
 *
 * While CTRL.RUN is set and CLKGATE is clear a virtual bit-clock timer
 * moves one stereo frame per tick (48 kHz nominal).  This gives the
 * guest-visible dynamics of the real block:
 *
 *  - STAT.BUSY              set while the engine is running
 *  - STAT.DMA_PREQ          set while the FIFO wants DMA service
 *                           (TX: level below half, RX: level above half)
 *  - STAT.FIFO_SERVICE_IRQ  level status: the FIFO needs service
 *                           (TX/READ_MODE=0: level <= half, i.e. wants
 *                           samples; RX/READ_MODE=1: level >= half, i.e.
 *                           wants draining).  An empty transmit FIFO
 *                           therefore asserts it immediately, which is
 *                           what EBOOT's BeepSound poll relies on.
 *                           Raises the ICOLL line if
 *                           CTRL.FIFO_SERVICE_IRQ_EN
 *  - STAT.FIFO_UNDERFLOW_IRQ / STAT.FIFO_OVERFLOW_IRQ
 *                           set on FIFO underrun/overrun; raise the line if
 *                           CTRL.FIFO_ERROR_IRQ_EN; all three STAT IRQ bits
 *                           are write-1-to-clear via the _CLR aliases
 *
 * The interrupt output is a single line per SAIF (SAIF0 = ICOLL source 59,
 * SAIF1 = source 58, per the i.MX28 interrupt table); it is asserted
 * exactly while a latched, enabled STAT IRQ bit is present.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/arm/mxs.h"
#include "hw/arm/mxs_saif.h"
#include "hw/audio/sgtl5000.h"
#include "hw/audio/bu26154.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define SAIF_CTRL       0x00
#define SAIF_STAT       0x10
#define SAIF_DATA       0x20
#define SAIF_VERSION    0x30

/* HW_SAIF_CTRL */
#define CTRL_SFTRST              (1u << 31)
#define CTRL_CLKGATE             (1u << 30)
#define CTRL_BITCLK_MULT_RATE    (0x7u << 27)
#define CTRL_BITCLK_BASE_RATE    (1u << 26)
#define CTRL_FIFO_ERROR_IRQ_EN   (1u << 25)
#define CTRL_FIFO_SERVICE_IRQ_EN (1u << 24)
#define CTRL_DMAWAIT_COUNT       (0x1fu << 16)
#define CTRL_CHANNEL_NUM_SELECT  (0x3u << 14)
#define CTRL_LRCLK_PULSE         (1u << 13)
#define CTRL_BIT_ORDER           (1u << 12)
#define CTRL_DELAY               (1u << 11)
#define CTRL_JUSTIFY             (1u << 10)
#define CTRL_LRCLK_POLARITY      (1u << 9)
#define CTRL_BITCLK_EDGE         (1u << 8)
#define CTRL_WORD_LENGTH         (0xfu << 4)
#define CTRL_BITCLK_48XFS_ENABLE (1u << 3)
#define CTRL_SLAVE_MODE          (1u << 2)
#define CTRL_READ_MODE           (1u << 1)
#define CTRL_RUN                 (1u << 0)

/* HW_SAIF_STAT */
#define STAT_PRESENT             (1u << 31)
#define STAT_DMA_PREQ            (1u << 16)
#define STAT_FIFO_UNDERFLOW_IRQ  (1u << 6)
#define STAT_FIFO_OVERFLOW_IRQ   (1u << 5)
#define STAT_FIFO_SERVICE_IRQ    (1u << 4)
#define STAT_BUSY                (1u << 0)

#define MXS_MAX_REGS 64
#define SAIF_FIFO_LEN 8

/* one stereo frame per tick at 48 kHz */
#define SAIF_FRAME_PERIOD_NS (1000000000ull / 48000)

typedef struct MXSSAIFState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[MXS_MAX_REGS];
    char *name;
    uint64_t size;
    bool trace;

    uint32_t fifo[SAIF_FIFO_LEN];
    unsigned fifo_len;
    bool engine_on;

    /*
     * Board-level codec links (see mxs_saif_set_codec).  Playback SAIF
     * (saif0) drains its TX FIFO into the codec DAC; capture SAIF (saif1)
     * fills its RX FIFO from the codec ADC.
     */
    DeviceState *dac_codec;
    DeviceState *adc_codec;
} MXSSAIFState;

#define TYPE_MXS_SAIF "mxs-saif"
OBJECT_DECLARE_SIMPLE_TYPE(MXSSAIFState, MXS_SAIF)

static uint32_t saif_ctrl(MXSSAIFState *s) { return s->regs[SAIF_CTRL >> 4]; }

static bool saif_running(MXSSAIFState *s)
{
    uint32_t c = saif_ctrl(s);

    return (c & CTRL_RUN) && !(c & CTRL_CLKGATE) && !(c & CTRL_SFTRST);
}

static void saif_update_irq(MXSSAIFState *s)
{
    uint32_t c = saif_ctrl(s);
    uint32_t st = s->regs[SAIF_STAT >> 4];
    bool level = false;

    if (st & STAT_FIFO_SERVICE_IRQ && (c & CTRL_FIFO_SERVICE_IRQ_EN)) {
        level = true;
    }
    if ((st & (STAT_FIFO_UNDERFLOW_IRQ | STAT_FIFO_OVERFLOW_IRQ)) &&
        (c & CTRL_FIFO_ERROR_IRQ_EN)) {
        level = true;
    }
    qemu_set_irq(s->irq, level);
}

static void saif_recompute_stat(MXSSAIFState *s)
{
    uint32_t c = saif_ctrl(s);
    uint32_t st = s->regs[SAIF_STAT >> 4];

    st &= ~(STAT_PRESENT | STAT_BUSY | STAT_DMA_PREQ |
            STAT_FIFO_SERVICE_IRQ);
    st |= STAT_PRESENT;
    /*
     * FIFO_SERVICE_IRQ is a level condition: the FIFO needs service.
     * TX: level at/below half wants samples (an empty FIFO asserts it
     * immediately); RX: level at/above half wants draining.
     */
    if (c & CTRL_READ_MODE) {
        if (s->fifo_len >= SAIF_FIFO_LEN / 2) {
            st |= STAT_FIFO_SERVICE_IRQ;
        }
    } else {
        if (s->fifo_len <= SAIF_FIFO_LEN / 2) {
            st |= STAT_FIFO_SERVICE_IRQ;
        }
    }
    if (saif_running(s)) {
        st |= STAT_BUSY;
        if (c & CTRL_READ_MODE) {
            /* capture: DMA should drain the FIFO once past half */
            if (s->fifo_len > SAIF_FIFO_LEN / 2) {
                st |= STAT_DMA_PREQ;
            }
        } else {
            /* playback: DMA should refill the FIFO while below half */
            if (s->fifo_len < SAIF_FIFO_LEN / 2) {
                st |= STAT_DMA_PREQ;
            }
        }
    }
    s->regs[SAIF_STAT >> 4] = st;
    saif_update_irq(s);
}

/*
 * Board codec frame plumbing.  The machine can wire either codec on
 * I2C0 (Freescale SGTL5000 from the pre-S97 Linux-DTS wiring, or the
 * real LAPIS/ROHM BU26154 used by wavedev2_BU26154.dll); dispatch the
 * frame exchange on the device type.
 */
static uint32_t saif_codec_adc_output(MXSSAIFState *s)
{
    DeviceState *cd = s->adc_codec;

    if (!cd) {
        return 0;
    }
    if (object_dynamic_cast(OBJECT(cd), TYPE_SGTL5000)) {
        return sgtl5000_adc_output(SGTL5000(cd));
    }
    if (object_dynamic_cast(OBJECT(cd), TYPE_BU26154)) {
        return bu26154_adc_output(BU26154(cd));
    }
    return 0;
}

static void saif_codec_dac_input(MXSSAIFState *s, uint32_t frame)
{
    DeviceState *cd = s->dac_codec;

    if (!cd) {
        return;
    }
    if (object_dynamic_cast(OBJECT(cd), TYPE_SGTL5000)) {
        sgtl5000_dac_input(SGTL5000(cd), frame);
    } else if (object_dynamic_cast(OBJECT(cd), TYPE_BU26154)) {
        bu26154_dac_input(BU26154(cd), frame);
    }
}

static void saif_rx_frame(MXSSAIFState *s)
{
    /*
     * Capture: the serial engine clocks one frame in from the codec ADC
     * (real host capture when a codec is linked; silence when the board
     * has no capture link or no audio backend).  Overflow sets the IRQ
     * bit and drops the frame.
     */
    uint32_t st = s->regs[SAIF_STAT >> 4];

    if (s->fifo_len < SAIF_FIFO_LEN) {
        uint32_t frame = 0;

        if (s->adc_codec) {
            frame = saif_codec_adc_output(s);
        }
        s->fifo[(0 + s->fifo_len) % SAIF_FIFO_LEN] = frame;
        s->fifo_len++;
    } else {
        st |= STAT_FIFO_OVERFLOW_IRQ;
    }
    s->regs[SAIF_STAT >> 4] = st;
}

static void saif_timer(void *opaque)
{
    MXSSAIFState *s = opaque;
    uint32_t st;

    if (!saif_running(s)) {
        s->engine_on = false;
        saif_recompute_stat(s);
        return;
    }

    st = s->regs[SAIF_STAT >> 4];
    if (saif_ctrl(s) & CTRL_READ_MODE) {
        saif_rx_frame(s);
    } else {
        /* Playback: the engine consumes one frame from the TX FIFO. */
        uint32_t frame = 0;

        if (s->fifo_len > 0) {
            frame = s->fifo[0];
            memmove(&s->fifo[0], &s->fifo[1],
                    (s->fifo_len - 1) * sizeof(uint32_t));
            s->fifo_len--;
        } else {
            st |= STAT_FIFO_UNDERFLOW_IRQ;
        }
        if (s->dac_codec) {
            saif_codec_dac_input(s, frame);
        }
    }
    s->regs[SAIF_STAT >> 4] = st;
    saif_recompute_stat(s);

    timer_mod(s->timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SAIF_FRAME_PERIOD_NS);
}

static void saif_engine_sync(MXSSAIFState *s)
{
    bool want = saif_running(s);

    if (want && !s->engine_on) {
        s->engine_on = true;
        timer_mod(s->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  SAIF_FRAME_PERIOD_NS);
    } else if (!want && s->engine_on) {
        timer_del(s->timer);
        s->engine_on = false;
        saif_recompute_stat(s);
    }
}

static uint64_t mxs_saif_read(void *opaque, hwaddr off, unsigned size)
{
    MXSSAIFState *s = MXS_SAIF(opaque);
    unsigned idx = MXS_BANK_INDEX(off);
    uint32_t v = 0;

    switch (idx) {
    case SAIF_CTRL >> 4:
        v = s->regs[SAIF_CTRL >> 4];
        break;
    case SAIF_STAT >> 4:
        v = s->regs[SAIF_STAT >> 4];
        break;
    case SAIF_DATA >> 4:
        if (s->fifo_len > 0) {
            /* pop from the head and return the frame actually consumed */
            v = s->fifo[0];
            memmove(&s->fifo[0], &s->fifo[1],
                    (s->fifo_len - 1) * sizeof(uint32_t));
            s->fifo_len--;
        } else {
            s->regs[SAIF_STAT >> 4] |= STAT_FIFO_UNDERFLOW_IRQ;
            v = 0;
        }
        saif_recompute_stat(s);
        saif_engine_sync(s);
        break;
    case SAIF_VERSION >> 4:
        v = 0x02010000;
        break;
    default:
        if (idx < MXS_MAX_REGS) {
            v = s->regs[idx];
        }
        break;
    }
    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, false, off, v);
    }
    return mxs_bank_extract(off, size, v);
}

static void mxs_saif_write(void *opaque, hwaddr off, uint64_t value,
                           unsigned size)
{
    MXSSAIFState *s = MXS_SAIF(opaque);
    unsigned idx = MXS_BANK_INDEX(off);

    if (idx >= MXS_MAX_REGS) {
        return;
    }
    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, true, off, (uint32_t)value);
    }

    if (idx == SAIF_DATA >> 4) {
        /* pushing a sample pair into the transmit FIFO */
        bool is_clr = (off & 0xf00) == 0x400; /* SET/CLR alias of DATA unused */

        (void)is_clr;
        if (s->fifo_len < SAIF_FIFO_LEN) {
            s->fifo[s->fifo_len++] = (uint32_t)value;
        } else {
            s->regs[SAIF_STAT >> 4] |= STAT_FIFO_OVERFLOW_IRQ;
        }
        saif_recompute_stat(s);
        saif_engine_sync(s);
        return;
    }

    uint32_t old = s->regs[idx];
    uint32_t val = mxs_bank_apply(old, off, value, size);

    if (idx == SAIF_CTRL >> 4) {
        val = mxs_bank_sftrst(old, val);
        if ((old & CTRL_SFTRST) && !(val & CTRL_SFTRST)) {
            /* coming out of software reset: engine and FIFO are clean */
            s->fifo_len = 0;
        }
        /* RUN/CLKGATE changes take effect immediately */
        s->regs[idx] = val;
        saif_engine_sync(s);
        saif_recompute_stat(s);
        return;
    }
    if (idx == SAIF_STAT >> 4) {
        /* only the W1C IRQ bits are writable by the guest */
        uint32_t w1c = (uint32_t)value &
                       (STAT_FIFO_UNDERFLOW_IRQ | STAT_FIFO_OVERFLOW_IRQ |
                        STAT_FIFO_SERVICE_IRQ);
        bool is_clr = ((off >> 2) & 3) == 1;   /* _CLR alias */
        bool is_set = ((off >> 2) & 3) == 2;   /* _SET alias */

        if (is_clr || !is_set) {
            s->regs[idx] &= ~w1c;
        } else {
            s->regs[idx] |= w1c;
        }
        saif_recompute_stat(s);
        return;
    }
    s->regs[idx] = val;
}

/* ---- DMA engine interface (APBX channels 4/5) ---- */

static int saif_dma_xfer(void *opaque, uint8_t *buf, int len, bool is_write)
{
    MXSSAIFState *s = opaque;
    int i;

    for (i = 0; i + 3 < len + 3; i += 4) {
        if (is_write) {
            /* memory -> SAIF FIFO */
            if (s->fifo_len < SAIF_FIFO_LEN) {
                uint32_t v = (uint32_t)buf[i] | ((uint32_t)buf[i + 1] << 8) |
                             ((uint32_t)buf[i + 2] << 16) |
                             ((uint32_t)buf[i + 3] << 24);
                s->fifo[s->fifo_len++] = v;
            } else {
                s->regs[SAIF_STAT >> 4] |= STAT_FIFO_OVERFLOW_IRQ;
                break;
            }
        } else {
            /* SAIF FIFO -> memory */
            uint32_t v = 0;
            if (s->fifo_len > 0) {
                v = s->fifo[0];
                memmove(&s->fifo[0], &s->fifo[1],
                        (s->fifo_len - 1) * sizeof(uint32_t));
                s->fifo_len--;
            } else {
                s->regs[SAIF_STAT >> 4] |= STAT_FIFO_UNDERFLOW_IRQ;
            }
            if (i + 3 < len) {
                buf[i] = v & 0xff;
                buf[i + 1] = (v >> 8) & 0xff;
                buf[i + 2] = (v >> 16) & 0xff;
                buf[i + 3] = (v >> 24) & 0xff;
            }
        }
    }
    saif_recompute_stat(s);
    saif_engine_sync(s);
    return len;
}

static const MXSDmaOps mxs_saif_dma_ops = {
    .xfer = saif_dma_xfer,
};

const MXSDmaOps *mxs_saif_get_dma_ops(void)
{
    return &mxs_saif_dma_ops;
}

void mxs_saif_set_codec(DeviceState *saif, DeviceState *codec, bool playback)
{
    MXSSAIFState *s = MXS_SAIF(saif);

    if (playback) {
        s->dac_codec = codec;
    } else {
        s->adc_codec = codec;
    }
}

unsigned mxs_saif_pump_capture(DeviceState *saif, unsigned nframes)
{
    MXSSAIFState *s = MXS_SAIF(saif);
    unsigned n;

    if (!(saif_ctrl(s) & CTRL_READ_MODE)) {
        return 0;   /* not in capture mode */
    }
    for (n = 0; n < nframes; n++) {
        if (s->fifo_len >= SAIF_FIFO_LEN) {
            saif_rx_frame(s);   /* records overflow, pushes nothing */
            break;
        }
        saif_rx_frame(s);
        saif_recompute_stat(s);
    }
    return n;
}

MXS_TRACE_WRAP(mxs_saif, "saif")

static const MemoryRegionOps mxs_saif_ops = {
    .read = mxs_saif_read_tr, .write = mxs_saif_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_saif_reset(DeviceState *d)
{
    MXSSAIFState *s = MXS_SAIF(d);

    memset(s->regs, 0, sizeof(s->regs));
    s->fifo_len = 0;
    timer_del(s->timer);
    s->engine_on = false;
    qemu_set_irq(s->irq, 0);
}

static void mxs_saif_realize(DeviceState *d, Error **e)
{
    MXSSAIFState *s = MXS_SAIF(d);
    SysBusDevice *sbd = SYS_BUS_DEVICE(d);
    g_autofree char *n = g_strdup_printf("mxs-%s", s->name ? s->name : "saif");

    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_saif_ops, s, n,
                          s->size ? s->size : 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, saif_timer, s);
    s->trace = mxs_trace_enabled(s->name ? s->name : "saif");
}

static const Property mxs_saif_props[] = {
    DEFINE_PROP_STRING("name", MXSSAIFState, name),
    DEFINE_PROP_UINT64("size", MXSSAIFState, size, 0x2000),
};

static int mxs_saif_post_load(void *opaque, int version_id)
{
    MXSSAIFState *s = opaque;

    saif_engine_sync(s);
    return 0;
}

static const VMStateDescription vmstate_mxs_saif = {
    .name = "mxs-saif", .version_id = 2, .minimum_version_id = 2,
    .post_load = mxs_saif_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSSAIFState, MXS_MAX_REGS),
        VMSTATE_UINT32_ARRAY(fifo, MXSSAIFState, SAIF_FIFO_LEN),
        VMSTATE_UINT32(fifo_len, MXSSAIFState),
        VMSTATE_END_OF_LIST()
    },
};

bool mxs_saif_push_playback(DeviceState *saif, uint32_t frame)
{
    MXSSAIFState *s = MXS_SAIF(saif);

    if (!s->dac_codec) {
        return false;   /* no board codec linked on this SAIF */
    }
    /*
     * Same action the serial engine performs for one TX frame: the word
     * leaves the TX FIFO and is clocked into the codec.  The FIFO is not
     * touched here because the caller supplies the word directly, which
     * keeps the aid usable while the guest has the engine stopped.
     */
    saif_codec_dac_input(s, frame);
    return true;
}

static void mxs_saif_class_init(ObjectClass *k, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(k);

    dc->realize = mxs_saif_realize;
    device_class_set_legacy_reset(dc, mxs_saif_reset);
    dc->vmsd = &vmstate_mxs_saif;
    device_class_set_props(dc, mxs_saif_props);
}

static const TypeInfo mxs_saif_info[] = {
{
    .name = TYPE_MXS_SAIF, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MXSSAIFState),
    .class_init = mxs_saif_class_init,
}
};
DEFINE_TYPES(mxs_saif_info)
