/*
 * Freescale SGTL5000 ultra-low-power stereo audio codec
 *
 * I2C slave at address 0x0a, 16-bit register address + 16-bit data
 * (big-endian bytes on the wire, exactly as the Linux sgtl5000 regmap
 * with reg_bits = 16 / val_bits = 16 drives it).
 *
 * Register map, power-on reset values and field semantics follow the
 * SGTL5000 datasheet as exercised by the mainline Linux driver
 * (sound/soc/codecs/sgtl5000.[ch]) -- register *addresses/values* are
 * hardware facts and are reproduced here for the emulation; no driver
 * code is copied.
 *
 * Audio gates honoured by this model (playback):
 *   CHIP_ANA_POWER.DAC_POWERUP     analog DAC powered
 *   CHIP_DIG_POWER.DAC_EN          digital DAC path enabled
 *   CHIP_DIG_POWER.I2S_IN_POWERUP  serial data-in interface on
 *   CHIP_ADCDAC_CTRL DAC_MUTE_L/R  left/right mute bits
 * and (capture):
 *   CHIP_ANA_POWER.ADC_POWERUP     analog ADC powered
 *   CHIP_DIG_POWER.ADC_EN          digital ADC path enabled
 *   CHIP_DIG_POWER.I2S_OUT_POWERUP serial data-out interface on
 *
 * Volume: CHIP_DAC_VOL is 0.5 dB/step attenuation from 0 dB (0x3c) to
 * about -90 dB (0xfc), applied to the PCM stream.  CHIP_MIC_CTRL gain
 * (0/20/30/40 dB) is applied to samples captured from the audio
 * backend (host microphone) before they reach the SAIF.
 *
 * The DAC data path ends at the QEMU audio backend (audiodev property,
 * standard QEMU 11 device audio API); the ADC data path starts there
 * and feeds the SAIF RX FIFO, i.e. guest wave-in is a real capture of
 * the host audio input, not synthesised silence.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "hw/audio/sgtl5000.h"
#include "qemu/audio.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qom/object.h"

/* ---- register addresses (hardware) ---- */
#define SGTL_CHIP_ID            0x0000
#define SGTL_CHIP_DIG_POWER     0x0002
#define SGTL_CHIP_CLK_CTRL      0x0004
#define SGTL_CHIP_I2S_CTRL      0x0006
#define SGTL_CHIP_SSS_CTRL      0x000a
#define SGTL_CHIP_ADCDAC_CTRL   0x000e
#define SGTL_CHIP_DAC_VOL       0x0010
#define SGTL_CHIP_PAD_STRENGTH  0x0014
#define SGTL_CHIP_ANA_ADC_CTRL  0x0020
#define SGTL_CHIP_ANA_HP_CTRL   0x0022
#define SGTL_CHIP_ANA_CTRL      0x0024
#define SGTL_CHIP_LINREG_CTRL   0x0026
#define SGTL_CHIP_REF_CTRL      0x0028
#define SGTL_CHIP_MIC_CTRL      0x002a
#define SGTL_CHIP_LINE_OUT_CTRL 0x002c
#define SGTL_CHIP_LINE_OUT_VOL  0x002e
#define SGTL_CHIP_ANA_POWER     0x0030
#define SGTL_CHIP_PLL_CTRL      0x0032
#define SGTL_CHIP_CLK_TOP_CTRL  0x0034
#define SGTL_CHIP_ANA_STATUS    0x0036
#define SGTL_CHIP_SHORT_CTRL    0x003c
#define SGTL_DAP_CTRL           0x0100
#define SGTL_MAX_REG            0x013a

/* CHIP_DIG_POWER */
#define SGTL_ADC_EN         0x0040
#define SGTL_DAC_EN         0x0020
#define SGTL_I2S_OUT_PWR    0x0002
#define SGTL_I2S_IN_PWR     0x0001
/* CHIP_ADCDAC_CTRL mute bits */
#define SGTL_DAC_MUTE_R     0x0008
#define SGTL_DAC_MUTE_L     0x0004
/* CHIP_ANA_POWER */
#define SGTL_DAC_POWERUP    0x0008
#define SGTL_ADC_POWERUP    0x0002
/* CHIP_MIC_CTRL */
#define SGTL_MIC_GAIN_MASK  0x0003

#define SGTL_REGCNT ((SGTL_MAX_REG >> 1) + 1)
#define SGTL_RING   65536

struct SGTL5000State {
    I2CSlave parent_obj;

    AudioBackend *audio_be;
    SWVoiceOut *dac_voice;
    SWVoiceIn  *adc_voice;

    /* register file: regs[reg >> 1] */
    uint16_t regs[SGTL_REGCNT];

    /* I2C session */
    uint8_t  i2c_in[4];       /* address hi/lo + data hi/lo */
    uint8_t  i2c_in_n;
    uint16_t cur_reg;         /* register selected by last write */
    uint8_t  read_byte;       /* 0 = high byte, 1 = low byte */

    /* playback staging -> audio backend */
    uint8_t outbuf[SGTL_RING];
    unsigned out_start, out_len;
    bool    dac_on;

    /* capture staging <- audio backend */
    uint8_t inbuf[SGTL_RING];
    unsigned in_start, in_len;
    bool    adc_on;

    /* gain tables (built once) */
    uint32_t dac_gain[0xc1];      /* attenuation index 0..192 (0.5 dB) */
    uint32_t mic_gain[4];         /* 0/20/30/40 dB */

    SGTL5000Stats stats;

    /* debug aid (BRAIN_SGTL5000_DEBUG=1): periodic stderr stats */
    bool dbg;
    int64_t last_dbg_ns;
};

#define SGTL5000_DBG(s, ...)                                          \
    do {                                                              \
        if ((s)->dbg) {                                               \
            fprintf(stderr, "[sgtl5000] " __VA_ARGS__);               \
        }                                                             \
    } while (0)

static bool sgtl5000_debug(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("BRAIN_SGTL5000_DEBUG");
        on = e && *e && *e != '0';
    }
    return on;
}

static uint16_t sgtl_get(SGTL5000State *s, uint16_t reg)
{
    return s->regs[reg >> 1];
}

static void sgtl_put(SGTL5000State *s, uint16_t reg, uint16_t v)
{
    s->regs[reg >> 1] = v;
}

static bool sgtl_dac_on(SGTL5000State *s)
{
    uint16_t dig = sgtl_get(s, SGTL_CHIP_DIG_POWER);
    uint16_t ana = sgtl_get(s, SGTL_CHIP_ANA_POWER);
    uint16_t mute = sgtl_get(s, SGTL_CHIP_ADCDAC_CTRL);

    return (dig & SGTL_DAC_EN) && (dig & SGTL_I2S_IN_PWR) &&
           (ana & SGTL_DAC_POWERUP) &&
           !(mute & (SGTL_DAC_MUTE_L | SGTL_DAC_MUTE_R));
}

static bool sgtl_adc_on(SGTL5000State *s)
{
    uint16_t dig = sgtl_get(s, SGTL_CHIP_DIG_POWER);
    uint16_t ana = sgtl_get(s, SGTL_CHIP_ANA_POWER);

    return (dig & SGTL_ADC_EN) && (dig & SGTL_I2S_OUT_PWR) &&
           (ana & SGTL_ADC_POWERUP);
}

/* ---- audio backend helpers ---- */

static void sgtl5000_out_cb(void *opaque, int free_b)
{
    SGTL5000State *s = opaque;

    if (!s->audio_be || !s->dac_voice) {
        return;   /* no backend: register-only model, silent sink */
    }
    if (!s->dac_on || !s->out_len) {
        return;
    }
    size_t done = 0;
    while (done < s->out_len) {
        size_t chunk = MIN(s->out_len - done, SGTL_RING - s->out_start);
        size_t n = audio_be_write(s->audio_be, s->dac_voice,
                                  s->outbuf + s->out_start, chunk);
        if (!n) {
            break;   /* backend busy; callback will come again */
        }
        s->out_start = (s->out_start + n) % SGTL_RING;
        s->out_len -= n;
        done += n;
        s->stats.dac_out_frames += n / 4;
    }
}

static void sgtl5000_in_cb(void *opaque, int avail_b)
{
    SGTL5000State *s = opaque;

    if (!s->audio_be || !s->adc_voice) {
        return;   /* no backend: register-only model, silent sink */
    }
    if (!s->adc_on || avail_b <= 0) {
        return;
    }
    uint8_t tmp[4096];
    while (avail_b > 0 && s->in_len < SGTL_RING) {
        size_t want = MIN((size_t)avail_b, sizeof(tmp));
        want = MIN(want, SGTL_RING - s->in_len);
        size_t got = audio_be_read(s->audio_be, s->adc_voice, tmp, want);
        if (!got) {
            break;
        }
        for (size_t i = 0; i < got; i++) {
            unsigned wp = (s->in_start + s->in_len) % SGTL_RING;
            s->inbuf[wp] = tmp[i];
        }
        s->in_len += got;
        s->stats.adc_in_bytes += got;
        avail_b -= got;
    }
}

/* ---- SAIF <-> codec frame plumbing ---- */

void sgtl5000_dac_input(SGTL5000State *s, uint32_t frame)
{
    int16_t l = (int16_t)(frame >> 16);
    int16_t r = (int16_t)(frame & 0xffff);

    s->stats.dac_in_frames++;

    if (!s->dac_on) {
        s->stats.dac_dropped++;
        return;
    }

    /* DAC volume: 0.5 dB per step below 0 dB (reg 0x3c = 0 dB). */
    uint16_t vol = sgtl_get(s, SGTL_CHIP_DAC_VOL);
    unsigned idx_l = MIN((unsigned)(vol & 0xff) - 0x3c, 0xc0);
    unsigned idx_r = MIN((unsigned)((vol >> 8) & 0xff) - 0x3c, 0xc0);
    int32_t gl = ((int32_t)l * (int32_t)s->dac_gain[idx_l]) >> 15;
    int32_t gr = ((int32_t)r * (int32_t)s->dac_gain[idx_r]) >> 15;

    gl = MAX(-32768, MIN(32767, gl));
    gr = MAX(-32768, MIN(32767, gr));

    if (s->out_len + 4 > SGTL_RING) {
        s->stats.dac_dropped++;   /* codec FIFO overrun: drop frame */
        return;
    }
    unsigned wp = (s->out_start + s->out_len) % SGTL_RING;
    s->outbuf[wp] = gl & 0xff;
    s->outbuf[(wp + 1) % SGTL_RING] = (gl >> 8) & 0xff;
    s->outbuf[(wp + 2) % SGTL_RING] = gr & 0xff;
    s->outbuf[(wp + 3) % SGTL_RING] = (gr >> 8) & 0xff;
    s->out_len += 4;

    if (s->dac_on) {
        sgtl5000_out_cb(s, 0);
    }

    if (s->dbg) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (now - s->last_dbg_ns > 1000000000ll) {
            s->last_dbg_ns = now;
            SGTL5000_DBG(s,
                "dac_in=%" PRIu64 " out=%" PRIu64 " drop=%" PRIu64
                " adc_out=%" PRIu64 " inB=%" PRIu64 " und=%" PRIu64
                " dig=0x%04x ana=0x%04x vol=0x%04x\n",
                s->stats.dac_in_frames, s->stats.dac_out_frames,
                s->stats.dac_dropped, s->stats.adc_out_frames,
                s->stats.adc_in_bytes, s->stats.adc_underrun,
                sgtl_get(s, SGTL_CHIP_DIG_POWER),
                sgtl_get(s, SGTL_CHIP_ANA_POWER),
                sgtl_get(s, SGTL_CHIP_DAC_VOL));
        }
    }
}

uint32_t sgtl5000_adc_output(SGTL5000State *s)
{
    int16_t l = 0, r = 0;

    s->stats.adc_out_frames++;
    if (!sgtl_adc_on(s)) {
        /*
         * The codec's ADC/I2S-out section is powered down: in real
         * hardware its I2S output is inactive, so the capture SAIF
         * clocks in silence.  The ring may hold host samples, but they
         * must not leak out while the ADC is off.
         */
        s->stats.adc_gated++;
        return 0;
    }
    if (s->in_len >= 4) {
        uint8_t b0 = s->inbuf[s->in_start];
        uint8_t b1 = s->inbuf[(s->in_start + 1) % SGTL_RING];
        uint8_t b2 = s->inbuf[(s->in_start + 2) % SGTL_RING];
        uint8_t b3 = s->inbuf[(s->in_start + 3) % SGTL_RING];
        l = (int16_t)(b0 | (b1 << 8));
        r = (int16_t)(b2 | (b3 << 8));
        s->in_start = (s->in_start + 4) % SGTL_RING;
        s->in_len -= 4;

        /* MIC/analog input gain (0/20/30/40 dB). */
        uint16_t mic = sgtl_get(s, SGTL_CHIP_MIC_CTRL) & SGTL_MIC_GAIN_MASK;
        uint32_t g = s->mic_gain[mic & 3];
        int64_t gl64 = ((int64_t)l * g) >> 15;
        int64_t gr64 = ((int64_t)r * g) >> 15;
        l = MIN(32767, MAX(-32768, (int32_t)gl64));
        r = MIN(32767, MAX(-32768, (int32_t)gr64));
    } else {
        s->stats.adc_underrun++;   /* no host samples yet: silence */
    }
    return ((uint32_t)(uint16_t)l << 16) | (uint16_t)r;
}

void sgtl5000_get_stats(SGTL5000State *s, SGTL5000Stats *st)
{
    *st = s->stats;
    st->adc_pending = s->in_len;
}

unsigned sgtl5000_debug_fill(SGTL5000State *s, uint32_t seed,
                             uint32_t nframes)
{
    /*
     * Stand in for the host microphone: write a deterministic stereo
     * S16LE pattern (left ramps by seed, right by seed + 3 * i) over
     * the whole ADC capture ring so the codec -> SAIF capture path can
     * be verified sample-by-sample without host audio hardware.
     */
    unsigned cap = SGTL_RING / 4;
    unsigned n = MIN(nframes, cap);

    for (unsigned i = 0; i < n; i++) {
        uint32_t l = 0x2000 + ((seed + i) & 0x1fff);
        uint32_t r = 0x2000 + ((seed + (i * 3u)) & 0x1fff);
        unsigned wp = (i * 4) % SGTL_RING;

        s->inbuf[wp] = l & 0xff;
        s->inbuf[(wp + 1) % SGTL_RING] = (l >> 8) & 0xff;
        s->inbuf[(wp + 2) % SGTL_RING] = r & 0xff;
        s->inbuf[(wp + 3) % SGTL_RING] = (r >> 8) & 0xff;
    }
    s->in_start = 0;
    s->in_len = n * 4;
    return n;
}

uint16_t sgtl5000_get_reg(SGTL5000State *s, uint16_t reg)
{
    if (reg == SGTL_CHIP_ID) {
        return 0xa000;
    }
    if (reg > SGTL_MAX_REG) {
        return 0;
    }
    return sgtl_get(s, reg);
}

/* ---- register effects ---- */

static void sgtl5000_update_paths(SGTL5000State *s)
{
    bool dac = sgtl_dac_on(s);
    bool adc = sgtl_adc_on(s);

    if (dac != s->dac_on) {
        s->dac_on = dac;
        if (!dac) {
            s->out_start = s->out_len = 0;   /* path off: discard staged */
        }
        if (s->audio_be && s->dac_voice) {
            audio_be_set_active_out(s->audio_be, s->dac_voice, dac);
        }
        SGTL5000_DBG(s, "DAC path %s\n", dac ? "on" : "off");
    }
    if (adc != s->adc_on) {
        s->adc_on = adc;
        if (!adc) {
            s->in_start = s->in_len = 0;
        }
        if (s->audio_be && s->adc_voice) {
            audio_be_set_active_in(s->audio_be, s->adc_voice, adc);
        }
        SGTL5000_DBG(s, "ADC path %s\n", adc ? "on" : "off");
    }
}

static void sgtl5000_reg_write(SGTL5000State *s, uint16_t reg, uint16_t v)
{
    if (reg == SGTL_CHIP_ID) {
        return;   /* read-only */
    }
    sgtl_put(s, reg, v);
    switch (reg) {
    case SGTL_CHIP_DIG_POWER:
    case SGTL_CHIP_ANA_POWER:
    case SGTL_CHIP_ADCDAC_CTRL:
        sgtl5000_update_paths(s);
        break;
    default:
        break;
    }
}

/* ---- I2C slave interface ---- */

static int sgtl5000_i2c_send(I2CSlave *i2c, uint8_t d)
{
    SGTL5000State *s = SGTL5000(i2c);

    if (s->i2c_in_n < 4) {
        s->i2c_in[s->i2c_in_n++] = d;
    }
    if (s->i2c_in_n == 2) {
        s->cur_reg = ((uint16_t)s->i2c_in[0] << 8) | s->i2c_in[1];
    } else if (s->i2c_in_n == 4) {
        uint16_t v = ((uint16_t)s->i2c_in[2] << 8) | s->i2c_in[3];
        SGTL5000_DBG(s, "I2C W reg 0x%04x = 0x%04x\n", s->cur_reg, v);
        sgtl5000_reg_write(s, s->cur_reg, v);
    }
    return 0;
}

static uint8_t sgtl5000_i2c_recv(I2CSlave *i2c)
{
    SGTL5000State *s = SGTL5000(i2c);
    uint16_t v = s->cur_reg == SGTL_CHIP_ID ? 0xa000
                                            : sgtl_get(s, s->cur_reg);
    uint8_t b;

    if (s->read_byte == 0) {
        b = v >> 8;
        s->read_byte = 1;
    } else {
        b = v & 0xff;
    }
    SGTL5000_DBG(s, "I2C R reg 0x%04x byte%d = 0x%02x\n",
                 s->cur_reg, s->read_byte - 1, b);
    return b;
}

static int sgtl5000_i2c_event(I2CSlave *i2c, enum i2c_event ev)
{
    SGTL5000State *s = SGTL5000(i2c);

    switch (ev) {
    case I2C_START_SEND:
        s->i2c_in_n = 0;
        break;
    case I2C_START_RECV:
        s->read_byte = 0;   /* keep s->cur_reg selected by the write phase */
        break;
    case I2C_FINISH:
        s->i2c_in_n = 0;
        break;
    case I2C_NACK:
    default:
        break;
    }
    return 0;
}

/* ---- device lifecycle ---- */

static void sgtl5000_reset(DeviceState *dev)
{
    SGTL5000State *s = SGTL5000(dev);
    static const struct { uint16_t reg, val; } defaults[] = {
        { SGTL_CHIP_DIG_POWER,    0x0000 },
        { SGTL_CHIP_I2S_CTRL,     0x0010 },
        { SGTL_CHIP_SSS_CTRL,     0x0010 },
        { SGTL_CHIP_ADCDAC_CTRL,  0x020c },
        { SGTL_CHIP_DAC_VOL,      0x3c3c },
        { SGTL_CHIP_PAD_STRENGTH, 0x015f },
        { SGTL_CHIP_ANA_ADC_CTRL, 0x0000 },
        { SGTL_CHIP_ANA_HP_CTRL,  0x1818 },
        { SGTL_CHIP_ANA_CTRL,     0x0111 },
        { SGTL_CHIP_LINREG_CTRL,  0x0000 },
        { SGTL_CHIP_REF_CTRL,     0x0000 },
        { SGTL_CHIP_MIC_CTRL,     0x0000 },
        { SGTL_CHIP_LINE_OUT_CTRL, 0x0000 },
        { SGTL_CHIP_LINE_OUT_VOL, 0x0404 },
        { SGTL_CHIP_ANA_POWER,    0x7060 },
        { SGTL_CHIP_PLL_CTRL,     0x5000 },
        { SGTL_CHIP_CLK_TOP_CTRL, 0x0000 },
        { SGTL_CHIP_ANA_STATUS,   0x0000 },
        { SGTL_CHIP_SHORT_CTRL,   0x0000 },
        { SGTL_DAP_CTRL,          0x0000 },
    };

    memset(s->regs, 0, sizeof(s->regs));
    for (size_t i = 0; i < ARRAY_SIZE(defaults); i++) {
        sgtl_put(s, defaults[i].reg, defaults[i].val);
    }
    s->i2c_in_n = 0;
    s->read_byte = 0;
    s->out_start = s->out_len = 0;
    s->in_start = s->in_len = 0;
    s->dac_on = s->adc_on = false;
}

static void sgtl5000_build_gains(SGTL5000State *s)
{
    /*
     * DAC: 0.5 dB attenuation per register step from 0 dB (0x3c) to
     * -96 dB (0xfc clamped); gain stored as Q15 (1.0 = 0x8000).
     * 10^(-0.025) = 0.9438743...; built multiplicatively with integer
     * arithmetic only (no libm dependency in the device model).
     */
    s->dac_gain[0] = 0x8000;
    for (unsigned i = 1; i <= 0xc0; i++) {
        s->dac_gain[i] =
            (uint32_t)(((uint64_t)s->dac_gain[i - 1] * 30921 + 16384) >> 15);
    }
    /*
     * MIC gain 0/20/30/40 dB -> amplitude 1/10/10^1.5/100, again as Q15.
     * 10^1.5 = 31.6227766... -> 1036218 (nearest int of 31.6227766*32768).
     */
    s->mic_gain[0] = 0x8000;
    s->mic_gain[1] = 327680;               /* x10   */
    s->mic_gain[2] = 1036218;              /* x31.6 */
    s->mic_gain[3] = 3276800;              /* x100  */
}

static void sgtl5000_realize(DeviceState *dev, Error **errp)
{
    SGTL5000State *s = SGTL5000(dev);
    struct audsettings as = {
        .freq = SGTL5000_FREQ_HZ,
        .nchannels = 2,
        .fmt = AUDIO_FORMAT_S16,
        .big_endian = false,
    };

    s->dbg = sgtl5000_debug();
    s->last_dbg_ns = 0;
    sgtl5000_build_gains(s);

    /*
     * The codec is always present on the board (I2C register model).
     * If no QEMU audio backend is available (no -audiodev), keep the
     * register model alive and simply do not open streams: the DAC/ADC
     * gates then route to a silent sink, which is the correct fallback
     * for a machine that boots without an audiodev.
     */
    Error *local_err = NULL;
    if (audio_be_check(&s->audio_be, &local_err)) {
        s->dac_voice = audio_be_open_out(s->audio_be, NULL, "sgtl5000.dac",
                                         s, sgtl5000_out_cb, &as);
        s->adc_voice = audio_be_open_in(s->audio_be, NULL, "sgtl5000.adc",
                                        s, sgtl5000_in_cb, &as);
        audio_be_set_active_out(s->audio_be, s->dac_voice, false);
        audio_be_set_active_in(s->audio_be, s->adc_voice, false);
    } else {
        error_free(local_err);
        s->audio_be = NULL;
        s->dac_voice = NULL;
        s->adc_voice = NULL;
    }
}

static void sgtl5000_exit(DeviceState *dev)
{
    SGTL5000State *s = SGTL5000(dev);

    if (s->audio_be) {
        if (s->dac_voice) {
            audio_be_close_out(s->audio_be, s->dac_voice);
        }
        if (s->adc_voice) {
            audio_be_close_in(s->audio_be, s->adc_voice);
        }
    }
}

static int sgtl5000_post_load(void *opaque, int version_id)
{
    SGTL5000State *s = opaque;

    s->dac_on = false;
    s->adc_on = false;
    sgtl5000_update_paths(s);
    return 0;
}

static const VMStateDescription vmstate_sgtl5000 = {
    .name = "sgtl5000",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = sgtl5000_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SGTL5000State, SGTL_REGCNT),
        VMSTATE_UINT8(i2c_in_n, SGTL5000State),
        VMSTATE_UINT16(cur_reg, SGTL5000State),
        VMSTATE_UINT8(read_byte, SGTL5000State),
        VMSTATE_UINT32(out_start, SGTL5000State),
        VMSTATE_UINT32(out_len, SGTL5000State),
        VMSTATE_UINT32(in_start, SGTL5000State),
        VMSTATE_UINT32(in_len, SGTL5000State),
        VMSTATE_END_OF_LIST()
    },
};

static const Property sgtl5000_props[] = {
    DEFINE_AUDIO_PROPERTIES(SGTL5000State, audio_be),
};

static void sgtl5000_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = sgtl5000_realize;
    dc->unrealize = sgtl5000_exit;
    device_class_set_legacy_reset(dc, sgtl5000_reset);
    dc->vmsd = &vmstate_sgtl5000;
    device_class_set_props(dc, sgtl5000_props);

    sc->send = sgtl5000_i2c_send;
    sc->recv = sgtl5000_i2c_recv;
    sc->event = sgtl5000_i2c_event;
}

static const TypeInfo sgtl5000_types[] = {
    {
        .name = TYPE_SGTL5000,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(SGTL5000State),
        .class_init = sgtl5000_class_init,
        .abstract = false,
    },
};

DEFINE_TYPES(sgtl5000_types)
