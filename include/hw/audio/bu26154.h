/*
 * LAPIS Semiconductor / ROHM BU26154MUV monaural audio CODEC with
 * touch-panel interface.
 *
 * This is the audio codec used by the real device under emulation
 * (CE registry: WaveDev = wavedev2_BU26154.dll).  Earlier emulator
 * rounds wired a Freescale SGTL5000 (taken from the Linux DTS) which
 * is NOT what the WinCE system drives; S97 replaces it with this
 * BU26154 register-faithful model.
 *
 * Datasheet: ROHM "BU26154MUV Monaural Audio CODEC with Touch Panel
 * Interface", Rev.002 (26.Oct.2015).  The model was written against that
 * document; the copy cited in earlier rounds
 * (work-log/2026-09-02/evidence_s97/BU26154MUV_datasheet_Rev002_89p.pdf)
 * is *not* in this repository, so anything below that cites a page is
 * from the datasheet text of pp.33-34 (touch panel interface, interrupt
 * wait settings), pp.44-48 (register descriptions: EQ tails, SR, CLKEN,
 * CLKIO, SOFTRST, RECPLAY, MCTIME, MAPCON, AREFPW, AINPW) and p.55
 * (OSRSEL, Mic Interface Control, SEMODE).  Register *fields* not covered
 * by that text are taken from the summary tables on pp.41-44 and are
 * marked "~"; no register value in this model is guessed silently.
 *
 * Control interface (datasheet "2 wire serial interface"): I2C slave,
 * 7-bit address 0x1a (0011010, SAD="L") or 0x1b (0011011, SAD="H").
 * Register index is 8-bit; even index = read address, odd index =
 * write address of the same 8-bit register word (word = index >> 1).
 * Continuous transfers step the index by two (next word).
 *
 * This header is shared with the i.MX28 SAIF model (hw/misc/mxs_saif.c)
 * exactly like include/hw/audio/sgtl5000.h so the SAIF serial engine
 * can push playback frames into the codec and pull captured frames out
 * of it; the SAIF supplies the frame clock, while the codec's internal
 * rate follows its programmed SR[3:0] (datasheet p.45).
 */
#ifndef HW_BU26154_H
#define HW_BU26154_H

#include "hw/i2c/i2c.h"

#define TYPE_BU26154 "bu26154"
OBJECT_DECLARE_SIMPLE_TYPE(BU26154State, BU26154)

/*
 * Default 7-bit slave address (datasheet: SAD="L" -> "0011010").
 * SAD="H" selects 0x1b; the machine property "address" overrides.
 */
#define BU26154_I2C_ADDR 0x1a

/*
 * Frame cadence the SAIF model pushes at, i.e. the LRCLK the i.MX28
 * generates.  It is *not* the codec's own rate: that one is programmed
 * through the Sampling Rate Setting Register SR[3:0] (datasheet Rev.002
 * p.45, reset value 0 = 8 kHz) and the model uses it for the datasheet
 * timing laws.  A guest that programs SR to something else than what
 * SAIF clocks in is out of the datasheet's contract and is reported.
 */
#define BU26154_FREQ_HZ 48000

/* MAPCON values select one of three register maps. */
#define BU26154_MAP0 0   /* audio (default) */
#define BU26154_MAP1 1   /* PLL settings + touch-panel pen-detect interface */
#define BU26154_MAP2 2   /* PLL external / "B" variant coefficients */

/* Number of register words per map (index space 0x00..0xff). */
#define BU26154_WORDS 128

typedef struct BU26154Stats {
    /* playback: frames handed over by the SAIF engine */
    uint64_t dac_in_frames;
    /* playback: frames actually delivered to the host audio backend */
    uint64_t dac_out_frames;
    /* playback: bytes handed to the host audio backend */
    uint64_t dac_out_bytes;
    /* playback: frames dropped (DAC powered down / muted / full) */
    uint64_t dac_dropped;
    /* capture: frames pulled by the SAIF engine (incl. silence) */
    uint64_t adc_out_frames;
    /* capture: bytes read from the host audio backend (real mic) */
    uint64_t adc_in_bytes;
    /* capture: bytes injected by the brain_micfill analysis aid */
    uint64_t adc_fill_bytes;
    /* capture: frames returned as silence while the ring was empty */
    uint64_t adc_underrun;
    /* capture: frames gated while the ADC was powered down */
    uint64_t adc_gated;
    /* capture: bytes currently buffered in the ADC ring (snapshot) */
    unsigned adc_pending;
    /* register-level diagnostics */
    uint8_t mapcon;               /* current MAPCON value */
    uint8_t sr;                   /* Sampling Rate register (MAP0 w0) */
    uint8_t vmicon;               /* VMIDCON field (MAP0 w0x10) */
    uint8_t micben;               /* MICBEN bit (MAP0 w0x10) */
    uint8_t micbcon;              /* MICBCON field (MAP0 w0x18) */
    uint8_t pgaen;                /* PGAEN bit (MAP0 w0x11) */
    uint8_t pgaatt;               /* PGAATT bit (MAP0 w0x11) */
    uint8_t adc_pwr;              /* ADC power reg (MAP0 w0x11) */
    uint8_t dac_pwr;              /* DAC power reg (MAP0 w0x12) */
    uint8_t sp_pwr;               /* SP amp power reg (MAP0 w0x13) */
    uint8_t pdatt;                /* Playback Digital Attenuator */
    uint8_t avvol;                /* Analog Volume */
    uint8_t rdvol;                /* Record Digital Attenuator */
    uint8_t avmute;               /* Amplifier Volume Ctrl Fn Enable */
    uint8_t dvmute;               /* Digital Volume Control */
    uint8_t clken;                /* Clock Enable reg (MAP0 w6, p.45) */
    uint8_t recplay;              /* RECPLAY[2:0] (MAP0 w9, p.47) */
    uint8_t mctime;               /* MCTIME[5:0] (MAP0 w0x0a, p.47) */
    uint8_t minif;                /* MINDIF|MINVOL (MAP0 w0x2d, p.55) */
    bool    mct_active;           /* mic charging mute window running */
    bool    clk_ok;               /* internal clock valid (pp.45-46) */
    /* true when a host audio backend is attached (not a silent sink) */
    bool backend_out;
    bool backend_in;
} BU26154Stats;

/* Playback: hand one 32-bit stereo frame (L << 16 | R) to the codec DAC. */
void bu26154_dac_input(BU26154State *s, uint32_t frame);

/* Capture: pull one 32-bit stereo frame (L << 16 | R) from the codec ADC. */
uint32_t bu26154_adc_output(BU26154State *s);

/* Analysis aid: fill the ADC capture ring with a deterministic PCM pattern
 * (same contract as sgtl5000_debug_fill). */
unsigned bu26154_debug_fill(BU26154State *s, uint32_t seed,
                            uint32_t nframes);

void bu26154_get_stats(BU26154State *s, BU26154Stats *st);

/* Debug: read back a register (idx = datasheet index byte, even = read
 * address).  Returns the value the chip would clock out for that word in
 * MAP0 (the map used by the audio driver). */
uint8_t bu26154_get_reg(BU26154State *s, uint8_t idx);

#endif /* HW_BU26154_H */
