/*
 * Freescale SGTL5000 ultra-low-power stereo audio codec
 *
 * The SHARP Brain PW-SH6 (i.MX28) connects an SGTL5000 to the chip's
 * SAIF0 (playback: CPU/DMA -> SAIF TX -> codec DAC) and SAIF1
 * (capture: codec ADC -> SAIF RX -> CPU/DMA) serial audio interfaces
 * (see wince/imx28-pwsh6.dts: codec@a = "fsl,sgtl5000" on I2C0,
 *  sound node compatible "fsl,mxs-audio-sgtl5000").
 *
 * This header is shared with the i.MX28 SAIF model (hw/misc/mxs_saif.c)
 * so the SAIF serial engine can push playback frames into the codec and
 * pull captured frames out of it at the model's fixed 48 kHz frame rate.
 */
#ifndef HW_SGTL5000_H
#define HW_SGTL5000_H

#include "hw/i2c/i2c.h"

#define TYPE_SGTL5000 "sgtl5000"
OBJECT_DECLARE_SIMPLE_TYPE(SGTL5000State, SGTL5000)

/*
 * SGTL5000 clocking is derived from the i.MX28 SAIF bit clock in real
 * hardware; the SAIF model runs a fixed 48 kHz stereo frame clock, so
 * the codec is driven at the same rate.
 */
#define SGTL5000_FREQ_HZ 48000

typedef struct SGTL5000Stats {
    /* playback: frames handed over by the SAIF engine */
    uint64_t dac_in_frames;
    /* playback: frames actually delivered to the audio backend */
    uint64_t dac_out_frames;
    /* playback: frames dropped (DAC powered down / muted / buffer full) */
    uint64_t dac_dropped;
    /* capture: frames pulled by the SAIF engine (incl. silence) */
    uint64_t adc_out_frames;
    /* capture: bytes taken from the audio backend (host mic) */
    uint64_t adc_in_bytes;
    /* capture: frames produced while the host provided no samples */
    uint64_t adc_underrun;
} SGTL5000Stats;

/* Playback: hand one 32-bit stereo frame (L << 16 | R) to the codec DAC. */
void sgtl5000_dac_input(SGTL5000State *s, uint32_t frame);

/* Capture: pull one 32-bit stereo frame (L << 16 | R) from the codec ADC. */
uint32_t sgtl5000_adc_output(SGTL5000State *s);

void sgtl5000_get_stats(SGTL5000State *s, SGTL5000Stats *st);

/* Debug: read back a register value (used by the brain_sgtl HMP aid). */
uint16_t sgtl5000_get_reg(SGTL5000State *s, uint16_t reg);

#endif /* HW_SGTL5000_H */
