#ifndef HW_MXS_SAIF_H
#define HW_MXS_SAIF_H

#define TYPE_MXS_SAIF "mxs-saif"

/*
 * Link a SAIF to the board's SGTL5000 codec (hw/audio/sgtl5000.c).
 *
 * playback = true  : the SAIF's transmit FIFO drains into the codec DAC
 *                    (i.MX28 reference wiring: SAIF0 is the I2S master
 *                    and carries the DAC data stream).
 * playback = false : the SAIF's receive FIFO is fed by the codec ADC
 *                    (reference wiring: SAIF1 carries the capture stream).
 *
 * The link is machine wiring; it is established once at realize time and
 * is not part of the device's migrated state.
 */
void mxs_saif_set_codec(DeviceState *saif, DeviceState *codec, bool playback);

/*
 * Analysis aid: synchronously pump up to nframes in from the linked
 * capture codec into the RX FIFO, exactly as the serial engine timer
 * would, but without depending on virtual time advancing (the guest may
 * be idle).  Only meaningful when the SAIF is in READ_MODE (capture).
 * Returns the number of frames actually pushed (FIFO bound).
 */
unsigned mxs_saif_pump_capture(DeviceState *saif, unsigned nframes);

#endif /* HW_MXS_SAIF_H */
