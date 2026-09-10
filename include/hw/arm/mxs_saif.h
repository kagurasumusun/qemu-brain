#ifndef HW_MXS_SAIF_H
#define HW_MXS_SAIF_H

#define TYPE_MXS_SAIF "mxs-saif"

/*
 * Link a SAIF to the board codec: the Brain's LAPIS/ROHM BU26154
 * (hw/audio/bu26154.c) by default, or an SGTL5000 (hw/audio/sgtl5000.c)
 * for the Linux-DTS configuration.  The dispatch in mxs_saif.c is by
 * device type, so either codec can be wired.
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

/*
 * Analysis aid: synchronously clock one playback frame out of the SAIF
 * into the linked DAC codec, exactly as the serial engine timer would,
 * but without depending on virtual time advancing.  @frame is the raw
 * 32-bit stereo word (L << 16 | R) that would have come out of the TX
 * FIFO.  Returns true when the frame was handed to a codec.
 *
 * This is the playback-side counterpart of mxs_saif_pump_capture() and
 * exists so the whole DAC chain (SAIF -> codec gain stages -> host audio
 * backend) can be exercised with known data before the guest reaches a
 * screen that plays sound.
 */
bool mxs_saif_push_playback(DeviceState *saif, uint32_t frame);

#endif /* HW_MXS_SAIF_H */
