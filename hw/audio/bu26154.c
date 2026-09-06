/*
 * LAPIS Semiconductor / ROHM BU26154MUV monaural audio CODEC with
 * touch-panel interface - QEMU device model.
 *
 * Register/protocol behaviour follows the ROHM BU26154MUV datasheet
 * Rev.002 (26.Oct.2015); see include/hw/audio/bu26154.h for the
 * control-interface summary and the evidence_s97 PDF for the source.
 *
 * Model scope (S97 phase A):
 *  - I2C slave protocol: 8-bit register index, even = read address,
 *    odd = write address of the same 8-bit word (word = index >> 1),
 *    continuous transfers step the index by two.
 *  - Three register maps selected by MAPCON (0x1c/0x1d); MAP0 = audio,
 *    MAP1 = PLL + touch-panel (pen detect) interface, MAP2 = PLL
 *    external / "B"-variant coefficients.  MAPCON = 0x3 is prohibited and is not
 *    accepted (p.48).  A power-on reset restores the datasheet initial
 *    values; the SOFTRST bit resets the CPU interface and its own
 *    register only, not the register file (p.46).
 *  - Register semantics: reserved bits read 0 and ignore writes;
 *    empty indices read 0 and ignore writes.
 *  - DAC/ADC data path with the same stereo frame plumbing as the
 *    SGTL5000 model so mxs_saif can drive either codec.  The frame
 *    cadence on the SAIF side is the 48 kHz LRCLK the SoC generates;
 *    the codec's own rate is programmed (SR[3:0], p.45) and is what the
 *    datasheet timing laws (MCTIME, p.47) are counted in.  Power gating
 *    is derived from the power-management registers (VMIDCON,
 *    DACREN/DACLEN, ADCEN, AVMUTE) plus the clock-enable rule below.
 *  - The record/playback paths only run with a valid internal clock
 *    (PLLOE, and the source CLKSEL[2:0] names, pp.45-46), and only after
 *    the MCTIME charging window opened by RECPLAY leaving 0x0 has
 *    expired (p.47).  RECPLAY may only be entered left from 0x0 (p.47).
 *  - With -audiodev the DAC
 *    stream is rendered to the host output and the host input is really
 *    captured into the ADC ring; with no backend the codec is the
 *    register model and both paths are a silent sink (the same fallback
 *    the SGTL5000 model uses).
 *
 * Caveat: the entries below were transcribed from the Rev.002 datasheet
 * tables.  What has been re-read against the datasheet text itself
 * (pp.45-48, 55) and is therefore precise: SR, CLKEN, CLKIO, SOFTRST,
 * RECPLAY, MCTIME, MAPCON, AREFPW, AINPW, OSRSEL, Mic Interface Control
 * and the sound-effect mode words.  The power-management bit maps on
 * pp.48 were confirmed register by register.  Still transcribed from the
 * register *table* only, i.e. field positions are not text-verified and
 * are marked "~": the MAP1 touch-panel scan block (SCEN, SCTHR*,
 * SCGAIN, the touch ADC result words), ALC and noise-gate settings, and
 * the MAP2 tails.  Those fields are stored and read back faithfully but
 * their *behaviour* (touch conversion, ALC gain ramping, EQ / sound
 * effect processing) is not modelled; nothing in the model invents a
 * result for them.
 *
 * S102 spec audit (Rev.002 register descriptions, pp.45-48 and 55, read
 * from the datasheet text rather than inferred): the SOFTRST scope, the
 * MAPCON and OSRSEL prohibited settings, the RECPLAY code set and its
 * "only through a stop" transition rule, the MCTIME field width and its
 * 40/fs + 128/fs per step law, the CLKEN/CLKIO internal-clock rule and
 * the Mic Interface Control MINVOL[2:0] field (mask, reset value and the
 * 6..27 dB gain, which is now applied to the capture path) were all
 * corrected against that text.
 *
 * S101 audit: AREFPW (HPREN|HPLEN|HPVDDEN|MICBEN|VMIDCON), AINPW
 * (PGAATT|PGAEN|ADCEN, no ADCREN bit), SPPW (SPMDSEL|AVREN|COEFSEL|SPEN|
 * AVLEN with b02 hard-wired high) and AVOL[5:0] (0x10 = +9 dB) were
 * re-verified against Rev.002 pp.46-55 and the wavedev2_BU26154.dll
 * register trace; AINPW/SPPW masks, the SPPW reset value and the AVVOL
 * width/gain law were corrected accordingly.
 */
#include "qemu/osdep.h"
#include <math.h>
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/i2c/i2c.h"
#include "hw/audio/bu26154.h"
#include "qemu/audio.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qom/object.h"

/* ---- register-word addresses (MAP0) ---- */
#define BU_REG_SR        0x00    /* Sampling Rate Setting [3:0]      */
#define BU_REG_CLKEN     0x06    /* Clock Enable                      */
#define BU_REG_CLKIO     0x07    /* Clock Input/Output Control        */
#define BU_REG_SOFTRST   0x08    /* Software Reset (write 0x11 bit0)  */
#define BU_REG_RECPLAY   0x09    /* Record/Playback Running Control   */
#define BU_REG_MCTIME    0x0a    /* Mic Input Charging Time           */
#define BU_REG_MAPCON    0x0e    /* Register MAP Control (0x1c/0x1d)  */
#define BU_REG_AREFPW    0x10    /* Analog Reference Power Mgmt       */
#define BU_REG_AINPW     0x11    /* Analog Input Power Mgmt           */
#define BU_REG_DACPW     0x12    /* DAC Power Management              */
#define BU_REG_SPPW      0x13    /* Speaker Amplifier Power Mgmt      */
#define BU_REG_TSDEN     0x16    /* Thermal Shutdown Control          */
#define BU_REG_ZCEN      0x17    /* Zero Cross Comparator Power Mgmt  */
#define BU_REG_MICB      0x18    /* MICBIAS Voltage Control           */
#define BU_REG_AVVOL     0x1d    /* Analog Volume Control             */
#define BU_REG_PDATT     0x1f    /* Playback Digital Attenuator       */
#define BU_REG_AVMUTE    0x24    /* Amplifier Volume Ctrl Fn Enable   */
#define BU_REG_EFFECT    0x38    /* Playback Effect Volume (0x70/71)  */
#define BU_REG_PDATTB    0x39    /* Playback Digital Attenuator B     */
#define BU_REG_DVMUTE    0x34    /* Digital Volume Control (0x68/69)  */
#define BU_REG_RDVOL     0x36    /* Record Digital Attenuator         */
#define BU_REG_OSRSEL    0x2c    /* DAC Clock Setting [5:4] (0x58/59) */
#define BU_REG_MINIF     0x2d    /* Mic Interface Control (0x5a/0x5b) */

/* bit positions within the registers above (datasheet bit maps) */
#define BU_BIT_COEFSEL   0x08    /* SPPW b03: select the "B" coefficients */
#define BU_BIT_DVMUTE    0x10    /* DVMUTE register b04                   */
#define BU_BIT_PGAATT    0x20    /* AINPW b05: mic amp 0 dB / -9 dB       */
#define BU_BIT_PGAEN     0x08    /* AINPW b03: microphone amplifier power */
#define BU_BIT_ADCEN     0x02    /* AINPW b01: ADC power                  */
#define BU_BIT_MICBEN    0x04    /* AREFPW b02: microphone bias circuit   */
#define BU_BIT_DACEN     0x06    /* DACPW b02|b01: DACREN | DACLEN        */
#define BU_BIT_AVMUTE    0x02    /* AVMUTE register b01                   */
/* Clock Enable register (0x0c/0x0d) bits, datasheet Rev.002 p.45 */
#define BU_BIT_TCLKEN    0x80    /* CLKEN b07: touch panel interface clock */
#define BU_BIT_PLLOE     0x04    /* CLKEN b02: PLL output enable           */
#define BU_BIT_PLLEN     0x02    /* CLKEN b01: PLL run / stop              */
#define BU_BIT_MCLKEN    0x01    /* CLKEN b00: MCLKI terminal input enable */

/* bit helpers */
#define BU_BIT(n) (1u << (n))

/* ---- per-register initial value / writable-mask ---- */
typedef struct BURegInit {
    uint8_t w;        /* register word (index >> 1) */
    uint8_t rv;       /* datasheet initial value    */
    uint8_t mask;     /* defined (writable) bits    */
} BURegInit;

/*
 * MAP0 (audio, datasheet pp.37-40).  w = index >> 1; index pairs are
 * listed as (even read / odd write) in the comments.
 */
static const BURegInit bu_map0_init[] = {
    { BU_REG_SR,      0x00, 0x0f }, /* 0x00/0x01 SR[3:0] */
    { BU_REG_CLKEN,   0x00, 0x87 }, /* 0x0c/0x0d TCLKEN|PLLOE|PLLEN|MCLKEN */
    { BU_REG_CLKIO,   0x00, 0x1f }, /* 0x0e/0x0f PLLISEL[1:0]|CLKSEL[2:0] */
    { BU_REG_SOFTRST, 0x00, 0x01 }, /* 0x10/0x11 SOFTRST */
    { BU_REG_RECPLAY, 0x00, 0x07 }, /* 0x12/0x13 RECPLAY[2:0] (p.47) */
    { BU_REG_MCTIME,  0x00, 0x3f }, /* 0x14/0x15 MCTIME[5:0] (p.47) */
    { BU_REG_MAPCON,  0x00, 0x03 }, /* 0x1c/0x1d MAPCON[1:0] (global) */
    { BU_REG_AREFPW,  0x00, 0xcf }, /* 0x20/0x21 HPREN|HPLEN|HPVDDEN|MICBEN|VMIDCON[1:0] */
    { BU_REG_AINPW,   0x00, 0x2a }, /* 0x22/0x23 PGAATT|PGAEN|ADCEN */
    { BU_REG_DACPW,   0x00, 0x06 }, /* 0x24/0x25 DACREN|DACLEN */
    { BU_REG_SPPW,    0x04, 0x9b }, /* 0x26/0x27 SPMDSEL|AVREN|COEFSEL|SPEN|AVLEN; b02 H-fix=1 */
    { BU_REG_TSDEN,   0x01, 0x01 }, /* 0x2c/0x2d TSDEN=1 */
    { BU_REG_ZCEN,    0x00, 0x02 }, /* 0x2e/0x2f ZCEN */
    { BU_REG_MICB,    0x00, 0x03 }, /* 0x30/0x31 MICBCON[1:0] */
    { BU_REG_AVVOL,   0x0a, 0x3f }, /* 0x3a/0x3b AVOL[5:0] init 01010b */
    { BU_REG_PDATT,   0xff, 0xff }, /* 0x3e/0x3f PDATT (0 dB) */
    { 0x23,           0x00, 0x3e }, /* 0x46/0x47 Play HPF2 setting */
    { BU_REG_AVMUTE,  0x00, 0x03 }, /* 0x48/0x49 AVMUTE|AVFADE */
    { 0x25,           0x00, 0x03 }, /* 0x4a/0x4b AVFCON[1:0] */
    { 0x26,           0x00, 0xff }, /* 0x4c/0x4d PHPF2C0L */
    { 0x27,           0x00, 0x3f }, /* 0x4e/0x4f PHPF2C0H */
    { 0x2c,           0x00, 0x30 }, /* 0x58/0x59 OSRSEL[5:4] (p.55) */
    { 0x2d,           0x84, 0x87 }, /* 0x5a/0x5b MINDIF|MINVOL=100b (p.55) */
    { 0x2e,           0x00, 0x87 }, /* 0x5c/0x5d SEMODE[7]|SEMODE[2:0] */
    { 0x30,           0xc0, 0xfe }, /* 0x60/0x61 SAI TX: PCMFO24|FMTO=1 */
    { 0x31,           0xc0, 0xfe }, /* 0x62/0x63 SAI RX: PCMFI24|FMTI=1 */
    { 0x32,           0x00, 0x11 }, /* 0x64/0x65 BSWP|MST */
    { 0x33,           0x01, 0xff }, /* 0x66/0x67 DSP filter enables, HPF1EN=1 */
    { 0x34,           0x00, 0x1b }, /* 0x68/0x69 DVMUTE|DVFADE|RALCEN|PALCEN */
    { 0x35,           0x00, 0xff }, /* 0x6a/0x6b DVFCON[3:0]|RMCON|LMCON */
    { 0x36,           0xff, 0xff }, /* 0x6c/0x6d RDVOL (0 dB) */
    { 0x38,           0xff, 0xff }, /* 0x70/0x71 Effect VOL (0 dB) */
    { 0x39,           0x40, 0x7f }, /* 0x72/0x73 RALCVOL (~) */
    { 0x3a,           0xe7, 0xff }, /* 0x74/0x75 EQGAIN0 */
    { 0x3b,           0xe7, 0xff }, /* 0x76/0x77 EQGAIN1 */
    { 0x3c,           0xe7, 0xff }, /* 0x78/0x79 EQGAIN2 */
    { 0x3d,           0xe7, 0xff }, /* 0x7a/0x7b EQGAIN3 */
    { 0x3e,           0xe7, 0xff }, /* 0x7c/0x7d EQGAIN4 */
    { 0x3f,           0x00, 0x07 }, /* 0x7e/0x7f HPF2CUT[2:0] */
    /* 0x80..0xa7 programmable EQ coefficients (words 0x40..0x53) */
    { 0x40, 0x00, 0xff }, { 0x41, 0x00, 0xff },
    { 0x42, 0x00, 0xff }, { 0x43, 0x00, 0xff },
    { 0x44, 0x00, 0xff }, { 0x45, 0x00, 0xff },
    { 0x46, 0x00, 0xff }, { 0x47, 0x00, 0xff },
    { 0x48, 0x00, 0xff }, { 0x49, 0x00, 0xff },
    { 0x4a, 0x00, 0xff }, { 0x4b, 0x00, 0xff },
    { 0x4c, 0x00, 0xff }, { 0x4d, 0x00, 0xff },
    { 0x4e, 0x00, 0xff }, { 0x4f, 0x00, 0xff },
    { 0x50, 0x00, 0xff }, { 0x51, 0x00, 0xff },
    { 0x52, 0x00, 0xff }, { 0x53, 0x00, 0xff },
    /* ALC block (note1 registers) */
    { 0x59,           0x02, 0x0f }, /* 0xb2/0xb3 RALCATK init 0010b */
    { 0x5a,           0x03, 0x0f }, /* 0xb4/0xb5 RALCDCY init 0011b */
    { 0x5c,           0x17, 0x1f }, /* 0xb8/0xb9 RALCLVL init 10111b */
    { 0x5d,           0x00, 0x1f }, /* 0xba/0xbb RALCMINGAIN */
    { 0x5e,           0x22, 0xff }, /* 0xbc/0xbd RSATEN|RSATMINGAIN init 0x22 (~) */
    { 0x5f,           0x00, 0x03 }, /* 0xbe/0xbf RALCZCTM */
    { 0x60,           0x04, 0x0f }, /* 0xc0/0xc1 PALCATK init 0100b */
    { 0x61,           0x05, 0x0f }, /* 0xc2/0xc3 PALCDCY init 0101b */
    { 0x62,           0x1b, 0x1f }, /* 0xc4/0xc5 PALCLVL init 11011b */
    { 0x63,           0x00, 0x1f }, /* 0xc6/0xc7 PALCMINGAIN */
    { 0x64,           0x40, 0x7f }, /* 0xc8/0xc9 PALCVOL init 1000000b (~) */
    { 0x65,           0x00, 0x03 }, /* 0xca/0xcb PALCZCTM */
    { 0x66,           0x00, 0x3a }, /* 0xcc/0xcd RALCFRTH|RALCFREN|RALCFRSP (~) */
    { 0x67,           0x00, 0x3a }, /* 0xce/0xcf PALCFRTH|PALCFREN|PALCFRSP (~) */
    { 0x6e,           0x00, 0x79 }, /* 0xdc/0xdd ZDTIME|ZDEN (~) */
    { 0x74,           0x01, 0x03 }, /* 0xe8/0xe9 MIN2EN|MIN1EN=1 */
};

/*
 * MAP1 -- the bank the MAPCON=0x1 register table (p.41) documents: PLL
 * setting, soft clip, touch ADC, headphone/speaker input select, the
 * programmable LPFs and the noise gate.  Verified against that table:
 *   0x20/0x21 SCEN      soft clip enable          init 0
 *   0x22-0x27 SCTHRH/M/L soft clip thresholds      init 0
 *   0x28/0x29 SCGAIN    soft clip gain             init 001b
 *   0x60/0x61           Touch ADC Control: TCHSEN, TCHA[2:0], TCHRSEL,
 *                       TCHMODE
 *   0x62/0x63 ADCR1     Touch ADC result 1         init 0
 *   0x64/0x65 ADCR2[3:0] Touch ADC result 2        init 0
 * Note the *soft clip* group: SCEN/SCTHR*/SCGAIN name the limiter, not the
 * touch panel, and the table's masks (0x01, 0x7f, 0xff, 0xff, 0x07) are
 * exactly what the entries below use.  Index 0x60/0x61 is bank-relative:
 * the audio bank (MAPCON=0) puts the SAI Transmitter Control register there
 * (p.56) and MAPCON=1 puts the Touch ADC Control register there, so p.34's
 * "touch panel interface interrupt circuit Enable" (0x61 = 0x38) is this
 * bank's word and needs MAPCON=1 -- the per-bank tables are what make that
 * legible at all.
 *
 * What this block really is, from the functional description (Rev.002
 * pp.33-34): the BU26154 touch interface is a *pen detector for the
 * 4-wire resistive plate* (YP/XP/YN/XN).  Its oscillation circuit is
 * enabled with MAP0 CLKEN.TCLKEN (the datasheet's own interrupt-wait
 * recipe writes 0x0d = 0x80 to enable and 0x0d = 0x00 to disable), the
 * interrupt circuit with w0x30 (0x60/0x61: the recipe writes index 0x61 =
 * 0x38 for "touch panel interface interrupt circuit Enable"), and a plate
 * contact pulls the open-drain IRQB pin low through the plate
 * resistance, with an internal pull-up holding it high otherwise.  IRQB is
 * also low while RESETB is low, with the first valid edge at least 1 ms
 * after reset release.
 *
 * It is *not* the right-edge touchkey strip: those nine flexible keys are
 * scanned by the EDNA2 MCU and reported in the mailbox (see
 * brain_kbd_touchkey_scan() in hw/input/brain_kbd.c), and the panel's
 * conversions go through the i.MX28 LRADC.  So nothing here fabricates a
 * touch conversion: the result words (0x62/0x63, 0x64/0x65) are stored and
 * read back exactly as written and are otherwise at their reset value.
 */
static const BURegInit bu_map1_init[] = {
    { 0x01, 0x00, 0x07 }, /* 0x02/0x03 FPLLM[2:0] */
    { 0x02, 0x00, 0xff }, /* 0x04/0x05 FPLLNL */
    { 0x03, 0x00, 0x01 }, /* 0x06/0x07 FPLLNH */
    { 0x04, 0x00, 0x0f }, /* 0x08/0x09 FPLLD[3:0] */
    { 0x05, 0x00, 0xff }, /* 0x0a/0x0b FPLLFL */
    { 0x06, 0x00, 0xff }, /* 0x0c/0x0d FPLLFH */
    { 0x07, 0x00, 0xff }, /* 0x0e/0x0f FPLLFDL */
    { 0x08, 0x00, 0xff }, /* 0x10/0x11 FPLLFDH */
    { 0x09, 0x00, 0x0f }, /* 0x12/0x13 FPLLV[3:0] */
    { BU_REG_MAPCON, 0x00, 0x03 }, /* 0x1c/0x1d MAPCON (global) */
    { 0x10, 0x00, 0x01 }, /* 0x20/0x21 SCEN */
    { 0x11, 0x00, 0x7f }, /* 0x22/0x23 SCTHRH[6:0] */
    { 0x12, 0x00, 0xff }, /* 0x24/0x25 SCTHRM */
    { 0x13, 0x00, 0xff }, /* 0x26/0x27 SCTHRL */
    { 0x14, 0x01, 0x07 }, /* 0x28/0x29 SCGAIN[2:0] init 001b */
    { 0x30, 0x70, 0xde }, /* 0x60/0x61 touch panel interface interrupt
                            * circuit: p.41 names the fields (TCHSEN,
                            * TCHA[2:0], TCHRSEL, TCHMODE) and p.34 enables
                            * it with 0x38.  Which of those bits each of the
                            * two gaps falls in is printed only in the
                            * register diagram's graphics, so the mask and
                            * reset value stay "~".
                            */
    { 0x31, 0x00, 0xff }, /* 0x62/0x63 ADCR1 (result, read-mostly) */
    { 0x32, 0x00, 0x0f }, /* 0x64/0x65 ADCR2[3:0] (result, read-mostly) */
    { 0x41, 0x00, 0x31 }, /* 0x82/0x83 HP input select */
    { 0x42, 0x00, 0x0f }, /* 0x84/0x85 SP amp input control */
    { 0x50, 0x00, 0x03 }, /* 0xa0/0xa1 Play LPF setting */
    { 0x51, 0x00, 0xff }, /* 0xa2/0xa3 PLPFC0L */
    { 0x52, 0x00, 0x3f }, /* 0xa4/0xa5 PLPFC0H */
    { 0x53, 0x00, 0x03 }, /* 0xa6/0xa7 Rec LPF setting */
    { 0x54, 0x00, 0xff }, /* 0xa8/0xa9 RLPFC0L */
    { 0x55, 0x00, 0x3f }, /* 0xaa/0xab RLPFC0H */
    /* noise gate */
    { 0x6d, 0x00, 0x01 }, /* 0xda/0xdb NGEN */
    { 0x6f, 0xd3, 0xff }, /* 0xde/0xdf NGMINGAIN init 11010011b (~) */
    { 0x70, 0x12, 0x3f }, /* 0xe0/0xe1 NGTH init 010010b (~) */
    { 0x71, 0x02, 0x03 }, /* 0xe2/0xe3 NGTHHYS init 10b */
    { 0x72, 0x14, 0x7f }, /* 0xe4/0xe5 NGSLOPE init 0010100b (~) */
    { 0x73, 0x02, 0x03 }, /* 0xe6/0xe7 NGGAINSTEP init 10b */
    { 0x74, 0x10, 0x77 }, /* 0xe8/0xe9 NGENVAVE|NGZTIM (~) */
    { 0x75, 0x15, 0xff }, /* 0xea/0xeb NGFDOUT|NGFDIN init 00010101b (~) */
    { 0x76, 0x00, 0xff }, /* 0xec/0xed NGENVMONL (read-mostly) */
    { 0x77, 0x00, 0xff }, /* 0xee/0xef NGENVMONL H (read-mostly) */
    { 0x78, 0x00, 0xff }, /* 0xf0/0xf1 NGENVMONR (read-mostly) */
    { 0x79, 0x00, 0xff }, /* 0xf2/0xf3 NGENVMONR H (read-mostly) */
    { 0x7a, 0x00, 0xff }, /* 0xf4/0xf5 NGGAINMON (read-mostly) */
};

/*
 * MAP2 (PLL external components + "B" variant coefficients,
 * datasheet pp.42-44).
 */
static const BURegInit bu_map2_init[] = {
    { 0x00, 0x01, 0x01 }, /* 0x00/0x01 EXMODE=1 */
    { 0x02, 0x26, 0x26 }, /* 0x04/0x05 level shifter for headphone: b02
                            * (p.34 writes 0x26 to turn it on, 0x22 off) */
    { 0x09, 0x01, 0x01 }, /* 0x12/0x13 reference current circuit for the
                            * audio system, on at reset (p.34) */
    { BU_REG_MAPCON, 0x00, 0x03 }, /* 0x1c/0x1d MAPCON (global) */
    { 0x12, 0x00, 0xff }, /* 0x24/0x25 P2B param0A */
    { 0x13, 0x00, 0xff }, /* 0x26/0x27 P2B param1A */
    { 0x14, 0x00, 0xff }, /* 0x28/0x29 P2B param2A */
    { 0x15, 0x00, 0xff }, /* 0x2a/0x2b P2B param0B */
    { 0x16, 0x00, 0xff }, /* 0x2c/0x2d P2B param1B */
    { 0x17, 0x00, 0xff }, /* 0x2e/0x2f P2B param2B */
    /* "B" variants (effect bank B) */
    { 0x23, 0x00, 0x3e }, /* 0x46/0x47 Play HPF2B */
    { 0x26, 0x00, 0xff }, /* 0x4c/0x4d PHPF2C0LB */
    { 0x27, 0x00, 0x3f }, /* 0x4e/0x4f PHPF2C0HB */
    { 0x2e, 0x00, 0x87 }, /* 0x5c/0x5d SEMODEB */
    { 0x33, 0x01, 0xff }, /* 0x66/0x67 filter enables B (HPF1ENB=1) */
    { 0x38, 0xff, 0xff }, /* 0x70/0x71 Effect VOL B */
    { 0x39, 0xff, 0xff }, /* 0x72/0x73 PDATT B */
    { 0x3a, 0xe7, 0xff }, { 0x3b, 0xe7, 0xff }, { 0x3c, 0xe7, 0xff },
    { 0x3d, 0xe7, 0xff }, { 0x3e, 0xe7, 0xff }, /* EQGAIN0..4 B */
    { 0x3f, 0x00, 0xff }, { 0x40, 0x00, 0xff }, /* EQ coef B */
    { 0x41, 0x00, 0xff }, { 0x42, 0x00, 0xff },
    { 0x43, 0x00, 0xff }, { 0x44, 0x00, 0xff },
    { 0x45, 0x00, 0xff }, { 0x46, 0x00, 0xff },
    { 0x47, 0x00, 0xff }, { 0x48, 0x00, 0xff },
    { 0x49, 0x00, 0xff }, { 0x4a, 0x00, 0xff },
    { 0x4b, 0x00, 0xff }, { 0x4c, 0x00, 0xff },
    { 0x4d, 0x00, 0xff }, { 0x4e, 0x00, 0xff },
    { 0x4f, 0x00, 0xff }, { 0x50, 0x00, 0xff },
    { 0x51, 0x00, 0xff }, { 0x52, 0x00, 0xff }, { 0x53, 0x00, 0xff },
};

#define BU_RING 65536

/* flat register-file index for (map, word) */
#define BU_REG_IDX(map, w) ((unsigned)(map) * BU26154_WORDS + (unsigned)(w))

struct BU26154State {
    I2CSlave parent_obj;

    /*
     * Host audio endpoint.  The machine forwards -audiodev as the codec
     * "audiodev" property (hw/arm/mxs.c -> mxs_i2c "codec-audiodev").
     * When a backend is present the DAC path really renders to the host
     * output and the ADC path really captures from the host input; with
     * no -audiodev the register model stays alive and the paths are a
     * silent sink (same fallback as the SGTL5000 model).
     */
    AudioBackend *audio_be;
    SWVoiceOut *dac_voice;
    SWVoiceIn  *adc_voice;

    /* register files: regs[map][word] (masked values only) */
    uint8_t regs[3 * BU26154_WORDS];
    uint8_t map;                 /* current MAPCON value 0..2 */

    /* I2C session */
    uint8_t cur_idx;             /* register index byte of this txn */
    bool    want_idx;            /* next send byte is an index byte */

    /* playback staging -> host output */
    uint8_t outbuf[BU_RING];
    unsigned out_start, out_len;
    bool    dac_on;

    /*
     * MIC Input Charging Time window (datasheet p.47): RECPLAY leaving
     * 0x0 mutes the record and playback signal paths until MCTIME has
     * elapsed.  mct_expire_ns is the virtual time the window ends at
     * (0 = inactive); the timer only re-evaluates the paths when it ends.
     */
    int64_t mct_expire_ns;
    QEMUTimer *mct_timer;

    /* capture staging <- host input */
    uint8_t inbuf[BU_RING];
    unsigned in_start, in_len;
    bool    adc_on;

    /*
     * Gain tables built once from the datasheet transfer tables
     * (Q15 linear).  pdatt[]/rdvol[] index the 0.5 dB playback and
     * record digital attenuator law (0x70..0xFF = -71.5..0.0 dB,
     * 0x6F = mute, 0x00..0x6E = "prohibited from setting" and treated
     * as mute); avvol[] indexes the analog volume law of AVOL[5:0];
     * pga[2] is the microphone amplifier gain (0 dB / -9 dB).
     */
    uint32_t pdatt[256];
    uint32_t rdvol[256];
    uint32_t effect[256];
    uint32_t avvol[0x40];
    uint32_t pga[2];
    uint32_t micvol[8];   /* MINVOL[2:0]: 6 dB .. 27 dB, p.55 */

    BU26154Stats stats;

    /* debug aid (BRAIN_BU26154_DEBUG=1): periodic stderr stats */
    bool dbg;
    int64_t last_dbg_ns;
};

#define BU26154_DBG(s, ...)                                           \
    do {                                                              \
        if ((s)->dbg) {                                               \
            fprintf(stderr, "[bu26154] " __VA_ARGS__);                \
        }                                                             \
    } while (0)

static bool bu26154_debug(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("BRAIN_BU26154_DEBUG");
        on = e && *e && *e != '0';
    }
    return on;
}

static uint8_t bu_get(BU26154State *s, unsigned map, unsigned w)
{
    return (map < 3 && w < BU26154_WORDS) ? s->regs[BU_REG_IDX(map, w)] : 0;
}

/* (re)load all three register files with their datasheet initial
 * values and clear the map select. */
static void bu26154_reset_file(BU26154State *s);
static void bu26154_update_paths(BU26154State *s);

/* ---- gain laws transcribed from the datasheet transfer tables ---- */

/* Q15 linear gain for a dB value. */
static uint32_t bu_db_to_q15(double db)
{
    double lin = pow(10.0, db / 20.0);
    double q15 = lin * 32768.0;

    if (q15 > (double)0x7fffffffu) {
        q15 = (double)0x7fffffffu;
    }
    return (uint32_t)(q15 + 0.5);
}

/*
 * The playback digital attenuator (PDATT / PDATTB), the record digital
 * attenuator (RDVOL) and the playback Effect Volume all share the same
 * law: 0x00..0x6E are "prohibited from setting", 0x6F is MUTE and
 * 0x70..0xFF run -71.5 dB .. 0.0 dB in 0.5 dB steps.  Prohibited codes
 * are modelled as mute: they are out of spec and must not produce a
 * louder-than-0 dB path.
 */
static void bu_build_atten_law(uint32_t *tab)
{
    for (unsigned v = 0; v < 256; v++) {
        if (v < 0x6f) {
            tab[v] = 0;                      /* prohibited -> mute */
        } else if (v == 0x6f) {
            tab[v] = 0;                      /* MUTE */
        } else {
            tab[v] = bu_db_to_q15(-71.5 + 0.5 * (double)(v - 0x70));
        }
    }
}

/*
 * Analog Volume (AVOL[5:0], datasheet "Analog Volume Control
 * Register").  0x00 = MUTE, 0x01..0x09 = -28..-2 dB, 0x0a = 0 dB,
 * 0x0b..0x19 = +2..+18 dB and 0x1a..0x3f are undefined ("-").
 * Undefined codes are clamped to the loudest defined step (+18 dB)
 * rather than muted: the datasheet marks them unsettable, so any value
 * there is a programming error and must not silence a working path.
 */
static void bu_build_avvol_law(uint32_t *tab)
{
    static const double db[0x1a] = {
        /* 0x00 */  -1e9,
        /* 0x01 */  -28.0, -24.0, -20.0, -16.0, -12.0,
        /* 0x06 */   -8.0,  -6.0,  -4.0,  -2.0,   0.0,
        /* 0x0b */    2.0,   4.0,   6.0,   7.0,   8.0,
        /* 0x10 */    9.0,  10.0,  11.0,  12.0,  13.0,
        /* 0x15 */   14.0,  15.0,  16.0,  17.0,  18.0,
    };

    for (unsigned v = 0; v < 0x40; v++) {
        unsigned idx = MIN(v, 0x19u);

        tab[v] = db[idx] < -1e8 ? 0 : bu_db_to_q15(db[idx]);
    }
}

static void bu26154_build_gains(BU26154State *s)
{
    bu_build_atten_law(s->pdatt);
    bu_build_atten_law(s->rdvol);
    bu_build_atten_law(s->effect);
    bu_build_avvol_law(s->avvol);
    s->pga[0] = bu_db_to_q15(0.0);      /* PGAATT=0: normal mode  */
    s->pga[1] = bu_db_to_q15(-9.0);     /* PGAATT=1: attenuation  */
    for (unsigned v = 0; v < 8; v++) {
        s->micvol[v] = bu_db_to_q15(6.0 + 3.0 * v);  /* MINVOL (p.55) */
    }
}

/* ---- register effects / power gating (datasheet-derived) ---- */

/*
 * Sampling Rate Setting Register (MAP0 w0, 0x00/0x01, datasheet p.45).
 * The codec's internal rate is programmed, not fixed: SR[3:0] = 0..8
 * selects 8 kHz .. 48 kHz and the reset value is 0 (8 kHz).  Codes
 * 0x9..0xf are not defined by the datasheet, so they keep the nominal
 * rate rather than changing it.
 */
static const uint32_t bu_sr_hz[16] = {
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
    0, 0, 0, 0, 0, 0, 0,
};

static uint32_t bu_fs_hz(BU26154State *s)
{
    uint32_t f = bu_sr_hz[bu_get(s, BU26154_MAP0, BU_REG_SR) & 0x0f];

    return f ? f : BU26154_FREQ_HZ;
}

/*
 * MIC Input Charging Time Register (MAP0 w0x0a, 0x14/0x15, datasheet
 * p.47): "the LSI work recording signal or playback signal are mute
 * when from RECPLAY is changed from 0x0 until MCTIME".  The law is
 * 40/fs for MCTIME = 0 and 128/fs per step above that (0x3f -> 8064/fs,
 * 168 ms at 48 kHz).
 */
static int64_t bu_mctime_ns(BU26154State *s)
{
    uint64_t n = bu_get(s, BU26154_MAP0, BU_REG_MCTIME) & 0x3f;
    uint64_t frames = n ? 128 * n : 40;

    return muldiv64(frames, 1000000000LL, bu_fs_hz(s));
}

static bool bu_in_mctime(BU26154State *s)
{
    return s->mct_expire_ns &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->mct_expire_ns;
}

static void bu_mct_tick(void *opaque)
{
    BU26154State *s = opaque;

    s->mct_expire_ns = 0;
    bu26154_update_paths(s);
}

/*
 * Clock Enable Register (MAP0 w6, 0x0c/0x0d) and Clock Input / Output
 * Control Register (MAP0 w7, 0x0e/0x0f), datasheet pp.45-46.  The
 * datasheet note "a function with (*) bit doesn't need internal clock to
 * change state" means everything else here only works with the internal
 * clock running: PLLOE "must be set to 1 ... otherwise internal clock
 * cannot be provided", and the source named by CLKSEL[2:0] must be
 * enabled (the PLL for CLKSEL 0/2/3, the MCLKI terminal for 4/6/7).
 * Without a clock the digital filters and the analog paths produce
 * nothing, so both streams are gated.
 */
static bool bu_clock_ok(BU26154State *s)
{
    uint8_t en = bu_get(s, BU26154_MAP0, BU_REG_CLKEN);
    uint8_t sel = bu_get(s, BU26154_MAP0, BU_REG_CLKIO) & 0x07;
    bool from_pll = (sel == 0x0 || sel == 0x2 || sel == 0x3);

    if (!(en & BU_BIT_PLLOE)) {
        return false;
    }
    return (en & (from_pll ? BU_BIT_PLLEN : BU_BIT_MCLKEN)) != 0;
}

/*
 * DAC Clock Setting Register (MAP0 w0x2c, 0x58/0x59, datasheet p.55)
 * groups the sampling rates the DAC clock is built for: 0x0 =
 * 8k/11.025k/12k, 0x1 = 16k/22.05k/24k, 0x2 = 32k/44.1k/48k, 0x3 =
 * prohibited.  A programming error here leaves the DAC unsynchronised on
 * real hardware, so surface it rather than papering over it.
 */
static unsigned bu_sr_group(unsigned sr)
{
    return sr <= 0x2 ? 0 : (sr <= 0x5 ? 1 : 2);
}

static void bu26154_check_sr_osr(BU26154State *s)
{
    unsigned sr = bu_get(s, BU26154_MAP0, BU_REG_SR) & 0x0f;
    unsigned osr = (bu_get(s, BU26154_MAP0, BU_REG_OSRSEL) >> 4) & 0x03;

    if (sr > 0x8 || osr > 0x2) {
        return;
    }
    if (bu_sr_group(sr) != osr) {
        qemu_log_mask(LOG_GUEST_ERROR, "bu26154: SR 0x%x (%u Hz) does not "
                      "match OSRSEL %u; the DAC clock group must cover the "
                      "sampling rate\n", sr, bu_sr_hz[sr], osr);
    }
}

static bool bu_vmid_on(BU26154State *s)
{
    /* VMIDCON[1:0] != 0 => analog reference power on. */
    return (bu_get(s, BU26154_MAP0, BU_REG_AREFPW) & 0x03) != 0;
}

static bool bu_dac_on(BU26154State *s)
{
    uint8_t pwr = bu_get(s, BU26154_MAP0, BU_REG_DACPW);
    uint8_t mute = bu_get(s, BU26154_MAP0, BU_REG_AVMUTE);

    return bu_vmid_on(s) && (pwr & BU_BIT_DACEN) && !(mute & BU_BIT_AVMUTE) &&
           bu_clock_ok(s) && !bu_in_mctime(s);
}

static bool bu_adc_on(BU26154State *s)
{
    uint8_t pw = bu_get(s, BU26154_MAP0, BU_REG_AINPW);

    return bu_vmid_on(s) && (pw & BU_BIT_ADCEN) &&
           bu_clock_ok(s) && !bu_in_mctime(s);
}

/*
 * The microphone path additionally needs the mic bias circuit (MICBEN)
 * and the microphone amplifier (PGAEN) powered up.  With either down the
 * PGA drives nothing into the ADC, so a real chip clocks out silence.
 */
static bool bu_mic_on(BU26154State *s)
{
    uint8_t ref = bu_get(s, BU26154_MAP0, BU_REG_AREFPW);
    uint8_t in = bu_get(s, BU26154_MAP0, BU_REG_AINPW);

    return (ref & BU_BIT_MICBEN) && (in & BU_BIT_PGAEN);
}

/* PGAATT (AINPW b05): 0 = normal mode 0 dB, 1 = attenuation mode -9 dB */
static unsigned bu_adc_pgaatt(BU26154State *s)
{
    return (bu_get(s, BU26154_MAP0, BU_REG_AINPW) & BU_BIT_PGAATT) ? 1 : 0;
}

static void bu26154_update_paths(BU26154State *s)
{
    bool dac = bu_dac_on(s);
    bool adc = bu_adc_on(s);

    if (dac != s->dac_on) {
        s->dac_on = dac;
        if (!dac) {
            /* DAC powered down / muted: discard staged playback data */
            s->out_start = s->out_len = 0;
        }
        if (s->audio_be && s->dac_voice) {
            audio_be_set_active_out(s->audio_be, s->dac_voice, dac);
        }
        BU26154_DBG(s, "DAC path %s\n", dac ? "on" : "off");
    }
    if (adc != s->adc_on) {
        s->adc_on = adc;
        if (!adc) {
            s->in_start = s->in_len = 0;
        }
        if (s->audio_be && s->adc_voice) {
            audio_be_set_active_in(s->audio_be, s->adc_voice, adc);
        }
        BU26154_DBG(s, "ADC path %s\n", adc ? "on" : "off");
    }
}

/* ---- host audio backend ---- */

static void bu26154_out_cb(void *opaque, int free_b)
{
    BU26154State *s = opaque;

    if (!s->audio_be || !s->dac_voice) {
        return;   /* no backend: register-only model, silent sink */
    }
    if (!s->dac_on || !s->out_len) {
        return;
    }
    size_t done = 0;

    while (done < s->out_len) {
        size_t chunk = MIN(s->out_len - done, BU_RING - s->out_start);
        size_t n = audio_be_write(s->audio_be, s->dac_voice,
                                  s->outbuf + s->out_start, chunk);

        if (!n) {
            break;   /* backend busy; the callback will come again */
        }
        s->out_start = (s->out_start + n) % BU_RING;
        s->out_len -= n;
        done += n;
        s->stats.dac_out_frames += n / 4;
        s->stats.dac_out_bytes += n;
    }
}

static void bu26154_in_cb(void *opaque, int avail_b)
{
    BU26154State *s = opaque;
    uint8_t tmp[4096];

    if (!s->audio_be || !s->adc_voice) {
        return;   /* no backend: register-only model */
    }
    if (!s->adc_on || avail_b <= 0) {
        return;
    }
    while (avail_b > 0 && s->in_len < BU_RING) {
        size_t want = MIN((size_t)avail_b, sizeof(tmp));
        size_t got;

        want = MIN(want, BU_RING - s->in_len);
        got = audio_be_read(s->audio_be, s->adc_voice, tmp, want);
        if (!got) {
            break;
        }
        for (size_t i = 0; i < got; i++) {
            unsigned wp = (s->in_start + s->in_len) % BU_RING;

            s->inbuf[wp] = tmp[i];
        }
        s->in_len += got;
        s->stats.adc_in_bytes += got;
        avail_b -= got;
    }
}

/* ---- register file ---- */

/*
 * Record/Playback Running Control Register (MAP0 w9, 0x12/0x13, datasheet
 * p.47).  RECPLAY[2:0] = 0x0 stop, 0x1 rec, 0x2 play, 0x3 rec+play,
 * 0x7 monitor; the other codes are undefined.  The datasheet also forbids
 * going from one running state to another: "Transition between other
 * states is prohibited.  Please move to the next movement once by all
 * means after having let recording/playback movement make a stop
 * (RECPLAY = 0x0)."  A start from the stopped state additionally opens
 * the MCTIME charging window, during which both paths are muted.
 */
static void bu26154_set_recplay(BU26154State *s, uint8_t nw)
{
    uint8_t old = bu_get(s, BU26154_MAP0, BU_REG_RECPLAY) & 0x07;

    switch (nw) {
    case 0x0: case 0x1: case 0x2: case 0x3: case 0x7:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "bu26154: RECPLAY 0x%x is not a "
                      "defined operation\n", nw);
        return;
    }
    if (old && nw && old != nw) {
        qemu_log_mask(LOG_GUEST_ERROR, "bu26154: RECPLAY 0x%x -> 0x%x "
                      "without a stop is prohibited\n", old, nw);
        return;
    }
    s->regs[BU_REG_IDX(BU26154_MAP0, BU_REG_RECPLAY)] = nw;
    BU26154_DBG(s, "RECPLAY %u -> %u\n", old, nw);

    if (old == 0x0 && nw != 0x0) {
        s->mct_expire_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                           bu_mctime_ns(s);
        timer_mod(s->mct_timer, s->mct_expire_ns);
    } else if (nw == 0x0) {
        s->mct_expire_ns = 0;
        timer_del(s->mct_timer);
    }
    bu26154_update_paths(s);
}

static void bu26154_reg_write(BU26154State *s, uint8_t idx, uint8_t v)
{
    unsigned w = idx >> 1;

    if (!(idx & 1)) {
        return;   /* even index = read address; writes are ignored */
    }
    if (idx == 0x1d) {           /* MAPCON (odd write index 0x1d) */
        if ((v & 0x03) == 0x03) {
            /* MAPCON = 0x3 is "prohibited from setting" (p.48). */
            qemu_log_mask(LOG_GUEST_ERROR, "bu26154: MAPCON 0x3 is "
                          "prohibited, map select unchanged\n");
            return;
        }
        s->map = v & 0x03;
        BU26154_DBG(s, "MAPCON <- %u\n", s->map);
        return;
    }
    if (idx == 0x11) {           /* Software Reset Register (p.46) */
        /*
         * "CPU interface and this register are reset by writing SOFTRST
         * bit to '1'.  And then, write '0' for releasing reset.  Only the
         * CPU interface (the index pointer and the transfer state) and
         * this register are reset - the audio register file and the map
         * select survive, which is what a driver that toggles SOFTRST in
         * the middle of its init sequence relies on.
         */
        if (v & 0x01) {
            s->regs[BU_REG_IDX(BU26154_MAP0, BU_REG_SOFTRST)] = 0x00;
            s->want_idx = true;
            s->cur_idx = 0;
            BU26154_DBG(s, "software reset (CPU interface)\n");
        }
        return;
    }
    if (w >= BU26154_WORDS) {
        return;
    }
    if (s->map == BU26154_MAP0 && w == BU_REG_OSRSEL && (v & 0x30) == 0x30) {
        qemu_log_mask(LOG_GUEST_ERROR, "bu26154: OSRSEL 0x3 is prohibited, "
                      "DAC clock setting unchanged\n");
        return;
    }
    if (s->map == BU26154_MAP0 && w == BU_REG_RECPLAY) {
        bu26154_set_recplay(s, v & 0x07);
        return;
    }
    if (s->map == BU26154_MAP0 && w == BU_REG_SPPW) {
        v |= 0x04;   /* SPPW b02 is hard-wired high ("H fix") on the part */
    }
    s->regs[BU_REG_IDX(s->map, w)] = v;  /* pre-masked by caller */
    if (s->map == BU26154_MAP0) {
        switch (w) {
        case BU_REG_AREFPW:
        case BU_REG_AINPW:
        case BU_REG_DACPW:
        case BU_REG_SPPW:
        case BU_REG_AVMUTE:
            bu26154_update_paths(s);
            break;
        case BU_REG_CLKEN:
        case BU_REG_CLKIO:
        case BU_REG_SR:
        case BU_REG_OSRSEL:
            bu26154_update_paths(s);
            bu26154_check_sr_osr(s);
            break;
        default:
            break;
        }
    }
}

/*
 * Write data masked by the register definition.  Called from the I2C
 * data path; idx is the current (odd) write index.
 */
static void bu26154_data_byte(BU26154State *s, uint8_t idx, uint8_t v)
{
    unsigned w = idx >> 1;
    const BURegInit *ri = NULL;
    const BURegInit *tab = bu_map0_init;
    size_t n = ARRAY_SIZE(bu_map0_init);

    if (s->map == BU26154_MAP1) {
        tab = bu_map1_init;
        n = ARRAY_SIZE(bu_map1_init);
    } else if (s->map == BU26154_MAP2) {
        tab = bu_map2_init;
        n = ARRAY_SIZE(bu_map2_init);
    }
    for (size_t i = 0; i < n; i++) {
        if (tab[i].w == w) {
            ri = &tab[i];
            break;
        }
    }
    if (!ri) {
        return;   /* empty index: ignore */
    }
    bu26154_reg_write(s, idx, v & ri->mask);
}

static uint8_t bu26154_read_word(BU26154State *s, uint8_t idx)
{
    unsigned w = idx >> 1;

    if (idx == 0x1c || idx == 0x1d) {
        return s->map;   /* MAPCON read-back */
    }
    if (w >= BU26154_WORDS) {
        return 0;
    }
    /* Only defined bits are stored (masked writes / masked resets),
     * so the raw file value already reads reserved bits as 0. */
    return s->regs[BU_REG_IDX(s->map, w)];
}

/* ---- device reset ---- */

static void bu26154_apply_inits(BU26154State *s, const BURegInit *tab,
                                size_t n, unsigned map)
{
    for (size_t i = 0; i < n; i++) {
        s->regs[BU_REG_IDX(map, tab[i].w)] = tab[i].rv & tab[i].mask;
    }
}

static void bu26154_reset_file(BU26154State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    bu26154_apply_inits(s, bu_map0_init, ARRAY_SIZE(bu_map0_init),
                        BU26154_MAP0);
    bu26154_apply_inits(s, bu_map1_init, ARRAY_SIZE(bu_map1_init),
                        BU26154_MAP1);
    bu26154_apply_inits(s, bu_map2_init, ARRAY_SIZE(bu_map2_init),
                        BU26154_MAP2);
    s->map = 0;
    s->mct_expire_ns = 0;
    if (s->mct_timer) {
        timer_del(s->mct_timer);
    }
}

static void bu26154_reset(DeviceState *dev)
{
    BU26154State *s = BU26154(dev);

    bu26154_reset_file(s);
    s->cur_idx = 0;
    s->want_idx = true;
    s->out_start = s->out_len = 0;
    s->in_start = s->in_len = 0;
    s->dac_on = s->adc_on = false;
    bu26154_update_paths(s);
}

/* ---- I2C slave interface ---- */

static int bu26154_i2c_send(I2CSlave *i2c, uint8_t d)
{
    BU26154State *s = BU26154(i2c);

    if (s->want_idx) {
        s->cur_idx = d;
        s->want_idx = false;
        BU26154_DBG(s, "I2C idx 0x%02x (%s)\n", d,
                    (d & 1) ? "write" : "read");
        return 0;
    }
    /* data byte for the current (odd) write index */
    BU26154_DBG(s, "I2C W idx 0x%02x = 0x%02x (map %u)\n",
                s->cur_idx, d, s->map);
    bu26154_data_byte(s, s->cur_idx, d);
    s->cur_idx = (s->cur_idx + 2) & 0xff;   /* continuous write: next word */
    return 0;
}

static uint8_t bu26154_i2c_recv(I2CSlave *i2c)
{
    BU26154State *s = BU26154(i2c);
    uint8_t v;

    if (s->want_idx) {
        v = 0;   /* read without a preceding index selection */
    } else {
        v = bu26154_read_word(s, s->cur_idx);
    }
    BU26154_DBG(s, "I2C R idx 0x%02x = 0x%02x (map %u)\n",
                s->cur_idx, v, s->map);
    s->cur_idx = (s->cur_idx + 2) & 0xff;   /* continuous read: next word */
    return v;
}

static int bu26154_i2c_event(I2CSlave *i2c, enum i2c_event ev)
{
    BU26154State *s = BU26154(i2c);

    switch (ev) {
    case I2C_START_SEND:
        s->want_idx = true;      /* next byte of this txn is the index */
        break;
    case I2C_START_RECV:
        /*
         * Read phase: the master selected the register index in the
         * preceding write phase (which ended with I2C_FINISH).  Serve
         * reads from cur_idx; do not expect a fresh index byte here.
         */
        s->want_idx = false;
        break;
    case I2C_FINISH:
        s->want_idx = true;
        break;
    case I2C_NACK:
    default:
        break;
    }
    return 0;
}

/* ---- SAIF <-> codec frame plumbing ---- */

void bu26154_dac_input(BU26154State *s, uint32_t frame)
{
    s->stats.dac_in_frames++;

    if (!s->dac_on) {
        s->stats.dac_dropped++;
        return;
    }
    if (s->out_len + 4 > BU_RING) {
        s->stats.dac_dropped++;   /* staging overrun: drop frame */
        return;
    }

    /*
     * Playback gain chain of the real part (datasheet block diagram:
     * SAI_SDIN -> DAC -> digital attenuator -> Effect Volume -> Sound
     * Effect -> analog volume -> SP/HP amplifier).  PDATT and Effect
     * VOL share the 0.5 dB attenuator law; AVVOL is the analog stage.
     * COEFSEL (Speaker Amplifier Power Management register) selects the
     * "B" coefficient set, so PDATTB/Effect VOLB apply instead.
     */
    uint8_t sppw = bu_get(s, BU26154_MAP0, BU_REG_SPPW);
    bool coefb = (sppw & 0x08) != 0;          /* COEFSEL b03 */
    uint8_t pd = bu_get(s, BU26154_MAP0, coefb ? 0x39 : BU_REG_PDATT);
    uint8_t ev = bu_get(s, BU26154_MAP0, 0x38);
    uint8_t av = bu_get(s, BU26154_MAP0, BU_REG_AVVOL) & 0x3f;
    uint64_t gp = ((uint64_t)s->pdatt[pd] * s->effect[ev]) >> 15;
    uint64_t ga = s->avvol[av];
    uint64_t g = (gp * ga) >> 15;

    if (g > 0x7fffffffu) {
        g = 0x7fffffffu;
    }

    /*
     * Mono codec: the datasheet has a single DAC/ADC pair, so both
     * halves of the stereo container the SAIF engine carries get the
     * same mono result.  Keep the stereo S16LE framing so the SAIF and
     * the host backend need no special case.
     */
    int32_t l = (int16_t)(frame >> 16);
    int32_t r = (int16_t)(frame & 0xffff);
    int32_t mono = (l + r) / 2;
    int32_t o = (int32_t)(((int64_t)mono * (int64_t)g) >> 15);

    o = MAX(-32768, MIN(32767, o));

    unsigned wp = (s->out_start + s->out_len) % BU_RING;
    s->outbuf[wp] = o & 0xff;
    s->outbuf[(wp + 1) % BU_RING] = (o >> 8) & 0xff;
    s->outbuf[(wp + 2) % BU_RING] = o & 0xff;
    s->outbuf[(wp + 3) % BU_RING] = (o >> 8) & 0xff;
    s->out_len += 4;

    /* Push to the host backend now; without one this is a silent sink. */
    bu26154_out_cb(s, 0);

    if (s->dbg) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (now - s->last_dbg_ns > 1000000000ll) {
            s->last_dbg_ns = now;
            BU26154_DBG(s,
                "dac_in=%" PRIu64 " dac_out=%" PRIu64 " drop=%" PRIu64
                " adc_out=%" PRIu64 " inB=%" PRIu64 " und=%" PRIu64
                " map=%u vmid=0x%02x dacpwr=0x%02x pdatt=0x%02x"
                " avvol=0x%02x be=%d\n",
                s->stats.dac_in_frames, s->stats.dac_out_frames,
                s->stats.dac_dropped, s->stats.adc_out_frames,
                s->stats.adc_in_bytes, s->stats.adc_underrun,
                s->map,
                bu_get(s, BU26154_MAP0, BU_REG_AREFPW),
                bu_get(s, BU26154_MAP0, BU_REG_DACPW),
                pd, av, s->audio_be != NULL);
        }
    }
}

uint32_t bu26154_adc_output(BU26154State *s)
{
    int32_t l = 0, r = 0;

    s->stats.adc_out_frames++;
    if (!s->adc_on) {
        /* ADC/I2S-in powered down: capture clocks in silence. */
        s->stats.adc_gated++;
        return 0;
    }
    if (s->in_len >= 4) {
        uint8_t b0 = s->inbuf[s->in_start];
        uint8_t b1 = s->inbuf[(s->in_start + 1) % BU_RING];
        uint8_t b2 = s->inbuf[(s->in_start + 2) % BU_RING];
        uint8_t b3 = s->inbuf[(s->in_start + 3) % BU_RING];

        l = (int16_t)(b0 | (b1 << 8));
        r = (int16_t)(b2 | (b3 << 8));
        s->in_start = (s->in_start + 4) % BU_RING;
        s->in_len -= 4;
    } else {
        s->stats.adc_underrun++;   /* no host samples yet: silence */
    }

    /*
     * Capture gain chain (datasheet block diagram: MIN -> PGA -> ADC ->
     * ALC -> record digital attenuator -> SAI_SDOUT).  With the mic bias
     * or the microphone amplifier powered down the PGA feeds nothing to
     * the ADC, so a real part clocks out silence regardless of what the
     * host input delivered.  DVMUTE silences the digital stage.
     */
    uint8_t dvm = bu_get(s, BU26154_MAP0, BU_REG_DVMUTE);
    uint32_t g = s->rdvol[bu_get(s, BU26154_MAP0, BU_REG_RDVOL)];

    g = (uint32_t)(((uint64_t)g * s->pga[bu_adc_pgaatt(s)]) >> 15);
    /*
     * The analog microphone volume in front of the PGA: MINVOL[2:0] in
     * the Mic Interface Control register adds 6 dB .. 27 dB in 3 dB
     * steps (datasheet p.55), which the capture level must follow.
     */
    g = (uint32_t)(((uint64_t)g *
                    s->micvol[bu_get(s, BU26154_MAP0, BU_REG_MINIF) & 0x07])
                   >> 15);
    if (!bu_mic_on(s) || (dvm & 0x10)) {
        g = 0;
    }

    l = MAX(-32768, MIN(32767, (int32_t)(((int64_t)l * g) >> 15)));
    r = MAX(-32768, MIN(32767, (int32_t)(((int64_t)r * g) >> 15)));

    return ((uint32_t)(uint16_t)l << 16) | (uint16_t)r;
}

void bu26154_get_stats(BU26154State *s, BU26154Stats *st)
{
    *st = s->stats;
    st->adc_pending = s->in_len;
    st->mapcon = s->map;
    st->sr = bu_get(s, BU26154_MAP0, BU_REG_SR);
    st->vmicon = bu_get(s, BU26154_MAP0, BU_REG_AREFPW) & 0x03;
    st->micben = (bu_get(s, BU26154_MAP0, BU_REG_AREFPW) >> 2) & 0x01;
    st->micbcon = bu_get(s, BU26154_MAP0, BU_REG_MICB) & 0x03;
    st->adc_pwr = bu_get(s, BU26154_MAP0, BU_REG_AINPW);
    st->clken = bu_get(s, BU26154_MAP0, BU_REG_CLKEN);
    st->recplay = bu_get(s, BU26154_MAP0, BU_REG_RECPLAY) & 0x07;
    st->mctime = bu_get(s, BU26154_MAP0, BU_REG_MCTIME) & 0x3f;
    st->minif = bu_get(s, BU26154_MAP0, BU_REG_MINIF);
    st->mct_active = s->mct_expire_ns &&
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->mct_expire_ns;
    st->clk_ok = bu_clock_ok(s);
    st->pgaen = (st->adc_pwr >> 3) & 0x01;
    st->pgaatt = (st->adc_pwr >> 5) & 0x01;
    st->dac_pwr = bu_get(s, BU26154_MAP0, BU_REG_DACPW);
    st->sp_pwr = bu_get(s, BU26154_MAP0, BU_REG_SPPW);
    st->pdatt = bu_get(s, BU26154_MAP0, BU_REG_PDATT);
    st->avvol = bu_get(s, BU26154_MAP0, BU_REG_AVVOL) & 0x3f;
    st->rdvol = bu_get(s, BU26154_MAP0, BU_REG_RDVOL);
    st->avmute = bu_get(s, BU26154_MAP0, BU_REG_AVMUTE);
    st->dvmute = bu_get(s, BU26154_MAP0, BU_REG_DVMUTE);
    st->backend_out = s->audio_be && s->dac_voice;
    st->backend_in = s->audio_be && s->adc_voice;
}

unsigned bu26154_debug_fill(BU26154State *s, uint32_t seed,
                            uint32_t nframes)
{
    unsigned cap = BU_RING / 4;
    unsigned n = MIN(nframes, cap);

    for (unsigned i = 0; i < n; i++) {
        uint32_t l = 0x2000 + ((seed + i) & 0x1fff);
        uint32_t r = 0x2000 + ((seed + (i * 3u)) & 0x1fff);
        unsigned wp = (i * 4) % BU_RING;

        s->inbuf[wp] = l & 0xff;
        s->inbuf[(wp + 1) % BU_RING] = (l >> 8) & 0xff;
        s->inbuf[(wp + 2) % BU_RING] = r & 0xff;
        s->inbuf[(wp + 3) % BU_RING] = (r >> 8) & 0xff;
    }
    s->in_start = 0;
    s->in_len = n * 4;
    /* Accounted separately from adc_in_bytes: those count bytes really
     * read from the host input, these are the analysis aid's stand-in. */
    s->stats.adc_fill_bytes += n * 4;
    return n;
}

uint8_t bu26154_get_reg(BU26154State *s, uint8_t idx)
{
    unsigned w = idx >> 1;

    if (idx == 0x1c || idx == 0x1d) {
        return s->map;
    }
    if (w >= BU26154_WORDS) {
        return 0;
    }
    /* report MAP0 (the map the audio driver uses) */
    return s->regs[BU_REG_IDX(BU26154_MAP0, w)];
}

/* ---- device lifecycle ---- */

static void bu26154_realize(DeviceState *dev, Error **errp)
{
    BU26154State *s = BU26154(dev);
    struct audsettings as = {
        .freq = BU26154_FREQ_HZ,
        .nchannels = 2,
        .fmt = AUDIO_FORMAT_S16,
        .big_endian = false,
    };

    s->dbg = bu26154_debug();
    s->last_dbg_ns = 0;
    bu26154_build_gains(s);
    s->mct_expire_ns = 0;
    s->mct_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bu_mct_tick, s);

    /*
     * The codec is always present on the board (I2C register model).
     * With a host audio backend the DAC path really renders to the host
     * output and the ADC path really captures from the host input.
     * Without -audiodev the register model stays alive and both paths
     * are a silent sink, which is the correct fallback for a machine
     * booted headless.
     *
     * The stream rate follows the SAIF frame clock (BU26154_FREQ_HZ):
     * on real hardware the SAIF supplies LRCLK/BCLK, so the codec's
     * Sampling Rate register (SR[3:0]) is a programming contract the
     * guest must match to that clock.  SR is reported in the stats so a
     * mismatch is visible rather than silently resampled.
     */
    Error *local_err = NULL;

    if (audio_be_check(&s->audio_be, &local_err)) {
        s->dac_voice = audio_be_open_out(s->audio_be, NULL, "bu26154.dac",
                                         s, bu26154_out_cb, &as);
        s->adc_voice = audio_be_open_in(s->audio_be, NULL, "bu26154.adc",
                                        s, bu26154_in_cb, &as);
        audio_be_set_active_out(s->audio_be, s->dac_voice, false);
        audio_be_set_active_in(s->audio_be, s->adc_voice, false);
    } else {
        error_free(local_err);
        s->audio_be = NULL;
        s->dac_voice = NULL;
        s->adc_voice = NULL;
    }
    bu26154_update_paths(s);
}

static void bu26154_exit(DeviceState *dev)
{
    BU26154State *s = BU26154(dev);

    if (s->mct_timer) {
        timer_del(s->mct_timer);
        timer_free(s->mct_timer);
        s->mct_timer = NULL;
    }

    if (s->audio_be) {
        if (s->dac_voice) {
            audio_be_close_out(s->audio_be, s->dac_voice);
        }
        if (s->adc_voice) {
            audio_be_close_in(s->audio_be, s->adc_voice);
        }
    }
}

static int bu26154_post_load(void *opaque, int version_id)
{
    BU26154State *s = opaque;

    /* dac_on/adc_on are derived from the register file; recompute them so
     * the host streams are re-armed to match the restored state. */
    s->dac_on = false;
    s->adc_on = false;
    bu26154_update_paths(s);
    return 0;
}

static const Property bu26154_props[] = {
    DEFINE_AUDIO_PROPERTIES(BU26154State, audio_be),
};

static const VMStateDescription vmstate_bu26154 = {
    .name = "bu26154",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = bu26154_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, BU26154State, 3 * BU26154_WORDS),
        VMSTATE_UINT8(map, BU26154State),
        VMSTATE_UINT8(cur_idx, BU26154State),
        VMSTATE_BOOL(want_idx, BU26154State),
        VMSTATE_UINT32(out_start, BU26154State),
        VMSTATE_UINT32(out_len, BU26154State),
        VMSTATE_UINT32(in_start, BU26154State),
        VMSTATE_UINT32(in_len, BU26154State),
        VMSTATE_END_OF_LIST()
    },
};

static void bu26154_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->realize = bu26154_realize;
    dc->unrealize = bu26154_exit;
    device_class_set_legacy_reset(dc, bu26154_reset);
    dc->vmsd = &vmstate_bu26154;
    device_class_set_props(dc, bu26154_props);

    sc->send = bu26154_i2c_send;
    sc->recv = bu26154_i2c_recv;
    sc->event = bu26154_i2c_event;
}

static const TypeInfo bu26154_types[] = {
    {
        .name = TYPE_BU26154,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(BU26154State),
        .class_init = bu26154_class_init,
        .abstract = false,
    },
};

DEFINE_TYPES(bu26154_types)
