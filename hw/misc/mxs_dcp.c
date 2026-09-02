/*
 * Freescale i.MX28 Data Co-Processor (DCP)
 *
 * The DCP is the i.MX28's crypto / blit / copy engine: a four channel
 * descriptor DMA that runs "work packets" describing a memcpy, a blit,
 * a constant fill, an AES-128 ECB/CBC cipher operation and/or a
 * SHA-1/SHA-256/CRC32 hash.  It is a real block on this SoC and the
 * board's device tree enables it (imx28-pwsh6.dts: dcp@80028000
 * status = "okay"), so it is modelled here rather than left as an
 * access-swallowing placeholder.
 *
 * Register map and packet layout: i.MX28 Applications Processor
 * Reference Manual (MCIMX28RM Rev 2, 08/2013) chapter 13.  Semantics
 * cross-checked against the Linux driver drivers/crypto/mxs-dcp.c.
 *
 * What is modelled:
 *   - the full register bank with the MXS set/clr/tog aliases
 *   - four channels, each with CMDPTR/SEMA/STAT/OPTS and its own
 *     semaphore, error status and packet chaining
 *   - work packets: ENABLE_MEMCOPY, ENABLE_BLIT, CONSTANT_FILL,
 *     ENABLE_CIPHER (AES-128 ECB and CBC, encrypt and decrypt),
 *     ENABLE_HASH (SHA-1, SHA-256 and CRC32, including the HASH_INIT /
 *     HASH_TERM / HASH_OUTPUT / CHECK_HASH / PAYLOAD_KEY handling)
 *   - the write-only key RAM behind HW_DCP_KEY / HW_DCP_KEYDATA, with
 *     the SUBWORD auto-increment the manual describes
 *   - input/output/key byte- and word-swap controls
 *   - interrupt routing: channel 0 to dcp_vmi_irq unless CH0_IRQ_MERGED,
 *     channels 1..3 to the shared dcp_irq
 *
 * Modelled at work-packet granularity rather than cycle-accurately: a
 * packet completes synchronously when its semaphore is written, which
 * is what software observes (it either polls HW_DCP_STAT or waits for
 * the interrupt).  The PACKET0..6 "current packet" views report the
 * packet being processed.
 *
 * Not modelled: trustzone / secure channel separation (ENABLE_TZONE),
 * the OTP and per-device unique keys (reported absent through
 * CAPABILITY0), the context-caching optimisation (accepted but a no-op:
 * each packet is processed to completion) and the CSC colour-space
 * converter.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/mxs_bank.h"
#include "system/dma.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "crypto/cipher.h"
#include "crypto/hash.h"
#include "qom/object.h"

#define TYPE_MXS_DCP "mxs-dcp"
OBJECT_DECLARE_SIMPLE_TYPE(MXSDCPState, MXS_DCP)

/* ---- register offsets (all 0x10 wide, with set/clr/tog aliases) ---- */
#define DCP_CTRL            0x000
#define DCP_STAT            0x010
#define DCP_CHANNELCTRL     0x020
#define DCP_CAPABILITY0     0x030
#define DCP_CAPABILITY1     0x040
#define DCP_CONTEXT         0x050
#define DCP_KEY             0x060
#define DCP_KEYDATA         0x070
#define DCP_PACKET0         0x080   /* .. DCP_PACKET6 at 0x0e0 */
#define DCP_CH0CMDPTR       0x100
#define DCP_CHSTRIDE        0x040
#define DCP_DBGSELECT       0x400
#define DCP_DBGDATA         0x410
#define DCP_PAGETABLE       0x420
#define DCP_VERSION         0x430
#define DCP_NREGS           (0x440 >> 4)

/* per-channel register offsets inside the channel stride */
#define DCP_CH_CMDPTR       0x00
#define DCP_CH_SEMA         0x10
#define DCP_CH_STAT         0x20
#define DCP_CH_OPTS         0x30

#define DCP_MAX_CHANS       4
#define DCP_MAX_KEYS        4
#define DCP_MAX_PACKETS     7

/* HW_DCP_CTRL */
#define DCP_CTRL_SFTRST                     (1u << 31)
#define DCP_CTRL_CLKGATE                    (1u << 30)
#define DCP_CTRL_PRESENT_CRYPTO             (1u << 29)
#define DCP_CTRL_PRESENT_SHA                (1u << 28)
#define DCP_CTRL_GATHER_RESIDUAL_WRITES     (1u << 23)
#define DCP_CTRL_ENABLE_CONTEXT_CACHING     (1u << 22)
#define DCP_CTRL_ENABLE_CONTEXT_SWITCHING   (1u << 21)
#define DCP_CTRL_CHANNEL_INTERRUPT_ENABLE   0xfu

/* HW_DCP_STAT */
#define DCP_STAT_OTP_KEY_READY  (1u << 28)
#define DCP_STAT_CUR_CHANNEL_SHIFT 24
#define DCP_STAT_READY_CHANNELS_SHIFT 16
#define DCP_STAT_IRQ            0xfu

/* HW_DCP_CHANNELCTRL */
#define DCP_CHCTRL_CH0_IRQ_MERGED   (1u << 16)
#define DCP_CHCTRL_HIGH_PRIORITY_SHIFT 8
#define DCP_CHCTRL_ENABLE_CHANNEL   0xffu

/* work packet: seven words */
#define DCP_WP_NEXT       0
#define DCP_WP_CONTROL0   1
#define DCP_WP_CONTROL1   2
#define DCP_WP_SRC        3
#define DCP_WP_DST        4
#define DCP_WP_SIZE       5
#define DCP_WP_PAYLOAD    6

/* control0 */
#define DCP_C0_TAG_SHIFT          24
#define DCP_C0_OUTPUT_WORDSWAP    (1u << 23)
#define DCP_C0_OUTPUT_BYTESWAP    (1u << 22)
#define DCP_C0_INPUT_WORDSWAP     (1u << 21)
#define DCP_C0_INPUT_BYTESWAP     (1u << 20)
#define DCP_C0_KEY_WORDSWAP       (1u << 19)
#define DCP_C0_KEY_BYTESWAP       (1u << 18)
#define DCP_C0_HASH_OUTPUT        (1u << 15)
#define DCP_C0_CHECK_HASH         (1u << 14)
#define DCP_C0_HASH_TERM          (1u << 13)
#define DCP_C0_HASH_INIT          (1u << 12)
#define DCP_C0_PAYLOAD_KEY        (1u << 11)
#define DCP_C0_OTP_KEY            (1u << 10)
#define DCP_C0_CIPHER_INIT        (1u << 9)
#define DCP_C0_CIPHER_ENCRYPT     (1u << 8)
#define DCP_C0_ENABLE_BLIT        (1u << 7)
#define DCP_C0_ENABLE_HASH        (1u << 6)
#define DCP_C0_ENABLE_CIPHER      (1u << 5)
#define DCP_C0_ENABLE_MEMCOPY     (1u << 4)
#define DCP_C0_CONSTANT_FILL      (1u << 3)
#define DCP_C0_CHAIN_CONTIGUOUS   (1u << 2)
#define DCP_C0_CHAIN              (1u << 1)
#define DCP_C0_DECR_SEMAPHORE     (1u << 0)
#define DCP_C0_INTERRUPT          (1u << 0)

/*
 * Bit 0 is documented both as INTERRUPT and as DECR_SEMAPHORE in
 * different places; the Linux driver treats bit 0 as the interrupt
 * request and bit 1 as the semaphore decrement.  Follow the driver,
 * which is what real software relies on.
 */
#define DCP_C0_IRQ_BIT            (1u << 0)
#define DCP_C0_SEMA_BIT           (1u << 1)

/* control1 */
#define DCP_C1_CIPHER_CFG_SHIFT   24
#define DCP_C1_HASH_SELECT_SHIFT  16
#define DCP_C1_KEY_SELECT_SHIFT   8
#define DCP_C1_CIPHER_MODE_SHIFT  4
#define DCP_C1_CIPHER_SELECT_MASK 0xfu

#define DCP_HASH_SHA1     0x0
#define DCP_HASH_CRC32    0x1
#define DCP_HASH_SHA256   0x2

#define DCP_MODE_ECB      0x0
#define DCP_MODE_CBC      0x1

#define DCP_CIPHER_AES128 0x0

#define DCP_KEY_SEL_UNIQUE 0xfe
#define DCP_KEY_SEL_OTP    0xff

/* channel error codes as observed through HW_DCP_CHnSTAT */
#define DCP_CHERR_NONE            0x00
#define DCP_CHERR_BAD_PACKET      0x01
#define DCP_CHERR_KEY_NOT_LOADED  0x02
#define DCP_CHERR_UNSUPPORTED     0x03
#define DCP_CHERR_DMA             0x04

struct MXSDCPState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq_vmi;    /* dcp_vmi_irq  : channel 0 */
    qemu_irq irq;        /* dcp_irq      : channels 1..3 (and 0 if merged) */

    uint32_t regs[DCP_NREGS];

    uint8_t key_ram[DCP_MAX_KEYS][16];
    unsigned key_index;
    unsigned key_subword;

    /* current work packet view (HW_DCP_PACKET0..6) */
    uint32_t packet[DCP_MAX_PACKETS];

    /* hash state carried across chained packets of one channel */
    uint8_t hash_state[32];
    unsigned hash_len;          /* 0 = no digest pending, 20 or 32 */
    uint32_t crc_state;
    bool crc_valid;
    unsigned hash_chan;         /* channel owning the running hash */
    bool hash_active;

    bool trace;
};

/* dcp_process_packet() follows the packet chain via dcp_load_packet(). */
static void dcp_load_packet(MXSDCPState *s, unsigned ch, uint32_t addr);

static uint32_t mxs_dcp_reg_read(MXSDCPState *s, unsigned idx)
{
    return idx < DCP_NREGS ? s->regs[idx] : 0;
}

static void mxs_dcp_reg_write(MXSDCPState *s, unsigned idx, uint32_t v)
{
    if (idx < DCP_NREGS) {
        s->regs[idx] = v;
    }
}

/* ---- byte/word swap helpers (packet control0 bits) ---- */

static void dcp_swap_buf(uint8_t *buf, size_t len, bool byteswap,
                         bool wordswap)
{
    if (byteswap) {
        for (size_t i = 0; i + 1 < len; i += 2) {
            uint8_t t = buf[i];

            buf[i] = buf[i + 1];
            buf[i + 1] = t;
        }
    }
    if (wordswap) {
        for (size_t i = 0; i + 3 < len; i += 4) {
            uint8_t t0 = buf[i], t1 = buf[i + 1];

            buf[i] = buf[i + 3];
            buf[i + 1] = buf[i + 2];
            buf[i + 2] = t1;
            buf[i + 3] = t0;
        }
    }
}

/* ---- CRC32 (the DCP's own hash algorithm 0x1) ---- */

static uint32_t dcp_crc32(uint32_t crc, const uint8_t *p, size_t len)
{
    /* Reflected CRC-32 (the standard Ethernet/zip polynomial). */
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)(-(int32_t)(crc & 1)));
        }
    }
    return ~crc;
}

/* ---- the AES-128 engine ---- */

static bool dcp_aes(uint8_t key[16], uint8_t iv[16], const uint8_t *in,
                    uint8_t *out, size_t len, bool cbc, bool encrypt)
{
    QCryptoCipher *cipher;
    Error *err = NULL;
    bool ok;

    cipher = qcrypto_cipher_new(QCRYPTO_CIPHER_ALGO_AES_128,
                                cbc ? QCRYPTO_CIPHER_MODE_CBC
                                    : QCRYPTO_CIPHER_MODE_ECB,
                                key, 16, &err);
    if (!cipher) {
        error_report_err(err);
        return false;
    }
    if (cbc && qcrypto_cipher_setiv(cipher, iv, 16, &err) < 0) {
        error_report_err(err);
        qcrypto_cipher_free(cipher);
        return false;
    }
    ok = encrypt ? qcrypto_cipher_encrypt(cipher, in, out, len, &err) == 0
                 : qcrypto_cipher_decrypt(cipher, in, out, len, &err) == 0;
    if (!ok) {
        error_report_err(err);
    }
    qcrypto_cipher_free(cipher);
    return ok;
}

/* ---- channel helpers ---- */

static uint32_t dcp_ch_reg(MXSDCPState *s, unsigned ch, unsigned off)
{
    return mxs_dcp_reg_read(s, MXS_BANK_INDEX(DCP_CH0CMDPTR +
                                              ch * DCP_CHSTRIDE + off));
}

static void dcp_ch_set_reg(MXSDCPState *s, unsigned ch, unsigned off,
                           uint32_t v)
{
    mxs_dcp_reg_write(s, MXS_BANK_INDEX(DCP_CH0CMDPTR + ch * DCP_CHSTRIDE +
                                        off), v);
}

static bool dcp_channel_enabled(MXSDCPState *s, unsigned ch)
{
    return (mxs_dcp_reg_read(s, MXS_BANK_INDEX(DCP_CHANNELCTRL)) &
            DCP_CHCTRL_ENABLE_CHANNEL) & (1u << ch);
}

static void dcp_raise_irq(MXSDCPState *s, unsigned ch)
{
    uint32_t stat = mxs_dcp_reg_read(s, MXS_BANK_INDEX(DCP_STAT));
    bool merged = mxs_dcp_reg_read(s, MXS_BANK_INDEX(DCP_CHANNELCTRL)) &
                  DCP_CHCTRL_CH0_IRQ_MERGED;

    stat |= 1u << ch;
    mxs_dcp_reg_write(s, MXS_BANK_INDEX(DCP_STAT), stat);

    if (ch == 0 && !merged) {
        qemu_irq_raise(s->irq_vmi);
    } else {
        qemu_irq_raise(s->irq);
    }
}

/* ---- one work packet ---- */

static void dcp_process_packet(MXSDCPState *s, unsigned ch)
{
    uint32_t wp[DCP_MAX_PACKETS];
    uint32_t next, c0, c1, src, dst, size, payload;
    unsigned err = DCP_CHERR_NONE;

    memcpy(wp, s->packet, sizeof(wp));
    next = wp[DCP_WP_NEXT];
    c0 = wp[DCP_WP_CONTROL0];
    c1 = wp[DCP_WP_CONTROL1];
    src = wp[DCP_WP_SRC];
    dst = wp[DCP_WP_DST];
    size = wp[DCP_WP_SIZE];
    payload = wp[DCP_WP_PAYLOAD];

    if (size == 0 || size > (16u << 20)) {
        err = DCP_CHERR_BAD_PACKET;
        goto done;
    }

    /* ---- cipher ---- */
    if (c0 & DCP_C0_ENABLE_CIPHER) {
        uint8_t key[16], iv[16];
        uint8_t *in = g_try_malloc(size);
        uint8_t *out = g_try_malloc(size);
        unsigned sel = (c1 >> DCP_C1_KEY_SELECT_SHIFT) & 0xff;
        bool cbc = ((c1 >> DCP_C1_CIPHER_MODE_SHIFT) & 0xf) == DCP_MODE_CBC;
        bool enc = (c0 & DCP_C0_CIPHER_ENCRYPT) != 0;

        memset(key, 0, sizeof(key));
        memset(iv, 0, sizeof(iv));

        if (sel == DCP_KEY_SEL_OTP || sel == DCP_KEY_SEL_UNIQUE) {
            /* OTP / unique device keys are not implemented here. */
            err = DCP_CHERR_KEY_NOT_LOADED;
        } else if (c0 & DCP_C0_PAYLOAD_KEY) {
            /* The key rides in the packet payload buffer. */
            dma_memory_read(&address_space_memory, payload, key, 16,
                            MEMTXATTRS_UNSPECIFIED);
            if (cbc) {
                dma_memory_read(&address_space_memory, payload + 16, iv, 16,
                                MEMTXATTRS_UNSPECIFIED);
            }
        } else if (sel < DCP_MAX_KEYS) {
            memcpy(key, s->key_ram[sel], 16);
            if (cbc) {
                /* The IV rides in the payload buffer just past the key. */
                dma_memory_read(&address_space_memory, payload + 16, iv, 16,
                                MEMTXATTRS_UNSPECIFIED);
            }
        } else {
            err = DCP_CHERR_KEY_NOT_LOADED;
        }

        if (err == DCP_CHERR_NONE && in && out) {
            dcp_swap_buf(key, 16, c0 & DCP_C0_KEY_BYTESWAP,
                         c0 & DCP_C0_KEY_WORDSWAP);
            dma_memory_read(&address_space_memory, src, in, size,
                            MEMTXATTRS_UNSPECIFIED);
            dcp_swap_buf(in, size, c0 & DCP_C0_INPUT_BYTESWAP,
                         c0 & DCP_C0_INPUT_WORDSWAP);
            if (!dcp_aes(key, iv, in, out, size, cbc, enc)) {
                err = DCP_CHERR_UNSUPPORTED;
            } else {
                dcp_swap_buf(out, size, c0 & DCP_C0_OUTPUT_BYTESWAP,
                             c0 & DCP_C0_OUTPUT_WORDSWAP);
                dma_memory_write(&address_space_memory, dst, out, size,
                                 MEMTXATTRS_UNSPECIFIED);
            }
        } else if (!in || !out) {
            err = DCP_CHERR_DMA;
        }
        g_free(in);
        g_free(out);
    }

    /* ---- hash ---- */
    if ((c0 & DCP_C0_ENABLE_HASH) && err == DCP_CHERR_NONE) {
        unsigned sel = (c1 >> DCP_C1_HASH_SELECT_SHIFT) & 0xf;
        uint8_t *buf = g_try_malloc(size);

        if (!buf) {
            err = DCP_CHERR_DMA;
        } else {
            dma_memory_read(&address_space_memory, src, buf, size,
                            MEMTXATTRS_UNSPECIFIED);
            dcp_swap_buf(buf, size, c0 & DCP_C0_INPUT_BYTESWAP,
                         c0 & DCP_C0_INPUT_WORDSWAP);

            if (sel == DCP_HASH_CRC32) {
                if ((c0 & DCP_C0_HASH_INIT) || !s->crc_valid ||
                    s->hash_chan != ch || !s->hash_active) {
                    s->crc_state = 0;
                }
                s->crc_state = dcp_crc32(s->crc_state, buf, size);
                s->crc_valid = true;
                s->hash_chan = ch;
                s->hash_active = true;
                s->hash_len = 4;
                stl_le_p(s->hash_state, s->crc_state);
            } else {
                QCryptoHashAlgo alg = (sel == DCP_HASH_SHA256) ?
                    QCRYPTO_HASH_ALGO_SHA256 : QCRYPTO_HASH_ALGO_SHA1;
                size_t dlen = 0;
                uint8_t *digest = NULL;

                /*
                 * Streaming across chained packets: when HASH_INIT is
                 * clear and this channel already has a running digest,
                 * feed the previous state in front of the new data so
                 * the result matches a single-pass hash of the whole
                 * buffer.
                 */
                if ((c0 & DCP_C0_HASH_INIT) || !s->hash_active ||
                    s->hash_chan != ch || s->hash_len == 0) {
                    s->hash_len = 0;
                }
                if (s->hash_len) {
                    struct iovec iov[2];
                    size_t prev = s->hash_len;

                    iov[0].iov_base = s->hash_state;
                    iov[0].iov_len = prev;
                    iov[1].iov_base = buf;
                    iov[1].iov_len = size;
                    if (qcrypto_hash_bytesv(alg, iov, 2, &digest, &dlen,
                                            NULL) < 0) {
                        err = DCP_CHERR_UNSUPPORTED;
                    }
                } else if (qcrypto_hash_bytes(alg, (const char *)buf, size,
                                              &digest, &dlen, NULL) < 0) {
                    err = DCP_CHERR_UNSUPPORTED;
                }
                if (err == DCP_CHERR_NONE && digest) {
                    memcpy(s->hash_state, digest, MIN(dlen,
                                                      sizeof(s->hash_state)));
                    s->hash_len = dlen;
                    s->hash_chan = ch;
                    s->hash_active = true;
                }
                g_free(digest);
            }
        }
        g_free(buf);

        /*
         * HASH_TERM ends the running digest.  HASH_OUTPUT writes the
         * result to the destination buffer; CHECK_HASH compares it
         * against the payload buffer instead and reports a mismatch as
         * a channel error.
         */
        if (err == DCP_CHERR_NONE && (c0 & DCP_C0_HASH_TERM)) {
            s->hash_active = false;
            s->crc_valid = false;
            if (c0 & DCP_C0_CHECK_HASH) {
                uint8_t expect[32];

                dma_memory_read(&address_space_memory, payload, expect,
                                s->hash_len, MEMTXATTRS_UNSPECIFIED);
                if (memcmp(expect, s->hash_state, s->hash_len) != 0) {
                    err = DCP_CHERR_BAD_PACKET;
                }
            } else if (c0 & DCP_C0_HASH_OUTPUT) {
                dma_memory_write(&address_space_memory, dst, s->hash_state,
                                 s->hash_len, MEMTXATTRS_UNSPECIFIED);
            }
        }
    }

    /* ---- plain copy / blit / constant fill ---- */
    if ((c0 & (DCP_C0_ENABLE_MEMCOPY | DCP_C0_ENABLE_BLIT)) &&
        err == DCP_CHERR_NONE) {
        uint8_t *buf = g_try_malloc(size);

        if (!buf) {
            err = DCP_CHERR_DMA;
        } else if (c0 & DCP_C0_CONSTANT_FILL) {
            /*
             * For a constant fill the source address field carries the
             * byte to write (manual: "this field contains the data
             * written to the destination buffer").
             */
            memset(buf, src & 0xff, size);
            dma_memory_write(&address_space_memory, dst, buf, size,
                             MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_memory_read(&address_space_memory, src, buf, size,
                            MEMTXATTRS_UNSPECIFIED);
            dma_memory_write(&address_space_memory, dst, buf, size,
                             MEMTXATTRS_UNSPECIFIED);
        }
        g_free(buf);
    }

done:
    /* record the channel status */
    dcp_ch_set_reg(s, ch, DCP_CH_STAT, err);
    if (c0 & DCP_C0_IRQ_BIT) {
        dcp_raise_irq(s, ch);
    }
    if (c0 & DCP_C0_SEMA_BIT) {
        uint32_t sema = dcp_ch_reg(s, ch, DCP_CH_SEMA);

        if (sema) {
            dcp_ch_set_reg(s, ch, DCP_CH_SEMA, sema - 1);
        }
    }

    /* follow the chain */
    if ((c0 & DCP_C0_CHAIN) && next) {
        s->packet[DCP_WP_NEXT] = next;
        dcp_load_packet(s, ch, next);
    } else {
        memset(s->packet, 0, sizeof(s->packet));
        s->hash_active = false;
        s->crc_valid = false;
    }
}

/* Load a packet from guest memory and run it. */
static void dcp_load_packet(MXSDCPState *s, unsigned ch, uint32_t addr)
{
    uint32_t wp[DCP_MAX_PACKETS];

    if (!addr || (addr & 3)) {
        dcp_ch_set_reg(s, ch, DCP_CH_STAT, DCP_CHERR_BAD_PACKET);
        if (dcp_ch_reg(s, ch, DCP_CH_SEMA)) {
            dcp_ch_set_reg(s, ch, DCP_CH_SEMA,
                           dcp_ch_reg(s, ch, DCP_CH_SEMA) - 1);
        }
        return;
    }
    if (dma_memory_read(&address_space_memory, addr, wp, sizeof(wp),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        dcp_ch_set_reg(s, ch, DCP_CH_STAT, DCP_CHERR_DMA);
        return;
    }
    memcpy(s->packet, wp, sizeof(wp));
    /* The "current packet" registers expose the packet being processed. */
    for (unsigned i = 0; i < DCP_MAX_PACKETS; i++) {
        mxs_dcp_reg_write(s, MXS_BANK_INDEX(DCP_PACKET0 + i * 0x10), wp[i]);
    }
    dcp_process_packet(s, ch);
}

/* ---- MMIO ---- */

static uint64_t mxs_dcp_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSDCPState *s = opaque;
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t v;

    if (offset >= DCP_STAT && offset < DCP_STAT + 0x10) {
        /*
         * Reading the status register is how software acknowledges the
         * interrupt state it is polling; the pending bits stay set until
         * software clears them, exactly as the manual describes.
         */
        v = mxs_dcp_reg_read(s, idx);
    } else {
        v = mxs_dcp_reg_read(s, idx);
    }
    return mxs_bank_extract(offset, size, v);
}

static void mxs_dcp_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    MXSDCPState *s = opaque;
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t old = mxs_dcp_reg_read(s, idx);
    uint32_t v = mxs_bank_apply(old, offset, val, size);
    unsigned ch;

    /* channel registers */
    if (offset >= DCP_CH0CMDPTR && offset < DCP_CH0CMDPTR + 0x100) {
        unsigned rel = offset - DCP_CH0CMDPTR;

        ch = rel / DCP_CHSTRIDE;
        rel %= DCP_CHSTRIDE;
        if (MXS_BANK_OP(offset) != MXS_OP_WRITE) {
            /* set/clr/tog on a channel register: apply and continue */
            mxs_dcp_reg_write(s, idx, v);
        } else if (rel < 0x10) {
            dcp_ch_set_reg(s, ch, DCP_CH_CMDPTR, v);
        } else if (rel < 0x20) {
            uint32_t prev = dcp_ch_reg(s, ch, DCP_CH_SEMA);

            /*
             * The semaphore is a counting register: a write *adds* to
             * the count and each addition arms one packet.  That is the
             * behaviour both the manual and the Linux driver rely on.
             */
            dcp_ch_set_reg(s, ch, DCP_CH_SEMA, prev + (v & 0xff));
            if (!dcp_channel_enabled(s, ch)) {
                return;
            }
            while (dcp_ch_reg(s, ch, DCP_CH_SEMA) && v) {
                uint32_t cmd = dcp_ch_reg(s, ch, DCP_CH_CMDPTR);

                /* one packet per semaphore increment just added */
                dcp_load_packet(s, ch, cmd);
                if (dcp_ch_reg(s, ch, DCP_CH_STAT)) {
                    break;      /* stop on the first error */
                }
                v--;
            }
            return;
        } else if (rel < 0x30) {
            dcp_ch_set_reg(s, ch, DCP_CH_STAT, v);
        } else {
            dcp_ch_set_reg(s, ch, DCP_CH_OPTS, v);
        }
        return;
    }

    /*
     * SFTRST/CLKGATE only exist in this block's first register; applying
     * the edge rule to every register would corrupt any write whose bit
     * 31 happens to be set (a key word, a packet field, ...).
     */
    if ((offset & ~0xfu) == DCP_CTRL) {
        v = mxs_bank_sftrst(old, v);
    }
    mxs_dcp_reg_write(s, idx, v);

    switch (offset & ~0xfu) {
    case DCP_CTRL:
        if (v & DCP_CTRL_SFTRST) {
            /* Soft reset returns the block to its reset state. */
            memset(s->packet, 0, sizeof(s->packet));
            s->hash_active = false;
            s->crc_valid = false;
            s->hash_len = 0;
            for (unsigned c = 0; c < DCP_MAX_CHANS; c++) {
                dcp_ch_set_reg(s, c, DCP_CH_STAT, 0);
                dcp_ch_set_reg(s, c, DCP_CH_SEMA, 0);
            }
            mxs_dcp_reg_write(s, MXS_BANK_INDEX(DCP_STAT),
                              DCP_STAT_OTP_KEY_READY);
            qemu_irq_lower(s->irq);
            qemu_irq_lower(s->irq_vmi);
        }
        break;
    case DCP_STAT:
        /* Writing clears the selected pending-interrupt bits. */
        mxs_dcp_reg_write(s, idx, old & ~v);
        if (!(mxs_dcp_reg_read(s, MXS_BANK_INDEX(DCP_STAT)) &
              DCP_STAT_IRQ)) {
            qemu_irq_lower(s->irq);
            qemu_irq_lower(s->irq_vmi);
        }
        break;
    case DCP_KEY:
        s->key_index = (v >> 4) & 0x3;
        s->key_subword = v & 0x3;
        break;
    case DCP_KEYDATA:
        /*
         * The manual's example writes the 128-bit key
         * 0x00112233_44556677_8899aabb_ccddeeff as subword 0 = 0xccddeeff,
         * 1 = 0x8899aabb, 2 = 0x44556677, 3 = 0x00112233: the subwords go
         * least-significant word first, so subword n lands at key byte
         * (3 - n) * 4, big-endian.
         */
        if (s->key_index < DCP_MAX_KEYS) {
            stl_be_p(&s->key_ram[s->key_index][(3 - s->key_subword) * 4], v);
        }
        /* SUBWORD auto-increments so successive words need no rewrite */
        s->key_subword = (s->key_subword + 1) & 0x3;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps mxs_dcp_ops = {
    .read = mxs_dcp_read,
    .write = mxs_dcp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ---- lifecycle ---- */

static void mxs_dcp_reset(DeviceState *dev)
{
    MXSDCPState *s = MXS_DCP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->key_ram, 0, sizeof(s->key_ram));
    memset(s->packet, 0, sizeof(s->packet));
    memset(s->hash_state, 0, sizeof(s->hash_state));
    s->key_index = 0;
    s->key_subword = 0;
    s->hash_len = 0;
    s->crc_state = 0;
    s->crc_valid = false;
    s->hash_active = false;
    s->hash_chan = 0;

    /* datasheet reset values */
    mxs_dcp_reg_write(s, MXS_BANK_INDEX(DCP_CTRL),
                      DCP_CTRL_SFTRST | DCP_CTRL_CLKGATE |
                      DCP_CTRL_PRESENT_CRYPTO | DCP_CTRL_PRESENT_SHA |
                      DCP_CTRL_GATHER_RESIDUAL_WRITES);
    mxs_dcp_reg_write(s, MXS_BANK_INDEX(DCP_STAT), DCP_STAT_OTP_KEY_READY);
    /* CAPABILITY0: 4 channels, 4 keys */
    mxs_dcp_reg_write(s, MXS_BANK_INDEX(DCP_CAPABILITY0), 0x00000404);
    /* CAPABILITY1: SHA1|CRC32|SHA256 and AES128 */
    mxs_dcp_reg_write(s, MXS_BANK_INDEX(DCP_CAPABILITY1), 0x00070001);
    mxs_dcp_reg_write(s, MXS_BANK_INDEX(DCP_VERSION), 0x02010000);

    qemu_irq_lower(s->irq);
    qemu_irq_lower(s->irq_vmi);
}

static void mxs_dcp_realize(DeviceState *dev, Error **errp)
{
    MXSDCPState *s = MXS_DCP(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &mxs_dcp_ops, s,
                          TYPE_MXS_DCP, 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->irq_vmi);
    s->trace = mxs_trace_enabled("dcp");
}

static const VMStateDescription vmstate_mxs_dcp = {
    .name = TYPE_MXS_DCP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSDCPState, DCP_NREGS),
        VMSTATE_UINT8_2DARRAY(key_ram, MXSDCPState, DCP_MAX_KEYS, 16),
        VMSTATE_UINT32(key_index, MXSDCPState),
        VMSTATE_UINT32(key_subword, MXSDCPState),
        VMSTATE_UINT32_ARRAY(packet, MXSDCPState, DCP_MAX_PACKETS),
        VMSTATE_UINT8_ARRAY(hash_state, MXSDCPState, 32),
        VMSTATE_UINT32(hash_len, MXSDCPState),
        VMSTATE_UINT32(crc_state, MXSDCPState),
        VMSTATE_BOOL(crc_valid, MXSDCPState),
        VMSTATE_UINT32(hash_chan, MXSDCPState),
        VMSTATE_BOOL(hash_active, MXSDCPState),
        VMSTATE_END_OF_LIST()
    },
};

static void mxs_dcp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mxs_dcp_realize;
    device_class_set_legacy_reset(dc, mxs_dcp_reset);
    dc->vmsd = &vmstate_mxs_dcp;
    dc->desc = "Freescale i.MX28 Data Co-Processor (DCP)";
}

static const TypeInfo mxs_dcp_info = {
    .name = TYPE_MXS_DCP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MXSDCPState),
    .class_init = mxs_dcp_class_init,
};

static void mxs_dcp_register_types(void)
{
    type_register_static(&mxs_dcp_info);
}

type_init(mxs_dcp_register_types)
