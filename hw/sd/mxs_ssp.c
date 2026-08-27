/*
 * Freescale i.MX28 (MXS) SSP controller in SD/MMC mode
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/sd/sd.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/core/qdev-properties.h"

#define SSP_CTRL0       0x0
#define SSP_CMD0        0x1
#define SSP_CMD1        0x2
#define SSP_XFER_SIZE   0x3
#define SSP_BLOCK_SIZE  0x4
#define SSP_COMPREF     0x5
#define SSP_COMPMASK    0x6
#define SSP_TIMING      0x7
#define SSP_CTRL1       0x8
#define SSP_DATA        0x9
#define SSP_SDRESP0     0xa
#define SSP_SDRESP1     0xb
#define SSP_SDRESP2     0xc
#define SSP_SDRESP3     0xd
#define SSP_DDR_CTRL    0xe
#define SSP_DLL_CTRL    0xf
#define SSP_STATUS      0x10
#define SSP_DLL_STS     0x11
#define SSP_DEBUG       0x12
#define SSP_VERSION     0x13

#define CTRL0_SFTRST        (1u << 31)
#define CTRL0_CLKGATE       (1u << 30)
#define CTRL0_RUN           (1u << 29)
#define CTRL0_LOCK_CS       (1u << 27)
#define CTRL0_IGNORE_CRC    (1u << 26)
#define CTRL0_READ          (1u << 25)
#define CTRL0_DATA_XFER     (1u << 24)
#define CTRL0_BUS_WIDTH(v)  (((v) >> 22) & 3)
#define CTRL0_WAIT_FOR_IRQ  (1u << 21)
#define CTRL0_LONG_RESP     (1u << 19)
#define CTRL0_CHECK_RESP    (1u << 18)
#define CTRL0_GET_RESP      (1u << 17)
#define CTRL0_ENABLE        (1u << 16)
#define CTRL0_XFER_COUNT(v) ((v) & 0xffff)

/* HW_SSP_BLOCK_SIZE: BLOCK_COUNT[27:4] holds (number of blocks - 1) */
#define SSP_BLOCK_COUNT(v)  (((v) >> 4) & 0xffffff)
#define SSP_BLOCK_LOG2(v)   ((v) & 0xf)

#define CTRL1_SDIO_IRQ          (1u << 31)
#define CTRL1_RESP_ERR_IRQ      (1u << 29)
#define CTRL1_RESP_TIMEOUT_IRQ  (1u << 27)
#define CTRL1_DATA_TIMEOUT_IRQ  (1u << 25)
#define CTRL1_DATA_CRC_IRQ      (1u << 23)
#define CTRL1_FIFO_UNDERRUN_IRQ (1u << 21)
#define CTRL1_RECV_TIMEOUT_IRQ  (1u << 17)
#define CTRL1_FIFO_OVERRUN_IRQ  (1u << 15)
/*
 * SSP_END_CMD / SSP_END_CMD_EN (i.MX23/28 SSP_CTRL1 bits 19/18): the
 * hardware raises END_CMD when a command sequence has finished (with or
 * without a response).  Model it: set at the end of a command, cleared
 * when a new command starts.
 *
 * Note (S74, 2026-08-22): the "ERROR: Failed to read MBR from SDHC" seen
 * at every boot is NOT a failure of the eMMC (slot 0) MBR read -- that
 * read is verified byte-exact against the image.  It is the MBR probe of
 * the second SSP slot (microSD, no card on the Brain), which gets no
 * response (rlen=0).  A/B verified (BRAIN_SSP_ENDCMD off) that the
 * slot-0 MBR read and the whole boot behave identically with or without
 * this bit, so it is kept purely for hardware fidelity, not because the
 * WinCE driver depends on it.
 */
#define CTRL1_END_CMD_IRQ       (1u << 19)
#define CTRL1_END_CMD_IRQ_EN    (1u << 18)
#define CTRL1_DMA_ENABLE        (1u << 13)
#define CTRL1_IRQ_MASK          (CTRL1_SDIO_IRQ | CTRL1_RESP_ERR_IRQ | \
                                 CTRL1_RESP_TIMEOUT_IRQ | \
                                 CTRL1_DATA_TIMEOUT_IRQ | \
                                 CTRL1_DATA_CRC_IRQ | \
                                 CTRL1_FIFO_UNDERRUN_IRQ | \
                                 CTRL1_RECV_TIMEOUT_IRQ | \
                                 CTRL1_FIFO_OVERRUN_IRQ)

#define STATUS_PRESENT          (1u << 31)
#define STATUS_MS_PRESENT       (1u << 30)
#define STATUS_SD_PRESENT       (1u << 29)
#define STATUS_CARD_DETECT      (1u << 28)
#define STATUS_DMASENSE         (1u << 21)
#define STATUS_DMATERM          (1u << 20)
#define STATUS_DMAREQ           (1u << 19)
#define STATUS_DMAEND           (1u << 18)
#define STATUS_SDIO_IRQ         (1u << 17)
#define STATUS_RESP_CRC_ERR     (1u << 16)
#define STATUS_RESP_ERR         (1u << 15)
#define STATUS_RESP_TIMEOUT     (1u << 14)
#define STATUS_DATA_CRC_ERR     (1u << 13)
#undef STATUS_TIMEOUT
#define STATUS_TIMEOUT          (1u << 12)
#define STATUS_RECV_TIMEOUT     (1u << 11)
#define STATUS_FIFO_OVRFLW      (1u << 9)
#define STATUS_FIFO_FULL        (1u << 8)
#define STATUS_FIFO_EMPTY       (1u << 5)
#define STATUS_FIFO_UNDRFLW     (1u << 4)
#define STATUS_CMD_BUSY         (1u << 3)
#define STATUS_DATA_BUSY        (1u << 2)
#define STATUS_BUSY             (1u << 0)

/* bits that are recomputed on every STATUS read */
#define STATUS_DYNAMIC          (STATUS_PRESENT | STATUS_MS_PRESENT | \
                                 STATUS_SD_PRESENT | STATUS_CARD_DETECT | \
                                 STATUS_FIFO_FULL | STATUS_FIFO_EMPTY | \
                                 STATUS_CMD_BUSY | STATUS_DATA_BUSY | \
                                 STATUS_BUSY)

/*
 * Staging buffer used to shuttle data between the SD bus and the guest.
 * The data phase itself is streamed, so this only bounds how much we pull
 * out of the card in one go - transfers of several megabytes work fine.
 */
#define SSP_FIFO_SIZE   4096

typedef struct MXSSSPState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq dma_irq;
    SDBus sdbus;

    uint32_t ctrl0;
    uint32_t cmd0;
    uint32_t cmd1;
    uint32_t xfer_size;
    uint32_t block_size;
    uint32_t timing;
    uint32_t ctrl1;
    uint32_t sdresp[4];
    uint32_t ddr_ctrl;
    uint32_t dll_ctrl;
    uint32_t status;

    /* PIO data staging */
    uint8_t fifo[SSP_FIFO_SIZE];
    uint32_t fifo_len;
    uint32_t fifo_pos;

    /* state of the data phase currently in flight */
    uint32_t data_remaining;    /* bytes still to come from/to the card */
    bool data_read;             /* direction: card -> host */
    bool multiblock;            /* needs an explicit STOP_TRANSMISSION */

    /* ---- BRAIN fault-zone experiment aid (verification-only) ----
     * mode 2 = virtual-time read delay: a DMA descriptor that starts a
     * zone read is held for exp_fault_delay_us of guest time before it
     * executes (see mxs_ssp_hold_until).  Guest timers keep advancing
     * during the wait, exactly as they would on real hardware with a
     * slow/retrying device. */
    uint32_t exp_fault_start;
    uint32_t exp_fault_len;
    uint32_t exp_fault_mode;    /* 0=off, 2=read-delay */
    uint32_t exp_fault_delay_us;
    bool fault_pending;         /* a zone read CCW is being held */
    uint64_t fault_deadline;    /* absolute QEMU_CLOCK_VIRTUAL ns */
    SDRequest fault_req;
} MXSSSPState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSSSPState, MXS_SSP)

/* ---- lightweight debug tracing, enabled with MXS_DEBUG=1 in the env ---- */
static bool mxs_dbg_on(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("MXS_DEBUG");
        on = e && *e != '0';
    }
    return on;
}
#define MXSDBG(fmt, ...) \
    do { if (mxs_dbg_on()) { fprintf(stderr, fmt "\n", ## __VA_ARGS__); } } while (0)

/*
 * Only the top half of CTRL1 holds interrupt status/enable pairs: an odd
 * numbered status bit is enabled by the even bit immediately below it,
 * starting at FIFO_OVERRUN_IRQ (bit 15) / _IRQ_EN (bit 14).  Everything
 * below that is transfer configuration - DMA_ENABLE, the word length and
 * the bus mode - and must not be mistaken for an interrupt source, or the
 * controller ends up asserting its line forever as soon as the guest
 * programs a normal SD/MMC transfer.
 */
#define CTRL1_FIRST_IRQ_BIT 15

static void mxs_ssp_update_irq(MXSSSPState *s)
{
    bool level = false;
    int i;

    for (i = CTRL1_FIRST_IRQ_BIT; i < 32; i += 2) {
        if ((s->ctrl1 & (1u << i)) && (s->ctrl1 & (1u << (i - 1)))) {
            level = true;
        }
    }
    qemu_set_irq(s->irq, level);
}

/*
 * A multiple block transfer leaves the card streaming until it is told to
 * stop.  Synthesise STOP_TRANSMISSION when the guest never sends one.
 */
static void mxs_ssp_stop_transmission(MXSSSPState *s)
{
    SDRequest stop = { .cmd = 12, .arg = 0, .crc = 0 };
    uint8_t resp[16];

    if (!s->multiblock) {
        return;
    }
    s->multiblock = false;
    sdbus_do_command(&s->sdbus, &stop, resp, sizeof(resp));
    MXSDBG("ssp: synthesised STOP_TRANSMISSION");
}

/* command execution body (response + data phase) */
static void mxs_ssp_exec_command(MXSSSPState *s, SDRequest *req)
{
    uint8_t resp[16];
    size_t rlen;

    s->status &= ~STATUS_RESP_TIMEOUT;
    s->sdresp[0] = s->sdresp[1] = s->sdresp[2] = s->sdresp[3] = 0;
    s->fifo_len = s->fifo_pos = 0;
    s->data_remaining = 0;
    s->data_read = false;

    rlen = sdbus_do_command(&s->sdbus, req, resp, sizeof(resp));

    /* the command sequence has ended: raise SSP_END_CMD (bit 19) */
    s->ctrl1 |= CTRL1_END_CMD_IRQ;

    MXSDBG("ssp: CMD%-2u arg=%08x ctrl0=%08x xfer=%u blk=%08x rlen=%zu",
           req->cmd, req->arg, s->ctrl0, s->xfer_size, s->block_size, rlen);

    if (s->ctrl0 & CTRL0_GET_RESP) {
        if (rlen == 4) {
            s->sdresp[0] = ldl_be_p(resp);
        } else if (rlen == 16) {
            s->sdresp[3] = ldl_be_p(resp);
            s->sdresp[2] = ldl_be_p(resp + 4);
            s->sdresp[1] = ldl_be_p(resp + 8);
            s->sdresp[0] = ldl_be_p(resp + 12);
        } else {
            s->status |= STATUS_RESP_TIMEOUT;
            s->ctrl1 |= CTRL1_RESP_TIMEOUT_IRQ;
        }
    }

    /*
     * Arm the data phase.  Nothing is transferred yet: the payload is
     * streamed on demand from the SSP_DATA register or the APBH DMA
     * channel, so multi megabyte CMD18 bursts work without buffering
     * everything up front.
     */
    if (s->ctrl0 & CTRL0_DATA_XFER) {
        uint32_t len = s->xfer_size ? s->xfer_size :
                                      CTRL0_XFER_COUNT(s->ctrl0);

        s->data_remaining = len;
        s->data_read = !!(s->ctrl0 & CTRL0_READ);
        s->multiblock = (req->cmd == 18 || req->cmd == 25 ||
                         SSP_BLOCK_COUNT(s->block_size) > 0);
        MXSDBG("ssp:   data phase %u bytes %s%s", len,
               s->data_read ? "from card" : "to card",
               s->multiblock ? " (multi block)" : "");
    }

    mxs_ssp_update_irq(s);
}

/* fault-zone check for the experiment aid: does this read touch the zone? */
static bool mxs_ssp_fault_hit(MXSSSPState *s, uint32_t cmd, uint32_t arg,
                              uint32_t ctrl0)
{
    uint64_t sec;

    if (s->exp_fault_mode != 2 || s->exp_fault_len == 0) {
        return false;
    }
    if ((cmd != 17 && cmd != 18) || !(ctrl0 & CTRL0_READ)) {
        return false;
    }
    /* the eMMC on SSP0 is SDHC: the command argument is the sector number */
    sec = arg;
    return sec >= s->exp_fault_start &&
           sec < (uint64_t)s->exp_fault_start + s->exp_fault_len;
}

/*
 * BRAIN fault-zone experiment aid (mode 2): DMA-CCW pre-scan hook.
 * The APBH engine calls this BEFORE executing a descriptor that carries
 * PIO words (a would-be SD command).  If the descriptor starts a zone
 * read, the channel is held for exp_fault_delay_us of virtual time and
 * the CCW executes unchanged when the hold expires -- the guest receives
 * the read result delayed, like a slow real device, with no other change
 * to protocol state.  A second call for the same command (the resumed
 * chain) releases the hold.
 */
static int64_t mxs_ssp_hold_until(void *opaque, uint32_t cmd, uint32_t arg,
                                  uint32_t ctrl0)
{
    MXSSSPState *s = MXS_SSP(opaque);
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!mxs_ssp_fault_hit(s, cmd, arg, ctrl0)) {
        return 0;
    }
    if (s->fault_pending && s->fault_req.cmd == cmd &&
        s->fault_req.arg == arg) {
        /* resumption of the held CCW: release and let it run */
        s->fault_pending = false;
        fprintf(stderr,
                "[brain-exp] held read released to guest chain CMD%u "
                "sec=%u vnow=%" PRIu64 " us\n",
                cmd, arg, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
        return 0;
    }
    if (!s->fault_pending) {
        s->fault_pending = true;
        s->fault_req.cmd = cmd;
        s->fault_req.arg = arg;
        s->fault_deadline = now + (uint64_t)s->exp_fault_delay_us * 1000;
        fprintf(stderr,
                "[brain-exp] zone read held at DMA CCW CMD%u sec=%u "
                "delay=%u us vnow=%" PRIu64 " us\n",
                cmd, arg, s->exp_fault_delay_us,
                qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
        return (int64_t)s->fault_deadline;
    }
    /* a different zone command while another hold is pending: no hold */
    return 0;
}

static void mxs_ssp_do_command(MXSSSPState *s)
{
    SDRequest req = {
        .cmd = s->cmd0 & 0xff,
        .arg = s->cmd1,
        .crc = 0,
    };

    /* a new command start clears the end-of-command flag (W1C status) */
    s->ctrl1 &= ~CTRL1_END_CMD_IRQ;

    /*
     * If a multiple block transfer has moved all of its data the card is
     * still streaming.  Most firmware issues STOP_TRANSMISSION itself, so
     * only synthesise one when the guest moves on to something else.
     */
    if (s->multiblock && !s->data_remaining && req.cmd != 12) {
        mxs_ssp_stop_transmission(s);
    }
    if (req.cmd == 12) {
        s->multiblock = false;
    }

    mxs_ssp_exec_command(s, &req);
}

/* make sure the staging buffer holds data, pulling more from the card */
static void mxs_ssp_fifo_fill(MXSSSPState *s)
{
    uint32_t n;

    if (s->fifo_pos < s->fifo_len || !s->data_read || !s->data_remaining) {
        return;
    }

    n = MIN(s->data_remaining, (uint32_t)SSP_FIFO_SIZE);
    sdbus_read_data(&s->sdbus, s->fifo, n);
    s->fifo_len = n;
    s->fifo_pos = 0;
    s->data_remaining -= n;

}

static bool mxs_ssp_dbg_on(void);   /* MXS_SSP_DBG register trace */

/* ---------------- APBH DMA interface ---------------- */

static void mxs_ssp_dma_pio(void *opaque, const uint32_t *words, int nwords)
{
    MXSSSPState *s = MXS_SSP(opaque);

    MXSDBG("ssp: dma pio %d words: %08x %08x %08x %08x %08x", nwords,
           nwords > 0 ? words[0] : 0, nwords > 1 ? words[1] : 0,
           nwords > 2 ? words[2] : 0, nwords > 3 ? words[3] : 0,
           nwords > 4 ? words[4] : 0);
    if (nwords > 0) {
        s->ctrl0 = words[0];
    }
    if (nwords > 1) {
        s->cmd0 = words[1];
    }
    if (nwords > 2) {
        s->cmd1 = words[2];
    }
    if (nwords > 3) {
        s->xfer_size = words[3];
    }
    if (nwords > 4) {
        s->block_size = words[4];
    }

    if (s->ctrl0 & CTRL0_ENABLE) {
        mxs_ssp_do_command(s);
    }
}

static int mxs_ssp_dma_xfer(void *opaque, uint8_t *buf, int len, bool to_device)
{
    MXSSSPState *s = MXS_SSP(opaque);

    MXSDBG("ssp: dma xfer %d bytes %s (fifo %u/%u)", len,
           to_device ? "to card" : "from card", s->fifo_pos, s->fifo_len);

    if (to_device) {
        uint32_t n = MIN((uint32_t)len, s->data_remaining ?
                         s->data_remaining : (uint32_t)len);

        sdbus_write_data(&s->sdbus, buf, n);
        if (s->data_remaining) {
            s->data_remaining -= n;
        }
    } else {
        int done = 0;

        /*
         * Fast path: if the internal FIFO is empty, pull data directly
         * into the target DMA buffer, bypassing intermediate staging.
         */
        if (s->fifo_pos >= s->fifo_len && s->data_read && s->data_remaining) {
            uint32_t direct = MIN((uint32_t)len, s->data_remaining);
            sdbus_read_data(&s->sdbus, buf, direct);
            s->data_remaining -= direct;
            done += direct;
        }

        while (done < len) {
            uint32_t avail;

            mxs_ssp_fifo_fill(s);
            avail = s->fifo_len - s->fifo_pos;
            if (!avail) {
                memset(buf + done, 0, len - done);
                break;
            }
            avail = MIN(avail, (uint32_t)(len - done));
            memcpy(buf + done, s->fifo + s->fifo_pos, avail);
            s->fifo_pos += avail;
            done += avail;
        }
    }
    return len;
}

static const MXSDmaOps mxs_ssp_dma_ops = {
    .pio = mxs_ssp_dma_pio,
    .xfer = mxs_ssp_dma_xfer,
    .hold_until = mxs_ssp_hold_until,
};

const MXSDmaOps *mxs_ssp_get_dma_ops(void)
{
    return &mxs_ssp_dma_ops;
}

/* ---------------- MMIO ---------------- */

static uint64_t mxs_ssp_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSSSPState *s = MXS_SSP(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val = 0;

    switch (idx) {
    case SSP_CTRL0:
        val = s->ctrl0;
        break;
    case SSP_CMD0:
        val = s->cmd0;
        break;
    case SSP_CMD1:
        val = s->cmd1;
        break;
    case SSP_XFER_SIZE:
        val = s->xfer_size;
        break;
    case SSP_BLOCK_SIZE:
        val = s->block_size;
        break;
    case SSP_TIMING:
        val = s->timing;
        break;
    case SSP_CTRL1:
        val = s->ctrl1;
        break;
    case SSP_DATA:
        if (s->fault_pending) {
            /* the deferred read has not been issued yet: no data exists */
            fprintf(stderr, "[brain-exp] SSP_DATA read while delayed "
                    "read pending (vnow=%" PRIu64 " us)\n",
                    qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
            val = 0;
            break;
        }
        mxs_ssp_fifo_fill(s);
        if (s->fifo_pos + 4 <= s->fifo_len) {
            val = ldl_le_p(s->fifo + s->fifo_pos);
            s->fifo_pos += 4;
        } else if (s->fifo_pos < s->fifo_len) {
            uint8_t tmp[4] = { 0 };

            memcpy(tmp, s->fifo + s->fifo_pos, s->fifo_len - s->fifo_pos);
            s->fifo_pos = s->fifo_len;
            val = ldl_le_p(tmp);
        } else {
            val = 0;
        }
        break;
    case SSP_SDRESP0:
    case SSP_SDRESP1:
    case SSP_SDRESP2:
    case SSP_SDRESP3:
        val = s->sdresp[idx - SSP_SDRESP0];
        break;
    case SSP_DDR_CTRL:
        val = s->ddr_ctrl;
        break;
    case SSP_DLL_CTRL:
        val = s->dll_ctrl;
        break;
    case SSP_STATUS:
        /*
         * The FIFO / busy / card presence bits are pure status; they must
         * never be latched into s->status or the guest's poll loops (which
         * spin on FIFO_EMPTY going low) would hang forever.
         */
        val = s->status & ~STATUS_DYNAMIC;
        if (s->fifo_pos >= s->fifo_len &&
            !(s->data_read && s->data_remaining)) {
            val |= STATUS_FIFO_EMPTY;
        } else {
            val |= STATUS_FIFO_FULL;
        }
        /* BRAIN fault-zone experiment aid (mode 2): while a zone read is
         * deferred, the controller reports busy, like real hardware with
         * a slow-responding device. */
        if (s->fault_pending) {
            val |= STATUS_BUSY | STATUS_CMD_BUSY | STATUS_DATA_BUSY;
        }
        if (sdbus_get_inserted(&s->sdbus)) {
            val |= STATUS_SD_PRESENT;
        } else {
            val |= STATUS_CARD_DETECT;
        }
        break;
    case SSP_DLL_STS:
        val = 3;    /* REF_LOCK | SLV_LOCK */
        break;
    case SSP_VERSION:
        val = 0x04000000;
        break;
    default:
        break;
    }

    return mxs_bank_extract(offset, size, val);
}

static void mxs_ssp_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    MXSSSPState *s = MXS_SSP(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);

    switch (idx) {
    case SSP_CTRL0: {
        uint32_t old = s->ctrl0;

        s->ctrl0 = mxs_bank_sftrst(old,
                                   mxs_bank_apply(old, offset, value, size));
        if ((s->ctrl0 & CTRL0_RUN) && !(old & CTRL0_RUN)) {
            mxs_ssp_do_command(s);
            /* a fault-delayed command keeps RUN set until it completes,
             * like real hardware with an in-flight operation */
            if (!s->fault_pending) {
                s->ctrl0 &= ~CTRL0_RUN;
            }
        }
        break;
    }
    case SSP_CMD0:
        s->cmd0 = mxs_bank_apply(s->cmd0, offset, value, size);
        break;
    case SSP_CMD1:
        s->cmd1 = mxs_bank_apply(s->cmd1, offset, value, size);
        break;
    case SSP_XFER_SIZE:
        s->xfer_size = mxs_bank_apply(s->xfer_size, offset, value, size);
        break;
    case SSP_BLOCK_SIZE:
        s->block_size = mxs_bank_apply(s->block_size, offset, value, size);
        break;
    case SSP_TIMING:
        s->timing = mxs_bank_apply(s->timing, offset, value, size);
        break;
    case SSP_CTRL1: {
        uint32_t oldv = s->ctrl1;
        uint32_t newv = mxs_bank_apply(oldv, offset, value, size);

        if (mxs_ssp_dbg_on()) {
            fprintf(stderr, "[mxs-DBG] ctrl1 dev=%p W +0x%03x %u: %08x -> %08x\n",
                    s, (unsigned)offset, (unsigned)MXS_BANK_OP(offset),
                    oldv, newv);
        }
        s->ctrl1 = newv;
        mxs_ssp_update_irq(s);
        break;
    }
    case SSP_DATA: {
        uint8_t tmp[4];
        uint32_t n = 4;

        if (s->fault_pending) {
            fprintf(stderr, "[brain-exp] SSP_DATA write while delayed "
                    "read pending (vnow=%" PRIu64 " us)\n",
                    qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
            break;
        }
        stl_le_p(tmp, (uint32_t)value);
        if (s->data_remaining && s->data_remaining < n) {
            n = s->data_remaining;
        }
        sdbus_write_data(&s->sdbus, tmp, n);
        if (s->data_remaining) {
            s->data_remaining -= n;
        }
        break;
    }
    case SSP_DDR_CTRL:
        s->ddr_ctrl = mxs_bank_apply(s->ddr_ctrl, offset, value, size);
        break;
    case SSP_DLL_CTRL:
        s->dll_ctrl = mxs_bank_apply(s->dll_ctrl, offset, value, size);
        break;
    default:
        break;
    }
}

MXS_TRACE_WRAP(mxs_ssp, "ssp")

/*
 * Register-level trace aid: with MXS_SSP_DBG=1 every SSP access is
 * logged with the instance pointer (to tell the eMMC slot from the
 * empty microSD slot) -- used to trace the WinCE SDHC driver command
 * sequences (S74).
 */
static bool mxs_ssp_dbg_on(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("MXS_SSP_DBG");

        on = e && *e != '0';
    }
    return on;
}
static uint64_t mxs_ssp_read_dbg(void *o, hwaddr a, unsigned sz)
{
    uint64_t v = mxs_ssp_read_tr(o, a, sz);

    if (mxs_ssp_dbg_on()) {
        fprintf(stderr, "[mxs-DBG] ssp dev=%p R +0x%03x sz=%u -> %llx\n",
                o, (unsigned)a, sz, (unsigned long long)v);
    }
    return v;
}
static void mxs_ssp_write_dbg(void *o, hwaddr a, uint64_t v, unsigned sz)
{
    if (mxs_ssp_dbg_on()) {
        fprintf(stderr, "[mxs-DBG] ssp dev=%p W +0x%03x sz=%u <- %llx\n",
                o, (unsigned)a, sz, (unsigned long long)v);
    }
    mxs_ssp_write_tr(o, a, v, sz);
}

static const MemoryRegionOps mxs_ssp_ops = {
    .read = mxs_ssp_read_dbg,
    .write = mxs_ssp_write_dbg,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_ssp_reset(DeviceState *dev)
{
    MXSSSPState *s = MXS_SSP(dev);

    s->ctrl0 = 0;
    s->cmd0 = 0;
    s->cmd1 = 0;
    s->xfer_size = 0;
    s->block_size = 0;
    s->timing = 0x00010000;
    s->ctrl1 = 0;
    s->status = 0;
    s->fifo_len = s->fifo_pos = 0;
    s->data_remaining = 0;
    s->data_read = false;
    s->multiblock = false;
    memset(s->sdresp, 0, sizeof(s->sdresp));
    s->fault_pending = false;
    s->fault_deadline = 0;
    qemu_set_irq(s->irq, 0);
}

static void mxs_ssp_init(Object *obj)
{
    mxs_ssp_trace = mxs_trace_enabled("ssp");

    MXSSSPState *s = MXS_SSP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &mxs_ssp_ops, s, "mxs-ssp", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_SD_BUS, DEVICE(obj), "sd-bus");
}

static const VMStateDescription vmstate_mxs_ssp = {
    .name = "mxs-ssp",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl0, MXSSSPState),
        VMSTATE_UINT32(cmd0, MXSSSPState),
        VMSTATE_UINT32(cmd1, MXSSSPState),
        VMSTATE_UINT32(xfer_size, MXSSSPState),
        VMSTATE_UINT32(block_size, MXSSSPState),
        VMSTATE_UINT32(ctrl1, MXSSSPState),
        VMSTATE_UINT32_ARRAY(sdresp, MXSSSPState, 4),
        VMSTATE_UINT32(status, MXSSSPState),
        VMSTATE_END_OF_LIST()
    }
};

/* BRAIN fault-zone experiment aid (verification-only, default off) */
static const Property mxs_ssp_properties[] = {
    DEFINE_PROP_UINT32("exp-fault-start", MXSSSPState, exp_fault_start, 0),
    DEFINE_PROP_UINT32("exp-fault-len", MXSSSPState, exp_fault_len, 0),
    DEFINE_PROP_UINT32("exp-fault-mode", MXSSSPState, exp_fault_mode, 0),
    DEFINE_PROP_UINT32("exp-fault-delay-us", MXSSSPState,
                       exp_fault_delay_us, 500000),
};

static void mxs_ssp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, mxs_ssp_reset);
    device_class_set_props(dc, mxs_ssp_properties);
    dc->vmsd = &vmstate_mxs_ssp;
}

static const TypeInfo mxs_ssp_types[] = {
    {
        .name           = TYPE_MXS_SSP,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSSSPState),
        .instance_init  = mxs_ssp_init,
        .class_init     = mxs_ssp_class_init,
    },
};

DEFINE_TYPES(mxs_ssp_types)
