/*
 * Freescale i.MX28 GPMI (General Purpose Media Interface) NAND controller.
 *
 * Register map from the i.MX28 reference manual (GPMI chapter) and the
 * Linux BSP driver (drivers/mtd/nand/gpmi-nand/gpmi-regs.h, which
 * supports the i.MX28):
 *
 *   0x000 GPMI_CTL            bit0 RUN, bit1 READ, bit2 WRITE, bit3 ERASE,
 *                             bit5 ECC_STATUS_CLR, bit8 BUS_WIDTH_4,
 *                             bit9 ECC_MODE_BCH, bits[11:10] ECC_STEP,
 *                             bits[15:14] ECC_POS
 *   0x004 GPMI_CONFIG         bit0 BCH_ECC, bits[2:1] BCH_BYTES,
 *                             bits[29:28] PAGE_SIZE (0:512 1:1K 2:2K 3:4K)
 *   0x008 GPMI_INT            W1C: bit0 ECC_ERR, bit1 TIMEOUT, bit2 CMD
 *   0x00C GPMI_INT_EN         bit0-2 interrupt enables
 *   0x010 GPMI_PAGE           page number (latched with RUN)
 *   0x014 GPMI_DATA           PIO data (command/status byte path)
 *   0x018 GPMI_STATUS         bit0 BUSY, data/status readback
 *   0x01C GPMI_CMD            NAND command byte (latched with RUN)
 *   0x020..0x030 GPMI_ADDR0..4 address bytes (latched with RUN)
 *   0x034 GPMI_DATA_RD        physical DMA address, page/OOB data buffer
 *   0x038 GPMI_STATUS_RD      physical DMA address, R/B status byte
 *   0x03C GPMI_TIMEOUT_CTRL   timeout value (GPMI clock cycles)
 *   0x040 GPMI_DMA_BUF        physical DMA address, cmd/addr/id buffer
 *   0x044 GPMI_TIMEOUT_STATUS W1C: bit0 CMD, bit1 DATA
 *
 * The controller has an internal DMA engine (no AHB DMA request): page
 * data is moved to/from the guest physical address programmed in
 * GPMI_DATA_RD, the R/B status byte to GPMI_STATUS_RD, and id bytes to
 * GPMI_DMA_BUF.
 *
 * The latched command values are the ones the i.MX28 controller accepts
 * (see gpmi_write_cmd() in the Linux gpmi-nand driver):
 *
 *   0x00 reset            0x90 read id           0x70 read status
 *   0x05 page read        0x30 oob read          0x20 sequential read
 *   0x08 program data     0x85 oob write         0x10 program execute
 *   0x60 erase address    0xD0 erase execute
 *
 * Two media modes:
 *
 *  - empty socket (default, the actual SHARP Brain PW-AJ2 configuration:
 *    the GPMI pins are unpopulated, the factory MP test list contains no
 *    NAND item, and the BSP reports "flash initialization failed" on
 *    every boot).  A RUN command is latched, the bus never answers, the
 *    GPMI_TIMEOUT_CTRL period elapses and GPMI_INT.TIMEOUT /
 *    GPMI_TIMEOUT_STATUS are raised exactly like the hardware does.
 *
 *  - populated socket (machine property "brain-gpmi-nand").  A full NAND
 *    device is modelled: the standard command set above with per-command
 *    busy timing (tRST/tR/tPROG/tBERS), R/B status, two-phase program
 *    (0x08 data load, then the hardware auto-issues 0x10) and erase
 *    (0x60 address, 0xD0 execute), and -- when ECC_MODE_BCH is set --
 *    real BCH ECC generation on program and decode on read with the
 *    i.MX28 standard 4 x 512-byte step layout (4-bit parity in OOB
 *    bytes 0..3, 16..19, 32..35, 48..51).  The flash content is kept in
 *    RAM or backed by a raw image file (e.g. the v4 eMMC 0x27800 Nand2
 *    mirror).
 *
 *    The Brain does not ship a NAND die, so chip id / geometry have no
 *    golden-reference values; they are machine properties with a
 *    64 MiB / 2KiB-page / 64-byte-OOB / 512-block default consistent
 *    with the \Nand2 volume size (0x1FFF1 sectors).
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/misc/mxs_bchlib.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/dma.h"

#define GPMI_CTL            0x00
#define GPMI_CONFIG         0x04
#define GPMI_INT            0x08
#define GPMI_INT_EN         0x0c
#define GPMI_PAGE           0x10
#define GPMI_DATA           0x14
#define GPMI_STATUS         0x18
#define GPMI_CMD            0x1c
#define GPMI_ADDR0          0x20
#define GPMI_ADDR1          0x24
#define GPMI_ADDR2          0x28
#define GPMI_ADDR3          0x2c
#define GPMI_ADDR4          0x30
#define GPMI_DATA_RD        0x34
#define GPMI_STATUS_RD      0x38
#define GPMI_TIMEOUT_CTRL   0x3c
#define GPMI_DMA_BUF        0x40
#define GPMI_TIMEOUT_STATUS 0x44

#define MXS_GPMI_NREGS      (GPMI_TIMEOUT_STATUS / 4 + 1)

/* GPMI_CTL */
#define GPMI_CTL_RUN          (1u << 0)
#define GPMI_CTL_READ         (1u << 1)
#define GPMI_CTL_WRITE        (1u << 2)
#define GPMI_CTL_ERASE        (1u << 3)
#define GPMI_CTL_ECC_STATUS_CLR (1u << 5)
#define GPMI_CTL_BUS_WIDTH_4  (1u << 8)
#define GPMI_CTL_ECC_MODE_BCH (1u << 9)
#define GPMI_CTL_ECC_STEP     0x3u
#define GPMI_CTL_ECC_POS      0x3u

/* GPMI_CONFIG */
#define GPMI_CONFIG_BCH_ECC   (1u << 0)
#define GPMI_CONFIG_BCH_BYTES 0x3u
#define GPMI_CONFIG_PAGE_SIZE 0x3u
#define GPMI_CONFIG_PAGE_SIZE_SH 28

/* GPMI_INT / GPMI_INT_EN */
#define GPMI_INT_ECC_ERR      (1u << 0)
#define GPMI_INT_TIMEOUT      (1u << 1)
#define GPMI_INT_CMD          (1u << 2)

/* GPMI_TIMEOUT_STATUS */
#define GPMI_TIMEOUT_STAT_CMD   (1u << 0)
#define GPMI_TIMEOUT_STAT_DATA  (1u << 1)

/* latched controller command values (i.MX28) */
#define GPMI_CMD_RESET        0x00
#define GPMI_CMD_PAGE_READ    0x05
#define GPMI_CMD_SEQ_READ     0x20
#define GPMI_CMD_OOB_READ     0x30
#define GPMI_CMD_PROG_DATA    0x08
#define GPMI_CMD_PROG_EXEC    0x10
#define GPMI_CMD_ERASE_ADDR   0x60
#define GPMI_CMD_ERASE_EXEC   0xD0
#define GPMI_CMD_READ_ID      0x90
#define GPMI_CMD_READ_STAT    0x70
#define GPMI_CMD_OOB_WRITE    0x85

/* default timeout when GPMI_TIMEOUT_CTRL is 0 (virtual 1 s) */
#define GPMI_TIMEOUT_DEFAULT_NS 1000000000ULL
/* GPMI clock ~48 MHz: one cycle ~= 21 ns */
#define GPMI_TIMEOUT_CYCLE_NS   21ULL

/*
 * NAND busy timing (virtual ns), order of magnitude from typical
 * 2KiB-page NAND data sheets: tRST ~200 us, tR ~50 us, tPROG ~1.2 ms,
 * tBERS ~50 ms.
 */
#define NAND_T_RST_NS       200000ULL
#define NAND_T_ID_NS        50000ULL
#define NAND_T_STAT_NS      50000ULL
#define NAND_T_READ_NS      50000ULL
#define NAND_T_PROG_NS      1200000ULL
#define NAND_T_ERASE_NS     50000000ULL

#define NAND_RB_READY       0x40
#define NAND_RB_BUSY        0x80

/* default chip geometry: 64 MiB, 2 KiB pages, 64 B OOB, 512 blocks */
#define GPMI_NAND_DEFAULT_BLOCKS   512
#define GPMI_NAND_DEFAULT_PAGES    64
#define GPMI_NAND_DEFAULT_PAGE     2048
#define GPMI_NAND_DEFAULT_OOB      64
/* EC F1 B3 95 (Samsung 64Mbit class, 2KiB page) */
#define GPMI_NAND_DEFAULT_ID       0x95b3f1ec

typedef struct MXSGpmiNand {
    uint8_t *flash;     /* blocks * pages * (page + oob) */
    bool *bad;          /* per block */
    unsigned blocks, pages, page, oob;
    uint32_t id;        /* little-endian id0..id3 */
    bool busy;
    /* block latched by the most recent 0x60 (erase address) */
    unsigned erase_block;
} MXSGpmiNand;

typedef struct MXSGpmiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *busy_timer;   /* chip busy period */
    QEMUTimer *timeout_timer;/* empty-socket timeout */
    uint32_t regs[MXS_GPMI_NREGS];
    char *name;
    uint64_t size;
    bool trace;

    /* latched at RUN */
    uint32_t latched_ctl;
    uint32_t latched_cmd;
    uint32_t latched_addr[5];
    uint32_t latched_page;
    uint32_t dma_buf;
    uint32_t data_rd;
    uint32_t status_rd;
    bool ecc_bch;
    bool prog_phase2;   /* inside the auto 0x10 program-execute phase */

    MXSGpmiNand nand;
    bool have_nand;
    char *nand_file;
    unsigned nand_blocks, nand_pages, nand_page, nand_oob;
    uint32_t nand_id;

    /* 4-bit BCH for the i.MX28 512-byte step layout */
    struct bch_control *bch4;
} MXSGpmiState;

#define TYPE_MXS_GPMI "mxs-gpmi"
OBJECT_DECLARE_SIMPLE_TYPE(MXSGpmiState, MXS_GPMI)

static void gpmi_update_irq(MXSGpmiState *s)
{
    uint32_t intreg = s->regs[GPMI_INT >> 2];
    uint32_t int_en = s->regs[GPMI_INT_EN >> 2];

    qemu_set_irq(s->irq, (intreg & int_en) != 0);
}

static void gpmi_set_int(MXSGpmiState *s, uint32_t bits)
{
    s->regs[GPMI_INT >> 2] |= bits;
    gpmi_update_irq(s);
}

static void gpmi_clear_int(MXSGpmiState *s, uint32_t bits)
{
    s->regs[GPMI_INT >> 2] &= ~bits;
    gpmi_update_irq(s);
}

/* decode a 2KiB-page, 5-address-cycle NAND address (512 blocks) */
static void gpmi_decode_addr(MXSGpmiState *s, unsigned *block,
                             unsigned *page, unsigned *col)
{
    unsigned lcol = (s->latched_addr[0] & 0xff) |
                    ((s->latched_addr[1] & 0xff) << 8);
    unsigned lpage = s->latched_addr[2] & 0xff;
    unsigned lblk = (s->latched_addr[3] & 0xff) |
                    ((s->latched_addr[4] & 0x0f) << 8);

    *col = lcol & (s->nand.page - 1);
    *page = lpage % s->nand.pages;
    *block = lblk % s->nand.blocks;
}

static uint8_t *gpmi_page_ptr(MXSGpmiState *s, unsigned block, unsigned page)
{
    unsigned off = ((block * s->nand.pages) + page) *
                   (s->nand.page + s->nand.oob);

    return s->nand.flash + off;
}

static unsigned gpmi_page_size(MXSGpmiState *s)
{
    return s->nand.page + s->nand.oob;
}

/*
 * i.MX28 default ECC layout for 2KiB pages: four 512-byte steps,
 * 4-bit BCH each, parity in OOB bytes [0..3] [16..19] [32..35] [48..51].
 */
static void gpmi_ecc_encode(MXSGpmiState *s, uint8_t *pg)
{
    uint8_t ecc[4];

    if (!s->bch4 || s->nand.page != 2048 || s->nand.oob < 52) {
        return;
    }
    for (int i = 0; i < 4; i++) {
        memset(ecc, 0, sizeof(ecc));
        encode_bch(s->bch4, pg + i * 512, 512, ecc);
        memcpy(pg + 2048 + i * 16, ecc, 4);
    }
}

/* returns number of uncorrectable steps */
static unsigned gpmi_ecc_decode(MXSGpmiState *s, uint8_t *pg)
{
    unsigned bad_steps = 0;

    if (!s->bch4 || s->nand.page != 2048 || s->nand.oob < 52) {
        return 0;
    }
    for (int i = 0; i < 4; i++) {
        uint8_t ecc[4];
        uint32_t errloc[16];

        memcpy(ecc, pg + 2048 + i * 16, 4);
        if (decode_bch(s->bch4, pg + i * 512, 512, ecc, NULL, NULL,
                       errloc) < 0) {
            bad_steps++;
        }
    }
    return bad_steps;
}

static void gpmi_write_rb(MXSGpmiState *s, uint8_t rb)
{
    dma_memory_write(&address_space_memory, s->status_rd, &rb, 1,
                     MEMTXATTRS_UNSPECIFIED);
}

/* chip busy period finished: perform the data movement, complete the op */
static void gpmi_nand_complete(void *opaque)
{
    MXSGpmiState *s = opaque;
    uint32_t cmd = s->latched_cmd;
    bool data_moved = false;

    if (!s->have_nand) {
        return;
    }

    if (s->prog_phase2) {
        /* the auto-issued 0x10 program-execute finished */
        s->prog_phase2 = false;
        goto done;
    }

    switch (cmd) {
    case GPMI_CMD_RESET:
        break;
    case GPMI_CMD_READ_ID:
    {
        uint8_t idbuf[16];

        memset(idbuf, 0xff, sizeof(idbuf));
        for (int i = 0; i < 4; i++) {
            idbuf[i] = (s->nand.id >> (8 * i)) & 0xff;
        }
        dma_memory_write(&address_space_memory, s->dma_buf, idbuf, 16,
                         MEMTXATTRS_UNSPECIFIED);
        data_moved = true;
        break;
    }
    case GPMI_CMD_READ_STAT:
        gpmi_write_rb(s, NAND_RB_READY);
        data_moved = true;
        break;
    case GPMI_CMD_PAGE_READ:
    case GPMI_CMD_SEQ_READ:
    case GPMI_CMD_OOB_READ:
    {
        unsigned block, page, col;
        uint8_t *pg;
        unsigned len;

        gpmi_decode_addr(s, &block, &page, &col);
        pg = gpmi_page_ptr(s, block, page);
        if (cmd == GPMI_CMD_OOB_READ) {
            dma_memory_write(&address_space_memory, s->data_rd,
                             pg + s->nand.page, s->nand.oob,
                             MEMTXATTRS_UNSPECIFIED);
            len = s->nand.oob;
        } else {
            len = s->nand.page + s->nand.oob - col;
            dma_memory_write(&address_space_memory, s->data_rd,
                             pg + col, len, MEMTXATTRS_UNSPECIFIED);
        }
        if (s->ecc_bch) {
            if (gpmi_ecc_decode(s, pg)) {
                gpmi_set_int(s, GPMI_INT_ECC_ERR);
            }
        }
        gpmi_write_rb(s, NAND_RB_READY);
        data_moved = true;
        break;
    }
    case GPMI_CMD_PROG_DATA:
    case GPMI_CMD_OOB_WRITE:
    {
        unsigned block, page, col;
        uint8_t *pg;
        uint8_t tmp[4096];
        unsigned len;

        gpmi_decode_addr(s, &block, &page, &col);
        pg = gpmi_page_ptr(s, block, page);
        if (cmd == GPMI_CMD_OOB_WRITE) {
            len = s->nand.oob;
            dma_memory_read(&address_space_memory, s->data_rd,
                            pg + s->nand.page, len, MEMTXATTRS_UNSPECIFIED);
        } else {
            len = s->nand.page + s->nand.oob - col;
            dma_memory_read(&address_space_memory, s->data_rd, tmp, len,
                            MEMTXATTRS_UNSPECIFIED);
            memcpy(pg + col, tmp, len);
        }
        if (s->ecc_bch) {
            gpmi_ecc_encode(s, pg);
        }
        gpmi_write_rb(s, NAND_RB_READY);
        data_moved = true;
        /*
         * The controller automatically issues the 0x10 program-execute
         * after the data load: arm the second busy phase.
         */
        if (cmd == GPMI_CMD_PROG_DATA) {
            s->prog_phase2 = true;
            s->nand.busy = true;
            s->regs[GPMI_STATUS >> 2] = NAND_RB_BUSY;
            timer_mod(s->busy_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      NAND_T_PROG_NS);
            return;   /* RUN stays set until phase 2 completes */
        }
        break;
    }
    case GPMI_CMD_PROG_EXEC:
        break;
    case GPMI_CMD_ERASE_ADDR:
    {
        unsigned block, page, col;

        gpmi_decode_addr(s, &block, &page, &col);
        s->nand.erase_block = block;
        break;
    }
    case GPMI_CMD_ERASE_EXEC:
    {
        unsigned block = s->nand.erase_block;
        uint8_t *base = gpmi_page_ptr(s, block, 0);

        memset(base, 0xff, s->nand.pages * gpmi_page_size(s));
        break;
    }
    default:
        break;
    }

    if (data_moved) {
        s->regs[GPMI_TIMEOUT_STATUS >> 2] |= GPMI_TIMEOUT_STAT_DATA;
    }

done:
    s->nand.busy = false;
    s->regs[GPMI_STATUS >> 2] = 0;
    s->regs[GPMI_CTL >> 2] &= ~GPMI_CTL_RUN;
    gpmi_set_int(s, GPMI_INT_CMD);
}

static void gpmi_start_command(MXSGpmiState *s)
{
    uint32_t ctl = s->latched_ctl;

    if (!s->have_nand) {
        /*
         * Empty socket: the command is latched and the bus never
         * answers.  The GPMI_TIMEOUT_CTRL period elapses, then the
         * timeout status/interrupt are raised (hardware behaviour).
         */
        uint64_t cycles = s->regs[GPMI_TIMEOUT_CTRL >> 2];
        uint64_t ns = cycles ? cycles * GPMI_TIMEOUT_CYCLE_NS
                             : GPMI_TIMEOUT_DEFAULT_NS;

        timer_mod(s->timeout_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
        return;
    }

    if (!(ctl & (GPMI_CTL_READ | GPMI_CTL_WRITE | GPMI_CTL_ERASE))) {
        /* no command type selected: nothing runs, RUN clears */
        s->regs[GPMI_CTL >> 2] &= ~GPMI_CTL_RUN;
        gpmi_set_int(s, GPMI_INT_CMD);
        return;
    }

    {
        uint64_t ns;

        switch (s->latched_cmd) {
        case GPMI_CMD_RESET:
            ns = NAND_T_RST_NS;
            break;
        case GPMI_CMD_READ_ID:
            ns = NAND_T_ID_NS;
            break;
        case GPMI_CMD_READ_STAT:
            ns = NAND_T_STAT_NS;
            break;
        case GPMI_CMD_PAGE_READ:
        case GPMI_CMD_SEQ_READ:
        case GPMI_CMD_OOB_READ:
            ns = NAND_T_READ_NS;
            break;
        case GPMI_CMD_PROG_DATA:
        case GPMI_CMD_OOB_WRITE:
        case GPMI_CMD_PROG_EXEC:
            ns = NAND_T_PROG_NS;
            break;
        case GPMI_CMD_ERASE_ADDR:
            ns = NAND_T_READ_NS;
            break;
        case GPMI_CMD_ERASE_EXEC:
            ns = NAND_T_ERASE_NS;
            break;
        default:
            ns = NAND_T_READ_NS;
            break;
        }
        s->nand.busy = true;
        s->regs[GPMI_STATUS >> 2] = NAND_RB_BUSY;
        timer_mod(s->busy_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
    }
}

static void gpmi_busy_timer_expire(void *opaque)
{
    gpmi_nand_complete(opaque);
}

static void gpmi_timeout_expire(void *opaque)
{
    MXSGpmiState *s = opaque;

    s->regs[GPMI_CTL >> 2] &= ~GPMI_CTL_RUN;
    s->regs[GPMI_TIMEOUT_STATUS >> 2] |= GPMI_TIMEOUT_STAT_CMD;
    gpmi_set_int(s, GPMI_INT_TIMEOUT);
}

static void gpmi_load_nand(MXSGpmiState *s)
{
    unsigned sz = s->nand_blocks * s->nand_pages *
                  (s->nand_page + s->nand_oob);

    s->nand.flash = g_malloc0(sz);
    memset(s->nand.flash, 0xff, sz);
    s->nand.bad = g_malloc0(s->nand_blocks);
    s->nand.blocks = s->nand_blocks;
    s->nand.pages = s->nand_pages;
    s->nand.page = s->nand_page;
    s->nand.oob = s->nand_oob;
    s->nand.id = s->nand_id;

    if (s->nand_file && *s->nand_file) {
        FILE *f = fopen(s->nand_file, "rb");
        unsigned got;

        if (f) {
            got = fread(s->nand.flash, 1, sz, f);
            fclose(f);
            fprintf(stderr, "[mxs-gpmi] loaded %u/%u bytes from %s\n",
                    got, sz, s->nand_file);
        } else {
            fprintf(stderr,
                    "[mxs-gpmi] cannot open %s, using blank flash\n",
                    s->nand_file);
        }
    }
    if (!s->bch4) {
        s->bch4 = init_bch(13, 4, 0);
    }
    s->have_nand = true;
}

static uint64_t mxs_gpmi_read(void *opaque, hwaddr off, unsigned size)
{
    MXSGpmiState *s = MXS_GPMI(opaque);
    uint32_t v = 0;

    if (off < sizeof(s->regs) * 4) {
        v = s->regs[off >> 2];
    }
    /* GPMI_STATUS bit0 mirrors the NAND busy state */
    if (off == GPMI_STATUS && s->have_nand && s->nand.busy) {
        v |= 0x1;
    }
    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, false, off, v);
    }
    return mxs_bank_extract(off, size, v);
}

static void mxs_gpmi_write(void *opaque, hwaddr off, uint64_t value,
                           unsigned size)
{
    MXSGpmiState *s = MXS_GPMI(opaque);

    if (off >= sizeof(s->regs) * 4) {
        return;
    }
    if (unlikely(s->trace || mxs_trace_live)) {
        mxs_trace_access(s->name, true, off, (uint32_t)value);
    }

    switch (off) {
    case GPMI_INT:
        /* write-1-to-clear */
        gpmi_clear_int(s, (uint32_t)value &
                       (GPMI_INT_ECC_ERR | GPMI_INT_TIMEOUT | GPMI_INT_CMD));
        return;
    case GPMI_TIMEOUT_STATUS:
        s->regs[GPMI_TIMEOUT_STATUS >> 2] &=
            ~((uint32_t)value & (GPMI_TIMEOUT_STAT_CMD | GPMI_TIMEOUT_STAT_DATA));
        return;
    case GPMI_CTL:
    {
        uint32_t old = s->regs[GPMI_CTL >> 2];
        uint32_t val = (uint32_t)value &
            (GPMI_CTL_RUN | GPMI_CTL_READ | GPMI_CTL_WRITE | GPMI_CTL_ERASE |
             GPMI_CTL_ECC_STATUS_CLR | GPMI_CTL_BUS_WIDTH_4 |
             GPMI_CTL_ECC_MODE_BCH | GPMI_CTL_ECC_STEP | GPMI_CTL_ECC_POS);

        s->regs[GPMI_CTL >> 2] = val;
        if ((val & GPMI_CTL_RUN) && !(old & GPMI_CTL_RUN)) {
            /* latch the command context and start the transfer */
            s->latched_ctl = val;
            s->latched_cmd = s->regs[GPMI_CMD >> 2];
            for (int i = 0; i < 5; i++) {
                s->latched_addr[i] = s->regs[(GPMI_ADDR0 + 4 * i) >> 2];
            }
            s->latched_page = s->regs[GPMI_PAGE >> 2];
            s->dma_buf = s->regs[GPMI_DMA_BUF >> 2];
            s->data_rd = s->regs[GPMI_DATA_RD >> 2];
            s->status_rd = s->regs[GPMI_STATUS_RD >> 2];
            s->ecc_bch = (val & GPMI_CTL_ECC_MODE_BCH) != 0 &&
                         (s->regs[GPMI_CONFIG >> 2] & GPMI_CONFIG_BCH_ECC);
            s->prog_phase2 = false;
            timer_del(s->timeout_timer);
            timer_del(s->busy_timer);
            gpmi_start_command(s);
        } else if (!(val & GPMI_CTL_RUN) && (old & GPMI_CTL_RUN)) {
            /* abort: stop any pending timer, clear busy */
            timer_del(s->timeout_timer);
            timer_del(s->busy_timer);
            s->nand.busy = false;
            s->prog_phase2 = false;
            s->regs[GPMI_STATUS >> 2] = 0;
        }
        return;
    }
    default:
        s->regs[off >> 2] = (uint32_t)value;
        return;
    }
}

static const MemoryRegionOps mxs_gpmi_ops = {
    .read = mxs_gpmi_read,
    .write = mxs_gpmi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_gpmi_reset(DeviceState *d)
{
    MXSGpmiState *s = MXS_GPMI(d);

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->timeout_timer);
    timer_del(s->busy_timer);
    s->nand.busy = false;
    s->prog_phase2 = false;
    qemu_set_irq(s->irq, 0);
}

static void mxs_gpmi_realize(DeviceState *d, Error **errp)
{
    MXSGpmiState *s = MXS_GPMI(d);
    SysBusDevice *sbd = SYS_BUS_DEVICE(d);
    g_autofree char *n = g_strdup_printf("mxs-%s", s->name ? s->name : "gpmi");

    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_gpmi_ops, s, n,
                          s->size ? s->size : 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    gpmi_timeout_expire, s);
    s->busy_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 gpmi_busy_timer_expire, s);
    s->trace = mxs_trace_enabled(s->name ? s->name : "gpmi") ||
               (getenv("BRAIN_GPMI_TRACE") &&
                getenv("BRAIN_GPMI_TRACE")[0] != '0');
    if (s->have_nand) {
        gpmi_load_nand(s);
    }
}

static const Property mxs_gpmi_props[] = {
    DEFINE_PROP_STRING("name", MXSGpmiState, name),
    DEFINE_PROP_UINT64("size", MXSGpmiState, size, 0x2000),
    /* NAND media (default off: the Brain socket is unpopulated) */
    DEFINE_PROP_BOOL("nand", MXSGpmiState, have_nand, false),
    DEFINE_PROP_STRING("nand-file", MXSGpmiState, nand_file),
    DEFINE_PROP_UINT32("nand-blocks", MXSGpmiState, nand_blocks,
                       GPMI_NAND_DEFAULT_BLOCKS),
    DEFINE_PROP_UINT32("nand-pages", MXSGpmiState, nand_pages,
                       GPMI_NAND_DEFAULT_PAGES),
    DEFINE_PROP_UINT32("nand-page", MXSGpmiState, nand_page,
                       GPMI_NAND_DEFAULT_PAGE),
    DEFINE_PROP_UINT32("nand-oob", MXSGpmiState, nand_oob,
                       GPMI_NAND_DEFAULT_OOB),
    DEFINE_PROP_UINT32("nand-id", MXSGpmiState, nand_id,
                       GPMI_NAND_DEFAULT_ID),
};

static const VMStateDescription vmstate_mxs_gpmi = {
    .name = "mxs-gpmi",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSGpmiState, MXS_GPMI_NREGS),
        VMSTATE_END_OF_LIST()
    },
};

static void mxs_gpmi_class_init(ObjectClass *k, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(k);

    dc->realize = mxs_gpmi_realize;
    device_class_set_legacy_reset(dc, mxs_gpmi_reset);
    dc->vmsd = &vmstate_mxs_gpmi;
    device_class_set_props(dc, mxs_gpmi_props);
}

static const TypeInfo mxs_gpmi_info[] = {
{
    .name = TYPE_MXS_GPMI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MXSGpmiState),
    .class_init = mxs_gpmi_class_init,
}
};
DEFINE_TYPES(mxs_gpmi_info)
