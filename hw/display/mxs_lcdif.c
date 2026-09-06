/*
 * Freescale i.MX28 (MXS) LCDIF display controller plus the MIPI-DBI panel
 * it drives on the SHARP Brain.
 *
 * The Brain's display module is not a timed-scan RGB panel.  The LCDIF runs
 * in MPU (CPU) mode and the driver talks to a DBI controller: it programs a
 * drawing window with CASET/RASET, issues RAMWR, and then streams
 * TRANSFER_COUNT words of pixels, which the panel latches into its GRAM.
 * QEMU therefore models the panel's GRAM: every push -- the whole screen or a
 * few lines -- lands in that frame memory, and the console shows the frame
 * memory.  That is what makes partial updates, the boot loader's picture and
 * the WinCE desktop coexist the way they do on the device.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "ui/console.h"
#include "ui/input.h"
#include "ui/pixel_ops.h"
#include "system/address-spaces.h"
#include "trace.h"

/* register indices (offset >> 4) */
#define LCDIF_CTRL              0x00
#define LCDIF_CTRL1             0x01
#define LCDIF_CTRL2             0x02
#define LCDIF_TRANSFER_COUNT    0x03
#define LCDIF_CUR_BUF           0x04
#define LCDIF_NEXT_BUF          0x05
#define LCDIF_TIMING            0x06
#define LCDIF_VDCTRL0           0x07
#define LCDIF_VDCTRL1           0x08
#define LCDIF_VDCTRL2           0x09
#define LCDIF_VDCTRL3           0x0a
#define LCDIF_VDCTRL4           0x0b
#define LCDIF_DVICTRL0          0x0c
#define LCDIF_DATA              0x18
#define LCDIF_BM_ERROR_STAT     0x19
#define LCDIF_CRC_STAT          0x1a
#define LCDIF_STAT              0x1b
#define LCDIF_VERSION           0x1c
#define LCDIF_DEBUG0            0x1d
#define LCDIF_DEBUG1            0x1e
#define LCDIF_DEBUG2            0x1f

#define LCDIF_NREGS             0x40

#define CTRL_SFTRST             (1u << 31)
#define CTRL_CLKGATE            (1u << 30)
#define CTRL_MASTER             (1u << 5)
#define CTRL_DATA_FORMAT_16     (1u << 3)
#define CTRL_DATA_FORMAT_18     (1u << 2)
#define CTRL_DATA_FORMAT_24     (1u << 1)
#define CTRL_RUN                (1u << 0)
#define CTRL_WORD_LENGTH(v)     (((v) >> 8) & 3)
#define CTRL_DOTCLK_MODE        (1u << 17)
#define CTRL_BYPASS_COUNT       (1u << 19)

/*
 * CTRL bit 16 separates the two kinds of LCDIF_DATA write on this panel: the
 * driver sets it while the register holds a parameter byte or a pixel word
 * and clears it while the register holds a DBI opcode.  Both the boot loader
 * (pc 0x8006f800) and WinCE's display driver (pc 0xc06d9b44) follow that
 * convention, which is how the panel's window gets programmed at all.
 */
#define CTRL_DATA_PHASE         (1u << 16)

#define CTRL1_BM_ERROR_IRQ_EN       (1u << 26)
#define CTRL1_BM_ERROR_IRQ          (1u << 25)
#define CTRL1_OVERFLOW_IRQ_EN       (1u << 15)
#define CTRL1_UNDERFLOW_IRQ_EN      (1u << 14)
#define CTRL1_CUR_FRAME_DONE_IRQ_EN (1u << 13)
#define CTRL1_VSYNC_EDGE_IRQ_EN     (1u << 12)
#define CTRL1_OVERFLOW_IRQ          (1u << 11)
#define CTRL1_UNDERFLOW_IRQ         (1u << 10)
#define CTRL1_CUR_FRAME_DONE_IRQ    (1u << 9)
#define CTRL1_VSYNC_EDGE_IRQ        (1u << 8)

/* MIPI DBI / DCS commands used against this panel */
#define DBI_NOP                 0x00
#define DBI_SW_RESET            0x01
#define DBI_SLEEP_OUT           0x11
#define DBI_DISPLAY_ON         0x29
#define DBI_CASET               0x2a
#define DBI_RASET               0x2b
#define DBI_RAMWR               0x2c
#define DBI_MADCTL              0x36

#define MADCTL_MY               0x80
#define MADCTL_MX               0x40
#define MADCTL_MV               0x20

/*
 * Largest pixel push accepted in one transfer.  4M words is far beyond any
 * panel this block drives; the guard stops a bogus TRANSFER_COUNT from making
 * the model allocate without bound.
 */
#define LCDIF_MAX_PUSH_WORDS    (4 * 1024 * 1024)

typedef struct MXSLcdifState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq irq_error;
    QemuConsole *con;
    QEMUTimer *vsync;

    uint32_t regs[LCDIF_NREGS];

    /* geometry currently presented to the UI */
    int cols;
    int rows;

    /*
     * The panel: its native pixel array and the DBI controller state the
     * driver programs.  The Brain's module is 480 x 854 (portrait) mounted
     * sideways in the enclosure, which is why the console is 854 x 480 and
     * why the size never depends on what the guest happens to be scanning.
     */
    uint32_t panel_w;
    uint32_t panel_h;
    uint32_t gram_words;                /* panel_w * panel_h, for vmstate */
    uint16_t *gram;

    uint16_t col_start, col_end;        /* CASET */
    uint16_t row_start, row_end;        /* RASET */

    /*
     * The rectangle the guest's own picture occupies, in GRAM coordinates:
     * the last push that covered its whole window.  The touchscreen needs it
     * because a finger is positioned relative to the picture, and a driver
     * that insets its drawing window (this one leaves 54 of the 854 rows for
     * the module's overscan) shifts where the guest's pixel 0 sits on the
     * glass.
     */
    uint16_t pic_x0, pic_y0, pic_x1, pic_y1;
    uint16_t write_x, write_y;          /* GRAM address pointer */
    bool writing;                       /* inside a RAMWR pixel stream */

    uint8_t dbi_cmd;                    /* opcode of the transfer in progress */
    uint8_t dbi_data[8];                /* its parameter bytes */
    int dbi_len;

    uint8_t madctl;                     /* scan direction, as programmed */
    uint32_t rotate;                    /* board mounting: 0/90/180/270 */
    uint32_t refresh_hz;

    /* damage accumulated in GRAM coordinates, waiting for the UI update */
    int dmg_x0, dmg_y0, dmg_x1, dmg_y1;
    bool damaged;
    bool need_resize;
} MXSLcdifState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSLcdifState, MXS_LCDIF)

#define SWAP_INT(a, b) do { int _t = (a); (a) = (b); (b) = _t; } while (0)

static int mxs_lcdif_bpp(MXSLcdifState *s);
static void mxs_lcdif_damage(MXSLcdifState *s, int x0, int y0, int x1, int y1);
static void mxs_lcdif_put_pixel(MXSLcdifState *s, uint32_t pix);

/*
 * Size of the panel array as the DBI controller presents it, i.e. after the
 * MADCTL swap bit.
 */
static void mxs_lcdif_array_size(const MXSLcdifState *s, int *w, int *h)
{
    int aw = s->panel_w, ah = s->panel_h;

    if (s->madctl & MADCTL_MV) {
        SWAP_INT(aw, ah);
    }
    *w = aw;
    *h = ah;
}

/*
 * GRAM coordinate -> console coordinate: the panel's own scan remap (MADCTL),
 * then the board mounting rotation.  Every step is an axis aligned swap or
 * reversal, so a rectangle in GRAM stays a rectangle on the console.
 */
static void mxs_lcdif_to_console(const MXSLcdifState *s, int gx, int gy,
                                 int *cx, int *cy)
{
    int ax = gx, ay = gy, aw, ah;

    mxs_lcdif_array_size(s, &aw, &ah);
    if (s->madctl & MADCTL_MV) {
        SWAP_INT(ax, ay);
    }
    if (s->madctl & MADCTL_MX) {
        ax = aw - 1 - ax;
    }
    if (s->madctl & MADCTL_MY) {
        ay = ah - 1 - ay;
    }

    switch (s->rotate) {
    case 90:
        *cx = ah - 1 - ay;
        *cy = ax;
        break;
    case 180:
        *cx = aw - 1 - ax;
        *cy = ah - 1 - ay;
        break;
    case 270:
        *cx = ay;
        *cy = aw - 1 - ax;
        break;
    default:
        *cx = ax;
        *cy = ay;
        break;
    }
}

/* Console geometry implied by the panel array and its mounting. */
static void mxs_lcdif_console_size(const MXSLcdifState *s, int *w, int *h)
{
    int aw, ah;

    mxs_lcdif_array_size(s, &aw, &ah);
    if (s->rotate == 90 || s->rotate == 270) {
        *w = ah;
        *h = aw;
    } else {
        *w = aw;
        *h = ah;
    }
}

static void mxs_lcdif_update_irq(MXSLcdifState *s)
{
    uint32_t c1 = s->regs[LCDIF_CTRL1];
    bool level = false;

    if ((c1 & CTRL1_CUR_FRAME_DONE_IRQ) && (c1 & CTRL1_CUR_FRAME_DONE_IRQ_EN)) {
        level = true;
    }
    if ((c1 & CTRL1_VSYNC_EDGE_IRQ) && (c1 & CTRL1_VSYNC_EDGE_IRQ_EN)) {
        level = true;
    }
    if ((c1 & CTRL1_UNDERFLOW_IRQ) && (c1 & CTRL1_UNDERFLOW_IRQ_EN)) {
        level = true;
    }
    if ((c1 & CTRL1_OVERFLOW_IRQ) && (c1 & CTRL1_OVERFLOW_IRQ_EN)) {
        level = true;
    }
    qemu_set_irq(s->irq, level);
    qemu_set_irq(s->irq_error,
                 (c1 & CTRL1_BM_ERROR_IRQ) && (c1 & CTRL1_BM_ERROR_IRQ_EN));
}

/*
 * Window registers.  DBI sends four bytes, two for the start coordinate and
 * two for the end, most significant byte first -- measured on the guest: to
 * address columns 0..479 it writes CASET then 00 00 01 DF.  A panel clamps
 * its window to its own array and so do we, because an unclamped window would
 * turn into an out-of-bounds GRAM write.
 */
static void mxs_lcdif_set_window(uint16_t *start, uint16_t *end,
                                 const uint8_t *p, int len, uint32_t limit)
{
    uint32_t s0, s1;

    if (!limit) {
        return;
    }
    if (len >= 4) {
        s0 = ((uint32_t)p[0] << 8) | p[1];
        s1 = ((uint32_t)p[2] << 8) | p[3];
    } else if (len >= 2) {
        s0 = p[0];
        s1 = p[1];
    } else {
        return;
    }
    s0 = MIN(s0, limit - 1);
    s1 = MIN(s1, limit - 1);
    if (s1 < s0) {
        s1 = s0;
    }
    *start = s0;
    *end = s1;
}

static void mxs_lcdif_dbi_flush(MXSLcdifState *s)
{
    const uint8_t *p = s->dbi_data;
    int len = s->dbi_len;
    uint8_t cmd = s->dbi_cmd;

    if (cmd == DBI_NOP) {
        return;
    }
    s->dbi_cmd = DBI_NOP;
    s->dbi_len = 0;

    switch (cmd) {
    case DBI_CASET:
        mxs_lcdif_set_window(&s->col_start, &s->col_end, p, len, s->panel_w);
        trace_mxs_lcdif_window("CASET", s->col_start, s->col_end);
        break;
    case DBI_RASET:
        mxs_lcdif_set_window(&s->row_start, &s->row_end, p, len, s->panel_h);
        trace_mxs_lcdif_window("RASET", s->row_start, s->row_end);
        break;
    case DBI_RAMWR:
        /* the memory write pointer goes to the start of the window */
        s->write_x = s->col_start;
        s->write_y = s->row_start;
        s->writing = true;
        trace_mxs_lcdif_ramwr(s->write_x, s->write_y);
        break;
    case DBI_MADCTL:
        if (len >= 1) {
            uint8_t old = s->madctl;

            s->madctl = p[0];
            trace_mxs_lcdif_madctl(old, s->madctl);
            mxs_lcdif_console_size(s, &s->cols, &s->rows);
            s->need_resize = true;
            mxs_lcdif_damage(s, 0, 0, s->panel_w - 1, s->panel_h - 1);
        }
        break;
    case DBI_SW_RESET:
        s->col_start = 0;
        s->col_end = s->panel_w - 1;
        s->row_start = 0;
        s->row_end = s->panel_h - 1;
        s->write_x = 0;
        s->write_y = 0;
        s->madctl = 0;
        s->writing = false;
        mxs_lcdif_console_size(s, &s->cols, &s->rows);
        s->need_resize = true;
        mxs_lcdif_damage(s, 0, 0, s->panel_w - 1, s->panel_h - 1);
        break;
    default:
        /*
         * Sleep out, display on/off and the vendor registers change nothing
         * that QEMU can show, but they are traced: if some driver ever uses
         * one of them to affect geometry, that is where to look.
         */
        trace_mxs_lcdif_dbi(cmd, len ? p[0] : 0);
        break;
    }
}

/*
 * One word on the DBI bus.  With the data-phase bit clear it is an opcode;
 * with the bit set it is a parameter byte, or a pixel while a RAMWR stream is
 * open.  The opcode is applied when the *next* opcode arrives or when the
 * LCDIF starts a transfer, since DBI has no explicit "end of parameters".
 */
static void mxs_lcdif_dbi_write(MXSLcdifState *s, uint32_t value)
{
    if (s->regs[LCDIF_CTRL] & CTRL_DATA_PHASE) {
        if (s->writing) {
            mxs_lcdif_put_pixel(s, value);
            return;
        }
        if (s->dbi_cmd != DBI_NOP && s->dbi_len < ARRAY_SIZE(s->dbi_data)) {
            s->dbi_data[s->dbi_len++] = value & 0xff;
        }
        return;
    }
    mxs_lcdif_dbi_flush(s);
    s->dbi_cmd = value & 0xff;
}

static void mxs_lcdif_damage(MXSLcdifState *s, int x0, int y0, int x1, int y1)
{
    x0 = MAX(x0, 0);
    y0 = MAX(y0, 0);
    x1 = MIN(x1, (int)s->panel_w - 1);
    y1 = MIN(y1, (int)s->panel_h - 1);
    if (x0 > x1 || y0 > y1) {
        return;
    }
    if (!s->damaged) {
        s->damaged = true;
        s->dmg_x0 = x0;
        s->dmg_y0 = y0;
        s->dmg_x1 = x1;
        s->dmg_y1 = y1;
        return;
    }
    s->dmg_x0 = MIN(s->dmg_x0, x0);
    s->dmg_y0 = MIN(s->dmg_y0, y0);
    s->dmg_x1 = MAX(s->dmg_x1, x1);
    s->dmg_y1 = MAX(s->dmg_y1, y1);
}

static void mxs_lcdif_put_pixel(MXSLcdifState *s, uint32_t pix)
{
    uint16_t *row;

    if (s->write_x >= s->panel_w || s->write_y >= s->panel_h) {
        return;
    }
    row = s->gram + (size_t)s->write_y * s->panel_w;
    if (row[s->write_x] != (uint16_t)pix) {
        row[s->write_x] = (uint16_t)pix;
        mxs_lcdif_damage(s, s->write_x, s->write_y,
                        s->write_x, s->write_y);
    }

    /* DBI address pointer: fill the window columns, then step down a row */
    if (s->write_x >= s->col_end) {
        s->write_x = s->col_start;
        if (s->write_y >= s->row_end) {
            s->write_y = s->row_start;
        } else {
            s->write_y++;
        }
    } else {
        s->write_x++;
    }
}

/*
 * The transfer the guest just kicked off.  A 1x1 count is a register transfer
 * (handled through LCDIF_DATA); anything bigger is a scan-out of DRAM into
 * the panel's GRAM, which is the only thing that puts pixels on the screen.
 */
static void mxs_lcdif_start_transfer(MXSLcdifState *s)
{
    uint32_t tc = s->regs[LCDIF_TRANSFER_COUNT];
    uint32_t lines = (tc >> 16) & 0xffff;
    uint32_t words = tc & 0xffff;
    uint32_t base = s->regs[LCDIF_CUR_BUF] ? s->regs[LCDIF_CUR_BUF]
                                           : s->regs[LCDIF_NEXT_BUF];
    int bpp = mxs_lcdif_bpp(s);
    size_t word_bytes = bpp == 16 ? 2 : 4;
    uint64_t total = (uint64_t)lines * words;
    size_t chunk_words = 32768;
    g_autofree uint8_t *chunk = NULL;
    hwaddr addr;
    size_t left;

    /* parameters collected for the RAMWR that armed this transfer */
    mxs_lcdif_dbi_flush(s);

    if (!words || !lines || total > LCDIF_MAX_PUSH_WORDS) {
        return;
    }
    if (!base) {
        return;
    }

    trace_mxs_lcdif_push(words, lines, base, bpp);

    /*
     * What the panel shows can only grow as far as this model is concerned:
     * the module scans its whole array every frame, and a transfer that covers
     * a band (the 480x54 and 480x48 refreshes WinCE does for one widget) is a
     * damage window, not a new picture geometry.  Taking the box from the
     * current window -- which is what "a push of the entire window defines the
     * picture" amounted to, since the comparison was against that same latched
     * window -- moved the origin used by the touchscreen to the middle of the
     * last repaint: measured as every delivered coordinate sitting 54 px left
     * of the finger after a 54-line push (runs/s89, mode ui: tap(252,130) came
     * back as plate X 1039, i.e. logical 198).
     *
     * Growing the box from *every* push has the same defect in the other
     * direction: a band refresh is a window of its own, so its column range
     * widened the picture box towards the panel's overscan side and the
     * touchscreen ended up measuring the finger from a corner the guest never
     * drew in.  Only a push that spans the panel in the *other* axis carries
     * information about where the picture sits: a full-height scan says where
     * the picture starts and ends along the columns, a full-width one says the
     * same about the rows -- and it is the rows that the touchkey strip band
     * lies beyond, because the module is mounted turned.  Grow, never shrink,
     * and only on a scan that can actually define an edge.
     */
    if (s->row_start == 0 && s->row_end == s->panel_h - 1) {
        s->pic_x0 = MIN(s->pic_x0, s->col_start);
        s->pic_x1 = MAX(s->pic_x1, s->col_end);
    }
    if (s->col_start == 0 && s->col_end == s->panel_w - 1) {
        s->pic_y0 = MIN(s->pic_y0, s->row_start);
        s->pic_y1 = MAX(s->pic_y1, s->row_end);
    }

    /*
     * The block streams its words straight out of DRAM -- there is no stride
     * register, so lines and columns are one contiguous run and the panel's
     * window decides where in GRAM they land.  Read it in chunks so a
     * full-screen push does not need a temporary buffer of its own size.
     */
    chunk = g_malloc(chunk_words * word_bytes);
    addr = base;
    left = total * word_bytes;
    while (left) {
        size_t n = MIN(left, chunk_words * word_bytes);
        size_t i;

        if (address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, chunk, n)
            != MEMTX_OK) {
            break;
        }
        for (i = 0; i < n; i += word_bytes) {
            uint32_t v = word_bytes == 2 ? lduw_le_p(chunk + i)
                                         : ldl_le_p(chunk + i);
            mxs_lcdif_put_pixel(s, v);
        }
        addr += n;
        left -= n;
    }
    s->writing = false;
}

static void mxs_lcdif_vsync_tick(void *opaque)
{
    MXSLcdifState *s = opaque;

    if (s->regs[LCDIF_CTRL] & CTRL_RUN) {
        s->regs[LCDIF_CTRL1] |= CTRL1_CUR_FRAME_DONE_IRQ | CTRL1_VSYNC_EDGE_IRQ;
        mxs_lcdif_update_irq(s);
        /*
         * Outside dot-clock mode the block pushes the programmed transfer
         * count and clears RUN again; Linux and WinCE both poll for that.
         */
        if (!(s->regs[LCDIF_CTRL] & CTRL_DOTCLK_MODE)) {
            s->regs[LCDIF_CTRL] &= ~CTRL_RUN;
        }
        if (s->regs[LCDIF_NEXT_BUF]) {
            s->regs[LCDIF_CUR_BUF] = s->regs[LCDIF_NEXT_BUF];
        }
        /*
         * A continuous scan (dot-clock mode, the way a plain RGB panel is
         * driven) refreshes its window on every frame, so the frame boundary
         * is where its pixels get read out of DRAM.
         */
        if (s->regs[LCDIF_CTRL] & CTRL_DOTCLK_MODE) {
            mxs_lcdif_start_transfer(s);
        }
    }
    timer_mod(s->vsync, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / (s->refresh_hz ? s->refresh_hz : 60));
}

static int mxs_lcdif_bpp(MXSLcdifState *s)
{
    uint32_t ctrl = s->regs[LCDIF_CTRL];

    /*
     * DATA_FORMAT_* selects the bits per pixel fed to the panel, WORD_LENGTH
     * how many bits cross the bus per word.  The Brain's display driver ends
     * its init with CTRL = 0x0c050021: no DATA_FORMAT bits and WORD_LENGTH 0,
     * i.e. 16-bit words carrying one RGB565 pixel each -- which is also what
     * the transfer count (480 words per line for a 480 pixel line) requires.
     */
    if (ctrl & CTRL_DATA_FORMAT_16) {
        return 16;
    }
    if (ctrl & CTRL_DATA_FORMAT_18) {
        return 32;     /* 18 bpp stored in 32 bit words */
    }
    if (ctrl & CTRL_DATA_FORMAT_24) {
        return 32;     /* 24 bpp stored in 32 bit words */
    }
    switch (CTRL_WORD_LENGTH(ctrl)) {
    case 0:     /* 16 bit */
        return 16;
    case 1:     /* 8 bit */
        return 8;
    case 2:     /* 18 bit, stored in 32 bit words */
    case 3:     /* 24 bit, stored in 32 bit words */
    default:
        return 32;
    }
}

static inline uint32_t mxs_lcdif_pix16(MXSLcdifState *s, uint16_t v)
{
    unsigned r, g, b;

    if (s->regs[LCDIF_CTRL] & CTRL_DATA_FORMAT_16) {
        /* ARGB1555 */
        r = ((v >> 10) & 0x1f) << 3;
        g = ((v >> 5) & 0x1f) << 3;
        b = (v & 0x1f) << 3;
    } else {
        /* RGB565 */
        r = ((v >> 11) & 0x1f) << 3;
        g = ((v >> 5) & 0x3f) << 2;
        b = (v & 0x1f) << 3;
    }
    return rgb_to_pixel32(r | (r >> 5), g | (g >> 6), b | (b >> 5));
}

static void mxs_lcdif_update_display(void *opaque)
{
    MXSLcdifState *s = opaque;
    DisplaySurface *surface;
    int x, y, cx0, cy0, cx1 = 0, cy1 = 0;

    if (!s->con) {
        return;
    }
    if (s->need_resize) {
        s->need_resize = false;
        qemu_console_resize(s->con, s->cols, s->rows);
    }
    if (!s->damaged) {
        return;
    }
    surface = qemu_console_surface(s->con);
    if (!surface || surface_bits_per_pixel(surface) != 32) {
        return;
    }

    mxs_lcdif_to_console(s, s->dmg_x0, s->dmg_y0, &cx0, &cy0);
    mxs_lcdif_to_console(s, s->dmg_x1, s->dmg_y1, &cx1, &cy1);
    s->damaged = false;

    if (!s->madctl && s->rotate == 270) {
        /*
         * The case this hardware actually uses: console column = GRAM row,
         * console row = last GRAM column minus the pixel index.  Worth a
         * dedicated loop because a full screen push is 410k pixels.
         */
        uint32_t *surf = (uint32_t *)surface_data(surface);
        int dstride = surface_stride(surface) / 4;

        for (y = s->dmg_y0; y <= s->dmg_y1; y++) {
            const uint16_t *row = s->gram + (size_t)y * s->panel_w;
            uint32_t *dest = surf + (size_t)(s->panel_w - 1 - s->dmg_x0)
                                    * dstride + y;

            for (x = s->dmg_x0; x <= s->dmg_x1; x++) {
                *dest = mxs_lcdif_pix16(s, row[x]);
                dest -= dstride;
            }
        }
    } else {
        for (y = s->dmg_y0; y <= s->dmg_y1; y++) {
            const uint16_t *row = s->gram + (size_t)y * s->panel_w;

            for (x = s->dmg_x0; x <= s->dmg_x1; x++) {
                int cx, cy;
                uint32_t *dest;

                mxs_lcdif_to_console(s, x, y, &cx, &cy);
                if (cx < 0 || cy < 0 || cx >= surface_width(surface)
                    || cy >= surface_height(surface)) {
                    continue;
                }
                dest = (uint32_t *)surface_data(surface)
                       + (size_t)cy * (surface_stride(surface) / 4) + cx;
                *dest = mxs_lcdif_pix16(s, row[x]);
            }
        }
    }

    dpy_gfx_update(s->con, MIN(cx0, cx1), MIN(cy0, cy1),
                   abs(cx1 - cx0) + 1, abs(cy1 - cy0) + 1);
    trace_mxs_lcdif_update(s->dmg_x0, s->dmg_y0, s->dmg_x1, s->dmg_y1);
}

static void mxs_lcdif_invalidate_display(void *opaque)
{
    MXSLcdifState *s = opaque;

    mxs_lcdif_damage(s, 0, 0, s->panel_w - 1, s->panel_h - 1);
}

static const GraphicHwOps mxs_lcdif_gfx_ops = {
    .invalidate = mxs_lcdif_invalidate_display,
    .gfx_update = mxs_lcdif_update_display,
};

/*
 * The box the guest's picture occupies, in console coordinates, together
 * with the size of the console (the glass): where picture column 0 sits on
 * the panel and how many rows the panel has.  Every touch decision is
 * measured from this one origin, so the plate law and the touchkey strip can
 * not disagree about where the finger is.  Returns false while there is no
 * panel to ask or no picture in it yet.  Any of the four outputs may be
 * NULL: a caller takes the parts it needs and ignores the rest.
 */
static bool mxs_lcdif_box_at(const MXSLcdifState *s, int *bx0, int *by0,
                             int *cols, int *rows)
{
    int gx, gy, x0, y0;

    if (s->cols < 1 || s->rows < 1) {
        return false;
    }
    /*
     * Normalise locally and write out only what was asked for.  Passing the
     * result straight through the caller's pointers was fatal for anyone who
     * wanted just the row origin -- which is what mxs_lradc_set_touch() does,
     * so the first mouse motion the guest received killed the emulator.
     */
    mxs_lcdif_to_console(s, s->pic_x0, s->pic_y0, &x0, &y0);
    mxs_lcdif_to_console(s, s->pic_x1, s->pic_y1, &gx, &gy);
    x0 = MIN(x0, gx);
    y0 = MIN(y0, gy);
    if (bx0) {
        *bx0 = x0;
    }
    if (by0) {
        *by0 = y0;
    }
    if (cols) {
        *cols = s->cols;
    }
    if (rows) {
        *rows = s->rows;
    }
    return true;
}

bool mxs_lcdif_touch_box(DeviceState *dev, int *bx0, int *by0,
                         int *cols, int *rows)
{
    if (!dev || !object_dynamic_cast(OBJECT(dev), TYPE_MXS_LCDIF)) {
        return false;
    }
    return mxs_lcdif_box_at(MXS_LCDIF(dev), bx0, by0, cols, rows);
}

/*
 * Geometry the touchscreen has to agree with.  A finger sits on the panel
 * array, so convert the front end's normalised absolute axis into a GRAM
 * coordinate and let the LRADC apply the plate law to it.  This function
 * deliberately knows nothing about the guest's display modes: the console
 * size and the mounting are all that is needed, which is why the touch
 * mapping stays correct from the boot loader through the desktop.
 */
bool mxs_lcdif_touch_position(DeviceState *dev, int axis_x, int axis_y,
                              int *px, int *py)
{
    MXSLcdifState *s;
    int cx, cy, bx0, by0;

    if (!dev || !object_dynamic_cast(OBJECT(dev), TYPE_MXS_LCDIF)) {
        return false;
    }
    s = MXS_LCDIF(dev);
    if (s->cols < 1 || s->rows < 1) {
        return false;
    }

    /*
     * The front end scales a pixel position p to p * INPUT_EVENT_ABS_MAX /
     * size, so undo it rounded to nearest.  Truncating here costs a
     * systematic half pixel on every sample and loses the last column of the
     * glass, which is precisely how "the right edge cannot be reached" would
     * look if the model were careless at this point.
     */
    cx = (axis_x * s->cols + INPUT_EVENT_ABS_MAX / 2) / INPUT_EVENT_ABS_MAX;
    cy = (axis_y * s->rows + INPUT_EVENT_ABS_MAX / 2) / INPUT_EVENT_ABS_MAX;
    cx = MIN(MAX(cx, 0), s->cols - 1);
    cy = MIN(MAX(cy, 0), s->rows - 1);

    /*
     * The plate law is expressed per pixel of the guest's picture, so answer
     * in that frame: take the picture's own box -- mapped through the same
     * array -> console transform, which is what carries the mounting rotation
     * and any mirroring -- and offset the finger from its top left corner.
     * Reporting the raw GRAM coordinate instead would hand the X plate an
     * array *column* (and, with the module mounted sideways, an inverted one).
     */
    if (!mxs_lcdif_box_at(s, &bx0, &by0, NULL, NULL)) {
        return false;
    }
    *px = cx - bx0;
    *py = cy - by0;
    return true;
}

static uint64_t mxs_lcdif_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSLcdifState *s = MXS_LCDIF(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= LCDIF_NREGS) {
        return 0;
    }

    switch (idx) {
    case LCDIF_VERSION:
        val = 0x04000000;
        break;
    case LCDIF_STAT:
        /* TXFIFO empty, LFIFO empty, not busy */
        val = (1u << 29) | (1u << 28) | (1u << 27);
        break;
    case LCDIF_DATA:
        /*
         * A DBI read (the panel's own read phase, used to fetch an ID).  The
         * module is modelled as write-only, so answer with 0 and let the
         * trace show that somebody asked.
         */
        trace_mxs_lcdif_dbi_read();
        val = 0;
        break;
    default:
        val = s->regs[idx];
        break;
    }

    return mxs_bank_extract(offset, size, val);
}

static void mxs_lcdif_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    MXSLcdifState *s = MXS_LCDIF(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= LCDIF_NREGS) {
        return;
    }
    if (idx == LCDIF_VERSION || idx == LCDIF_STAT) {
        return;
    }

    val = mxs_bank_apply(s->regs[idx], offset, value, size);

    switch (idx) {
    case LCDIF_CTRL: {
        uint32_t old = s->regs[idx];

        val = mxs_bank_sftrst(old, val);
        s->regs[idx] = val;
        if ((val & CTRL_RUN) && !(old & CTRL_RUN)) {
            /*
             * Complete the transfer here rather than from a timer: the DBI
             * bus is far ahead of a guest that spins on RUN, and the driver
             * reprograms the block a few dozen instructions later.  Clearing
             * RUN now is what the hardware's self-clearing RUN bit means to
             * that reader -- measured, WinCE writes CTRL with RUN set and
             * polls it back clear before touching anything else.
             */
            s->regs[idx] = val & ~CTRL_RUN;
            s->regs[LCDIF_CTRL1] |= CTRL1_CUR_FRAME_DONE_IRQ |
                                    CTRL1_VSYNC_EDGE_IRQ;
            if (s->regs[LCDIF_NEXT_BUF]) {
                s->regs[LCDIF_CUR_BUF] = s->regs[LCDIF_NEXT_BUF];
            }
            if (val & CTRL_MASTER) {
                mxs_lcdif_start_transfer(s);
            }
            mxs_lcdif_update_irq(s);
        }
        break;
    }
    case LCDIF_CTRL1:
        s->regs[idx] = val;
        mxs_lcdif_update_irq(s);
        break;
    case LCDIF_DATA:
        s->regs[idx] = val;
        mxs_lcdif_dbi_write(s, val);
        break;
    default:
        s->regs[idx] = val;
        break;
    }
}

static const MemoryRegionOps mxs_lcdif_ops = {
    .read = mxs_lcdif_read,
    .write = mxs_lcdif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_lcdif_reset(DeviceState *dev)
{
    MXSLcdifState *s = MXS_LCDIF(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[LCDIF_CTRL] = CTRL_SFTRST | CTRL_CLKGATE;
    s->dbi_cmd = DBI_NOP;
    s->dbi_len = 0;
    s->writing = false;
    s->madctl = 0;
    s->col_start = 0;
    s->col_end = s->panel_w - 1;
    s->row_start = 0;
    s->row_end = s->panel_h - 1;
    s->write_x = 0;
    s->write_y = 0;
    s->pic_x0 = 0;
    s->pic_y0 = 0;
    s->pic_x1 = s->panel_w - 1;
    s->pic_y1 = s->panel_h - 1;
    if (s->gram) {
        memset(s->gram, 0, (size_t)s->panel_w * s->panel_h * sizeof(*s->gram));
    }
    mxs_lcdif_console_size(s, &s->cols, &s->rows);
    s->damaged = false;
    s->need_resize = true;
}

static void mxs_lcdif_realize(DeviceState *dev, Error **errp)
{
    MXSLcdifState *s = MXS_LCDIF(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);


    if (s->panel_w < 1 || s->panel_h < 1 ||
        s->panel_w > 8192 || s->panel_h > 8192) {
        error_setg(errp, "mxs-lcdif: panel size %ux%u out of range",
                   s->panel_w, s->panel_h);
        return;
    }
    if (s->rotate != 0 && s->rotate != 90 && s->rotate != 180 &&
        s->rotate != 270) {
        error_setg(errp, "mxs-lcdif: rotate must be 0, 90, 180 or 270");
        return;
    }

    s->gram_words = s->panel_w * s->panel_h;
    s->gram = g_new0(uint16_t, s->gram_words);

    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_lcdif_ops, s,
                          "mxs-lcdif", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->irq_error);

    mxs_lcdif_console_size(s, &s->cols, &s->rows);
    s->con = graphic_console_init(dev, 0, &mxs_lcdif_gfx_ops, s);
    qemu_console_resize(s->con, s->cols, s->rows);
    mxs_lcdif_damage(s, 0, 0, s->panel_w - 1, s->panel_h - 1);

    s->vsync = timer_new_ns(QEMU_CLOCK_VIRTUAL, mxs_lcdif_vsync_tick, s);
    timer_mod(s->vsync, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / (s->refresh_hz ? s->refresh_hz : 60));
}

static void mxs_lcdif_unrealize(DeviceState *dev)
{
    MXSLcdifState *s = MXS_LCDIF(dev);

    g_free(s->gram);
    s->gram = NULL;
}

static const Property mxs_lcdif_properties[] = {
    DEFINE_PROP_UINT32("panel-w", MXSLcdifState, panel_w, 480),
    DEFINE_PROP_UINT32("panel-h", MXSLcdifState, panel_h, 854),
    DEFINE_PROP_UINT32("rotate", MXSLcdifState, rotate, 0),
    DEFINE_PROP_UINT32("refresh-hz", MXSLcdifState, refresh_hz, 60),
};

static int mxs_lcdif_post_load(void *opaque, int version_id)
{
    MXSLcdifState *s = opaque;

    mxs_lcdif_console_size(s, &s->cols, &s->rows);
    s->damaged = false;
    s->need_resize = true;
    mxs_lcdif_damage(s, 0, 0, s->panel_w - 1, s->panel_h - 1);
    return 0;
}

static const VMStateDescription vmstate_mxs_lcdif = {
    .name = "mxs-lcdif",
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = mxs_lcdif_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSLcdifState, LCDIF_NREGS),
        VMSTATE_UINT16(col_start, MXSLcdifState),
        VMSTATE_UINT16(col_end, MXSLcdifState),
        VMSTATE_UINT16(row_start, MXSLcdifState),
        VMSTATE_UINT16(row_end, MXSLcdifState),
        VMSTATE_UINT16(pic_x0, MXSLcdifState),
        VMSTATE_UINT16(pic_y0, MXSLcdifState),
        VMSTATE_UINT16(pic_x1, MXSLcdifState),
        VMSTATE_UINT16(pic_y1, MXSLcdifState),
        VMSTATE_UINT16(write_x, MXSLcdifState),
        VMSTATE_UINT16(write_y, MXSLcdifState),
        VMSTATE_UINT8(madctl, MXSLcdifState),
        VMSTATE_BOOL(writing, MXSLcdifState),
        VMSTATE_UINT8(dbi_cmd, MXSLcdifState),
        VMSTATE_UINT8_ARRAY(dbi_data, MXSLcdifState, 8),
        VMSTATE_INT32(dbi_len, MXSLcdifState),
        VMSTATE_UINT32(gram_words, MXSLcdifState),
        VMSTATE_VBUFFER_UINT32(gram, MXSLcdifState, 2, NULL, gram_words),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_lcdif_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mxs_lcdif_realize;
    dc->unrealize = mxs_lcdif_unrealize;
    device_class_set_legacy_reset(dc, mxs_lcdif_reset);
    device_class_set_props(dc, mxs_lcdif_properties);
    dc->vmsd = &vmstate_mxs_lcdif;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo mxs_lcdif_types[] = {
    {
        .name           = TYPE_MXS_LCDIF,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSLcdifState),
        .class_init     = mxs_lcdif_class_init,
    },
};

DEFINE_TYPES(mxs_lcdif_types)
