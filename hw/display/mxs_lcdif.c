/*
 * Freescale i.MX28 (MXS) LCDIF display controller
 *
 * Enough of the block to let the WinCE / Linux display drivers of the
 * SHARP Brain scan a framebuffer out of DRAM onto a QEMU console.
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
#include "ui/pixel_ops.h"
#include "system/address-spaces.h"

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

#define VDCTRL0_ENABLE_PRESENT      (1u << 28)
#define VDCTRL0_VSYNC_PULSE_WIDTH_UNIT (1u << 20)

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
    int invalidate;

    /*
     * Latched description of the last real frame the guest pushed out.
     * The MPU ("system") interface is also used to send single word
     * commands to the panel, with TRANSFER_COUNT temporarily set to 1x1,
     * so the live register values cannot be used directly.
     */
    uint32_t fb_addr;
    uint32_t fb_width;
    uint32_t fb_height;
    int      fb_bpp;
    bool     fb_argb1555;

    /* board defaults, used until the guest programs the transfer count */
    uint32_t default_width;
    uint32_t default_height;
    uint32_t rotate;        /* 0/90/180/270 */
    uint32_t refresh_hz;
} MXSLcdifState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSLcdifState, MXS_LCDIF)

static int mxs_lcdif_bpp(MXSLcdifState *s);

/*
 * Automatic orientation: a landscape TRANSFER_COUNT (width >= height)
 * is shown as-is.  A portrait scan is turned by board lcd-rotate
 * (default 270) so the window is landscape.  No extra machine property
 * is required at runtime.
 */
static uint32_t mxs_lcdif_view_rotate(const MXSLcdifState *s, int src_w,
                                      int src_h)
{
    if (src_w >= src_h) {
        return 0;
    }
    return s->rotate;
}

static void mxs_lcdif_out_size(const MXSLcdifState *s, int src_w, int src_h,
                               int *out_w, int *out_h)
{
    uint32_t rotate = mxs_lcdif_view_rotate(s, src_w, src_h);
    bool swap_xy = (rotate == 90 || rotate == 270);

    *out_w = swap_xy ? src_h : src_w;
    *out_h = swap_xy ? src_w : src_h;
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
 * The guest started a transfer.  If it looks like a framebuffer scan out
 * (bus master mode, a plausible picture size) remember where and how big
 * it was so that the console keeps showing it afterwards.
 */
static void mxs_lcdif_latch_frame(MXSLcdifState *s)
{
    uint32_t tc = s->regs[LCDIF_TRANSFER_COUNT];
    uint32_t w = tc & 0xffff;
    uint32_t h = (tc >> 16) & 0xffff;
    uint32_t base = s->regs[LCDIF_CUR_BUF] ? s->regs[LCDIF_CUR_BUF]
                                           : s->regs[LCDIF_NEXT_BUF];

    if (!(s->regs[LCDIF_CTRL] & CTRL_MASTER) || !base) {
        return;
    }
    /*
     * MPU interface reuses the same registers to send panel command
     * words (TRANSFER_COUNT 1x1).  Those are not a scanout.  Only a
     * bus-master transfer large enough to be a picture updates the
     * console descriptor.  Silicon still completes the small transfer
     * (RUN self-clears); we just do not treat it as a new framebuffer.
     */
    if (w < 64 || h < 64 || w > 4096 || h > 4096) {
        return;
    }

    s->fb_addr = base;
    s->fb_width = w;
    s->fb_height = h;
    s->fb_bpp = mxs_lcdif_bpp(s);
    s->fb_argb1555 = (s->regs[LCDIF_CTRL] & CTRL_DATA_FORMAT_16) != 0;
    s->invalidate = 1;
    {
        /* BRAIN_LCDTRACE: log each latched frame descriptor so we can find
         * when the guest pushes a full panel-sized framebuffer. */
        if (unlikely(getenv("BRAIN_LCDTRACE"))) {
            fprintf(stderr, "brain-lcdfb-latch: base=0x%08x w=%u h=%u "
                    "bpp=%d pc=0x%08x\n",
                    base, w, h, s->fb_bpp, (unsigned)mxs_trace_guest_pc());
        }
    }
}

static void mxs_lcdif_vsync_tick(void *opaque)
{
    MXSLcdifState *s = opaque;

    if (s->regs[LCDIF_CTRL] & CTRL_RUN) {
        s->regs[LCDIF_CTRL1] |= CTRL1_CUR_FRAME_DONE_IRQ | CTRL1_VSYNC_EDGE_IRQ;
        mxs_lcdif_update_irq(s);
        /*
         * In non "master + dotclk" mode the RUN bit self clears once the
         * frame has been pushed out.  Linux/WinCE poll it.
         */
        if (!(s->regs[LCDIF_CTRL] & CTRL_DOTCLK_MODE)) {
            s->regs[LCDIF_CTRL] &= ~CTRL_RUN;
        }
        if (s->regs[LCDIF_NEXT_BUF]) {
            s->regs[LCDIF_CUR_BUF] = s->regs[LCDIF_NEXT_BUF];
        }
    }
    /*
     * Vsync only raises the hardware IRQ.  The console copies DRAM
     * when the guest latches a new frame (invalidate).  Re-copying
     * on every tick flashed the window and doubled DMA cost.
     */
    timer_mod(s->vsync, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / (s->refresh_hz ? s->refresh_hz : 60));
}

static int mxs_lcdif_bpp(MXSLcdifState *s)
{
    uint32_t ctrl = s->regs[LCDIF_CTRL];

    /*
     * The framebuffer pixel format is selected by the
     * DATA_FORMAT_* bits in CTRL (16/18/24 bpp), not by
     * WORD_LENGTH which controls how many bits are pushed out
     * on the panel bus.  The WinCE display driver for the
     * PW-SH6 sets CTRL = 0x0c050021 at the end of its init
     * (DATA_FORMAT_24 + DATA_FORMAT_18 + MASTER + RUN), so
     * it expects 24 bits per pixel to be stored in 32 bit
     * words in DRAM.
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

/*
 * Dump the last latched framebuffer as an unrotated PPM (binary P6) to
 * @path.  Returns 0 on success.  Used by the 'brain_lcdfb' HMP command to
 * verify what the guest is actually displaying (splash / desktop / menu)
 * when QEMU runs headless without an SDL/VNC console.
 */
int mxs_lcdif_dump_fb_opt(DeviceState *dev, const char *path,
                          uint32_t base, uint32_t w, uint32_t h, int bpp)
{
    MXSLcdifState *s = MXS_LCDIF(dev);
    FILE *fp;
    int src_w, src_h, y, x;
    hwaddr fb_base;
    int src_stride;
    g_autofree uint8_t *line = NULL;

    if (!path) {
        return -1;
    }
    if (!base || !w || !h) {
        /* not specified: use the last latched descriptor */
        if (!s->fb_addr) {
            return -1;
        }
        fb_base = s->fb_addr;
        src_w = s->fb_width;
        src_h = s->fb_height;
        src_w = src_w < 1 ? 1 : src_w;
        src_h = src_h < 1 ? 1 : src_h;
        bpp = s->fb_bpp;
    } else {
        fb_base = base;
        src_w = w;
        src_h = h;
        if (bpp <= 0) {
            bpp = s->fb_bpp;
        }
    }
    if (src_w > 4096 || src_h > 4096 || src_w < 1 || src_h < 1) {
        return -1;
    }
    src_stride = src_w * (bpp / 8);

    fp = fopen(path, "wb");
    if (!fp) {
        return -1;
    }
    fprintf(fp, "P6\n%d %d\n255\n", src_w, src_h);

    line = g_malloc(src_stride);
    for (y = 0; y < src_h; y++) {
        address_space_read(&address_space_memory, fb_base + (hwaddr)y * src_stride,
                           MEMTXATTRS_UNSPECIFIED, line, src_stride);
        for (x = 0; x < src_w; x++) {
            uint32_t pix, r, g, b;

            switch (bpp) {
            case 16: {
                uint16_t v = lduw_le_p(line + x * 2);
                if (s->fb_argb1555) {
                    r = ((v >> 10) & 0x1f) << 3;
                    g = ((v >> 5) & 0x1f) << 3;
                    b = (v & 0x1f) << 3;
                } else {
                    r = ((v >> 11) & 0x1f) << 3;
                    g = ((v >> 5) & 0x3f) << 2;
                    b = (v & 0x1f) << 3;
                }
                break;
            }
            case 8: {
                uint8_t v = line[x];
                r = g = b = v;
                break;
            }
            default: {
                uint32_t v = ldl_le_p(line + x * 4);
                r = (v >> 16) & 0xff;
                g = (v >> 8) & 0xff;
                b = v & 0xff;
                break;
            }
            }
            pix = rgb_to_pixel32(r, g, b);
            fputc((pix >> 16) & 0xff, fp);
            fputc((pix >> 8) & 0xff, fp);
            fputc(pix & 0xff, fp);
        }
    }

    fclose(fp);
    return 0;
}

/* dump the last latched descriptor (existing behaviour) */
int mxs_lcdif_dump_fb(DeviceState *dev, const char *path)
{
    return mxs_lcdif_dump_fb_opt(dev, path, 0, 0, 0, 0);
}

static inline uint32_t mxs_lcdif_pix16(MXSLcdifState *s, uint16_t v)
{
    unsigned r, g, b;

    if (s->fb_argb1555) {
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
    DisplaySurface *surface = qemu_console_surface(s->con);
    int src_w, src_h, out_w, out_h, bpp, y, x;
    hwaddr base;
    g_autofree uint8_t *fb = NULL;
    uint8_t *line;
    uint32_t *dest;
    int src_stride;
    size_t fb_bytes;
    uint32_t rotate;
    int surf_w, surf_h;

    if (!s->fb_addr) {
        return;
    }
    if (!s->invalidate) {
        return;
    }

    src_w = s->fb_width;
    src_h = s->fb_height;
    bpp = s->fb_bpp;
    base = s->fb_addr;
    rotate = mxs_lcdif_view_rotate(s, src_w, src_h);
    mxs_lcdif_out_size(s, src_w, src_h, &out_w, &out_h);

    /*
     * Never shrink below the viewed panel (PW-SH6: 854x480 landscape,
     * 5.5" 121.1mm x 68.0mm).  Realize used to size the console to the
     * unrotated 480x854 scan, then never-shrink kept height 854 and
     * clipped the right edge of a landscape blit.  Grow only.
     */
    {
        int pw, ph, need_w, need_h;

        mxs_lcdif_out_size(s, s->default_width, s->default_height, &pw, &ph);
        need_w = MAX(out_w, pw);
        need_h = MAX(out_h, ph);
        if (need_w < s->cols) {
            need_w = s->cols;
        }
        if (need_h < s->rows) {
            need_h = s->rows;
        }
        if (need_w != s->cols || need_h != s->rows) {
            s->cols = need_w;
            s->rows = need_h;
            qemu_console_resize(s->con, need_w, need_h);
            surface = qemu_console_surface(s->con);
            s->invalidate = 1;
        }
    }
    if (!surface || surface_bits_per_pixel(surface) != 32) {
        return;
    }
    surf_w = surface_width(surface);
    surf_h = surface_height(surface);
    if (surf_w < 1 || surf_h < 1) {
        return;
    }

    src_stride = src_w * (bpp / 8);
    fb_bytes = (size_t)src_h * src_stride;
    fb = g_malloc(fb_bytes);
    address_space_read(&address_space_memory, base, MEMTXATTRS_UNSPECIFIED,
                       fb, fb_bytes);

    /* Clear the (possibly larger) console so a smaller frame does not
     * leave stale pixels around the edges. */
    memset(surface_data(surface), 0,
           (size_t)surface_stride(surface) * surface_height(surface));

    /*
     * PW-SH6 default: 32 bpp 480x854 portrait, console rotate 270.
     * Keep the generic loop for other formats; this path avoids a
     * per-pixel switch and a dest-row lookup on every sample.
     */
    if (bpp == 32 && rotate == 270) {
        uint8_t *surf = surface_data(surface);
        int dstride = surface_stride(surface);

        for (y = 0; y < src_h; y++) {
            uint32_t *src = (uint32_t *)(fb + (size_t)y * src_stride);
            int dx = y;

            if (dx < 0 || dx >= surf_w) {
                continue;
            }
            for (x = 0; x < src_w; x++) {
                uint32_t v = le32_to_cpu(src[x]);
                int dy = src_w - 1 - x;

                if (dy < 0 || dy >= surf_h) {
                    continue;
                }
                dest = (uint32_t *)(surf + (size_t)dy * dstride);
                dest[dx] = rgb_to_pixel32((v >> 16) & 0xff, (v >> 8) & 0xff,
                                          v & 0xff);
            }
        }
    } else {
        int dstride = surface_stride(surface);
        uint8_t *surf = surface_data(surface);

        for (y = 0; y < src_h; y++) {
            line = fb + (size_t)y * src_stride;
            for (x = 0; x < src_w; x++) {
                uint32_t pix;
                int dx, dy;

                switch (bpp) {
                case 16:
                    pix = mxs_lcdif_pix16(s, lduw_le_p(line + x * 2));
                    break;
                case 8: {
                    uint8_t v = line[x];
                    pix = rgb_to_pixel32(v, v, v);
                    break;
                }
                default: {
                    uint32_t v = ldl_le_p(line + x * 4);
                    pix = rgb_to_pixel32((v >> 16) & 0xff, (v >> 8) & 0xff,
                                         v & 0xff);
                    break;
                }
                }

                switch (rotate) {
                case 90:
                    dx = src_h - 1 - y;
                    dy = x;
                    break;
                case 180:
                    dx = src_w - 1 - x;
                    dy = src_h - 1 - y;
                    break;
                case 270:
                    dx = y;
                    dy = src_w - 1 - x;
                    break;
                default:
                    dx = x;
                    dy = y;
                    break;
                }
                if (dx < 0 || dy < 0 || dx >= surf_w || dy >= surf_h) {
                    continue;
                }
                dest = (uint32_t *)(surf + (size_t)dy * dstride);
                dest[dx] = pix;
            }
        }
    }

    dpy_gfx_update_full(s->con);
    s->invalidate = 0;
}

static void mxs_lcdif_invalidate_display(void *opaque)
{
    MXSLcdifState *s = opaque;

    s->invalidate = 1;
}

static const GraphicHwOps mxs_lcdif_gfx_ops = {
    .invalidate = mxs_lcdif_invalidate_display,
    .gfx_update = mxs_lcdif_update_display,
};

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
            mxs_lcdif_latch_frame(s);
            /*
             * Outside of the continuous "dotclk" mode the block pushes the
             * programmed transfer count out and clears RUN again.  The
             * WinCE MPU panel driver sends single command words this way
             * and spins on RUN with a fairly small retry count, so the
             * transfer has to look instantaneous.
             */
            if (!(val & CTRL_DOTCLK_MODE)) {
                s->regs[idx] &= ~CTRL_RUN;
                s->regs[LCDIF_CTRL1] |= CTRL1_CUR_FRAME_DONE_IRQ |
                                        CTRL1_VSYNC_EDGE_IRQ;
                if (s->regs[LCDIF_NEXT_BUF]) {
                    s->regs[LCDIF_CUR_BUF] = s->regs[LCDIF_NEXT_BUF];
                }
                mxs_lcdif_update_irq(s);
            }
        }
        break;
    }
    case LCDIF_CTRL1:
        s->regs[idx] = val;
        mxs_lcdif_update_irq(s);
        break;
    case LCDIF_NEXT_BUF:
        s->regs[idx] = val;
        if (!s->regs[LCDIF_CUR_BUF]) {
            s->regs[LCDIF_CUR_BUF] = val;
        }
        /*
         * The WinCE display driver re-arms the LCDIF by writing the
         * NEXT_BUF register every time it has new pixels in DRAM.  Re-latch
         * the framebuffer descriptor and force a refresh so the SDL window
         * tracks the new contents.
         */
        if (s->con && (s->regs[LCDIF_CTRL] & CTRL_MASTER)) {
            mxs_lcdif_latch_frame(s);
        }
        break;
    case LCDIF_CUR_BUF:
        s->regs[idx] = val;
        if (s->con && (s->regs[LCDIF_CTRL] & CTRL_MASTER)) {
            mxs_lcdif_latch_frame(s);
        }
        break;
    default:
        s->regs[idx] = val;
        break;
    }
}

MXS_TRACE_WRAP(mxs_lcdif, "lcdif")

static const MemoryRegionOps mxs_lcdif_ops = {
    .read = mxs_lcdif_read_tr,
    .write = mxs_lcdif_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_lcdif_reset(DeviceState *dev)
{
    MXSLcdifState *s = MXS_LCDIF(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[LCDIF_CTRL] = CTRL_SFTRST | CTRL_CLKGATE;
    s->fb_addr = 0;
    s->fb_width = 0;
    s->fb_height = 0;
    s->fb_bpp = 16;
    s->fb_argb1555 = false;
    s->invalidate = 1;
}

static void mxs_lcdif_realize(DeviceState *dev, Error **errp)
{
    mxs_lcdif_trace = mxs_trace_enabled("lcdif");

    MXSLcdifState *s = MXS_LCDIF(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_lcdif_ops, s,
                          "mxs-lcdif", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->irq_error);

    s->con = graphic_console_init(dev, 0, &mxs_lcdif_gfx_ops, s);
    /* Viewed panel: 480x854 scan + 270° = 854x480 landscape. */
    mxs_lcdif_out_size(s, s->default_width, s->default_height,
                       &s->cols, &s->rows);
    qemu_console_resize(s->con, s->cols, s->rows);

    s->vsync = timer_new_ns(QEMU_CLOCK_VIRTUAL, mxs_lcdif_vsync_tick, s);
    timer_mod(s->vsync, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / (s->refresh_hz ? s->refresh_hz : 60));
}

static const Property mxs_lcdif_properties[] = {
    DEFINE_PROP_UINT32("width", MXSLcdifState, default_width, 854),
    DEFINE_PROP_UINT32("height", MXSLcdifState, default_height, 480),
    DEFINE_PROP_UINT32("rotate", MXSLcdifState, rotate, 0),
    DEFINE_PROP_UINT32("refresh-hz", MXSLcdifState, refresh_hz, 60),
};

static const VMStateDescription vmstate_mxs_lcdif = {
    .name = "mxs-lcdif",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSLcdifState, LCDIF_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_lcdif_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mxs_lcdif_realize;
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
