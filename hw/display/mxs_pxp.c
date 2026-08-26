/*
 * Freescale i.MX23/i.MX28 (MXS) PXP - Pixel Pipeline
 *
 * The PXP is a small 2D blitter sitting between system memory and the
 * LCDIF.  It can convert colour spaces, scale, crop, rotate/flip and
 * composite up to eight overlays on top of the "S0" source surface.
 *
 * The WinCE display driver of the SHARP Brain drives it exclusively
 * through the "next command" mechanism: it writes a pointer to a packed
 * copy of the register file into HW_PXP_NEXT, the block loads those
 * values into its registers, performs the operation and raises its
 * interrupt.
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
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"

/* register indices (offset >> 4) */
#define PXP_CTRL            0x00
#define PXP_STAT            0x01
#define PXP_OUTBUF          0x02
#define PXP_OUTBUF2         0x03
#define PXP_OUTSIZE         0x04
#define PXP_S0BUF           0x05
#define PXP_S0UBUF          0x06
#define PXP_S0VBUF          0x07
#define PXP_S0PARAM         0x08
#define PXP_S0BACKGROUND    0x09
#define PXP_S0CROP          0x0a
#define PXP_S0SCALE         0x0b
#define PXP_S0OFFSET        0x0c
#define PXP_CSCCOEF0        0x0d
#define PXP_CSCCOEF1        0x0e
#define PXP_CSCCOEF2        0x0f
#define PXP_NEXT            0x10
#define PXP_S0COLORKEYLOW   0x18
#define PXP_S0COLORKEYHIGH  0x19
#define PXP_OLCOLORKEYLOW   0x1a
#define PXP_OLCOLORKEYHIGH  0x1b
#define PXP_DEBUGCTRL       0x1d
#define PXP_DEBUG           0x1e
#define PXP_VERSION         0x1f
#define PXP_OL(n)           (0x20 + (n) * 4)
#define PXP_OLSIZE(n)       (0x21 + (n) * 4)
#define PXP_OLPARAM(n)      (0x22 + (n) * 4)

#define PXP_NUM_OL          8
#define PXP_NREGS           0x200       /* 0x2000 / 0x10 */

/* HW_PXP_CTRL */
#define CTRL_SFTRST                 (1u << 31)
#define CTRL_CLKGATE                (1u << 30)
#define CTRL_EN_REPEAT              (1u << 28)
#define CTRL_BLOCK_SIZE             (1u << 23)
#define CTRL_ALPHA_OUTPUT           (1u << 22)
#define CTRL_IN_PLACE               (1u << 21)
#define CTRL_DELTA                  (1u << 20)
#define CTRL_CROP                   (1u << 19)
#define CTRL_SCALE                  (1u << 18)
#define CTRL_UPSAMPLE               (1u << 17)
#define CTRL_SUBSAMPLE              (1u << 16)
#define CTRL_S0_FORMAT(v)           (((v) >> 12) & 0xf)
#define CTRL_VFLIP                  (1u << 11)
#define CTRL_HFLIP                  (1u << 10)
#define CTRL_ROTATE(v)              (((v) >> 8) & 3)
#define CTRL_OUTBUF_FORMAT(v)       (((v) >> 4) & 0xf)
#define CTRL_ENABLE_LCD_HANDSHAKE   (1u << 3)
#define CTRL_NEXT_IRQ_ENABLE        (1u << 2)
#define CTRL_IRQ_ENABLE             (1u << 1)
#define CTRL_ENABLE                 (1u << 0)

/* HW_PXP_STAT */
#define STAT_NEXT_IRQ               (1u << 3)
#define STAT_AXI_READ_ERROR         (1u << 2)
#define STAT_AXI_WRITE_ERROR        (1u << 1)
#define STAT_IRQ                    (1u << 0)

/* HW_PXP_NEXT */
#define NEXT_POINTER(v)             ((v) & 0xfffffffcu)
#define NEXT_ENABLED                (1u << 0)

/* pixel formats (shared by S0, OL and OUTBUF, values overlap) */
#define FMT_ARGB8888    0x0
#define FMT_RGB888      0x1
#define FMT_RGB888P     0x2
#define FMT_ARGB1555    0x3
#define FMT_RGB565      0x4
#define FMT_RGB555      0x5
#define FMT_YUV444      0x7
#define FMT_UYVY1P422   0xa
#define FMT_VYUY1P422   0xb

/* OLnPARAM */
#define OLPARAM_ROP(v)              (((v) >> 16) & 0xf)
#define OLPARAM_ALPHA(v)            (((v) >> 8) & 0xff)
#define OLPARAM_FORMAT(v)           (((v) >> 4) & 0xf)
#define OLPARAM_ENABLE_COLORKEY     (1u << 3)
#define OLPARAM_ALPHA_CNTL(v)       (((v) >> 1) & 3)
#define OLPARAM_ENABLE              (1u << 0)

#define ALPHA_CNTL_EMBEDDED 0
#define ALPHA_CNTL_OVERRIDE 1
#define ALPHA_CNTL_MULTIPLY 2
#define ALPHA_CNTL_ROPS     3

/*
 * Layout of the "next command" structure in memory.  It is a packed copy
 * of the register file, with the read-only HW_PXP_STAT and the CSC
 * coefficients (which the driver programs directly) left out.
 */
enum {
    CMD_CTRL = 0,
    CMD_OUTBUF,
    CMD_OUTBUF2,
    CMD_OUTSIZE,
    CMD_S0BUF,
    CMD_S0UBUF,
    CMD_S0VBUF,
    CMD_S0PARAM,
    CMD_S0BACKGROUND,
    CMD_S0CROP,
    CMD_S0SCALE,
    CMD_S0OFFSET,
    CMD_S0COLORKEYLOW,
    CMD_S0COLORKEYHIGH,
    CMD_OLCOLORKEYLOW,
    CMD_OLCOLORKEYHIGH,
    CMD_OL0,                                    /* 8 x (buf, size, param) */
    CMD_NEXT = CMD_OL0 + PXP_NUM_OL * 3,
    CMD_WORDS,
};

/* how long (in virtual ns) a blit is pretended to take */
#define PXP_LATENCY_NS  20000

typedef struct MXSPxpState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;

    uint32_t regs[PXP_NREGS];
    bool     running_next;      /* current op came from a next command */
    uint32_t chain;             /* remaining command chain budget */
} MXSPxpState;

OBJECT_DECLARE_SIMPLE_TYPE(MXSPxpState, MXS_PXP)

/* ------------------------------------------------------------------ */
/* pixel helpers - everything is funnelled through ARGB8888            */
/* ------------------------------------------------------------------ */

static int pxp_pixel_bytes(unsigned fmt)
{
    switch (fmt) {
    case FMT_ARGB8888:
    case FMT_RGB888:
    case FMT_YUV444:
        return 4;
    case FMT_RGB888P:
        return 3;
    case FMT_ARGB1555:
    case FMT_RGB565:
    case FMT_RGB555:
    case FMT_UYVY1P422:
    case FMT_VYUY1P422:
        return 2;
    default:
        return 0;
    }
}

static inline uint32_t expand5(uint32_t v)
{
    return (v << 3) | (v >> 2);
}

static inline uint32_t expand6(uint32_t v)
{
    return (v << 2) | (v >> 4);
}

static uint32_t pxp_unpack(unsigned fmt, uint32_t raw)
{
    uint32_t r, g, b, a;

    switch (fmt) {
    case FMT_ARGB8888:
        return raw;
    case FMT_RGB888:
    case FMT_RGB888P:
        return 0xff000000u | (raw & 0xffffffu);
    case FMT_ARGB1555:
        a = (raw & 0x8000) ? 0xff : 0x00;
        r = expand5((raw >> 10) & 0x1f);
        g = expand5((raw >> 5) & 0x1f);
        b = expand5(raw & 0x1f);
        return (a << 24) | (r << 16) | (g << 8) | b;
    case FMT_RGB555:
        r = expand5((raw >> 10) & 0x1f);
        g = expand5((raw >> 5) & 0x1f);
        b = expand5(raw & 0x1f);
        return 0xff000000u | (r << 16) | (g << 8) | b;
    case FMT_RGB565:
    default:
        r = expand5((raw >> 11) & 0x1f);
        g = expand6((raw >> 5) & 0x3f);
        b = expand5(raw & 0x1f);
        return 0xff000000u | (r << 16) | (g << 8) | b;
    }
}

static uint32_t pxp_pack(unsigned fmt, uint32_t argb)
{
    uint32_t r = (argb >> 16) & 0xff;
    uint32_t g = (argb >> 8) & 0xff;
    uint32_t b = argb & 0xff;
    uint32_t a = (argb >> 24) & 0xff;

    switch (fmt) {
    case FMT_ARGB8888:
        return argb;
    case FMT_RGB888:
    case FMT_RGB888P:
        return argb & 0xffffffu;
    case FMT_ARGB1555:
        return ((a & 0x80) ? 0x8000 : 0) |
               ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3);
    case FMT_RGB555:
        return ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3);
    case FMT_RGB565:
    default:
        return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    }
}

static uint32_t pxp_load(hwaddr base, unsigned stride, int x, int y,
                         unsigned fmt)
{
    int bytes = pxp_pixel_bytes(fmt);
    hwaddr a = base + (hwaddr)y * stride + (hwaddr)x * bytes;
    uint8_t buf[4] = { 0 };
    uint32_t raw = 0;

    if (!bytes) {
        return 0xff000000u;
    }
    address_space_read(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                       buf, bytes);
    switch (bytes) {
    case 2:
        raw = lduw_le_p(buf);
        break;
    case 3:
        raw = buf[0] | (buf[1] << 8) | (buf[2] << 16);
        break;
    default:
        raw = ldl_le_p(buf);
        break;
    }
    return pxp_unpack(fmt, raw);
}

static void pxp_store(hwaddr base, unsigned stride, int x, int y,
                      unsigned fmt, uint32_t argb)
{
    int bytes = pxp_pixel_bytes(fmt);
    hwaddr a = base + (hwaddr)y * stride + (hwaddr)x * bytes;
    uint32_t raw;
    uint8_t buf[4];

    if (!bytes) {
        return;
    }
    raw = pxp_pack(fmt, argb);
    switch (bytes) {
    case 2:
        stw_le_p(buf, raw);
        break;
    case 3:
        buf[0] = raw & 0xff;
        buf[1] = (raw >> 8) & 0xff;
        buf[2] = (raw >> 16) & 0xff;
        break;
    default:
        stl_le_p(buf, raw);
        break;
    }
    address_space_write(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                        buf, bytes);
}

/* ------------------------------------------------------------------ */
/* geometry                                                            */
/* ------------------------------------------------------------------ */

typedef struct PxpGeom {
    unsigned rotate;        /* 0, 90, 180, 270 */
    bool hflip;
    bool vflip;
    int canvas_w;           /* composition surface, before rotation */
    int canvas_h;
    int phys_w;             /* output buffer, after rotation */
    int phys_h;
    unsigned out_fmt;
    unsigned out_stride;
    hwaddr out_base;
} PxpGeom;

/*
 * Map a pixel of the composition surface onto the rotated/flipped
 * output buffer.  Returns false when it falls outside.
 */
static bool pxp_map_out(const PxpGeom *g, int cx, int cy, int *px, int *py)
{
    int x = g->hflip ? g->canvas_w - 1 - cx : cx;
    int y = g->vflip ? g->canvas_h - 1 - cy : cy;

    switch (g->rotate) {
    case 90:
        *px = g->canvas_h - 1 - y;
        *py = x;
        break;
    case 180:
        *px = g->canvas_w - 1 - x;
        *py = g->canvas_h - 1 - y;
        break;
    case 270:
        *px = y;
        *py = g->canvas_w - 1 - x;
        break;
    default:
        *px = x;
        *py = y;
        break;
    }
    return *px >= 0 && *py >= 0 && *px < g->phys_w && *py < g->phys_h;
}

static uint32_t pxp_blend(uint32_t dst, uint32_t src, unsigned alpha)
{
    unsigned i;
    uint32_t out = 0;

    if (alpha >= 0xff) {
        return src | 0xff000000u;
    }
    if (!alpha) {
        return dst;
    }
    for (i = 0; i < 3; i++) {
        unsigned s = (src >> (i * 8)) & 0xff;
        unsigned d = (dst >> (i * 8)) & 0xff;
        unsigned v = (s * alpha + d * (255 - alpha) + 127) / 255;
        out |= (v & 0xff) << (i * 8);
    }
    return out | 0xff000000u;
}

/* ------------------------------------------------------------------ */
/* the blitter                                                         */
/* ------------------------------------------------------------------ */

static void pxp_do_overlay(MXSPxpState *s, const PxpGeom *g, int n)
{
    uint32_t param = s->regs[PXP_OLPARAM(n)];
    uint32_t size = s->regs[PXP_OLSIZE(n)];
    hwaddr base = s->regs[PXP_OL(n)];
    unsigned fmt = OLPARAM_FORMAT(param);
    unsigned actl = OLPARAM_ALPHA_CNTL(param);
    unsigned galpha = OLPARAM_ALPHA(param);
    bool ckey = param & OLPARAM_ENABLE_COLORKEY;
    uint32_t cklow = s->regs[PXP_OLCOLORKEYLOW] & 0xffffff;
    uint32_t ckhigh = s->regs[PXP_OLCOLORKEYHIGH] & 0xffffff;
    int x0 = ((size >> 24) & 0xff) * 8;
    int y0 = ((size >> 16) & 0xff) * 8;
    int w = ((size >> 8) & 0xff) * 8;
    int h = (size & 0xff) * 8;
    unsigned stride;
    int x, y;

    if (!(param & OLPARAM_ENABLE) || !base || w <= 0 || h <= 0) {
        return;
    }
    stride = w * pxp_pixel_bytes(fmt);

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            int cx = x0 + x, cy = y0 + y, px, py;
            uint32_t src, dst;
            unsigned alpha;

            if (cx >= g->canvas_w || cy >= g->canvas_h) {
                continue;
            }
            if (!pxp_map_out(g, cx, cy, &px, &py)) {
                continue;
            }
            src = pxp_load(base, stride, x, y, fmt);
            if (ckey && cklow <= ckhigh) {
                uint32_t rgb = src & 0xffffff;
                if (rgb >= cklow && rgb <= ckhigh) {
                    continue;   /* transparent */
                }
            }
            switch (actl) {
            case ALPHA_CNTL_OVERRIDE:
                alpha = galpha;
                break;
            case ALPHA_CNTL_MULTIPLY:
                alpha = (((src >> 24) & 0xff) * galpha) / 255;
                break;
            case ALPHA_CNTL_ROPS:
            case ALPHA_CNTL_EMBEDDED:
            default:
                alpha = (src >> 24) & 0xff;
                break;
            }
            dst = pxp_load(g->out_base, g->out_stride, px, py, g->out_fmt);
            pxp_store(g->out_base, g->out_stride, px, py, g->out_fmt,
                      pxp_blend(dst, src, alpha));
        }
    }
}

static void pxp_blit(MXSPxpState *s)
{
    uint32_t ctrl = s->regs[PXP_CTRL];
    uint32_t outsize = s->regs[PXP_OUTSIZE];
    uint32_t s0param = s->regs[PXP_S0PARAM];
    uint32_t s0crop = s->regs[PXP_S0CROP];
    uint32_t s0scale = s->regs[PXP_S0SCALE];
    PxpGeom g;
    hwaddr s0base = s->regs[PXP_S0BUF];
    unsigned s0fmt = CTRL_S0_FORMAT(ctrl);
    unsigned s0stride;
    int s0w, s0h, srcx, srcy, dstx, dsty, dstw, dsth;
    uint32_t xscale, yscale;
    int x, y;
    int n;

    memset(&g, 0, sizeof(g));
    g.rotate = 90 * CTRL_ROTATE(ctrl);
    g.hflip = (ctrl & CTRL_HFLIP) != 0;
    g.vflip = (ctrl & CTRL_VFLIP) != 0;
    g.out_fmt = CTRL_OUTBUF_FORMAT(ctrl);
    g.out_base = s->regs[PXP_OUTBUF];
    g.phys_w = (outsize >> 12) & 0xfff;
    g.phys_h = outsize & 0xfff;
    if (g.phys_w <= 0 || g.phys_h <= 0 || !g.out_base) {
        return;
    }
    g.out_stride = g.phys_w * pxp_pixel_bytes(g.out_fmt);
    if (g.rotate == 90 || g.rotate == 270) {
        g.canvas_w = g.phys_h;
        g.canvas_h = g.phys_w;
    } else {
        g.canvas_w = g.phys_w;
        g.canvas_h = g.phys_h;
    }

    /* source surface: dimensions are given in blocks of eight pixels */
    s0w = ((s0param >> 8) & 0xff) * 8;
    s0h = (s0param & 0xff) * 8;
    dstx = ((s0param >> 24) & 0xff) * 8;
    dsty = ((s0param >> 16) & 0xff) * 8;

    if (ctrl & CTRL_CROP) {
        srcx = ((s0crop >> 24) & 0xff) * 8;
        srcy = ((s0crop >> 16) & 0xff) * 8;
        dstw = ((s0crop >> 8) & 0xff) * 8;
        dsth = (s0crop & 0xff) * 8;
    } else {
        srcx = srcy = 0;
        dstw = s0w;
        dsth = s0h;
    }
    if (!s0w) {
        s0w = dstw;
    }
    if (!s0h) {
        s0h = dsth;
    }

    xscale = s0scale & 0x7fff;
    yscale = (s0scale >> 16) & 0x7fff;
    if (!(ctrl & CTRL_SCALE) || !xscale) {
        xscale = 0x1000;
    }
    if (!(ctrl & CTRL_SCALE) || !yscale) {
        yscale = 0x1000;
    }

    if (s0base && dstw > 0 && dsth > 0) {
        s0stride = s0w * pxp_pixel_bytes(s0fmt);

        for (y = 0; y < dsth; y++) {
            int sy = srcy + (int)(((uint64_t)y * yscale) >> 12);

            if (sy >= s0h) {
                sy = s0h - 1;
            }
            for (x = 0; x < dstw; x++) {
                int sx = srcx + (int)(((uint64_t)x * xscale) >> 12);
                int cx = dstx + x, cy = dsty + y, px, py;
                uint32_t pix;

                if (sx >= s0w) {
                    sx = s0w - 1;
                }
                if (cx >= g.canvas_w || cy >= g.canvas_h) {
                    continue;
                }
                if (!pxp_map_out(&g, cx, cy, &px, &py)) {
                    continue;
                }
                pix = pxp_load(s0base, s0stride, sx, sy, s0fmt);
                if (ctrl & CTRL_ALPHA_OUTPUT) {
                    pix = (pix & 0xffffff) |
                          ((outsize & 0xff000000u));
                }
                pxp_store(g.out_base, g.out_stride, px, py, g.out_fmt, pix);
            }
        }
    }

    for (n = 0; n < PXP_NUM_OL; n++) {
        pxp_do_overlay(s, &g, n);
    }
}

static void mxs_pxp_update_irq(MXSPxpState *s)
{
    uint32_t ctrl = s->regs[PXP_CTRL];
    uint32_t stat = s->regs[PXP_STAT];
    bool level = false;

    if ((stat & STAT_IRQ) && (ctrl & CTRL_IRQ_ENABLE)) {
        level = true;
    }
    if ((stat & STAT_NEXT_IRQ) && (ctrl & CTRL_NEXT_IRQ_ENABLE)) {
        level = true;
    }
    qemu_set_irq(s->irq, level);
}

static void mxs_pxp_load_next(MXSPxpState *s)
{
    uint32_t ptr = NEXT_POINTER(s->regs[PXP_NEXT]);
    uint32_t cmd[CMD_WORDS];
    int i;

    if (!ptr) {
        return;
    }
    address_space_read(&address_space_memory, ptr, MEMTXATTRS_UNSPECIFIED,
                       cmd, sizeof(cmd));
    for (i = 0; i < CMD_WORDS; i++) {
        cmd[i] = le32_to_cpu(cmd[i]);
    }

    s->regs[PXP_CTRL] = cmd[CMD_CTRL];
    s->regs[PXP_OUTBUF] = cmd[CMD_OUTBUF];
    s->regs[PXP_OUTBUF2] = cmd[CMD_OUTBUF2];
    s->regs[PXP_OUTSIZE] = cmd[CMD_OUTSIZE];
    s->regs[PXP_S0BUF] = cmd[CMD_S0BUF];
    s->regs[PXP_S0UBUF] = cmd[CMD_S0UBUF];
    s->regs[PXP_S0VBUF] = cmd[CMD_S0VBUF];
    s->regs[PXP_S0PARAM] = cmd[CMD_S0PARAM];
    s->regs[PXP_S0BACKGROUND] = cmd[CMD_S0BACKGROUND];
    s->regs[PXP_S0CROP] = cmd[CMD_S0CROP];
    s->regs[PXP_S0SCALE] = cmd[CMD_S0SCALE];
    s->regs[PXP_S0OFFSET] = cmd[CMD_S0OFFSET];
    s->regs[PXP_S0COLORKEYLOW] = cmd[CMD_S0COLORKEYLOW];
    s->regs[PXP_S0COLORKEYHIGH] = cmd[CMD_S0COLORKEYHIGH];
    s->regs[PXP_OLCOLORKEYLOW] = cmd[CMD_OLCOLORKEYLOW];
    s->regs[PXP_OLCOLORKEYHIGH] = cmd[CMD_OLCOLORKEYHIGH];
    for (i = 0; i < PXP_NUM_OL; i++) {
        s->regs[PXP_OL(i)] = cmd[CMD_OL0 + i * 3];
        s->regs[PXP_OLSIZE(i)] = cmd[CMD_OL0 + i * 3 + 1];
        s->regs[PXP_OLPARAM(i)] = cmd[CMD_OL0 + i * 3 + 2];
    }
    s->regs[PXP_NEXT] = cmd[CMD_NEXT];
}

static void mxs_pxp_start(MXSPxpState *s, bool from_next)
{
    s->running_next = from_next;
    timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              PXP_LATENCY_NS);
}

static void mxs_pxp_complete(void *opaque)
{
    MXSPxpState *s = opaque;
    bool chain;

    pxp_blit(s);

    s->regs[PXP_CTRL] &= ~CTRL_ENABLE;
    s->regs[PXP_STAT] |= STAT_IRQ;
    if (s->running_next) {
        s->regs[PXP_STAT] |= STAT_NEXT_IRQ;
    }
    s->regs[PXP_NEXT] &= ~NEXT_ENABLED;

    /* a command may point at a follow up command */
    chain = NEXT_POINTER(s->regs[PXP_NEXT]) != 0 && s->chain > 0;
    mxs_pxp_update_irq(s);

    if (chain) {
        s->chain--;
        mxs_pxp_load_next(s);
        if (s->regs[PXP_CTRL] & CTRL_ENABLE) {
            mxs_pxp_start(s, true);
            return;
        }
    }
    s->chain = 0;
}

/* ------------------------------------------------------------------ */

static uint64_t mxs_pxp_read(void *opaque, hwaddr offset, unsigned size)
{
    MXSPxpState *s = MXS_PXP(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= PXP_NREGS) {
        return 0;
    }
    switch (idx) {
    case PXP_VERSION:
        val = 0x02000000;
        break;
    default:
        val = s->regs[idx];
        break;
    }
    return mxs_bank_extract(offset, size, val);
}

static void mxs_pxp_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    MXSPxpState *s = MXS_PXP(opaque);
    unsigned idx = MXS_BANK_INDEX(offset);
    uint32_t val;

    if (idx >= PXP_NREGS || idx == PXP_VERSION) {
        return;
    }
    val = mxs_bank_apply(s->regs[idx], offset, value, size);

    switch (idx) {
    case PXP_CTRL: {
        uint32_t old = s->regs[idx];

        val = mxs_bank_sftrst(old, val);
        s->regs[idx] = val;
        if ((val & CTRL_ENABLE) && !(old & CTRL_ENABLE) &&
            !(val & (CTRL_SFTRST | CTRL_CLKGATE))) {
            s->chain = 64;
            mxs_pxp_start(s, false);
        }
        mxs_pxp_update_irq(s);
        break;
    }
    case PXP_STAT:
        s->regs[idx] = val;
        mxs_pxp_update_irq(s);
        break;
    case PXP_NEXT:
        s->regs[idx] = val;
        if (NEXT_POINTER(val) &&
            !(s->regs[PXP_CTRL] & (CTRL_SFTRST | CTRL_CLKGATE))) {
            /*
             * Writing a pointer arms the next command: the block fetches
             * the register set from memory and runs it.
             */
            s->regs[idx] |= NEXT_ENABLED;
            s->chain = 64;
            mxs_pxp_load_next(s);
            if (s->regs[PXP_CTRL] & CTRL_ENABLE) {
                mxs_pxp_start(s, true);
            }
        }
        break;
    default:
        s->regs[idx] = val;
        break;
    }
}

MXS_TRACE_WRAP(mxs_pxp, "pxp")

static const MemoryRegionOps mxs_pxp_ops = {
    .read = mxs_pxp_read_tr,
    .write = mxs_pxp_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_pxp_reset(DeviceState *dev)
{
    MXSPxpState *s = MXS_PXP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[PXP_CTRL] = CTRL_SFTRST | CTRL_CLKGATE;
    s->chain = 0;
    s->running_next = false;
    qemu_set_irq(s->irq, 0);
}

static void mxs_pxp_realize(DeviceState *dev, Error **errp)
{
    MXSPxpState *s = MXS_PXP(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    mxs_pxp_trace = mxs_trace_enabled("pxp");

    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_pxp_ops, s,
                          "mxs-pxp", 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, mxs_pxp_complete, s);
}

static const VMStateDescription vmstate_mxs_pxp = {
    .name = "mxs-pxp",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSPxpState, PXP_NREGS),
        VMSTATE_BOOL(running_next, MXSPxpState),
        VMSTATE_UINT32(chain, MXSPxpState),
        VMSTATE_END_OF_LIST()
    }
};

static void mxs_pxp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = mxs_pxp_realize;
    device_class_set_legacy_reset(dc, mxs_pxp_reset);
    dc->vmsd = &vmstate_mxs_pxp;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo mxs_pxp_types[] = {
    {
        .name           = TYPE_MXS_PXP,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(MXSPxpState),
        .class_init     = mxs_pxp_class_init,
    },
};

DEFINE_TYPES(mxs_pxp_types)
