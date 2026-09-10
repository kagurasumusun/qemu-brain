/*
 * Freescale i.MX28 (MXS) I2C controller.
 *
 * Register layout from i.MX28 RM ch.27 and Linux
 * drivers/i2c/busses/i2c-mxs.c.  The Brain WinCE BSP drives the block in
 * PIO mode: it programs CTRL0 with count/flags, sets RUN, and polls
 * CTRL0/DEBUG0, then reads/writes DATA for each byte.  We perform the
 * whole transfer synchronously against the QEMU I2C bus the moment RUN
 * rises and report DATA_ENGINE_CMPLT_IRQ.
 *
 * An ack-all slave is registered at every address so BSP probes do not
 * NAK; a real slave may occupy a given address instead.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/arm/mxs.h"
#include "hw/misc/mxs_bank.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qom/object.h"

#define MXS_MAX_REGS    512

/* CTRL0 */
#define CTRL0_SFTRST            (1u << 31)
#define CTRL0_CLKGATE           (1u << 30)
#define CTRL0_RUN               (1u << 29)
#define CTRL0_PIO_MODE          (1u << 24)
#define CTRL0_RETAIN_CLOCK      (1u << 22)
#define CTRL0_POST_SEND_STOP    (1u << 21)
#define CTRL0_PRE_SEND_START    (1u << 20)
#define CTRL0_MASTER_MODE       (1u << 19)
#define CTRL0_DIRECTION         (1u << 18) /* 1=read 0=write */
#define CTRL0_XFER_COUNT_SHIFT  0
#define CTRL0_XFER_COUNT_MASK   0x0000ffff

/* CTRL1 */
#define CTRL1_CLR_GOT_A_NAK     (1u << 28)
#define CTRL1_BUS_FREE_IRQ      (1u << 7)
#define CTRL1_DATA_ENGINE_CMPLT (1u << 6)
#define CTRL1_NO_SLAVE_ACK      (1u << 5)
#define CTRL1_EARLY_TERM        (1u << 3)
#define CTRL1_MASTER_LOSS       (1u << 2)
#define CTRL1_SLAVE_STOP_IRQ    (1u << 1)
#define CTRL1_SLAVE_IRQ         (1u << 0)

/* Interrupt status flags: bits[7:0] (plus CLR_GOT_A_NAK in bit 28) are
 * set by the data engine and cleared by the guest writing 1 to them. */
#define CTRL1_W1C_MASK          (0x100000ffu)

#define STAT_BUS_BUSY           (1u << 11)
#define STAT_CLK_GEN_BUSY       (1u << 10)
#define STAT_DATA_ENGINE_BUSY   (1u << 9)
#define STAT_GOT_A_NAK          (1u << 28)

#define DEBUG0_DMAREQ           (1u << 31)

typedef struct MXSI2CState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[MXS_MAX_REGS];
    char *name;
    uint64_t size;
    uint32_t ctrl_idx;
    bool trace;
    qemu_irq irq;
    I2CBus *bus;

    /* State of an in-flight PIO write: bytes still to be pushed through
     * DATA after the address byte, and whether the address was already
     * consumed. */
    uint32_t xfer_left;
    bool xfer_started;

    /* Optional board codec attached at a single address (SGTL5000). */
    char *codec_type;
    uint8_t codec_addr;
    char *codec_audiodev;
    I2CSlave *codec;
} MXSI2CState;

#define TYPE_MXS_I2C_REAL "mxs-i2c-real"
OBJECT_DECLARE_SIMPLE_TYPE(MXSI2CState, MXS_I2C_REAL)

static bool brain_i2c_debug(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("BRAIN_I2C_DEBUG");

        on = e && *e && *e != '0';
    }
    return on;
}

#define TYPE_MXS_I2C_ACK "mxs-i2c-ack-slave"
typedef struct MXSAckSlave { I2CSlave parent_obj; } MXSAckSlave;
DECLARE_INSTANCE_CHECKER(MXSAckSlave, MXS_ACK_SLAVE, TYPE_MXS_I2C_ACK)

static int mxs_ack_send(I2CSlave *s, uint8_t d) { return 0; }
static uint8_t mxs_ack_recv(I2CSlave *s) { return 0; }
static int mxs_ack_event(I2CSlave *s, enum i2c_event e) { return 0; }
static void mxs_ack_class_init(ObjectClass *oc, const void *d)
{
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);
    sc->send = mxs_ack_send; sc->recv = mxs_ack_recv; sc->event = mxs_ack_event;
}

static void mxs_i2c_update_irq(MXSI2CState *s)
{
    /* CTRL1 bit 6 (DATA_ENGINE_CMPLT) doubles as the model's IRQ status:
     * set by the engine on completion, cleared by the guest's W1C write of
     * 0x40/0x78.  No separate enable bits are modelled. */
    uint32_t c = s->regs[0x40 >> 4];
    qemu_set_irq(s->irq, (c & CTRL1_DATA_ENGINE_CMPLT) != 0);
}

static void mxs_i2c_finish(MXSI2CState *s, uint32_t status)
{
    s->regs[0] &= ~CTRL0_RUN;
    s->regs[0x40 >> 4] |= status;
    s->xfer_left = 0;
    s->xfer_started = false;
    mxs_i2c_update_irq(s);
}

static uint32_t mxs_i2c_reg_read(MXSI2CState *s, unsigned idx)
{
    switch (idx) {
    case 0x00:
        return s->regs[idx];
    case 0x40 >> 4:       /* CTRL1 */
        return s->regs[idx];
    case 0x50 >> 4:       /* STAT: engine idle after RUN consumed */
        return s->regs[idx] & ~(STAT_BUS_BUSY | STAT_CLK_GEN_BUSY |
                                STAT_DATA_ENGINE_BUSY);
    case 0xa0 >> 4:       /* DATA receive register */
        return s->regs[idx];
    case 0xb0 >> 4:       /* DEBUG0: report DMAREQ high to release BSP poll */
        return DEBUG0_DMAREQ;
    case 0xd0 >> 4:       /* VERSION */
        return 0x01040000;
    default:
        return s->regs[idx];
    }
}

static void mxs_i2c_kick(MXSI2CState *s)
{
    uint32_t ctrl0 = s->regs[0];
    unsigned count = ctrl0 & CTRL0_XFER_COUNT_MASK;
    bool is_read = ctrl0 & CTRL0_DIRECTION;
    bool start   = ctrl0 & CTRL0_PRE_SEND_START;
    bool stop    = ctrl0 & CTRL0_POST_SEND_STOP;

    if (!count) {
        mxs_i2c_finish(s, CTRL1_DATA_ENGINE_CMPLT);
        return;
    }

    if (start) {
        /*
         * PIO write: the first DATA write carries the slave address <<1.
         * We cannot start the bus before we see that byte, so defer.
         * PIO read: BSP has queued address through a prior write; the
         * address register is the most recent DATA byte latched below.
         */
        if (is_read) {
            uint8_t addr = s->regs[0xa0 >> 4] >> 1;
            if (brain_i2c_debug()) {
                fprintf(stderr, "[i2c-debug] %s START_RECV addr=0x%02x "
                        "count=%u pc=0x%08x\n", s->name, addr, count,
                        (unsigned)mxs_trace_guest_pc());
            }
            if (i2c_start_recv(s->bus, addr) < 0) {
                mxs_i2c_finish(s, CTRL1_NO_SLAVE_ACK);
                return;
            }
            for (unsigned i = 0; i < count; i++) {
                /* The master NACKs the final byte of a read transaction;
                 * QEMU applies the pending ACK/NACK on the following
                 * i2c_recv(), so arm it *before* the last receive. */
                if (i == count - 1) {
                    i2c_nack(s->bus);
                }
                s->regs[0xa0 >> 4] = i2c_recv(s->bus);
                if (brain_i2c_debug()) {
                    fprintf(stderr, "[i2c-debug] %s RECV[%u] 0x%02x "
                            "pc=0x%08x\n", s->name, i,
                            (uint8_t)s->regs[0xa0 >> 4],
                            (unsigned)mxs_trace_guest_pc());
                }
            }
            if (stop) i2c_end_transfer(s->bus);
            mxs_i2c_finish(s, CTRL1_DATA_ENGINE_CMPLT);
            return;
        }
        /*
         * The SHARP BSP drives the controller without setting
         * PIO_MODE: it writes CTRL0 with RUN|START|MASTER and then
         * pushes bytes into HW_I2C_DATA (address first, then payload).
         * Treat that as a PIO write regardless of the PIO_MODE bit --
         * finishing here would clear RUN and swallow every DATA write
         * (observed on repaired4: the EDSH6 I2C probe writes DATA
         * 0x34 with run=0 and no transfer ever happens).
         */
        return;
    }

    /* No START: continue an existing transfer.  Only reads hit this in
     * the BSP flow. */
    if (is_read) {
        for (unsigned i = 0; i < count; i++) {
            if (i == count - 1) {
                i2c_nack(s->bus);
            }
            s->regs[0xa0 >> 4] = i2c_recv(s->bus);
        }
        if (stop) i2c_end_transfer(s->bus);
    }
    mxs_i2c_finish(s, CTRL1_DATA_ENGINE_CMPLT);
}

static uint64_t mxs_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    MXSI2CState *s = MXS_I2C_REAL(opaque);
    unsigned idx = MXS_BANK_INDEX(off);
    if (idx >= MXS_MAX_REGS) return 0;
    uint32_t v = mxs_i2c_reg_read(s, idx);
    if (unlikely(s->trace || mxs_trace_live))
        mxs_trace_access(s->name, false, off, v);
    return mxs_bank_extract(off, size, v);
}

static void mxs_i2c_write(void *opaque, hwaddr off, uint64_t value, unsigned size)
{
    MXSI2CState *s = MXS_I2C_REAL(opaque);
    unsigned idx = MXS_BANK_INDEX(off);
    uint32_t old, val;
    if (idx >= MXS_MAX_REGS) return;

    old = s->regs[idx];
    val = mxs_bank_apply(old, off, value, size);
    if (idx == s->ctrl_idx) val = mxs_bank_sftrst(old, val);
    s->regs[idx] = val;
    if (unlikely(s->trace || mxs_trace_live))
        mxs_trace_access(s->name, true, off, (uint32_t)value);

    switch (idx) {
    case 0x00: {   /* CTRL0 */
        if (brain_i2c_debug()) {
            fprintf(stderr, "[i2c-debug] %s CTRL0 W 0x%08x (was 0x%08x) "
                    "pc=0x%08x\n", s->name, val, old,
                    (unsigned)mxs_trace_guest_pc());
        }
        bool rising_run = (val & CTRL0_RUN) && !(old & CTRL0_RUN);
        if (rising_run && (val & CTRL0_MASTER_MODE)) {
            mxs_i2c_kick(s);
        }
        break;
    }
    case 0xa0 >> 4: { /* DATA (PIO byte) */
        bool run   = s->regs[0] & CTRL0_RUN;
        bool start = s->regs[0] & CTRL0_PRE_SEND_START;
        bool dir   = s->regs[0] & CTRL0_DIRECTION;

        if (brain_i2c_debug()) {
            fprintf(stderr, "[i2c-debug] %s DATA W 0x%02x (run=%d start=%d "
                    "dir=%d count=%u left=%u pc=0x%08x)\n", s->name,
                    (uint8_t)val, !!run, !!start, !!dir,
                    s->regs[0] & CTRL0_XFER_COUNT_MASK, s->xfer_left,
                    (unsigned)mxs_trace_guest_pc());
        }
        if (run && start && !dir && !s->xfer_started) {
            /* First DATA byte of a master write: the 7-bit slave address
             * left-shifted with the R/W bit.  DATA is a 32-bit register;
             * stale upper bytes must not leak into the address (observed:
             * probe byte 0x34 with 0x100 leftover produced addr 0x9a). */
            uint8_t addr = ((uint8_t)val) >> 1;
            if (brain_i2c_debug()) {
                fprintf(stderr, "[i2c-debug] %s START_SEND addr=0x%02x "
                        "pc=0x%08x\n", s->name, addr,
                        (unsigned)mxs_trace_guest_pc());
            }
            if (i2c_start_send(s->bus, addr) < 0) {
                mxs_i2c_finish(s, CTRL1_NO_SLAVE_ACK);
                break;
            }
            /* XFER_COUNT includes the address byte: a count of 1 is an
             * address-only probe; larger counts leave (count - 1) payload
             * bytes to be pushed through further DATA writes. */
            unsigned count = s->regs[0] & CTRL0_XFER_COUNT_MASK;
            s->xfer_started = true;
            s->xfer_left = count > 1 ? count - 1 : 0;
            if (count <= 1) {
                if (s->regs[0] & CTRL0_POST_SEND_STOP)
                    i2c_end_transfer(s->bus);
                mxs_i2c_finish(s, CTRL1_DATA_ENGINE_CMPLT);
            }
        } else if (run && s->xfer_started && s->xfer_left > 0) {
            /* Payload byte of a multi-byte write.  Count down against the
             * XFER_COUNT latched at start time -- the guest never rewrites
             * CTRL0 between DATA writes, so re-reading its count here (as
             * the old code did) could never reach zero. */
            if (brain_i2c_debug()) {
                fprintf(stderr, "[i2c-debug] %s SEND 0x%02x pc=0x%08x\n",
                        s->name, (uint8_t)val,
                        (unsigned)mxs_trace_guest_pc());
            }
            i2c_send(s->bus, (uint8_t)val);
            if (--s->xfer_left == 0) {
                if (s->regs[0] & CTRL0_POST_SEND_STOP)
                    i2c_end_transfer(s->bus);
                mxs_i2c_finish(s, CTRL1_DATA_ENGINE_CMPLT);
            }
        }
        break;
    }
    case 0x40 >> 4:  /* CTRL1 (plus its +0x4 SET / +0x8 CLR / +0xc TOG) */
        if (MXS_BANK_OP(off) == MXS_OP_WRITE) {
            /* The interrupt flags in bits[7:0] (and CLR_GOT_A_NAK in bit
             * 28) are write-1-to-clear; the WinCE BSP acknowledges by
             * writing 0x78 / 0x40 straight at CTRL1.  Keep the bits it
             * wrote as 0 and clear the ones it wrote as 1, while the
             * SET/CLR/TOG aliases keep the value mxs_bank_apply()
             * already computed. */
            s->regs[idx] = (val & ~CTRL1_W1C_MASK) |
                           (old & ~val & CTRL1_W1C_MASK);
        }
        mxs_i2c_update_irq(s);
        break;
    }
}

MXS_TRACE_WRAP(mxs_i2c, "i2c")
static const MemoryRegionOps mxs_i2c_ops = {
    .read = mxs_i2c_read_tr, .write = mxs_i2c_write_tr,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void mxs_i2c_reset(DeviceState *dev)
{
    MXSI2CState *s = MXS_I2C_REAL(dev);
    memset(s->regs, 0, sizeof(s->regs));
    s->xfer_left = 0;
    s->xfer_started = false;
}

static void mxs_i2c_realize(DeviceState *dev, Error **errp)
{
    MXSI2CState *s = MXS_I2C_REAL(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    g_autofree char *n = g_strdup_printf("mxs-%s", s->name ? s->name : "i2c");
    memory_region_init_io(&s->iomem, OBJECT(s), &mxs_i2c_ops, s, n,
                          s->size ? s->size : 0x2000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(dev, "i2c-bus");
    /*
     * A real board codec (SGTL5000 @ 0x0a) occupies its address: create
     * it first so the ack-all sweep below does not shadow it.  The codec
     * carries its own "audiodev" property (standard QEMU device audio);
     * when the machine has one we pass it through.
     */
    if (s->codec_type && *s->codec_type && s->codec_addr < 0x80) {
        I2CSlave *sl = I2C_SLAVE(qdev_new(s->codec_type));

        qdev_prop_set_uint8(DEVICE(sl), "address", s->codec_addr);
        if (s->codec_audiodev && *s->codec_audiodev) {
            qdev_prop_set_string(DEVICE(sl), "audiodev", s->codec_audiodev);
        }
        i2c_slave_realize_and_unref(sl, s->bus, &error_fatal);
        s->codec = sl;
    }
    for (unsigned a = 0; a < 0x80; a++) {
        if (s->codec && a == s->codec_addr) {
            continue;   /* real slave owns this address */
        }
        DeviceState *sl = qdev_new(TYPE_MXS_I2C_ACK);
        qdev_prop_set_uint8(sl, "address", a);
        qdev_realize_and_unref(sl, BUS(s->bus), &error_fatal);
    }
    s->trace = mxs_trace_enabled(s->name ? s->name : "i2c");
}

DeviceState *mxs_i2c_codec_device(DeviceState *dev)
{
    MXSI2CState *s = MXS_I2C_REAL(dev);

    return DEVICE(s->codec);
}

static const Property mxs_i2c_props[] = {
    DEFINE_PROP_STRING("name", MXSI2CState, name),
    DEFINE_PROP_UINT64("size", MXSI2CState, size, 0x2000),
    DEFINE_PROP_UINT32("ctrl-idx", MXSI2CState, ctrl_idx, 0),
    DEFINE_PROP_STRING("codec-type", MXSI2CState, codec_type),
    DEFINE_PROP_UINT8("codec-addr", MXSI2CState, codec_addr, 0),
    DEFINE_PROP_STRING("codec-audiodev", MXSI2CState, codec_audiodev),
};

static const VMStateDescription vmstate_mxs_i2c = {
    .name = "mxs-i2c-real", .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, MXSI2CState, MXS_MAX_REGS),
        VMSTATE_END_OF_LIST()
    },
};

static void mxs_i2c_class_init(ObjectClass *k, const void *d)
{
    DeviceClass *dc = DEVICE_CLASS(k);
    dc->realize = mxs_i2c_realize;
    device_class_set_legacy_reset(dc, mxs_i2c_reset);
    dc->vmsd = &vmstate_mxs_i2c;
    device_class_set_props(dc, mxs_i2c_props);
}

static const TypeInfo mxs_i2c_types[] = {
    { .name = TYPE_MXS_I2C_REAL, .parent = TYPE_SYS_BUS_DEVICE,
      .instance_size = sizeof(MXSI2CState), .class_init = mxs_i2c_class_init },
    { .name = TYPE_MXS_I2C_ACK, .parent = TYPE_I2C_SLAVE,
      .instance_size = sizeof(MXSAckSlave), .class_init = mxs_ack_class_init },
};
DEFINE_TYPES(mxs_i2c_types)
