/*
 * Helpers for the Freescale MXS (i.MX23/28) register banks.
 *
 * Almost every register of the MXS peripherals is 0x10 bytes wide:
 *
 *   +0x0  read/write
 *   +0x4  set   (reg |=  value)
 *   +0x8  clear (reg &= ~value)
 *   +0xc  toggle(reg ^=  value)
 *
 * Registers may be accessed with 8, 16 or 32 bit accesses.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#ifndef HW_MISC_MXS_BANK_H
#define HW_MISC_MXS_BANK_H

#define MXS_BANK_INDEX(offset)  ((offset) >> 4)
#define MXS_BANK_OP(offset)     (((offset) >> 2) & 3)
#define MXS_BANK_BYTE(offset)   ((offset) & 3)

#define MXS_OP_WRITE    0
#define MXS_OP_SET      1
#define MXS_OP_CLR      2
#define MXS_OP_TOG      3

/* mask covering the bytes touched by an access of @size at @offset */
static inline uint32_t mxs_bank_mask(hwaddr offset, unsigned size)
{
    unsigned byte = MXS_BANK_BYTE(offset);

    if (size >= 4) {
        return 0xffffffffu;
    }
    return (((1ull << (size * 8)) - 1) << (byte * 8)) & 0xffffffffu;
}

/* value as it should be merged into the 32 bit register */
static inline uint32_t mxs_bank_shift(hwaddr offset, uint64_t value)
{
    return (uint32_t)(value << (MXS_BANK_BYTE(offset) * 8));
}

static inline uint64_t mxs_bank_extract(hwaddr offset, unsigned size,
                                        uint32_t regval)
{
    unsigned byte = MXS_BANK_BYTE(offset);

    if (size >= 4) {
        return regval;
    }
    return (regval >> (byte * 8)) & ((1ull << (size * 8)) - 1);
}

/*
 * Apply a write to a single 32 bit register honouring the set/clr/tog
 * aliases.  Returns the new register value.
 */
static inline uint32_t mxs_bank_apply(uint32_t old, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    uint32_t mask = mxs_bank_mask(offset, size);
    uint32_t val = mxs_bank_shift(offset, value) & mask;

    switch (MXS_BANK_OP(offset)) {
    case MXS_OP_SET:
        return old | val;
    case MXS_OP_CLR:
        return old & ~val;
    case MXS_OP_TOG:
        return old ^ val;
    case MXS_OP_WRITE:
    default:
        return (old & ~mask) | val;
    }
}

/*
 * Every MXS block has the same soft reset protocol in bit 31/30 of its
 * first register:
 *
 *   SFTRST  (bit 31) holds the block in reset
 *   CLKGATE (bit 30) gates the block clock
 *
 * Asserting SFTRST also asserts CLKGATE in hardware, but CLKGATE stays
 * software writable afterwards.  Firmware (both Linux' stmp_reset_block()
 * and the WinCE BSP) performs the release sequence with plain read/modify/
 * write cycles on the register - e.g. "clear CLKGATE and spin until it
 * reads back 0" while SFTRST is still asserted.  Therefore CLKGATE must
 * only be forced on the *rising edge* of SFTRST; keeping it pinned while
 * SFTRST is set would deadlock the guest.
 */
#define MXS_SFTRST_BIT      (1u << 31)
#define MXS_CLKGATE_BIT     (1u << 30)

/*
 * Development aid: setting MXS_TRACE to "all" or to a comma separated list
 * of block names makes the models dump every register access to stderr.
 * It is only consulted once, at realize time.
 *
 * Independently of that, 'mxs_trace_live' can be flipped at runtime with
 * the 'brain_trace' HMP command to trace *all* MXS devices (with the
 * accessing guest PC) -- invaluable when reverse engineering driver flows.
 */
extern bool mxs_trace_live;
uint32_t mxs_trace_guest_pc(void);

static inline bool mxs_trace_enabled(const char *name)
{
    const char *e = getenv("MXS_TRACE");

    if (!e || !name) {
        return false;
    }
    return !strcmp(e, "all") || strstr(e, name) != NULL;
}

static inline void mxs_trace_access(const char *name, bool write,
                                    hwaddr offset, uint32_t value)
{
    static const char *const op[4] = { "  ", "set", "clr", "tog" };

    fprintf(stderr, "[mxs] %-8s %s +0x%03x %s 0x%08x pc=0x%08x\n", name,
            write ? "W" : "R", (unsigned)offset,
            write ? op[MXS_BANK_OP(offset)] : "->", value,
            (unsigned)mxs_trace_guest_pc());
}

static inline uint32_t mxs_bank_sftrst(uint32_t old, uint32_t new_val)
{
    if ((new_val & MXS_SFTRST_BIT) && !(old & MXS_SFTRST_BIT)) {
        new_val |= MXS_CLKGATE_BIT;
    }
    return new_val;
}

/*
 * Wrap a model's read/write handlers with a tracing version.  Use
 *   MXS_TRACE_WRAP(mxs_icoll, "icoll")
 * right before the MemoryRegionOps and point those at
 * mxs_icoll_read_tr / mxs_icoll_write_tr; set mxs_icoll_trace from the
 * device's init function with mxs_trace_enabled("icoll").
 */
#define MXS_TRACE_WRAP(prefix, blkname)                                      \
static bool prefix##_trace;                                                  \
static uint64_t prefix##_read_tr(void *o, hwaddr a, unsigned sz)             \
{                                                                            \
    uint64_t v = prefix##_read(o, a, sz);                                    \
    if (unlikely(prefix##_trace || mxs_trace_live)) {                        \
        mxs_trace_access(blkname, false, a, (uint32_t)v);                    \
    }                                                                        \
    return v;                                                                \
}                                                                            \
static void prefix##_write_tr(void *o, hwaddr a, uint64_t v, unsigned sz)    \
{                                                                            \
    if (unlikely(prefix##_trace || mxs_trace_live)) {                        \
        mxs_trace_access(blkname, true, a, (uint32_t)v);                     \
    }                                                                        \
    prefix##_write(o, a, v, sz);                                             \
}

#endif /* HW_MISC_MXS_BANK_H */
