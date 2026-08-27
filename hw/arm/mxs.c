/*
 * Freescale i.MX28 (MXS) SoC and the SHARP Brain (PW-xx) machine.
 *
 * The machine emulates enough of the i.MX28 boot ROM to fetch the
 * encrypted SB bootstream out of the eMMC exactly like the real chip
 * does (MBR -> 0x53 partition -> boot control block -> SB image), so an
 * untouched `emmc.img` dump boots the original XLDR/EBOOT/WinCE chain.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/arm/mxs.h"
#include "hw/arm/mxs_pwm.h"
#include "hw/arm/mxs_saif.h"
#include "hw/misc/mxs_bank.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "qemu/option.h"
#include "qemu/config-file.h"
#include "hw/core/boards.h"
#include "system/runstate.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/char/pl011.h"
#include "hw/sd/sd.h"
#include "hw/i2c/i2c.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"
#include "exec/watchpoint.h"
#include "accel/tcg/cpu-ops.h"
#include "target/arm/cpu.h"
#include "system/memory-internal.h"
#include "crypto/cipher.h"
#include "cpu.h"
#include "monitor/hmp.h"
#include "monitor/monitor.h"
#include "qobject/qdict.h"
#include "ui/input.h"
#include "brain_stats.h"

/* FAT reader for the SD-card payload (Brainux / WinCE EBOOT) */
#include "hw/arm/mxs_fat.c"

#define MXS_SECTOR_SIZE     512

/* ------------------------------------------------------------------ */
/* SB (bootstream) image format                                        */
/* ------------------------------------------------------------------ */

#define SB_BLOCK_SIZE       16

typedef struct QEMU_PACKED SBHeader {
    uint8_t  sha1[20];
    uint8_t  signature[4];      /* "STMP" */
    uint8_t  major;
    uint8_t  minor;
    uint16_t flags;
    uint32_t image_size;        /* in 16 byte blocks */
    uint32_t first_boot_tag_off;
    uint32_t first_boot_sec_id;
    uint16_t nr_keys;
    uint16_t key_dict_off;      /* in blocks */
    uint16_t header_size;       /* in blocks */
    uint16_t nr_sections;
    uint16_t sec_hdr_size;      /* in blocks */
    uint8_t  rand_pad0[6];
    uint64_t timestamp;
    uint8_t  product_ver[12];
    uint8_t  component_ver[12];
    uint16_t drive_tag;
    uint8_t  rand_pad1[6];
} SBHeader;

typedef struct QEMU_PACKED SBSectionHeader {
    uint32_t identifier;
    uint32_t offset;            /* in blocks */
    uint32_t size;              /* in blocks */
    uint32_t flags;
} SBSectionHeader;

#define SB_SECTION_BOOTABLE     (1u << 0)
#define SB_SECTION_CLEARTEXT    (1u << 1)

#define SB_INST_NOP     0x0
#define SB_INST_TAG     0x1
#define SB_INST_LOAD    0x2
#define SB_INST_FILL    0x3
#define SB_INST_JUMP    0x4
#define SB_INST_CALL    0x5
#define SB_INST_MODE    0x6

/* i.MX HAB image vector table */
#define IVT_TAG         0xd1

/* ------------------------------------------------------------------ */

typedef struct BrainMachineState {
    MachineState parent_obj;

    ARMCPU *cpu;
    DeviceState *icoll;
    DeviceState *pinctrl;
    DeviceState *apbh;
    DeviceState *apbx;
    DeviceState *ssp[4];
    DeviceState *lcdif;
    DeviceState *kbd;
    DeviceState *lradc;
    uint32_t edna2_touchkey;   /* +0x404 key bits (see brain_kbd.c) */

    MemoryRegion ocram;
    MemoryRegion ocram_alias;
    MemoryRegion rom;
    MemoryRegion trampoline;
    MemoryRegion edna2_mb_iomem;
    uint8_t edna2_mb[0x1000];

    /* boot state, recomputed on every system reset */
    uint32_t entry;
    BlockBackend *emmc_blk;
    BlockBackend *sd_blk;
    char *boot_mode;
    uint32_t lcd_width;
    uint32_t lcd_height;
    uint32_t lcd_rotate;
    bool verbose;
    /*
     * Strict-hardware fidelity mode.  When true (the default) every QEMU
     * side-channel that compensates for a piece of hardware the real device
     * has but QEMU does not model is disabled, so the guest fails exactly
     * like the real Brain does.  The individual switches below let a user
     * re-enable a single aid for analysis.
     *
     *   strict_hw           master switch
     *   aid_edna2_status    EDNA2 MCU mailbox status block seeding
     *   aid_edna2_resp      EDNA2 MCU command-response heuristics
     *   aid_region4_remap   FMD Region 4 sector-window remap
     *   aid_sd_launcher     EBOOT-equivalent SD launcher auto-boot
     *   aid_ignore_bus_err  ignore unmapped / aborted memory transactions
     */
    bool strict_hw;
    bool aid_edna2_status;
    bool aid_edna2_uninit;
    bool aid_edna2_resp;
    bool aid_region4_remap;
    bool aid_sd_launcher;
    bool aid_ignore_bus_err;
    /*
     * BRAIN fault-zone experiment aid (verification-only, default off).
     * Injects faulty responses for guest reads of the fault-erased head
     * sectors (Nand2 LBA 0x27800.., Nand4 LBA 0x47800..) to reproduce the
     * real unit's "しばらくお待ちください" hang in QEMU:
     *   exp_fault_mode 0 = off, 1 = read error (R1 ADDRESS_ERROR),
     *                  2 = virtual-time read delay, 3 = trace only.
     */
    uint32_t exp_fault_start;
    uint32_t exp_fault_len;
    uint32_t exp_fault_mode;
    uint32_t exp_fault_delay_us;
    /*
     * EDNA2 MCU command-processing state (real-device model, not an aid).
     * The MCU executes the mailbox command protocol: the guest writes a
     * command byte to +0xE8 and kicks the doorbell at +0x3C; the MCU
     * polls the mailbox, latches the command, processes it in real time
     * and posts the done flag in +0xE8 bit0 plus command-specific
     * results (e.g. the touchkey calibration ready flags in +0x404).
     * Decoded from keybd_EDNA2.dll + mailbox traces (S11 report).
     */
    QEMUTimer *edna2_mcu_timer;
    bool edna2_mcu_busy;        /* command in flight */
    bool edna2_mcu_latched;     /* command byte latched (stage 2) */
    uint8_t edna2_mcu_cmd;      /* latched command byte */
    bool sb_fat_mode;          /* MODE 0x09 toggled by SB image */
    /*
     * GPMI NAND media (hw/misc/mxs_gpmi.c).  Default off: the Brain
     * PW-AJ2 GPMI socket is unpopulated (no NAND item in the factory MP
     * test list, BSP reports "flash initialization failed" on every
     * boot).  'gpmi-nand=true' attaches a full NAND device model
     * (64 MiB / 2 KiB pages / 64 B OOB default) optionally backed by a
     * raw image file.
     */
    bool gpmi_nand;
    char *gpmi_nand_file;
    struct arm_boot_info boot_info;
} BrainMachineState;

#define TYPE_BRAIN_MACHINE MACHINE_TYPE_NAME("brain")
OBJECT_DECLARE_SIMPLE_TYPE(BrainMachineState, BRAIN_MACHINE)

/* ------------------------------------------------------------------ */
/* boot ROM emulation                                                  */
/* ------------------------------------------------------------------ */

static bool mxs_aes_cbc(const uint8_t key[16], const uint8_t iv[16],
                        const uint8_t *in, uint8_t *out, size_t len,
                        bool encrypt)
{
    QCryptoCipher *cipher;
    Error *err = NULL;
    bool ok = true;

    cipher = qcrypto_cipher_new(QCRYPTO_CIPHER_ALGO_AES_128,
                                QCRYPTO_CIPHER_MODE_CBC, key, 16, &err);
    if (!cipher) {
        error_report_err(err);
        return false;
    }
    if (qcrypto_cipher_setiv(cipher, iv, 16, &err) < 0) {
        error_report_err(err);
        qcrypto_cipher_free(cipher);
        return false;
    }
    if (encrypt) {
        ok = qcrypto_cipher_encrypt(cipher, in, out, len, &err) == 0;
    } else {
        ok = qcrypto_cipher_decrypt(cipher, in, out, len, &err) == 0;
    }
    if (!ok) {
        error_report_err(err);
    }
    qcrypto_cipher_free(cipher);
    return ok;
}

/*
 * Resolve a CALL/JUMP target: the WinCE bootstreams point at a HAB
 * image vector table rather than at real code.
 */
static uint32_t mxs_resolve_entry(uint32_t addr)
{
    uint8_t ivt[32];

    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       ivt, sizeof(ivt));
    if (ivt[0] == IVT_TAG && ((ivt[3] & 0xf0) == 0x40) &&
        lduw_be_p(ivt + 1) == 0x20) {
        uint32_t entry = ldl_le_p(ivt + 4);

        if (entry) {
            return entry;
        }
    }
    return addr;
}

typedef struct SBRun {
    uint32_t first_entry;
    uint32_t last_entry;
    int n_entries;
} SBRun;

static void mxs_sb_exec_section(BrainMachineState *bms, const uint8_t *data,
                                size_t len, SBRun *run)
{
    size_t pos = 0;

    while (pos + 16 <= len) {
        uint8_t opcode = data[pos + 1];
        uint32_t addr = ldl_le_p(data + pos + 4);
        uint32_t count = ldl_le_p(data + pos + 8);
        uint32_t arg = ldl_le_p(data + pos + 12);

        switch (opcode) {
        case SB_INST_NOP:
            pos += 16;
            break;

        case SB_INST_TAG:
            pos += 16;
            break;

        case SB_INST_LOAD: {
            size_t payload = (count + 15) & ~(size_t)15;

            if (pos + 16 + count > len) {
                warn_report("mxs-rom: truncated LOAD at %zu", pos);
                return;
            }
            if (bms->verbose) {
                info_report("mxs-rom: LOAD 0x%08x len 0x%x", addr, count);
            }
            address_space_write(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, data + pos + 16,
                                count);
            pos += 16 + payload;
            break;
        }

        case SB_INST_FILL: {
            g_autofree uint8_t *buf = g_malloc(count ? count : 1);
            uint32_t i;

            for (i = 0; i < count; i += 4) {
                stl_le_p(buf + i, arg);
            }
            if (bms->verbose) {
                info_report("mxs-rom: FILL 0x%08x len 0x%x pat 0x%08x",
                            addr, count, arg);
            }
            address_space_write(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, buf, count);
            pos += 16;
            break;
        }

        case SB_INST_CALL:
        case SB_INST_JUMP: {
            uint32_t entry = mxs_resolve_entry(addr);

            if (bms->verbose) {
                info_report("mxs-rom: %s 0x%08x -> entry 0x%08x",
                            opcode == SB_INST_CALL ? "CALL" : "JUMP",
                            addr, entry);
            }
            if (!run->n_entries) {
                run->first_entry = entry;
            }
            run->last_entry = entry;
            run->n_entries++;
            pos += 16;
            break;
        }

        case SB_INST_MODE:
            /*
             * SHARP Brain XLDR uses MODE 0x09 to switch from raw
             * eMMC sector reads to FAT filesystem reads on the
             * same storage.  When the guest later issues a LOAD
             * to the SB-base area, the data is taken from the FAT
             * image, not from raw sectors.
             */
            if (arg == 0x09) {
                bms->sb_fat_mode = true;
            } else if (arg == 0x00) {
                bms->sb_fat_mode = false;
            }
            pos += 16;
            break;

        default:
            warn_report("mxs-rom: unknown SB opcode %d at %zu", opcode, pos);
            return;
        }
    }
}

/*
 * Load the bootstream found at @sb_offset (byte offset into the eMMC)
 * into guest memory.  Returns true on success.
 */
static bool mxs_rom_load_sb(BrainMachineState *bms, uint64_t sb_offset,
                            SBRun *run)
{
    SBHeader hdr;
    g_autofree uint8_t *image = NULL;
    uint8_t dek[16];
    uint8_t iv[16];
    size_t image_size;
    int i;

    if (blk_pread(bms->emmc_blk, sb_offset, sizeof(hdr), &hdr, 0) < 0) {
        error_report("mxs-rom: cannot read SB header");
        return false;
    }
    if (memcmp(hdr.signature, "STMP", 4)) {
        error_report("mxs-rom: no STMP signature at offset 0x%" PRIx64,
                     sb_offset);
        return false;
    }

    bms->sb_fat_mode = false;

    image_size = (size_t)le32_to_cpu(hdr.image_size) * SB_BLOCK_SIZE;
    if (!image_size || image_size > 16 * MiB) {
        error_report("mxs-rom: bogus SB image size %zu", image_size);
        return false;
    }
    image = g_malloc(image_size);
    if (blk_pread(bms->emmc_blk, sb_offset, image_size, image, 0) < 0) {
        error_report("mxs-rom: short read of SB image");
        return false;
    }

    memcpy(iv, image, 16);

    if (bms->verbose) {
        info_report("mxs-rom: SB v%d.%d, %zu bytes, %d section(s), %d key(s)",
                    hdr.major, hdr.minor, image_size,
                    le16_to_cpu(hdr.nr_sections), le16_to_cpu(hdr.nr_keys));
    }

    memset(dek, 0, sizeof(dek));
    if (le16_to_cpu(hdr.nr_keys)) {
        /*
         * The key dictionary entry holds the data encryption key wrapped
         * with the OTP key.  Production Brain units are shipped with an
         * all zero OTP key, so unwrap with zeroes.
         */
        size_t off = (size_t)le16_to_cpu(hdr.key_dict_off) * SB_BLOCK_SIZE;
        uint8_t zero[16] = { 0 };

        if (off + 32 > image_size) {
            error_report("mxs-rom: key dictionary out of range");
            return false;
        }
        if (!mxs_aes_cbc(zero, iv, image + off + 16, dek, 16, false)) {
            return false;
        }
        if (bms->verbose) {
            info_report("mxs-rom: DEK %02x%02x%02x%02x...",
                        dek[0], dek[1], dek[2], dek[3]);
        }
    }

    for (i = 0; i < le16_to_cpu(hdr.nr_sections); i++) {
        size_t shoff = (size_t)le16_to_cpu(hdr.header_size) * SB_BLOCK_SIZE +
                       (size_t)i * sizeof(SBSectionHeader);
        SBSectionHeader sh;
        size_t soff, ssize;
        g_autofree uint8_t *sdata = NULL;

        if (shoff + sizeof(sh) > image_size) {
            break;
        }
        memcpy(&sh, image + shoff, sizeof(sh));
        soff = (size_t)le32_to_cpu(sh.offset) * SB_BLOCK_SIZE;
        ssize = (size_t)le32_to_cpu(sh.size) * SB_BLOCK_SIZE;

        if (soff + ssize > image_size) {
            warn_report("mxs-rom: section %d out of range", i);
            continue;
        }
        if (!(le32_to_cpu(sh.flags) & SB_SECTION_BOOTABLE)) {
            continue;
        }

        sdata = g_malloc(ssize);
        if (le32_to_cpu(sh.flags) & SB_SECTION_CLEARTEXT) {
            memcpy(sdata, image + soff, ssize);
        } else {
            if (!mxs_aes_cbc(dek, iv, image + soff, sdata, ssize, false)) {
                return false;
            }
        }
        mxs_sb_exec_section(bms, sdata, ssize, run);
    }

    return run->n_entries > 0;
}

/*
 * Walk MBR -> 0x53 partition -> boot control block to find the SB image,
 * exactly like the i.MX28 ROM does when booting from SD/MMC.
 */
static bool mxs_rom_find_and_load(BrainMachineState *bms, SBRun *run)
{
    uint8_t sector[MXS_SECTOR_SIZE];
    uint32_t boot_lba = 0;
    uint32_t sig, primary_tag, nr_copies;
    uint32_t sb_sector = 0;
    unsigned i;

    if (!bms->emmc_blk) {
        return false;
    }

    if (blk_pread(bms->emmc_blk, 0, sizeof(sector), sector, 0) < 0) {
        error_report("mxs-rom: cannot read MBR");
        return false;
    }

    if (sector[510] == 0x55 && sector[511] == 0xaa) {
        for (i = 0; i < 4; i++) {
            const uint8_t *e = sector + 446 + i * 16;

            if (e[4] == 0x53) {
                boot_lba = ldl_le_p(e + 8);
                break;
            }
        }
    }
    if (!boot_lba) {
        /* no boot partition: the ROM then expects the BCB at sector 0 */
        warn_report("mxs-rom: no 0x53 partition in MBR, trying sector 0");
    }

    if (blk_pread(bms->emmc_blk, (uint64_t)boot_lba * MXS_SECTOR_SIZE,
                  sizeof(sector), sector, 0) < 0) {
        error_report("mxs-rom: cannot read boot control block");
        return false;
    }

    sig = ldl_le_p(sector);
    if (sig != 0x00112233) {
        /* Maybe the SB image sits directly at the partition start */
        if (!memcmp(sector + 20, "STMP", 4)) {
            return mxs_rom_load_sb(bms, (uint64_t)boot_lba * MXS_SECTOR_SIZE,
                                   run);
        }
        error_report("mxs-rom: bad boot control block signature 0x%08x", sig);
        return false;
    }

    primary_tag = ldl_le_p(sector + 4);
    nr_copies = ldl_le_p(sector + 12);
    if (nr_copies > 8) {
        nr_copies = 8;
    }

    for (i = 0; i < nr_copies; i++) {
        const uint8_t *di = sector + 16 + i * 20;
        uint32_t tag = ldl_le_p(di + 8);
        uint32_t first = ldl_le_p(di + 12);

        if (bms->verbose) {
            info_report("mxs-rom: BCB drive %u tag %u sector %u count %u",
                        i, tag, first, ldl_le_p(di + 16));
        }
        if (tag == primary_tag) {
            sb_sector = first;
            break;
        }
    }
    if (!sb_sector) {
        error_report("mxs-rom: no drive entry for boot tag %u", primary_tag);
        return false;
    }

    if (bms->verbose) {
        info_report("mxs-rom: bootstream at sector %u", sb_sector);
    }

    return mxs_rom_load_sb(bms, (uint64_t)sb_sector * MXS_SECTOR_SIZE, run);
}

/* Address of the tiny "return from XLDR" trampoline inside the fake ROM */
#define MXS_TRAMPOLINE_ADDR (MXS_ROM_BASE + 0x100)

/*
 * SD-card WinCE/Brainux interrupt.  The SHARP Brain's stock EBOOT
 * scans the boot partition's root for a file matching the
 * machine-specific "edxxNexe.BIN" pattern (or "/NK/EDSH6EXE.BIN"
 * for the buildbrain layout), interprets it as either a packed
 * B000FF\n image (which gets copied to 0xa0200000 and jumped into)
 * or a raw ARM binary (jumped into as-is at its natural address).
 * Re-implementing the full EBOOT to do this on the SD card is
 * several thousand lines of WinCE code, so we play the role of the
 * SD-interrupt here: after the eMMC boot stream has set up
 * EBOOT, look on the SD card for one of the launchers and run
 * it.  This is what a real Brain would do if the user had
 * installed a "Launch Linux" entry point on the SD card.
 */
static bool mxs_rom_try_sd_image(BrainMachineState *bms)
{
    /*
     * The buildbrain SD image lays the file under /NK/EDSH6EXE.BIN
     * to keep the root directory uncluttered.  The stock WinCE
     * boot from a Brain SD card looks for the file directly in
     * the root, named after the model (e.g. "edsh6exe.bin" for
     * PW-SH6).  Try both layouts in that order.
     */
    static const char *const brain_launchers[] = {
        /* The "brain" machine models a SHARP Brain PW-SH6, so only
         * the SH6 / NA3 launchers from the buildbrain SD image are
         * tried.  (PW-SH6 is the most common 2nd-generation unit
         * the upstream docs cover.) */
        "edsh6exe.bin", "nk/edsH6EXE.BIN", "nk/ednA3EXE.BIN",
        "u-boot.sb", "nk.bin",
        NULL
    };
    g_autofree uint8_t *image = g_malloc(2 * MiB);
    int n;

    if (!bms->sd_blk) {
        return false;
    }
    n = mxs_fat_read_file(bms->sd_blk, brain_launchers, image, 2 * MiB);
    if (n < 0) {
        return false;
    }
    if (bms->verbose) {
        info_report("mxs-rom: SD launcher %d bytes", n);
    }

    /*
     * B000FF\n is the SHARP WinCE packed-image header.  The u-Boot packed
     * version of edsh6exe.bin starts with that magic, so the
     * destination address and image length follow.
     */
    if (n > 15 && memcmp(image, "B000FF\n", 7) == 0) {
        uint32_t dst_vaddr = ldl_le_p(image + 7);
        uint32_t length   = ldl_le_p(image + 11);
        uint32_t paddr;

        if (length == 0 || length > (uint32_t)(n - 15)) {
            length = n - 15;
        }
        /* 0xa0200000 in WinCE virtual == 0x40200000 physical on the
         * i.MX28.  Translate once and copy. */
        paddr = dst_vaddr - 0xa0000000u;
        if (bms->verbose) {
            info_report("mxs-rom: SD packed image vaddr 0x%08x paddr "
                        "0x%08x len 0x%x", dst_vaddr, paddr, length);
        }
        address_space_write(&address_space_memory, paddr,
                            MEMTXATTRS_UNSPECIFIED, image + 15, length);
        /* mxs_rom_load_sb() resets the CPU as a side effect; we
         * need to leave it pointing at the just-deposited image
         * instead. */
        cpu_set_pc(CPU(bms->cpu), paddr);
        bms->cpu->env.thumb = 0;
        bms->cpu->env.regs[13] = 0x40200000 + 0x80000;   /* 512 KiB stack */
        return true;
    }

    /* No B000FF\n magic: treat as a raw ARM binary, drop it at the
     * SD card's natural address and start it. */
    address_space_write(&address_space_memory, 0x40200000,
                        MEMTXATTRS_UNSPECIFIED, image, n);
    cpu_set_pc(CPU(bms->cpu), 0x40200000);
    bms->cpu->env.thumb = 0;
    bms->cpu->env.regs[13] = 0x40200000 + 0x80000;
    return true;
}

/*
 * Human-monitor passthrough that lets a user with a QEMU
 * -monitor unix:... socket "press the Launch Linux button" on
 * the running WinCE guest.  The real hardware does this through
 * brainlilo.exe; in QEMU we re-implement the equivalent scan
 * (mxs_rom_try_sd_image) and re-direct the CPU at the launcher
 * that ends up at 0x40200000.  No system reset is performed
 * because the launcher wipes the old program state itself and
 * expects to take over from the WinCE scheduler.
 */
void hmp_brain_lilo(Monitor *mon, const QDict *qdict)
{
    BrainMachineState *bms;

    if (!mon || !current_machine) {
        return;
    }
    bms = BRAIN_MACHINE(current_machine);
    if (!bms->sd_blk) {
        monitor_printf(mon, "brain_lilo: no SD card attached\n");
        return;
    }
    if (mxs_rom_try_sd_image(bms)) {
        monitor_printf(mon,
                       "brain_lilo: SD launcher deposited at 0x40200000, "
                       "PC=0x40200000\n");
    } else {
        monitor_printf(mon,
                       "brain_lilo: no suitable launcher found on the SD card\n");
    }
}

/*
 * ------------------------------------------------------------------
 * Analysis aids: brain_watch / BRAIN_UARTWATCH
 *
 * Two debugging aids used to locate guest code paths dynamically
 * (e.g. which driver prints the touchkey calibration failure).  Both
 * are inert unless invoked (HMP command / env var) and add no state
 * changes when unused.
 * ------------------------------------------------------------------
 */

/*
 * Guest-VA data watchpoint.  arm_debug_excp_handler ignores plain
 * (non BP_CPU) watchpoints, so we chain our own debug handler that
 * dumps the vCPU registers and the stack of the accessing code on a
 * hit.  The previous handler is kept and called for everything else.
 * NOTE: TCGCPUOps is a shared const object - we operate on a private
 * g_memdup2 copy, never on the original.
 */
static void (*brain_prev_debug_excp_handler)(CPUState *cs);
static int brain_watch_hit_count;
/*
 * counted brain_bwatch: per-address remaining-hit counters.  The
 * debug handler decrements the entry matching the faulting PC and
 * keeps the VM running (cpu-exec.c auto-resumes via
 * brain_bwatch_auto_resume) until every entry is exhausted, then all
 * BP_GDB breakpoints are removed.
 */
#define BRAIN_BWATCH_MAX 16
static struct {
    vaddr va;
    int remaining;
} brain_bwatch_slots[BRAIN_BWATCH_MAX];
static void brain_watch_debug_excp_handler(CPUState *cs);

/* chain our debug handler once; safe to call multiple times */
static void brain_chain_debug_handler(CPUState *cs)
{
    if (brain_prev_debug_excp_handler) {
        return;
    }
    TCGCPUOps *copy = g_memdup2(cs->cc->tcg_ops, sizeof(TCGCPUOps));

    brain_prev_debug_excp_handler = cs->cc->tcg_ops->debug_excp_handler;
    copy->debug_excp_handler = brain_watch_debug_excp_handler;
    ((CPUClass *)cs->cc)->tcg_ops = copy;
    fprintf(stderr, "[brain-watch] debug handler chained\n");
}

static void brain_watch_debug_excp_handler(CPUState *cs)
{
    if (cs->watchpoint_hit) {
        CPUWatchpoint *wp = cs->watchpoint_hit;
        CPUARMState *env = &ARM_CPU(cs)->env;
        uint8_t buf[128];
        int i, w;

        fprintf(stderr,
                "[brain-watch] HIT %d va=0x%08x pc=0x%08x r0=%08x r1=%08x "
                "r2=%08x r3=%08x r4=%08x r5=%08x r6=%08x r7=%08x "
                "r8=%08x r9=%08x r10=%08x r11=%08x lr=%08x sp=%08x "
                "cpsr=%08x flags=%x\n",
                ++brain_watch_hit_count,
                (unsigned)wp->hitaddr, (unsigned)env->regs[15],
                (unsigned)env->regs[0], (unsigned)env->regs[1],
                (unsigned)env->regs[2], (unsigned)env->regs[3],
                (unsigned)env->regs[4], (unsigned)env->regs[5],
                (unsigned)env->regs[6], (unsigned)env->regs[7],
                (unsigned)env->regs[8], (unsigned)env->regs[9],
                (unsigned)env->regs[10], (unsigned)env->regs[11],
                (unsigned)env->regs[14], (unsigned)env->regs[13],
                (unsigned)cpsr_read(env), wp->flags);
        for (w = 0; w < 4; w++) {
            if (cpu_memory_rw_debug(cs, env->regs[13] + w * 0x80,
                                    buf, sizeof(buf), 0) == 0) {
                fprintf(stderr, "[brain-watch] stack+0x%03x:",
                        w * 0x80);
                for (i = 0; i < 32; i++) {
                    fprintf(stderr, " %08x", ldl_le_p(buf + i * 4));
                }
                fprintf(stderr, "\n");
            }
        }
        /* dump-once: remove only the hit watchpoint (others stay
         * armed so several BRAIN_WATCH entries each fire once) */
        cpu_watchpoint_remove_by_ref(cs, wp);
        cs->watchpoint_hit = NULL;
        return;
    }
    if (cs->exception_index == EXCP_DEBUG) {
        /*
         * Execution breakpoint (brain_bwatch): report the PC and
         * registers, then remove the breakpoint so the guest can
         * continue.  This lets us observe when a specific guest
         * routine (e.g. keybd_EDNA2 SetDirectKey at 0xc0878b10) is
         * actually reached.
         *
         * Counted breakpoints: the BP check fires *before* the
         * instruction at the BP executes, so a plain resume would
         * re-trigger forever without advancing.  We therefore switch
         * the vCPU to one-insn singlestep (SSTEP_ENABLE|SSTEP_NOIRQ);
         * the post-step EXCP_DEBUG clears the step and the guest
         * continues until the PC reaches a BP again.  This yields one
         * pre-execution register snapshot per real visit to the BP.
         */
        CPUARMState *env = &ARM_CPU(cs)->env;
        vaddr pc = env->regs[15];
        int i, alive = 0;

        if (cs->singlestep_enabled & SSTEP_ENABLE) {
            /* post-step: the BP instruction has now executed */
            cs->singlestep_enabled = 0;
            for (i = 0; i < BRAIN_BWATCH_MAX; i++) {
                if (brain_bwatch_slots[i].remaining > 0) {
                    alive = 1;
                    break;
                }
            }
            if (!alive) {
                fprintf(stderr, "[brain-bwatch] all disarmed\n");
                brain_bwatch_auto_resume = 0;
            }
            return;
        }

        for (i = 0; i < BRAIN_BWATCH_MAX; i++) {
            if (brain_bwatch_slots[i].remaining > 0 &&
                brain_bwatch_slots[i].va == pc) {
                brain_bwatch_slots[i].remaining--;
                if (brain_bwatch_slots[i].remaining == 0) {
                    /* this slot is done: remove only ITS breakpoint so
                     * other addresses keep counting independently */
                    cpu_breakpoint_remove(cs,
                                          brain_bwatch_slots[i].va, BP_GDB);
                    fprintf(stderr,
                            "[brain-bwatch] 0x%08lx disarmed\n",
                            (unsigned long)brain_bwatch_slots[i].va);
                }
            }
        }
        for (i = 0; i < BRAIN_BWATCH_MAX; i++) {
            if (brain_bwatch_slots[i].remaining > 0) {
                alive = 1;
                break;
            }
        }
        fprintf(stderr,
                "[brain-bwatch] HIT %d pc=0x%08x r0=%08x r1=%08x r2=%08x "
                "r3=%08x r4=%08x r5=%08x lr=%08x sp=%08x cpsr=%08x "
                "(alive=%d)\n",
                ++brain_watch_hit_count,
                (unsigned)pc,
                (unsigned)env->regs[0], (unsigned)env->regs[1],
                (unsigned)env->regs[2], (unsigned)env->regs[3],
                (unsigned)env->regs[4], (unsigned)env->regs[5],
                (unsigned)env->regs[14], (unsigned)env->regs[13],
                (unsigned)cpsr_read(env), alive);
        if (!alive) {
            cpu_breakpoint_remove_all(cs, BP_GDB);
            fprintf(stderr, "[brain-bwatch] all disarmed\n");
        }
        /* single-step over the BP insn so the guest advances; the
         * post-step handler above clears the step (and, on the final
         * hit, clears auto_resume so the next EXCP_DEBUG stops) */
        cs->singlestep_enabled = SSTEP_ENABLE | SSTEP_NOIRQ;
        return;
    }
    if (brain_prev_debug_excp_handler) {
        brain_prev_debug_excp_handler(cs);
    }
}

/*
 * brain_bwatch <va> [count] -- execution breakpoint.
 *
 * Stops the guest at <va> once (dumps PC + registers to stderr via
 * brain_watch_debug_excp_handler) and removes itself so the guest
 * continues.  With count > 1 the guest keeps running and the BP stays
 * armed for `count` hits (cpu-exec.c auto-resumes via
 * brain_bwatch_auto_resume).  Used to prove whether a specific guest
 * routine is reached (e.g. keybd_EDNA2 SetDirectKey at 0xc0878b10
 * after `sendkey y`).
 */
static void brain_bwatch_arm(Monitor *mon, CPUState *cs, vaddr va,
                             int count)
{
    CPUBreakpoint *bp = NULL;
    int i;

    brain_chain_debug_handler(cs);
    /*
     * Use BP_GDB rather than BP_CPU: on ARM926 (ARMv5) the arch
     * debug check (arm_debug_check_breakpoint) requires MDSCR_EL1.TDE
     * which does not exist pre-ARMv7, so BP_CPU breakpoints never
     * match.  BP_GDB bypasses the arch check in
     * check_for_breakpoints_slow().
     */
    if (cpu_breakpoint_insert(cs, va, BP_GDB, &bp) != 0) {
        if (mon) {
            monitor_printf(mon, "brain_bwatch: insert failed\n");
        }
        return;
    }
    for (i = 0; i < BRAIN_BWATCH_MAX; i++) {
        if (brain_bwatch_slots[i].remaining == 0) {
            brain_bwatch_slots[i].va = va;
            brain_bwatch_slots[i].remaining = count > 1 ? count : 1;
            break;
        }
    }
    brain_bwatch_auto_resume = 1;
    if (mon) {
        monitor_printf(mon, "brain_bwatch: 0x%08lx count %d inserted (%p)\n",
                       (unsigned long)va, count > 1 ? count : 1, (void *)bp);
    }
}

void hmp_brain_bwatch(Monitor *mon, const QDict *qdict)
{
    BrainMachineState *bms;
    CPUState *cs;

    if (!current_machine) {
        return;
    }
    bms = BRAIN_MACHINE(current_machine);
    cs = CPU(bms->cpu);
    brain_bwatch_arm(mon, cs, qdict_get_int(qdict, "va"),
                     qdict_haskey(qdict, "count") ?
                     qdict_get_int(qdict, "count") : 1);
}

void hmp_brain_watch(Monitor *mon, const QDict *qdict)
{
    uint64_t addr = qdict_get_int(qdict, "addr");
    uint64_t len = qdict_get_int(qdict, "len");
    BrainMachineState *bms = BRAIN_MACHINE(current_machine);
    CPUWatchpoint *wp = NULL;
    CPUState *cs = CPU(bms->cpu);

    brain_chain_debug_handler(cs);
    cpu_watchpoint_insert(cs, addr, len,
                          BP_MEM_ACCESS | BP_STOP_BEFORE_ACCESS, &wp);
    monitor_printf(mon, "brain_watch: 0x%llx len %llu inserted (%p)\n",
                   (unsigned long long)addr, (unsigned long long)len,
                   (void *)wp);
}

/*
 * brain_wwatch <addr> [len] -- write-only watchpoint (BP_MEM_WRITE).
 * Like brain_watch but fires only on stores, so read-heavy globals
 * (e.g. keybd g_keydata at 0xc087a750, read by the scan every cycle)
 * do not trip it on every read.
 */
void hmp_brain_wwatch(Monitor *mon, const QDict *qdict)
{
    uint64_t addr = qdict_get_int(qdict, "addr");
    uint64_t len = qdict_get_int(qdict, "len");
    BrainMachineState *bms = BRAIN_MACHINE(current_machine);
    CPUWatchpoint *wp = NULL;
    CPUState *cs = CPU(bms->cpu);

    brain_chain_debug_handler(cs);
    cpu_watchpoint_insert(cs, addr, len,
                          BP_MEM_WRITE | BP_STOP_BEFORE_ACCESS, &wp);
    monitor_printf(mon, "brain_wwatch: 0x%llx len %llu inserted (%p)\n",
                   (unsigned long long)addr, (unsigned long long)len,
                   (void *)wp);
}

void hmp_brain_unwatch(Monitor *mon, const QDict *qdict)
{
    uint64_t addr = qdict_get_int(qdict, "addr");
    uint64_t len = qdict_get_int(qdict, "len");
    BrainMachineState *bms = BRAIN_MACHINE(current_machine);

    cpu_watchpoint_remove(CPU(bms->cpu), addr, len, BP_MEM_ACCESS);
    monitor_printf(mon, "brain_unwatch: 0x%llx len %llu removed\n",
                   (unsigned long long)addr, (unsigned long long)len);
}

/*
 * DUART shadow (env BRAIN_UARTWATCH="<substring>"): watch the guest's
 * serial output byte stream; when the recent window ends with the
 * substring, dump the vCPU registers and stack of the writer (the
 * whole DebugPrint call chain is live while a message is output).
 * The shadow dispatches to the pl011's own MemoryRegion so there is
 * no address-space re-entry (and no second chardev).
 */
static MemoryRegion brain_duart_shadow;
static char brain_duart_win[64];
static int brain_duart_winlen;
static bool brain_duart_armed;

static uint64_t brain_duart_shadow_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    SysBusDevice *sbd = opaque;
    MemoryRegion *mr = sysbus_mmio_get_region(sbd, 0);
    uint64_t v = 0;

    memory_region_dispatch_read(mr, offset, &v, size_memop(size),
                                MEMTXATTRS_UNSPECIFIED);
    return v;
}

static void brain_duart_shadow_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    SysBusDevice *sbd = opaque;
    MemoryRegion *mr = sysbus_mmio_get_region(sbd, 0);
    const char *needle;
    int i;

    memory_region_dispatch_write(mr, offset, value, size_memop(size),
                                 MEMTXATTRS_UNSPECIFIED);
    if (!brain_duart_armed || offset != 0 /* UARTDR */) {
        return;
    }
    needle = getenv("BRAIN_UARTWATCH");
    if (!needle || !*needle) {
        return;
    }
    if (brain_duart_winlen < (int)sizeof(brain_duart_win) - 1) {
        brain_duart_win[brain_duart_winlen++] = value & 0xff;
    } else {
        memmove(brain_duart_win, brain_duart_win + 1,
                sizeof(brain_duart_win) - 1);
        brain_duart_win[sizeof(brain_duart_win) - 2] = value & 0xff;
    }
    brain_duart_win[brain_duart_winlen] = '\0';
    if (strlen(needle) <= (size_t)brain_duart_winlen &&
        !strcmp(brain_duart_win + brain_duart_winlen - strlen(needle),
                needle)) {
        CPUState *cs = current_cpu;
        CPUARMState *env = &ARM_CPU(cs)->env;
        uint8_t buf[128];
        int w;

        fprintf(stderr,
                "[brain-uartwatch] MATCH %s pc=%08x lr=%08x sp=%08x "
                "r0=%08x r1=%08x r2=%08x r3=%08x r4=%08x r5=%08x "
                "r6=%08x r7=%08x\n",
                needle, (unsigned)env->regs[15], (unsigned)env->regs[14],
                (unsigned)env->regs[13], (unsigned)env->regs[0],
                (unsigned)env->regs[1], (unsigned)env->regs[2],
                (unsigned)env->regs[3], (unsigned)env->regs[4],
                (unsigned)env->regs[5], (unsigned)env->regs[6],
                (unsigned)env->regs[7]);
        for (w = 0; w < 16; w++) {
            if (cpu_memory_rw_debug(cs, env->regs[13] + w * 0x80,
                                    buf, sizeof(buf), 0) == 0) {
                fprintf(stderr, "[brain-uartwatch] stack+0x%03x:",
                        w * 0x80);
                for (i = 0; i < 32; i++) {
                    fprintf(stderr, " %08x", ldl_le_p(buf + i * 4));
                }
                fprintf(stderr, "\n");
            }
        }
        /* message buffer at r4 and the arg pointers */
        for (w = 0; w < 8; w++) {
            vaddr p = (w == 0) ? env->regs[4] :
                      (w == 1) ? env->regs[5] :
                      (w == 2) ? env->regs[0] :
                      (w == 3) ? env->regs[1] :
                      (w == 4) ? env->regs[2] :
                      (w == 5) ? env->regs[3] :
                      (w == 6) ? env->regs[14] :
                                 env->regs[15];
            if (cpu_memory_rw_debug(cs, p, buf, sizeof(buf), 0) == 0) {
                fprintf(stderr, "[brain-uartwatch] reg%dbuf(0x%08x):",
                        w, (unsigned)p);
                for (i = 0; i < 32; i++) {
                    fprintf(stderr, " %08x", ldl_le_p(buf + i * 4));
                }
                fprintf(stderr, "\n");
            }
        }
        brain_duart_armed = false;
        brain_duart_winlen = 0;
    }
}

static const MemoryRegionOps brain_duart_shadow_ops = {
    .read = brain_duart_shadow_read,
    .write = brain_duart_shadow_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * Read 32 bits from a physical address and print them.  Used for
 * poking at the WinCE NK image at runtime to figure out where the
 * BSP keeps its state (e.g. `g_bPwmInit`, `g_BklData`,
 * `BSP_PRESENT_CH_MASK`) so we can patch it on the fly.
 */
void hmp_brain_pread(Monitor *mon, const QDict *qdict)
{
    hwaddr addr = qdict_get_int(qdict, "addr");

    if (!current_machine) {
        return;
    }
    uint8_t buf[256];
    MemTxResult r = address_space_read(&address_space_memory, addr,
                                       MEMTXATTRS_UNSPECIFIED, buf,
                                       sizeof(buf));
    if (r != MEMTX_OK) {
        monitor_printf(mon, "brain_pread: read failed at 0x%08x\n",
                       (unsigned)(addr & 0xffffffff));
        return;
    }
    monitor_printf(mon, "brain_pread 0x%08x:\n", (unsigned)(addr & 0xffffffff));
    for (int i = 0; i < 256; i += 16) {
        monitor_printf(mon, "  %08llx:",
                       (unsigned long long)(addr + i));
        for (int j = 0; j < 16; j++) {
            monitor_printf(mon, " %02x", buf[i + j]);
        }
        monitor_printf(mon, "  ");
        for (int j = 0; j < 16; j++) {
            unsigned char c = buf[i + j];
            monitor_printf(mon, "%c",
                           (c >= 0x20 && c < 0x7f) ? c : '.');
        }
        monitor_printf(mon, "\n");
    }
}

/*
 * brain_vread <va> [len] -- read guest *virtual* memory.
 *
 * The keybd_EDNA2 .rdata section is a WinCE CopySection: the guest
 * accesses it through VA 0xc087a000 while the physical backing is a
 * RAM copy allocated at module load time, NOT the XIP ROM location.
 * brain_pread() (physical) therefore reads the wrong bytes for such
 * addresses.  This command walks the guest MMU (using the ARM debug
 * translation hook) and dumps the physical page that the guest VA
 * actually resolves to, so we can inspect the runtime value of
 * variables like 0xc087a750 (SetDirectKey guard).
 */
void hmp_brain_vread(Monitor *mon, const QDict *qdict)
{
    vaddr va = qdict_get_int(qdict, "va");
    int len = qdict_get_try_int(qdict, "len", 64);
    BrainMachineState *bms = BRAIN_MACHINE(current_machine);
    CPUState *cs = CPU(bms->cpu);
    hwaddr pa;
    MemTxAttrs attrs = MEMTXATTRS_UNSPECIFIED;
    uint8_t buf[256];

    if (!current_machine) {
        return;
    }
    if (len > (int)sizeof(buf)) {
        len = sizeof(buf);
    }
    pa = arm_cpu_get_phys_page_attrs_debug(cs, va & ~(vaddr)0xfff, &attrs);
    if (pa == (hwaddr)-1) {
        monitor_printf(mon, "brain_vread: no translation for VA 0x%08lx\n",
                       (unsigned long)va);
        return;
    }
    pa += va & 0xfff;
    memset(buf, 0, sizeof(buf));
    address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
    monitor_printf(mon, "brain_vread VA 0x%08lx -> PA 0x%08llx:\n",
                   (unsigned long)va, (unsigned long long)pa);
    for (int i = 0; i < len; i += 16) {
        monitor_printf(mon, "  %08lx:",
                       (unsigned long)(va + i));
        for (int j = 0; j < 16 && i + j < len; j++) {
            monitor_printf(mon, " %02x", buf[i + j]);
        }
        monitor_printf(mon, "  ");
        for (int j = 0; j < 16 && i + j < len; j++) {
            unsigned char c = buf[i + j];
            monitor_printf(mon, "%c",
                           (c >= 0x20 && c < 0x7f) ? c : '.');
        }
        monitor_printf(mon, "\n");
    }
}

/*
 * Write 32 bits to a physical address.  Used to flip bits in the
 * WinCE NK's `.data` / `.bss` section at runtime (for instance to
 * set `g_bPwmInit = TRUE` once we have located the symbol, or to
 * rewrite the BSP's `BSP_PRESENT_CH_MASK` constant).
 */
void hmp_brain_pwrite(Monitor *mon, const QDict *qdict)
{
    hwaddr addr = qdict_get_int(qdict, "addr");
    uint64_t value = qdict_get_int(qdict, "value");
    unsigned size = qdict_get_try_int(qdict, "size", 4);
    uint8_t buf[8];

    if (!current_machine) {
        return;
    }
    if (size == 1) {
        buf[0] = value & 0xff;
    } else if (size == 2) {
        stl_le_p(buf, value & 0xffff);
    } else if (size == 4) {
        stl_le_p(buf, value & 0xffffffff);
    } else {
        monitor_printf(mon, "brain_pwrite: size must be 1, 2 or 4\n");
        return;
    }
    MemTxResult r = address_space_write(&address_space_memory, addr,
                                        MEMTXATTRS_UNSPECIFIED, buf, size);
    if (r != MEMTX_OK) {
        monitor_printf(mon, "brain_pwrite: write failed at 0x%08x\n",
                       (unsigned)(addr & 0xffffffff));
        return;
    }
    monitor_printf(mon, "brain_pwrite 0x%08x <- 0x%lx (size %u)\n",
                   (unsigned)(addr & 0xffffffff), (unsigned long)value,
                   (unsigned)size);
}

/*
 * Toggle live tracing of every MXS MMIO access (with guest PC) to
 * stderr -- the 'mxs_trace_live' switch consulted by MXS_TRACE_WRAP.
 */
void hmp_brain_trace(Monitor *mon, const QDict *qdict)
{
    const char *arg = qdict_get_try_str(qdict, "arg");

    if (!arg || !*arg) {
        monitor_printf(mon, "mxs live trace is %s\n",
                       mxs_trace_live ? "on" : "off");
        return;
    }
    mxs_trace_live = !strcmp(arg, "on") || !strcmp(arg, "1");
    monitor_printf(mon, "mxs live trace %s\n",
                   mxs_trace_live ? "ON" : "OFF");
}

/*
 * Variant limited to the EDNA2 coprocessor shared mailbox page
 * (battdrvr/keybd_EDNA2/edna2_powermgr traffic).  The mailbox trace is
 * a subset of the full mxs trace: either switch turns the logs on, this
 * one exists so suspend/wake analysis does not drown in timer traffic.
 */
static bool brain_mb_trace_live;

void hmp_brain_mbtrace(Monitor *mon, const QDict *qdict)
{
    const char *arg = qdict_get_try_str(qdict, "arg");

    if (!arg || !*arg) {
        monitor_printf(mon, "edna2 mailbox trace is %s\n",
                       brain_mb_trace_live ? "on" : "off");
        return;
    }
    brain_mb_trace_live = !strcmp(arg, "on") || !strcmp(arg, "1");
    monitor_printf(mon, "edna2 mailbox trace %s\n",
                   brain_mb_trace_live ? "ON" : "OFF");
}

/*
 * Dump the Brain bring-up runtime statistics (see include/brain_stats.h):
 * TB translation health, deferred-SCTLR quirk application sites,
 * exception mix, ICOLL acknowledgement matching and timer expiry.
 */
void hmp_brain_stats(Monitor *mon, const QDict *qdict)
{
#ifndef _WIN32
    char *buf = NULL;
    size_t len = 0;
    FILE *f = open_memstream(&buf, &len);

    if (!f) {
        monitor_printf(mon, "brain_stats: open_memstream failed\n");
        return;
    }
    brain_stats_dump(f);
    fclose(f);
    monitor_printf(mon, "%s", buf);
    g_free(buf);
#else
    g_autofree char *filename = NULL;
    int fd = g_file_open_tmp("brain_stats_XXXXXX", &filename, NULL);
    if (fd < 0) {
        monitor_printf(mon, "brain_stats: g_file_open_tmp failed\n");
        return;
    }
    FILE *f = fdopen(fd, "w");
    if (!f) {
        close(fd);
        monitor_printf(mon, "brain_stats: fdopen failed\n");
        return;
    }
    brain_stats_dump(f);
    fclose(f);

    g_autofree char *contents = NULL;
    if (g_file_get_contents(filename, &contents, NULL, NULL)) {
        monitor_printf(mon, "%s", contents);
    }
    g_unlink(filename);
#endif
}

/*
 * brain_pmemsave <addr> <len> "filename"
 *
 * Raw physical-memory dump side-channel for the Brain machine.
 * This fork's stock pmemsave is unreliable for our analysis flow
 * (gdb VA dumps can not see other processes' user pages on WinCE;
 * the HMP parser also requires quoted filename strings), so provide
 * a direct address_space_read -> FILE* pipe with hole-tolerant
 * zero fill.  128 MiB dumps in ~3 s.
 */
void hmp_brain_pmemsave(Monitor *mon, const QDict *qdict)
{
    uint64_t addr = qdict_get_int(qdict, "addr");
    uint64_t len = qdict_get_int(qdict, "size");
    const char *filename = qdict_get_try_str(qdict, "filename");
    FILE *fp;
    uint8_t *buf;
    uint64_t off = 0;
    const uint64_t chunk = 64 * 1024;

    if (!filename) {
        monitor_printf(mon, "brain_pmemsave: filename required\n");
        return;
    }
    fp = fopen(filename, "wb");
    if (!fp) {
        monitor_printf(mon, "brain_pmemsave: cannot open %s\n", filename);
        return;
    }
    buf = g_malloc(chunk);
    while (off < len) {
        uint64_t n = MIN(chunk, len - off);
        MemTxResult r;

        r = address_space_read(&address_space_memory, addr + off,
                               MEMTXATTRS_UNSPECIFIED, buf, n);
        if (r != MEMTX_OK) {
            memset(buf, 0, n);  /* unmapped hole: keep offsets stable */
        }
        if (fwrite(buf, 1, n, fp) != n) {
            monitor_printf(mon, "brain_pmemsave: write error\n");
            break;
        }
        off += n;
    }
    g_free(buf);
    fclose(fp);
    monitor_printf(mon, "brain_pmemsave: wrote 0x%llx bytes to %s\n",
                   (unsigned long long)off, filename);
}

/*
 * Dump the tail of the Brain event ring (VECTOR ack/LEVELACK/suppress/
 * quirk defer+apply/...).  Optional argument: number of entries (max 256).
 */
void hmp_brain_events(Monitor *mon, const QDict *qdict)
{
    unsigned last = qdict_get_try_int(qdict, "last", 32);
#ifndef _WIN32
    char *buf = NULL;
    size_t len = 0;
    FILE *f = open_memstream(&buf, &len);

    if (!f) {
        monitor_printf(mon, "brain_events: open_memstream failed\n");
        return;
    }
    brain_events_dump(f, last);
    fclose(f);
    monitor_printf(mon, "%s", buf);
    g_free(buf);
#else
    g_autofree char *filename = NULL;
    int fd = g_file_open_tmp("brain_events_XXXXXX", &filename, NULL);
    if (fd < 0) {
        monitor_printf(mon, "brain_events: g_file_open_tmp failed\n");
        return;
    }
    FILE *f = fdopen(fd, "w");
    if (!f) {
        close(fd);
        monitor_printf(mon, "brain_events: fdopen failed\n");
        return;
    }
    brain_events_dump(f, last);
    fclose(f);

    g_autofree char *contents = NULL;
    if (g_file_get_contents(filename, &contents, NULL, NULL)) {
        monitor_printf(mon, "%s", contents);
    }
    g_unlink(filename);
#endif
}

/*
 * Inject a touch event from the QEMU monitor (e.g. when QEMU
 * runs headless and the SDL mouse isn't available).  Coordinates
 * are 0..0x7fff in the same space the SHARP Brain BSP reads back
 * from LRADC plate channels 2 (X) / 3 (Y).  The event is delivered
 * through the QEMU input handler bus, so the LRADC's
 * `mxs_lradc_touch_event` callback updates the same touch state
 * that the SDL mouse would have.
 */
void hmp_brain_lcdfb(Monitor *mon, const QDict *qdict)
{
    BrainMachineState *bms = BRAIN_MACHINE(current_machine);
    const char *path = qdict_get_try_str(qdict, "path");

    if (!current_machine || !bms->lcdif) {
        monitor_printf(mon, "brain_lcdfb: no machine / no LCDIF\n");
        return;
    }
    if (!path || !*path) {
        monitor_printf(mon, "brain_lcdfb: path required\n");
        return;
    }
    {
        uint32_t base = qdict_get_try_int(qdict, "base", 0);
        uint32_t w = qdict_get_try_int(qdict, "width", 0);
        uint32_t h = qdict_get_try_int(qdict, "height", 0);
        int bpp = qdict_get_try_int(qdict, "bpp", 0);
        if (mxs_lcdif_dump_fb_opt(bms->lcdif, path, base, w, h, bpp) == 0) {
            monitor_printf(mon, "brain_lcdfb: wrote %s\n", path);
        } else {
            monitor_printf(mon,
                           "brain_lcdfb: no frame latched or write failed\n");
        }
    }
}

void hmp_brain_touch(Monitor *mon, const QDict *qdict)
{
    BrainMachineState *bms;
    int x = qdict_get_int(qdict, "x");
    int y = qdict_get_int(qdict, "y");
    int down = qdict_get_int(qdict, "down");

    if (!current_machine) {
        return;
    }
    bms = BRAIN_MACHINE(current_machine);
    InputEvent evt;
    InputMoveEvent move_x = { .axis = INPUT_AXIS_X, .value = x & 0x7fff };
    InputMoveEvent move_y = { .axis = INPUT_AXIS_Y, .value = y & 0x7fff };
    InputBtnEvent btn = { .button = INPUT_BUTTON_LEFT, .down = !!down };

    evt.type = INPUT_EVENT_KIND_ABS;
    evt.u.abs.data = &move_x;
    qemu_input_event_send_impl(NULL, &evt);

    evt.u.abs.data = &move_y;
    qemu_input_event_send_impl(NULL, &evt);

    evt.type = INPUT_EVENT_KIND_BTN;
    evt.u.btn.data = &btn;
    qemu_input_event_send_impl(NULL, &evt);

    /*
     * The headless machine has no active UI console, so the input
     * handler bus above can race with (or miss) console binding.
     * Drive the LRADC model state directly as well so that
     * brain_touch is deterministic regardless of the input layer.
     */
    if (bms->lradc) {
        mxs_lradc_set_touch(bms->lradc, x, y, !!down);
    }

    /*
     * The EDNA2 MCU watches the touch panel in parallel with the
     * host LRADC and pulses its attention line (ICOL 33) on panel
     * activity.  The host touch PDD powers itself down when idle
     * (serial: "LOG(1): OFF state") and only this MCU pulse wakes
     * it back up on the real device; mirror that here so touches
     * work in the OFF state too.
     */
    if (bms->kbd && down) {
        brain_kbd_edna2_pulse_ext(bms->kbd);
    }

    monitor_printf(mon, "brain_touch: x=%d y=%d down=%d\n", x, y, down);
}

/*
 * The i.MX28 on-chip ROM probes the microSD slot as a secondary
 * boot device when the eMMC path fails.  We re-implement that here
 * as a simple "look for a recognised file in the first FAT partition
 * and execute it as either an SB bootstream (for the Brainux
 * `u-boot.sb`) or a raw loader image (for `edsh6exe.bin` etc.)."
 */
static bool mxs_rom_try_sd(BrainMachineState *bms, SBRun *run)
{
    static const char *const brain_candidates[] = {
        "u-boot.sb", "edsh6exe.bin", "edsh5exe.bin", "edsh4exe.bin",
        "edsh3exe.bin", "edsh2exe.bin", "edsh1exe.bin",
        "edna3exe.bin", "edna2exe.bin", "edna1exe.bin",
        "ednhexebin", "nk.bin", NULL
    };
    g_autofree uint8_t *image = g_malloc(2 * MiB);
    int n;

    if (!bms->sd_blk) {
        return false;
    }
    n = mxs_fat_read_file(bms->sd_blk, brain_candidates, image, 2 * MiB);
    if (n < 0) {
        return false;
    }
    if (bms->verbose) {
        info_report("mxs-rom: SD payload %d bytes", n);
    }
    /* SB image? */
    if (n > 24 && memcmp(image + 20, "STMP", 4) == 0) {
        /* Re-use the SB loader by writing the image to the eMMC
         * BlockBackend in memory: mxs_rom_load_sb() reads from the
         * emmc backend, so we temporarily point it at the SD block and
         * load from offset 0. */
        BlockBackend *old = bms->emmc_blk;
        uint64_t sb_offset = 0;

        bms->emmc_blk = bms->sd_blk;
        bool ok = mxs_rom_load_sb(bms, sb_offset, run);
        bms->emmc_blk = old;
        return ok;
    }
    /* Raw loader (WinCE EBOOT): load to OCRAM at offset 0 and start
     * it directly. */
    address_space_write(&address_space_memory, MXS_OCRAM_BASE,
                        MEMTXATTRS_UNSPECIFIED, image, n);
    run->first_entry = MXS_OCRAM_BASE;
    run->last_entry = MXS_OCRAM_BASE;
    run->n_entries = 1;
    return true;
}

/*
 * EDNA2 coprocessor shared mailbox: on the real Brain, the EDNA2 MCU
 * shares status with the main CPU through a mailbox page in DRAM at
 * PA 0x400EA000.  The WinCE EDNA2 stack (keybd_EDNA2.dll, battdrvr.dll,
 * edna2_powermgr.dll) keys off the "mailbox ready" doorbell word at
 * offset 0x3c before publishing the status records (battery mV at
 * +0x7c, level at +0x80, the EDSH6 message ring at +0x110, ...).
 * Raise it on every system reset so the guest EDNA2 stack always
 * sees a present MCU, mirroring the real MCU coming out of reset
 * together with the application processor.  The guest consumes and
 * clears the word once handled.
 *
 * NOTE: this is a hardware-fidelity aid, NOT a suspend fix.  The
 * ~181 s PMWM_EDPOWERONOFF auto-power-off fires regardless of the
 * mailbox/battery content (verified 2026-08-07, see
 * docs/BRAIN_DEV_STATUS.md): it is the SHARP APO idle timer.
 */
#define BRAIN_EDNA2_MAILBOX_OFF    0xEA000
#define BRAIN_EDNA2_DOORBELL_OFF   (BRAIN_EDNA2_MAILBOX_OFF + 0x3c)

/*
 * ------------------------------------------------------------------
 * EDNA2 MCU command protocol (real-device model)
 *
 * The EDNA2 MCU is a separate microcontroller on the Brain PCB that
 * shares a mailbox page with the main CPU (PA 0x400EA000).  Beyond the
 * reset-time status block it executes a doorbell-driven command
 * protocol, decoded from the guest consumers (keybd_EDNA2.dll,
 * battdrvr.dll, lradc.dll) and the mailbox traffic of a DiagOS boot
 * (S11 report, wince repo):
 *
 *   +0xE8   command byte (guest writes, MCU reads)
 *   +0x3C   doorbell: guest writes 1 to kick the MCU
 *   +0xE8 bit0   done flag: MCU sets it when the command completes
 *   +0x404  touchkey result word: bit 0x10 = "calibration data
 *           valid / ready" flag tested by VMCopy.dll's touchkey
 *           reader (0xc05f2d48: tst r2, #0x10); the remaining
 *           low bits are per-key PRESSED state (0x2,0x4,0x8,0x20,
 *           0x40,0x100,0x200,0x400,0x800)
 *
 * Observed guest sequencing: the doorbell is written FIRST and the
 * command byte to +0xE8 right after, so the MCU latches the command
 * one poll period after the kick.  Command semantics (consumer-side
 * evidence): 0x01 and 0xD0..0xD5 are touchkey calibration commands
 * (longer processing); everything else is a plain status/handshake.
 *
 * Timing: an MCU command round-trip is milliseconds on real hardware;
 * we model 3 ms for a status command and 30 ms for a calibration
 * command of virtual time.  Guest timers keep advancing while the MCU
 * works.  This is the real-protocol model, replacing the old
 * dead-mailbox behaviour that made the touchkey calibration fail.
 * ------------------------------------------------------------------
 */
#define BRAIN_EDNA2_MCU_CMD_OFF       0xe8
#define BRAIN_EDNA2_MCU_TOUCHKEY_OFF  0x404
#define BRAIN_EDNA2_MCU_POLL_US       1000   /* mailbox poll period */
#define BRAIN_EDNA2_MCU_STATUS_US     3000   /* status command latency */
#define BRAIN_EDNA2_MCU_CALIB_US      30000  /* touchkey calibration */

static void brain_edna2_mcu_latch(BrainMachineState *bms);
static void brain_edna2_mcu_execute(BrainMachineState *bms);

static void brain_edna2_mcu_tick(void *opaque)
{
    BrainMachineState *bms = opaque;

    if (!bms->edna2_mcu_latched) {
        bms->edna2_mcu_latched = true;
        brain_edna2_mcu_latch(bms);
        return;
    }
    brain_edna2_mcu_execute(bms);
}

/* stage 2: latch the command byte one poll period after the kick and
 * arm the processing timer for the command's latency */
static void brain_edna2_mcu_latch(BrainMachineState *bms)
{
    uint32_t delay;

    bms->edna2_mcu_cmd = bms->edna2_mb[BRAIN_EDNA2_MCU_CMD_OFF];
    switch (bms->edna2_mcu_cmd) {
    case 0x01:
    case 0xd0: case 0xd1: case 0xd2:
    case 0xd3: case 0xd4: case 0xd5:
        delay = BRAIN_EDNA2_MCU_CALIB_US;
        break;
    default:
        delay = BRAIN_EDNA2_MCU_STATUS_US;
        break;
    }
    timer_mod(bms->edna2_mcu_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (uint64_t)delay * 1000);
    if (mxs_trace_live || brain_mb_trace_live) {
        fprintf(stderr, "[edna2-mcu] cmd 0x%02x latched, busy %u us\n",
                bms->edna2_mcu_cmd, delay);
    }
}

/* stage 3: command completed - post the done flag and the
 * command-specific results into the mailbox */

static void brain_edna2_mcu_execute(BrainMachineState *bms)
{
    bms->edna2_mcu_busy = false;
    bms->edna2_mcu_latched = false;

    bms->edna2_mb[BRAIN_EDNA2_MCU_CMD_OFF] |= 0x01;   /* done flag */

    /*
     * Mailbox protocol header (+0x00..+0x0c).
     *
     * The EDNA2 MCU API in k.coredll (snapshot 0xc004cdbc / wait
     * 0xc004ce4c, consumed by sx8650_touchscreen.dll's touchkey
     * calibration at 0xc07c2778) treats the first four words as a
     * range/state pair and requires (signed):
     *     mb[0x00] < mb[0x08]  AND  mb[0x04] < mb[0x0c]
     * otherwise the wait returns "failed" and the calibration aborts
     * ("touchkey_point_calibration() FAILED!" at every boot, because
     * the mailbox header was all zeros).
     *
     * The calibration then reads mb[0x0c] as two 16-bit touch-area
     * centre coordinates (low = x, high = y) and expands them by
     * +/-0x23 into the touch rectangle it hands back to the MCU
     * (0xc07c282c: ldrh/ldr/lsr + sub/add #0x23).  On the 800x480
     * panel the centre is (400, 240) = 0x00F00190.
     */
    stl_le_p(bms->edna2_mb + 0x00, 0x00000000u);
    stl_le_p(bms->edna2_mb + 0x04, 0x00000000u);
    stl_le_p(bms->edna2_mb + 0x08, 0x0000031Fu);
    stl_le_p(bms->edna2_mb + 0x0c, 0x00F00190u);

    /*
     * Touchkey calibration result block (mailbox +0x800..+0x824).
     *
     * VMCopy.dll's ioctl 0x80002093 handler (VMC_IOControl dispatch
     * 0xc05f2ca0 -> 0xc05f5ba8) scans 16-bit entries at +0x818..+0x822
     * and requires every one to be non-zero and not -1, then copies
     * 12 bytes from +0x818 into the caller's buffer.  The mailbox is
     * plain DRAM that nothing in the guest ever initialises, so the
     * entries were all zero and the calibration aborted
     * ("touchkey_point_calibration() FAILED!").  Seed six entries
     * with 0x0100 (1.0 in Q8.8) so the data looks calibrated; the
     * sx8650 driver only requires them non-zero.
     */
    stl_le_p(bms->edna2_mb + 0x818, 0x01000100u);
    stl_le_p(bms->edna2_mb + 0x81c, 0x01000100u);
    stl_le_p(bms->edna2_mb + 0x820, 0x01000100u);

    /*
     * k.coredll's MCU API keeps a 16-byte cache of this header at
     * VA 0xc0022afc (snapshot 0xc004cdbc / wait 0xc004ce4c read it;
     * nothing in the guest ever writes it back from the mailbox, so
     * it stays all-zero and the calibration's range check fails).
     * Mirror the header there through the CPU's page tables so the
     * guest sees the same values as the mailbox.
     */
    {
        MemTxAttrs attrs = {};
        hwaddr page = arm_cpu_get_phys_page_attrs_debug(
            CPU(bms->cpu), 0xc0022000, &attrs);

        if (page != (hwaddr)-1) {
            hwaddr g = page + (0xc0022afc & 0xfff);

            stl_le_phys(&address_space_memory, g + 0x00, 0x00000000u);
            stl_le_phys(&address_space_memory, g + 0x04, 0x00000000u);
            stl_le_phys(&address_space_memory, g + 0x08, 0x0000031Fu);
            stl_le_phys(&address_space_memory, g + 0x0c, 0x00F00190u);
        }
    }

    switch (bms->edna2_mcu_cmd) {
    case 0x01:
    case 0xd0: case 0xd1: case 0xd2:
    case 0xd3: case 0xd4: case 0xd5:
        /*
         * touchkey calibration: post the touchkey result word.
         *
         * VMCopy.dll touchkey reader (0xc05f2d24, verified against
         * repaired4):
         *   +0x400 must read 0x3037 (magic)
         *   +0x404 bit 0x10 (0x10) = "touchkey data valid/ready"
         *   +0x404 key bits (0x2,0x4,0x8,0x20,0x40,0x100,0x200,
         *   0x400,0x800) = per-key PRESSED state; a set bit makes the
         *   calibration treat the key as touched and fail
         *   (tst r3, r2 -> bne 0xc05f2dcc -> r4=0 -> fail).
         *
         * The previous seed 0x08000080 (bits 7+27) was a guess: bit
         * 0x10 was clear, so the driver always took the fail path and
         * touchkey_point_calibration() FAILED at every boot.  With no
         * keys touched the correct value is 0x00000010.
         */
        stl_le_p(bms->edna2_mb + BRAIN_EDNA2_MCU_TOUCHKEY_OFF,
                 0x00000010u | bms->edna2_touchkey);
        break;
    default:
        break;
    }

    if (mxs_trace_live || brain_mb_trace_live) {
        fprintf(stderr, "[edna2-mcu] cmd 0x%02x done at vnow=%" PRIu64
                " us (done-flag posted, +0x404=0x%08x)\n",
                bms->edna2_mcu_cmd,
                qemu_clock_get_us(QEMU_CLOCK_VIRTUAL),
                ldl_le_p(bms->edna2_mb + BRAIN_EDNA2_MCU_TOUCHKEY_OFF));
    }
}

/* stage 1: doorbell kick -> poll period -> latch (see header comment
 * for why the command byte is read a poll period later) */
static void brain_edna2_mcu_kick(BrainMachineState *bms)
{
    if (bms->edna2_mcu_busy) {
        /* the MCU is a single command-at-a-time machine */
        return;
    }
    bms->edna2_mcu_busy = true;
    timer_mod(bms->edna2_mcu_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (uint64_t)BRAIN_EDNA2_MCU_POLL_US * 1000);
    if (mxs_trace_live || brain_mb_trace_live) {
        fprintf(stderr, "[edna2-mcu] doorbell kick at vnow=%" PRIu64 " us\n",
                qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    }
}

/*
 * Debug aid: the EDNA2 mailbox page is plain DRAM, so the generic
 * MXS MMIO trace can not see who touches it.  For wedge/suspend
 * analysis we overlay a logged IO region on the mailbox page that
 * reports every access (guest PC included) while mxs_trace_live is
 * on.  The backing store is copied from / to the DRAM page so guest
 * behaviour is unchanged.  Release builds should drop the overlay.
 */
/*
 * Touch calibration affine for sx8650_touchscreen.dll.
 *
 * TouchPanelCalibrateAPoint (0xc07c2cc8) computes
 *     x' = (a*raw_x + b*raw_y + c) << 2 / d
 *     y' = (e*raw_x + f*raw_y + g) << 2 / d
 * over the int32 struct at VA 0xc07c71f4 with a valid flag at
 * +0x1c.  The GWES MDD recomputes this struct from the registry
 * CalibrationData, but on this image the recomputed values map the
 * real raw coordinates to points far outside the 800x480 screen
 * (verified in runs/tch11/tch12: raw (1924,1972) -> (1601,972)),
 * so the model posts the registry-consistent coefficients instead:
 * with a=200,b=0,c=-177600,e=0,f=120,g=-108720,d=2073 the factory
 * centre raw (1931,1961) maps to (400,240).
 */
/*
 * Touch linear calibration (Q8.8 int16[8]) at VA 0xc07c71c8, consumed by
 * the PDD fetch (0xc07c1c78 -> TouchPanelCalibrateAPoint 0xc07c23bc,
 * mode 0):
 *
 *     x' = cal[2] + cal[0]*raw_x/cal[1]   (clamped to [0, 799])
 *     y' = cal[5] + cal[3]*raw_y/cal[4]   (clamped to [0, 479])
 *
 * The registry CalibrationData maps raw X [888, 2961] over 800 px and
 * raw Y [906, 3039] over 480 px (nk_main.bin file 0x1be9f8d).  With the
 * ROM default identity coefficients raw 1924 -> x' = 1924 > 799, so the
 * fetch sets the pen-up flag (0xc07c1e84: flags |= 0x10) and every tap
 * is discarded.  Post the registry-consistent coefficients:
 * scale X = 800/2073 = 0.3859 (Q8.8 99), offset -888*0.3859 = -343;
 * scale Y = 480/2133 = 0.2250 (Q8.8 58), offset -906*0.2250 = -204.
 * Centre raw (1931,1961) then maps to (402, 240).
 */
#define BRAIN_TOUCH_CAL_VA   0xc07c71c8u
#define BRAIN_TOUCH_CAL_PAGE 0xc07c7000u
static void brain_inject_touch_cal(BrainMachineState *bms)
{
    static const uint16_t cal[8] = {
        0x0063, 0x0100, 0xfea9, 0x003a, 0x0100, 0xff34, 0x0000, 0xffff,
    };
    MemTxAttrs attrs = {};
    hwaddr page = arm_cpu_get_phys_page_attrs_debug(
        CPU(bms->cpu), BRAIN_TOUCH_CAL_PAGE, &attrs);
    int i;

    if (page == (hwaddr)-1) {
        return;
    }
    for (i = 0; i < 8; i++) {
        stw_le_phys(&address_space_memory,
                    page + (BRAIN_TOUCH_CAL_VA & 0xfff) + i * 2, cal[i]);
    }
}

/*
 * Touch-area valid flag (u32 at VA 0xc07c71f0, between the Q8.8 block
 * and the affine), read by the PDD's penup-delay delegation as
 * "calibration rect is valid" (0xc07c2564 -> [0xc07c71f0]).  It is
 * set to 1 by the driver's own calibration-apply path
 * (0xc07c2788: after IsRectEmpty(&rect at 0xc07c71d8) comes back
 * false), which only runs during a calibration-UI session
 * (0xc07c2574 is not called anywhere in the normal boot; verified in
 * runs/tch33 by breakpoint).  If the flag stays 0, every release of a
 * tap outside a registered gesture zone arms the gesture session
 * (0xc07c5630 -> [0xc07c726c]+8 = 1, 0xc07c5838) and the session can
 * never be cleared again: every subsequent fetch ORs the pen-up bit
 * into its flags (0xc07c1f10) and the IST (0xc07c3d00) drops the
 * sample before GWES ever sees a pen-down, so only the first tap
 * after boot reaches GWES.  With the flag set, the release path
 * clears the session (0xc07c585c/0xc07c5864) and consecutive taps
 * keep flowing: verified in runs/tch33 where GWES dispatch
 * (0xc01786e4) received 0x10000003 (pen down) and 5 (pen up) at
 * (401, 242) for repeated taps.  On the real device the flag is
 * left set from the factory calibration; post it together with the
 * coefficients below.
 */
#define BRAIN_TOUCH_AREA_VA   0xc07c71f0u
#define BRAIN_TOUCH_AREA_PAGE 0xc07c7000u
static void brain_inject_touch_area_flag(BrainMachineState *bms)
{
    MemTxAttrs attrs = {};
    hwaddr page = arm_cpu_get_phys_page_attrs_debug(
        CPU(bms->cpu), BRAIN_TOUCH_AREA_PAGE, &attrs);

    if (page == (hwaddr)-1) {
        return;
    }
    stl_le_phys(&address_space_memory,
                page + (BRAIN_TOUCH_AREA_VA & 0xfff), 1);
}

#define BRAIN_TOUCH_AFF_VA   0xc07c71f4u
#define BRAIN_TOUCH_AFF_PAGE 0xc07c7000u
static void brain_inject_touch_affine(BrainMachineState *bms)
{
    static const uint32_t aff[8] = {
        200, 0, (uint32_t)-177600, 0, 120,
        (uint32_t)-108720, 2073, 1,
    };
    MemTxAttrs attrs = {};
    hwaddr page = arm_cpu_get_phys_page_attrs_debug(
        CPU(bms->cpu), BRAIN_TOUCH_AFF_PAGE, &attrs);
    int i;

    if (page == (hwaddr)-1) {
        return;
    }
    for (i = 0; i < 8; i++) {
        stl_le_phys(&address_space_memory,
                    page + (BRAIN_TOUCH_AFF_VA & 0xfff) + i * 4, aff[i]);
    }
}

static uint64_t brain_edna2_mb_read(void *opaque, hwaddr offset, unsigned size)
{
    BrainMachineState *bms = opaque;

    /*
     * The OAL polls the mailbox every 100 ms from boot, which makes
     * this a reliable injection point for the touch affine (the GWES
     * MDD applies its own - wrong - coefficients once at boot).
     */
    brain_inject_touch_affine(bms);
    brain_inject_touch_cal(bms);
    brain_inject_touch_area_flag(bms);

    /*
     * The OAL polls the mailbox every 100 ms from boot, so this is a
     * reliable injection point for the touch calibration: the PDD's
     * TouchPanelEnable may rewrite its .data with the (missing)
     * registry values after the last MCU command, and the periodic
     * re-post here wins over any such overwrite.
     */
    uint64_t v = 0;

    memcpy(&v, bms->edna2_mb + offset, size);
    if (mxs_trace_live || brain_mb_trace_live) {
        fprintf(stderr, "[edna2-mb] R +0x%03x -> 0x%llx pc=0x%08x\n",
                (unsigned)offset, (unsigned long long)v,
                (unsigned)mxs_trace_guest_pc());
    }
    return v;
}

/*
 * EDNA2 MCU status block (+0x30 .. +0x73 of the shared mailbox).
 *
 * The WinCE OAL power monitor (OALIoCtl... 0x8020981c, traced in
 * docs/BRAIN_DEV_STATUS.md round 10) runs:
 *
 *     memcmp(EDNA2_mailbox + 0x30, zero_reference, 0x44)
 *
 * every timer tick.  When the 68-byte MCU status block is ALL ZERO the
 * OAL believes the MCU is dead/absent and calls the power function
 * (0x80211ae8) on every tick with IRQ/FIQ masked.  That function runs
 * the SRAM clock-switch sequence (OCRAM 0x3000) which parks the CPU in
 * WFI until the next timer interrupt, so the boot thread is starved and
 * the system never finishes booting before the ~181 s APO powers it off
 * (boot stall at serial line 219, "OALInitCpuHclkClock_change:
 * CLK_H=198MHz!").
 *
 * On real hardware the EDNA2 MCU posts this status block (battery /
 * charger status records) right after reset, so it is never all zero.
 * Emulate that: seed the block with a small non-zero pattern so the OAL
 * takes the normal "MCU present" path (idle WFI, no power-function
 * storm).  The guest itself overwrites parts of the window (observed:
 * +0x30/+0x38/+0x54/+0x74 get zeroed), but any surviving non-zero word
 * keeps the memcmp from matching; the write path below re-seeds only if
 * the whole window somehow becomes zero again.
 *
 * Verified by experiment (2026-08-09): poking +0x30..+0x73 with 1s at
 * stall time stops the power-function storm -- the CPU leaves the SRAM
 * WFI loop (0x327c) and idles at the normal OAL WFI (0x80212284).
 */
#define BRAIN_EDNA2_STATUS_OFF  (BRAIN_EDNA2_MAILBOX_OFF + 0x30)
#define BRAIN_EDNA2_STATUS_REL  0x30
#define BRAIN_EDNA2_STATUS_LEN  0x44

static void brain_edna2_seed_status(BrainMachineState *bms)
{
    uint8_t pat[BRAIN_EDNA2_STATUS_LEN];
    int i;

    if (!bms->aid_edna2_status) {
        return;
    }
    if (bms->aid_edna2_uninit) {
        /*
         * "Uninitialized" MCU status block (experimental fidelity aid).
         *
         * The WCEPRJ.EXE launcher (SJIS "\u3057\u3070\u3089\u304f\u304a\u5f85\u3061\u304f\u3060\u3055\u3044")
         * decides whether it must show the "please wait" screen by calling
         * 0x129c3c, which reads the EDNA2 MCU status block and treats
         * the unit as *initialized* only when buf[0] == 1, or when
         * buf[1] & 0x86 == 0.  The default all-ones seed makes buf[0]=1
         * so the launcher skips the wait screen and goes straight to
         * the "initialize?" dialog -- which is NOT what the broken real
         * unit does (it hangs on "please wait").
         *
         * With this aid enabled we post buf[0]=0 and buf[1]=0x02, i.e.
         * "not initialized", which makes 0x129c3c return 0 and the
         * launcher show the wait screen exactly like the real unit.
         * This is a hypothesis-driven aid: the exact byte pattern the
         * real EDNA2 MCU reports on a failed/never-initialized unit is
         * not yet captured from hardware.
         */
        memset(pat, 0, sizeof(pat));
        stl_le_p(pat + 1, 0x02);   /* buf[1] |= 0x02  (bit1 of 0x86 mask) */
    } else {
        for (i = 0; i < (int)sizeof(pat); i += 4) {
            stl_le_p(pat + i, 1);
        }
    }
    address_space_write(&address_space_memory,
                        MXS_DRAM_BASE + BRAIN_EDNA2_STATUS_OFF,
                        MEMTXATTRS_UNSPECIFIED, pat, sizeof(pat));
    if (mxs_trace_live || brain_mb_trace_live) {
        fprintf(stderr, "[edna2-mb] MCU status block seeded (+0x30, %u bytes)\n",
                (unsigned)sizeof(pat));
    }
}

static void brain_edna2_mb_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    BrainMachineState *bms = opaque;

    if (mxs_trace_live || brain_mb_trace_live) {
        fprintf(stderr, "[edna2-mb] W +0x%03x <- 0x%llx pc=0x%08x\n",
                (unsigned)offset, (unsigned long long)value,
                (unsigned)mxs_trace_guest_pc());
    }
    memcpy(bms->edna2_mb + offset, &value, size);

    /* Doorbell: the guest kicks the MCU by writing 1 to +0x3C. */
    if (offset == 0x3c && size == 4 && (value & 1)) {
        brain_edna2_mcu_kick(bms);
    }

    /*
     * Keep the MCU status block non-zero: if a guest write zeroes the
     * last non-zero word in the +0x30..+0x73 window, re-post the block
     * (a real MCU would keep its status current as well).  Disabled in
     * strict-HW mode: a real unit with a dead/absent MCU leaves the
     * window zero, which is exactly the boot-stall condition we want to
     * reproduce faithfully.
     */
    if (bms->aid_edna2_status &&
        offset < BRAIN_EDNA2_STATUS_REL + BRAIN_EDNA2_STATUS_LEN &&
        offset + size > BRAIN_EDNA2_STATUS_REL) {
        const uint8_t *p = bms->edna2_mb + BRAIN_EDNA2_STATUS_REL;
        bool any = false;
        int i;

        for (i = 0; i < BRAIN_EDNA2_STATUS_LEN; i++) {
            if (p[i]) {
                any = true;
                break;
            }
        }
        if (!any) {
            brain_edna2_seed_status(bms);
        }
    }

    /*
     * EDNA2 MCU response emulation (experimental, wedge analysis).
     *
     * On the real Brain the EDNA2 MCU services the shared mailbox and
     * acknowledges command/status writes.  In QEMU there is no MCU, so
     * guest drivers that wait for an MCU response would stall forever.
     * These handshakes are heuristic and not a faithful model of the
     * EDNA2 protocol, so they are OFF in strict-HW mode.
     */
    if (bms->aid_edna2_resp) {
        if (offset == 0x0e0 && size == 4 && value == 1) {
            uint32_t zero = 0;

            memcpy(bms->edna2_mb + 0x0e0, &zero, 4);
            if (mxs_trace_live || brain_mb_trace_live) {
                fprintf(stderr, "[edna2-mb] MCU resp: +0x0e0 cleared\n");
            }
        }
        if (offset == 0x1e0 && size == 4) {
            uint32_t ack = 1;

            memcpy(bms->edna2_mb + 0x1e4, &ack, 4);
            if (mxs_trace_live || brain_mb_trace_live) {
                fprintf(stderr, "[edna2-mb] MCU resp: +0x1e4 ack=1\n");
            }
        }
    }

    /*
     * Debug aid: if BRAIN_MBSTOP is set, halt the VM the moment any
     * guest code writes the mailbox.  The mailbox is only reachable
     * while a driver (udevice.exe) is active, so this freezes the VM in
     * the middle of driver execution -- letting us dump the driver DLL
     * code/data with the monitor before it gets swapped out again.
     * Analysis-only; remove for release.
     */
    if (getenv("BRAIN_MBSTOP")) {
        uint32_t pc = mxs_trace_guest_pc();

        /*
         * Only halt when a driver-DLL address (0xc0xxxxxx) writes the
         * mailbox: that is the moment a driver is actively executing, so
         * the VM freezes mid-driver and we can dump the driver DLL.
         * Kernel/OAL writes (0x80xxxxxx) are ignored so boot can proceed
         * to the suspend cascade without halting at every battery write.
         *
         * Idle-period writers are excluded so guest time keeps flowing
         * and the ~181 s APO fires in reasonable wall time:
         *   - battdrvr 0xc0660000-0xc0680000 (LRADC battery polling)
         *   - edna2_powermgr 0xc07d0000-0xc07f0000 (APO activity pulses)
         * The suspend-cascade actors that still halt:
         *   - EDNA2 0xc088xxxx, display 0xc06dxxxx, mailbox-reset 0xc05fxxxx
         */
        if ((pc & 0xff000000) == 0xc0000000 &&
            !(0xc0660000 <= pc && pc < 0xc0680000) &&
            !(0xc07d0000 <= pc && pc < 0xc07f0000)) {
            fprintf(stderr, "[edna2-mb] DRIVER wrote +0x%03x <- 0x%llx "
                    "pc=0x%08x -- halting for driver dump\n",
                    (unsigned)offset, (unsigned long long)value, pc);
            qemu_system_vmstop_request_prepare();
            qemu_system_vmstop_request(RUN_STATE_DEBUG);
        }
    }
}

static const MemoryRegionOps brain_edna2_mb_ops = {
    .read = brain_edna2_mb_read,
    .write = brain_edna2_mb_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void brain_edna2_raise_doorbell(void)
{
    uint32_t doorbell = 1;

    address_space_write(&address_space_memory,
                        MXS_DRAM_BASE + BRAIN_EDNA2_DOORBELL_OFF,
                        MEMTXATTRS_UNSPECIFIED,
                        (uint8_t *)&doorbell, sizeof(doorbell));
}


static void brain_cpu_reset(void *opaque)
{
    BrainMachineState *bms = opaque;
    CPUState *cs = CPU(bms->cpu);
    SBRun run = { 0 };
    uint32_t entry;

    cpu_reset(cs);

    bms->edna2_mcu_busy = false;
    bms->edna2_mcu_latched = false;
    timer_del(bms->edna2_mcu_timer);

    /*
     * The EDNA2 MCU is a separate, functional microcontroller on the
     * Brain PCB that comes out of reset together with the application
     * processor and immediately posts a mailbox doorbell (+0x3c) and a
     * non-zero battery/charger status block (+0x30..+0x73).  This is
     * real hardware behaviour (the MCU is alive even when the main OS is
     * wedged -- SD/diag boot proves it), not a QEMU compensation, so it
     * is always modelled.  The 'aid-edna2-status' switch only exists to
     * suppress it for controlled experiments.
     */
    memset(bms->edna2_mb, 0, sizeof(bms->edna2_mb));
    bms->edna2_touchkey = 0;
    if (bms->aid_edna2_status) {
        brain_edna2_raise_doorbell();
        brain_edna2_seed_status(bms);
    }

    /*
     * VMCopy.dll touchkey block (real EDNA2 MCU posts it at boot):
     *
     *   +0x400  0x3037: touchkey status magic.  VMCopy.dll's touchkey
     *          reader (0xc05f2d24, verified against repaired4) bails
     *          out unless the word equals 0x3037
     *          (ldr r3,[r1]; cmp r3,#0x3037).
     *   +0x404  bit 0x10 = touchkey data valid/ready; the low bits
     *          0x2,0x4,0x8,0x20,0x40,0x100,0x200,0x400,0x800 are the
     *          per-key pressed state (set = pressed).  No keys are
     *          pressed at boot, so the word is 0x10.
     */
    stl_le_p(bms->edna2_mb + 0x400, 0x00003037u);
    stl_le_p(bms->edna2_mb + BRAIN_EDNA2_MCU_TOUCHKEY_OFF,
             0x00000010u | bms->edna2_touchkey);

    /*
     * If we used -kernel, the ARM boot loader has already taken
     * care of the kernel image, the device tree and the ATAG list;
     * nothing more for the bootrom to do.
     */
    if (bms->boot_info.kernel_filename && bms->boot_info.kernel_filename[0]) {
        return;
    }

    if (!mxs_rom_find_and_load(bms, &run)) {
        if (!mxs_rom_try_sd(bms, &run)) {
            error_report("mxs-rom: failed to load a bootstream from the "
                         "eMMC or microSD; the CPU will stay at the reset "
                         "vector");
            return;
        }
    }

    if (bms->boot_mode && !strcmp(bms->boot_mode, "full")) {
        /*
         * Run the whole chain: start at the first CALL target (XLDR) with
         * LR pointing at a trampoline that continues with the last entry
         * (EBOOT).  The real ROM performs the intermediate LOADs between
         * the two calls, we have already done all of them.
         */
        uint8_t tramp[8];

        stl_le_p(tramp + 0, 0xe51ff004);        /* ldr pc, [pc, #-4] */
        stl_le_p(tramp + 4, run.last_entry);
        address_space_write(&address_space_memory, MXS_TRAMPOLINE_ADDR,
                            MEMTXATTRS_UNSPECIFIED, tramp, sizeof(tramp));
        cpu_set_pc(cs, run.first_entry);
        bms->cpu->env.regs[14] = MXS_TRAMPOLINE_ADDR;
        entry = run.first_entry;
    } else {
        /*
         * Default: skip XLDR.  Its only job is to bring up the DDR
         * controller, which QEMU does not need.
         */
        cpu_set_pc(cs, run.last_entry);
        entry = run.last_entry;
    }

    /* The ROM enters the image in ARM state, supervisor mode, IRQs off. */
    bms->cpu->env.thumb = 0;
    bms->cpu->env.regs[13] = MXS_OCRAM_BASE + MXS_OCRAM_SIZE - 0x100;

    info_report("mxs-rom: starting bootstream at 0x%08x", entry);

    /*
     * After EBOOT has set itself up, the SHARP Brain WinCE EBOOT
     * (or, on the buildbrain SD image, the equivalent init code in
     * the linux-loader SB image) scans the SD card for a launcher
     * file and runs it.  This is what the "Launch Linux" entry
     * does in the WinCE menu.  We re-implement that scan here so
     * that booting the SHARP Brain with both an eMMC image (the
     * stock WinCE install) and a buildbrain SD card gives the same
     * result as the real hardware: WinCE boots and then Linux
     * takes over.
     *
     * The trick is to only fire the interrupt when an SD card is
     * actually attached and the eMMC boot succeeded - otherwise
     * this would hijack the pure-WinCE boot on hosts that don't
     * have a microSD card at all.
     */
    /*
     * The "Launch Linux / SD launcher" EBOOT scan is a convenience aid
     * that re-implements EBOOT's FAT scan inside QEMU.  It is not part of
     * the i.MX28 boot ROM, so it is disabled in strict-HW mode.  The real
     * secondary-boot path (mxs_rom_try_sd) above is untouched: that models
     * the i.MX28 ROM's SD probe when the eMMC boot path fails.
     */
    if (bms->aid_sd_launcher && bms->sd_blk && bms->emmc_blk) {
        if (mxs_rom_try_sd_image(bms)) {
            info_report("mxs-rom: handed control over to the SD-card "
                        "launcher (the EBOOT equivalent)");
        }
    }
}

/* ------------------------------------------------------------------ */
/* machine                                                             */
/* ------------------------------------------------------------------ */

static DeviceState *mxs_create_simple(const char *type, hwaddr base,
                                      DeviceState *icoll, int irq)
{
    DeviceState *dev = qdev_new(type);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, base);
    if (icoll && irq >= 0) {
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(icoll, irq));
    }
    return dev;
}

static DeviceState *mxs_create_named(const char *type, const char *name,
                                     hwaddr base, uint64_t size)
{
    DeviceState *dev = qdev_new(type);

    qdev_prop_set_string(dev, "name", name);
    qdev_prop_set_uint64(dev, "size", size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return dev;
}

static DeviceState *mxs_create_named_irq(const char *type, const char *name,
                                         hwaddr base, uint64_t size,
                                         DeviceState *icoll, int irq)
{
    DeviceState *dev = mxs_create_named(type, name, base, size);

    if (icoll && irq >= 0) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           qdev_get_gpio_in(icoll, irq));
    }
    return dev;
}

static void mxs_create_usb(const char *name, hwaddr base,
                           DeviceState *icoll, int irq)
{
    DeviceState *dev = qdev_new(TYPE_MXS_USBCTRL);

    qdev_prop_set_string(dev, "name", name);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, qdev_get_gpio_in(icoll, irq));
}

static void mxs_create_dummy(const char *name, hwaddr base, uint64_t size)
{
    DeviceState *dev = qdev_new(TYPE_MXS_DUMMY);

    qdev_prop_set_string(dev, "name", name);
    qdev_prop_set_uint64(dev, "size", size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
}



static void brain_init(MachineState *machine)
{
    BrainMachineState *bms = BRAIN_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    DeviceState *icoll, *dev;
    DriveInfo *di;
    int i;

    /*
     * Apply the per-instance bus-error policy to the machine class.  The
     * core checks MACHINE_GET_CLASS(machine)->ignore_memory_transaction_failures
     * during address-space routing, so mirror the property here before any
     * CPU or device is realized.
     */
    MACHINE_GET_CLASS(machine)->ignore_memory_transaction_failures =
        bms->aid_ignore_bus_err;

    {
        Object *cpuobj = object_new(machine->cpu_type);
        /* Work around the WinCE OAL's MMU-toggle-in-SRAM sequence (see
         * ARM_CP_SUPPRESS_TB_EXIT in target/arm/cpregs.h). */
        object_property_set_bool(cpuobj, "mmu-prefetch-quirk", true,
                                 &error_fatal);
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
        bms->cpu = ARM_CPU(cpuobj);
    }

    /* on chip SRAM, the ROM loads the first stage here */
    memory_region_init_ram(&bms->ocram, NULL, "mxs.ocram", MXS_OCRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, MXS_OCRAM_BASE, &bms->ocram);

    /*
     * The SHARP Brain WinCE OAL runs its CPU clock-switch sequence
     * (OALInitCpuHclkClock_change) from the uncached alias 0xa8003000,
     * which the guest page table maps back to OCRAM (L1[A80]=0x402,
     * PA 0x00003000).  The sequence briefly disables the MMU (SCTLR M
     * off) while still executing in this alias window: the pipeline
     * has already decoded the next instruction, so real i.MX28
     * hardware keeps running straight-line code and even the branch
     * target of "mov pc, r2" is fetched with the pre-toggle state.
     * QEMU, however, applies the MMU toggle at the TB boundary, so the
     * branch target would be fetched with the MMU *off* -- as a plain
     * physical 0xa800xxxx address, which is unmapped.
     *
     * Mirror the hardware: give 0xa8000000 a physical alias onto the
     * OCRAM so that even an MMU-off fetch of 0xa8003000..0xa801ffff
     * lands in on-chip SRAM.  This is the uncached alias window the
     * OAL deliberately uses.
     */
    memory_region_init_alias(&bms->ocram_alias, NULL, "mxs.ocram-alias",
                             &bms->ocram, 0, MXS_OCRAM_SIZE);
    memory_region_add_subregion(sysmem, 0xa8000000, &bms->ocram_alias);

    /* DRAM */
    memory_region_add_subregion(sysmem, MXS_DRAM_BASE, machine->ram);

    /* EDNA2 mailbox page: overlay a logged IO region for wedge analysis
     * (see brain_edna2_mb_ops).  Only visible while mxs_trace_live is on. */
    memory_region_init_io(&bms->edna2_mb_iomem, NULL, &brain_edna2_mb_ops,
                          bms, "brain-edna2-mb", 0x1000);
    memory_region_add_subregion_overlap(sysmem,
                                        MXS_DRAM_BASE + BRAIN_EDNA2_MAILBOX_OFF,
                                        &bms->edna2_mb_iomem, 1);

    /* on chip boot ROM: we only need it to hold the return trampoline */
    memory_region_init_ram(&bms->rom, NULL, "mxs.rom", MXS_ROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, MXS_ROM_BASE, &bms->rom);

    /* interrupt collector */
    icoll = qdev_new(TYPE_MXS_ICOLL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(icoll), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(icoll), 0, MXS_ICOLL_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(icoll), 0,
                       qdev_get_gpio_in(DEVICE(bms->cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(icoll), 1,
                       qdev_get_gpio_in(DEVICE(bms->cpu), ARM_CPU_FIQ));
    bms->icoll = icoll;

    /* system control blocks */
    mxs_create_simple(TYPE_MXS_CLKCTRL, MXS_CLKCTRL_BASE, NULL, -1);
    mxs_create_named(TYPE_MXS_POWER, "power", MXS_POWER_BASE, 0x10000);
    mxs_create_simple(TYPE_MXS_DIGCTL, MXS_DIGCTL_BASE, icoll,
                      MXS_IRQ_DIGCTL);
    mxs_create_simple(TYPE_MXS_OCOTP, MXS_OCOTP_BASE, NULL, -1);
    mxs_create_simple(TYPE_MXS_RTC, MXS_RTC_BASE, icoll, MXS_IRQ_RTC_1MSEC);

    /* timers */
    dev = qdev_new(TYPE_MXS_TIMROT);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MXS_TIMROT_BASE);
    for (i = 0; i < 4; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i,
                           qdev_get_gpio_in(icoll, MXS_IRQ_TIMER0 + i));
    }

    /* pin controller / GPIO */
    dev = qdev_new(TYPE_MXS_PINCTRL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MXS_PINCTRL_BASE);
    for (i = 0; i < 5; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i,
                           qdev_get_gpio_in(icoll, MXS_IRQ_GPIO0 - i));
    }
    bms->pinctrl = dev;

    /* APBH DMA */
    dev = qdev_new(TYPE_MXS_APBH);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MXS_APBH_BASE);
    for (i = 0; i < 4; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i,
                           qdev_get_gpio_in(icoll, MXS_IRQ_APBH_DMA0 + i));
    }
    bms->apbh = dev;

    /* SSP controllers */
    for (i = 0; i < 4; i++) {
        static const hwaddr ssp_base[4] = {
            MXS_SSP0_BASE, MXS_SSP1_BASE, MXS_SSP2_BASE, MXS_SSP3_BASE,
        };

        dev = qdev_new(TYPE_MXS_SSP);
        /* BRAIN fault-zone experiment aid (mode 2 = virtual-time read
         * delay) lives on SSP0, the eMMC controller. */
        if (i == 0 && bms->exp_fault_len && bms->exp_fault_mode == 2) {
            qdev_prop_set_uint32(dev, "exp-fault-start",
                                 bms->exp_fault_start);
            qdev_prop_set_uint32(dev, "exp-fault-len", bms->exp_fault_len);
            qdev_prop_set_uint32(dev, "exp-fault-mode", bms->exp_fault_mode);
            qdev_prop_set_uint32(dev, "exp-fault-delay-us",
                                 bms->exp_fault_delay_us);
            warn_report("brain: exp fault delay: sec 0x%x len 0x%x "
                        "delay %u us", bms->exp_fault_start,
                        bms->exp_fault_len, bms->exp_fault_delay_us);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, ssp_base[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           qdev_get_gpio_in(icoll, MXS_IRQ_SSP0 + i));
        mxs_apbh_attach(bms->apbh, i, mxs_ssp_get_dma_ops(), dev);
        bms->ssp[i] = dev;
    }

    /* the eMMC lives on SSP0 */
    di = drive_get(IF_SD, 0, 0);
    if (di) {
        DeviceState *card = qdev_new(TYPE_EMMC);

        bms->emmc_blk = blk_by_legacy_dinfo(di);
        qdev_prop_set_drive_err(card, "drive", bms->emmc_blk, &error_fatal);
        /*
         * brain-region4-remap is a QEMU-only sector-window remap that makes
         * the FMD NAND driver's built-in default Region 4 start (0xf6800)
         * resolve to the real FAT32 partition (0x18c809) when EBOOT has not
         * supplied a corrected region table.  The real silicon has no such
         * remap -- on a unit whose EBOOT config is missing/corrupt the
         * driver reads garbage at 0xf6800 and the volume does not mount,
         * exactly the failure we reproduce in strict-HW mode.  The
         * interim-final eMMC already carries the corrected region table
         * inside NK, so the remap is a no-op there either way, but we
         * still gate it so a stock/iso image fails realistically.
         */
        if (bms->aid_region4_remap) {
            qdev_prop_set_bit(card, "brain-region4-remap", true);
        }
        /* BRAIN fault-zone experiment aid (verification-only) */
        if (bms->exp_fault_len && bms->exp_fault_mode) {
            qdev_prop_set_uint32(card, "exp-fault-start",
                                 bms->exp_fault_start);
            qdev_prop_set_uint32(card, "exp-fault-len", bms->exp_fault_len);
            qdev_prop_set_uint32(card, "exp-fault-mode", bms->exp_fault_mode);
            warn_report("brain: exp fault zone: sec 0x%x len 0x%x mode %u",
                        bms->exp_fault_start, bms->exp_fault_len,
                        bms->exp_fault_mode);
        }
        qdev_realize_and_unref(card,
                               qdev_get_child_bus(bms->ssp[0], "sd-bus"),
                               &error_fatal);
    } else {
        warn_report("no eMMC image given (use -drive if=sd,file=emmc.img)");
    }

    /* the microSD card lives on SSP1 */
    di = drive_get(IF_SD, 0, 1);
    if (di) {
        DeviceState *card = qdev_new(TYPE_SD_CARD);

        bms->sd_blk = blk_by_legacy_dinfo(di);
        qdev_prop_set_drive_err(card, "drive", bms->sd_blk, &error_fatal);
        qdev_realize_and_unref(card,
                               qdev_get_child_bus(bms->ssp[1], "sd-bus"),
                               &error_fatal);
    }

    /* display */
    dev = qdev_new(TYPE_MXS_LCDIF);
    qdev_prop_set_uint32(dev, "width", bms->lcd_width);
    qdev_prop_set_uint32(dev, "height", bms->lcd_height);
    qdev_prop_set_uint32(dev, "rotate", bms->lcd_rotate);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MXS_LCDIF_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(icoll, MXS_IRQ_LCDIF));
    bms->lcdif = dev;

    /* debug UART is a PL011 */
    /* debug UART is a PL011 */
    DeviceState *duart = pl011_create(MXS_DUART_BASE,
                    qdev_get_gpio_in(icoll, MXS_IRQ_DUART), serial_hd(0));

    /* analysis aid: shadow the DUART when BRAIN_UARTWATCH is set */
    if (getenv("BRAIN_UARTWATCH")) {
        brain_duart_armed = true;
        memory_region_init_io(&brain_duart_shadow, NULL,
                              &brain_duart_shadow_ops, SYS_BUS_DEVICE(duart),
                              "brain-duart-shadow", 0x1000);
        memory_region_add_subregion_overlap(sysmem, MXS_DUART_BASE,
                                            &brain_duart_shadow, 2);
    }

    /* application UARTs */
    for (i = 0; i < 5; i++) {
        dev = qdev_new(TYPE_MXS_AUART);
        qdev_prop_set_chr(dev, "chardev", serial_hd(i + 1));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MXS_AUART0_BASE + i * 0x2000);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           qdev_get_gpio_in(icoll, MXS_IRQ_AUART0 + i));
    }

    /* LRADC: battery monitoring and the resistive touch screen */
    dev = qdev_new(TYPE_MXS_LRADC);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    bms->lradc = dev;
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MXS_LRADC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(icoll, MXS_IRQ_LRADC_TOUCH));
    for (i = 0; i < 8; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i + 1,
                           qdev_get_gpio_in(icoll, MXS_IRQ_LRADC_CH0 + i));
    }

    /* keyboard matrix hanging off the GPIOs */
    dev = qdev_new(TYPE_BRAIN_KBD);
    object_property_set_link(OBJECT(dev), "pinctrl", OBJECT(bms->pinctrl),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    bms->kbd = dev;
    brain_kbd_set_touchkey_state(dev, &bms->edna2_touchkey,
                                 (uint32_t *)(bms->edna2_mb +
                                              BRAIN_EDNA2_MCU_TOUCHKEY_OFF));
    /*
     * EDNA2 MCU attention line (power key / wake / key press pulse).
     * The keybd_EDNA2 driver registers InterruptInitialize(33, ...):
     * the keyboard matrix IRQ on the real hardware is MXS GPIO-based
     * and ends up at ICOLL 33 (verified: 0xc08735fc r0=0x21, success
     * at 0xc0873600 r0=1).  ICOLL 63 is the APB interrupt (guest
     * self-pulses it via SOFTIRQ for other reasons), so the keyboard
     * pulse must go to 33, not 63.
     */
    qdev_connect_gpio_out_named(dev, "edna2-int", 0,
                                qdev_get_gpio_in(icoll, 33));

    /* Blocks we only need to swallow register accesses for. */
    mxs_create_dummy("hsadc", MXS_HSADC_BASE, 0x2000);
    mxs_create_dummy("perfmon", MXS_PERFMON_BASE, 0x2000);
    /*
     * BCH ECC engine (0x8000A000) and GPMI NAND controller
     * (0x8000C000).  Both hang off ICOLL input 41 (the i.MX28 DTS
     * 'nand-controller@8000c000' node lists interrupts = <41>).
     * The GPMI socket is empty by default -- the Brain PW-AJ2 has no
     * NAND die fitted -- so a latched command times out exactly like
     * the real board.  'gpmi-nand=true' attaches a full NAND device.
     */
    {
        DeviceState *d = qdev_new(TYPE_MXS_BCH);

        qdev_prop_set_string(d, "name", "bch");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, MXS_BCH_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(d), 0, qdev_get_gpio_in(icoll, 41));
    }
    {
        DeviceState *d = qdev_new(TYPE_MXS_GPMI);

        qdev_prop_set_string(d, "name", "gpmi");
        qdev_prop_set_bit(d, "nand", bms->gpmi_nand);
        if (bms->gpmi_nand_file && *bms->gpmi_nand_file) {
            qdev_prop_set_string(d, "nand-file", bms->gpmi_nand_file);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, MXS_GPMI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(d), 0, qdev_get_gpio_in(icoll, 41));
    }
    mxs_create_dummy("etm", MXS_ETM_BASE, 0x2000);
    /* APBX DMA: SAIF/SPDIF/AUART/USB peripheral channels */
    {
        static const int apbx_irq[16] = {
            78, 79, 66, 0, 80, 81, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77,
        };

        dev = qdev_new(TYPE_MXS_APBX);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MXS_APBX_BASE);
        for (i = 0; i < 16; i++) {
            if (apbx_irq[i]) {
                sysbus_connect_irq(SYS_BUS_DEVICE(dev), i,
                                   qdev_get_gpio_in(icoll, apbx_irq[i]));
            }
        }
        bms->apbx = dev;
    }
    mxs_create_dummy("dcp", MXS_DCP_BASE, 0x2000);
    mxs_create_simple(TYPE_MXS_PXP, MXS_PXP_BASE, icoll, MXS_IRQ_PXP);
    mxs_create_dummy("axi-ahb0", MXS_AXI_AHB0_BASE, 0x2000);
    mxs_create_dummy("can0", MXS_CAN0_BASE, 0x2000);
    mxs_create_dummy("can1", MXS_CAN1_BASE, 0x2000);
    mxs_create_dummy("simdbg", MXS_SIMDBG_BASE, 0x4000);
    /* SAIF FIFO/IRQ lines per the i.MX28 interrupt table */
    dev = mxs_create_named_irq(TYPE_MXS_SAIF, "saif0", MXS_SAIF0_BASE, 0x2000,
                               icoll, 59);
    mxs_apbx_attach(bms->apbx, 4, mxs_saif_get_dma_ops(), dev);
    dev = mxs_create_named_irq(TYPE_MXS_SAIF, "saif1", MXS_SAIF1_BASE, 0x2000,
                               icoll, 58);
    mxs_apbx_attach(bms->apbx, 5, mxs_saif_get_dma_ops(), dev);
    mxs_create_dummy("audioout", MXS_AUDIOOUT_BASE, 0x4000);
    mxs_create_dummy("audioin", MXS_AUDIOIN_BASE, 0x4000);
    mxs_create_dummy("spdif", MXS_SPDIF_BASE, 0x2000);
    mxs_create_named_irq(TYPE_MXS_I2C_REAL, "i2c0", MXS_I2C0_BASE, 0x2000,
                         icoll, MXS_IRQ_I2C0);
    mxs_create_named_irq(TYPE_MXS_I2C_REAL, "i2c1", MXS_I2C1_BASE, 0x2000,
                         icoll, MXS_IRQ_I2C1);
    mxs_create_named(TYPE_MXS_PWM, "pwm", MXS_PWM_BASE, 0x2000);
    mxs_create_named(TYPE_MXS_USBPHY, "usbphy0", MXS_USBPHY0_BASE, 0x2000);
    mxs_create_named(TYPE_MXS_USBPHY, "usbphy1", MXS_USBPHY1_BASE, 0x2000);
    mxs_create_usb("usbctrl0", MXS_USBCTRL0_BASE, icoll, MXS_IRQ_USB0);
    mxs_create_usb("usbctrl1", MXS_USBCTRL1_BASE, icoll, MXS_IRQ_USB1);
    mxs_create_dummy("dflpt", MXS_DFLPT_BASE, 0x20000);
    mxs_create_dummy("emi", MXS_EMI_BASE, 0x10000);
    mxs_create_dummy("enet", MXS_ENET_BASE, 0x10000);

    /*
     * If the user passed -kernel / -dtb / -initrd / -append we let
     * QEMU's standard ARM loader do the work: it writes the kernel
     * image at the right place, fills in the ATAG list, loads the
     * device tree, and arranges for the CPU to start executing it
     * after our reset hook.  This is the same path used by the
     * Versatile and Realview boards.
     */
    if (machine->kernel_filename) {
        bms->boot_info.ram_size = machine->ram_size;
        bms->boot_info.kernel_filename = machine->kernel_filename;
        bms->boot_info.kernel_cmdline = machine->kernel_cmdline;
        bms->boot_info.initrd_filename = machine->initrd_filename;
        bms->boot_info.dtb_filename = machine->dtb;
        bms->boot_info.board_id = 4153;       /* see mach-types */
        bms->boot_info.psci_conduit = QEMU_PSCI_CONDUIT_DISABLED;
        bms->boot_info.loader_start = 0;
        bms->boot_info.smp_loader_start = 0;
        bms->boot_info.smp_bootreg_addr = 0;
        bms->boot_info.gic_cpu_if_addr = 0;
        bms->boot_info.primary_cpu = bms->cpu;
        bms->boot_info.entry = MXS_OCRAM_BASE;
        arm_load_kernel(bms->cpu, machine, &bms->boot_info);
    }

    /*
     * Analysis aids that must be armed before the guest boots (the
     * monitor is not usable that early):
     *   BRAIN_BWATCH=<va>[:<count>][,<va>[:<count>]...]  -- execution
     *      breakpoints (count>1 auto-resumes; e.g.
     *      BRAIN_BWATCH=0xc08735fc:40,0xc0878b10:40)
     *   BRAIN_WATCH=<va>:<len>[,<va>:<len>...] -- data watchpoints,
     *      BP_MEM_ACCESS only (BP_STOP_BEFORE_ACCESS halts the VM at
     *      boot; with plain BP_MEM_ACCESS the first hit records and
     *      the debug handler dumps once, then removes itself so boot
     *      continues)
     */
    {
        const char *bw = getenv("BRAIN_BWATCH");
        const char *ww = getenv("BRAIN_WATCH");

        if (bw && *bw) {
            char *dup = g_strdup(bw);
            char *save = NULL;
            char *tok = strtok_r(dup, ",", &save);

            while (tok) {
                char *colon = strchr(tok, ':');
                unsigned long va;
                int count = 1;

                if (colon) {
                    *colon = '\0';
                    count = atoi(colon + 1);
                }
                va = strtoul(tok, NULL, 0);
                brain_bwatch_arm(NULL, CPU(bms->cpu), va, count);
                fprintf(stderr, "[brain-bwatch] env %s count=%d\n",
                        tok, count);
                tok = strtok_r(NULL, ",", &save);
            }
            g_free(dup);
        }
        if (ww && *ww) {
            char *dup = g_strdup(ww);
            char *save = NULL;
            char *tok = strtok_r(dup, ",", &save);

            while (tok) {
                char *colon = strchr(tok, ':');
                CPUWatchpoint *wp = NULL;
                unsigned long va, len = 4;

                if (colon) {
                    *colon = '\0';
                    len = strtoul(colon + 1, NULL, 0);
                }
                va = strtoul(tok, NULL, 0);
                brain_chain_debug_handler(CPU(bms->cpu));
                cpu_watchpoint_insert(CPU(bms->cpu), va, len,
                                      BP_MEM_ACCESS, &wp);
                fprintf(stderr, "[brain-watch] env %s len=%lu\n", tok, len);
                tok = strtok_r(NULL, ",", &save);
            }
            g_free(dup);
        }
    }

    qemu_register_reset(brain_cpu_reset, bms);
}

static char *brain_get_boot_mode(Object *obj, Error **errp)
{
    BrainMachineState *bms = BRAIN_MACHINE(obj);

    return g_strdup(bms->boot_mode ? bms->boot_mode : "eboot");
}

static void brain_set_boot_mode(Object *obj, const char *value, Error **errp)
{
    BrainMachineState *bms = BRAIN_MACHINE(obj);

    if (strcmp(value, "eboot") && strcmp(value, "full")) {
        error_setg(errp, "boot-mode must be 'eboot' or 'full'");
        return;
    }
    g_free(bms->boot_mode);
    bms->boot_mode = g_strdup(value);
}

static bool brain_get_verbose(Object *obj, Error **errp)
{
    return BRAIN_MACHINE(obj)->verbose;
}

static void brain_set_verbose(Object *obj, bool value, Error **errp)
{
    BRAIN_MACHINE(obj)->verbose = value;
}

static bool brain_get_gpmi_nand(Object *obj, Error **errp)
{
    return BRAIN_MACHINE(obj)->gpmi_nand;
}

static void brain_set_gpmi_nand(Object *obj, bool value, Error **errp)
{
    BRAIN_MACHINE(obj)->gpmi_nand = value;
}

static char *brain_get_gpmi_nand_file(Object *obj, Error **errp)
{
    return BRAIN_MACHINE(obj)->gpmi_nand_file;
}

static void brain_set_gpmi_nand_file(Object *obj, const char *value,
                                     Error **errp)
{
    BrainMachineState *bms = BRAIN_MACHINE(obj);

    g_free(bms->gpmi_nand_file);
    bms->gpmi_nand_file = g_strdup(value);
}

/*
 * strict-hw master switch.  When on (default) every QEMU-only guest aid is
 * disabled so the emulation tracks real silicon failures.  Setting it off
 * re-enables every aid at once (the historic lenient behaviour).
 */
static bool brain_get_strict_hw(Object *obj, Error **errp)
{
    return BRAIN_MACHINE(obj)->strict_hw;
}

static void brain_set_strict_hw(Object *obj, bool value, Error **errp)
{
    BrainMachineState *bms = BRAIN_MACHINE(obj);

    bms->strict_hw = value;
    if (!value) {
        /* Re-enable the QEMU-only guest aids.  aid_edna2_status is left on
         * unconditionally because it models real MCU behaviour. */
        bms->aid_edna2_resp = true;
        bms->aid_region4_remap = true;
        bms->aid_sd_launcher = true;
        bms->aid_ignore_bus_err = true;
    }
}

/* Typed getters/setters so object_property_add_bool gets the right function
 * signatures; one per guest-aid switch. */
#define BRAIN_AID_ACCESSOR(pid, field)                                     \
    static bool brain_get_aid_##pid(Object *obj, Error **errp)             \
    { return BRAIN_MACHINE(obj)->field; }                                  \
    static void brain_set_aid_##pid(Object *obj, bool v, Error **errp)     \
    { BRAIN_MACHINE(obj)->field = v; }

BRAIN_AID_ACCESSOR(edna2_status,  aid_edna2_status)
BRAIN_AID_ACCESSOR(edna2_uninit,  aid_edna2_uninit)
BRAIN_AID_ACCESSOR(edna2_resp,    aid_edna2_resp)
BRAIN_AID_ACCESSOR(region4_remap, aid_region4_remap)
BRAIN_AID_ACCESSOR(sd_launcher,   aid_sd_launcher)
BRAIN_AID_ACCESSOR(ignore_bus_err,aid_ignore_bus_err)

#define BRAIN_EXP_FAULT_ACCESSOR(pid, field)                               \
    static void brain_get_exp_##pid(Object *obj, Visitor *v,              \
                                    const char *name, void *opaque,        \
                                    Error **errp)                          \
    { visit_type_uint32(v, name, &BRAIN_MACHINE(obj)->field, errp); }      \
    static void brain_set_exp_##pid(Object *obj, Visitor *v,              \
                                    const char *name, void *opaque,        \
                                    Error **errp)                          \
    { visit_type_uint32(v, name, &BRAIN_MACHINE(obj)->field, errp); }

BRAIN_EXP_FAULT_ACCESSOR(fault_start, exp_fault_start)
BRAIN_EXP_FAULT_ACCESSOR(fault_len,   exp_fault_len)
BRAIN_EXP_FAULT_ACCESSOR(fault_mode,  exp_fault_mode)
BRAIN_EXP_FAULT_ACCESSOR(fault_delay_us, exp_fault_delay_us)

static void brain_add_aid_prop(Object *obj, const char *name,
                              bool (*get)(Object *, Error **),
                              void (*set)(Object *, bool, Error **),
                              const char *desc)
{
    object_property_add_bool(obj, name, get, set);
    object_property_set_description(obj, name, desc);
}

static void brain_instance_init(Object *obj)
{
    BrainMachineState *bms = BRAIN_MACHINE(obj);

    bms->boot_mode = g_strdup("eboot");
    /*
     * The panel is a 480x854 portrait MIPI/MPU module mounted sideways in
     * the clamshell, so the WinCE display driver scans out a 480 wide,
     * 854 tall framebuffer that has to be turned counter clockwise to give
     * the 854x480 landscape picture the user sees.
     */
    bms->lcd_width = 480;
    bms->lcd_height = 854;
    bms->lcd_rotate = 270;
    bms->verbose = true;

    bms->edna2_mcu_busy = false;
    bms->edna2_mcu_latched = false;
    bms->edna2_mcu_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        brain_edna2_mcu_tick, bms);

    /*
     * Strict-hardware fidelity is the DEFAULT: synthetic guest aids that
     * paper over a real failure (Region-4 sector remap, EBOOT-equivalent
     * SD auto-launch, heuristic EDNA2 MCU responses, bus-error
     * suppression) are off, so the guest behaves exactly like the real
     * Brain.  The EDNA2 mailbox doorbell/status seeding stays ON because
     * it models a real, functional MCU (the MCU powers the keyboard,
     * battery and touch-keys even when the main OS is wedged).  Each aid
     * can be toggled through its object property; 'strict-hw=off' turns
     * the four QEMU aids on at once for the historic lenient behaviour.
     */
    bms->strict_hw = true;
    /*
     * The EDNA2 MCU status-block + doorbell seeding (aid-edna2-status)
     * is OFF by default.  Empirically (2026-08-26, A/B on emmc_repaired4)
     * this QEMU-side injection is read by the main OS boot control
     * (WCEPRJ/EdAppCtrl) as "MCU initialized/present" and routes the
     * guest into the DiagApp factory-Selftest path instead of the normal
     * main OS first-run setup (the yellow date/time dialog).  With it off
     * the main OS boots to the date/time setup and is keyboard-interactive.
     * 'aid-edna2-status=on' reproduces the historic diagnostic-menu
     * behaviour for comparison.
     */
    bms->aid_edna2_status = false;    /* QEMU-only injection, off by default */
    bms->aid_edna2_uninit = false;    /* hypothesis aid, off by default */
    bms->aid_edna2_resp = false;     /* heuristic, off by default */
    bms->aid_region4_remap = false;  /* QEMU-only sector remap */
    bms->aid_sd_launcher = false;    /* EBOOT-equivalent auto-boot */
    bms->aid_ignore_bus_err = false; /* report real bus aborts */
    bms->exp_fault_start = 0;        /* fault-zone experiment aid: off */
    bms->exp_fault_len = 0;
    bms->exp_fault_mode = 0;         /* 0=off 1=error 2=delay 3=trace */
    bms->exp_fault_delay_us = 500000; /* 500 ms virtual-time delay */

    object_property_add_str(obj, "boot-mode", brain_get_boot_mode,
                            brain_set_boot_mode);
    object_property_set_description(obj, "boot-mode",
        "'eboot' (default, jump straight into EBOOT) or "
        "'full' (also run the XLDR DDR init)");
    object_property_add_bool(obj, "rom-verbose", brain_get_verbose,
                             brain_set_verbose);

    object_property_add_bool(obj, "strict-hw", brain_get_strict_hw,
                             brain_set_strict_hw);
    object_property_set_description(obj, "strict-hw",
        "Faithful hardware mode (default on): disable every QEMU-only "
        "guest aid so the system fails exactly like the real broken "
        "Brain.  Turn off to re-enable all aids at once.");

    object_property_add_bool(obj, "gpmi-nand", brain_get_gpmi_nand,
                             brain_set_gpmi_nand);
    object_property_set_description(obj, "gpmi-nand",
        "Attach a NAND device to the GPMI controller (default off: "
        "the Brain socket is unpopulated, an empty-socket timeout is "
        "the real hardware behaviour)");
    object_property_add_str(obj, "gpmi-nand-file", brain_get_gpmi_nand_file,
                            brain_set_gpmi_nand_file);
    object_property_set_description(obj, "gpmi-nand-file",
        "Raw image backing the GPMI NAND media (needs gpmi-nand=true)");

    brain_add_aid_prop(obj, "aid-edna2-status",
                       brain_get_aid_edna2_status,
                       brain_set_aid_edna2_status,
                       "Seed the EDNA2 mailbox MCU status block on reset "
                       "(off in strict-HW mode)");
    brain_add_aid_prop(obj, "aid-edna2-uninit",
                       brain_get_aid_edna2_uninit,
                       brain_set_aid_edna2_uninit,
                       "Seed the EDNA2 MCU status block as 'not "
                       "initialized' (buf[0]=0, buf[1]&0x86!=0) so the "
                       "WCEPRJ launcher shows the real unit's "
                       "'please wait' screen (hypothesis aid, off by "
                       "default)");
    brain_add_aid_prop(obj, "aid-edna2-resp",
                       brain_get_aid_edna2_resp,
                       brain_set_aid_edna2_resp,
                       "Emulate EDNA2 MCU command-response handshakes "
                       "(heuristic; off in strict-HW mode)");
    brain_add_aid_prop(obj, "aid-region4-remap",
                       brain_get_aid_region4_remap,
                       brain_set_aid_region4_remap,
                       "Remap FMD Region 4 window 0xf6800 -> real FAT32 "
                       "partition (off in strict-HW mode)");
    brain_add_aid_prop(obj, "aid-sd-launcher",
                       brain_get_aid_sd_launcher,
                       brain_set_aid_sd_launcher,
                       "After eMMC boot, scan the SD card for a launcher "
                       "and run it (EBOOT equivalent; off in strict-HW "
                       "mode)");
    brain_add_aid_prop(obj, "aid-ignore-bus-err",
                       brain_get_aid_ignore_bus_err,
                       brain_set_aid_ignore_bus_err,
                       "Ignore unmapped/aborted memory transactions "
                       "(off in strict-HW mode)");

    object_property_add(obj, "exp-fault-start", "uint32",
                        brain_get_exp_fault_start,
                        brain_set_exp_fault_start, NULL, NULL);
    object_property_set_description(obj, "exp-fault-start",
        "BRAIN fault-zone experiment aid: first sector of the injected "
        "zone (verification-only, off when len=0/mode=0)");
    object_property_add(obj, "exp-fault-len", "uint32",
                        brain_get_exp_fault_len,
                        brain_set_exp_fault_len, NULL, NULL);
    object_property_set_description(obj, "exp-fault-len",
        "BRAIN fault-zone experiment aid: zone length in sectors");
    object_property_add(obj, "exp-fault-mode", "uint32",
                        brain_get_exp_fault_mode,
                        brain_set_exp_fault_mode, NULL, NULL);
    object_property_set_description(obj, "exp-fault-mode",
        "BRAIN fault-zone experiment aid: 0=off 1=read-error 2=read-delay "
        "3=trace-only");
    object_property_add(obj, "exp-fault-delay-us", "uint32",
                        brain_get_exp_fault_delay_us,
                        brain_set_exp_fault_delay_us, NULL, NULL);
    object_property_set_description(obj, "exp-fault-delay-us",
        "BRAIN fault-zone experiment aid: virtual-time delay per zone "
        "read in mode 2");

    object_property_add_uint32_ptr(obj, "lcd-width", &bms->lcd_width,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "lcd-height", &bms->lcd_height,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "lcd-rotate", &bms->lcd_rotate,
                                   OBJ_PROP_FLAG_READWRITE);
}

static void brain_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SHARP Brain (Freescale i.MX28, ARM926EJ-S)";
    mc->init = brain_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "mxs.dram";
    /*
     * In strict-HW mode (the default) we let unmapped / aborted memory
     * transactions surface as real prefetch/data aborts, the way the
     * i.MX28 does.  The historic QEMU default of ignoring them masked
     * guest bus errors and let a broken boot proceed further than real
     * hardware ever would, which was one of the fidelity gaps reported by
     * the user.  The 'aid-ignore-bus-err' machine property can re-enable
     * the lenient behaviour for analysis (the class field is global, so
     * it is applied from brain_instance_init based on the default).
     */
    mc->ignore_memory_transaction_failures = false;
    mc->min_cpus = 1;
    mc->max_cpus = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
}

static const TypeInfo brain_machine_types[] = {
    {
        .name           = TYPE_BRAIN_MACHINE,
        .parent         = TYPE_MACHINE,
        .instance_size  = sizeof(BrainMachineState),
        .instance_init  = brain_instance_init,
        .class_init     = brain_machine_class_init,
        .interfaces     = arm_machine_interfaces,
    },
};

DEFINE_TYPES(brain_machine_types)

