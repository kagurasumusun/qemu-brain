/* forward prototype */
int mxs_fat_read_file(BlockBackend *blk, const char *const *candidates,
                      uint8_t *out, uint32_t max_bytes);

/*
 * Tiny FAT12/16/32 reader used by the Brain machine's bootrom to
 * pick up the SD-card payload (a Brainux `u-boot.sb` bootstream or
 * a raw `edxxNexe.bin` Windows CE loader).
 *
 * The reader is intentionally minimal: it knows about the MBR, the
 * (single) FAT partition, the root directory, and a flat list of
 * 8.3 entries.  Long file names, subdirectories and multi-FAT
 * layouts are not implemented because the boot partition of the
 * Brainux SD image is a single FAT16/32 with a flat boot directory.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"
#include "system/block-backend.h"

/* Read a sector from the block backend into @buf (which must be
 * 512 bytes long). */
static int mxs_fat_read_sector(BlockBackend *blk, uint64_t lba,
                               void *buf, size_t buf_size)
{
    if (buf_size < 512) {
        return -1;
    }
    return blk_pread(blk, lba * 512, 512, buf, 0) < 0 ? -1 : 0;
}

typedef struct {
    BlockBackend *blk;

    /* geometry (from the MBR partition entry) */
    uint64_t part_lba;
    uint64_t part_sectors;

    /* FAT BPB */
    uint16_t byts_per_sec;
    uint8_t  sec_per_clus;
    uint16_t rsvd_sec;
    uint8_t  num_fats;
    uint16_t root_entries;
    uint16_t tot_sec16;
    uint32_t tot_sec32;
    uint16_t fats_sec16;
    uint32_t fats_sec32;
    uint32_t root_clus;

    /* derived */
    uint32_t fat_start_lba;
    uint32_t root_start_lba;
    uint32_t data_start_lba;
    uint32_t total_clusters;

    bool is_fat32;
} MxsFAT;

static int mxs_fat_parse_bpb(MxsFAT *f, const uint8_t *sec)
{
    f->byts_per_sec = lduw_le_p(sec + 11);
    f->sec_per_clus = sec[13];
    f->rsvd_sec     = lduw_le_p(sec + 14);
    f->num_fats     = sec[16];
    f->root_entries = lduw_le_p(sec + 17);
    f->tot_sec16    = lduw_le_p(sec + 19);
    f->tot_sec32    = ldl_le_p(sec + 32);
    f->fats_sec16   = lduw_le_p(sec + 22);
    f->fats_sec32   = ldl_le_p(sec + 36);
    f->root_clus    = ldl_le_p(sec + 44);

    if (f->byts_per_sec != 512) {
        /* the Brainux SD image uses 512-byte sectors; anything else
         * means we are looking at a filesystem we don't know how to
         * read. */
        return -1;
    }
    if (f->sec_per_clus == 0) {
        return -1;
    }

    uint64_t tot = f->tot_sec16 ? f->tot_sec16 : f->tot_sec32;
    uint32_t fat_size = f->fats_sec16 ? f->fats_sec16 : f->fats_sec32;
    uint32_t root_dir_secs = (f->root_entries * 32 + 511) / 512;
    uint64_t data_secs = tot - (f->rsvd_sec + f->num_fats * fat_size +
                                root_dir_secs);
    uint32_t csize = f->sec_per_clus;
    if (csize == 0) {
        return -1;
    }
    f->total_clusters = data_secs / csize;
    f->is_fat32 = f->total_clusters >= 65525;
    if (!f->is_fat32) {
        f->root_clus = 0;   /* FAT12/16: root is contiguous */
    }

    f->fat_start_lba  = f->part_lba + f->rsvd_sec;
    f->root_start_lba = f->fat_start_lba + f->num_fats * fat_size;
    f->data_start_lba = f->root_start_lba + root_dir_secs;

    return 0;
}

static int mxs_fat_open(MxsFAT *f, BlockBackend *blk, uint64_t part_lba,
                        uint64_t part_sectors)
{
    uint8_t sec[512];
    unsigned i;
    uint8_t type;

    f->blk = blk;
    f->part_lba = part_lba;
    f->part_sectors = part_sectors;

    if (mxs_fat_read_sector(blk, part_lba, sec, sizeof(sec)) < 0) {
        return -1;
    }

    /* Look for the first FAT partition in the MBR.  The Brainux image
     * uses a plain MBR with one primary FAT partition. */
    if (sec[510] != 0x55 || sec[511] != 0xaa) {
        return -1;
    }
    type = 0;
    for (i = 0; i < 4; i++) {
        const uint8_t *e = sec + 446 + i * 16;

        type = e[4];
        if (type == 0x04 || type == 0x06 || type == 0x07 ||
            type == 0x0b || type == 0x0c || type == 0x0e ||
            type == 0x14 || type == 0x16 || type == 0x17) {
            uint32_t lba = ldl_le_p(e + 8);
            uint32_t sz = ldl_le_p(e + 12);

            f->part_lba = lba;
            f->part_sectors = sz;
            if (mxs_fat_read_sector(blk, lba, sec, sizeof(sec)) < 0) {
                return -1;
            }
            goto bp_start;
        }
    }
    /* No recognised FAT partition; try the volume as if it were a
     * superfloppy. */
    if (mxs_fat_read_sector(blk, part_lba, sec, sizeof(sec)) < 0) {
        return -1;
    }

bp_start:
    /* BPB starts with 0xEB 0x?? 0x90 (FAT32/FAT16) or 0xE9 (FAT12/16) */
    if (sec[0] != 0xeb && sec[0] != 0xe9) {
        return -1;
    }
    return mxs_fat_parse_bpb(f, sec);
}

static uint32_t mxs_fat_get_entry(MxsFAT *f, uint32_t cluster)
{
    uint8_t sec[512];
    uint32_t lba;
    uint32_t offset;
    uint32_t fat32_mask = 0x0fffffffu;

    if (f->is_fat32) {
        lba = f->fat_start_lba + (cluster * 4) / 512;
        offset = (cluster * 4) & 511;
    } else {
        lba = f->fat_start_lba + (cluster * 3) / (512 * 2);
        offset = (cluster * 3) & 1023;
    }
    if (mxs_fat_read_sector(f->blk, lba, sec, sizeof(sec)) < 0) {
        return 0;
    }

    if (f->is_fat32) {
        return ldl_le_p(sec + offset) & fat32_mask;
    } else {
        if (offset == 510) {
            uint16_t lo = lduw_le_p(sec + 510);
            uint16_t hi;
            uint8_t next[512];

            if (mxs_fat_read_sector(f->blk, lba + 1, next, sizeof(next)) < 0) {
                return 0;
            }
            hi = lduw_le_p(next);
            return ((hi << 16) | lo) & 0x0fffffffu;
        }
        return lduw_le_p(sec + offset);
    }
}

static int mxs_fat_read_cluster(MxsFAT *f, uint32_t cluster, uint8_t *buf,
                                uint32_t max_bytes)
{
    uint32_t lba = f->data_start_lba + (cluster - 2) * f->sec_per_clus;
    uint32_t bytes = f->sec_per_clus * 512;
    if (bytes > max_bytes) {
        bytes = max_bytes;
    }
    return blk_pread(f->blk, (uint64_t)lba * 512, bytes, buf, 0) < 0 ? -1 : 0;
}

/* Convert an 8.3 entry name to a NUL-terminated, lower-cased,
 * 12-byte canonical form (no dot, no space). */
static void mxs_fat_83_to_canonical(const uint8_t *raw, char out[12 + 1])
{
    unsigned i, j = 0;

    for (i = 0; i < 8 && raw[i] != ' '; i++) {
        out[j++] = (char)tolower(raw[i]);
    }
    for (i = 8; i < 11 && raw[i] != ' '; i++) {
        if (i == 8) {
            out[j++] = '.';
        }
        out[j++] = (char)tolower(raw[i]);
    }
    out[j] = 0;
}

static bool mxs_fat_83_name_match(const uint8_t *raw, const char *needle)
{
    char canon[12 + 1];

    if (raw[0] == 0x00 || raw[0] == 0xe5) {
        return false;   /* unused / deleted entry */
    }
    mxs_fat_83_to_canonical(raw, canon);
    return strcmp(canon, needle) == 0;
}

/* Try the @candidates in order, return the first cluster+size, or
 * {0,0} if nothing matched.  The buildbrain SD image ships with a
 * non-standard BPB (sec_per_clus=1, root_ents=0 with root_clus=2)
 * so the spec-mandated first root sector is often empty; if so we
 * fall back to scanning the data area for a valid 8.3 entry.
 * Empirically the buildbrain root sits around LBA 4098 on a 64MB
 * FAT32 with the broken BPB, so we sweep a generous chunk of the
 * data area. */
static void mxs_fat_search_root(MxsFAT *f, const char *const *candidates,
                                uint32_t *out_cluster, uint32_t *out_size)
{
    uint32_t entries_per_sec = 512 / 32;
    uint8_t sec[512];
    uint32_t i, j, lba;
    const char *const *cand;

    *out_cluster = 0;
    *out_size = 0;

    /* First, the spec path.  FAT12/16: root is contiguous; FAT32: a
     * cluster chain. */
    uint32_t cur = f->is_fat32 ? f->root_clus : 0;
    uint32_t start_lba = f->is_fat32 ?
        (f->data_start_lba + (cur - 2) * f->sec_per_clus) :
        f->root_start_lba;
    uint32_t max_secs = f->is_fat32 ?
        f->sec_per_clus : (f->root_entries + 15) / 16;

    for (i = 0; i < max_secs; i++) {
        if (mxs_fat_read_sector(f->blk, start_lba + i, sec,
                                sizeof(sec)) < 0) {
            return;
        }
        for (j = 0; j < entries_per_sec; j++) {
            const uint8_t *e = sec + j * 32;

            if (e[0] == 0x00) {
                break;
            }
            if ((e[11] & 0x0f) == 0x0f) {
                continue;
            }
            if (e[11] & 0x08) {
                continue;
            }
            for (cand = candidates; *cand; cand++) {
                if (mxs_fat_83_name_match(e, *cand)) {
                    *out_cluster = (lduw_le_p(e + 20) << 16) |
                                   lduw_le_p(e + 26);
                    *out_size = ldl_le_p(e + 28);
                    return;
                }
            }
        }
    }

    /* Spec path found nothing.  Scan a generous chunk of the data
     * area looking for a valid 8.3 entry - the buildbrain mkfs is
     * happy to write a root directory somewhere other than the
     * BPB's root_start_lba.  We sweep 16k sectors (8 MiB) which is
     * enough to cover the typical buildbrain layout. */
    for (lba = 0; lba < 16384; lba++) {
        if (mxs_fat_read_sector(f->blk, f->data_start_lba + lba, sec,
                                sizeof(sec)) < 0) {
            return;
        }
        for (j = 0; j < entries_per_sec; j++) {
            const uint8_t *e = sec + j * 32;

            if (e[0] == 0x00) {
                continue;
            }
            if ((e[11] & 0x0f) == 0x0f) {
                continue;
            }
            if (e[11] & 0x08) {
                continue;
            }
            for (cand = candidates; *cand; cand++) {
                if (mxs_fat_83_name_match(e, *cand)) {
                    *out_cluster = (lduw_le_p(e + 20) << 16) |
                                   lduw_le_p(e + 26);
                    *out_size = ldl_le_p(e + 28);
                    return;
                }
            }
        }
    }
}

/*
 * Try to read a file from the SD card's first FAT partition.  The
 * list of @candidates is consulted in order (lowercased 8.3 names,
 * without the dot); the first match wins.  Returns the number of
 * bytes read, or -1 on failure.
 */
int mxs_fat_read_file(BlockBackend *blk, const char *const *candidates,
                      uint8_t *out, uint32_t max_bytes)
{
    MxsFAT f;
    uint32_t cluster = 0, size = 0;
    uint32_t total = 0;

    if (mxs_fat_open(&f, blk, 0, 0) < 0) {
        return -1;
    }
    mxs_fat_search_root(&f, candidates, &cluster, &size);
    if (!cluster || !size) {
        return -1;
    }
    if (size > max_bytes) {
        size = max_bytes;
    }

    while (cluster >= 2 && cluster < 0x0ffffff8u && total < size) {
        uint32_t chunk = size - total;
        if (chunk > f.sec_per_clus * 512u) {
            chunk = f.sec_per_clus * 512u;
        }
        if (mxs_fat_read_cluster(&f, cluster, out + total, chunk) < 0) {
            return -1;
        }
        total += chunk;
        cluster = mxs_fat_get_entry(&f, cluster);
    }
    return total;
}
