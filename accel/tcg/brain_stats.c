/*
 * SHARP Brain / WinCE bring-up: lightweight runtime statistics (impl).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "brain_stats.h"

uint64_t brain_stat_count[BST_MAX];
BrainEvent brain_events[BRAIN_EVENT_RING];
uint32_t brain_events_pos;

BrainExcpFault brain_last_fault;

/* see include/brain_stats.h -- auto-resume across counted brain_bwatch */
int brain_bwatch_auto_resume;

static const char *const brain_stat_names[BST_MAX] = {
    [BST_TB_TRANSLATED]            = "tb-translated",
    [BST_TB_ONE_INSN_MMIO_PAGE]    = "tb-1insn-mmio-page",
    [BST_TB_ONE_INSN_CF]           = "tb-1insn-cflags",
    [BST_TB_ONE_INSN_SS]           = "tb-1insn-ss",
    [BST_TB_FLUSH]                 = "tb-flush",
    [BST_IO_RECOMPILE]             = "io-recompile",
    [BST_IO_RECOMPILE_SKIPPED]     = "io-recompile-skipped",
    [BST_QUIRK_SCTLR_DEFER]        = "quirk-sctlr-defer",
    [BST_QUIRK_APPLY_TBEND]        = "quirk-apply-tbend",
    [BST_QUIRK_APPLY_LOOKUP]       = "quirk-apply-lookup",
    [BST_QUIRK_APPLY_EXCEPTION]    = "quirk-apply-excp",
    [BST_QUIRK_APPLY_PENDING]      = "quirk-apply-flush",
    [BST_EXCP_IRQ]                 = "excp-irq",
    [BST_EXCP_FIQ]                 = "excp-fiq",
    [BST_EXCP_UNDEF]               = "excp-undef",
    [BST_EXCP_PABORT]              = "excp-pabort",
    [BST_EXCP_DABORT]              = "excp-dabort",
    [BST_EXCP_SVC]                 = "excp-svc",
    [BST_EXCP_OTHER]               = "excp-other",
    [BST_ICOLL_IRQ_ASSERT]         = "icoll-irq-assert",
    [BST_ICOLL_IRQ_DEASSERT]       = "icoll-irq-deassert",
    [BST_ICOLL_VECTOR_RSE]         = "icoll-vector-rse",
    [BST_ICOLL_LEVELACK]           = "icoll-levelack",
    [BST_ICOLL_SUPPRESS_NEW]       = "icoll-suppressed",
    [BST_ICOLL_AUTORELEASE]        = "icoll-autorelease",
    [BST_TIMROTn_EXPIRE0]          = "timrot0-expire",
    [BST_TIMROTn_EXPIRE1]          = "timrot1-expire",
    [BST_TIMROTn_EXPIRE2]          = "timrot2-expire",
    [BST_TIMROTn_EXPIRE3]          = "timrot3-expire",
};

void brain_stats_dump(FILE *f)
{
    int i;

    for (i = 0; i < BST_MAX; i++) {
        if (brain_stat_count[i]) {
            fprintf(f, "  %-24s %" PRIu64 "\n",
                    brain_stat_names[i] ? brain_stat_names[i] : "?",
                    (uint64_t)brain_stat_count[i]);
        }
    }
}

void brain_events_dump(FILE *f, unsigned last)
{
    uint32_t total = brain_events_pos;
    uint32_t start, i, n;

    if (!total) {
        fprintf(f, "  (no events)\n");
        return;
    }
    n = MIN(last, BRAIN_EVENT_RING);
    if (n > total) {
        n = total;
    }
    start = brain_events_pos - n;
    for (i = 0; i < n; i++) {
        BrainEvent *e = &brain_events[(start + i) & (BRAIN_EVENT_RING - 1)];
        char tag[5];

        tag[0] = e->kind >> 24;
        tag[1] = e->kind >> 16;
        tag[2] = e->kind >> 8;
        tag[3] = e->kind;
        tag[4] = 0;
        fprintf(f, "  #%06u %s a=%08x b=%08x c=%08x d=%08x\n",
                start + i, tag, e->a, e->b, e->c, e->d);
    }
}
