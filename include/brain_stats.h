/*
 * SHARP Brain / WinCE bring-up: lightweight runtime statistics.
 *
 * Cheap counters and a tiny event ring used to diagnose TB/TCG
 * misbehaviour (1-insn TB storms, deferred-MMU quirk application,
 * interrupt collector latching) without recompiling.  Everything is
 * compiled in but costs a couple of increments per event; dumps are
 * exposed through the `brain_stats` / `brain_events` HMP commands.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef BRAIN_STATS_H
#define BRAIN_STATS_H

#include <stdint.h>
#include <stdio.h>

typedef enum BrainStat {
    /* translation health */
    BST_TB_TRANSLATED,          /* tb_gen_code completed */
    BST_TB_ONE_INSN_MMIO_PAGE,  /* phys_pc == -1: executing from MMIO */
    BST_TB_ONE_INSN_CF,         /* cflags COUNT == 1 (forced single)  */
    BST_TB_ONE_INSN_SS,         /* architectural single step          */
    BST_TB_FLUSH,               /* full TB cache flushes              */
    BST_IO_RECOMPILE,           /* cpu_io_recompile rewinds           */
    BST_IO_RECOMPILE_SKIPPED,   /* rewinds elided (non-deterministic) */
    /* deferred-MMU quirk */
    BST_QUIRK_SCTLR_DEFER,      /* suppress-TB-exit SCTLR writes      */
    BST_QUIRK_APPLY_TBEND,      /* apply at natural TB end (gen code) */
    BST_QUIRK_APPLY_LOOKUP,     /* apply from arm_get_tb_cpu_state    */
    BST_QUIRK_APPLY_EXCEPTION,  /* apply from exception entry         */
    BST_QUIRK_APPLY_PENDING,    /* applies that flushed a real toggle */
    /* exceptions */
    BST_EXCP_IRQ,
    BST_EXCP_FIQ,
    BST_EXCP_UNDEF,
    BST_EXCP_PABORT,
    BST_EXCP_DABORT,
    BST_EXCP_SVC,               /* swi */
    BST_EXCP_OTHER,
    /* interrupt collector */
    BST_ICOLL_IRQ_ASSERT,
    BST_ICOLL_IRQ_DEASSERT,
    BST_ICOLL_VECTOR_RSE,       /* VECTOR read in RSE mode            */
    BST_ICOLL_LEVELACK,
    BST_ICOLL_SUPPRESS_NEW,     /* pending masked by in-service level */
    BST_ICOLL_AUTORELEASE,      /* serving level auto-released        */
    /* timers */
    BST_TIMROTn_EXPIRE0,
    BST_TIMROTn_EXPIRE1,
    BST_TIMROTn_EXPIRE2,
    BST_TIMROTn_EXPIRE3,
    BST_MAX
} BrainStat;

extern uint64_t brain_stat_count[BST_MAX];

static inline void brain_stat_inc(BrainStat s)
{
    brain_stat_count[s]++;
}

/* event ring: (kind, a, b, c) tuples, kind is a 4-char tag */
typedef struct BrainEvent {
    uint32_t kind;
    uint32_t a, b, c, d;
} BrainEvent;

#define BRAIN_EVENT_RING 256
extern BrainEvent brain_events[BRAIN_EVENT_RING];
extern uint32_t brain_events_pos;

static inline void brain_log_event(uint32_t kind, uint32_t a,
                                   uint32_t b, uint32_t c, uint32_t d)
{
    uint32_t p = brain_events_pos++ & (BRAIN_EVENT_RING - 1);
    brain_events[p].kind = kind;
    brain_events[p].a = a;
    brain_events[p].b = b;
    brain_events[p].c = c;
    brain_events[p].d = d;
}

/* helpers to form a 4-char tag */
#define BSTAG(c0, c1, c2, c3) \
    (((uint32_t)(uint8_t)(c0) << 24) | ((uint32_t)(uint8_t)(c1) << 16) | \
     ((uint32_t)(uint8_t)(c2) << 8) | (uint32_t)(uint8_t)(c3))

/*
 * brain_bwatch with a count > 1: keep the guest running across hits.
 * cpu_handle_exception() in accel/tcg/cpu-exec.c checks this flag and
 * turns the EXCP_DEBUG exit into a plain resume (*ret = 0) so the
 * vCPU loop does not stop; the debug_excp_handler in hw/arm/mxs.c
 * manages the per-address hit counters and clears the flag when all
 * breakpoints are exhausted.  Only meaningful while a counted
 * breakpoint is armed.
 */
extern int brain_bwatch_auto_resume;

void brain_stats_dump(FILE *f);
void brain_events_dump(FILE *f, unsigned last);

#endif /* BRAIN_STATS_H */
