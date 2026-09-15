# Phase 2 — Touch pipeline: status and measurements (2026-09-16, HEAD a48e1ea)

Scope: PW-AJ2 resistive touch path reproduce-ability. Fact vs hypothesis split below.

## FIXED (pushed)

1. **CTRL1 alias-clobber bug** — `7be37a1`
   - Symptom measured: `LRADC2_IRQ` line latched HIGH after the first conversion
     of every press and never fell until release.
   - Root cause: `case LRADC_CTRL1` in `mxs_lradc_write()` applied its W1C
     formula to writes through the set/clr/tog shadow aliases, even though
     `mxs_bank_apply()` already folded the alias into `val`:
     `new = (old & ~(val & 0xff00)) | (val & ~0xff00)` re-ORs the old register,
     so `CLR` of channel-pend bits [7:0] and IRQ-enable bits [23:16] never took
     effect.
   - Fix: keep the W1C overlay for *direct* writes only; aliases store exactly
     what `mxs_bank_apply()` computed.
   - Evidence (guest MMIO write log): the sampler does
     `CTRL1_SET 0x40000` (ch2 IRQ en) → DELAY1 KICK → 0.111 ms later
     `CTRL1_CLR 0x40000`, `CTRL1_CLR 0x4`. With the fix, `irq line 2` toggles
     up/down around each conversion (measured via new `mxs_lradc_irq` trace).

2. **DELAYn field swap** — `a48e1ea`
   - HW layout (i.MX28 RM + packing code at lradc.dll 0xc0671e54):
     `[31:24] TRIGGER_LRADCS, [19:16] TRIGGER_DELAYS, [20] KICK,
     [15:11] DELAY(5b, ~2 kHz tick), [10:0] LOOP_COUNT(11b)`.
   - Model decoded LOOP_COUNT from [15:11] and DELAY from [10:0] (swapped).
     Under the fix the sampler's standard kick word
     (trg 0x24, DELAY=31, LOOP=2047) decodes as intended; previously it armed
     31 loops of 1 s each.

## OPEN — measured facts

A. **End-to-end**: taps on the setup-wizard screens produce **zero framebuffer
   change** (10-tap grid over the date row, keyboard icon, 決定/戻る buttons).
   `screendump` before/after: diff = 0 pixels.

B. **The guest-side LRADC pipeline IS live during a press** (write-log captured
   around pen-down):
   - TD detect IRQ asserts at +0.0 ms (line 16).
   - First sampling burst starts at **+0.6 ms** with the standard
     3-phase sequence (`0x24/0x0c/0x04` kicks), plate values sane
     (mid-press ≈ 0x0808 ≈ 50 % FS, release 0x0000).
   - Sampling keeps a fast cadence for ~30 ms, then decays to single-channel
     reads ~170 ms apart, and re-accelerates into full 12×3 funnel batches
     right after pen-up. (Same signature pre/post CTRL1 fix; with the fix the
     IRQ-en window per conversion now closes properly.)
   - touch.dll's own IST (`0x4544`, WaitForSingleObject 2 s) delivers via
     `GetSampler`; the guest code path that feeds GWES is present.

C. **Geometry doubt — plate law vs factory calibration**
   - Model law: rawX = 160..3712 across (800 px), rawY = 3964..211 across 480.
   - Guest factory hive `CalibrationData` (from nk hive):
     `1931,1961 889,2993 888,906 2961,920 2958,3039`.
   - The model law is consistent with the guest calib **only if the logical
     surface is 800×480**. The LCD panel is 854-wide (model docs claim the
     guest insets its picture; the screendump clearly shows drawn content —
     including OS UI elements — well beyond column 800, so if the inset story
     is right, GWES draws 800-wide **for touch purposes** but the wizard
     picture visibly extends past 800 ⇒ contradiction).
   - Unverified which one is wrong: the plate-law constants or the
     'picture inset' assumption. A press at console (814, 12) currently
     resolves to px=814 → `in_strip` (px ≥ BRAIN_PLATE_X_SPAN=800) and is
     diverted into the touchkey-strip path (`brain_kbd_touchkey_strip`)
     instead of the plate. 54 columns at the right edge are unreachable as
     touch. If the real digitizer spans the full LCD, that is a bug.
   - Arbitration plan: reach the wizard's calibration screen (crosshair at
     known logical positions) and check whether OS-adjudicated calibration
     converges back to the factory values when fed our samples.

D. **Setup wizard navigation**: current screen is 「日付と時刻の設定」
   (digits + cursor keys + 決定 expected; hardware `sendkey 9/1/ret/tab/down`
   produced no visible change — key delivery to the app unverified, kernel
   keyboard path itself is *not* in phase-2 scope per user).

E. **Printer's eye view of remaining touch suspects** (ordered):
   1. Coordinate/law mismatch (C).
   2. touch.dll sample qualification (its `func_41d4`-path on IRQ) discarding
      live samples — would need guest-side breakpointing to prove.
   3. The interactive wizard's buttons simply not accepting synthetic press
      cadence (quick taps): the `mxs_lradc_kick` synchronous conversion still
      generates only ONE conversion edge before the CTRL1 CLR arrives;
      whether CE's level-latched ICOLL delivery catches all of them needs a
      held-tap retest with `mxs_lradc_irq` tracing (tap6 showed full-speed
      sampling during 3 s holds and correct IRQ pulsing — that part looks right).

F. **Idle-suspend**: the guest power manager shuts down at ~2–4 min idle
   (`PMI_PowerDown`/`VMC_PowerDown` on serial) which terminates QEMU runs —
   every experiment must fit between boot-complete (~50 s) and suspend.

## Next steps
- Calibration-screen arbitration for the plate law (C).
- Held-tap IRQ-window verification with the new trace (E3).
- Display-shear task (second half of the Phase-2 request) — see notes in repo
  about the panel scan/GRAM rotation.

## Diagnostics added on this branch (uncommitted-at-time-of-writing)
- `mxs_lradc_hwaccess` / `mxs_lradc_reads` trace events for full LRADC MMIO
  access logs, and matching unaligned-read dispatch through
  `mxs_bank_extract()`. Boot-verified.
