---
name: Step 6 Session 5 — OL→CL transition UNRESOLVED (v1/v2/v3 tried)
description: Grinding + lurch at CROSSFADE→CLOSED_LOOP; 3 fix iterations, user reports v3 feels worse than Session 3 baseline; pick up next session
type: project
---

# Step 6 Session 5 — OL→CL transition hardening (UNRESOLVED)

**Branch:** `custom_foc`  **Date:** 2026-04-07  **Status:** NOT CLOSED — pick up next session

## Symptom

At CROSSFADE→CLOSED_LOOP: grinding noise + brief slowdown + lurch. Reverse direction
consistently worse than forward. ~70% of transitions are rough (Iq@CL ≈ 0 instead of ~10A)
even after three fix iterations. User's end-of-day assessment: v3 "feels less robust than
the morning tests" — possible regression from Session 3 baseline.

## Three iterations tried (all in tree right now as v3 state)

**v1** — bumpless PI + seed ekf_omega_pi + seed spd_cmd_rpm = ekf_rpm + ESC slew from actual speed
**v2** — added EKF convergence gate + changed spd_cmd_rpm seed to ekf_omega_pi/RPM_TO_ERAD_S
**v3** — reverted CFOC_OL_TARGET_RPM from 1200 → 1400 RPM (kept all v1+v2 fixes)

## Current code state (end of day)

| File:line | Content | Origin |
|---|---|---|
| `custom_foc.h:75` | `CFOC_OL_TARGET_RPM = 1400.0f` | v3 |
| `custom_foc.c:657-658` | `ekf_omega_filt = ekf_omega_pi = ol_omega_e` (crossfade entry) | v1 Fix B |
| `custom_foc.c:687-716` | EKF convergence gate (20% tolerance, +500ms cap) | v2 Fix E |
| `custom_foc.c:743` | `pi_spd.integral = ol_Iq_ref` | v1 Fix A |
| `custom_foc.c:744` | `spd_cmd_rpm = ekf_omega_pi / RPM_TO_ERAD_S` | v2 Fix F |
| `esc_app.c:196-203` | `esc_speed_cmd = CFOC_GetSpeedRPM()` on first CL tick | v1 Fix D |

## Key bags

- `rosbag2_2026_04_07-18_04_22` — **Session 3 ground, last known-good baseline** (1400 RPM)
- `rosbag2_2026_04_07-19_38_44` — Session 5 v3 ground (1400 RPM + all fixes) — 4/6 rough
- `rosbag2_2026_04_07-19_41_21` — Session 5 v3 ground — 2/2 rough + Iq sign anomaly at reverse #1

Analysis: `python3 tests/analyze_cl_transition.py <bag_dir>`

## Anomaly to investigate first

Bag 19_41_21 #1: motor speed -1202 RPM (reverse) but Iq telemetry +0.06 A (positive sign).
Expected sign should match direction. Either telemetry Park transform bug or real control issue.

## Hypotheses for next session (ordered by priority)

1. **v2 regressed from Session 3.** User's subjective feedback is the most important data
   point. Session 3 was solid; v3 is inconsistent. **Action:** quantitatively diff bag
   18_04_22 vs 19_38_44 with the analyzer. If Session 3 was mostly smooth, the regression
   is in Fix E (gate) or Fix F (new PI seed).

2. **EKF gate (Fix E) may be holding crossfade too long.** Up to 1000ms at α=1.0 (pure EKF)
   if gate never passes — bad EKF estimate drives angle all that time. **Action:** disable
   gate, retest.

3. **Fix F + Fix D interaction.** CFOC seeds `spd_cmd_rpm = ekf_omega_pi/RPM` (τ=100ms LPF),
   ESC overrides 1ms later with `ekf_omega_filt/RPM` (τ=20ms LPF). Mismatch between the two
   LPFs could create a tick-two speed error. **Action:** make CFOC and ESC use the same
   signal.

4. **Hardware asymmetry forward/reverse.** Consistent across all v1/v2/v3 — may be
   motor/mechanical, not firmware. **Action:** bench no-load test both directions.

5. **Full revert fallback.** If hypothesis 1 confirms v2 regression:
   `git diff 5f27d90 -- motor_test1_20260227/` shows everything changed since Session 3.

## Recommended first experiment next session

Disable Fix E (EKF gate) — comment out the `if (!ekf_ok && !xf_cap)` block in
`custom_foc.c:711-716`. Keep everything else. Ground test. If Session 3 reliability returns,
the gate is the culprit.

## What to keep vs drop (recommendation)

- **KEEP:** Fix A (bumpless PI integral), Fix B (seed ekf_omega_pi), v3 (1400 RPM)
- **DROP:** v1 Fix C (superseded)
- **TEST WITHOUT:** Fix E (EKF gate) — most likely regression source
- **REVIEW:** Fix F + Fix D — may conflict on LPF signal used

## Full details

See `docs/custom_foc_plan.md` Section 5, "Session 5" subsection for complete narrative,
rosbag data, per-transition tables, and hypothesis details.
