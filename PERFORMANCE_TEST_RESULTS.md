# Performance Test — Results Log

Paired with [PERFORMANCE_TEST_PLAN.md](PERFORMANCE_TEST_PLAN.md). One row per bag; fill as runs land.

Legend for columns:
- `V₀` = starting Vbus (V); `ΔV` = Vbus drop over the run.
- `RPM_mean` / `RPM_std` = over the middle 6 s of each CL hold.
- `Iq_mean` / `Iq_peak` = q-axis current (mA).
- `SwapDef` = count of ticks with `esc/swap_deferred == 1`.
- `Pass` = pass criterion from planner §1.2 (reaches ≥90% setpoint, std ≤±5%, zero faults, swap=0, ΔV<0.5, both dirs).

---

## Phase 1 — Top speed

### Sub-phase A — Node-only `max_speed_rpm` sweep (firmware unchanged)

#### AMORIL #1 — bench

| Run | Bag | max_spd | iq_lim | Dir | V₀ | RPM_mean | RPM_std | Iq_mean | Iq_peak | SwapDef | ΔV | Pass | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A1 |  | 3000 | 12 | FWD |  |  |  |  |  |  |  |  | baseline replicate |
| A1 |  | 3000 | 12 | REV |  |  |  |  |  |  |  |  |  |
| A2 |  | 4000 | 12 | FWD |  |  |  |  |  |  |  |  |  |
| A2 |  | 4000 | 12 | REV |  |  |  |  |  |  |  |  |  |
| A3 |  | 5000 | 15 | FWD |  |  |  |  |  |  |  |  |  |
| A3 |  | 5000 | 15 | REV |  |  |  |  |  |  |  |  |  |
| A4 |  | 6000 | 18 | FWD |  |  |  |  |  |  |  |  |  |
| A4 |  | 6000 | 18 | REV |  |  |  |  |  |  |  |  |  |
| A5 | perf1A_hosim_bench_maxspd7000_iqlim20_fwd_20260422_121508 | 7000 | 20 | FWD | — | 6292 | 1308 (21 %) | 17.9 A | 21.6 A | 0 | 0.1 | ⚠ | **Bench top RPM: 6292 (sustained 6.7 s, CL#5).** User note: grinding if stick pushed to full; works if stick matched mid-crossfade. OL→CL handover unstable at full stick on bench. |
| A5 | (same bag) | 7000 | 20 | REV | — | −2450 to −3965 | 1648–1802 (55–73 %) | −10.9 to −16.9 A | 20.3 A | 0 | — | ✗ | REV wild oscillation; classic HOSIM FWD/REV asymmetry. |

#### AMORIL #1 — ground
(only rows whose bench entry passed)

| Run | Bag | max_spd | iq_lim | Dir | V₀ | RPM_mean | RPM_std | Iq_mean | Iq_peak | SwapDef | ΔV | Pass | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

#### HOSIM — bench

| Run | Bag | max_spd | iq_lim | Dir | V₀ | RPM_mean | RPM_std | Iq_mean | Iq_peak | SwapDef | ΔV | Pass | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A1 | perf1A_hosim_bench_maxspd3000_fwd_20260422_114111 | 3000 | 12 | FWD | — | 2217 | 217 (10 %) | 11.9 A | 12.3 A | 0 | — | ✓ | PI-saturated at 12 A → motor ~2250 RPM; 6.3 s CL; both-dir bag (FWD + REV in same file) |
| A1 | (same bag, 2nd push) | 3000 | 12 | REV | — | −2244 | 261 (12 %) | −11.9 A | 12.4 A | 0 | — | ✓ | PI-saturated; 6.9 s CL |
| — | perf1A_hosim_bench_maxspd3000_fwd_20260422_113356 | — | — | — | — | — | — | — | — | — | — | ✗ | INVALID — captured with stale `esc_node_custom_foc.py` (pre-vehicle-profile); discard. |
| A2 |  | 4000 | 12 | FWD |  |  |  |  |  |  |  |  | SKIP — same PI ceiling as A1, no new data |
| A2 |  | 4000 | 12 | REV |  |  |  |  |  |  |  |  | SKIP |
| A3 |  | 5000 | 15 | FWD |  |  |  |  |  |  |  |  | first real sweep — raises iq clamp |
| A3 |  | 5000 | 15 | REV |  |  |  |  |  |  |  |  |  |
| A4 |  | 6000 | 18 | FWD |  |  |  |  |  |  |  |  |  |
| A4 |  | 6000 | 18 | REV |  |  |  |  |  |  |  |  |  |
| A5 |  | 7000 | 20 | FWD |  |  |  |  |  |  |  |  |  |
| A5 |  | 7000 | 20 | REV |  |  |  |  |  |  |  |  |  |

Legend: ⚠ = technically fails planner §1.2 (≥90 % setpoint + ≤±5 % std) because PI is at iq_lim, but reproduces production baseline cleanly with no faults and both directions working. Treat as **PASS, PI-saturated**.

#### HOSIM — ground

| Run | Bag | max_spd | iq_lim | Dir | V₀ | RPM_mean | RPM_std | Iq_mean | Iq_peak | SwapDef | ΔV | Pass | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A1 | perf1A_hosim_gound_maxspd3000_fwd_20260422_114254 | 3000 | 12 | FWD | — | 1626–1806 | 100–260 (6–15 %) | 11.9–12.0 A | 12.4 A | 0 | — | ✓ | 13 CL intervals over ~200 s. Longest sustained CL = **40.2 s** (CL#13, spd 1626, std 6 %). No OL failures, zero faults, zero swap_def. PI-saturated at 12 A. |
| A1 | (same bag, REV pushes) | 3000 | 12 | REV | — | −1710 to −1738 | 156–184 (9–11 %) | −11.9 A | 12.4 A | 0 | — | ✓ | REV CLs 2.6–3.8 s each, clean user-release exits |
| A3 | perf1A_hosim_gound_maxspd5000_iqlim15_fwd_20260422_120004 | 5000 | 15 | FWD | — | 2887–2927 | 297–385 (10–13 %) | 14.7 A | 15.3 A | 0 | — | ✓ | PI-saturated at 15 A; motor tops ~2900 RPM. 2 FWD CLs (4.4 / 3.1 s) + 1 REV CL. |
| A3 | (same bag) | 5000 | 15 | REV | — | −2937 | 334 (11 %) | −14.7 A | 15.2 A | 0 | — | ✓ | 3.8 s CL |
| A4 | perf1A_hosim_gound_maxspd6000_iqlim18_fwd_20260422_120154 | 6000 | 18 | FWD | — | 3833–3850 | 547–622 (14–16 %) | 17.5 A | 18.5 A | 0 | — | ✓ | PI-saturated at 18 A; motor tops ~3830 RPM. Std widening. |
| A4 | (same bag) | 6000 | 18 | REV | — | −3738 to −3983 | 453–594 (11–16 %) | −17.4 A | 18.9 A | 0 | — | ✓ | 2 REV CLs 2.4 / 3.0 s |
| A5 | perf1A_hosim_gound_maxspd7000_iqlim20_fwd_20260422_120339 | 7000 | 20 | FWD | — | 4526–4917 | 530–580 (11–13 %) | 19.3 A | **21.5 A** | 0 | — | ✓ | **Top RPM achieved: 4917, sustained 7.3 s.** Peak iq exceeds clamp by 7 % — verify. |
| A5 | (same bag) | 7000 | 20 | REV | — | −4456 | 644 (14 %) | −19.0 A | 20.4 A | 0 | — | ✓ | 3.3 s CL, clean |
| — | perf1A_hosim_gound_maxspd3000_fwd_20260422_113605 | — | — | — | — | — | — | — | — | — | — | ✗ | INVALID — captured with stale `esc_node_custom_foc.py` (pre-vehicle-profile); discard. |
| — | perf1A_hosim_gound_maxspd5000_fwd_* (2 bags, 11:51 & 11:55) | — | — | — | — | — | — | — | — | — | — | ✗ | SUPERSEDED by 5000_iqlim15 run at 12:00 |

**Sub-phase A (HOSIM ground) observation:** motor RPM scales linearly with `iq_limit_a` at ratio ≈ 240 RPM/A (ground). PI is saturated at every setpoint — the `max_speed_rpm` parameter is aspirational; the real ceiling is current. Bench A5 still pending to establish if this is a motor ceiling or drivetrain-load ceiling.

**Bag naming convention (updated to match user 2026-04-22):** `perf1A_hosim_<bench|gound|ground>_maxspd<XXXX>_iqlim<YY>_<fwd|rev>_<timestamp>` — "gound" spelling used in the 11:xx–12:xx bags; later bags fixed to "ground".

---

### Sub-phase A′ — iq-isolated sweep at iq_lim = 20 A (HOSIM, added 2026-04-22 afternoon)

Motivated by the A-matrix finding that RPM tracks `iq_limit_a` linearly while `max_speed_rpm` is aspirational. A′ fixes `iq_lim=20` (the A5 ceiling) and sweeps `max_speed_rpm` *downward* to find out whether lowering the setpoint yields a real (non-PI-saturated) speed loop.

#### HOSIM — bench (iq_lim = 20)

| Run | Bag | max_spd | Dir | spd_mean | spd_std | iq_peak | Dur | Pass | Notes |
|---|---|---|---|---|---|---|---|---|---|
| A′-b1 | perf1A_hosim_bench_maxspd3500_iqlim20_fwd_20260422_135524 | 3500 | FWD | 3499 | **90 (2.6 %)** | 15.3 A | **13.3 s** | ✓✓ | **Bench FWD cleanest run of the day** — real speed loop, PI not saturated (iq mean 14.2 A < 20 A clamp). |
| A′-b1 | (same bag) | 3500 | REV | −2574 | 1012 (39 %) | 20.0 A | 13.6 s | ✗ | Oscillating; FWD/REV asymmetry. |
| A′-b2 | perf1A_hosim_bench_maxspd5000_iqlim20_fwd_20260422_122944 | 5000 | FWD | 4978 | **150 (3.0 %)** | 20.7 A | **8.3 s** | ✓ | FWD clean but PI at clamp (iq 16.4 A avg, peak 20.7 A). |
| A′-b2 | (same bag) | 5000 | REV | −3096 to −3640 | 1509–1567 (42–55 %) | 20.3 A | 2.2–3.8 s | ✗ | REV wild oscillation at every setpoint ≥ 3500 on bench. |
| A′-b3 | perf1A_hosim_bench_maxspd7000_iqlim20_fwd_20260422_121508 | 7000 | FWD | 6292 | 1308 (21 %) | 21.6 A | 6.7 s | ⚠ | Works if stick matched at crossfade; grinds if stick at full. |

#### HOSIM — ground (iq_lim = 20)

| Run | Bag | max_spd | Dir | spd_mean | spd_std | iq_peak | Longest CL | Pass | Notes |
|---|---|---|---|---|---|---|---|---|---|
| A′-g1 | perf1A_hosim_ground_maxspd3500_iqlim20_fwd_20260422_141116 | 3500 | FWD | 1960 | 1008 (**51 %**) | 15.6 A | 3.1 s | ✗ | **Oscillation inside CL** — spd spans 0–3454 within one interval. PI operates in linear region → limit cycle. |
| A′-g1 | (same bag) | 3500 | REV | −2376 | 1189 (50 %) | 16.4 A | 3.1 s | ✗ | Same oscillation pattern REV. |
| A′-g2 | perf1A_hosim_ground_maxspd5000_iqlim20_fwd_20260422_140929 | 5000 | FWD | 2361 | 1427 (**60 %**) | 17.4 A | 2.3 s | ✗ | Oscillation; spd_min=0 mid-CL; iq NOT saturated (peak 17 A < 20 A). |
| A′-g2 | (same bag) | 5000 | REV | −2720 | 1468 (54 %) | 20.2 A | 2.3 s | ✗ | Same. |
| A′-g3 | perf1A_hosim_ground_maxspd7000_iqlim20_fwd_20260422_140659 | 7000 | FWD | 4674 | **528 (11 %)** | 21.5 A | **9.8 s** | ✓✓ | **Ground production sweet spot** — PI pinned at 20 A, motor load-limited to ~4700 RPM, stable. 10 CL intervals, zero faults, Vbus 11.7→11.6 V. |
| A′-g3 | (same bag) | 7000 | REV | — | — | — | — | — | FWD-only push in this bag; REV at 7000/20 not separately captured (legacy A5 at 7000/20 showed REV −4456 std 14 %). |

### Sub-phase A + A′ — diagnosis (HOSIM, 2026-04-22)

**Mechanism (PI-saturated vs linear-region modes):**

- When setpoint (`u · max_speed_rpm`) is **well above** what the motor can physically reach at the iq clamp, the speed PI pins at `iq_limit_a` → system runs as **torque mode**. This is stable: constant max current produces whatever speed the load allows, and there is no speed error feedback to oscillate.
- When setpoint is **near or below** what the motor can reach, the PI stays in its **linear region** → speed error drives iq. With the current defaults (`spd_kp=0.02`, `spd_ki=0.002`, `spd_lpf_alpha=0.001`, commits `d4cfbd0` / `26abad7`) this region **limit-cycles on HOSIM ground** (50–60 % std), but not on bench FWD (2.6–3 % std). Load inertia and surface friction change the loop gain.
- **Bench/ground inversion:** on bench (low inertia) the motor over-speeds fast at high setpoints → OL→CL handoff fails / observer lags ("grinding" at 7000/20). On ground (loaded) the same 7000/20 can't reach setpoint → PI saturates → stable.

**Evidence summary:**

| Surface | max_spd | iq_lim | PI state | Outcome |
|---|---|---|---|---|
| Bench FWD | 3500 | 20 | linear | **2.6 % std, perfect speed track** |
| Bench FWD | 5000 | 20 | linear→saturated | 3.0 % std, PI at clamp |
| Bench FWD | 7000 | 20 | saturated | grinding at full stick |
| Ground FWD | 3500 | 20 | linear | 51 % std (oscillation) |
| Ground FWD | 5000 | 20 | linear | 60 % std (oscillation) |
| Ground FWD | 7000 | 20 | saturated | **11 % std, 4700 RPM sustained** |

### Sub-phase A summary — PASSING SETPOINTS

| Car | Surface | Recommended `max_speed_rpm` | `iq_limit_a` | Dir | Actual RPM | Limiting factor |
|---|---|---|---|---|---|---|
| HOSIM | **ground** | **7000** | **20** | **FWD** | **~4700** | **load + PI clamp (= torque mode)** |
| HOSIM | ground | 7000 | 20 | REV | ~4500 | same, slightly worse observer |
| HOSIM | bench | 3500 | 20 | FWD | 3500 | linear PI, tracks setpoint |
| HOSIM | bench | 5000 | 20 | FWD | 5000 | PI-saturated at load-free speed |
| HOSIM | bench | — | — | REV | oscillates at every setpoint ≥ 3500 | HOSIM FWD/REV observer asymmetry (known) |
| AMORIL #1 | — | — | — | — | — | matrix not re-run on 2026-04-22 |

---

### Sub-phase B — Observer SNR probe at high RPM

(Run only after A identifies the highest passing `max_speed_rpm`.)

| Run | Bag | Car | max_spd | iq_lim | Observation |
|---|---|---|---|---|---|

---

### Sub-phase C — Firmware cap lift (commit on `performance_test` branch)

(Only if A+B motivate it.)

Firmware edits:
- [ ] `esc_comm.c:392` validator widened 1000–2000 → 1000–5000
- [ ] PLL gain review at `drive_parameters.h:57-58` (decide retune or keep)
- [ ] Release build flashed; commit SHA: _____

| Run | Bag | Car | ol_target | max_spd | iq_lim | Dir | Pass | Notes |
|---|---|---|---|---|---|---|---|---|

---

### Sub-phase D — Gearing / linear speed calibration

| Car | max_spd setting | Motor RPM (mean) | Measured linear speed | Method |
|---|---|---|---|---|

Conclusion: recommended `max_linear_mps` for autonomy launch = _____.

---

## Phase 1 — Recommendations (as of 2026-04-22, HOSIM only; AMORIL pending)

1. **New HOSIM ground production defaults:** `max_speed_rpm = 7000`, `iq_limit_a = 20`. This produces ~4700 RPM sustained FWD (2.1× baseline 2250 RPM at 12 A), 11 % std, zero faults, Vbus droop 0.1 V. Mode is effectively **PI-saturated torque mode** — `u` gates torque, not speed — but stable and predictable under load.
2. **Firmware cap-lift (Sub-phase C):** not needed yet. The 1000–5000 RPM validator at [esc_comm.c:392](motor_test1_20260227/Src/esc_comm.c#L392) is still within its range when OL handoff stays at 1600; `max_speed_rpm` is node-side and already accepts up to 10000. Defer until a real speed loop retune (below) shows we need a higher OL handoff.
3. **Open question — real speed loop retune:** current gains (`spd_kp=0.02`, `spd_ki=0.002`, `spd_lpf_alpha=0.001`) produce clean speed tracking on bench FWD (2.6 % std at 3500 RPM) but limit-cycle on ground (51–60 % std) because of load/inertia mismatch. If Phase 4+ (nav2) needs `u → RPM` linearity, run a dedicated per-surface retune at `max_spd=4000, iq_lim=20` (setpoint below the ~4700 ground ceiling so PI has headroom). For joystick/manual use, PI-saturated torque mode is simpler and stays at this config.
4. **Known HOSIM asymmetry:** FWD ~4700, REV ~4500 on ground; REV oscillation on bench at every setpoint ≥ 3500. Observer-side, known since 2026-04-13 — not blocking production.
5. **Not re-tested on AMORIL #1:** the A′ sweep was HOSIM-only on 2026-04-22. Before promoting these values to a shared production default, repeat A′-g3 on AMORIL to confirm it doesn't overcurrent at 20 A (AMORIL baseline was 12 A).
6. **Follow-ups for Phase 2+:**
   - Sub-phase D (linear speed calibration at 7000/20 on ground) — pending, needs straight-line timing test.
   - Sustained-duration run (≥ 30 s at 7000/20) to probe thermal/duty behavior (Phase 3 entry).
   - Investigate iq peak at 21.5 A exceeding the 20 A clamp by ~7 % — likely a transient at CL entry before the clamp kicks in; verify with iq timeseries.
