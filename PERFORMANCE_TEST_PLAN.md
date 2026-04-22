# Performance & Limits Test Plan

Branch: `performance_test`. Forked from `main` @ `a13f112` (production baseline 2026-04-22).

Goal: now that both AMORIL #1 and HOSIM run the same firmware reliably at the baseline operating point, characterize where each knob's practical ceiling is. Starting with **top speed**, then current/torque, then thermal/duty, then battery/range.

This file is the planner — one phase per section, each with a hypothesis, a protocol, the metrics we'll record, and explicit pass/fail/abort criteria. Results land in a sibling `PERFORMANCE_TEST_RESULTS.md` (created when bags are in).

> **Ground rules (all phases):**
> 1. Bench (wheels off / in fixture) before ground.
> 2. Every run recorded with `ros2 bag record -a` in the workspace's `bag_file/` directory; filenames encode date+phase+index.
> 3. Fresh battery per phase, logged initial Vbus.
> 4. Abort immediately on: `esc/faults != none`, `esc/swap_deferred` sticking at 1 for >250 ms, unprompted `cfoc_state → IDLE` with stick held, Vbus < 11.5 V, any audible grinding.
> 5. Each new firmware-edit iteration bumps a commit on `performance_test`; node-side sweeps don't require rebuild.

---

## Phase 1 — Top speed ceiling

### 1.1 Hypothesis

Production CLI runs `max_speed_rpm=3000` and `ol_target_rpm=1600`. The motor + observer combo can probably sustain higher than 3 kRPM mechanical; the question is where the three ceilings are:

- **Observer ceiling:** at what CL speed does legacy STO lose SNR / PLL track?
- **Firmware cap:** `ol_target_rpm` is firmware-capped at 2000 RPM (see `esc_comm.c` validator); CL has no explicit cap, but speed-PI `iq_limit_a` bounds it.
- **Hardware ceiling:** B-G431B-ESC1 rated ~40 A continuous; small motor pole-pair-2 + Rs=0.1 Ω + Ls=10 µH — thermal / back-EMF margin unknown at >5 kRPM.

Expected order of events as we push `max_speed_rpm` upward: speed PI saturates at `iq_limit_a` → motor stops accelerating → `esc/speed_rpm` plateaus below `u * max_speed_rpm` setpoint (PI can't reach target). Second failure mode: observer noise grows with RPM (wider `esc/iq_ma` band, eventual `swap_deferred` spikes).

### 1.2 Sub-phase A — Node-only sweep (no firmware rebuild)

Leave firmware as-is (`ol_target_rpm ≤ 2000` cap). Sweep `max_speed_rpm` on the node CLI and measure what the motor actually achieves in closed-loop.

**Matrix** (run on each car, FWD first, then REV in a separate bag):

| run | max_speed_rpm | iq_limit_a | notes |
|---|---|---|---|
| A1 | 3000 | 12 | baseline — sanity replicate |
| A2 | 4000 | 12 | stretch #1 |
| A3 | 5000 | 15 | stretch #2, raise PI ceiling |
| A4 | 6000 | 18 | stretch #3 |
| A5 | 7000 | 20 | at hardware `iq_limit_a` ceiling |

Keep everything else at production values (`vehicle:=amoril|hosim`, `ol_target_rpm:=1600`, `cl_mode:=full`, `observer_mode:=0`).

**Protocol per run**
1. Fresh start, log Vbus.
2. `ros2 bag record -a -o bag_file/perf1A_<car>_maxspd<X>_<dir>_<ts>`.
3. Push stick fully (FWD); hold 10 s; release; wait 3 s.
4. If stable, a second 10 s hold; release.
5. Stop bag.
6. Repeat for REV in a new bag (always return to READY between).

**Metrics extracted from bags**
- `esc/speed_rpm` mean and std over the middle 6 s of each CL hold.
- `esc/iq_ma` mean and std; flag if > `iq_limit_a * 900 mA` (≈75 % saturation) sustained.
- `esc/swap_deferred` count of 1→1 intervals.
- `esc/vbus_v` delta from start to end of run.
- `cfoc_state` integrity (no transition other than READY↔FWD/REV↔CLOSED_LOOP while stick held).

**Pass criterion for a given `max_speed_rpm` setting:** motor reaches ≥ 90 % of setpoint RPM, speed std ≤ ±5 % of mean, zero faults, swap_deferred stays 0, Vbus droop < 0.5 V, same pattern both directions.

**Fail signatures and their meaning**
- Motor plateaus well below target → speed PI saturated; try A-next with higher `iq_limit_a`.
- Speed std > ±10 % → observer noise ceiling; stop pushing max_speed.
- `swap_deferred` spikes → observer lost lock; stop.
- Vbus > 0.5 V droop → battery-limited, not motor-limited; redo with full pack and note.

### 1.3 Sub-phase B — Observer SNR probe at high RPM (no firmware rebuild)

If Sub-phase A shows the motor reaches e.g. 5 kRPM without issue, add a short run that intentionally *reduces* `iq_limit_a` at high RPM to widen SNR margin (lower current, same BEMF → higher SNR) and see if that stabilizes observer noise further. Single-run validation, not a full matrix.

### 1.4 Sub-phase C — Firmware cap lift (first rebuild on this branch)

Only if A and B expose a real benefit from a higher `ol_target_rpm` handoff (better observer lock at start of CL because BEMF is already up).

**Edit:** [esc_comm.c:392](motor_test1_20260227/Src/esc_comm.c#L392) —

```c
// before:
(val >= 1000) && (val <= 2000)
// after (performance_test branch):
(val >= 1000) && (val <= 5000)
```

Also consider the PLL gain comment at [drive_parameters.h:57-58](motor_test1_20260227/Inc/drive_parameters.h#L57-L58): `PLL_KP_GAIN=638`, `PLL_KI_GAIN=18` were proven OK at 1500–1600 handoff but diverged at 1500 RPM previously with different gains — retune if we push `ol_target_rpm` > 2500.

Flash Release build, repeat A-matrix **with** `ol_target_rpm:=2500`, `3000`, `3500`. Same pass criteria.

**Commit gate on this branch:** only bump a commit when a sub-phase has at least one successful bag on both cars *and* the new settings are reproducible across cold-start + warm restart.

### 1.5 Sub-phase D — Gearing / linear-speed sanity

`max_speed_rpm` sets motor RPM, not wheel speed. Measure actual linear speed at the highest sustainable setting using an external reference (IMU double-integration is too noisy for this — prefer a straight-line visual timing test or GPS/SLAM if available).

Target: fill in `max_linear_mps` in the node's `proportional` CLI recipe with a real number, not the current placeholder `1.0 m/s`.

### 1.6 Phase 1 deliverables

1. Table of sustained RPM vs `max_speed_rpm` setpoint, per car, per direction.
2. Identified ceiling (motor, observer, or battery) with evidence.
3. Either: (a) node-only recommended `max_speed_rpm` for new production baseline, OR (b) firmware `ol_target_rpm` cap-lift patch + updated PLL gains + Release binary.
4. Updated `FIRMWARE_GUIDE.md` / `ESC_CONTROL_GUIDE.md` with the new ceiling numbers.

---

## Phase 2 — Torque / acceleration envelope (outline)

Focus: `iq_limit_a`, `ol_iq_a`, `boost_iq_a`, motor-timer-1 PWM headroom.

- Hypothesis: acceleration off-the-line is stiction-limited at low `ol_iq_a` and PI-limited in CL.
- Variables: `ol_iq_a` (to 20 A), `iq_limit_a` (to 20 A), `boost_iq_a` window (firmware default `ESC_TORQUE_RAMP_MS = 5 ms`).
- Key metric: `esc/speed_rpm` slope (dRPM/dt) during the first 500 ms of CL; peak `iq_ma`.
- Abort: coil whine, shaft judder, overcurrent fault (`0x40`).

To be detailed after Phase 1 results lock the RPM budget.

---

## Phase 3 — Thermal / duty cycle (outline)

Focus: sustained load over minutes, not seconds.

- Hypothesis: tested runs so far are <30 s; sustained 2+ min at high current will rise motor/MOSFET temp and raise Rs, which in turn destabilizes STO angle.
- Variables: duration, ambient airflow, current setpoint.
- Instrumentation: motor case thermocouple or IR spot; `esc/vbus_v` trend over time.
- Abort: case > 80 °C, ESC board smell, Vbus droop > 1 V from full pack.

---

## Phase 4 — Battery / range (outline)

Focus: how long a single pack lasts in realistic duty.

- Variables: different usage profiles (full throttle bursts vs steady cruise vs nav2 low-speed).
- Metric: Vbus vs wall-clock time; when does it cross 11.5 V.
- Integration: can we expose `esc/vbus_v` to the RPi5 battery indicator / auto-RTL logic.

---

## Phase 5 — Control-loop latency & jitter (outline)

Focus: end-to-end delay from `cmd_vel_stamped` to wheel response.

- Matters for future nav2 / MPC integration.
- Method: step `cmd_vel_stamped` with a known timestamp; look at `esc/command` echo and `esc/speed_rpm` lag.
- Variables: `cmd_hz` (20 → 50 → 100), `slew_rate`, UART baud (already near upper practical limit).

---

## Appendix — Baseline production values (frozen 2026-04-22)

For reference during sweeps. Any deviation in a bag filename / results entry.

| Param | AMORIL #1 | HOSIM |
|---|---|---|
| `vehicle` | `amoril` | `hosim` |
| `ol_iq_a` | 8.0 A (profile) | 15.0 A (profile) |
| `max_speed_rpm` | 3000 | 3000 |
| `iq_limit_a` | 12.0 | 12.0 |
| `ol_target_rpm` | 1600 | 1600 |
| `ol_ramp_ms` | 4000 | 4000 |
| `xf_duration_ms` | 200 | 200 |
| `xf_dwell_ms` | 100 | 100 |
| `cl_mode` | full | full |
| `observer_mode` | 0 (legacy STO) | 0 |

Firmware: `main @ c0cd5e6`, same binary both cars.
