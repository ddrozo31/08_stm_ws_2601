# custom_foc Branch — Firmware Reference (Step 6 closeout, 2026-04-08)

This document is the handoff reference for the `custom_foc` branch at the close of Step 6.
It captures the final parameter set, operating envelope, and the items most worth revisiting
when the project moves toward production.

For working notes and session-by-session history see [`custom_foc_plan.md`](custom_foc_plan.md).
For the user-facing operating manual see [`ESC_User_Guide.md`](ESC_User_Guide.md).

---

## 1. Hardware target

| Item | Value |
|---|---|
| MCU | STM32G431CBU @ 170 MHz |
| Board | B-G431B-ESC1 |
| Motor (validated) | AMORIL SPMSM, 2 pole pairs, Rs=0.1 Ω, Ls=10 µH, ψf=9.75e-4 Wb |
| Battery | 3S LiPo, nominal 11.1 V (12.4–12.9 V observed in ground tests) |
| Comm link | USART2, 1843200 baud, RPi5 ↔ STM32 |

---

## 2. Build configurations

| Config | `BUILD_ESC` define | Purpose |
|---|---|---|
| Release | defined | RPi5 deployment — ESC comm + state machine, ASPEP off |
| Debug   | undefined | Motor Pilot tuning — ASPEP on, ESC layer inactive |

Compiler: `-Ofast -g3 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mcpu=cortex-m4`

Build artifacts: `STM32CubeIDE/{Release,Debug}/motor_test1_20260227.elf`

---

## 3. Control architecture

```
TIM1 ADC ISR (25 kHz)         SysTick (1 kHz)              ESC layer (1 kHz)
─────────────────────         ─────────────────             ──────────────────
CFOC_HighFrequencyTask    →   CFOC_MediumFrequencyTask  →  ESC_APP_Tick
  • ADC → Iα Iβ                 • State machine             • UART RX/TX
  • Park (CORDIC sin/cos)       • Speed PI                  • Cmd → CFOC_SetSpeed
  • EKF predict + update        • Vbus update (10 Hz)       • Telemetry @ 10 Hz
  • Current PI (Iq, Id)         • Runtime config apply
  • SVM + OVM clamp
  • PWM update
```

State machines:
- **CFOC** (`custom_foc.c`): `IDLE → ALIGNMENT → OPEN_LOOP → CROSSFADE → CLOSED_LOOP` (+ `FAULT`)
- **ESC**  (`esc_app.c`):    `BOOT → WAIT_NEUTRAL → READY ↔ FORWARD/REVERSE (via BRAKE) → FAULT`

Production control mode: **speed mode**. ESC sets `u × esc_max_spd_rpm` once CFOC reaches
CLOSED_LOOP; the speed PI in CFOC regulates Iq.

---

## 4. Final parameter set

### 4.1 Sample rates and timing

| Define | Value | Notes |
|---|---|---|
| `CFOC_PWM_FREQ_HZ` | 25 000 | Center-aligned, ARR = 3400 |
| `CFOC_TS` | 40 µs | HF task period |
| `CFOC_MF_TS` | 1 ms | MF task period |
| `CFOC_DEAD_TIME_NS` | 800 (total, 400/edge) | Vdt ≈ 0.24 V at Vbus = 12.5 V |

### 4.2 ADC / current sensing

| Define | Value |
|---|---|
| `CFOC_RSHUNT` | 0.003 Ω |
| `CFOC_AMP_GAIN` | 9.14 |
| `CFOC_ADC_TO_AMPS` | 0.02938 A/count |
| `CFOC_VBUS_RATIO` | 0.09625 |
| `CFOC_CALIB_SAMPLES` | 64 (boot) |
| `CFOC_OFFSET_EMA_ALPHA` | 0.002 (continuous IDLE tracking, τ ≈ 20 ms) |

### 4.3 Startup (alignment + open loop)

| Define | Value | Notes |
|---|---|---|
| `CFOC_ALIGN_MS` | 300 | Compile-time default; runtime overridable |
| `CFOC_ALIGN_ID` | 3.0 A | Compile-time default; runtime overridable |
| `CFOC_OL_RAMP_MS` | 3000 | Speed ramp duration |
| `CFOC_OL_TARGET_RPM` | **1400** | Crossfade handoff speed (Session 3 baseline) |
| `CFOC_OL_IQ_RAMP_MS` | 500 | Current ramp duration |
| `CFOC_OL_IQ_TARGET` | 5.0 A | OL Iq target |
| `CFOC_OL_ID_REF` | 0.0 A | SPMSM → no flux weakening |
| `CFOC_OL_MAX_MS` | 8000 | Safety timeout |

At 1400 RPM: BEMF = 0.286 V, Vdt ≈ 0.24 V → **Vdt/BEMF ≈ 84% (16% margin).**

### 4.4 Crossfade

| Define | Value | Notes |
|---|---|---|
| `CFOC_XF_BEMF_SQ_THRESH` | 0.05 V² | Guard threshold (~1100 RPM equivalent) |
| `CFOC_XF_DWELL_MS` | 200 | BEMF must exceed threshold this long |
| `CFOC_XF_DURATION_MS` | 500 | Linear blend of OL and EKF angle |

CL entry seeds `pi_spd.integral = 0`, `spd_cmd_rpm = CFOC_OL_TARGET_RPM × ol_direction`.
Bumpless transfer is **not** done — see §6 production-review item 1.

### 4.5 EKF (25 kHz)

| Define | Value | Notes |
|---|---|---|
| `CFOC_EKF_Q_I` | 1.33e-4 A² | Current process noise |
| `CFOC_EKF_Q_E` | 1.33e-3 V² | BEMF process noise |
| `CFOC_EKF_R_I` | 3.33e-3 A² | Measurement noise |
| `CFOC_EKF_SPEED_LPF_ALPHA` | 0.002 | τ ≈ 20 ms — drives commutation angle |
| `CFOC_EKF_SPD_PI_LPF_ALPHA` | 0.0004 | τ ≈ 100 ms — drives speed PI feedback |

Q values are scaled for 25 kHz operation (continuous Q × Ts). Sin/cos comes from CORDIC (Session 1).

### 4.6 Current PI (Iq, Id)

| Define | Value | Notes |
|---|---|---|
| `CFOC_PI_IQ_KP` | 0.063 | = Ls × ωc, ωc = 2π × 1000 rad/s |
| `CFOC_PI_IQ_KI` | 0.025 | Discretized (Ki × Ts), continuous = 628 |
| `CFOC_PI_ID_KP` | 0.063 | Same plant |
| `CFOC_PI_ID_KI` | 0.025 | |
| `CFOC_PI_VMAX_PER_VBUS` | 2/π ≈ 0.6366 | Six-step OVM ceiling |

PI `out_max` is updated every MF tick to `cfoc_vbus_rt × CFOC_PI_VMAX_PER_VBUS` so it tracks
battery discharge. Circle limiter in HF task uses the same runtime ceiling.

### 4.7 Speed PI (1 kHz)

| Define | Value | Notes |
|---|---|---|
| `CFOC_PI_SPD_KP` | 0.01 A/RPM | 100 RPM error → 1 A correction |
| `CFOC_PI_SPD_KI` | 0.001 A/(RPM·s) | Discretized (× MF_Ts) |
| `CFOC_PI_SPD_IQ_MAX` | 10.0 A | Default; runtime overridable via 0xCC |

### 4.8 ESC layer

| Define | Value | Notes |
|---|---|---|
| `ESC_NEUTRAL_DEADBAND` | 0.02 | |u| below this = neutral |
| `ESC_DEFAULT_MAX_SPD_RPM` | 5000 | Runtime overridable via 0xCC |
| `ESC_DEFAULT_IQ_LIMIT_A` | 10.0 | Runtime overridable via 0xCC |
| `ESC_TIMEOUT_MS` | 500 | No-command auto-stop |
| `ESC_RESTART_DELAY_MS` | 500 | Back-off between auto-restarts |
| `ESC_TELEMETRY_EVERY` | 100 | 10 Hz telemetry |
| `ESC_WRONG_ANGLE_RPM` | 50 | Speed sign mismatch threshold |

### 4.9 Runtime-configurable parameters (UART 0xCC frames)

Sent once at startup by RPi5; firmware uses compile-time defaults if never received.

| param_id | Field | Range | Source |
|---|---|---|---|
| 0x01 | `max_iq_a` | 1.0–15.0 A | CFOC_SetIqLimit |
| 0x02 | `max_speed_rpm` | 100–8000 RPM | esc_max_spd_rpm |
| 0x03 | `ol_iq_a` | 1.0–10.0 A | CFOC_SetStartupParams |
| 0x04 | `ol_ramp_ms` | 500–5000 ms | CFOC_SetStartupParams |
| 0x05 | `align_ms` | 100–1000 ms | CFOC_SetStartupParams |
| 0x06 | `align_id_a` | 1.0–10.0 A | CFOC_SetStartupParams |

---

## 5. Operating envelope (hardware verified)

| Quantity | Range | Source |
|---|---|---|
| Top speed (joystick max) | ~5000 RPM | ground test bag `rosbag2_2026_04_07-18_19_36` |
| Sustained Iq (full throttle) | ±12 A | bag `rosbag2_2026_04_08-11_02_43` |
| Crossfade handoff | 1400 RPM | Session 3 baseline |
| Forward & reverse | both | confirmed Session 3 onward |
| Battery range | 11.0–12.9 V | observed across all ground bags |
| OL→CL rough rate (under load) | ~5% | residual; observer-side, see Step 8 |
| Steady-state Iq stdev (Session 2 vs S1) | −81% | bag-derived |

**Failure modes that are by-design:**
- **Stall fault**: if axle is held by external load, observer loses lock and motor faults out.
  This is a physics constraint of any sensorless drive without HFI.
- **Wrong-angle retry**: if EKF locks 180° off, the firmware detects via speed-sign mismatch,
  stops, waits 500 ms, and auto-retries the same direction.
- **OL→CL rough transitions under load**: ~5% rate; observer-side, see §6 item 1.

---

## 6. Production-review items (ranked by likely payoff)

These are not bugs in the current branch — they are the highest-leverage things to look at
when the project moves toward production. Each is a self-contained experiment.

1. **Adaptive-R EKF observer rebuild (Step 8)**
   Replace the discrete OL → CROSSFADE → CL state machine with a single continuous EKF that
   uses adaptive measurement noise: `R(t) = R₀ × max(1, BEMF_thresh² / BEMF_est²)`. Eliminates
   the death-spiral failure mode under load. **Highest payoff, highest effort (1–2 weeks).**
   Validation target: bag `rosbag2_2026_04_08-10_23_17` CL #4 must not death-spiral.

2. **Speed PI feedback bandwidth**
   PI currently sees τ = 100 ms (`CFOC_EKF_SPD_PI_LPF_ALPHA = 0.0004`). May be too slow during
   the OL → CL transient. Cheap experiment: try τ = 20–50 ms (α = 0.001–0.002) and re-bag.
   Risk: more noise on PI input → more torque ripple at steady state.

3. **Voltage ceiling tradeoff (OVM vs linear SVM)**
   Currently `CFOC_PI_VMAX_PER_VBUS = 2/π` (six-step OVM, ~7.96 V at Vbus=12.5 V). Rolling back
   to linear SVM (`1/√3` ≈ 7.21 V) would lose Iq smoothness from Session 2 but might improve
   OL→CL robustness by keeping the modulator out of nonlinear waveform territory.

4. **Crossfade speed margin**
   1400 RPM gives 16% Vdt/BEMF margin. 1600 RPM gives 27% margin and was the Session 1–2 baseline.
   Cost: ~0.4 s longer startup. Worth re-evaluating once Step 8 is in place.

5. **Hardware dead-time reduction**
   `CFOC_DEAD_TIME_NS = 800` is conservative. Shrinking the DTG register lowers Vdt and improves
   observability margin at every speed. Bench-validate first — too low risks shoot-through.

6. **ADC offset persistence**
   Offsets are recalibrated every boot from 64 samples (`CFOC_CALIB_SAMPLES`). Persisting the last
   known-good offset to flash and using it as a sanity check could shorten boot and detect
   sensor drift between sessions.

7. **Telemetry rate**
   Currently 10 Hz (`ESC_TELEMETRY_EVERY = 100`). Nav stack closed-loop control may want 50 Hz
   for tighter velocity feedback. UART has the bandwidth (15 bytes × 50 Hz × 10 bits = 7.5 kbps,
   negligible at 1.84 Mbaud).

8. **Fault history logging**
   `CFOC_AckFault` currently auto-clears on FAULT_OVER. Production should preserve a fault count
   and last-fault-reason in flash for field diagnosis.

9. **CAN bus link option**
   UART works in the lab. CAN would be more robust for vehicle deployment with EMI from the
   power stage. The G431 has FDCAN; would require RPi5-side CAN HAT.

10. **Cogging compensation**
    Could improve low-speed smoothness if the application demands precise parking maneuvers.
    Not currently a problem because the operating regime is "drive, not crawl."

11. **Field weakening**
    Not implemented (`CFOC_OL_ID_REF = 0`). Would extend top speed beyond the back-EMF-limited
    point if the application demands it.

---

## 7. Files of record

| File | Role |
|---|---|
| [`motor_test1_20260227/Inc/custom_foc.h`](../motor_test1_20260227/Inc/custom_foc.h) | All `#define`s in §4 above |
| [`motor_test1_20260227/Src/custom_foc.c`](../motor_test1_20260227/Src/custom_foc.c) | FOC, EKF, state machine, PI, SVM |
| [`motor_test1_20260227/Src/esc_app.c`](../motor_test1_20260227/Src/esc_app.c) | ESC layer state machine + telemetry |
| [`motor_test1_20260227/Src/esc_comm.c`](../motor_test1_20260227/Src/esc_comm.c) | UART RX/TX (0xAA cmd / 0xCC config / 0xBB telemetry) |
| [`docs/custom_foc_plan.md`](custom_foc_plan.md) | Working plan + session history |
| [`docs/ESC_User_Guide.md`](ESC_User_Guide.md) | User-facing operating manual |
| [`tests/analyze_cl_transition.py`](../tests/analyze_cl_transition.py) | Bag analyzer for OL→CL transitions |

---

## 8. Branch state at closure

- Branch: `custom_foc` @ commit `0724cc7` ("Step 6 closed: revert Session 4/5, defer observer
  rewrite to Step 8")
- Pushed to `origin/custom_foc`
- 98 commits ahead of `main` (Steps 1–6 of the custom-FOC rewrite)
- **Next work moves to the `mrad_ws_2601_zulu` repo on a new `nav_rc_car` branch** for ROS2
  nav stack integration (Step 7). Firmware on `custom_foc` is the stable foundation —
  no further changes expected until Step 8 (observer rebuild) starts post-Step 7.
