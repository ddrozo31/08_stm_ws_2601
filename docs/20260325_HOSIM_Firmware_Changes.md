# HOSIM Firmware Changes — Summary

**Branch:** `HOSIM`
**Date:** 2026-03-25
**Baseline:** `baseline/AMORIL` tag (AMORIL #1 verified working, smooth hard floor)

---

## Context

The HOSIM RC car uses the same motor (2852 3100KV) as AMORIL #1 but has a stiffer,
heavier drivetrain. The original AMORIL baseline firmware failed on HOSIM because:

1. **Drivetrain stiction** — phase currents too low to spin the HOSIM drivetrain from rest.
2. **Observer SNR problem** — at SWITCH_OVER, `Rs×I > BEMF`, causing the STO to converge
   to the 180° wrong-angle solution → motor sounds odd, wheels erratic, then stops.

The SNR constraint is: `SNR = BEMF / (Rs×I) = (Ke × RPM) / (Rs × I)` must be ≥ 1.0
for reliable observer lock. At AMORIL #1 working point (6A, 2800 RPM): SNR = 1.17 ✓.

---

## Changes in `drive_parameters.h`

### Phase Profile

| Parameter | AMORIL #1 baseline | HOSIM current | Reason |
|-----------|-------------------|---------------|--------|
| PHASE1_DURATION | 1200 ms | **1500 ms** | Longer alignment under higher friction |
| PHASE1_FINAL_CURRENT_A | 4.0 A | **8.0 A** | Break HOSIM drivetrain stiction |
| PHASE2_FINAL_CURRENT_A | 6.0 A | **8.0 A** | Sustain motion against heavier load |
| PHASE3_DURATION | 1200 ms | **1500 ms** | Slower ramp |
| PHASE3_FINAL_SPEED | 1500 RPM | **1000 RPM** | Reduced intermediate target |
| PHASE3_FINAL_CURRENT_A | 6.0 A | **8.0 A** | Higher current for HOSIM |
| PHASE4_DURATION | 1800 ms | **2500 ms** | Slower ramp (240 RPM/s) |
| PHASE4_FINAL_SPEED | 2000 RPM | **1600 RPM** | Reachable target at HOSIM load |
| PHASE4_FINAL_CURRENT_A | 8.0 A | **10.0 A** | Peak current before step-down |
| PHASE5_DURATION | 2000 ms | **4000 ms** | Long stabilization before SWITCH_OVER |
| PHASE5_FINAL_SPEED | 2800 RPM | **1700 RPM** | Lowered to match HOSIM capability at 6A |
| PHASE5_FINAL_CURRENT_A | 6.0 A | **6.0 A** | Step-DOWN from 10A for better observer SNR |

**Phase 5 strategy:** Step DOWN from 10A (phase 4) to 6A in phase 5 to improve observer SNR
before SWITCH_OVER. The higher current in phases 1–4 overcomes stiction; phase 5 prioritizes
BEMF-to-noise ratio for reliable observer lock.

### Observer / SWITCH_OVER

| Parameter | AMORIL #1 baseline | HOSIM current | Reason |
|-----------|-------------------|---------------|--------|
| OBS_MINIMUM_SPEED_RPM | 2500 | **1700** | Match new phase 5 target; HOSIM cannot reach 2500 at 6A |
| NB_CONSECUTIVE_TESTS | 4 | **12** | Observer must be stable 12ms before SWITCH_OVER |
| TRANSITION_DURATION | 100 ms | **50 ms** | Halved to reduce speed drop during open→closed-loop blend (~325 RPM vs ~650 RPM) |

---

## Changes in `mc_app_hooks.c`

| Parameter | AMORIL #1 baseline | HOSIM current | Reason |
|-----------|-------------------|---------------|--------|
| ESC_REVUP_SPEED_RPM | 2500.0f | **1700.0f** | Must always match OBS_MINIMUM_SPEED_RPM |

---

## Status History

### Iteration 1 (2026-03-25)

| Issue | Status |
|-------|--------|
| Drivetrain stiction (phases 1–4) | ✓ Solved — wheels spin during ramp |
| Phase 4/5 grinding (too fast ramp) | ✓ Solved — 240 RPM/s ramp, lower targets |
| SWITCH_OVER never firing (OBS speed unreachable) | ✓ Solved — fires at 1700 RPM |
| Post-SWITCH_OVER crash (180° wrong-angle) | ✗ Ongoing → see iter 2 |

**Root cause of iter 1 failure:**
At phase 5 exit (6A, 1700 RPM): BEMF = 0.425 V, Rs×I = 0.600 V → **SNR = 0.71** (< 1.0).
The STO observer locked to the 180° wrong-angle solution → motor entered RUN with inverted
flux angle → erratic motion → ANY_STOP.

---

### Iteration 2 (2026-03-26) — Applied

Changes applied to firmware:
- `PHASE5_FINAL_CURRENT_A`: 6.0 → **4.0 A**
- `PHASE5_FINAL_SPEED_UNIT`: 1700 → **1600 RPM**
- `PHASE5_DURATION`: 4000 → **5000 ms**
- `OBS_MINIMUM_SPEED_RPM`: 1700 → **1600**
- `ESC_REVUP_SPEED_RPM`: 1700.0f → **1600.0f**

At 4A, 1600 RPM: BEMF = 0.4 V, Rs×I = 0.4 V → SNR = **1.0 (breakeven)**.

**Risk:** HOSIM speed plateau at 6A was 1602–1668 RPM — margin is tight for sustaining
1600 RPM at only 4A under drivetrain load. If the motor drops below 1600 RPM in phase 5,
SWITCH_OVER will not fire.

**Parallel recommendation:** Lubricate HOSIM differentials and driveshafts. This would
allow the motor to reach 2800 RPM at 6A (SNR = 1.17), the proven AMORIL #1 working point.

---

## SNR Quick Reference

| Condition | Current | RPM | BEMF | Rs×I | SNR | Result |
|-----------|---------|-----|------|------|-----|--------|
| AMORIL #1 (working) | 6 A | 2800 | 0.700 V | 0.600 V | 1.17 | ✓ |
| HOSIM attempt (12A) | 12 A | 2000 | 0.500 V | 1.200 V | 0.42 | ✗ |
| HOSIM iter 1 | 6 A | 1700 | 0.425 V | 0.600 V | 0.71 | ✗ |
| HOSIM iter 2 (current) | 4 A | 1600 | 0.400 V | 0.400 V | 1.00 | ? |

`Ke = 0.25 V/kRPM`, `Rs = 0.1 Ω`, `Pole pairs = 2`

---

## Multi-Car Robustness Layer (2026-03-26)

Alongside iter 2, a runtime configuration system was added to avoid reflashing for per-car tuning.

### New UART Config Frame (0xCC)

```
[0xCC] [param_id] [val_lo] [val_hi] [XOR of bytes 1..3]
```

| param_id | Parameter    | Encoding              | HOSIM | AMORIL |
|----------|--------------|-----------------------|-------|--------|
| 0x01     | max_iq_a     | int16 × 0.1 A         | 100 (10.0 A) | 120 (12.0 A) |
| 0x02     | revup_rpm    | int16 RPM             | 1600 | 2500 |
| 0x03     | boost_iq_a   | int16 × 0.1 A         | 100 (10.0 A) | 120 (12.0 A) |

Firmware falls back to compile-time defaults if no 0xCC frame is received.

### Wrong-Angle Auto-Retry

Previously: wrong-angle detection (speed sign mismatch after SWITCH_OVER) → `ESC_READY` → requires joystick release + re-push.

Now: stays in `ESC_FORWARD`/`ESC_REVERSE`, sets `esc_restart_delay = 500 ms`, then IDLE auto-restart path re-triggers the full rev-up transparently. No user action required.

### ROS2 Node: `esc_node_trigger_config.py`

New node in `~/mrad_ws_2601_zulu/src/zulu_esc/zulu_esc/`. Adds:
- `vehicle_profile` param: `'HOSIM'` or `'AMORIL'` applies preset values in one arg
- Sends 0xCC config frames at startup before first drive command
- Wrong-angle retries are transparent (no joystick changes needed)

```bash
# HOSIM
ros2 run zulu_esc esc_node_trigger_config --ros-args -p vehicle_profile:=HOSIM

# AMORIL
ros2 run zulu_esc esc_node_trigger_config --ros-args -p vehicle_profile:=AMORIL
```
