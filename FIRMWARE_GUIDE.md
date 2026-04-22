# STM32G431 ESC Firmware — Operator Guide

Target board: **B-G431B-ESC1**.
Motor: small BLDC, 2 pole pairs, Rs=0.1 Ω, Ls=10 µH (Motor Pilot measured).
Role: UART-controlled motor node for an RC vehicle. Receives normalized command `u ∈ [-1, 1]` from a Raspberry Pi 5 and streams telemetry back.

> **Production baseline (2026-04-22):** branch `main`, commit `c0cd5e6`, `observer_mode=0` (legacy STO + discrete crossfade). Validated on both AMORIL #1 and HOSIM cars with the **same binary**.

---

## 1. Build configurations

Two STM32CubeIDE build configurations; selected by the `BUILD_ESC` preprocessor define.

| Configuration | `BUILD_ESC` | Purpose | UART behavior |
|---|---|---|---|
| **Release** | defined | Production ESC for RPi5 | ASPEP disabled; custom 5-byte command / 15-byte telemetry frames |
| **Debug** | *(undefined)* | Motor Pilot tuning | ASPEP enabled; Motor Pilot drives the motor |

To add `BUILD_ESC` in CubeIDE:
> Project → Properties → C/C++ Build → Settings → MCU GCC Compiler → Preprocessor → **Defined symbols** → add `BUILD_ESC` → set configuration to **Release** → Apply.

**Artifacts:**
- `STM32CubeIDE/Release/motor_test1_20260227.elf` — flash for RPi5 deployment
- `STM32CubeIDE/Debug/motor_test1_20260227.elf` — flash for Motor Pilot

---

## 2. Flash procedure

Same binary on both cars. Only the ROS 2 node's per-vehicle `ol_iq_a` changes.

1. Build the **Release** configuration in STM32CubeIDE.
2. Connect the ST-Link via SWD.
3. Flash via CubeIDE Run/Debug, or `STM32_Programmer_CLI` from the command line.
4. Power-cycle the ESC. The firmware comes up in `BOOT → WAIT_NEUTRAL`.

---

## 3. UART protocol

**Line:** USART2, **1 843 200 baud**, 8N1. DMA-free — direct-register polling TX, RXNE-interrupt RX.

### 3.1 Command frame (RPi5 → STM32, 5 bytes)
```
[0xAA] [cmd_lo] [cmd_hi] [reserved] [XOR_chk]
```
`cmd` is `int16_t`: −32768 = −1.0, +32767 = +1.0. `XOR_chk` = XOR of bytes 1..3.

### 3.2 Config frame (RPi5 → STM32, 5 bytes)
```
[0xCC] [param_id] [val_lo] [val_hi] [XOR_chk]
```
Same XOR as the command frame. Sent once at RPi5 node startup; firmware uses compile-time defaults if never received.

| ID | Name | Scaling | Range |
|---|---|---|---|
| 0x01 | `max_iq_a` | ×0.1 A | 10–150 |
| 0x02 | `revup_rpm` | RPM | 1600–5000 |
| 0x03 | `boost_iq_a` | ×0.1 A | 10–150 |
| 0x04 | `max_speed_rpm` | RPM | 1000–10000 |
| 0x05 | `iq_limit_a` | ×0.1 A | 10–200 |
| 0x06 | `ol_iq_a` | ×0.1 A | 20–150 |
| 0x07 | `ol_ramp_ms` | ms | 1000–8000 |
| 0x08 | `align_ms` | ms | 100–2000 |
| 0x09 | `align_id_a` | ×0.1 A | 10–150 |
| 0x0A | `xf_duration_ms` | ms | 100–1000 |
| 0x0B | `xf_dwell_ms` | ms | 50–500 |
| 0x0C | `ol_target_rpm` | RPM | 1000–2000 |
| 0x0D | `spd_kp` | ×0.001 A/RPM | 5–50 |
| 0x0E | `spd_ki` | ×0.0001 A/RPM | 5–50 |
| 0x0F | `spd_lpf_alpha` | ×0.0001 | 2–20 |
| 0x14 | `observer_mode` | 0=legacy / 1=adaptive-R | — |
| 0x15 | `ekf_omega_thresh` | elec rad/s | 50–400 |
| 0x16 | `ekf_r0` | A² ×10000 | 1–10000 |
| 0x17 | `ekf_qe` | ×10000 | 1–10000 |
| 0x18 | `ekf_vf_prior_lock` | 0/1 | — |

### 3.3 Telemetry frame (STM32 → RPi5, 21 bytes @ ~10 Hz)
```
[0xBB] [spd] [esc_st] [faults] [u] [v] [iq] [id] [mc_st]
       [cfoc_st] [innov] [kappa] [residual] [XOR_chk]
```
`spd` int16 RPM · `u` int16 raw · `v` uint16 (V × 10) · `iq`/`id` int16 mA · `innov` uint16 (A × 1000) · `kappa`/`residual` uint16 (× 10000).

---

## 4. Motor operation

### 4.1 Startup sequence
```
BOOT → WAIT_NEUTRAL → READY ↔ FORWARD/REVERSE (via BRAKE) → FAULT
```

1. Alignment: ≈500 ms current injection on d-axis (align_id_a).
2. Open-loop ramp: current held at `ol_iq_a`, frequency ramped over `ol_ramp_ms` to `ol_target_rpm`.
3. Crossfade: blend the open-loop V/f angle into the STO angle over `xf_duration_ms + xf_dwell_ms`.
4. Closed-loop: speed PI (`spd_kp`/`spd_ki`/`spd_lpf_alpha`) drives Iq to track the commanded RPM.

### 4.2 Observer

`observer_mode` (0xCC param 0x14):
- **0 — legacy STO** (production). V/f angle during OL, STO after crossfade.
- **1 — adaptive-R EKF** (dormant). Compiled in but dead path at `ol_target_rpm ≤ 2000` because BEMF SNR is below what the EKF can resolve on this motor. Do not enable without lifting the 2000 RPM cap and re-running a dedicated observer test.

### 4.3 Per-vehicle configuration

Same firmware. Only `ol_iq_a` changes.

| Vehicle | `ol_iq_a` | Reason |
|---|---|---|
| AMORIL #1 | **8.0 A** | Lighter drivetrain — 8 A breaks stiction cleanly. |
| HOSIM | **15.0 A** | Heavier drivetrain + ground friction; anything below ~12 A plateaus before reaching target RPM. |

Both cars use `ol_target_rpm=1600`, `xf_duration_ms=200`, `xf_dwell_ms=100`, `cl_mode=full`.

---

## 5. Safety behavior

- `esc_cmd_value` initializes to `0x7FFF` (non-neutral) — `WAIT_NEUTRAL` blocks until the host sends explicit neutral. Prevents motor start on firmware boot.
- Direction reversal (FORWARD ↔ REVERSE) always passes through `BRAKE`; motor must reach MCSDK `IDLE` before restarting.
- No command for 500 ms → motor stops (timeout watchdog).
- Any MCSDK fault → `FAULT`; auto-acknowledges on `FAULT_OVER` → back to `WAIT_NEUTRAL`.
- Wrong-angle detect (speed sign disagrees with commanded direction in RUN) → stop + 500 ms back-off + auto-retry, no joystick release needed.

---

## 6. Tuning with Motor Pilot

1. Build **Debug** configuration (`BUILD_ESC` *not* defined). ASPEP is enabled.
2. Flash via ST-Link.
3. Open Motor Pilot, connect to the board.
4. Tune observer, PI gains, rev-up phases as needed.
5. Export changes back into `drive_parameters.h` / `pmsm_motor_parameters.h`.
6. Rebuild Release and re-flash for deployment.

> Do not leave Debug flashed on a deployed car — the ESC communication layer is inactive and the RPi5 node will time out.

---

## 7. Known issues & closed paths

- **Adaptive-R EKF closed 2026-04-20** at `ol_target_rpm ≤ 2000`. Two independent blockers (θ-swap guard locking on HOSIM, `lock_kappa` stuck at 0 on AMORIL). Re-opening requires lifting the firmware RPM cap and a dedicated observer session.
- **Thermal/battery degradation** of legacy STO after ~3 h of aggressive testing is physical, not a firmware bug: warm motor → higher Rs → noisier STO angle → speed PI chases. Recovery is cool + fresh pack; no code change needed.

---

## 8. Source map

| File | Role |
|---|---|
| `motor_test1_20260227/Src/mc_app_hooks.c` | ESC state machine, 1 kHz hook |
| `motor_test1_20260227/Src/esc_comm.c` | UART RX/TX, 0xCC param parser |
| `motor_test1_20260227/Src/esc_app.c` | 21-byte telemetry, param getters |
| `motor_test1_20260227/Src/custom_foc.c` | Custom FOC inner loop, adaptive-R EKF path |
| `motor_test1_20260227/Src/esc_ekf_observer.c` | EKF predict/update |
| `motor_test1_20260227/Inc/drive_parameters.h` | PWM freq, observer gains, rev-up profile, `USE_ADAPTIVE_R_EKF` |
| `motor_test1_20260227/Inc/pmsm_motor_parameters.h` | Motor electrical params |
