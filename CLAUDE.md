# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32G431-based BLDC motor controller for an RC vehicle, targeting the B-G431B-ESC1 board. Uses ST Motor Control SDK v6.4.1 (FOC, sensorless). The board acts as a UART-controlled motor node receiving normalized commands `u ∈ [-1, 1]` from a Raspberry Pi 5 and streaming back telemetry (estimated speed, state, faults).

## Build Configurations

Two STM32CubeIDE build configurations, selected via the `BUILD_ESC` preprocessor define:

| Configuration | Define | Purpose |
|---------------|--------|---------|
| **Release** | `BUILD_ESC` defined | RPi5 ESC firmware — ASPEP disabled, ESC comm layer active, optimized |
| **Debug** | *(not defined)* | Motor Pilot tuning — ASPEP enabled, ESC layer inactive, debug symbols |

**To add `BUILD_ESC` to Release in STM32CubeIDE:**
> Project → Properties → C/C++ Build → Settings → MCU GCC Compiler → Preprocessor → Defined symbols → add `BUILD_ESC` → set configuration to "Release" → Apply.

Build artifacts:
- `STM32CubeIDE/Release/motor_test1_20260227.elf` — flash for RPi5 deployment
- `STM32CubeIDE/Debug/motor_test1_20260227.elf` — flash for Motor Pilot tuning

**Key compiler flags:** `-Ofast -g3 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mcpu=cortex-m4`
**Base defines:** `ARM_MATH_CM4`, `USE_HAL_DRIVER`, `STM32G431xx`

**What each `BUILD_ESC` guard controls:**

| File | `#ifdef BUILD_ESC` (Release) | `#ifndef BUILD_ESC` (Debug) |
|------|------------------------------|------------------------------|
| `mc_tasks.c` | `ASPEP_start()` skipped | `ASPEP_start()` called |
| `mc_app_hooks.c` | Full ESC state machine + UART init | Minimal stubs, Motor Pilot drives motor |
| `stm32_mc_common_it.c` | `ESC_COMM_UART_RxISR()` + flag clears | ASPEP IRQ handler runs normally |

## ESC Communication Layer (esc_comm)

Two files own USART2 exclusively — ASPEP/MCP is disabled:

| File | Role |
|------|------|
| `Inc/esc_comm.h` | Public interface: `ESC_COMM_Init`, `GetCommand`, `HasNewCommand`, `SendTelemetry`, `GetMaxIqA`, `GetRevupRPM`, `GetBoostIqA` |
| `Src/esc_comm.c` | RXNE interrupt-driven RX parser (0xAA + 0xCC frames) + direct-register blocking TX |

**Command frame (RPi5 → STM32, 5 bytes):** `[0xAA][cmd_lo][cmd_hi][reserved][XOR_chk]`
- `cmd` is `int16_t` scaled: −32768 = −1.0, +32767 = +1.0

**Config frame (RPi5 → STM32, 5 bytes):** `[0xCC][param_id][val_lo][val_hi][XOR_chk]`
- Same 5-byte structure and XOR-of-bytes-1..3 checksum as the command frame
- `param_id 0x01` = max_iq_a: `int16_t` × 0.1 A (e.g. 120 = 12.0 A, range 10–150)
- `param_id 0x02` = revup_rpm: `int16_t` RPM (range 1600–5000)
- `param_id 0x03` = boost_iq_a: `int16_t` × 0.1 A (range 10–150)
- `param_id 0x04` = max_speed_rpm: `int16_t` RPM (range 1000–10000)
- `param_id 0x05` = iq_limit_a: `int16_t` × 0.1 A (range 10–200)
- `param_id 0x06` = ol_iq_a: `int16_t` × 0.1 A (range 20–150)
- `param_id 0x07` = ol_ramp_ms: `int16_t` ms (range 1000–8000)
- `param_id 0x08` = align_ms: `int16_t` ms (range 100–2000)
- `param_id 0x09` = align_id_a: `int16_t` × 0.1 A (range 10–150)
- `param_id 0x0A` = xf_duration_ms: `int16_t` ms (crossfade blend, range 100–1000)
- `param_id 0x0B` = xf_dwell_ms: `int16_t` ms (crossfade dwell, range 50–500)
- `param_id 0x0C` = ol_target_rpm: `int16_t` RPM (OL/crossfade speed, range 1000–2000)
- `param_id 0x0D` = spd_kp: `int16_t` × 0.001 A/RPM (speed PI Kp, range 5–50)
- `param_id 0x0E` = spd_ki: `int16_t` × 0.0001 A/RPM (speed PI Ki, range 5–50)
- `param_id 0x0F` = spd_lpf_alpha: `int16_t` × 0.0001 (speed PI LPF, range 2–20)
- Sent once at startup by RPi5; firmware uses compile-time defaults if never received
- Values stored in `volatile float` RAM vars; read by `esc_app.c` on every tick

**Telemetry frame (STM32 → RPi5, 15 bytes):** `[0xBB][spd_lo][spd_hi][esc_st][faults][u_lo][u_hi][v_lo][v_hi][iq_lo][iq_hi][id_lo][id_hi][mc_st][XOR_chk]`
- `speed` = `int16_t` RPM, `esc_st` = ESC_State_t byte, `faults` = lower byte of fault bitmask
- `u` = `int16_t` raw command (−32768..+32767), `v` = `uint16_t` DC bus voltage in tenths of Volts (e.g. 126 = 12.6 V)
- `iq_ma` = `int16_t` q-axis actual current in mA, `id_ma` = `int16_t` d-axis current in mA
- `mc_st` = `uint8_t` MCSDK internal state (IDLE=0, START=4, RUN=6, ANY_STOP=8, SWITCH_OVER=19, etc.)

**Key choices:**
- RX: RXNEIE enabled directly via `USART2->CR1`; `ESC_COMM_UART_RxISR()` called from `USART2_IRQHandler` USER CODE BEGIN 0; self-synchronising SOF search accepts both 0xAA and 0xCC. HAL_UART_Receive_IT cannot be used — the project's custom LL-based `USART2_IRQHandler` never calls `HAL_UART_IRQHandler`.
- TX: direct register polling (TXE per byte, no TC wait); 10 bytes ≈ 43 µs at 1843200 baud.
- `ASPEP_start()` is commented out in `mc_tasks.c` — re-enable for Motor Pilot tuning.
- All ASPEP-sensitive USART flags (TC, ORE, FE, NE, IDLE) are cleared in USER CODE BEGIN 0 before the generated ASPEP handler runs, preventing `ASPEP_HWDataTransmittedIT` / `ASPEP_HWReset` from being called with uninitialised `aspepOverUartA`.

## Application Layer (mc_app_hooks.c)

`MC_APP_PostMediumFrequencyHook_M1()` runs every ~1 ms and implements the full ESC state machine:

**States:** `BOOT` → `WAIT_NEUTRAL` → `READY` ↔ `FORWARD` / `REVERSE` (via `BRAKE`) → `FAULT`

**Control mode: torque (current) mode**
Rev-up always uses MCSDK open-loop phases 1–5. Once MCSDK reaches RUN, the ESC
switches to `MC_ProgramTorqueRampMotor1_F(u * cfg_max_iq_a, ESC_TORQUE_RAMP_MS)`.

**Key parameters (mc_app_hooks.c) — compile-time defaults, all runtime-overridable via UART config frame:**
- `ESC_REVUP_SPEED_RPM 1600.0f` — open-loop rev-up target (HOSIM branch); must match `OBS_MINIMUM_SPEED_RPM`
- `ESC_REVUP_RAMP_MS 3000U` — speed ramp during rev-up (3000 ms = 533 RPM/s, avoids field slip under load)
- `ESC_MAX_IQ_A 12.0f` — default |u|=1.0 torque (overridden at runtime via `ESC_CFG_PARAM_MAX_IQ`)
- `ESC_BOOST_IQ_A 12.0f` — RUN-entry boost current for 300 ms (overridden via `ESC_CFG_PARAM_BOOST_IQ`)
- `ESC_TORQUE_RAMP_MS 5U` — near-instant torque ramp on RUN entry
- `ESC_NEUTRAL_DEADBAND 0.02f` — |u| below this = neutral
- `ESC_TIMEOUT_MS 500U` — stop motor if no command for 500 ms
- `ESC_TELEMETRY_EVERY 100U` — send telemetry every 100 calls (~10 Hz)
- `ESC_RESTART_DELAY_MS 500U` — back-off between auto-restarts (wrong-angle retry or observer loss)

**Runtime-configurable params (UART 0xCC frames, sent by RPi5 at node startup):**
- `cfg_max_iq_a` — max torque current; HOSIM default 10 A, AMORIL default 12 A
- `cfg_boost_iq_a` — RUN-entry boost; same defaults as max_iq_a
- `cfg_revup_rpm` — rev-up target; HOSIM 1600 RPM, AMORIL 2500 RPM

**Sensorless operating limits (HOSIM branch — hardware tuning in progress 2026-03-26):**
- **Observer handoff speed: 1600 RPM** — `OBS_MINIMUM_SPEED_RPM = 1600` (HOSIM iter2; AMORIL baseline was 2500)
- **SNR constraint:** `SNR = (Ke × RPM) / (Rs × I) ≥ 1.0` required for correct observer lock; at 1600 RPM/4A: SNR = 1.0 (breakeven)
- **Phase 5:** 4 A, 1600 RPM, 5000 ms — step-down from phase 4 (10 A) to reduce Rs×I before SWITCH_OVER
- **SWITCH_OVER (MCSDK_19):** 50 ms blend (halved from 100 ms); speed drop ~325 RPM
- **Wrong-angle retry:** if STO locks to 180° wrong angle (speed sign inverted), firmware stops, sets 500 ms back-off, and auto-restarts — no joystick release required
- **Full stall faults** — if axle stopped by external load, observer loses lock; physics constraint of sensorless FOC

**Safety rules enforced:**
- `esc_cmd_value` initialised to `0x7FFF` (non-neutral) — WAIT_NEUTRAL blocks until host explicitly sends neutral
- Direction reversal (FORWARD↔REVERSE) always passes through BRAKE; motor must reach IDLE before restarting
- Timeout watchdog stops motor in FORWARD/REVERSE if no command for 500 ms
- Any MCSDK fault → ESC_FAULT; auto-ack on FAULT_OVER → back to WAIT_NEUTRAL
- Wrong-angle detected (speed sign mismatch in RUN) → stop + 500 ms back-off + auto-restart in same direction

## No Automated Tests

This is embedded firmware. Testing is hardware-based:
1. Build → flash via ST-Link (SWD)
2. Use **Motor Pilot** (ST GUI) for tuning and motor verification
3. Send UART commands from RPi5 at 1843200 baud

## Architecture

### Firmware layers

```
Application layer      main.c, mc_app_hooks.c, mc_tasks.c
Communication layer    aspep.c, mcp.c, usart_aspep_driver.c, hf_registers.c
Motor Control SDK      mc_interface.c, mc_tasks_foc.c, speed_torq_ctrl.c, pwm_curr_fdbk.c
HAL / CMSIS            STM32G4xx_HAL_Driver/, CMSIS/
```

### Key source files (`motor_test1_20260227/Src/`)

| File | Role |
|------|------|
| `main.c` | System init, peripheral setup, superloop |
| `mc_tasks.c` | Medium-frequency control tasks (speed ramp, state checks) |
| `mc_tasks_foc.c` | High-frequency FOC algorithm (called from TIM1 interrupt) |
| `mc_interface.c` | Motor state machine (IDLE→START→RUN→FAULT) |
| `mc_api.c` | Public API: `MC_StartMotor1()`, `MC_StopMotor1()`, `MC_ProgramSpeedRampMotor1()` |
| `aspep.c` | ASPEP framing: CRC-4, DATA/PING/BEACON/ACK/NACK packets |
| `mcp.c` | Motor Control Protocol on top of ASPEP |
| `usart_aspep_driver.c` | USART2 + DMA driver for ASPEP |

### Key header files (`motor_test1_20260227/Inc/`)

| File | Role |
|------|------|
| `drive_parameters.h` | PWM frequency (25 kHz), FOC rate, observer gains, rev-up phases |
| `pmsm_motor_parameters.h` | Motor electrical params: pole pairs=2, Rs=0.1Ω, Ls=10µH (Motor Pilot measured 2026-03-12) |
| `mc_api.h` | Public motor control API declarations |
| `mc_type.h` | Fault codes, state enum, type definitions |

### Hardware peripherals (STM32G431CBU)

| Peripheral | Use |
|-----------|-----|
| TIM1 | 6-channel PWM, 3-phase motor drive |
| ADC1/ADC2 | Phase current + DC bus voltage sensing |
| USART2 | RPi5 link — 1843200 baud, DMA, ASPEP protocol |
| COMP1/2/4 | Analog comparators for overcurrent |
| OPAMP1/2/3 | Signal conditioning for current sense |
| CORDIC | Hardware sin/cos for FOC |
| DAC3 | Debug/monitoring output |

### ASPEP communication protocol

Frames: 4-byte header (CRC-4, polynomial x⁴+x+1) + variable payload.
Packet types: `DATA_PACKET (0x9)`, `PING (0x6)`, `BEACON (0x5)`, `ACK (0xA)`, `NACK (0xF)`.

### Motor state machine (mc_interface.c)

States: `IDLE(0)` → `ICLWAIT(12)` → `OFFSET_CALIB(17)` → `ALIGNMENT(2)` → `START(4)` → `RUN(6)` → `FAULT_NOW` / `FAULT_OVER`

## Project Goals (phased)

1. **Bare minimum** ✓ — closed-loop FOC + UART `[-1,1]` input + speed telemetry + timeout/fault stop
2. **Full ESC** ✓ — 7-state ESC state machine, extended 15-byte telemetry (speed + state + faults + u + vbus + iq + id + mcsdk_state), hardware verified
3. **Torque mode on-ground** ✓ — car moves forward and reverse on smooth hard floor (hardware verified 2026-03-12); sustained closed-loop RUN at 2250–2754 RPM with IMU-confirmed acceleration
4. **Multi-car robustness** (in progress, 2026-03-26) — UART-configurable params (0xCC frame), vehicle profile support in `esc_node_trigger_config.py`, transparent wrong-angle auto-retry; HOSIM firmware iteration 2 pending hardware validation
5. **Next** — HOSIM iter2 hardware test, surface variation testing, ROS2 nav stack integration

## Device Configuration

Peripheral pin mapping and clock config live in `motor_test1_20260227/motor_test1_20260227.ioc` (STM32CubeMX). Regenerating from CubeMX will overwrite HAL init files — keep custom code in designated `USER CODE` sections.

Linker script: `motor_test1_20260227/STM32CubeIDE/STM32G431CBUX_FLASH.ld`
