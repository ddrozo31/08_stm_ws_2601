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
| `Inc/esc_comm.h` | Public interface: `ESC_COMM_Init`, `GetCommand`, `HasNewCommand`, `SendTelemetry` |
| `Src/esc_comm.c` | RXNE interrupt-driven RX parser + direct-register blocking TX |

**Command frame (RPi5 → STM32, 5 bytes):** `[0xAA][cmd_lo][cmd_hi][reserved][XOR_chk]`
- `cmd` is `int16_t` scaled: −32768 = −1.0, +32767 = +1.0

**Telemetry frame (STM32 → RPi5, 15 bytes):** `[0xBB][spd_lo][spd_hi][esc_st][faults][u_lo][u_hi][v_lo][v_hi][iq_lo][iq_hi][id_lo][id_hi][mc_st][XOR_chk]`
- `speed` = `int16_t` RPM, `esc_st` = ESC_State_t byte, `faults` = lower byte of fault bitmask
- `u` = `int16_t` raw command (−32768..+32767), `v` = `uint16_t` DC bus voltage in Volts
- `iq_ma` = `int16_t` q-axis actual current in mA, `id_ma` = `int16_t` d-axis current in mA
- `mc_st` = `uint8_t` MCSDK internal state (IDLE=0, START=4, RUN=6, ANY_STOP=8, SWITCH_OVER=19, etc.)

**Key choices:**
- RX: RXNEIE enabled directly via `USART2->CR1`; `ESC_COMM_UART_RxISR()` called from `USART2_IRQHandler` USER CODE BEGIN 0; self-synchronising SOF search. HAL_UART_Receive_IT cannot be used — the project's custom LL-based `USART2_IRQHandler` never calls `HAL_UART_IRQHandler`.
- TX: direct register polling (TXE per byte, no TC wait); 10 bytes ≈ 43 µs at 1843200 baud.
- `ASPEP_start()` is commented out in `mc_tasks.c` — re-enable for Motor Pilot tuning.
- All ASPEP-sensitive USART flags (TC, ORE, FE, NE, IDLE) are cleared in USER CODE BEGIN 0 before the generated ASPEP handler runs, preventing `ASPEP_HWDataTransmittedIT` / `ASPEP_HWReset` from being called with uninitialised `aspepOverUartA`.

## Application Layer (mc_app_hooks.c)

`MC_APP_PostMediumFrequencyHook_M1()` runs every ~1 ms and implements the full ESC state machine:

**States:** `BOOT` → `WAIT_NEUTRAL` → `READY` ↔ `FORWARD` / `REVERSE` (via `BRAKE`) → `FAULT`

**Control mode: torque (current) mode**
Rev-up always uses MCSDK open-loop phases 1–5. Once MCSDK reaches RUN, the ESC
switches to `MC_ProgramTorqueRampMotor1_F(u * ESC_MAX_IQ_A, ESC_TORQUE_RAMP_MS)`.

**Key parameters (mc_app_hooks.c):**
- `ESC_REVUP_SPEED_RPM 2500.0f` — open-loop rev-up target speed; must equal `OBS_MINIMUM_SPEED_RPM`
- `ESC_REVUP_RAMP_MS 500U` — speed ramp during rev-up
- `ESC_MAX_IQ_A 12.0f` — |u|=1.0 maps to this Iq (12 A); motor rated 15 A safe
- `ESC_TORQUE_RAMP_MS 5U` — near-instant torque ramp on RUN entry (motor decelerates ~90 ms after SWITCH_OVER)
- `ESC_NEUTRAL_DEADBAND 0.02f` — |u| below this = neutral
- `ESC_TIMEOUT_MS 500U` — stop motor if no command for 500 ms
- `ESC_TELEMETRY_EVERY 100U` — send telemetry every 100 calls (~10 Hz)

**Sensorless operating limits (hardware-verified on smooth hard floor, 2026-03-12):**
- **Minimum reliable speed: ±2500 RPM** — BEMF floor; `OBS_MINIMUM_SPEED_RPM = 2500` matches this
- **Rev-up duration: ~7.1 s** — must hold throttle through the full open-loop rev-up for SWITCH_OVER to fire
- **SWITCH_OVER (MCSDK_19)** — 100 ms blend from open-loop to closed-loop; speed may drop ~400–600 RPM during blend
- **Observer handoff speed: 2500 RPM** — BEMF ≈ 0.625 V; sufficient with correct Ls=10 µH model
- **PHASE5 capped at 2800 RPM** — going higher causes speed-band violation (mech/elec ratio < 0.3125) → ANY_STOP
- **Full stall faults** — if axle is stopped by external load, observer loses lock; physics constraint of sensorless FOC

**Safety rules enforced:**
- `esc_cmd_value` initialised to `0x7FFF` (non-neutral) — WAIT_NEUTRAL blocks until host explicitly sends neutral
- Direction reversal (FORWARD↔REVERSE) always passes through BRAKE; motor must reach IDLE before restarting
- Timeout watchdog stops motor in FORWARD/REVERSE if no command for 500 ms
- Any MCSDK fault → ESC_FAULT; auto-ack on FAULT_OVER → back to WAIT_NEUTRAL

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
4. **Next** — further tuning, surface variation testing, integration with ROS2 nav stack

## Device Configuration

Peripheral pin mapping and clock config live in `motor_test1_20260227/motor_test1_20260227.ioc` (STM32CubeMX). Regenerating from CubeMX will overwrite HAL init files — keep custom code in designated `USER CODE` sections.

Linker script: `motor_test1_20260227/STM32CubeIDE/STM32G431CBUX_FLASH.ld`
