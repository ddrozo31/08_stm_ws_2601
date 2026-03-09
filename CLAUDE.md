# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32G431-based BLDC motor controller for an RC vehicle, targeting the B-G431B-ESC1 board. Uses ST Motor Control SDK v6.4.1 (FOC, sensorless). The board acts as a UART-controlled motor node receiving normalized commands `u ∈ [-1, 1]` from a Raspberry Pi 5 and streaming back telemetry (estimated speed, state, faults).

## Build

The project uses STM32CubeIDE (Eclipse CDT managed makefile build). There is no standalone CLI build script — the build is managed by the IDE.

To build from command line (requires arm-none-eabi-gcc in PATH):
```bash
make -C motor_test1_20260227/STM32CubeIDE/Debug/
```

Build artifacts output to `motor_test1_20260227/STM32CubeIDE/Debug/`:
- `motor_test1_20260227.elf` — flash with ST-Link via STM32CubeIDE or OpenOCD
- `motor_test1_20260227.hex` / `.bin` — alternate flash formats

**Key compiler flags:** `-Ofast -g3 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mcpu=cortex-m4`
**Defines:** `ARM_MATH_CM4`, `USE_HAL_DRIVER`, `STM32G431xx`

## ESC Communication Layer (esc_comm)

Two files own USART2 exclusively — ASPEP/MCP is disabled:

| File | Role |
|------|------|
| `Inc/esc_comm.h` | Public interface: `ESC_COMM_Init`, `GetCommand`, `HasNewCommand`, `SendTelemetry` |
| `Src/esc_comm.c` | RXNE interrupt-driven RX parser + direct-register blocking TX |

**Command frame (RPi5 → STM32, 5 bytes):** `[0xAA][cmd_lo][cmd_hi][reserved][XOR_chk]`
- `cmd` is `int16_t` scaled: −32768 = −1.0, +32767 = +1.0

**Telemetry frame (STM32 → RPi5, 10 bytes):** `[0xBB][spd_lo][spd_hi][state][faults][u_lo][u_hi][v_lo][v_hi][XOR_chk]`
- `speed` = `int16_t` RPM, `state` = ESC_State_t byte, `faults` = lower byte of fault bitmask
- `u` = `int16_t` raw command (−32768..+32767), `v` = `uint16_t` DC bus voltage in Volts

**Key choices:**
- RX: RXNEIE enabled directly via `USART2->CR1`; `ESC_COMM_UART_RxISR()` called from `USART2_IRQHandler` USER CODE BEGIN 0; self-synchronising SOF search. HAL_UART_Receive_IT cannot be used — the project's custom LL-based `USART2_IRQHandler` never calls `HAL_UART_IRQHandler`.
- TX: direct register polling (TXE per byte, no TC wait); 10 bytes ≈ 43 µs at 1843200 baud.
- `ASPEP_start()` is commented out in `mc_tasks.c` — re-enable for Motor Pilot tuning.
- All ASPEP-sensitive USART flags (TC, ORE, FE, NE, IDLE) are cleared in USER CODE BEGIN 0 before the generated ASPEP handler runs, preventing `ASPEP_HWDataTransmittedIT` / `ASPEP_HWReset` from being called with uninitialised `aspepOverUartA`.

## Application Layer (mc_app_hooks.c)

`MC_APP_PostMediumFrequencyHook_M1()` runs every ~1 ms and implements the full ESC state machine:

**States:** `BOOT` → `WAIT_NEUTRAL` → `READY` ↔ `FORWARD` / `REVERSE` (via `BRAKE`) → `FAULT`

**Key parameters:**
- `ESC_MAX_SPEED_RPM 15000` — |u|=1.0 maps to this RPM
- `ESC_RAMP_MS 100` — speed ramp duration per command
- `ESC_NEUTRAL_DEADBAND 0.02f` — |u| below this = neutral
- `ESC_TIMEOUT_MS 500` — stop motor if no command for 500 ms
- `ESC_TELEMETRY_EVERY 100` — send telemetry every 100 calls (~10 Hz)

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
| `drive_parameters.h` | PWM frequency (25 kHz), FOC rate, observer gains |
| `pmsm_motor_parameters.h` | Motor electrical params: pole pairs=2, Rs=0.1Ω, Ls=10µH |
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
2. **Full ESC** ✓ — 7-state ESC state machine, extended 10-byte telemetry (speed + state + faults + u + vbus), hardware verified
3. **Next** — TBD

## Device Configuration

Peripheral pin mapping and clock config live in `motor_test1_20260227/motor_test1_20260227.ioc` (STM32CubeMX). Regenerating from CubeMX will overwrite HAL init files — keep custom code in designated `USER CODE` sections.

Linker script: `motor_test1_20260227/STM32CubeIDE/STM32G431CBUX_FLASH.ld`
