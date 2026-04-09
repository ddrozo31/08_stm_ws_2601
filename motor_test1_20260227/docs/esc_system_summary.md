# ESC System — Full Characteristics, Features & Configuration

## 1. Hardware Platform

| Item | Value |
|------|-------|
| MCU | STM32G431CBU (Cortex-M4F, 170 MHz) |
| Board | B-G431B-ESC1 (ST reference ESC) |
| Motor type | BLDC/PMSM, sensorless |
| Motor params | Rs=0.1 Ω, Ls=10 µH, 2 pole pairs, Ψf=9.75e-4 Wb |
| PWM | 25 kHz center-aligned (TIM1), 3-phase 6-channel |
| Current sense | 3-shunt via OPAMP1/2/3, Rshunt=3 mΩ, gain=9.14 |
| UART link | USART2 → RPi5, 1843200 baud |
| Dead time | 800 ns total (400 ns per edge) |

## 2. Control Architecture

```
RPi5  ──UART──▶  esc_comm  ──▶  esc_app  ──▶  custom_foc  ──▶  TIM1 PWM
                 (parser)      (ESC FSM)     (FOC + EKF)      (motor)
```

**Two execution rates:**
- **HF (25 kHz):** ADC → Clarke → Park → PI(Iq,Id) → InvPark → SVM → PWM + EKF observer
- **MF (1 kHz):** State machine, speed PI, crossfade logic, OL ramp, telemetry

## 3. Motor State Machine (CFOC)

```
IDLE → ALIGNMENT → OPEN_LOOP → CROSSFADE → CLOSED_LOOP
                                                ↓
                                             FAULT
```

| State | What happens |
|-------|-------------|
| IDLE | PWM off, ADC offsets tracked via EMA (α=0.002) |
| ALIGNMENT | Apply d-axis current to lock rotor to known angle |
| OPEN_LOOP | Ramp speed 0→target with fixed Iq; forced electrical angle |
| CROSSFADE | Blend OL angle → EKF angle; BEMF threshold + dwell guard |
| CLOSED_LOOP | Speed PI sets Iq_ref; EKF provides angle + speed feedback |
| FAULT | PWM disabled; auto-ack attempts recovery |

## 4. ESC State Machine (esc_app)

```
BOOT → WAIT_NEUTRAL → READY ↔ FORWARD/REVERSE (via BRAKE) → FAULT
```

| Feature | Detail |
|---------|--------|
| Direction reversal | Always passes through BRAKE; motor must stop first |
| Timeout watchdog | 500 ms no-command → auto stop |
| Wrong-angle detect | Speed sign mismatch → stop + 500 ms backoff + auto-restart |
| Auto-restart | Observer loss (CFOC_IDLE while running) → back to READY |
| Neutral interlock | Must receive neutral command before first start |

## 5. UART Protocol

### Command frame (RPi5 → STM32, 5 bytes)

```
[0xAA] [cmd_lo] [cmd_hi] [0x00] [XOR of bytes 1..3]
```
- `cmd`: int16, −32768 = −1.0, +32767 = +1.0

### Config frame (RPi5 → STM32, 5 bytes)

```
[0xCC] [param_id] [val_lo] [val_hi] [XOR of bytes 1..3]
```
- Sent once at startup; firmware uses compile-time defaults if never received

### Telemetry frame (STM32 → RPi5, 15 bytes)

```
[0xBB][spd_lo][spd_hi][esc_st][faults][u_lo][u_hi][v_lo][v_hi]
      [iq_lo][iq_hi][id_lo][id_hi][cfoc_st][XOR of bytes 1..13]
```

| Field | Type | Description |
|-------|------|-------------|
| spd | int16 | Motor speed [RPM] |
| esc_st | uint8 | ESC state (0=BOOT, 1=WAIT_NEUTRAL, 2=READY, 3=FORWARD, 4=BRAKE, 5=REVERSE, 6=FAULT) |
| faults | uint8 | Fault bitmask (0x02=OVER_VOLT, 0x04=UNDER_VOLT, 0x08=OVER_TEMP, 0x10=START_UP, 0x20=SPEED_FDBK, 0x40=OVER_CURR, 0x80=SW_ERROR) |
| u | int16 | Raw command echo (−32768..+32767) |
| v | uint16 | DC bus voltage [tenths of V] (e.g. 126 = 12.6 V) |
| iq | int16 | q-axis current [mA] |
| id | int16 | d-axis current [mA] |
| cfoc_st | uint8 | CFOC state (0=IDLE, 1=ALIGNMENT, 2=OPEN_LOOP, 3=CROSSFADE, 4=CLOSED_LOOP, 5=FAULT) |

- Telemetry rate: 10 Hz

## 6. Runtime-Configurable Parameters (0xCC frames)

All parameters are sent by the ROS2 node at startup. If never received, firmware uses compile-time defaults.

| Param ID | Name | Encoding | Range | FW Default | Node Default | Effect |
|----------|------|----------|-------|------------|--------------|--------|
| 0x01 | max_iq_a | int16 × 0.1 A | 10–150 | 12.0 A | — | Max torque current |
| 0x02 | revup_rpm | int16 RPM | 1600–5000 | 1600 RPM | — | Rev-up target speed |
| 0x03 | boost_iq_a | int16 × 0.1 A | 10–150 | 12.0 A | — | RUN-entry boost |
| **0x04** | **max_speed_rpm** | int16 RPM | 1000–10000 | 5000 RPM | 3000 | u=1.0 maps to this speed |
| **0x05** | **iq_limit_a** | int16 × 0.1 A | 10–200 | 10.0 A | 12.0 | Speed PI Iq clamp |
| **0x06** | **ol_iq_a** | int16 × 0.1 A | 20–150 | 5.0 A | 8.0 | Open-loop Iq target |
| **0x07** | **ol_ramp_ms** | int16 ms | 1000–8000 | 3000 ms | 4000 | OL speed ramp duration |
| **0x08** | **align_ms** | int16 ms | 100–2000 | 300 ms | 500 | Alignment duration |
| **0x09** | **align_id_a** | int16 × 0.1 A | 10–150 | 3.0 A | 5.0 | Alignment d-axis current |
| **0x0A** | **xf_duration_ms** | int16 ms | 100–1000 | 500 ms | 500 | Crossfade blend duration |
| **0x0B** | **xf_dwell_ms** | int16 ms | 50–500 | 200 ms | 200 | BEMF dwell before blend |
| **0x0C** | **ol_target_rpm** | int16 RPM | 1000–2000 | 1400 RPM | 1400 | OL target / handoff speed |
| **0x0D** | **spd_kp** | int16 × 0.001 | 5–50 | 0.01 A/RPM | 0.01 | Speed PI Kp |
| **0x0E** | **spd_ki** | int16 × 0.0001 | 5–50 | 0.001 A/RPM | 0.001 | Speed PI Ki (discretized) |
| **0x0F** | **spd_lpf_alpha** | int16 × 0.0001 | 2–20 | 0.0004 | 0.0004 | Speed PI LPF filter |

> **Bold** = actively sent by `esc_node_custom_foc.py` at startup. Params 0x01–0x03 are legacy (torque-mode era) and not currently sent by the node.

## 7. Compile-Time-Only Parameters (require firmware rebuild)

| Parameter | Value | Location |
|-----------|-------|----------|
| PWM frequency | 25 kHz | `custom_foc.h` |
| Current PI Kp | 0.063 | `custom_foc.h` |
| Current PI Ki | 0.025 (discretized) | `custom_foc.h` |
| Vmax/Vbus ratio | 2/π (six-step OVM) | `custom_foc.h` |
| EKF angle LPF α | 0.002 (τ=20 ms) | `custom_foc.h` |
| EKF Q_I | 1.33e-4 A² | `custom_foc.h` |
| EKF Q_E | 1.33e-3 V² | `custom_foc.h` |
| EKF R_I | 3.33e-3 A² | `custom_foc.h` |
| BEMF² crossfade threshold | 0.05 V² | `custom_foc.h` |
| Dead time compensation | 800 ns | `custom_foc.h` |
| Neutral deadband | 0.02 | `esc_app.c` |
| Command timeout | 500 ms | `esc_app.c` |
| Restart delay | 500 ms | `esc_app.c` |
| Wrong-angle threshold | 50 RPM | `esc_app.c` |
| Debug log buffer | 1000 entries (19 KB) | `custom_foc.h` |

## 8. ROS2 Node Features (esc_node_custom_foc.py)

| Feature | Detail |
|---------|--------|
| Control input | `cmd_vel_stamped` (geometry_msgs/TwistStamped, linear.x) |
| Two-phase control | Startup: fixed `u_startup=0.30`; CL: proportional `u = vx / max_linear_mps` |
| Slew rate limiter | 0.05 u/tick at 20 Hz → smooth OL→CL handoff |
| Timeout | 0.5 s no cmd_vel → forced neutral |
| Config delivery | All 0xCC frames sent once at port open, before any motor command |

### Published Topics

| Topic | Type | Content |
|-------|------|---------|
| `esc/speed_rpm` | Int16 | Motor speed [RPM] |
| `esc/state` | String | ESC state name |
| `esc/faults` | String | Active faults or "none" |
| `esc/command` | Float32 | Current u value (−1.0..+1.0) |
| `esc/vbus_v` | Float32 | DC bus voltage [V] |
| `esc/iq_ma` | Int16 | q-axis current [mA] |
| `esc/id_ma` | Int16 | d-axis current [mA] |
| `esc/cfoc_state` | String | CFOC state name |

### Subscribed Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `cmd_vel_stamped` | TwistStamped | Velocity command (linear.x) |

### Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| port | /dev/serial/by-id/usb-STMicro... | Serial port path |
| baudrate | 1843200 | UART baud rate |
| cmd_hz | 20 | Command send rate [Hz] |
| deadband | 0.05 | Neutral deadband on linear.x |
| u_max | 1.0 | Max command magnitude |
| u_startup | 0.30 | Fixed command during startup phases |
| slew_rate | 0.05 | Max |du| per sender cycle |
| max_linear_mps | 1.0 | linear.x that maps to u=1.0 |
| cmd_vel_timeout | 0.5 | Seconds before forced neutral |

## 9. Safety Features

- **Neutral interlock** — WAIT_NEUTRAL blocks until host explicitly sends neutral
- **Direction reversal through BRAKE** — motor must reach IDLE before restarting opposite direction
- **Command timeout** — 500 ms watchdog stops motor if no command received
- **Wrong-angle auto-retry** — detects STO 180° lock, stops + 500 ms backoff + auto-restarts
- **Observer loss recovery** — auto-restart if CFOC goes IDLE unexpectedly during run
- **FAULT auto-ack** — attempts recovery on FAULT_OVER transition
- **Overcurrent protection** — COMP1/2/4 hardware comparators → TIM1 BRK → CFOC_FaultStop
- **Slew rate limiter** — node-side, prevents sudden command jumps during OL→CL transition

## 10. Current Best-Tuned Config (2026-04-09 Baseline)

```bash
ros2 run zulu_esc esc_node_custom_foc --ros-args \
  -p xf_duration_ms:=200 -p xf_dwell_ms:=100 -p ol_target_rpm:=1400 \
  -p spd_kp:=0.02 -p spd_ki:=0.002 -p spd_lpf_alpha:=0.001
```

### Validation Results

| Metric | 04-08 Baseline (xf=500) | 04-09 Best (xf=200) |
|--------|-------------------------|----------------------|
| Clean transition rate | 10% (1/10) | **100% (3/3)** |
| Avg speed drop at CL entry | 547 RPM | **14 RPM** (−97%) |
| Iq std post-transition (2 s) | 3.13 A | **0.46 A** (−85%) |
| Faults | 0 | 0 |
| Auto-restarts | 13 | 2 |

### Key Changes That Achieved This

1. **Bumpless transfer** — PI integral seeded with last OL Iq (was zeroed → torque dip)
2. **Shorter crossfade (200 ms)** — less time with mixed OL/EKF angle = less bad blend risk
3. **Shorter dwell (100 ms)** — EKF converged by OL ramp end; less unnecessary waiting
4. **Faster speed PI** — Kp×2 + Ki×2 + LPF τ 100→40 ms = tighter tracking during transition

### Reference Commits

- Firmware: `d4cfbd0` on `custom_foc` branch (08_stm_ws_2601)
- Node: `26abad7` on `nav_rc_car` branch (mrad_ws_2601_zulu)

### Reference Bags (08_stm_ws_2601/bag_file/)

| Bag | Config | Result |
|-----|--------|--------|
| `rosbag2_2026_04_09-11_15_58` | Bench, default params | Baseline bench |
| `rosbag2_2026_04_09-11_23_08` | Ground, xf=300, dwell=150 | 0% clean |
| `rosbag2_2026_04_09-11_29_19` | Ground, xf=300, dwell=150 | 50% clean |
| `rosbag2_2026_04_09-11_42_34` | Ground, xf=200, dwell=100 | **100% clean** ← baseline |

## 11. Known Limitations

- **Observer-limited transitions** — discrete OL→CL crossfade with fixed EKF noise is the architectural ceiling for transition quality
- **Minimum crossfade speed ~1400 RPM** — below this, Vdt/BEMF ratio kills observer accuracy (1200 RPM = physics floor, Vdt/BEMF=98%)
- **Full stall = fault** — sensorless FOC cannot operate at zero speed (physics constraint)
- **Step 8 (deferred)** — adaptive-R EKF rebuild is the fundamental fix for observer-side limitations; deferred until after ROS2 nav stack (Step 7)

## 12. Source File Map

### Firmware (motor_test1_20260227/)

| File | Role |
|------|------|
| `Src/custom_foc.c` | Core FOC: HF+MF tasks, EKF, PI controllers, state machine, crossfade |
| `Inc/custom_foc.h` | All FOC #defines, PI struct, public API |
| `Src/esc_app.c` | ESC state machine: BOOT→READY→FWD/REV→BRAKE→FAULT |
| `Inc/esc_app.h` | ESC state enum, public API |
| `Src/esc_comm.c` | UART RX parser (0xAA+0xCC), TX telemetry, config storage |
| `Inc/esc_comm.h` | Protocol defines, param IDs, getter declarations |
| `Src/esc_ekf_observer.c` | 4-state EKF: [Iα, Iβ, eα, eβ], BEMF extraction |
| `Inc/esc_ekf_observer.h` | EKF handle struct and API |
| `Src/mc_app_hooks.c` | SysTick hook: calls CFOC_MediumFrequencyTask + ESC_APP_Tick |
| `Src/mc_tasks.c` | ASPEP disabled under BUILD_ESC; calls CFOC init |
| `Src/main.c` | System init, peripheral setup, superloop |

### ROS2 Node (mrad_ws_2601_zulu/src/zulu_esc/zulu_esc/)

| File | Role |
|------|------|
| `esc_node_custom_foc.py` | Full ESC node: cmd_vel → UART commands, telemetry → ROS topics, 0xCC config |
