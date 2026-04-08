# Custom FOC Implementation Plan

**Branch:** `custom_foc` (forked from `MCSDK_EKF`)
**Target:** B-G431B-ESC1 (STM32G431CBU), AMORIL #1 RC car
**Date:** 2026-04-03

---

## 1. Why: MCSDK Is the Problem

After 6 iterations on branch `MCSDK_EKF`, we proved that the MCSDK state machine
cannot be made to work with an external EKF observer:

| Attempt | Approach | Result |
|---------|----------|--------|
| Step 5 | EKF feeds `hElAngle` only | STO PLL corrupted by foreign angle |
| Step 6a | BEMF threshold gate | Same — STO still reads hElAngle |
| Step 6b | State-restricted override (RUN only) | Never reaches RUN — circular dependency |
| Step 6c | Full STO takeover (angle+speed+convergence) | Still no SWITCH_OVER — unknown internal check |

**Root cause:** The MCSDK has an opaque, deeply entangled state machine with undocumented
internal checks beyond `STO_PLL_IsObserverConverged`. We cannot override all of them
without modifying the library source, and at that point we might as well write our own.

**The EKF itself works.** It converges, it tracks angle, it estimates speed. The problem
is exclusively the MCSDK wrapper refusing to transition states.

---

## 2. Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    TIM1 UPDATE ISR (25 kHz)              │
│                                                          │
│  ┌──────────┐   ┌──────┐   ┌────┐   ┌─────────┐        │
│  │ Read ADC │──▶│Clarke│──▶│Park│──▶│ PI (Iq) │─┐      │
│  │ Ia, Ib   │   │→Iα,Iβ│   │→Id,Iq│ │ PI (Id) │ │      │
│  └──────────┘   └──────┘   └────┘   └─────────┘ │      │
│        │                      ▲                   ▼      │
│        │                      │            ┌───────────┐ │
│        │               ┌──────┘            │Circle Lim │ │
│        │               │ θ_e               │  Vd, Vq   │ │
│        │          ┌─────────┐              └─────┬─────┘ │
│        │          │   EKF   │                    │       │
│        │          │Observer │              ┌─────▼─────┐ │
│        ├─────────▶│ x=[iα,  │              │ Rev Park  │ │
│        │  Iα,Iβ   │  iβ,eα, │              │  → Vα,Vβ  │ │
│        │          │  eβ]    │              └─────┬─────┘ │
│        │          └─────────┘                    │       │
│        │               │                   ┌─────▼─────┐ │
│        │          ω_e, θ_e                 │   SVM     │ │
│        │                                   │ → TIM1    │ │
│        │                                   │  CCR1/2/3 │ │
│        │                                   └───────────┘ │
│  ┌─────▼─────┐                                           │
│  │  Vα, Vβ   │◀─── stored from previous cycle            │
│  │ (for EKF) │                                           │
│  └───────────┘                                           │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│              1 kHz Medium-Frequency Task                 │
│                                                          │
│  ┌───────────┐  ┌────────────┐  ┌──────────────────┐    │
│  │ ESC State │  │ Speed PI   │  │ Open-loop ramp   │    │
│  │ Machine   │  │ (in RUN)   │  │ (in STARTUP)     │    │
│  └───────────┘  └────────────┘  └──────────────────┘    │
│                                                          │
│  ┌───────────┐  ┌────────────┐                           │
│  │ UART Comm │  │ Telemetry  │                           │
│  │ (esc_comm)│  │ (10 Hz)    │                           │
│  └───────────┘  └────────────┘                           │
└─────────────────────────────────────────────────────────┘
```

---

## 3. What We Keep vs. What We Replace

### KEEP (from MCSDK / existing code)

| Component | Source | Why |
|-----------|--------|-----|
| CubeMX peripheral init | `main.c` (MX_TIM1_Init, MX_ADC1/2_Init, etc.) | Correct pin/clock/DMA config |
| HAL drivers | `STM32G4xx_HAL_Driver/` | Stable, tested |
| CORDIC peripheral | `MX_CORDIC_Init` | Hardware sin/cos in ~6 cycles |
| ESC comm layer | `esc_comm.c/h` | UART protocol with RPi5 |
| EKF observer | `esc_ekf_observer.c/h` | Proven algorithm, 8 unit tests |
| Motor parameters | `pmsm_motor_parameters.h` | Rs=0.1Ω, Ls=10µH, p=2 |
| Rosbag analyzer | `tests/analyze_rosbag.py` | Reusable analysis tool |

### REPLACE (write from scratch)

| Component | MCSDK Source | Our New File |
|-----------|-------------|--------------|
| FOC ISR + math | `mc_tasks_foc.c` (850 lines) | `custom_foc.c` (~200 lines) |
| State machine | `mc_interface.c` + `mc_tasks.c` | `custom_foc.c` (states in enum) |
| SVM + PWM write | `pwm_curr_fdbk_ovm.c` (510 lines) | `custom_svm.c` (~120 lines) |
| ADC current read | `r3_2_g4xx_pwm_curr_fdbk.c` | `custom_svm.c` (~40 lines) |
| PI controllers | `pid_regulator.c` (730 lines) | `custom_foc.c` (~30 lines) |
| Clarke/Park math | `mc_math.c` | `custom_foc.c` (inline, ~20 lines) |
| App hooks | `mc_app_hooks.c` (complex) | `custom_foc.c` (integrated) |

### REMOVE (no longer needed)

- All MCSDK library files under `MCSDK_v6.4.1-Full/MotorControl/`
- STO/PLL observer (`sto_pll_speed_pos_fdbk.c`)
- ASPEP/MCP protocol (`aspep.c`, `mcp.c`, `usart_aspep_driver.c`)
- Motor Pilot registers (`hf_registers.c`, `mcp_config.c`)
- Speed-torque controller (`speed_torq_ctrl.c`)
- Rev-up controller (`revup_ctrl_sixstep.c`)

> **Note:** We won't delete these files in Step 1. We'll exclude them from the build
> (remove from STM32CubeIDE project sources) and add our new files instead.

---

## 4. Motor State Machine

Simple, transparent, fully under our control:

```
                    ┌─────────┐
           ┌───────│  FAULT  │◀── overcurrent / timeout / EKF diverge
           │       └────┬────┘
           │            │ fault cleared
           │            ▼
           │       ┌─────────┐
           │       │  IDLE   │◀── power-on / stop command / startup fail
           │       └────┬────┘
           │            │ command received, |u| > deadband
           │            ▼
           │     ┌──────────────┐
           │     │  ALIGNMENT   │  d-axis pulse (optional, 100ms)
           │     └──────┬───────┘
           │            │
           │            ▼
           │     ┌──────────────┐
           │     │   OPEN_LOOP  │  ramp angle + current injection
           │     │   (startup)  │  EKF runs in parallel, tracking BEMF
           │     └──────┬───────┘
           │            │ EKF BEMF² > threshold for 200ms
           │            ▼
           │     ┌──────────────┐
           │     │  CROSSFADE   │  blend: θ = α·θ_OL + (1−α)·θ_EKF
           │     │   (50ms)     │  α ramps 1→0
           │     └──────┬───────┘
           │            │ α = 0 (fully EKF)
           │            ▼
           │     ┌──────────────┐
           └─────│  CLOSED_LOOP │  EKF angle drives FOC
                 │    (RUN)     │  Speed PI + torque command
                 └──────────────┘
```

**Key difference from MCSDK:** The crossfade is a simple linear blend, not a complex
state machine with convergence checks, reliability counters, and speed FIFOs.

---

## 5. Implementation Steps (Detailed Roadmap)

### Step 1: Bare ISR — Read Currents, Output Zero Voltage ✅ DONE (2026-04-03)

**Goal:** TIM1 fires at 25 kHz, reads ADC injected channels, writes zero PWM.
Motor doesn't move. Validates ISR timing and ADC readings.

**Files created/modified:**
- `motor_test1_20260227/Src/custom_foc.c` — ISR, state machine shell, self-calibration
- `motor_test1_20260227/Inc/custom_foc.h` — public API + hardware constants
- `motor_test1_20260227/Src/stm32g4xx_mc_it.c` — ISR routing (ADC1_2, TIM1_UP, TIM1_BRK)
- `motor_test1_20260227/Src/stm32_mc_common_it.c` — SysTick → MF task, USART2, HardFault
- `motor_test1_20260227/Src/main.c` — `CFOC_Init()` replaces `MX_MotorControl_Init()`
- `motor_test1_20260227/STM32CubeIDE/.project` — removed all MCSDK sources, added custom_foc.c

**Hardware validation results (B-G431B-ESC1, AMORIL car):**

| Metric | Expected | Measured | Status |
|--------|----------|----------|--------|
| ISR rate | 25 kHz | ~25 kHz (451k counts in ~18s) | ✅ |
| adc_offset_a | ~2048 | 2532 | ✅ |
| adc_offset_b | ~2048 | 1453 (OPAMP2 bias) | ✅ |
| isr_Ialpha | ~0 A | -0.029 A | ✅ |
| isr_Ibeta | ~0 A | 0.59 A (Phase B offset noise) | ⚠️ acceptable |
| cfoc_state | CFOC_IDLE | CFOC_IDLE | ✅ |

**Key lessons learned (for future reference):**
1. **HAL ADC state machine is incompatible with LL.** After `HAL_ADCEx_Calibration_Start`,
   `HAL_ADCEx_InjectedStart` silently fails. Must use LL for ADC enable/JSQR/JADSTART.
2. **ADC trigger source `TIM1_CH4` doesn't work** on this hardware config.
   Use `TIM1_TRGO` instead (TRGO = OC4REF, configured in MX_TIM1_Init).
3. **ADC is left-aligned** (configured by CubeMX). `LL_ADC_INJ_ReadConversionData12`
   returns 16-bit left-aligned value — must `>> 4` for true 12-bit.
4. **Self-calibration in ISR** avoids blocking polling loops. First 64 ISR calls
   accumulate ADC samples, then compute offsets. No startup hang risk.
5. **TIM2 slave-trigger mechanism** from MCSDK's `startTimers` doesn't work without
   full R3_2_Init. Use `LL_TIM_EnableCounter(TIM1)` directly — safe at 50% duty.

**Actual new code:** ~200 lines (custom_foc.c) + ~60 lines (ISR routing)

---

### Step 2: Open-Loop Startup — Ramp Angle + Current Injection ✅ DONE (2026-04-04)

**Goal:** Motor spins in open-loop at commanded speed. No observer feedback yet.

**What to implement:**
1. **1 kHz medium-frequency task** (SysTick or TIM6):
   - Open-loop angle integrator: `θ_OL += ω_cmd × Ts` (wraps at 2π)
   - Speed ramp: `ω_cmd` ramps from 0 to target over 3000ms
   - Current reference: `Iq_ref` ramps 0 → 5A over first 500ms
2. **25 kHz HF task additions:**
   - Park transform using `θ_OL`
   - PI controller for Iq (torque): `Vq = Kp·(Iq_ref - Iq) + Ki·∫`
   - PI controller for Id (flux): `Vd = Kp·(0 - Id) + Ki·∫` (Id = 0 for SPMSM)
   - Circle limitation: if `√(Vq²+Vd²) > Vmax`, scale both
   - Inverse Park: `Vα, Vβ = RevPark(Vq, Vd, θ_OL)`
3. **SVM (Space Vector Modulation):**
   - Standard 7-segment center-aligned SVM
   - Sector detection from Vα, Vβ
   - Duty cycle calculation → TIM1 CCR1/2/3
   - No overmodulation needed initially (can add later)

**PI gains (starting point from MCSDK):**
- Iq: Kp = Ls × ωc = 10µH × 2π×1000 = 0.063, Ki = Rs × ωc = 0.1 × 2π×1000 = 628
- Discretized at 25 kHz: Ki_discrete = Ki × Ts = 628/25000 = 0.025
- These will need tuning on hardware

**Hardware test:** Motor should spin up to 1600 RPM in open-loop. 
Record rosbag → `python3 tests/analyze_rosbag.py`

**Estimated new code:** ~200 lines (PI + SVM + open-loop ramp)

**Hardware validation results (B-G431B-ESC1, AMORIL car, 2026-04-04):**

| Metric | Expected | Measured | Status |
|--------|----------|----------|--------|
| Alignment Id | 3.0 A | 3.1 A | ✅ |
| Alignment Vd | ~0.3 V (R×I) | 0.34 V | ✅ |
| Open-loop Iq | ramp to 5 A | ~1.9 A at t=500ms | ✅ (ramp in progress) |
| PI saturation | no | Vq < 0.4 V | ✅ |
| Motor spin | continuous | yes, wheel turns | ✅ |
| Grinding noise | — | mild, expected (OL angle mismatch) | ⚠️ will fix in Step 3 |

**Key bugs found during debug:**
1. **Phase V on ADC2, not ADC1** — was reading Phase W instead of Phase V (zero current seen despite extreme PWM duty)
2. **PI antiwindup** needed conditional integration (simple clamp insufficient)
3. **Button bounce** on EXTI caused double start/stop

**Debug tools created:** RAM log buffer (500 × 13B) + `tests/parse_cfoc_log.py` + GDB `dump binary memory`

---

### Step 3: EKF + Crossfade to Closed-Loop ✅ DONE (2026-04-04)

**Goal:** Motor transitions from open-loop to EKF-driven closed-loop. **This is the critical step.**

**What was implemented:**
1. **EKF at 25 kHz (HF task)** — Euler discretization requires Ts < 2×Ls/Rs = 200µs;
   at 1 kHz (Ts=1ms), a1 = 1−Ts×Rs/Ls = −9999 → divergence. Moved to HF task (Ts=40µs, a1=0.6).
   Q noise scaled ÷25 for higher rate. R unchanged.
2. **Dead-time compensation** — 800ns dead time creates Vdt≈0.24V/phase error (~73% of BEMF
   at 1600 RPM). Soft-sign compensation in αβ frame: `I/(|I|+0.5)` avoids zero-crossing step.
3. **Speed crossfade (NOT angle crossfade)** — EKF angle has ~50° residual offset; blending
   angles caused field loss → 29A overcurrent spike. Instead blend angular velocity:
   `ω_blend = (1−α)×ω_OL + α×ω_EKF_filtered`, then integrate for commutation angle.
4. **EKF speed LPF** — Raw EKF speed oscillates ±2000 RPM at electrical frequency.
   First-order LPF τ=20ms (α=0.002) smooths before angle integration.
5. **Crossfade trigger** — `ol_ramp_ms >= OL_RAMP_MS` + 200ms dwell counter.
   BEMF² threshold alone was unreliable at low speed.

**Hardware validation results (B-G431B-ESC1, AMORIL car, 2026-04-04):**

| Metric | Expected | Measured | Status |
|--------|----------|----------|--------|
| EKF convergence | during OL ramp | yes, speed tracks | ✅ |
| Crossfade trigger | at ramp end + 200ms | ~3200ms | ✅ |
| Crossfade smoothness | no jerks | smooth acceleration | ✅ |
| Closed-loop sustained | indefinite | yes, runs until button stop | ✅ |
| Overcurrent faults | none | none | ✅ |
| Motor sound | smooth | smooth, consistent acceleration | ✅ |

**Key bugs found during debug (12+ HW iterations, cfoc5–cfoc16):**
1. **EKF Euler instability at 1 kHz** — a1=−9999 → immediate divergence → crash at Kalman gain
2. **~100° EKF angle offset** — dead time not compensated in EKF voltage feed
3. **DT compensation wrong sign** — subtraction made offset worse (100°→170°), flipped to addition
4. **DT compensation too large** — TIM1 DTG uses DEAD_TIME_COUNTS/2, actual per-edge=400ns not 800ns
5. **Premature crossfade** — soft-sign DT at low current let EKF converge to false angle at ~0 RPM
6. **Angle crossfade overcurrent** — ~50° angle error + blend → field loss → 29A positive feedback
7. **Jerky closed-loop** — raw EKF speed noise ±2000 RPM at electrical frequency

**Known limitation:** Motor accelerates to ~10000 RPM (no speed controller, fixed Iq=5A). Expected — speed control is Step 4.

**Actual new code:** ~300 lines added/modified in custom_foc.c + custom_foc.h

---

### Step 4: Speed PI for RUN Mode ✅ DONE (2026-04-04)

**Goal:** Closed-loop speed control. EKF provides speed feedback, PI sets Iq reference.

**What was implemented:**
1. **Speed PI** (runs at 1 kHz in MF task, CLOSED_LOOP state only):
   - `Iq_ref = PI(speed_cmd - speed_filtered)`
   - Speed feedback from LPF'd EKF speed (τ=20ms), NOT raw EKF speed
   - Motoring-only output clamp: forward → Iq∈[0, 10A], reverse → Iq∈[-10A, 0]
   - PI integrator seeded to 0 at CL entry (bumpless transfer)
   - `ekf_rpm` now signed (× direction) for correct PI error
2. **Log expanded** to 19 bytes per entry (+Iq_ref_x100 field)

**Speed PI gains (hardware-tuned):**
- Kp = 0.01 A/RPM — 100 RPM error → 1 A correction
- Ki = 0.001 A/(RPM·s) — discrete, already multiplied by MF_Ts

**Hardware validation results (B-G431B-ESC1, AMORIL car, 2026-04-04):**

| Metric | Expected | Measured | Status |
|--------|----------|----------|--------|
| Speed target | 1600 RPM | 1585 RPM (last-quarter mean) | ✅ |
| Speed stability | low std | std=520 RPM | ✅ acceptable |
| Steady-state Iq_ref | regulated | ~7.9 A | ✅ |
| Duration | indefinite | ran full 8s log, no faults | ✅ |
| OL→CL transition | smooth | brief overshoot to ~3600 RPM | ⚠️ acceptable |

**Key bugs found during tuning (7 HW iterations, cfoc20–cfoc26):**
1. **PI integrator seed too high** (5A) → overshoot to 5000+ RPM with no load
2. **Raw EKF speed for PI** → bang-bang oscillation (±2000 RPM noise)
3. **OL Iq too low** (2A) → lost field orientation, grinding noise
4. **Crossfade Iq taper** → EKF lost signal at zero current
5. **Regen braking** (-10A) → angle tracking fails at high speed
6. **Slow LPF + mild braking** (-2A, τ=40ms) → motor stalled from over-braking

**Known limitation:** Initial overshoot to ~3600 RPM before settling (no-load,
wheels off ground). With real load, friction absorbs excess kinetic energy.

**Actual new code:** ~50 lines added/modified

---

### Step 5: ESC State Machine + UART Integration — DONE (2026-04-04)

**Goal:** Full ESC firmware — drop-in replacement for the MCSDK version.

**What was implemented:**

1. **New CFOC APIs** (`custom_foc.h/c`):
   - `CFOC_SetSpeed(rpm)` — speed PI target (speed mode)
   - `CFOC_SetTorque(iq_ref)` — direct torque mode (not used in final)
   - `CFOC_GetIqd(*iq, *id)` — d-q current readback for telemetry
   - `CFOC_AckFault()` — CFOC_FAULT → CFOC_IDLE transition
   - `CFOC_IsRunning()` — true if ALIGNMENT..CLOSED_LOOP
   - `CFOC_FaultStop()` — like Stop() but → CFOC_FAULT

2. **ESC state machine** (`esc_app.c/h` — NEW files):
   - States: BOOT → WAIT_NEUTRAL → READY → FORWARD/REVERSE → BRAKE → FAULT
   - Speed mode: `CFOC_SetSpeed(u × 5000 RPM)` once CFOC reaches CLOSED_LOOP
   - CFOC handles startup internally (ALIGNMENT → OPEN_LOOP → CROSSFADE)
   - Wrong-angle detection + auto-restart with 500ms backoff
   - 500ms command timeout → stop
   - 10 Hz telemetry (15-byte frame: speed, states, faults, iq, id, vbus)

3. **Wiring:**
   - `main.c`: `ESC_APP_Init()` after `CFOC_Init()` under `BUILD_ESC`
   - `SysTick`: `ESC_APP_Tick()` after `CFOC_MediumFrequencyTask()` under `BUILD_ESC`
   - `BRK handler`: `CFOC_FaultStop()` instead of `CFOC_Stop()`
   - Button: `#ifndef BUILD_ESC` guard (Debug-only)

4. **ROS2 node** (`esc_node_custom_foc.py` in `zulu_esc` package):
   - Joystick control: Hold RB + left stick Y
   - Slew rate limiter (0.05/tick at 20 Hz)
   - Publishes 8 topics: speed_rpm, state, cfoc_state, command, iq_ma, id_ma, vbus_v, faults

**Hardware validation (3 iterations):**

| Test | Mode | Result |
|------|------|--------|
| 01 (`esc_test.py`) | Torque | 3 bugs found: boost overshoot, stale telemetry, raw EKF noise |
| 02 (`esc_test.py`) | Torque (fixed) | Smooth up to u=0.3, chunky/aggressive above — torque mode inherent |
| 03 (ROS2 rosbag) | **Speed** | Smooth across full range, proportional: u=0.4→2000, u=0.7→3500, u=1.0→5000 RPM |

**Key design decision — torque → speed mode:**
Pure torque mode on a low-impedance motor (Rs=0.1Ω, Ls=10µH) is inherently aggressive
above ~1600 RPM — excess Iq directly accelerates the rotor with no regulation. Speed mode
with the existing PI controller (Step 4) naturally limits current to what's needed.

**Rosbag validation (42.2s, 6484 messages, 423 telemetry frames):**
- Forward: u=0.4→2000 RPM, u=0.7→3500 RPM, u=1.0→5000 RPM
- Reverse: symmetric, negative RPM
- Zero faults, clean state transitions (IDLE→ALIGNMENT→OPEN_LOOP→CROSSFADE→CLOSED_LOOP)
- Telemetry: speed, iq, id all cleared correctly on stop (no stale values)

**Actual new code:** ~250 lines (esc_app.c) + ~100 lines (CFOC API additions) + ~250 lines (ROS2 node)

---

### Step 5b: Ground Tuning & Runtime Config — DONE (2026-04-04)

**Goal:** Reliable on-ground operation with runtime-tunable startup parameters.

**Problems found during on-ground testing (5 rosbags, ~260s total):**

1. **Speed saturation under load** — Speed PI saturated at 10A Iq clamp, limiting
   ground speed to ~2500 RPM regardless of command above u=0.4.
   Fix: runtime-configurable `iq_limit` (raised to 12A) and `max_speed_rpm` (lowered to 3000).

2. **Joystick noise during startup → grinding** — Proportional stick during
   ALIGNMENT→OPEN_LOOP→CROSSFADE caused varying speed target, disrupting field tracking.
   Fix: two-phase control in ROS2 node — fixed `u_startup` during startup, proportional
   only after CLOSED_LOOP (same pattern as old `esc_node_trigger.py`).

3. **Open-loop grinding on ground** — Default startup (3A align, 5A OL, 3s ramp) too
   weak for ground friction. Motor field slips against load during OL ramp.
   Fix: runtime-configurable startup params. Ground-validated conservative profile:
   align 700ms @ 6A, OL ramp 5000ms @ 10A. Consistent startup, no grinding.

4. **Vbus hardcoded at 12V** — Implemented real ADC reading via ADC1 regular channel 1.
   Status: IN REVIEW — reading needs validation on 3S LiPo (expected 9.0-12.6V).

**Runtime config params (0xCC UART frames, sent by RPi5 at node startup):**

| Param ID | Name | Encoding | Default (ground) | Range |
|----------|------|----------|-------------------|-------|
| 0x04 | max_speed_rpm | int16 RPM | 3000 | 1000-10000 |
| 0x05 | iq_limit | int16 × 0.1A | 12.0A | 1.0-20.0A |
| 0x06 | ol_iq | int16 × 0.1A | 8.0A | 2.0-15.0A |
| 0x07 | ol_ramp_ms | int16 ms | 4000 | 1000-8000 |
| 0x08 | align_ms | int16 ms | 500 | 100-2000 |
| 0x09 | align_id | int16 × 0.1A | 5.0A | 1.0-15.0A |

**Tested profiles:**

| Profile | align | OL Iq | OL ramp | Startup time | Result |
|---------|-------|-------|---------|-------------|--------|
| Default | 300ms@3A | 5A | 3000ms | ~4.0s | Grinding on ground |
| Ground | 500ms@5A | 8A | 4000ms | ~5.2s | Better, some grinding |
| Conservative | 700ms@6A | 10A | 5000ms | ~6.4s | No grinding, consistent |

---

### Step 6: Hardening & Optimization

**Goal:** Production-ready firmware.

- [x] Validate Vbus ADC reading on 3S LiPo — confirmed 12.4–12.9 V across ground tests (2026-04-07)
- [x] CORDIC hardware sin/cos — Session 1 (2026-04-06), fixed FUNCTION_COSINE bug in commit 12d1ac6
- [x] Overmodulation (OVM) — Session 2 (2026-04-07): PI Vmax raised to Vbus×2/π, float-level OVM clamp
- [x] ADC sampling window optimization — Session 2 (2026-04-07): CC4 = CCR_max + (ARR−CCR_max)/2 per tick
- [x] Fix circle limiter: use runtime Vbus instead of compile-time CFOC_PI_VMAX — Session 3 (2026-04-07)
- [x] Lower crossfade speed: 1600 → 1400 RPM bench + ground validated — Session 3 (2026-04-07)
- [x] Lower crossfade speed: 1400 → 1200 RPM bench + ground validated — Session 4 (2026-04-07)
- [ ] Smooth OL→CL transition (grinding + lurch at EKF lock) — Session 5 **UNRESOLVED, pick up next session** (see Session 5 notes below)
- [ ] Odometry from wheel speed + IMU — Session 6 (ROS2, no firmware dependency)
- [x] ~~Test on HOSIM car~~ — **OUT OF SCOPE**: HOSIM is a separate vehicle with different drivetrain
  characteristics and hardware availability. Firmware is AMORIL-validated; HOSIM port is its own
  work thread and should not block Step 6 closure.
- [x] ~~ROS2 nav stack integration~~ — **OUT OF SCOPE for Step 6**: Nav stack requires odometry,
  costmap config, and a full nav2 bringup — a substantial independent work thread. `/cmd_vel_stamped`
  interface is already nav2-compatible; the integration can proceed as Step 7 once odometry is ready.

> **Scope decision (2026-04-07):** Step 6 is closed when the three remaining firmware/ROS2 items
> are done: circle limiter fix, crossfade speed reduction, and wheel odometry. MultiCar (HOSIM)
> and Nav Stack are explicitly deferred — they are orthogonal scopes that should not inflate Step 6.
>
> **On minimum crossfade speed:** The physics set a hard floor. At 1600 RPM, BEMF=0.327 V and
> dead-time drop Vdt≈0.24 V (73% of BEMF). At 1200 RPM, Vdt ≈ 98% of BEMF — EKF voltage model
> becomes unreliable. Target is 1400 RPM for Session 3 (conservative step), then evaluate 1200 RPM
> in Session 4 if the crossfade is clean. 800 RPM is physically implausible with this motor at
> 800 ns dead time — Vdt would exceed BEMF entirely.
>
> **On odometry:** Pure ROS2 software, no firmware dependency. Requires knowing AMORIL #1 gear
> ratio and wheel circumference to convert `/esc/speed_rpm` → m/s. IMU fusion adds heading.
> Target: `nav_msgs/Odometry` on `/odom` at 10 Hz, rosbag-verified.

---

### Step 6 Session Log

#### Session 1 — 2026-04-06 (bench-validated, ground-validated 2026-04-07)

**Branch:** `custom_foc`  **Files:** `custom_foc.h`, `custom_foc.c`

| Change | Detail |
|---|---|
| CORDIC sin/cos | Replaced `sincosf()` in 25 kHz HF ISR. Bug in first commit (FUNCTION_SINE→wrong read order); fixed in 12d1ac6 to FUNCTION_COSINE, precision 6 cycles, defensive CSR re-issue every tick |
| Dual speed LPF | `ekf_omega_filt` τ=20 ms (angle + telemetry), `ekf_omega_pi` τ=100 ms (speed PI only). Goal: 97% attenuation of 53 Hz EKF noise vs 85% before |
| Adaptive Vdt | `cfoc_vdt_rt = CFOC_GetVbusV() × CFOC_VDT_PER_VBUS` updated every 100 ms in MF task. Replaces hardcoded 12 V in EKF voltage model |

**Bench bag** `rosbag2_2026_04_06-18_01_53`: 37.8 s, fwd+rev, 0 faults, ~2990 RPM at u=1.0.
**Ground bag** `rosbag2_2026_04_07-10_48_43`: 115 s, fwd only, 0 faults, max 3147 RPM, Iq stdev 1.769 A at u>0.8.

---

#### Session 2 — 2026-04-07 (ground-validated fwd + rev)

**Branch:** `custom_foc`  **Files:** `custom_foc.h`, `custom_foc.c`

| Change | Detail |
|---|---|
| SVM runtime Vbus | Added `cfoc_vbus_rt` (was missing — SVM_Apply had hardcoded 12 V). Updated every 100 ms alongside `cfoc_vdt_rt`. Fixes ~6% voltage error at 12.8 V Vbus |
| PI Vmax → OVM ceiling | `CFOC_PI_VMAX_PER_VBUS = 2/π ≈ 0.6366`. PI out_max updated at runtime: `cfoc_vbus_rt × 2/π`. Raises ceiling from 6.9 V (Vbus/√3) to 7.64 V (+15%). OVM clamp moved to float level (Va/Vb/Vc) before CCR conversion — symmetric six-step clipping |
| ADC sampling window | CC4 updated every ISR tick inside SVM_Apply: `CC4 = CCR_max + (ARR−CCR_max)/2`. ADC fires in settled all-low-side window regardless of duty cycle. Init (calibration phase) stays at ARR−1 |

**Ground bags (AMORIL #1):**

| Bag | Dir | Duration | Max RPM | Iq stdev @ u>0.8 | Faults |
|---|---|---|---|---|---|
| `rosbag2_2026_04_07-11_33_20` | Fwd+Rev | 136 s | 3006 / −2647 RPM | 1.266 / 1.016 A | none |
| `rosbag2_2026_04_07-11_41_33` | Fwd only | 124 s | 3091 RPM | **0.332 A** | none |

**Iq chatter improvement vs Session 1 baseline (u>0.8, sustained full throttle):**
- Pre-Session 2: 1.769 A stdev (rosbag 10:48)
- Post-Session 2: 0.332 A stdev (rosbag 11:41) → **−81%**

**Note:** Motor is Iq-limited (12 A clamp) at these speeds, not voltage-limited. OVM headroom not yet exercised. Will appear at higher `max_speed_rpm` or lower Iq limit.

**ROS2 node change** (`esc_node_custom_foc.py` in `mrad_ws_2601_zulu`):
- Replaced `Joy` subscriber with `cmd_vel_stamped` (`geometry_msgs/TwistStamped`)
- `twist.linear.x` → u: proportional in CLOSED_LOOP, fixed `u_startup` during startup phases
- New params: `max_linear_mps` (default 1.0 m/s), `cmd_vel_timeout` (default 0.5 s → neutral)
- Deadband scaled to m/s: `|vx| < deadband × max_linear_mps`

**IMU crash analysis** (`rosbag2_2026_04_07-11_41_33`):
- t+1.98 s OL abort: wheel spin during startup (gyro built 0→0.78 rad/s), not a crash — user released stick
- t+123.22 s: |a|=25.88 m/s² diagonal impact (ax=+16, ay=+20) — wall/corner hit in CLOSED_LOOP. Firmware held CLOSED_LOOP through impact, user stopped 1.4 s later. No fault triggered.

---

#### Session 3 — 2026-04-07 (firmware fixes, bench validation pending)

**Branch:** `custom_foc`  **Files:** `custom_foc.h`, `custom_foc.c`

**Goal:** Close two open firmware items from the Step 6 checklist.

| Change | Detail |
|---|---|
| Circle limiter bug fix | `CFOC_VMAX_RT` macro added to `custom_foc.h`. Circle limiter in `CFOC_HighFrequencyTask` now computes `vmax_rt = cfoc_vbus_rt × CFOC_PI_VMAX_PER_VBUS` per call and uses it for both the `Vmax_sq` threshold and the scale factor. Previously used the compile-time constant `CFOC_PI_VMAX` (12V nominal), which diverged from the runtime PI limits at any Vbus ≠ 12V. The mismatch was small (~0.08V at 12.8V) but created an inconsistency: PI outputs were clamped to the correct ceiling while the circle limiter used a stale one. |
| Crossfade speed 1600 → 1400 RPM | `CFOC_OL_TARGET_RPM` lowered from 1600 to 1400. Startup time reduces by ~0.4 s (saves ~400ms of OL ramp at same ramp rate). Physics check: at 1400 RPM, BEMF = Ψf × ω_e = 9.75e-4 × 293 rad/s = 0.286 V; Vdt ≈ 0.24 V → Vdt/BEMF = 84% (acceptable, dead-time comp is active in EKF). At 1200 RPM, Vdt/BEMF ≈ 98% — marginal; defer to Session 4 after 1400 RPM is validated. |

> **Why not 1200 RPM directly?** Conservative step-down avoids having to debug EKF divergence
> and crossfade failure simultaneously. If 1400 RPM validates cleanly on bench, Session 4 can
> drop to 1200 RPM with a single constant change.

**Validation results — DONE (2026-04-07):**

| Bag | Type | Duration | CL entries | Max RPM fwd | Max RPM rev | Faults |
|---|---|---|---|---|---|---|
| `rosbag2_2026_04_07-17_57_34` | Bench | 55.6s | 3 | +3169 | −3108 | none |
| `rosbag2_2026_04_07-18_04_22` | Ground | 144.9s | 5 | +3096 | −3024 | none |

- Crossfade at 1400 RPM fully stable — 8/8 CLOSED_LOOP entries across both bags, 0 faults.
- Startup time unchanged at ~6.4s (time-triggered ramp; reducing `ol_ramp_ms` is a Session 4 option).
- Major IMU impact during ground test (accel 13.6/18.1 m/s², ~2.2g diagonal at t=74.9s): CLOSED_LOOP held, no fault.
- Longest sustained CL run: 51.4s at avg u=0.967 (segment 1, ground bag). No degradation over time.
- **Next crossfade step (Session 4):** try 1200 RPM. Physics: Vdt/BEMF≈98% — marginal but compensation is active. One constant change (`CFOC_OL_TARGET_RPM 1200.0f`), bench validate first.

---

#### Session 4 — 2026-04-07 (bench + ground validation pending)

**Branch:** `custom_foc`  **Files:** `custom_foc.h`

**Goal:** Push crossfade handoff speed from 1400 → 1200 RPM. This is the physics floor — at 1200 RPM,
Vdt/BEMF=98%, meaning dead-time compensation is carrying almost all the voltage model accuracy.
Below 1200 RPM (e.g. 1000 RPM), Vdt > BEMF entirely and the EKF voltage model breaks down.

| Change | Detail |
|---|---|
| Crossfade speed 1400 → 1200 RPM | `CFOC_OL_TARGET_RPM` set to `1200.0f`. At 1200 RPM: BEMF=0.245V, Vdt≈0.24V, Vdt/BEMF=98%. The BEMF² threshold (0.05) now corresponds to ~1100 RPM equivalent — the guard may engage slightly before ramp completion at this speed. |

> **Physics floor note:** This is the last planned crossfade step. Going to 1000 RPM would require
> Vdt < BEMF which cannot be achieved without reducing dead time in hardware (DTG register) or
> accepting EKF divergence. 1200 RPM is the practical minimum for this motor+hardware combination.

**Expected outcome:** Startup time reduces by ~0.5s (motor reaches 1200 RPM instead of 1400 RPM).
Watch for: CROSSFADE failing to trigger (BEMF guard too tight), or EKF angle error causing
overcurrent at CL entry. If either occurs, revert to 1400 RPM.

**Validation results — DONE (2026-04-07):**

| Bag | Type | Duration | CL entries | Max RPM fwd | Max RPM rev | Faults |
|---|---|---|---|---|---|---|
| `rosbag2_2026_04_07-18_17_57` | Bench | 38.4s | 2 | +3139 | −3387 | none |
| `rosbag2_2026_04_07-18_19_36` | Ground | 259.2s | 9 | +3141 | −3072 | none |

- 11/11 CLOSED_LOOP entries, 0 faults across both bags. EKF locks at 1200 RPM despite Vdt/BEMF=98%.
- Major IMU impact (ground bag t=247.25s): gyro_z=4.88 rad/s (highest recorded), accel_y=19.60 m/s² (~2.0g). CLOSED_LOOP held, 0 faults.
- Longest validation run to date: 259.2s ground bag, 9 CL entries, mixed fwd/rev.
- Startup time unchanged at ~6.4s (time-triggered ramp; motor reaches 1200 RPM mid-ramp but waits for ol_ramp_ms).
- **1200 RPM confirmed as practical minimum.** Do not attempt 1000 RPM — Vdt > BEMF, EKF voltage model breaks down.
- **Crossfade speed reduction work COMPLETE.** Final: 1600→1400→1200 RPM over Sessions 3–4.
- **Known issue:** At EKF lock (CROSSFADE→CLOSED_LOOP), grinding noise + brief slowdown + lurch reported on ground. Root cause diagnosed → fixed in Session 5.

---

#### Session 5 — 2026-04-07 (UNRESOLVED — pick up next session)

**Branch:** `custom_foc`  **Files:** `custom_foc.c`, `custom_foc.h`, `esc_app.c`

**Goal:** Eliminate grinding + stop-then-lurch at CROSSFADE→CLOSED_LOOP transition.

**Status: NOT CLOSED.** After three iterations (v1, v2, v3) the behavior is still inconsistent.
User's subjective assessment at end of day: "operation is less robust than before, even the tests
this morning" — i.e., current code (v3) may have regressed from the Session 3 baseline.

---

##### v1 — Initial fix (2026-04-07 afternoon)

Initial root-cause hypothesis: two independent problems at CL entry.

1. **Grinding / slowdown during CROSSFADE** — `ekf_omega_pi` (τ=100ms LPF, feeds speed PI) was
   not seeded at crossfade entry, starts lagged below actual OL speed.
2. **Stop-then-lurch at CL entry** — `pi_spd.integral` reset to 0 + speed command jump from
   1200 RPM to `u × max_speed_rpm` in one tick → PI error saturates Iq.

**v1 fixes applied:**

| Fix | File | Change |
|---|---|---|
| A — Bumpless PI entry | `custom_foc.c` | `pi_spd.integral = ol_Iq_ref` at CL entry instead of 0. |
| B — Seed ekf_omega_pi | `custom_foc.c` | `ekf_omega_pi = ol_omega_e` at crossfade entry. |
| C — Speed cmd to actual EKF | `custom_foc.c` | `spd_cmd_rpm = ekf_rpm` at CL entry — not `OL_TARGET_RPM`. |
| D — ESC slew seeded to actual | `esc_app.c` | `esc_speed_cmd = CFOC_GetSpeedRPM()` on first CL tick. |

**v1 ground test (bags 18_55_41, 18_57_14):** Still grinding. Correlation analysis showed:

```
Bag 18_55_41:  #1 RPM@CL=+459  Iq@CL=-0.03  ROUGH   ← impossible RPM drop
               #5 RPM@CL=-1314 Iq@CL=-8.17  SMOOTH
Bag 18_57_14:  #1 RPM@CL=+458  Iq@CL=-0.06  ROUGH
               #2 RPM@CL=+1344 Iq@CL=+8.16  SMOOTH
```

Pattern: ~30% of transitions were SMOOTH (Iq@CL ≈ 8-10A, RPM near crossfade speed). The rest
were ROUGH with RPM dropping from ~1440 to ~460 in the 100ms window at CL entry and Iq collapsing
to zero — **physically impossible** inertia-wise, so it's a control artifact.

##### v2 — EKF gate + correct PI seed (2026-04-07 evening)

Code review revealed two bugs in v1:

**Bug 1 — v1's Fix C used raw `ekf_rpm` (unfiltered):**
The speed PI uses `ekf_omega_pi / RPM_TO_ERAD_S` as feedback (custom_foc.c:734). Seeding
`spd_cmd_rpm = ekf_rpm` (raw) at 1200 RPM with Vdt/BEMF=98% could produce a large one-tick
speed error that corrupts the PI integral (even though ESC overrides `spd_cmd_rpm` 1ms later).

**Bug 2 — angle integration speed collapses at CL entry:**
During CROSSFADE, angle uses `omega_blend = (1-α)×ol_omega_e + α×ekf_omega_filt`. In CLOSED_LOOP,
it switches to pure `ekf_omega_filt`. If `ekf_omega_filt` has drifted low during the crossfade
tail (as α→1 the `ol_omega_e` anchor disappears), the commutation angle suddenly slows → torque
drops → motor physically decelerates. This explains the real (not just telemetry-aliased) RPM drop.

**v2 fixes applied on top of v1:**

| Fix | File | Change |
|---|---|---|
| E — EKF convergence gate | `custom_foc.c` | At end of crossfade, require `\|ekf_omega_filt - ol_omega_e\| < 20%` before entering CL. If not met, hold at `xf_blend_ms = CFOC_XF_DURATION_MS` (α=1.0) and wait. Hard cap: +500ms max extension. |
| F — PI seed from filtered signal | `custom_foc.c` | `spd_cmd_rpm = ekf_omega_pi / RPM_TO_ERAD_S` (the PI's own feedback LPF) — guarantees `speed_err = 0` on first CL tick regardless of raw EKF noise. Replaces v1 Fix C. |

**v2 ground test (bag 19_25_25):** Still inconsistent. 11 transitions, reverse worse than forward:

```
Forward:  #1 +1034/-0.05  #2 +1402/+3.38  #3 +429/-0.01  #4 +1319/+7.91  #5 +1551/+1.93
Reverse:  #6  -920/+0.03  #7 -1023/+0.06  #8 -1403/-8.52 #9 -509/-0.03 #10 -1414/-9.50 #11 -464/+0.02
```

Of the 6 reverse transitions, 4 were rough (Iq ≈ 0). User observed: "grinding noise is worse
in reverse" and "forward works with some help from joystick cmd". The EKF gate does limit the
timing but can't prevent `ekf_omega_filt` from collapsing in the first 100ms of CL when the raw
EKF estimate becomes noisy at 1200 RPM.

##### v3 — Revert OL target to 1400 RPM (2026-04-07 late evening)

**Decision:** At 1200 RPM, Vdt/BEMF ≈ 98% is the physics floor. The EKF simply cannot reliably
estimate speed there. Session 3 validated 1400 RPM (Vdt/BEMF ≈ 84%, 16% margin) with 0 faults.
Revert OL target to 1400, keep v2 fixes (E, F) and v1 fixes (A, B, D) as they are still
unconditionally correct.

**v3 change:** `CFOC_OL_TARGET_RPM: 1200 → 1400` (`custom_foc.h:75`).

**v3 ground test (bags 19_38_44, 19_41_21):** User reports operation feels **worse** than
morning Session 3/4 tests. Transition summary:

```
Bag 19_38_44 (v3, 1400 RPM):
  #1 +1536/+3.84  partial    #2 -1579/-9.95 SMOOTH     #3  +490/-0.06 ROUGH
  #4  +683/+9.90  WEIRD (Iq ok, RPM low)
  #5 +1296/+0.12  ROUGH      #6  +537/+0.01 ROUGH

Bag 19_41_21 (v3, 1400 RPM):
  #1 -1202/+0.06  ROUGH (direction negative but Iq positive — sign inversion??)
  #2  +682/-0.01  ROUGH
```

**Key anomaly in bag 19_41_21 #1:** motor speed is -1202 RPM (reverse) but Iq is +0.06 A
(positive sign). Park transform or direction handling regression?

---

##### Current code state (end of 2026-04-07)

| Location | State |
|---|---|
| `custom_foc.h:75` | `CFOC_OL_TARGET_RPM = 1400.0f` (reverted from 1200, v3) |
| `custom_foc.c:657-658` | Fix B: `ekf_omega_filt = ekf_omega_pi = ol_omega_e` at crossfade entry |
| `custom_foc.c:687-746` | Fix E (EKF gate) + Fix F (`spd_cmd_rpm = ekf_omega_pi / RPM_TO_ERAD_S`) + Fix A (`pi_spd.integral = ol_Iq_ref`) |
| `esc_app.c:196-203` | Fix D: `esc_speed_cmd = CFOC_GetSpeedRPM()` on first CL tick |

**None of these are validated.** The v3 state is what's in the tree right now.

---

##### Hypotheses to investigate next session

1. **v2/v3 may have regressed from Session 3 baseline.** User's subjective feedback is the
   most important data point: "less robust than morning". Session 3 bag 18_04_22 was the last
   known-good ground test. Compare transitions there vs 19_38_44 quantitatively.
   - **Action:** run `analyze_cl_transition.py` on bag 18_04_22 (Session 3) and diff against
     19_38_44. If Session 3 had mostly smooth transitions, the regression is in v2 (Fix E or F).

2. **The EKF gate (Fix E) may be holding crossfade too long.** Extending crossfade by up to
   500ms keeps the motor at pure-EKF angle integration (α=1.0) for up to 1000ms total. If the
   EKF is noisy during that extension, the angle accumulates error and torque degrades.
   - **Action:** try disabling Fix E (skip the gate, commit at 500ms always) while keeping Fix F.

3. **Fix F (`spd_cmd_rpm = ekf_omega_pi / RPM`) may interact badly with ESC overriding 1ms later.**
   CFOC sets `spd_cmd_rpm` at CL entry → PI runs 1 tick with zero error → ESC overrides with
   `CFOC_GetSpeedRPM()` which is `ekf_omega_filt / RPM_TO_ERAD_S` (a DIFFERENT LPF). On the
   second tick, `speed_err = ekf_omega_filt/RPM − ekf_omega_pi/RPM` = the difference between
   two LPFs of the same signal. Small but non-zero.
   - **Action:** make CFOC and ESC use the SAME LPF. Either expose `ekf_omega_pi` via a new
     getter and have ESC use it, or have CFOC seed from `ekf_omega_filt` instead.

4. **Reverse-direction Iq sign inversion (bag 19_41_21 #1).** Telemetry shows Iq=+0.06 for a
   motor running at -1202 RPM. Expected: Iq should have the direction sign. Is it a telemetry
   reporting quirk (Iq_ma is signed but derived from Park with ol_theta_e which is already
   direction-signed), or a real control issue?
   - **Action:** inspect `esc_send_telemetry` in `esc_app.c` and the Park transform path.
     Check if Iq reporting matches the direction convention used by ol_Iq_ref.

5. **Hardware/mechanical factor.** "Reverse worse than forward" persisted across all v1/v2/v3
   tests on the same hardware. Asymmetric motor characteristics (Ke, Rs, commutation) between
   directions? Wheel friction at start? Bearing asymmetry?
   - **Action:** bench-test at no-load both directions and compare. If no-load is symmetric,
     the asymmetry is load/traction related. If no-load is also asymmetric, it's electrical.

6. **Consider full revert to Session 3 baseline as a known-good fallback.** If tomorrow's
   analysis confirms v2/v3 regressed, revert all v1/v2/v3 changes and restart from Session 3.
   - `git diff 5f27d90 -- motor_test1_20260227/` shows exactly what changed since Session 3.

---

##### What to keep vs drop (my recommendation for tomorrow)

| Change | Keep? | Rationale |
|---|---|---|
| v1 Fix A (`pi_spd.integral = ol_Iq_ref`) | **KEEP** | Unconditionally correct; zero downside |
| v1 Fix B (`ekf_omega_pi = ol_omega_e` seed) | **KEEP** | Correct; matches v1 Fix A's intent |
| v1 Fix C (`spd_cmd_rpm = ekf_rpm` raw) | **DROP** | Superseded by v2 Fix F |
| v1 Fix D (`esc_speed_cmd = CFOC_GetSpeedRPM()`) | **REVIEW** | See hypothesis 3 — may conflict with Fix F |
| v2 Fix E (EKF convergence gate) | **TEST WITHOUT IT** | Hypothesis 2 — may be causing worse behavior |
| v2 Fix F (`spd_cmd_rpm = ekf_omega_pi / RPM`) | **REVIEW** | See hypothesis 3 |
| v3 (`CFOC_OL_TARGET_RPM = 1400`) | **KEEP** | Physics-motivated; 1200 was too close to floor |

**Recommended first experiment tomorrow:** disable Fix E (EKF gate), keep everything else at v3
state, ground test. If that recovers Session 3 reliability, the gate is the regression. If not,
move to hypothesis 3 (align CFOC/ESC on the same LPF).

---

##### Relevant rosbag inventory

| Bag | Context | Key result |
|---|---|---|
| `rosbag2_2026_04_07-17_57_34` | Session 3 bench (1400 RPM, circle limiter fix) | 0 faults |
| `rosbag2_2026_04_07-18_04_22` | Session 3 ground (1400 RPM) | **Last known-good baseline** |
| `rosbag2_2026_04_07-18_39_58` | Session 4 ground (1200 RPM) | Worked but grinding reported |
| `rosbag2_2026_04_07-18_42_33` | Session 4 ground (1200 RPM) | Same |
| `rosbag2_2026_04_07-18_55_41` | Session 5 v1 ground | Still grinding; 4/5 rough |
| `rosbag2_2026_04_07-18_57_14` | Session 5 v1 ground | 5/7 rough |
| `rosbag2_2026_04_07-19_25_25` | Session 5 v2 ground (1200 RPM + gate + Fix F) | 4/6 reverse rough |
| `rosbag2_2026_04_07-19_38_44` | Session 5 v3 ground (1400 RPM + gate + Fix F) | 4/6 rough; user says worse feel |
| `rosbag2_2026_04_07-19_41_21` | Session 5 v3 ground | 2/2 rough; Iq sign anomaly #1 |

Analysis tool: `tests/analyze_cl_transition.py <bag_dir>`

---

## 6. File Structure (Final)

```
motor_test1_20260227/
├── Inc/
│   ├── custom_foc.h              ← NEW: public API + config
│   ├── esc_comm.h                ← KEEP
│   ├── esc_ekf_observer.h        ← KEEP
│   ├── pmsm_motor_parameters.h   ← KEEP (motor Rs, Ls, p)
│   ├── drive_parameters.h        ← KEEP (EKF tuning params)
│   └── main.h                    ← KEEP (HAL config)
├── Src/
│   ├── main.c                    ← MODIFY: call CFOC_Init() instead of MCboot()
│   ├── custom_foc.c              ← NEW: FOC ISR + state machine + PI + SVM
│   ├── esc_comm.c                ← KEEP
│   ├── esc_ekf_observer.c        ← KEEP
│   └── stm32g4xx_it.c            ← MODIFY: route TIM1 ISR to our handler
├── MCSDK_v6.4.1-Full/            ← EXCLUDE from build (keep for reference)
└── STM32CubeIDE/
    └── .cproject                 ← MODIFY: update source includes
```

---

## 7. Critical Technical Details

### ADC Current Reading

The B-G431B-ESC1 uses 3-shunt current sensing through OPAMP1/2/3:
- **Phase A:** ADC1 injected rank 1, channel 3 (OPAMP1 output)
- **Phase B:** ADC1 injected rank 2, channel 12 (OPAMP2 output)  
- **Phase C:** ADC2 injected rank 1, VOPAMP3 internal channel

Trigger: TIM1 TRGO = OC4REF rising edge (center of PWM ON-time).
**Note:** `TIM1_CH4` trigger source did not work; `TIM1_TRGO` works reliably.

**Offset calibration:** Self-calibrates inside the 25 kHz ISR — first 64 samples
at 50% duty (zero current) are accumulated, then averaged. No blocking loops.
Measured offsets: Phase A ≈ 2532, Phase B ≈ 1453 (OPAMP-dependent).

**Current conversion:**
```c
// ADC is 12-bit left-aligned → must >> 4 for true 12-bit value
// Current = (ADC_offset - ADC_raw) × (Vref / 4096) / (Rshunt × Gain)
// B-G431B-ESC1: Rshunt = 0.003Ω, Gain = 9.14 (OPAMP PGA configuration)
// Scale factor: 3.3V / (4096 × 0.003 × 9.14) ≈ 0.02938 A/count
#define CFOC_ADC_TO_AMPS  (CFOC_VREF / (4096.0f * CFOC_RSHUNT * CFOC_AMP_GAIN))
```

**ADC init sequence (must use LL, not HAL):**
```c
HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);  // calibrate (disables ADC)
LL_ADC_Enable(ADC1);                                      // re-enable via LL
LL_ADC_INJ_ConfigQueueContext(ADC1, ...TIM1_TRGO...);    // configure JSQR
LL_ADC_INJ_StartConversion(ADC1);                         // arm for trigger
```

### SVM (Space Vector Modulation)

Standard 7-segment center-aligned, no overmodulation (Step 2):

```c
// Input: Vα, Vβ in [-1, 1] normalized to Vbus/√3
// Output: duty_a, duty_b, duty_c in [0, PWM_PERIOD]

// Sector detection
float X = Vβ;
float Y = (√3/2)·Vα - (1/2)·Vβ;
float Z = -(√3/2)·Vα - (1/2)·Vβ;
int sector = (X>0) | ((Y>0)<<1) | ((Z>0)<<2);  // lookup table → 1-6

// T1, T2 calculation (sector-dependent)
// T0 = PWM_PERIOD - T1 - T2
// Duty cycles = center-aligned placement of T0/T1/T2
```

### PI Controller (Simplified)

```c
typedef struct {
    float Kp, Ki;
    float integral;
    float out_min, out_max;
} PI_t;

float PI_Run(PI_t *pi, float error) {
    pi->integral += pi->Ki * error;
    // Anti-windup clamp
    if (pi->integral > pi->out_max) pi->integral = pi->out_max;
    if (pi->integral < pi->out_min) pi->integral = pi->out_min;
    float out = pi->Kp * error + pi->integral;
    // Output clamp
    if (out > pi->out_max) out = pi->out_max;
    if (out < pi->out_min) out = pi->out_min;
    return out;
}
```

### Crossfade Angle Blending

```c
// Ensure angles are within π of each other before blending
float diff = theta_ekf - theta_ol;
if (diff > M_PI)  diff -= 2*M_PI;
if (diff < -M_PI) diff += 2*M_PI;
float theta = theta_ol + alpha * diff;  // alpha: 0→1 over 50ms
// Normalize to [-π, π]
if (theta > M_PI)  theta -= 2*M_PI;
if (theta < -M_PI) theta += 2*M_PI;
```

---

## 8. Risk Assessment

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| SVM sector errors → shoot-through | Medium | Hardware break (COMP1/2/4) protects against overcurrent. Test with low duty first. |
| ADC timing wrong → current distortion | Medium | Start with mid-PWM sampling (CC4 trigger already set). Add sector-dependent window later. |
| PI instability → current oscillation | Low | Start with low gains, increase gradually. Hardware break is safety net. |
| EKF crossfade angle jump | Medium | Angle unwrap + slow 50ms blend. If jump detected, abort to IDLE. |
| Open-loop → closed-loop speed mismatch | Medium | EKF speed should match OL speed at crossfade. If >20% mismatch, don't crossfade. |

---

## 9. Estimated Timeline

| Step | Description | Code | Test |
|------|-------------|------|------|
| 1 | Bare ISR + ADC | ~80 lines | Scope + debugger |
| 2 | Open-loop + PI + SVM | ~200 lines | Motor spins |
| 3 | EKF crossfade | ~60 lines | CLOSED_LOOP reached |
| 4 | Speed PI | ~40 lines | Speed hold |
| 5 | ESC integration | ~150 lines | Full bench test |
| 6 | Hardening | ~100 lines | Multi-car |
| **Total** | | **~630 lines** | |

Each step is independently testable on hardware. If Step 2 works (motor spins in
open-loop), the EKF crossfade in Step 3 should succeed — there are no opaque state
machine checks to fight.

---

## 10. Success Criteria

**Minimum viable:** Motor reaches CLOSED_LOOP (Step 3) and holds speed for 10+ seconds.

**Full success:** Drop-in replacement for MCSDK firmware — RPi5 sends commands, motor
responds, telemetry flows, faults handled — identical external behavior to the working
MCSDK firmware but with EKF-based sensorless control that works at all speeds.
