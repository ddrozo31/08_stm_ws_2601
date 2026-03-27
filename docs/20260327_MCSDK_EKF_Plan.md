# MCSDK EKF Observer — Implementation Plan

**Branch:** `MCSDK_EKF`
**Date:** 2026-03-27
**Base:** `HOSIM` branch (iter3 — Phase 5 at 8A, 3000ms)

---

## 1. Motivation

### Why the current observer fails across drivetrains

The MCSDK uses a **Luenberger State Observer + PLL** for sensorless rotor angle estimation.
The observer models the stator current error between measured and estimated values, extracts
the BEMF vector (Eα, Eβ), then feeds a PLL to recover angle and speed.

The fundamental problem for variable-load applications:

```
BEMF = Ke × ω   →   at low RPM, BEMF → 0

SNR = BEMF / (Rs × I) = (Ke × ω) / (Rs × I)

When SNR < 1: Rs×I dominates → observer locks to 180° wrong-angle solution
```

The HOSIM tuning journey exposed this clearly:

| Iter | Phase 5 | RPM | SNR | Ground result |
|------|---------|-----|-----|---------------|
| 1 | 6 A | 1700 | 0.71 | ✗ wrong-angle crash |
| 2 | 4 A | 1600 | 1.00 | ✓ bench / ✗ ground (4A can't hold RPM under load) |
| 3 | 8 A | 1600+ | 0.50 | pending — SNR < 1 but NB_CONSECUTIVE_TESTS=12 filters it |

Every iteration is fighting a trade-off between current (torque to hold RPM) and current
(noise that buries BEMF). The Luenberger observer has **fixed gains** — it cannot adapt
to the noise floor of the specific drivetrain.

### What an EKF provides

An Extended Kalman Filter is an optimal estimator for noisy nonlinear systems. Applied to
PMSM sensorless control it provides:

- **Tunable noise model (Q, R matrices)**: explicitly separates process noise from
  measurement noise. The observer adapts to the actual SNR rather than using fixed gains.
- **Better convergence at SWITCH_OVER**: the EKF's covariance propagation naturally handles
  the high-noise, low-BEMF regime at the transition point.
- **Lower minimum speed floor**: literature shows BEMF-based EKF working reliably at
  200–600 RPM for SPM motors vs. ~1600 RPM with Luenberger+PLL.
- **Drivetrain agnostic**: once Q/R are tuned on one vehicle, the same firmware works
  across HOSIM, AMORIL, and future drivetrains without phase profile changes.

### What the EKF does NOT fix

- **Zero-speed startup**: BEMF ∝ ω; at standstill BEMF = 0. No observer based on BEMF can
  estimate angle at zero speed for an SPM outrunner (no saliency, HFI inapplicable).
  The open-loop rev-up phase is still required.
- **Parameter sensitivity**: Rs, Ls, Ψf errors affect EKF just as they affect Luenberger.
  Well-characterised motor parameters remain a prerequisite.

### Success criterion

> The same firmware (same Q/R, same OBS_MINIMUM_SPEED_RPM) achieves SWITCH_OVER reliably
> on both HOSIM (heavy drivetrain) and AMORIL (lighter drivetrain) without per-vehicle
> phase profile tuning. Target: OBS_MINIMUM_SPEED_RPM ≤ 600 RPM.

---

## 2. Motor / System Parameters

These are required inputs for the EKF model. Current values from Motor Pilot measurements:

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Stator resistance | Rs | 0.100 Ω | Motor Pilot |
| Stator inductance | Ls | 10.0 μH | Motor Pilot |
| Back-EMF constant | Ke | 0.25 V/kRPM (mech) | derived |
| PM flux linkage | Ψf | Ke / (p × 2π/60) | ~0.00076 V·s/rad |
| Pole pairs | p | 2 | motor spec |
| PWM / FOC rate | Ts_foc | 40 μs (25 kHz) | drive_parameters.h |
| Speed loop rate | Ts_spd | 1 ms (1 kHz) | drive_parameters.h |
| DC bus voltage | Vbus | ~11.1 V (3S LiPo) | telemetry |
| Current sensing | — | inline shunts, ±15 A range | hardware |

**Step 1 task:** Verify Ψf by cross-checking Motor Pilot output with open-circuit BEMF
measurement at known RPM (spin motor by hand with oscilloscope on any phase pair).

---

## 3. PMSM EKF State Model

### 3.1 State vector

We use the **extended BEMF** representation in the stationary (αβ) frame:

```
x = [iα, iβ, eα, eβ]ᵀ
```

where the extended BEMF components are:
```
eα = -Ψf · ω · sin(θ)
eβ =  Ψf · ω · cos(θ)
```

From these we recover:
```
θ  = atan2(-eα, eβ)
ω  = √(eα² + eβ²) / Ψf
```

This is the standard Chen / Piippo / Kim formulation (see references). It avoids
explicit ω in the state vector (reducing Jacobian complexity) and is the most
common embedded EKF form for PMSM.

### 3.2 Continuous state equations

```
diα/dt = (Vα - Rs·iα - eα) / Ls
diβ/dt = (Vβ - Rs·iβ - eβ) / Ls
deα/dt = -ω · eβ      (BEMF rotates at electrical speed)
deβ/dt =  ω · eα
```

The BEMF dynamics require ω, which we compute from eα, eβ at each step (not a state).

### 3.3 Discrete model (Euler, Ts = Ts_foc)

```
x[k+1] = f(x[k], u[k])

iα[k+1] = iα[k] + Ts/Ls · (Vα[k] - Rs·iα[k] - eα[k])
iβ[k+1] = iβ[k] + Ts/Ls · (Vβ[k] - Rs·iβ[k] - eβ[k])
eα[k+1] = eα[k] - ω[k] · eβ[k] · Ts
eβ[k+1] = eβ[k] + ω[k] · eα[k] · Ts

where ω[k] = √(eα[k]² + eβ[k]²) / Ψf
```

Measurement model (we measure stator currents):
```
y[k] = [iα[k], iβ[k]]ᵀ  +  v[k]
```

### 3.4 Jacobian F (linearisation of f around x[k])

```
F = ∂f/∂x =

[ 1 - Ts·Rs/Ls    0          -Ts/Ls       0       ]
[    0         1-Ts·Rs/Ls      0        -Ts/Ls    ]
[    0             0        1-ω·Ts·∂eβ/∂eα  ...  ]  (BEMF coupling terms)
[    0             0           ...           ...  ]
```

Full 4×4 Jacobian derivation: see Section 7 (Reference Equations).

### 3.5 Noise covariance matrices

**Process noise Q** (model uncertainty — dominated by ω estimation error in BEMF dynamics):
```
Q = diag([q_i, q_i, q_e, q_e])

Typical starting point:
  q_i = (Ts/Ls)² · σ_V²    (voltage measurement/PWM uncertainty)
  q_e = (Ψf · Δω · Ts)²    (speed variation between steps)
```

**Measurement noise R** (current sensing noise):
```
R = diag([r_i, r_i])

r_i = σ_I²   (current ADC noise variance — characterise from motor-off ADC samples)
```

**Tuning strategy:** Start with R fixed (measured), tune Q by observing convergence speed
vs. noise rejection. Larger Q → faster convergence, less filtering (trusts model less).
Smaller Q → smoother angle estimate, slower convergence.

---

## 4. Implementation Steps

### Step 1 — Motor parameter verification (prerequisite)

**Goal:** Confirm Rs, Ls, Ψf to ±10%.

Actions:
1. Motor Pilot: record Rs, Ls from existing measurements in `pmsm_motor_parameters.h`.
2. Open-circuit BEMF check: spin motor shaft by hand (or drill), measure peak phase
   voltage on oscilloscope at a known RPM. Compute Ke = V_peak / RPM.
3. Cross-check Ψf = Ke / (p × 2π/60) agrees with Motor Pilot value.
4. Record σ_I: log `esc/iq_ma` and `esc/id_ma` with motor stopped — compute variance.
   This directly sets R matrix diagonal.

**Deliverable:** Confirmed parameter table in this doc (Section 2 above, update values).

---

### Step 2 — Python EKF prototype (offline validation)

**Goal:** Validate EKF model and Q/R tuning on real rosbag data before any firmware work.

**Setup:** The telemetry frame already publishes `esc/iq_ma`, `esc/id_ma` (= iα, iβ in the
sensor frame at the time of sampling). The applied voltage vector (Vα, Vβ) is not directly
published but can be reconstructed from the MCSDK modulation index and Vbus.

Actions:
1. Write `tests/ekf_pmsm.py` — standalone Python EKF class:
   ```
   class PMSM_EKF:
       def __init__(self, Rs, Ls, psi_f, Ts, Q, R): ...
       def predict(self, Va, Vb): ...
       def update(self, ia, ib): ...
       def angle(self) -> float: ...  # radians
       def speed_rpm(self) -> float: ...
   ```
2. Replay one of the 2026-03-27 rosbags through the EKF.
3. Compare EKF speed estimate vs. `esc/speed_rpm` (the MCSDK observer's output).
4. Observe EKF behaviour during SWITCH_OVER transition — does it track cleanly?
5. Try progressively lower Vα/Vβ (simulating lower RPM) — find the minimum speed
   where EKF angle is stable (< ±15° error vs. MCSDK reference).
6. Document optimal Q/R values.

**Deliverable:** `tests/ekf_pmsm.py` with unit test + Q/R parameter table.

---

### Step 3 — Timing and fixed-point analysis

**Goal:** Confirm the EKF fits within the STM32G431 timing budget.

STM32G431 facts:
- 170 MHz, FPU (single-precision float32), no double-precision in hardware
- FOC ISR: 25 kHz → 6,800 cycles per period (tight for a matrix EKF)
- Speed loop: 1 kHz → 170,000 cycles per period (very comfortable)

**Decision:** Run the EKF at the **speed loop rate (1 kHz)**, not the FOC rate.

At 1 kHz the EKF computes a new angle estimate every 1 ms. The FOC ISR interpolates the
angle between EKF updates using the last estimated ω (same approach MCSDK uses with its
internal PLL). This is standard practice in embedded sensorless PMSM.

**4-state float32 EKF at 1 kHz on G431:**
- State predict: ~30 multiply-adds
- Jacobian: ~16 elements, ~50 operations
- P predict: Fᵀ×P×F + Q: ~64+16 = ~80 multiply-adds
- K = P×Hᵀ×(H×P×Hᵀ+R)⁻¹: 2×2 matrix inverse (trivial for diagonal R)
- State update + P update: ~40 operations
- Total: ~250 float ops → ~500 cycles at 1 instruction/cycle → **0.3% CPU at 1 kHz**

Actions:
1. Benchmark with a DWT cycle counter around EKF update in a test build.
2. Confirm float32 angle precision is sufficient (int16 angle = 65536 counts/2π rev →
   LSB = 0.0055°; float32 has 7 decimal digits → more than adequate).

**Deliverable:** DWT timing measurement (target < 5000 cycles at 1 kHz).

---

### Step 4 — C implementation: `esc_ekf_observer.c / .h`

**Goal:** Clean, self-contained EKF module, no MCSDK dependencies.

File layout:
```
motor_test1_20260227/
  Inc/esc_ekf_observer.h    — public API
  Src/esc_ekf_observer.c    — EKF implementation
```

Public API:
```c
typedef struct {
    float x[4];       // state: [ia, ib, ea, eb]
    float P[4][4];    // covariance
    float Q[4][4];    // process noise
    float R[2][2];    // measurement noise
    float Rs, Ls, psi_f, Ts;
} EKF_Handle_t;

void  EKF_Init(EKF_Handle_t *h, float Rs, float Ls, float psi_f, float Ts,
               float q_i, float q_e, float r_i);
void  EKF_Update(EKF_Handle_t *h, float Va, float Vb, float ia_meas, float ib_meas);
float EKF_GetAngle(const EKF_Handle_t *h);      // electrical angle, radians
float EKF_GetSpeedRPM(const EKF_Handle_t *h);   // mechanical RPM
```

Compile-time switch in `drive_parameters.h`:
```c
#define USE_EKF_OBSERVER    1    // 0 = legacy STO+PLL, 1 = EKF
```

Actions:
1. Implement `EKF_Init` — initialise x=0, P=I×large, set Q/R from parameters.
2. Implement `EKF_Update`:
   a. Compute ω from eα, eβ
   b. Predict: x̂⁻ = f(x, u), P⁻ = F×P×Fᵀ + Q
   c. Update: K = P⁻×Hᵀ×(H×P⁻×Hᵀ + R)⁻¹, x = x̂⁻ + K×(y - H×x̂⁻), P = (I-K×H)×P⁻
3. Implement `EKF_GetAngle` = atan2f(-ea, eb)
4. Implement `EKF_GetSpeedRPM` = sqrtf(ea²+eb²)/psi_f × 60/(2π×p)
5. Unit test in `tests/esc_ekf_test.c`: step response, convergence from x=0, no NaN.

**Deliverable:** Compilable, tested `esc_ekf_observer.c/.h`.

---

### Step 5 — MCSDK integration

**Goal:** Feed EKF angle output into MCSDK's FOC without modifying SDK internals.

MCSDK angle/speed interface — key structure:
```c
// In SpeednPosFdbk.h:
typedef struct {
    int16_t hElAngle;           // electrical angle (int16, full circle = 65536)
    int16_t hAvrMecSpeedUnit;   // averaged mechanical speed in SPEED_UNIT
    ...
} SpeednPosFdbk_Handle_t;
```

The `STO_PLL_M1` handle (defined in `mc_config.c`) populates `hElAngle` and
`hAvrMecSpeedUnit` once per speed loop cycle via `SPD_CalcElAngle()`.

**Integration strategy:** Hook into the speed loop callback in `mc_app_hooks.c`
(already our custom code) where `MC_GetMecSpeedAverageMotor1()` is called.
After the speed loop runs, overwrite `STO_PLL_M1.SPD_Handle.hElAngle` with the
EKF angle if `USE_EKF_OBSERVER == 1`.

More precisely:
1. Call `EKF_Update()` once per speed loop tick (1 kHz) with the latest Vα, Vβ, iα, iβ.
2. Convert EKF angle (float radians) to MCSDK int16: `hElAngle = (int16_t)(θ / (2π) × 65536)`.
3. Write to `STO_PLL_M1.SPD_Handle.hElAngle`.
4. Optionally: write EKF speed to `hAvrMecSpeedUnit` for the speed controller.

**Vα, Vβ input:** Available from MCSDK via `MC_GetPhaseCurrentMotor1()` direction or via
`pDrive->pFOCVars->Vα` (check `FOCVars_t` in `mc_interface.h`).

Actions:
1. Locate `pFOCVars` or equivalent struct that holds the αβ voltage reference.
2. Add EKF handle initialisation in `ESC_Init()` (or equivalent init hook).
3. Add EKF update call in the 1 kHz speed loop hook.
4. Add angle/speed write-back with `USE_EKF_OBSERVER` guard.
5. Test: compile with `USE_EKF_OBSERVER=0` (must be identical to current HOSIM build).
   Then `USE_EKF_OBSERVER=1` — verify no compile errors, no runtime fault at startup.

**Deliverable:** Working build with EKF feeding the MCSDK angle pipeline.

---

### Step 6 — OBS_MINIMUM_SPEED_RPM reduction campaign

**Goal:** Progressively lower the SWITCH_OVER threshold and validate on hardware.

Protocol for each threshold reduction:
1. Set new `OBS_MINIMUM_SPEED_RPM` and matching `ESC_REVUP_SPEED_RPM`.
2. Build and flash. Run 5 consecutive starts on HOSIM (ground, full load).
3. Record: did SWITCH_OVER fire? Did RUN sustain? Any wrong-angle crashes?
4. Repeat on AMORIL.
5. If both vehicles pass 5/5: proceed to next reduction. If not: investigate, adjust Q/R.

Reduction schedule:

| Stage | OBS_MINIMUM_SPEED_RPM | Notes |
|-------|----------------------|-------|
| Baseline (HOSIM iter3) | 1600 | Current — EKF replaces Luenberger, same threshold |
| Stage A | 1200 | 25% reduction |
| Stage B | 800 | 50% reduction |
| Stage C | 600 | Target — should eliminate Phase 5 step-down need |
| Stage D | 400 | Stretch goal |

At Stage C/D: if RPM target is achievable in Phase 4, Phase 5 can potentially be
**eliminated entirely**, making startup faster and removing the current/SNR trade-off.

**Deliverable:** Validated minimum speed for each drivetrain, recorded in this doc.

---

### Step 7 — NB_CONSECUTIVE_TESTS re-tuning

**Goal:** With EKF providing cleaner angle estimates, reduce the false-positive filter.

Current value: `NB_CONSECUTIVE_TESTS = 12` — observer must report valid speed for 12
consecutive ms before SWITCH_OVER fires. This was added to filter 180° wrong-angle
transients at SWITCH_OVER (observed at 1500 RPM with Luenberger+PLL).

With EKF, the angle estimate should be smoother at the transition. Run the same 5-start
protocol at the validated Stage C/D speed while reducing NB_CONSECUTIVE_TESTS from 12
toward 4 (the AMORIL #1 baseline).

Reducing this value makes SWITCH_OVER faster and less sensitive to momentary RPM dips
during Phase 5.

**Deliverable:** Validated NB_CONSECUTIVE_TESTS for EKF builds.

---

## 5. The Full Picture

```
Open-loop rev-up (Phases 1–5)        Closed-loop RUN
─────────────────────────────────── │ ────────────────────────────────────
                                     │
Motor driven by imposed stator field │ Motor driven by estimated rotor angle
  - No angle feedback                │   - EKF tracks angle from BEMF
  - Phases 1-4: high current,        │   - Speed PID controls torque
    ramp speed 0→1600 RPM            │   - Arbitrary command from ROS2
  - Phase 5: hold at SWITCH_OVER     │
    threshold (EKF validates lock)   │
                                     │
SWITCH_OVER fires when:              │
  EKF reports speed ≥ OBS_MINIMUM    │
  for NB_CONSECUTIVE_TESTS ms        │
```

### What changes with EKF vs. Luenberger

```
Luenberger + PLL:
  Fixed gains → good SNR required → must keep SNR ≥ 1 → Phase 5 at 4A → can't hold RPM
                                                                            under load

EKF:
  Optimal gains adapt to noise → works at SNR ≈ 0.3–0.5 → Phase 5 at 8A is fine →
  motor holds RPM → reliable SWITCH_OVER → same firmware works for any drivetrain
```

### Cross-vehicle matrix (target state)

| Vehicle | Drivetrain | Startup | SWITCH_OVER | RUN |
|---------|-----------|---------|-------------|-----|
| HOSIM | heavy, high friction | iter3 profile | EKF at 600 RPM | ✓ |
| AMORIL | lighter, lower friction | same profile | EKF at 600 RPM | ✓ |
| Future car | unknown | same profile | EKF at 600 RPM | ✓ |

The per-vehicle 0xCC config frames (max_iq, revup_rpm, boost_iq) remain available for
fine-tuning but should no longer be necessary for reliable SWITCH_OVER.

---

## 6. File Map

```
08_stm_ws_2601/
├── docs/
│   ├── 20260325_HOSIM_Firmware_Changes.md   ← iter history
│   └── 20260327_MCSDK_EKF_Plan.md           ← this file
├── motor_test1_20260227/
│   ├── Inc/
│   │   ├── drive_parameters.h               ← USE_EKF_OBSERVER flag here (Step 4)
│   │   ├── esc_ekf_observer.h               ← new (Step 4)
│   │   ├── esc_comm.h
│   │   └── pmsm_motor_parameters.h          ← Rs, Ls, Ψf source
│   └── Src/
│       ├── esc_ekf_observer.c               ← new (Step 4)
│       ├── mc_app_hooks.c                   ← EKF hook point (Step 5)
│       └── esc_comm.c
└── tests/
    ├── ekf_pmsm.py                          ← Python prototype (Step 2)
    └── esc_ekf_test.c                       ← C unit tests (Step 4)
```

---

## 7. Reference Equations — Full 4×4 Jacobian

State: `x = [iα, iβ, eα, eβ]`, input: `u = [Vα, Vβ]`, ω = √(eα²+eβ²)/Ψf

```
F = ∂f/∂x =

[ 1-Ts·Rs/Ls     0        -Ts/Ls       0      ]
[     0      1-Ts·Rs/Ls    0         -Ts/Ls   ]
[     0          0      1-Ts·ω·∂ω/∂eα·eβ   -Ts·ω + ... ]  ← full terms below
[     0          0         ...             ...  ]
```

BEMF row detail (using ω = (eα²+eβ²)^(1/2) / Ψf):

```
∂eα_next/∂eα = 1 - Ts·(∂ω/∂eα)·eβ = 1 - Ts·eα·eβ / (Ψf²·ω)
∂eα_next/∂eβ = -Ts·ω - Ts·(∂ω/∂eβ)·eβ = -Ts·ω - Ts·eβ² / (Ψf²·ω)

∂eβ_next/∂eα = Ts·ω + Ts·(∂ω/∂eα)·eα = Ts·ω + Ts·eα² / (Ψf²·ω)
∂eβ_next/∂eβ = 1 + Ts·(∂ω/∂eβ)·eα = 1 + Ts·eα·eβ / (Ψf²·ω)
```

Measurement Jacobian H (we measure iα, iβ):
```
H = [ 1  0  0  0 ]
    [ 0  1  0  0 ]
```

---

## 8. Key References

1. **Kim & Sul (2011)** — "High Performance PMSM Drives Without Rotational Position Sensors
   Using Reduced Order Extended Luenberger Observers" — foundational BEMF-state observer
2. **Piippo et al. (2008)** — "Adaptation of Motor Parameters in Sensorless PMSM Drives"
   — EKF with online Rs adaptation (future extension)
3. **Bolognani et al. (2001)** — "Design and Implementation of Model Predictive Control for
   Electric Motor Drive" — SPM EKF at low speed, SNR analysis
4. **Chen et al. (1998)** — "An Extended Electromotive Force Model for Sensorless Control
   of Interior PM Synchronous Motors" — BEMF state formulation (eα, eβ state vector)
5. **ST MCSDK documentation** — `STO_PLL_Handle_t`, `SpeednPosFdbk_Handle_t` interfaces
   in `SpeednPosFdbk.h`, `STO_PMSM_Motor_Parameters.h`

---

*Document maintained on `MCSDK_EKF` branch. Update Section 6 Stage results as hardware
tests are completed.*
