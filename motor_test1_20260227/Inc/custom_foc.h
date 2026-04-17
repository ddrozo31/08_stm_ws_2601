/**
 * @file    custom_foc.h
 * @brief   Custom FOC motor control — replaces MCSDK motor control stack.
 *
 * Provides sensorless FOC with EKF observer for B-G431B-ESC1 board.
 * Runs at 25 kHz (HF task in ADC ISR) + 1 kHz (MF task in SysTick).
 *
 * Step 3: EKF crossfade — open-loop → closed-loop sensorless.
 */
#ifndef CUSTOM_FOC_H
#define CUSTOM_FOC_H

#include <stdint.h>
#include <stdbool.h>

/* ── Motor state machine ─────────────────────────────────────────────────── */
typedef enum {
  CFOC_IDLE = 0,
  CFOC_ALIGNMENT,
  CFOC_OPEN_LOOP,
  CFOC_CROSSFADE,
  CFOC_CLOSED_LOOP,
  CFOC_FAULT
} CFOC_State_t;

/* ── Hardware parameters (B-G431B-ESC1) ──────────────────────────────────── */

/* TIM1: 170 MHz clock, center-aligned, 25 kHz PWM */
#define CFOC_TIM_CLK_HZ         170000000U
#define CFOC_PWM_FREQ_HZ        25000U
#define CFOC_PWM_PERIOD         (CFOC_TIM_CLK_HZ / CFOC_PWM_FREQ_HZ)  /* 6800 */
#define CFOC_PWM_HALF_PERIOD    (CFOC_PWM_PERIOD / 2U)                 /* 3400 (ARR) */
#define CFOC_TS                 (1.0f / (float)CFOC_PWM_FREQ_HZ)       /* 40 µs */
#define CFOC_MF_TS              0.001f  /* 1 kHz medium-frequency period [s] */

/* ADC current sensing: 3-shunt via OPAMP1/2/3
 * Rshunt = 0.003 Ω, gain = 9.14 (OPAMP config on B-G431B-ESC1)
 * Per count: 3.3/(4096 × 0.003 × 9.14) = 0.02938 A/count */
#define CFOC_RSHUNT             0.003f
#define CFOC_AMP_GAIN           9.14f
#define CFOC_VREF               3.3f
#define CFOC_ADC_TO_AMPS        (CFOC_VREF / (4096.0f * CFOC_RSHUNT * CFOC_AMP_GAIN))

/* Vbus sensing: voltage divider ratio */
#define CFOC_VBUS_RATIO         0.09625565501973242f

/* Motor parameters (AMORIL — from pmsm_motor_parameters.h) */
#define CFOC_RS                 0.1f      /* Stator resistance [Ω] */
#define CFOC_LS                 10.0e-6f  /* Stator inductance [H] */
#define CFOC_POLE_PAIRS         2U
#define CFOC_PSI_F              9.75e-4f  /* PM flux linkage [Wb] */

/* ── Dead-time compensation (for EKF voltage feed) ──────────────────────── */
/* HW_DEAD_TIME_NS=800 is total dead time; TIM1 DTG register is set to
 * DEAD_TIME_COUNTS/2, so per-edge dead time = 400 ns.
 * Vdt = Vbus × 2×Tdt_edge / Tpwm = 12 × 2×400ns / 40µs = 0.24V
 * BEMF at 1600 RPM = 0.33V → Vdt is 73% of BEMF (significant). */
#define CFOC_DEAD_TIME_NS       800U
/* Vdt = Vbus × 2×Tdt_edge / Tpwm. The per-Vbus factor is constant;
 * actual Vdt is computed at runtime using measured Vbus (updated every 100ms). */
#define CFOC_VDT_PER_VBUS       ((float)CFOC_DEAD_TIME_NS * 1e-9f / CFOC_TS)

/* ── Calibration ─────────────────────────────────────────────────────────── */
#define CFOC_CALIB_SAMPLES      64U      /* ADC samples for bootstrap offset */
#define CFOC_OFFSET_EMA_ALPHA   0.002f   /* EMA smoothing for continuous offset
                                          * tracking in IDLE. τ ≈ 1/α/f_pwm
                                          * = 1/0.002/25000 = 20 ms */

/* ── Alignment parameters ────────────────────────────────────────────────── */
#define CFOC_ALIGN_MS           300U      /* Alignment duration [ms] */
#define CFOC_ALIGN_ID           3.0f      /* d-axis alignment current [A] */

/* ── Open-loop startup parameters ────────────────────────────────────────── */
#define CFOC_OL_RAMP_MS         3000U     /* Speed ramp duration [ms] */
#define CFOC_OL_TARGET_RPM      1400.0f   /* Open-loop target / EKF handoff speed [RPM].
                                           * At 1400 RPM: BEMF = 0.286 V, Vdt ≈ 0.24 V →
                                           * Vdt/BEMF ≈ 84% (16% margin). Session 3 baseline.
                                           * 1200 RPM (Session 4) was the physics floor and was
                                           * abandoned — Vdt/BEMF=98% left no margin. 1600 RPM
                                           * (Sessions 1–2) also worked but startup was 0.4s slower.
                                           * Known issue: ~5% rough CL transitions on ground under
                                           * load — observer-side limitation, deferred to Step 8
                                           * (adaptive-R EKF rebuild). */
#define CFOC_OL_IQ_RAMP_MS      500U      /* Current ramp duration [ms] */
#define CFOC_OL_IQ_TARGET       5.0f      /* Open-loop Iq target [A] */
#define CFOC_OL_ID_REF          0.0f      /* d-axis current reference (SPMSM → 0) */
#define CFOC_OL_MAX_MS          8000U     /* Safety timeout: max open-loop duration [ms] */

/* ── EKF crossfade parameters ────────────────────────────────────────────── */
/*
 * EKF runs at 25 kHz in HF task. Crossfade from OL→EKF angle when
 * BEMF magnitude is large enough (motor spinning fast enough for
 * reliable angle estimate).
 *
 * BEMF² threshold: eα²+eβ² > CFOC_XF_BEMF_SQ_THRESH
 *   At 1400 RPM: ω_e = 1400×2π/60×2 = 293 rad/s
 *   BEMF = Ψf × ω_e = 9.75e-4 × 293 = 0.286 V
 *   BEMF² = 0.082. Threshold at 0.05 ≈ 1100 RPM equivalent — comfortably below target.
 * Note: crossfade is time-triggered (ramp_ms completion + dwell), not BEMF-only.
 * The BEMF threshold is an additional guard — ramp completion is the primary trigger.
 */
#define CFOC_XF_BEMF_SQ_THRESH   0.05f     /* BEMF² trigger for crossfade [V²] */
#define CFOC_XF_DWELL_MS          200U      /* BEMF must exceed threshold for this long */
#define CFOC_XF_DURATION_MS       500U      /* Crossfade blend duration [ms] (slow for safety) */

/* EKF speed filter — two independent LPFs from the same raw EKF speed:
 *
 * 1. ANGLE LPF (τ=20ms): drives commutation angle integration.
 *    Filters 53 Hz electrical noise by ~85%. Fast enough to track
 *    real speed ramps without lag. Proven stable — do not change.
 *    α = Ts / (τ + Ts) = 40µs / 20.04ms ≈ 0.002
 *
 * 2. PI LPF (τ=100ms): feeds the speed PI controller only.
 *    Filters 53 Hz noise by ~97%, leaving <50 RPM noise on PI input.
 *    Slower than angle LPF — the PI loop bandwidth (~3 Hz) is fine
 *    for RC car control; commands slew at 0.05 u/tick at 20 Hz.
 *    α = Ts / (τ + Ts) = 40µs / 100.04ms ≈ 0.0004 */
#define CFOC_EKF_SPEED_LPF_ALPHA    0.002f   /* τ ≈  20 ms — angle integration */
#define CFOC_EKF_SPD_PI_LPF_ALPHA   0.0004f  /* τ ≈ 100 ms — speed PI feedback */

/* EKF tuning — process/measurement noise.
 * EKF now runs at 25 kHz (was 1 kHz). Process noise scales with Ts
 * (Q_discrete = Q_continuous × Ts), so divide original 1 kHz values by 25.
 * R is per-sample and stays the same. */
#define CFOC_EKF_Q_I              1.33e-4f  /* Current process noise [A²] (3.33e-3/25) */
#define CFOC_EKF_Q_E              1.33e-3f  /* BEMF process noise [V²]   (3.33e-2/25) */
#define CFOC_EKF_R_I              3.33e-3f  /* Measurement noise [A²] (unchanged) */

/* ── Current PI controller parameters ────────────────────────────────────── */
/*
 * Current PI bandwidth ωc ≈ 2π×1000 rad/s (1 kHz crossover):
 *   Kp = Ls × ωc = 10µH × 6283 = 0.063
 *   Ki = Rs × ωc = 0.1  × 6283 = 628.3  → discrete: Ki×Ts = 628.3/25000 = 0.025
 *
 * Vmax ceiling:
 *   Linear SVPWM limit:  Vbus / √3  ≈ 12 / 1.732 = 6.93 V
 *   Six-step (OVM) ceil: Vbus × 2/π ≈ 12 × 0.6366 = 7.64 V  (+15 %)
 *
 * The PI out_max is updated at runtime in CFOC_MediumFrequencyTask to
 * cfoc_vbus_rt × CFOC_PI_VMAX_PER_VBUS so it tracks battery discharge.
 * CFOC_PI_VMAX is the compile-time initialiser (12 V nominal).
 */
#define CFOC_PI_IQ_KP           0.063f
#define CFOC_PI_IQ_KI           0.025f    /* Already discretized (Ki × Ts) */
#define CFOC_PI_ID_KP           0.063f
#define CFOC_PI_ID_KI           0.025f
#define CFOC_PI_VMAX_PER_VBUS   (2.0f / 3.14159265359f)  /* six-step ceil: 2/π ≈ 0.6366 */
#define CFOC_PI_VMAX            (12.0f * CFOC_PI_VMAX_PER_VBUS)  /* nominal init ≈ 7.64 V */

/* ── Speed PI controller parameters (Step 4) ────────────────────────────── */
/*
 * Speed PI runs at 1 kHz (MF task) in CLOSED_LOOP state.
 * Output is Iq_ref [A].
 *
 * Conservative starting gains:
 *   Kp = 0.01 A/RPM — 100 RPM error → 1 A correction
 *   Ki = 0.001 A/(RPM·s) — already discretized (Ki_continuous × MF_Ts)
 *        Ki_continuous = 1.0 A/(RPM·s), Ki_d = 1.0 × 0.001 = 0.001
 *
 * Speed command: set externally via CFOC_SetSpeed() from ESC layer.
 */
#define CFOC_PI_SPD_KP          0.01f     /* Speed Kp [A/RPM] */
#define CFOC_PI_SPD_KI          0.001f    /* Speed Ki [A/RPM], discretized (× MF_Ts) */
#define CFOC_PI_SPD_IQ_MAX      10.0f     /* Max |Iq_ref| from speed PI [A] */

/* ── PI controller type ──────────────────────────────────────────────────── */
typedef struct {
  float Kp;
  float Ki;           /* Pre-multiplied by Ts (discrete integrator gain) */
  float integral;
  float out_min;
  float out_max;
} CFOC_PI_t;

/* ── Debug log buffer (RAM ring, dumped via debugger) ────────────────────── */
#define CFOC_LOG_SIZE  1000U  /* 1000 samples @ 125 Hz = 8.0 s capture
                               * (19 KB; covers OL ramp + crossfade + CL) */

typedef struct __attribute__((packed)) {
  uint16_t tick_ms;      /* ms since CFOC_Start (wraps at 65535) */
  uint8_t  state;        /* CFOC_State_t */
  int16_t  Iq_x100;     /* measured Iq [A] × 100 */
  int16_t  Id_x100;     /* measured Id [A] × 100 */
  int16_t  Vq_x100;     /* PI output Vq [V] × 100 */
  int16_t  Vd_x100;     /* PI output Vd [V] × 100 */
  int16_t  theta_x10;   /* FOC angle (OL or blended) [deg] × 10 */
  int16_t  ekf_theta_x10; /* EKF estimated angle [deg] × 10 */
  int16_t  ekf_rpm;      /* EKF estimated speed [RPM] (signed) */
  int16_t  Iq_ref_x100;  /* Iq reference from speed PI [A] × 100 */
  uint16_t innov_x1000;  /* EKF innovation LPF magnitude [A] × 1000 (stall diag) */
} CFOC_LogEntry_t;       /* 21 bytes per entry, 21 KB total */

extern volatile CFOC_LogEntry_t cfoc_log[CFOC_LOG_SIZE];
extern volatile uint32_t cfoc_log_idx;
extern volatile uint8_t  cfoc_log_running;

/* ── Public API ──────────────────────────────────────────────────────────── */

/** Initialize custom FOC: calibrate ADC offsets, configure TIM1 PWM, start ISRs. */
void CFOC_Init(void);

/** 25 kHz high-frequency task — called from ADC1_2 ISR. */
void CFOC_HighFrequencyTask(void);

/** 1 kHz medium-frequency task — called from SysTick. */
void CFOC_MediumFrequencyTask(void);

/** Get current motor state. */
CFOC_State_t CFOC_GetState(void);

/** LPF magnitude of the EKF current innovation [A]. Stall-detection diag. */
float CFOC_GetInnovMag(void);

/** Lock-confidence κ = |Iq| / max(|ω_e|, ω_min) [A·s/rad]. LPF τ≈100ms.
 *  Diagnostic-only in this build (no gate wired). */
float CFOC_GetLockKappa(void);

/** Lock-confidence voltage-balance residual |Vq - (Rs·Iq + Ψf·ω_e)| [V]. LPF τ≈100ms. */
float CFOC_GetLockResidual(void);

/** Command motor start (from ESC layer). direction: +1 or -1. */
void CFOC_Start(int8_t direction);

/** Command motor stop. */
void CFOC_Stop(void);

/** Emergency stop → CFOC_FAULT state (called from TIM1 BRK ISR). */
void CFOC_FaultStop(void);

/** Acknowledge fault: CFOC_FAULT → CFOC_IDLE. No-op if not in FAULT. */
void CFOC_AckFault(void);

/** Returns 1 if motor is active (ALIGNMENT through CLOSED_LOOP). */
uint8_t CFOC_IsRunning(void);

/** Set torque (Iq) reference directly, bypassing speed PI.
 *  Activates torque mode — call every MF tick while in CLOSED_LOOP.
 *  @param iq_ref  q-axis current [A], positive = forward torque. */
void CFOC_SetTorque(float iq_ref);

/** Set speed reference for speed PI mode.
 *  @param rpm  target speed [RPM], signed (positive=forward). */
void CFOC_SetSpeed(float rpm);

/** Set the speed PI Iq output clamp at runtime.
 *  @param iq_max  max |Iq_ref| [A]. Takes effect on next CFOC_Start(). */
void CFOC_SetIqLimit(float iq_max);

/** Set startup parameters at runtime (0 or negative = keep current value).
 *  All take effect on next CFOC_Start(). */
void CFOC_SetStartupParams(float ol_iq_a, float ol_ramp_ms,
                            float align_ms, float align_id_a);

/** Set crossfade parameters at runtime (0 or negative = keep current value).
 *  @param xf_dur_ms   Crossfade blend duration [ms].
 *  @param xf_dwell_ms BEMF dwell time before crossfade [ms].
 *  @param ol_rpm      Open-loop target / crossfade speed [RPM]. */
void CFOC_SetCrossfadeParams(float xf_dur_ms, float xf_dwell_ms, float ol_rpm);

/** Set speed PI tuning at runtime (0 or negative = keep current value).
 *  @param kp        Speed PI Kp [A/RPM].
 *  @param ki        Speed PI Ki [A/RPM], already discretized (× MF_Ts).
 *  @param lpf_alpha Speed PI feedback LPF alpha (EKF speed filter). */
void CFOC_SetSpeedPIParams(float kp, float ki, float lpf_alpha);

/** Set Step 8 adaptive-R EKF params at runtime.
 *  Pass 0xFF for mode / vf_lock to keep the current value; pass <= 0 for
 *  floats to keep them. Values take effect on the next CFOC_Start() cycle
 *  (EKF_Init uses them when transitioning ALIGNMENT → OPEN_LOOP).
 *
 *  @param mode          0 = discrete (legacy), 1 = adaptive-R, 0xFF = keep
 *  @param omega_thresh  EKF adaptive-R knee [elec rad/s]
 *  @param R0            Healthy-speed R [A²]
 *  @param Qe            BEMF process noise variance
 *  @param vf_lock       0 = off, 1 = force-seed EKF BEMF from V/f, 0xFF = keep
 *  No-op build when USE_ADAPTIVE_R_EKF=0 (function body elided). */
void CFOC_SetAdaptiveREkfParams(uint8_t mode, float omega_thresh,
                                float R0, float Qe, uint8_t vf_lock);

/** Step 8 telemetry accessors. Return 0 in non-adaptive builds. */
uint8_t CFOC_GetObserverMode(void);  /*!< current observer mode (0=discrete, 1=adaptive) */
float   CFOC_GetEkfR(void);          /*!< latest pushed R value [A²]   */
float   CFOC_GetEkfBemfMag(void);    /*!< latest |e| = √(eα²+eβ²) [V] */

/** Step 8 CL-gate diagnostic: 1 if θ-swap guard blocked, else 0. */
uint8_t CFOC_GetSwapDeferred(void);

/** Step 8 CL-gate diagnostic: consecutive ms that |e|>threshold held. */
uint32_t CFOC_GetClHysteresisMs(void);

/** Get measured d-q currents from most recent HF cycle.
 *  @param[out] iq  q-axis current [A].
 *  @param[out] id  d-axis current [A]. */
void CFOC_GetIqd(float *iq, float *id);

/** Get measured phase currents in Amps (α-β frame). */
void CFOC_GetCurrents(float *ia, float *ib);

/** Get estimated electrical angle [rad]. */
float CFOC_GetAngle(void);

/** Get estimated speed [RPM]. */
float CFOC_GetSpeedRPM(void);

/** Read DC bus voltage via ADC1 regular conversion.
 *  Single-shot, ~2µs blocking. Call at low rate (e.g. 10 Hz). */
float CFOC_GetVbusV(void);

#endif /* CUSTOM_FOC_H */
