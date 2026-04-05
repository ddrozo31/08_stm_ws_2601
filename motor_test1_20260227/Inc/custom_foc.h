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
#define CFOC_VDT                (12.0f * (float)CFOC_DEAD_TIME_NS * 1e-9f / CFOC_TS)

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
#define CFOC_OL_TARGET_RPM      1600.0f   /* Open-loop target speed [RPM] */
#define CFOC_OL_IQ_RAMP_MS      500U      /* Current ramp duration [ms] */
#define CFOC_OL_IQ_TARGET       5.0f      /* Open-loop Iq target [A] */
#define CFOC_OL_ID_REF          0.0f      /* d-axis current reference (SPMSM → 0) */
#define CFOC_OL_MAX_MS          8000U     /* Safety timeout: max open-loop duration [ms] */

/* ── EKF crossfade parameters ────────────────────────────────────────────── */
/*
 * EKF runs at 1 kHz in MF task. Crossfade from OL→EKF angle when
 * BEMF magnitude is large enough (motor spinning fast enough for
 * reliable angle estimate).
 *
 * BEMF² threshold: eα²+eβ² > CFOC_XF_BEMF_SQ_THRESH
 *   At 1600 RPM: ω_e = 1600×2π/60×2 = 335 rad/s
 *   BEMF = Ψf × ω_e = 9.75e-4 × 335 = 0.327 V
 *   BEMF² = 0.107. Threshold at half = 0.05 (≈1100 RPM equivalent).
 */
#define CFOC_XF_BEMF_SQ_THRESH   0.05f     /* BEMF² trigger for crossfade [V²] */
#define CFOC_XF_DWELL_MS          200U      /* BEMF must exceed threshold for this long */
#define CFOC_XF_DURATION_MS       500U      /* Crossfade blend duration [ms] (slow for safety) */

/* EKF speed filter — smooth noisy speed before angle integration.
 * τ = 20 ms filters the electrical-frequency oscillation (~53 Hz at 1600 RPM,
 * period 18.8 ms) by ~85%. Tracks speed changes on ~100 ms timescale.
 * α = Ts / (τ + Ts) = 40µs / 20.04ms ≈ 0.002 */
#define CFOC_EKF_SPEED_LPF_ALPHA  0.002f    /* τ ≈ 20 ms */

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
 * Vmax ≈ Vbus/√3 ≈ 12V/1.732 ≈ 6.9V (for 3S LiPo ~12V nominal)
 */
#define CFOC_PI_IQ_KP           0.063f
#define CFOC_PI_IQ_KI           0.025f    /* Already discretized (Ki × Ts) */
#define CFOC_PI_ID_KP           0.063f
#define CFOC_PI_ID_KI           0.025f
#define CFOC_PI_VMAX            6.9f      /* Max voltage magnitude [V] */

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
 * Speed command: fixed CFOC_OL_TARGET_RPM for now (1600 RPM).
 * ESC integration (Step 5) will add external speed/torque commands.
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
} CFOC_LogEntry_t;       /* 19 bytes per entry, 19 KB total */

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

#endif /* CUSTOM_FOC_H */
