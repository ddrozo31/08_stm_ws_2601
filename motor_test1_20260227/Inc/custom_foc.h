/**
 * @file    custom_foc.h
 * @brief   Custom FOC motor control — replaces MCSDK motor control stack.
 *
 * Provides sensorless FOC with EKF observer for B-G431B-ESC1 board.
 * Runs at 25 kHz (HF task in ADC ISR) + 1 kHz (MF task in SysTick).
 *
 * Step 2: Open-loop startup with PI current control + SVM.
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

/* ── PI controller parameters ────────────────────────────────────────────── */
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

/* ── PI controller type ──────────────────────────────────────────────────── */
typedef struct {
  float Kp;
  float Ki;           /* Pre-multiplied by Ts (discrete integrator gain) */
  float integral;
  float out_min;
  float out_max;
} CFOC_PI_t;

/* ── Debug log buffer (RAM ring, dumped via debugger) ────────────────────── */
#define CFOC_LOG_SIZE  500U  /* 500 samples @ 1 kHz = 500 ms capture */

typedef struct __attribute__((packed)) {
  uint16_t tick_ms;     /* ms since CFOC_Start (wraps at 65535) */
  uint8_t  state;       /* CFOC_State_t */
  int16_t  Iq_x100;    /* measured Iq [A] × 100 */
  int16_t  Id_x100;    /* measured Id [A] × 100 */
  int16_t  Vq_x100;    /* PI output Vq [V] × 100 */
  int16_t  Vd_x100;    /* PI output Vd [V] × 100 */
  int16_t  theta_x10;  /* electrical angle [deg] × 10 */
} CFOC_LogEntry_t;      /* 13 bytes per entry, 6.5 KB total */

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

/** Get measured phase currents in Amps (α-β frame). */
void CFOC_GetCurrents(float *ia, float *ib);

/** Get estimated electrical angle [rad]. */
float CFOC_GetAngle(void);

/** Get estimated speed [RPM]. */
float CFOC_GetSpeedRPM(void);

#endif /* CUSTOM_FOC_H */
