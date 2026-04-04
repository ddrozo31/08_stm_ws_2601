/**
 * @file    custom_foc.h
 * @brief   Custom FOC motor control — replaces MCSDK motor control stack.
 *
 * Provides sensorless FOC with EKF observer for B-G431B-ESC1 board.
 * Runs at 25 kHz (HF task in ADC ISR) + 1 kHz (MF task in SysTick).
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

/* ADC current sensing: 3-shunt via OPAMP1/2/3
 * Rshunt = 0.003 Ω, gain = 9.14 (OPAMP config on B-G431B-ESC1)
 * ADC 12-bit left-aligned → raw is 0..65535, but JDR range is 0..4095
 * Current = (offset - raw) / (4096 × Rshunt × Gain / Vref)
 * Scale: Vref/(Rshunt×Gain) = 3.3/(0.003×9.14) = 120.35 A full-scale
 * Per count: 120.35/4096 = 0.02938 A/count */
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
#define CFOC_CALIB_SAMPLES      64U  /* ADC samples for offset calibration */

/* ── Public API ──────────────────────────────────────────────────────────── */

/** Initialize custom FOC: calibrate ADC offsets, configure TIM1 PWM, start ISRs. */
void CFOC_Init(void);

/** 25 kHz high-frequency task — called from ADC1_2 ISR. */
void CFOC_HighFrequencyTask(void);

/** 1 kHz medium-frequency task — called from SysTick. */
void CFOC_MediumFrequencyTask(void);

/** Get current motor state. */
CFOC_State_t CFOC_GetState(void);

/** Command motor start (from ESC layer). */
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
