/**
 * @file    custom_foc.c
 * @brief   Custom FOC motor control — replaces MCSDK motor control stack.
 *
 * Step 1: Bare ISR — read ADC phase currents, apply Clarke transform,
 *         output zero voltage (50% duty on all phases).
 *         Motor does not move. Validates ISR timing and ADC readings.
 */

#include "custom_foc.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_hal.h"
#include <math.h>

/* ── ADC handles (declared in main.c) ───────────────────────────────────── */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

/* ── Private state ───────────────────────────────────────────────────────── */
static volatile CFOC_State_t cfoc_state = CFOC_IDLE;

/* ADC offset calibration (zero-current mid-point) */
static int32_t adc_offset_a = 2048;  /* ADC1 injected rank 1 (Phase A) */
static int32_t adc_offset_b = 2048;  /* ADC1 injected rank 2 (Phase B) */

/* Measured currents in Amps (α-β frame), written by HF ISR */
static volatile float isr_Ialpha = 0.0f;
static volatile float isr_Ibeta  = 0.0f;

/* ISR cycle counter for diagnostics */
static volatile uint32_t isr_count = 0U;

/* 1/sqrt(3) for Clarke transform */
#define INV_SQRT3  0.57735026919f

/* ── ADC offset calibration ──────────────────────────────────────────────── */

/**
 * Calibrate ADC offsets with PWM at 50% (zero current).
 * Must be called AFTER TIM1 PWM is running but BEFORE motor moves.
 * Reads CFOC_CALIB_SAMPLES injected conversions and averages.
 */
static void CFOC_CalibrateOffsets(void)
{
  int32_t sum_a = 0;
  int32_t sum_b = 0;

  /* Wait for a few PWM cycles to stabilize */
  HAL_Delay(10);

  for (uint32_t i = 0; i < CFOC_CALIB_SAMPLES; i++)
  {
    /* Wait for injected conversion complete (JEOS on ADC1) */
    while (!LL_ADC_IsActiveFlag_JEOS(ADC1)) { /* spin */ }
    LL_ADC_ClearFlag_JEOS(ADC1);

    /* Read injected data: 12-bit left-aligned, shift right by 4 to get 0..4095 */
    sum_a += (int32_t)(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1));
    sum_b += (int32_t)(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2));
  }

  adc_offset_a = sum_a / (int32_t)CFOC_CALIB_SAMPLES;
  adc_offset_b = sum_b / (int32_t)CFOC_CALIB_SAMPLES;
}

/* ── TIM1 PWM startup ───────────────────────────────────────────────────── */

/**
 * Start TIM1 PWM outputs and ADC injected triggers.
 * Uses TIM2 trigger for synchronized start (same as MCSDK startTimers).
 */
static void CFOC_StartPWM(void)
{
  /* Set all duty cycles to 50% (zero voltage in center-aligned mode) */
  LL_TIM_OC_SetCompareCH1(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH2(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH3(TIM1, CFOC_PWM_HALF_PERIOD / 2U);

  /* CC4 triggers ADC injected conversions — set near top of count */
  LL_TIM_OC_SetCompareCH4(TIM1, CFOC_PWM_HALF_PERIOD - 1U);

  /* Enable TIM1 outputs (MOE bit) */
  LL_TIM_EnableAllOutputs(TIM1);

  /* Enable TIM1 update interrupt (for R3_2 sampling point reconfiguration
   * — we keep this even though we don't use R3_2; the TIM1 UP ISR just
   * clears the flag in Step 1) */
  LL_TIM_EnableIT_UPDATE(TIM1);

  /* Start ADC injected conversions (triggered by TIM1 CC4) */
  LL_ADC_INJ_StartConversion(ADC1);
  LL_ADC_INJ_StartConversion(ADC2);

  /* Enable JEOS interrupt on ADC2 — this fires the HF FOC ISR at 25 kHz */
  LL_ADC_EnableIT_JEOS(ADC2);

  /* Synchronized start via TIM2 trigger (same technique as MCSDK) */
  LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_ITR1);
  LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_TRIGGER);

  /* Enable TIM2 clock temporarily, fire update to trigger TIM1 */
  uint32_t tim2_on = LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM2);
  if (!tim2_on)
  {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  }
  LL_TIM_GenerateEvent_UPDATE(TIM2);
  if (!tim2_on)
  {
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM2);
  }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void CFOC_Init(void)
{
  /* Start OPAMPs (current sense amplifiers) */
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);

  /* Calibrate ADCs */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  /* Start PWM with 50% duty (zero voltage) */
  CFOC_StartPWM();

  /* Calibrate ADC offsets (zero current) */
  CFOC_CalibrateOffsets();

  cfoc_state = CFOC_IDLE;
}

void CFOC_HighFrequencyTask(void)
{
  /* ── 1. Read phase currents from injected ADC ────────────────────────── */
  /* ADC1 injected rank 1 = Phase A (OPAMP1), rank 2 = Phase B (OPAMP2)
   * Data is 12-bit left-aligned in JDR, use Data12 accessor for 0..4095 */
  int32_t raw_a = (int32_t)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
  int32_t raw_b = (int32_t)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);

  /* Convert to Amps: I = (offset - raw) × scale
   * Sign convention: positive current = current flowing into motor */
  float Ia = (float)(adc_offset_a - raw_a) * CFOC_ADC_TO_AMPS;
  float Ib = (float)(adc_offset_b - raw_b) * CFOC_ADC_TO_AMPS;

  /* ── 2. Clarke transform: (Ia, Ib) → (Iα, Iβ) ──────────────────────── */
  /* Iα = Ia
   * Iβ = (Ia + 2·Ib) / √3  */
  float Ialpha = Ia;
  float Ibeta  = (Ia + 2.0f * Ib) * INV_SQRT3;

  /* Store for telemetry / debug */
  isr_Ialpha = Ialpha;
  isr_Ibeta  = Ibeta;

  /* ── 3. Step 1: Output zero voltage (50% duty) ──────────────────────── */
  /* In center-aligned mode, 50% of ARR = zero average voltage.
   * All three phases at same duty → no current flows. */
  LL_TIM_OC_SetCompareCH1(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH2(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH3(TIM1, CFOC_PWM_HALF_PERIOD / 2U);

  isr_count++;
}

void CFOC_MediumFrequencyTask(void)
{
  /* Step 1: nothing to do at 1 kHz yet.
   * Future steps add: open-loop ramp, EKF convergence check, speed PI. */
}

CFOC_State_t CFOC_GetState(void)
{
  return cfoc_state;
}

void CFOC_Start(int8_t direction)
{
  (void)direction;
  /* Step 1: no-op. Motor start implemented in Step 2. */
}

void CFOC_Stop(void)
{
  /* Step 1: force 50% duty (zero voltage) */
  LL_TIM_OC_SetCompareCH1(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH2(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH3(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  cfoc_state = CFOC_IDLE;
}

void CFOC_GetCurrents(float *ia, float *ib)
{
  if (ia) *ia = isr_Ialpha;
  if (ib) *ib = isr_Ibeta;
}

float CFOC_GetAngle(void)
{
  return 0.0f;  /* Step 1: no angle estimation */
}

float CFOC_GetSpeedRPM(void)
{
  return 0.0f;  /* Step 1: no speed estimation */
}
