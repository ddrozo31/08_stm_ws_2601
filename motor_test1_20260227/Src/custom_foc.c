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
static volatile int32_t adc_offset_a = 2048;
static volatile int32_t adc_offset_b = 2048;

/* Self-calibration accumulators (run inside ISR for first N samples) */
static volatile uint32_t calib_count = 0U;
static volatile int32_t  calib_sum_a = 0;
static volatile int32_t  calib_sum_b = 0;
static volatile uint8_t  calib_done  = 0U;

/* Measured currents in Amps (α-β frame), written by HF ISR */
static volatile float isr_Ialpha = 0.0f;
static volatile float isr_Ibeta  = 0.0f;

/* ISR cycle counter for diagnostics */
static volatile uint32_t isr_count = 0U;


/* 1/sqrt(3) for Clarke transform */
#define INV_SQRT3  0.57735026919f

/* ── Public API ──────────────────────────────────────────────────────────── */

void CFOC_Init(void)
{
  /* Start OPAMPs (current sense amplifiers) */
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);

  /* Calibrate ADCs (internal offset calibration, leaves ADC disabled) */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  /* ── Enable ADCs with LL (bypass HAL state machine) ────────────────── */
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) { /* wait */ }
  LL_ADC_ClearFlag_ADRDY(ADC1);

  LL_ADC_Enable(ADC2);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC2)) { /* wait */ }
  LL_ADC_ClearFlag_ADRDY(ADC2);

  /* ── Configure injected sequence + trigger via LL ──────────────────── */
  /* ADC1: 2 ranks — ch3 (Phase A via OPAMP1), ch12 (Phase B via OPAMP2) */
  LL_ADC_INJ_ConfigQueueContext(ADC1,
      LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
      LL_ADC_INJ_TRIG_EXT_RISING,
      LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
      LL_ADC_CHANNEL_3,
      LL_ADC_CHANNEL_12,
      LL_ADC_CHANNEL_0, LL_ADC_CHANNEL_0);

  /* ADC2: 2 ranks — VOPAMP3 (Phase C), ch3 */
  LL_ADC_INJ_ConfigQueueContext(ADC2,
      LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
      LL_ADC_INJ_TRIG_EXT_RISING,
      LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
      LL_ADC_CHANNEL_VOPAMP3_ADC2,
      LL_ADC_CHANNEL_3,
      LL_ADC_CHANNEL_0, LL_ADC_CHANNEL_0);

  /* Arm both ADCs for external trigger (JADSTART) */
  LL_ADC_INJ_StartConversion(ADC1);
  LL_ADC_INJ_StartConversion(ADC2);

  /* ── TIM1 setup ────────────────────────────────────────────────────── */
  /* Set all duty cycles to 50% (zero voltage in center-aligned mode) */
  LL_TIM_OC_SetCompareCH1(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH2(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH3(TIM1, CFOC_PWM_HALF_PERIOD / 2U);

  /* CC4 triggers ADC injected conversions — set near top of count */
  LL_TIM_OC_SetCompareCH4(TIM1, CFOC_PWM_HALF_PERIOD - 1U);

  /* Enable TIM1 outputs (MOE bit) */
  LL_TIM_EnableAllOutputs(TIM1);

  /* Enable TIM1 update interrupt */
  LL_TIM_EnableIT_UPDATE(TIM1);

  /* Enable JEOS interrupt on ADC2 — the 25 kHz FOC ISR.
   * Self-calibration happens inside the ISR for the first N samples. */
  LL_ADC_EnableIT_JEOS(ADC2);

  /* Start TIM1 counter — CC4 match triggers ADC injected conversions */
  LL_TIM_EnableCounter(TIM1);

  cfoc_state = CFOC_IDLE;
}

void CFOC_HighFrequencyTask(void)
{
  /* ── 1. Read phase currents from injected ADC ────────────────────────── */
  /* ADC is left-aligned (12-bit << 4); shift right to get 0..4095 */
  int32_t raw_a = (int32_t)(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) >> 4);
  int32_t raw_b = (int32_t)(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) >> 4);

  /* ── Self-calibration: accumulate first N samples at zero current ───── */
  if (!calib_done)
  {
    calib_sum_a += raw_a;
    calib_sum_b += raw_b;
    calib_count++;

    if (calib_count >= CFOC_CALIB_SAMPLES)
    {
      adc_offset_a = calib_sum_a / (int32_t)CFOC_CALIB_SAMPLES;
      adc_offset_b = calib_sum_b / (int32_t)CFOC_CALIB_SAMPLES;
      calib_done = 1U;
    }

    /* During calibration, keep 50% duty and skip current calculation */
    isr_count++;
    return;
  }

  /* ── 2. Convert to Amps ─────────────────────────────────────────────── */
  float Ia = (float)(adc_offset_a - raw_a) * CFOC_ADC_TO_AMPS;
  float Ib = (float)(adc_offset_b - raw_b) * CFOC_ADC_TO_AMPS;

  /* ── 3. Clarke transform: (Ia, Ib) → (Iα, Iβ) ──────────────────────── */
  float Ialpha = Ia;
  float Ibeta  = (Ia + 2.0f * Ib) * INV_SQRT3;

  /* Store for telemetry / debug */
  isr_Ialpha = Ialpha;
  isr_Ibeta  = Ibeta;

  /* ── 4. Step 1: Output zero voltage (50% duty) ──────────────────────── */
  LL_TIM_OC_SetCompareCH1(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH2(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH3(TIM1, CFOC_PWM_HALF_PERIOD / 2U);

  isr_count++;
}

void CFOC_MediumFrequencyTask(void)
{
  /* Step 1: nothing to do at 1 kHz yet. */
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
