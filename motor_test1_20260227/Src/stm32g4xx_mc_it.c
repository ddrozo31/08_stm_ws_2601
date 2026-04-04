/**
 * @file    stm32g4xx_mc_it.c
 * @brief   Motor control peripheral ISRs (custom FOC version).
 *
 * Replaces the MCSDK-generated version. Handles:
 * - ADC1_2: end-of-injected-sequence → 25 kHz FOC task
 * - TIM1 UP: clear update flag (no R3_2 sampling reconfiguration)
 * - TIM1 BRK: overcurrent/overvoltage → disable PWM, set fault
 */

#include "main.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_tim.h"
#include "custom_foc.h"

/* ── ADC1/ADC2 end-of-injected-sequence (25 kHz FOC ISR) ────────────────── */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
void ADC1_2_IRQHandler(void)
{
  LL_ADC_ClearFlag_JEOS(ADC2);
  CFOC_HighFrequencyTask();
}

/* ── TIM1 Update (PWM period boundary) ──────────────────────────────────── */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
void TIM1_UP_TIM16_IRQHandler(void)
{
  LL_TIM_ClearFlag_UPDATE(TIM1);
  /* Custom FOC: no sampling-point reconfiguration needed in Step 1.
   * Future: sector-dependent ADC window adjustment goes here. */
}

/* ── TIM1 Break (overcurrent / overvoltage) ──────────────────────────────── */

void TIM1_BRK_TIM15_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_BRK(TIM1))
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    CFOC_Stop();  /* Overcurrent: PWM already disabled by hardware */
  }

  if (LL_TIM_IsActiveFlag_BRK2(TIM1))
  {
    LL_TIM_ClearFlag_BRK2(TIM1);
    CFOC_Stop();  /* Overvoltage */
  }
}
