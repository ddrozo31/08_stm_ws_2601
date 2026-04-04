/**
 * @file    stm32_mc_common_it.c
 * @brief   Interrupt handlers for motor control (custom FOC version).
 *
 * Replaces the MCSDK-generated version. Handles:
 * - USART2: ESC comm RX (BUILD_ESC) or unused
 * - HardFault: disable TIM1 outputs, hang
 * - SysTick: HAL tick + custom FOC 1 kHz MF task
 * - EXTI15_10: user button (unused in custom FOC)
 */

#include "main.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_hal.h"
#include "custom_foc.h"

#ifdef BUILD_ESC
#include "esc_comm.h"
#endif

/* ── USART2 IRQ ──────────────────────────────────────────────────────────── */

void USART2_IRQHandler(void)
{
#ifdef BUILD_ESC
  /* ESC build: process one RXNE byte for the ESC command frame parser.
   * Clear all error/status flags to prevent stale-flag re-entry. */
  ESC_COMM_UART_RxISR();
  WRITE_REG(USART2->ICR, USART_ICR_TCCF | USART_ICR_FECF | USART_ICR_ORECF
                        | USART_ICR_NECF | USART_ICR_IDLECF);
#endif
}

/* ── HardFault ───────────────────────────────────────────────────────────── */

void HardFault_Handler(void)
{
  LL_TIM_DisableAllOutputs(TIM1);
  while (1) { /* hang */ }
}

/* ── SysTick (1 kHz) ────────────────────────────────────────────────────── */

void SysTick_Handler(void)
{
  HAL_IncTick();
  CFOC_MediumFrequencyTask();
}

/* ── User button (PC10) ──────────────────────────────────────────────────── */

void EXTI15_10_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    /* Button press — unused in custom FOC Step 1 */
  }
}
