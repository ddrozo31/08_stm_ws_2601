/**
  ******************************************************************************
  * @file    esc_comm.c
  * @brief   ESC UART communication layer -- frame parser and telemetry sender.
  *
  * Reception: direct RXNE interrupt via USART2_IRQHandler USER CODE section.
  *   HAL_UART_Receive_IT cannot be used because the project's custom
  *   USART2_IRQHandler (stm32_mc_common_it.c) never calls HAL_UART_IRQHandler.
  *   ESC_COMM_Init() enables RXNEIE directly on USART2->CR1.
  *   ESC_COMM_UART_RxISR() is called from the USER CODE BEGIN 0 section of
  *   USART2_IRQHandler and processes one byte per call.
  *
  * Transmission: direct register polling (not HAL_UART_Transmit) to avoid the
  *   HAL lock conflict with concurrent UART operations.
  *   6 bytes at 1843200 baud takes ~33 us.
  ******************************************************************************
  */

#include "esc_comm.h"
#include "main.h"       /* for USART2 peripheral base address and register defs */
#include <string.h>     /* for memcpy */

/* -------------------------------------------------------------------------- */
/* RX state                                                                   */
/* -------------------------------------------------------------------------- */

/* Accumulation buffer for one complete command frame. */
static uint8_t rx_buf[ESC_CMD_FRAME_LEN];

/* Current write index into rx_buf (0 = waiting for SOF). */
static uint8_t rx_idx = 0U;

/* Shared command state -- written only by RXNE ISR, read by application layer.
 * Initialised to 0x7FFF (full-forward / non-neutral) so that the WAIT_NEUTRAL
 * state stays active until the host explicitly sends a neutral frame.         */
static volatile int16_t esc_cmd_value = 0x7FFF;
static volatile uint8_t esc_cmd_fresh = 0U;

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
  * @brief Initialise the ESC communication layer.
  *
  * Enables the USART2 RXNE interrupt directly via the CR1 register.
  * ESC_COMM_UART_RxISR() must be called from USART2_IRQHandler to
  * actually process received bytes.
  */
void ESC_COMM_Init(void)
{
  rx_idx = 0U;

  /* Enable RXNE interrupt directly -- HAL_UART_Receive_IT cannot be used
   * because the project's USART2_IRQHandler does not call HAL_UART_IRQHandler. */
  USART2->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
}

/**
  * @brief Returns the most recent validated command value.
  *
  * Scaled int16_t: -32768 maps to u = -1.0, +32767 maps to u = +1.0.
  */
int16_t ESC_COMM_GetCommand(void)
{
  return esc_cmd_value;
}

/**
  * @brief Returns 1 if a new command frame arrived since the last call.
  *
  * Clears the freshness flag on read -- call once per control cycle.
  */
uint8_t ESC_COMM_HasNewCommand(void)
{
  uint8_t fresh = esc_cmd_fresh;
  esc_cmd_fresh = 0U;
  return fresh;
}

/**
  * @brief Build and transmit one telemetry frame to the RPi5.
  *
  * Uses direct register polling -- not HAL_UART_Transmit -- to avoid any
  * HAL lock interaction with the UART peripheral.
  *
  * @param speed_rpm  Signed average motor speed in RPM.
  * @param state      ESC_State_t cast to uint8_t.
  * @param faults     Lower byte of MC_GetCurrentFaultsMotor1().
  */
void ESC_COMM_SendTelemetry(int16_t speed_rpm, uint8_t state, uint8_t faults,
                            int16_t cmd_raw, uint16_t vbus_v)
{
  uint8_t frame[ESC_TLM_FRAME_LEN];

  /* [0xBB][spd_lo][spd_hi][state][faults][u_lo][u_hi][v_lo][v_hi][chk] */
  frame[0] = ESC_TLM_SOF;
  frame[1] = (uint8_t)( speed_rpm & 0xFF);
  frame[2] = (uint8_t)((speed_rpm >> 8) & 0xFF);
  frame[3] = state;
  frame[4] = faults;
  frame[5] = (uint8_t)( cmd_raw & 0xFF);
  frame[6] = (uint8_t)((cmd_raw >> 8) & 0xFF);
  frame[7] = (uint8_t)( vbus_v & 0xFF);
  frame[8] = (uint8_t)((vbus_v >> 8) & 0xFF);
  frame[9] = frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^
             frame[5] ^ frame[6] ^ frame[7] ^ frame[8];

  for (uint8_t i = 0U; i < ESC_TLM_FRAME_LEN; i++)
  {
    while ((USART2->ISR & USART_ISR_TXE_TXFNF) == 0U) {}
    USART2->TDR = frame[i];
  }
  /* No TC wait: USART2_IRQHandler clears TC on any RXNE, which would stall
   * this loop permanently. TXE-per-byte is sufficient -- the last byte is
   * already in the shift register and will complete on its own. */
}

/* -------------------------------------------------------------------------- */
/* RXNE byte processor (called from USART2_IRQHandler USER CODE section)      */
/* -------------------------------------------------------------------------- */

/**
  * @brief Process one received byte through the command frame parser.
  *
  * Called internally from ESC_COMM_UART_RxISR().
  */
static void ESC_COMM_ProcessByte(uint8_t byte)
{
  if (rx_idx == 0U)
  {
    /* Waiting for start-of-frame. */
    if (byte == ESC_CMD_SOF)
    {
      rx_buf[0] = byte;
      rx_idx    = 1U;
    }
    /* Any other byte: silently discard (self-synchronising). */
  }
  else
  {
    rx_buf[rx_idx] = byte;
    rx_idx++;

    if (rx_idx >= ESC_CMD_FRAME_LEN)
    {
      /* Full frame: validate XOR checksum over payload bytes [1..3]. */
      uint8_t chk = rx_buf[1] ^ rx_buf[2] ^ rx_buf[3];

      if (chk == rx_buf[4])
      {
        int16_t raw;
        (void)memcpy(&raw, &rx_buf[1], sizeof(int16_t));
        esc_cmd_value = raw;
        esc_cmd_fresh = 1U;
      }
      rx_idx = 0U;
    }
  }
}

/**
  * @brief USART2 RXNE handler -- call from USART2_IRQHandler USER CODE BEGIN 0.
  *
  * Reads one byte from USART2->RDR (which clears the RXNE flag) and passes
  * it to the frame parser. Safe to call unconditionally; checks the flag first.
  */
void ESC_COMM_UART_RxISR(void)
{
  if ((USART2->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    uint8_t byte = (uint8_t)(USART2->RDR & 0xFFU);  /* reading RDR clears RXNE */
    ESC_COMM_ProcessByte(byte);
  }
}
