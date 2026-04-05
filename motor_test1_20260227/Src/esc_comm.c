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

/* Accumulation buffer for one complete frame (cmd or cfg — both 5 bytes). */
static uint8_t rx_buf[ESC_CMD_FRAME_LEN];

/* Index into rx_buf (0 = waiting for SOF) and SOF byte of current frame. */
static uint8_t rx_idx = 0U;
static uint8_t rx_sof = 0U;

/* Shared command state -- written only by RXNE ISR, read by application layer.
 * Initialised to 0x7FFF (full-forward / non-neutral) so that the WAIT_NEUTRAL
 * state stays active until the host explicitly sends a neutral frame.         */
static volatile int16_t esc_cmd_value = 0x7FFF;
static volatile uint8_t esc_cmd_fresh = 0U;

/* Runtime config state -- written by RXNE ISR on valid 0xCC frames.
 * -1.0f means "not configured": mc_app_hooks falls back to compile-time defaults. */
static volatile float esc_cfg_max_iq_a   = -1.0f;
static volatile float esc_cfg_revup_rpm  = -1.0f;
static volatile float esc_cfg_boost_iq_a = -1.0f;
static volatile float esc_cfg_max_spd_rpm = -1.0f;
static volatile float esc_cfg_iq_limit_a  = -1.0f;
static volatile float esc_cfg_ol_iq_a     = -1.0f;
static volatile float esc_cfg_ol_ramp_ms  = -1.0f;
static volatile float esc_cfg_align_ms    = -1.0f;
static volatile float esc_cfg_align_id_a  = -1.0f;

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

  /* Enable the 8-byte RX FIFO before enabling the interrupt.
   *
   * Problem: TIM1_UP (FOC ISR) runs at priority 0 — the highest possible.
   * USART2_IRQn runs at priority 3 and cannot preempt it.  At 1843200 baud
   * one byte takes 5.4 µs; a typical FOC ISR takes ~10 µs.  During that
   * window 2 bytes arrive but only 1 fits in the single-entry RDR, so the
   * second byte triggers ORE and is silently discarded — corrupting the
   * command frame and causing checksum failures.
   *
   * Fix: enable the 8-byte hardware FIFO (FIFOEN).  Now up to 8 bytes
   * (43 µs worth) can accumulate during any ISR preemption without ORE.
   * FIFOEN requires the USART to be disabled (UE=0) during the write. */
  USART2->CR1 &= ~USART_CR1_UE;           /* disable USART to write FIFOEN */
  USART2->CR1 |=  USART_CR1_FIFOEN;       /* enable 8-byte RX/TX FIFO      */
  USART2->CR1 |=  USART_CR1_UE;           /* re-enable USART               */

  /* Enable RXNE/RXFNE interrupt -- fires whenever at least one byte is in
   * the FIFO (threshold-based interrupt RXFTIE is not needed here). */
  USART2->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
}

/* -------------------------------------------------------------------------- */
/* Config getters                                                              */
/* -------------------------------------------------------------------------- */

/** @brief Max torque current (A) set by config frame, or -1 if not configured. */
float ESC_COMM_GetMaxIqA(void)   { return esc_cfg_max_iq_a; }

/** @brief Rev-up target speed (RPM) set by config frame, or -1 if not configured. */
float ESC_COMM_GetRevupRPM(void) { return esc_cfg_revup_rpm; }

/** @brief RUN-entry boost current (A) set by config frame, or -1 if not configured. */
float ESC_COMM_GetBoostIqA(void) { return esc_cfg_boost_iq_a; }

/** @brief Max speed RPM (u=1.0 maps to this), or -1 if not configured. */
float ESC_COMM_GetMaxSpeedRPM(void) { return esc_cfg_max_spd_rpm; }

/** @brief Speed PI Iq clamp (A), or -1 if not configured. */
float ESC_COMM_GetIqLimitA(void) { return esc_cfg_iq_limit_a; }

/** @brief Open-loop Iq target (A), or -1 if not configured. */
float ESC_COMM_GetOlIqA(void) { return esc_cfg_ol_iq_a; }

/** @brief OL speed ramp duration (ms), or -1 if not configured. */
float ESC_COMM_GetOlRampMs(void) { return esc_cfg_ol_ramp_ms; }

/** @brief Alignment duration (ms), or -1 if not configured. */
float ESC_COMM_GetAlignMs(void) { return esc_cfg_align_ms; }

/** @brief Alignment d-axis current (A), or -1 if not configured. */
float ESC_COMM_GetAlignIdA(void) { return esc_cfg_align_id_a; }

/* -------------------------------------------------------------------------- */
/* Command accessors                                                           */
/* -------------------------------------------------------------------------- */

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
                            int16_t cmd_raw, uint16_t vbus_v,
                            int16_t iq_ma, int16_t id_ma, uint8_t mcsdk_st)
{
  uint8_t frame[ESC_TLM_FRAME_LEN];

  /* [0xBB][spd_lo][spd_hi][esc_st][faults][u_lo][u_hi][v_lo][v_hi]
   *       [iq_lo][iq_hi][id_lo][id_hi][mcsdk_st][XOR of bytes 1..13] */
  frame[0]  = ESC_TLM_SOF;
  frame[1]  = (uint8_t)( speed_rpm & 0xFF);
  frame[2]  = (uint8_t)((speed_rpm >> 8) & 0xFF);
  frame[3]  = state;
  frame[4]  = faults;
  frame[5]  = (uint8_t)( cmd_raw & 0xFF);
  frame[6]  = (uint8_t)((cmd_raw >> 8) & 0xFF);
  frame[7]  = (uint8_t)( vbus_v & 0xFF);
  frame[8]  = (uint8_t)((vbus_v >> 8) & 0xFF);
  frame[9]  = (uint8_t)( iq_ma & 0xFF);
  frame[10] = (uint8_t)((iq_ma >> 8) & 0xFF);
  frame[11] = (uint8_t)( id_ma & 0xFF);
  frame[12] = (uint8_t)((id_ma >> 8) & 0xFF);
  frame[13] = mcsdk_st;
  frame[14] = frame[1]  ^ frame[2]  ^ frame[3]  ^ frame[4]  ^
              frame[5]  ^ frame[6]  ^ frame[7]  ^ frame[8]  ^
              frame[9]  ^ frame[10] ^ frame[11] ^ frame[12] ^ frame[13];

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
    /* Waiting for start-of-frame: accept command (0xAA) or config (0xCC). */
    if ((byte == ESC_CMD_SOF) || (byte == ESC_CFG_SOF))
    {
      rx_buf[0] = byte;
      rx_sof    = byte;
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
        if (rx_sof == ESC_CMD_SOF)
        {
          /* Drive command frame: update command value. */
          int16_t raw;
          (void)memcpy(&raw, &rx_buf[1], sizeof(int16_t));
          esc_cmd_value = raw;
          esc_cmd_fresh = 1U;
        }
        else
        {
          /* Config frame: [param_id][val_lo][val_hi] */
          uint8_t param_id = rx_buf[1];
          int16_t val;
          (void)memcpy(&val, &rx_buf[2], sizeof(int16_t));

          if (param_id == ESC_CFG_PARAM_MAX_IQ)
          {
            /* val = int16_t × 0.1 A; valid range 10–150 (1.0–15.0 A) */
            if ((val >= 10) && (val <= 150))
            {
              esc_cfg_max_iq_a = (float)val * 0.1f;
            }
          }
          else if (param_id == ESC_CFG_PARAM_REVUP)
          {
            /* val = int16_t RPM; valid range 1600–5000
             * Lower bound matches OBS_MINIMUM_SPEED_RPM in drive_parameters.h */
            if ((val >= 1600) && (val <= 5000))
            {
              esc_cfg_revup_rpm = (float)val;
            }
          }
          else if (param_id == ESC_CFG_PARAM_BOOST_IQ)
          {
            /* val = int16_t × 0.1 A; valid range 10–150 (1.0–15.0 A) */
            if ((val >= 10) && (val <= 150))
            {
              esc_cfg_boost_iq_a = (float)val * 0.1f;
            }
          }
          else if (param_id == ESC_CFG_PARAM_MAX_SPD)
          {
            /* val = int16_t RPM; valid range 1000–10000 */
            if ((val >= 1000) && (val <= 10000))
            {
              esc_cfg_max_spd_rpm = (float)val;
            }
          }
          else if (param_id == ESC_CFG_PARAM_IQ_LIMIT)
          {
            /* val = int16_t × 0.1 A; valid range 10–200 (1.0–20.0 A) */
            if ((val >= 10) && (val <= 200))
            {
              esc_cfg_iq_limit_a = (float)val * 0.1f;
            }
          }
          else if (param_id == ESC_CFG_PARAM_OL_IQ)
          {
            /* val = int16_t × 0.1 A; valid range 20–150 (2.0–15.0 A) */
            if ((val >= 20) && (val <= 150))
            {
              esc_cfg_ol_iq_a = (float)val * 0.1f;
            }
          }
          else if (param_id == ESC_CFG_PARAM_OL_RAMP)
          {
            /* val = int16_t ms; valid range 1000–8000 */
            if ((val >= 1000) && (val <= 8000))
            {
              esc_cfg_ol_ramp_ms = (float)val;
            }
          }
          else if (param_id == ESC_CFG_PARAM_ALIGN_MS)
          {
            /* val = int16_t ms; valid range 100–2000 */
            if ((val >= 100) && (val <= 2000))
            {
              esc_cfg_align_ms = (float)val;
            }
          }
          else if (param_id == ESC_CFG_PARAM_ALIGN_ID)
          {
            /* val = int16_t × 0.1 A; valid range 10–150 (1.0–15.0 A) */
            if ((val >= 10) && (val <= 150))
            {
              esc_cfg_align_id_a = (float)val * 0.1f;
            }
          }
          /* Unknown param_id: silently ignore. */
        }
      }
      rx_idx = 0U;
      rx_sof = 0U;
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
  /* Drain every byte currently in the FIFO.  With FIFOEN enabled the RXFNE
   * flag stays set until the FIFO is empty, so a single loop handles both
   * the common case (1 byte) and the burst case (several bytes queued during
   * a higher-priority ISR such as the TIM1_UP FOC ISR). */
  while ((USART2->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    uint8_t byte = (uint8_t)(USART2->RDR & 0xFFU);  /* reading RDR pops one byte */
    ESC_COMM_ProcessByte(byte);
  }
}
