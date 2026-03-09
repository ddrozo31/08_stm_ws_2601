/**
  ******************************************************************************
  * @file    esc_comm.h
  * @brief   ESC UART communication layer -- frame parser and telemetry sender.
  *
  * This layer owns USART2 exclusively (ASPEP/MCP is disabled).
  * It knows nothing about motor control -- it only moves bytes.
  *
  * Command frame  (RPi5 -> STM32)  5 bytes:
  *   [0xAA] [cmd_lo] [cmd_hi] [reserved] [checksum]
  *            |_ int16_t u, scaled: -32768 = -1.0 / +32767 = +1.0
  *                                          XOR of bytes [1]..[3]
  *
  * Telemetry frame (STM32 -> RPi5) 6 bytes:
  *   [0xBB] [spd_lo] [spd_hi] [state] [faults] [checksum]
  *           |_ int16_t RPM    ESC state  fault mask  XOR of bytes [1]..[4]
  *
  * ESC state byte values reported in telemetry:
  *   0=BOOT  1=WAIT_NEUTRAL  2=READY  3=FORWARD  4=BRAKE  5=REVERSE  6=FAULT
  *
  * RX architecture note:
  *   The project's USART2_IRQHandler (stm32_mc_common_it.c) is a custom LL-
  *   based handler that does NOT call HAL_UART_IRQHandler. HAL_UART_Receive_IT
  *   cannot be used. Instead, ESC_COMM_UART_RxISR() must be called from the
  *   USER CODE BEGIN USART2_IRQHandler 0 section of that file.
  ******************************************************************************
  */

#ifndef ESC_COMM_H
#define ESC_COMM_H

#include <stdint.h>

/* Frame delimiters */
#define ESC_CMD_SOF        0xAAU
#define ESC_TLM_SOF        0xBBU

/* Frame sizes (bytes) */
#define ESC_CMD_FRAME_LEN  5U
#define ESC_TLM_FRAME_LEN  10U

/* Initialise the layer and enable the USART2 RXNE interrupt. */
void    ESC_COMM_Init(void);

/* Returns the most recently validated command value (int16_t, raw scaled). */
int16_t ESC_COMM_GetCommand(void);

/* Returns 1 if a new command arrived since the last call; clears the flag. */
uint8_t ESC_COMM_HasNewCommand(void);

/* Build and transmit one telemetry frame (blocking, 10 bytes).
 *   speed_rpm : signed average motor speed in RPM
 *   state     : ESC_State_t cast to uint8_t
 *   faults    : lower byte of MC_GetCurrentFaultsMotor1()
 *   cmd_raw   : raw int16 command currently active (-32768..+32767)
 *   vbus_v    : DC bus voltage in whole Volts (from VBS_GetAvBusVoltage_V) */
void    ESC_COMM_SendTelemetry(int16_t speed_rpm, uint8_t state, uint8_t faults,
                               int16_t cmd_raw, uint16_t vbus_v);

/* Call from USART2_IRQHandler USER CODE BEGIN 0 -- processes one RXNE byte. */
void    ESC_COMM_UART_RxISR(void);

#endif /* ESC_COMM_H */
