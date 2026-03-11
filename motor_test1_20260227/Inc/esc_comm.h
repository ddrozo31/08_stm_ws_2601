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
  * Telemetry frame (STM32 -> RPi5) 15 bytes:
  *   [0xBB][spd_lo][spd_hi][esc_st][faults][u_lo][u_hi][v_lo][v_hi]
  *         [iq_lo][iq_hi][id_lo][id_hi][mcsdk_st][XOR_chk]
  *
  *   spd      int16_t  RPM (average mechanical speed)
  *   esc_st   uint8_t  ESC state  0=BOOT 1=WAIT_NEUTRAL 2=READY
  *                                3=FORWARD 4=BRAKE 5=REVERSE 6=FAULT
  *   faults   uint8_t  lower byte of MC_GetCurrentFaultsMotor1()
  *   u        int16_t  raw command (-32768..+32767)
  *   v        uint16_t DC bus voltage in whole Volts
  *   iq       int16_t  actual q-axis current in milliAmps
  *   id       int16_t  actual d-axis current in milliAmps
  *   mcsdk_st uint8_t  MCI_State_t: 0=IDLE 4=START 6=RUN 10=FAULT_NOW 11=FAULT_OVER
  *   XOR_chk          XOR of bytes [1]..[13]
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
#define ESC_TLM_FRAME_LEN  15U

/* Initialise the layer and enable the USART2 RXNE interrupt. */
void    ESC_COMM_Init(void);

/* Returns the most recently validated command value (int16_t, raw scaled). */
int16_t ESC_COMM_GetCommand(void);

/* Returns 1 if a new command arrived since the last call; clears the flag. */
uint8_t ESC_COMM_HasNewCommand(void);

/* Build and transmit one telemetry frame (blocking, 15 bytes).
 *   speed_rpm : signed average motor speed in RPM
 *   state     : ESC_State_t cast to uint8_t
 *   faults    : lower byte of MC_GetCurrentFaultsMotor1()
 *   cmd_raw   : raw int16 command currently active (-32768..+32767)
 *   vbus_v    : DC bus voltage in whole Volts (from VBS_GetAvBusVoltage_V)
 *   iq_ma     : actual q-axis current in milliAmps (from MC_GetIqdMotor1_F)
 *   id_ma     : actual d-axis current in milliAmps (from MC_GetIqdMotor1_F)
 *   mcsdk_st  : MCI_State_t cast to uint8_t */
void    ESC_COMM_SendTelemetry(int16_t speed_rpm, uint8_t state, uint8_t faults,
                               int16_t cmd_raw, uint16_t vbus_v,
                               int16_t iq_ma, int16_t id_ma, uint8_t mcsdk_st);

/* Call from USART2_IRQHandler USER CODE BEGIN 0 -- processes one RXNE byte. */
void    ESC_COMM_UART_RxISR(void);

#endif /* ESC_COMM_H */