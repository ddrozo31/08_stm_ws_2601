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
#define ESC_CMD_SOF        0xAAU   /*!< Drive command frame SOF (RPi5 → STM32) */
#define ESC_TLM_SOF        0xBBU   /*!< Telemetry frame SOF   (STM32 → RPi5)  */
#define ESC_CFG_SOF        0xCCU   /*!< Config frame SOF      (RPi5 → STM32)  */

/* Frame sizes (bytes) */
#define ESC_CMD_FRAME_LEN  5U
#define ESC_TLM_FRAME_LEN  21U

/* Config frame (RPi5 → STM32, 5 bytes):
 *   [0xCC] [param_id] [val_lo] [val_hi] [XOR of bytes 1..3]
 *
 * param_id values and encoding:
 *   ESC_CFG_PARAM_MAX_IQ    val = int16_t × 0.1 A  (e.g. 120 = 12.0 A, range 10–150)
 *   ESC_CFG_PARAM_REVUP     val = int16_t RPM        (range 1600–5000)
 *   ESC_CFG_PARAM_BOOST_IQ  val = int16_t × 0.1 A  (e.g. 120 = 12.0 A, range 10–150)
 *   ESC_CFG_PARAM_MAX_SPD   val = int16_t RPM        (range 1000–10000)
 *   ESC_CFG_PARAM_IQ_LIMIT  val = int16_t × 0.1 A  (e.g. 120 = 12.0 A, range 10–200)
 *   ESC_CFG_PARAM_OL_IQ     val = int16_t × 0.1 A  (e.g. 80  = 8.0 A,  range 20–150)
 *   ESC_CFG_PARAM_OL_RAMP   val = int16_t ms        (range 1000–8000)
 *   ESC_CFG_PARAM_ALIGN_MS  val = int16_t ms        (range 100–2000)
 *   ESC_CFG_PARAM_ALIGN_ID  val = int16_t × 0.1 A  (e.g. 50  = 5.0 A,  range 10–150)
 *
 * Config frames can be sent at any time; values take effect on the next motor start.
 * If a param is never configured, the firmware compile-time defaults are used. */
#define ESC_CFG_PARAM_MAX_IQ    0x01U  /*!< Max torque current (Iq) */
#define ESC_CFG_PARAM_REVUP     0x02U  /*!< Rev-up target speed     */
#define ESC_CFG_PARAM_BOOST_IQ  0x03U  /*!< RUN-entry boost current */
#define ESC_CFG_PARAM_MAX_SPD   0x04U  /*!< Max speed RPM (u=1.0 maps to this) */
#define ESC_CFG_PARAM_IQ_LIMIT  0x05U  /*!< Speed PI Iq clamp [A × 0.1]       */
#define ESC_CFG_PARAM_OL_IQ     0x06U  /*!< Open-loop Iq target [A × 0.1]     */
#define ESC_CFG_PARAM_OL_RAMP   0x07U  /*!< OL speed ramp duration [ms]        */
#define ESC_CFG_PARAM_ALIGN_MS  0x08U  /*!< Alignment duration [ms]            */
#define ESC_CFG_PARAM_ALIGN_ID  0x09U  /*!< Alignment d-axis current [A × 0.1] */
#define ESC_CFG_PARAM_XF_DUR   0x0AU  /*!< Crossfade blend duration [ms]      */
#define ESC_CFG_PARAM_XF_DWELL 0x0BU  /*!< Crossfade dwell time [ms]          */
#define ESC_CFG_PARAM_OL_RPM   0x0CU  /*!< OL target / crossfade speed [RPM]  */
#define ESC_CFG_PARAM_SPD_KP   0x0DU  /*!< Speed PI Kp [× 0.001 A/RPM]       */
#define ESC_CFG_PARAM_SPD_KI   0x0EU  /*!< Speed PI Ki [× 0.0001 A/RPM]      */
#define ESC_CFG_PARAM_SPD_LPF  0x0FU  /*!< Speed PI LPF alpha [× 0.0001]     */

/* Step 8 adaptive-R EKF params (Phase B hooks, consumed in Phase C).
 * All five are no-ops while USE_ADAPTIVE_R_EKF=0 OR observer_mode=0.
 * Ranges chosen to cover the (ω_thresh, Q_e) grid that Phase A swept. */
#define ESC_CFG_PARAM_OBS_MODE   0x14U /*!< 0 = discrete (legacy), 1 = adaptive-R */
#define ESC_CFG_PARAM_EKF_WTHRESH 0x15U /*!< ω_thresh [elec rad/s, int16, 50–400]  */
#define ESC_CFG_PARAM_EKF_R0     0x16U /*!< R0 × 10000 [A², int16, 1–10000]        */
#define ESC_CFG_PARAM_EKF_QE     0x17U /*!< Q_e × 10000 [int16, 1–10000]           */
#define ESC_CFG_PARAM_EKF_VFPL   0x18U /*!< V/f prior force-lock below ω_thresh (0/1) */

/* Initialise the layer and enable the USART2 RXNE interrupt. */
void    ESC_COMM_Init(void);

/* Returns the most recently validated command value (int16_t, raw scaled). */
int16_t ESC_COMM_GetCommand(void);

/* Returns 1 if a new command arrived since the last call; clears the flag. */
uint8_t ESC_COMM_HasNewCommand(void);

/* Build and transmit one telemetry frame (blocking, 21 bytes).
 * Frame: [0xBB][spd_lo][spd_hi][esc_st][faults][u_lo][u_hi][v_lo][v_hi]
 *        [iq_lo][iq_hi][id_lo][id_hi][mc_st][innov_lo][innov_hi]
 *        [kappa_lo][kappa_hi][res_lo][res_hi][XOR of bytes 1..19]
 *   kappa_x10000: lock-confidence κ [×10000]  (uint16 saturating)
 *   res_x1000   : lock residual |Vq-(Rs·Iq+Ψf·ω)| [V × 1000] (uint16 saturating) */
void    ESC_COMM_SendTelemetry(int16_t speed_rpm, uint8_t state, uint8_t faults,
                               int16_t cmd_raw, uint16_t vbus_v,
                               int16_t iq_ma, int16_t id_ma, uint8_t mcsdk_st,
                               uint16_t innov_x1000,
                               uint16_t kappa_x10000, uint16_t res_x1000);

/* Runtime-configurable parameters (set by config frame; -1 if not configured).
 * mc_app_hooks.c falls back to compile-time defaults when these return <= 0. */
float   ESC_COMM_GetMaxIqA(void);     /*!< Max torque current (A) or -1 if not set */
float   ESC_COMM_GetRevupRPM(void);   /*!< Rev-up target speed (RPM) or -1 if not set */
float   ESC_COMM_GetBoostIqA(void);   /*!< RUN-entry boost current (A) or -1 if not set */
float   ESC_COMM_GetMaxSpeedRPM(void); /*!< Max speed RPM or -1 if not set */
float   ESC_COMM_GetIqLimitA(void);    /*!< Speed PI Iq clamp (A) or -1 if not set */
float   ESC_COMM_GetOlIqA(void);       /*!< OL Iq target (A) or -1 if not set */
float   ESC_COMM_GetOlRampMs(void);    /*!< OL ramp duration (ms) or -1 if not set */
float   ESC_COMM_GetAlignMs(void);     /*!< Alignment duration (ms) or -1 if not set */
float   ESC_COMM_GetAlignIdA(void);    /*!< Alignment Id (A) or -1 if not set */
float   ESC_COMM_GetXfDurationMs(void); /*!< Crossfade duration (ms) or -1    */
float   ESC_COMM_GetXfDwellMs(void);   /*!< Crossfade dwell (ms) or -1       */
float   ESC_COMM_GetOlTargetRPM(void); /*!< OL target speed (RPM) or -1      */
float   ESC_COMM_GetSpdKp(void);       /*!< Speed PI Kp (A/RPM) or -1        */
float   ESC_COMM_GetSpdKi(void);       /*!< Speed PI Ki (A/RPM) or -1        */
float   ESC_COMM_GetSpdLpfAlpha(void); /*!< Speed PI LPF alpha or -1         */

/* Step 8 adaptive-R EKF getters. Return a sentinel (-1 for floats, 0xFF for
 * observer_mode, 0xFF for vf_prior_lock) when the host never sent the param. */
uint8_t ESC_COMM_GetObserverMode(void);      /*!< 0=discrete, 1=adaptive, 0xFF=unset */
float   ESC_COMM_GetEkfOmegaThreshRad(void); /*!< elec rad/s, or -1 if unset   */
float   ESC_COMM_GetEkfR0(void);             /*!< A² (decoded), or -1 if unset */
float   ESC_COMM_GetEkfQe(void);             /*!< decoded, or -1 if unset      */
uint8_t ESC_COMM_GetEkfVfPriorLock(void);    /*!< 0/1, 0xFF=unset              */

/* Call from USART2_IRQHandler USER CODE BEGIN 0 -- processes one RXNE byte. */
void    ESC_COMM_UART_RxISR(void);

#endif /* ESC_COMM_H */