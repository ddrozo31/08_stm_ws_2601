/**
  ******************************************************************************
  * @file    pmsm_motor_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the motor to drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PMSM_MOTOR_PARAMETERS_H
#define PMSM_MOTOR_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM           2 /* Number of motor pole pairs */
#define RS                      0.050 /* Stator resistance , ohm -- educated guess 50mOhm typical for 3100KV RC motor */
#define LS                      0.000004 /* Stator inductance, H (4 uH -- reduced from 5 uH;
                                                 true value ~1 uH but C5 overflow prevents going below ~3.2 uH.
                                                 With Rs=50mOhm: C1/F1=8192*0.05/(4e-6*25000)=0.5 (stable).
                                                 C5~25866 fits int16_t (3uH would overflow at ~34488).
                                                 Closer to true Ls => better current model accuracy under load.) */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define MOTOR_VOLTAGE_CONSTANT  0.25 /*!< Volts RMS ph-ph /kRPM -- Motor Pilot identification result (theoretical ~0.228 for 3100KV) */
#define MOTOR_MAX_SPEED_RPM     15000 /*!< Maximum rated speed  */
#define NOMINAL_CURRENT_A       10  /*!< Raised 5->10A for on-ground load */

#define ID_DEMAG_A              -10 /*!< Demagnetization current -- matched to NOMINAL_CURRENT_A */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        300 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR          400  /*!< Number of pulses per
                                            revolution */

#endif /* PMSM_MOTOR_PARAMETERS_H */
/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
