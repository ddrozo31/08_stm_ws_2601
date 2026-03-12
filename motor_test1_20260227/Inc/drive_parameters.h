
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
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
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM           15000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM           0 /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS     3 /*!< Number of speed measurement errors before main sensor goes in fault */

/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD                  0.99 /*!< Very tolerant: fault only on total divergence; needed at 1500 RPM weak BEMF */


/* State observer scaling factors F1
 * Ls=4uH, Rs=50mOhm: C1=F1*RS/(LS*TF_RATE)=8192*0.05/(4e-6*25000)=4096
 * and C1/F1=0.5 -- stable (requires C1/F1 < 1). Better current model vs old 5uH/0.1Ohm.
 * C5 ~= 20693*(5/4) = 25866 -- fits in int16_t (3uH would overflow at ~34488).
 * GAIN1 scaled from original -19661 (F1=16384) -> -9830 (F1=8192, same effective gain). */
#define F1                                  8192
#define F2                                  8192
#define F1_LOG                              LOG2((8192))
#define F2_LOG                              LOG2((8192))

/* State observer constants */
#define GAIN1                               -16000  /* Increased: faster BEMF tracking at 1500 RPM weak BEMF under load */
#define GAIN2                               19648

/* Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                         1500  /* Increased: faster angle tracking at low-speed BEMF */
#define PLL_KI_GAIN                         60  /* Increased: faster angle lock under drivetrain load */
#define PLL_KPDIV                           16384
#define PLL_KPDIV_LOG                       LOG2((PLL_KPDIV))
#define PLL_KIDIV                           65535
#define PLL_KIDIV_LOG                       LOG2((PLL_KIDIV))

#define STO_FIFO_DEPTH_DPP                  64 /*!< Depth of the FIFO used  to average mechanical speed in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG              LOG2((64))
#define STO_FIFO_DEPTH_UNIT                 64 /*!< Depth of the FIFO used to average mechanical speed in the unit defined by #SPEED_UNIT */
#define M1_BEMF_CONSISTENCY_TOL             64 /* Parameter for B-emf amplitude-speed consistency */
#define M1_BEMF_CONSISTENCY_GAIN            16 /* Parameter for B-emf amplitude-speed consistency */

/* USER CODE BEGIN angle reconstruction M1 */
#define PARK_ANGLE_COMPENSATION_FACTOR      0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR  0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                       25000
#define PWM_FREQ_SCALING                    1
#define LOW_SIDE_SIGNALS_ENABLING           LS_PWM_TIMER
#define SW_DEADTIME_NS                      750 /*!< Dead-time to be inserted by FW, only if low side signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE           1 /*!< FOC execution rate in number of PWM cycles */
#define ISR_FREQUENCY_HZ                    (PWM_FREQUENCY/REGULATION_EXECUTION_RATE) /*!< @brief FOC execution rate in Hz */

/* Gains values for torque and flux control loops
 * Firmware uses Ls=5µH (model), true motor Ls~1µH.  Kp/Ki scale with Ls:
 * base values at Ls=1µH were Kp=247, Ki=395; ×5 for Ls=5µH model.
 * Refine with Motor Pilot live parameter write after startup is confirmed. */
#define PID_TORQUE_KP_DEFAULT               247  /* Ls=1uH true motor; x5=1235 was wrong (loop saw real Ls, not model) */
#define PID_TORQUE_KI_DEFAULT               395
#define PID_TORQUE_KD_DEFAULT               0
#define PID_FLUX_KP_DEFAULT                 247
#define PID_FLUX_KI_DEFAULT                 395
#define PID_FLUX_KD_DEFAULT                 0

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                            4096
#define TF_KIDIV                            16384
#define TF_KDDIV                            8192
#define TF_KPDIV_LOG                        LOG2((4096))
#define TF_KIDIV_LOG                        LOG2((16384))
#define TF_KDDIV_LOG                        LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING        DISABLE

#define PID_SPEED_KP_DEFAULT                75/(SPEED_UNIT/10) /* Reduced 150->75: integrator holds steady-state, KP only corrects small errors */
#define PID_SPEED_KI_DEFAULT                5/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT                0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ             (uint16_t)1000 /*!<Execution rate of speed regulation loop (Hz) */

/* Speed PID parameter dividers */
#define SP_KPDIV                            512
#define SP_KIDIV                            512
#define SP_KDDIV                            16
#define SP_KPDIV_LOG                        LOG2((512))
#define SP_KIDIV_LOG                        LOG2((512))
#define SP_KDDIV_LOG                        LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV         0 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING      DISABLE
#define IQMAX_A                             10 /*!< Raised 5->10A for on-ground load */

/* Default settings */
#define DEFAULT_CONTROL_MODE                MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM            6000
#define DEFAULT_TARGET_SPEED_UNIT           (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A          0
#define DEFAULT_FLUX_COMPONENT_A            0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V              14 /*!< Over-voltage threshold */
#define UD_VOLTAGE_THRESHOLD_V              8 /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                     TURN_OFF_PWM /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C          70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C         10 /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS           DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this
                                                         feature may be used to bypass HW over-current protection
                                                         (if supported by power stage) */
#define OVP_INVERTINGINPUT_MODE             INT_MODE
#define OVP_INVERTINGINPUT_MODE2            INT_MODE
#define OVP_SELECTION                       COMP_Selection_COMP1
#define OVP_SELECTION2                      COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/

/* Phase 1 */
#define PHASE1_DURATION                     1200 /*milliseconds -- longer alignment for stall-start */
#define PHASE1_FINAL_SPEED_UNIT             (0*SPEED_UNIT/U_RPM)
#define PHASE1_FINAL_CURRENT_A              4.0

/* Phase 2 */
#define PHASE2_DURATION                     1200 /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT             (500*SPEED_UNIT/U_RPM)
#define PHASE2_FINAL_CURRENT_A              4.0

/* Phase 3 */
#define PHASE3_DURATION                     1200 /*milliseconds */
#define PHASE3_FINAL_SPEED_UNIT             (1500*SPEED_UNIT/U_RPM)
#define PHASE3_FINAL_CURRENT_A              4.0

/* Phase 4 */
#define PHASE4_DURATION                     1500 /*milliseconds */
#define PHASE4_FINAL_SPEED_UNIT             (2600*SPEED_UNIT/U_RPM)
#define PHASE4_FINAL_CURRENT_A              4.0

/* Phase 5 */
#define PHASE5_DURATION                     6000 /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT             (6500*SPEED_UNIT/U_RPM)
#define PHASE5_FINAL_CURRENT_A              4.0

#define ENABLE_SL_ALGO_FROM_PHASE           2

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG                  90  /*!< degrees [0...359] */

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM               1500  /* Handoff at 1500 RPM (matches 903c040 that briefly moved the car) */
#define NB_CONSECUTIVE_TESTS                4 /* corresponding to former
                                                 NB_CONSECUTIVE_TESTS / (TF_REGULATION_RATE / MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT              21 /*!< It expresses how much estimated speed can exceed forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT              11 /*!< It expresses how much estimated speed can be below forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */

#define TRANSITION_DURATION                 200 /* Switch over duration, ms â€” increased for angle convergence */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(47)

/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(47)

/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES                 (6 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

/* **** Potentiometer parameters **** */
/** @brief Sampling time set to the ADC channel used by the potentiometer component */
#define POTENTIOMETER_ADC_SAMPLING_TIME_M1  LL_ADC_SAMPLING_CYCLE(47)

/**
 * @brief Speed reference set to Motor 1 when the potentiometer is at its maximum
 *
 * This value is expressed in #SPEED_UNIT.
 *
 * Default value is #MAX_APPLICATION_SPEED_UNIT.
 *
 * @sa POTENTIOMETER_MIN_SPEED_M1
 */
#define POTENTIOMETER_MAX_SPEED_M1          MAX_APPLICATION_SPEED_UNIT

/**
 * @brief Speed reference set to Motor 1 when the potentiometer is at its minimum
 *
 * This value is expressed in #SPEED_UNIT.
 *
 * Default value is 10 % of #MAX_APPLICATION_SPEED_UNIT.
 *
 * @sa POTENTIOMETER_MAX_SPEED_M1
 */
#define POTENTIOMETER_MIN_SPEED_M1          ((MAX_APPLICATION_SPEED_UNIT)/10)

/**
 * @brief Potentiometer change threshold to trigger speed reference update for Motor 1
 *
 * When the potentiometer value differs from the current speed reference by more than this
 * threshold, the speed reference set to the motor is adjusted to match the potentiometer value.
 *
 * The threshold is expressed in u16digits. Its default value is set to 13% of the potentiometer
 * aquisition range
 *
 */
 #define POTENTIOMETER_SPEED_ADJUSTMENT_RANGE_M1 (655)

/**
 * @brief Acceleration used to compute ramp duration when setting speed reference to Motor 1
 *
 * This acceleration is expressed in #SPEED_UNIT/s. Its default value is 100 Hz/s (provided
 * that #SPEED_UNIT is #U_01HZ).
 *
 */
 #define POTENTIOMETER_RAMP_SLOPE_M1        1000

/**
 * @brief Bandwith of the low pass filter applied on the potentiometer values
 *
 * @see SpeedPotentiometer_Handle_t::LPFilterBandwidthPOW2
 */
#define POTENTIOMETER_LPF_BANDWIDTH_POW2_M1 4

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
