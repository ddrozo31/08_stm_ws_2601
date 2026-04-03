
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
 * Ls=10uH, Rs=100mOhm (Motor Pilot measured): C1=F1*RS/(LS*TF_RATE)=8192*0.1/(10e-6*25000)=3277
 * and C1/F1=0.40 -- stable (requires C1/F1 < 1).
 * C5 ~= 25866*(4/10) = 10346 -- well within int16_t range (was 25866 at Ls=4uH).
 * GAIN1=-9830, F1=8192: stable baseline (verified hardware). */
#define F1                                  8192
#define F2                                  8192
#define F1_LOG                              LOG2((8192))
#define F2_LOG                              LOG2((8192))

/* State observer constants */
#define GAIN1                               -9830  /* Stable baseline: -16000 caused observer divergence at 1500 RPM handoff (bogus 30kRPM readings) */
#define GAIN2                               19648

/* Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                         638  /* Stable baseline: 1500 caused observer divergence at 1500 RPM handoff */
#define PLL_KI_GAIN                         18  /* Stable baseline: 60 caused observer divergence at 1500 RPM handoff */
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
 * Motor Pilot measured Rs=0.1 Ohm, Ls=10 uH. Pole-zero cancellation verified:
 * KI/KP*(KPDIV/KIDIV) = (395/247)*(4096/16384) = 0.400 = Ts*Rs/Ls = 40e-6*0.1/10e-6.
 * Bandwidth: (247/4096)*5.52/10e-6 ~ 33000 rad/s ~ 5300 Hz. No change needed. */
#define PID_TORQUE_KP_DEFAULT               247  /* Motor Pilot Rs=0.1 Ohm, Ls=10 uH -- pole-zero cancellation verified */
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
#define PHASE1_DURATION                     1200 /*milliseconds -- AMORIL: restored 1500->1200ms; lighter drivetrain, shorter alignment */
#define PHASE1_FINAL_SPEED_UNIT             (0*SPEED_UNIT/U_RPM)
#define PHASE1_FINAL_CURRENT_A              4.0  /* AMORIL: reduced 8->4A — lighter drivetrain, less stiction */

/* Phase 2 */
#define PHASE2_DURATION                     1200 /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT             (500*SPEED_UNIT/U_RPM)
#define PHASE2_FINAL_CURRENT_A              4.0  /* AMORIL: reduced 8->4A */

/* Phase 3 */
#define PHASE3_DURATION                     1200 /* ms -- AMORIL: restored 1500->1200ms; 417 RPM/s (500->1000 in 1200ms) */
#define PHASE3_FINAL_SPEED_UNIT             (1000*SPEED_UNIT/U_RPM)
#define PHASE3_FINAL_CURRENT_A              5.0  /* AMORIL: reduced 8->5A; sufficient torque for lighter drivetrain */

/* Phase 4 */
#define PHASE4_DURATION                     1800 /* ms -- AMORIL: shortened 2500->1800ms; 333 RPM/s (1000->1600 in 1800ms) */
#define PHASE4_FINAL_SPEED_UNIT             (1600*SPEED_UNIT/U_RPM)
#define PHASE4_FINAL_CURRENT_A              5.0  /* AMORIL: reduced 10->5A */

/* Phase 5 */
#define PHASE5_DURATION                     3000 /* ms -- hold 1600 RPM steady for EKF observer lock.
                                                    AMORIL: 4A at 1600 RPM → SNR = (0.25×1.6)/(0.1×4) = 1.0 (breakeven).
                                                    Lighter drivetrain can hold 1600 RPM at 4A (unlike HOSIM which needed 8A). */
#define PHASE5_FINAL_SPEED_UNIT             (1600*SPEED_UNIT/U_RPM)
#define PHASE5_FINAL_CURRENT_A              4.0  /* AMORIL: reduced 8->4A — SNR=1.0, lighter drivetrain holds speed at 4A */

#define ENABLE_SL_ALGO_FROM_PHASE           2

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG                  90  /*!< degrees [0...359] */

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM               1600  /* EKF campaign baseline: same threshold for HOSIM and AMORIL.
                                                    AMORIL original was 2500 RPM (Luenberger); EKF targets 600 RPM.
                                                    Must match ESC_REVUP_SPEED_RPM in mc_app_hooks.c */
#define NB_CONSECUTIVE_TESTS                4  /* AMORIL: restored 12->4 — original AMORIL value; lighter drivetrain,
                                                  better SNR → observer locks faster, fewer consecutive tests needed */
#define SPEED_BAND_UPPER_LIMIT              21 /*!< It expresses how much estimated speed can exceed forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT              11 /*!< It expresses how much estimated speed can be below forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */

#define TRANSITION_DURATION                 50  /* HOSIM: halved 100->50ms — reduces speed drop during blend from ~650 to ~325 RPM;
                                                   post-SWITCH_OVER speed ~1675 RPM (vs ~1350 RPM at 100ms), keeps BEMF above observer floor */

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

/* ── EKF Observer (Step 5 MCSDK integration) ─────────────────────────────── *
 *  0 = legacy STO+PLL only (HOSIM iter3 behaviour)                           *
 *  1 = EKF fully replaces STO: overwrites angle, speed, and convergence      *
 * --------------------------------------------------------------------------- */
#define USE_EKF_OBSERVER          1

/* PM flux linkage [Wb].  Derived from MOTOR_VOLTAGE_CONSTANT = 0.25 V_rms_LL/kRPM:
 *   Ψf = Ke_LL_rms × √2/√3 / (1000 RPM × 2π/60 × p) = 9.75×10⁻⁴ Wb  */
#define EKF_PSI_F_WB              9.75e-4f

/* EKF noise covariance — tuned from noise_floor bag (2026-03-27):
 *   σ_I = 57 mA (ADC noise floor, motored-off current measurement)
 *   q_i = σ_I² = 3.33×10⁻³ A² ; q_e = 10 × q_i (BEMF states more uncertain)
 *   r_i = σ_I² = 3.33×10⁻³ A² (measurement noise matches process noise floor) */
#define EKF_Q_CURRENT             3.33e-3f   /*!< Process noise: current states [A²]  */
#define EKF_Q_BEMF                3.33e-2f   /*!< Process noise: BEMF states          */
#define EKF_R_CURRENT             3.33e-3f   /*!< Measurement noise: currents [A²]    */

/* EKF convergence gate — how long (ms at 1 kHz MF rate) the EKF BEMF must
 * exceed EKF_BEMF_SQ_CONVERGE before declaring observer-converged and
 * triggering SWITCH_OVER.  500 ms gives the EKF a stable BEMF estimate
 * well into Phase 5 before committing to closed-loop. */
#define EKF_CONVERGE_DWELL_MS     500U

/* EKF BEMF² threshold [V²] for the convergence dwell counter.
 * At Ψf = 9.75e-4 Wb, 1600 RPM → |BEMF|² ≈ 0.107 V².
 * 0.05 V² corresponds to ~1100 RPM equivalent. */
#define EKF_BEMF_SQ_CONVERGE      0.05f

/* EKF BEMF² threshold [V²] for HF angle/speed override.
 * Lower than the convergence threshold so the EKF starts feeding
 * angle/speed into STO_PLL_M1._Super early in Phase 3–4.
 * 0.005 V² ≈ 360 RPM equivalent. */
#define EKF_BEMF_SQ_OVERRIDE      0.005f

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
