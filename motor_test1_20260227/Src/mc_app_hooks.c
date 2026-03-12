/**
  ******************************************************************************
  * @file    mc_app_hooks.c
  * @brief   ESC application layer — ESC state machine, command mapping, telemetry.
  *
  * ESC state machine states:
  *   BOOT         – one-shot init; transitions to WAIT_NEUTRAL immediately.
  *   WAIT_NEUTRAL – requires u≈0 before enabling (boot safety / post-fault).
  *   READY        – neutral confirmed; motor stopped; accepts drive commands.
  *   FORWARD      – motor running forward (positive RPM).
  *   BRAKE        – decelerating to zero before direction reversal.
  *   REVERSE      – motor running reverse (negative RPM).
  *   FAULT        – MCSDK fault active; waiting for hardware clearance.
  *
  * Transition rules:
  *   BOOT         → WAIT_NEUTRAL    always (one shot)
  *   WAIT_NEUTRAL → READY           when new command received with |u| < deadband
  *   READY        → FORWARD         when u >  deadband  (motor already stopped)
  *   READY        → REVERSE         when u < -deadband  (motor already stopped)
  *   FORWARD      → READY           when |u| < deadband or timeout (stop motor)
  *   FORWARD      → BRAKE           when u < -deadband (sign reversal)
  *   REVERSE      → READY           when |u| < deadband or timeout (stop motor)
  *   REVERSE      → BRAKE           when u >  deadband (sign reversal)
  *   BRAKE        → FORWARD         when MCSDK=IDLE and u >  deadband
  *   BRAKE        → REVERSE         when MCSDK=IDLE and u < -deadband
  *   BRAKE        → READY           when MCSDK=IDLE and |u| < deadband
  *   any          → FAULT           when MCSDK fault detected
  *   FAULT        → WAIT_NEUTRAL    when MCSDK reaches FAULT_OVER (auto-ack)
  *
  * Telemetry reports ESC state (not MCSDK state) so the host always knows
  * the application-level status.
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
  * @ingroup MCAppHooks
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_app_hooks.h"
#include "mc_config.h"         /* PotRegConv_M1 (ADC scheduler slot) */

#ifdef BUILD_ESC
#include "mc_interface.h"      /* MCI_State_t, IDLE, START, RUN, FAULT_NOW, FAULT_OVER */
#include "mc_api.h"            /* MC_StartMotor1, MC_StopMotor1, MC_ProgramSpeedRampMotor1_F */
#include "mc_config_common.h"  /* BusVoltageSensor_M1 extern */
#include "bus_voltage_sensor.h"/* VBS_GetAvBusVoltage_V */
#include "esc_comm.h"          /* ESC_COMM_Init, ESC_COMM_GetCommand, ESC_COMM_SendTelemetry */
#include <math.h>              /* fabsf */
#endif

/** @addtogroup MCSDK  @{ */
/** @addtogroup MCTasks  @{ */
/** @defgroup MCAppHooks Motor Control Applicative hooks  @{ */

#ifdef BUILD_ESC
/* ESC state machine type --------------------------------------------------- */

typedef enum
{
  ESC_BOOT         = 0,  /* initial state — transitions to WAIT_NEUTRAL on first call */
  ESC_WAIT_NEUTRAL = 1,  /* require u≈0 before enabling (boot safety / post-fault)    */
  ESC_READY        = 2,  /* neutral confirmed; motor stopped; awaiting commands        */
  ESC_FORWARD      = 3,  /* motor running forward (positive RPM)                      */
  ESC_BRAKE        = 4,  /* decelerating to zero before direction reversal             */
  ESC_REVERSE      = 5,  /* motor running reverse (negative RPM)                      */
  ESC_FAULT        = 6,  /* MCSDK fault active; waiting for hardware clearance         */
} ESC_State_t;

/* Parameters --------------------------------------------------------------- */

/* Rev-up target speed (RPM): the speed programmed into the speed loop while
 * the motor is still in open-loop START state.  Once the STO observer locks
 * and the motor transitions to RUN, torque mode takes over and this value is
 * no longer used.  Must be >= OBS_MINIMUM_SPEED_RPM (2500). */
#define ESC_REVUP_SPEED_RPM   2500.0f

/* Speed ramp applied during the rev-up transition (ms). */
#define ESC_REVUP_RAMP_MS     500U

/* Maximum torque current (Amps).  |u|=1.0 maps to this Iq.
 * Drivetrain analysis (3.142kg, 4WD, 3 diffs, ~10.6:1 ratio, Kt~0.003 N·m/A):
 *   Stiction break (mu=0.3, smooth floor): ~9.2N -> ~4.8A needed at wheel
 *   Stiction break (mu=0.5, carpet):      ~15.4N -> ~8.1A (near hw limit)
 * Off-ground rosbag: observer stable up to 6.4A open-loop, 4.7A in RUN.
 * 7A raises wheel force to ~11N -- breaks stiction on smooth/low-friction surfaces.
 * NOMINAL_CURRENT_A=10, IQMAX_A=10 in pmsm_motor_parameters.h support this. */
#define ESC_MAX_IQ_A          7.0f

/* Torque ramp duration (ms).  Short: torque response should track the
 * joystick quickly; the load sets the actual speed. */
#define ESC_TORQUE_RAMP_MS    50U

/* |u| below this threshold is treated as neutral. */
#define ESC_NEUTRAL_DEADBAND  0.02f

/* Time without a valid command before stopping the motor (ms).
 * Applies only in FORWARD and REVERSE. */
#define ESC_TIMEOUT_MS        500U

/* Send one telemetry frame every N hook calls (~10 Hz at 1 kHz task rate). */
#define ESC_TELEMETRY_EVERY   100U

/* Private state ------------------------------------------------------------ */

static ESC_State_t esc_state         = ESC_BOOT;
static uint16_t    esc_timeout_ctr   = 0U;
static uint16_t    esc_telemetry_ctr = 0U;

/* -------------------------------------------------------------------------- */

/**
  * @brief Hook called at the end of MCboot().
  *
  * Registers the potentiometer ADC conversion slot (keeps the ADC regular-
  * conversion scheduler balanced) and initialises the ESC UART layer.
  */
__weak void MC_APP_BootHook(void)
{
  (void)RCM_RegisterRegConv(&PotRegConv_M1);
  ESC_COMM_Init();

/* USER CODE BEGIN BootHook */

/* USER CODE END BootHook */
}

/**
  * @brief Hook called every Medium Frequency task cycle (~1 kHz).
  *
  * Reads the latest UART command, runs the ESC state machine, and sends
  * telemetry to the host at ~10 Hz.
  */
__weak void MC_APP_PostMediumFrequencyHook_M1(void)
{
  /* ---- Read inputs -------------------------------------------------------- */

  uint8_t     new_cmd = ESC_COMM_HasNewCommand();   /* 1 if fresh frame arrived */
  int16_t     raw     = ESC_COMM_GetCommand();       /* int16, −32768..+32767    */
  float       u       = (float)raw / 32767.0f;       /* normalised [−1.0, +1.0]  */
  MCI_State_t mci_st  = MC_GetSTMStateMotor1();
  float       speed   = MC_GetAverageMecSpeedMotor1_F();   /* signed RPM         */
  uint32_t    faults  = MC_GetCurrentFaultsMotor1();

  /* ---- Timeout watchdog --------------------------------------------------- */

  if (new_cmd != 0U)
  {
    esc_timeout_ctr = 0U;
  }
  else if (esc_timeout_ctr < ESC_TIMEOUT_MS)
  {
    esc_timeout_ctr++;
  }

  uint8_t timed_out = (esc_timeout_ctr >= ESC_TIMEOUT_MS) ? 1U : 0U;

  /* ---- Fault detection (priority: overrides all other states) ------------- */
  /* Also catch FAULT_OVER: current-faults bitmask is 0 in that state but
   * MC_StartMotor1() still refuses to run until acknowledged. */

  if (((faults != 0U) || (mci_st == FAULT_NOW) || (mci_st == FAULT_OVER))
      && (esc_state != ESC_FAULT))
  {
    (void)MC_StopMotor1();
    esc_state = ESC_FAULT;
  }

  /* ---- ESC state machine -------------------------------------------------- */

  switch (esc_state)
  {
    /* ---------------------------------------------------------------------- */
    case ESC_BOOT:
      esc_state = ESC_WAIT_NEUTRAL;
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_WAIT_NEUTRAL:
      /* esc_cmd_value initialises to 0x7FFF (non-neutral), so this condition
       * stays false until the host explicitly sends a neutral frame.
       * No fresh-flag check needed — the value itself carries the intent.   */
      if (fabsf(u) < ESC_NEUTRAL_DEADBAND)
      {
        esc_state = ESC_READY;
      }
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_READY:
      /* Motor is stopped. Accept drive commands in either direction.
       * Require new_cmd to avoid restarting on a stale command value.
       * Start with a speed-mode rev-up target; torque mode takes over once
       * the STO observer locks and the MCSDK transitions to RUN state. */
      if ((new_cmd != 0U) && (u > ESC_NEUTRAL_DEADBAND))
      {
        if (MC_StartMotor1())
        {
          (void)MC_ProgramSpeedRampMotor1_F(ESC_REVUP_SPEED_RPM, ESC_REVUP_RAMP_MS);
          esc_state = ESC_FORWARD;
        }
      }
      else if ((new_cmd != 0U) && (u < -ESC_NEUTRAL_DEADBAND))
      {
        if (MC_StartMotor1())
        {
          (void)MC_ProgramSpeedRampMotor1_F(-ESC_REVUP_SPEED_RPM, ESC_REVUP_RAMP_MS);
          esc_state = ESC_REVERSE;
        }
      }
      /* Neutral or no fresh command: stay READY. */
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_FORWARD:
      if (timed_out || (fabsf(u) < ESC_NEUTRAL_DEADBAND))
      {
        /* Neutral or communication timeout: stop and return to READY. */
        if ((mci_st == RUN) || (mci_st == START))
        {
          (void)MC_StopMotor1();
        }
        esc_state = ESC_READY;
      }
      else if (u < -ESC_NEUTRAL_DEADBAND)
      {
        /* Sign reversal: must brake to zero before entering REVERSE. */
        if ((mci_st == RUN) || (mci_st == START))
        {
          (void)MC_StopMotor1();
        }
        esc_state = ESC_BRAKE;
      }
      else if (mci_st == RUN)
      {
        /* Direction sanity: STO observer can converge to the 180-deg wrong
         * angle solution.  If estimated speed sign disagrees with the forward
         * command, stop cleanly back to READY so the user can retry without
         * having to re-send neutral (avoids the FAULT -> WAIT_NEUTRAL path). */
        if (speed < -50.0f)
        {
          (void)MC_StopMotor1();
          esc_state = ESC_READY;
        }
        else
        {
          /* Observer locked at correct angle: torque mode.
           * Joystick maps directly to Iq -- speed is set by the load. */
          (void)MC_ProgramTorqueRampMotor1_F(u * ESC_MAX_IQ_A, ESC_TORQUE_RAMP_MS);
        }
      }
      else if (mci_st == IDLE)
      {
        /* Motor exited to IDLE while joystick is still commanding forward
         * (observer failed or direction-check rejected).  Auto-restart so
         * the car keeps moving without requiring the user to re-push. */
        if (new_cmd != 0U)
        {
          if (MC_StartMotor1())
          {
            (void)MC_ProgramSpeedRampMotor1_F(ESC_REVUP_SPEED_RPM, ESC_REVUP_RAMP_MS);
          }
        }
      }
      /* START: open-loop rev-up in progress, no action needed. */
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_BRAKE:
      /* Wait for the MCSDK to reach IDLE (motor coasted/stopped).
       * Require new_cmd before dispatching to FORWARD/REVERSE to avoid acting
       * on a stale command if communication was lost during braking. */
      if (mci_st == IDLE)
      {
        if ((new_cmd != 0U) && (u < -ESC_NEUTRAL_DEADBAND))
        {
          /* Reverse pending: motor is stopped, safe to start in reverse. */
          if (MC_StartMotor1())
          {
            (void)MC_ProgramSpeedRampMotor1_F(-ESC_REVUP_SPEED_RPM, ESC_REVUP_RAMP_MS);
            esc_state = ESC_REVERSE;
          }
        }
        else if ((new_cmd != 0U) && (u > ESC_NEUTRAL_DEADBAND))
        {
          /* Changed mind during braking: go forward instead. */
          if (MC_StartMotor1())
          {
            (void)MC_ProgramSpeedRampMotor1_F(ESC_REVUP_SPEED_RPM, ESC_REVUP_RAMP_MS);
            esc_state = ESC_FORWARD;
          }
        }
        else
        {
          /* Neutral, no fresh command, or comm lost: motor stopped, go to READY. */
          esc_state = ESC_READY;
        }
      }
      /* Still decelerating: stay in BRAKE, motor handles the stop. */
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_REVERSE:
      if (timed_out || (fabsf(u) < ESC_NEUTRAL_DEADBAND))
      {
        /* Neutral or communication timeout: stop and return to READY. */
        if ((mci_st == RUN) || (mci_st == START))
        {
          (void)MC_StopMotor1();
        }
        esc_state = ESC_READY;
      }
      else if (u > ESC_NEUTRAL_DEADBAND)
      {
        /* Sign reversal: must brake to zero before entering FORWARD. */
        if ((mci_st == RUN) || (mci_st == START))
        {
          (void)MC_StopMotor1();
        }
        esc_state = ESC_BRAKE;
      }
      else if (mci_st == RUN)
      {
        /* Direction sanity: same as FORWARD -- if observer locked at wrong
         * angle, estimated speed will be positive despite reverse command.
         * Stop cleanly back to READY for immediate retry. */
        if (speed > 50.0f)
        {
          (void)MC_StopMotor1();
          esc_state = ESC_READY;
        }
        else
        {
          /* Observer locked at correct angle: torque mode.
           * u is negative here -- negative Iq -- reverse torque. */
          (void)MC_ProgramTorqueRampMotor1_F(u * ESC_MAX_IQ_A, ESC_TORQUE_RAMP_MS);
        }
      }
      else if (mci_st == IDLE)
      {
        /* Same as FORWARD: auto-restart in reverse if joystick still pushed. */
        if (new_cmd != 0U)
        {
          if (MC_StartMotor1())
          {
            (void)MC_ProgramSpeedRampMotor1_F(-ESC_REVUP_SPEED_RPM, ESC_REVUP_RAMP_MS);
          }
        }
      }
      /* START: open-loop rev-up in progress, no action needed. */
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_FAULT:
      /* FAULT_OVER: the fault condition is gone but not yet acknowledged.
       * Acknowledge it, then restart immediately if joystick still pushed
       * (RC-car behaviour: observer loss is transient, retry is correct).
       * Only fall back to WAIT_NEUTRAL if joystick is at neutral. */
      if (mci_st == FAULT_OVER)
      {
        (void)MC_AcknowledgeFaultMotor1();
        if (u > ESC_NEUTRAL_DEADBAND)
        {
          if (MC_StartMotor1())
          {
            (void)MC_ProgramSpeedRampMotor1_F(ESC_REVUP_SPEED_RPM, ESC_REVUP_RAMP_MS);
          }
          esc_state = ESC_FORWARD;
        }
        else if (u < -ESC_NEUTRAL_DEADBAND)
        {
          if (MC_StartMotor1())
          {
            (void)MC_ProgramSpeedRampMotor1_F(-ESC_REVUP_SPEED_RPM, ESC_REVUP_RAMP_MS);
          }
          esc_state = ESC_REVERSE;
        }
        else
        {
          esc_state = ESC_WAIT_NEUTRAL;
        }
      }
      break;

    /* ---------------------------------------------------------------------- */
    default:
      esc_state = ESC_WAIT_NEUTRAL;
      break;
  }

  /* ---- Telemetry ---------------------------------------------------------- */

  esc_telemetry_ctr++;
  if (esc_telemetry_ctr >= ESC_TELEMETRY_EVERY)
  {
    esc_telemetry_ctr = 0U;

    int16_t  spd_rpm = (int16_t)speed;
    uint8_t  esc_st  = (uint8_t)esc_state;
    uint8_t  fault_b = (uint8_t)(faults & 0xFFU);
    int16_t  cmd_raw = ESC_COMM_GetCommand();
    uint16_t vbus_v  = VBS_GetAvBusVoltage_V((const BusVoltageSensor_Handle_t *)&BusVoltageSensor_M1);
    qd_f_t   iqd     = MC_GetIqdMotor1_F();
    int16_t  iq_ma   = (int16_t)(iqd.q * 1000.0f);   /* Amps -> milliAmps */
    int16_t  id_ma   = (int16_t)(iqd.d * 1000.0f);
    uint8_t  mc_st   = (uint8_t)mci_st;

    ESC_COMM_SendTelemetry(spd_rpm, esc_st, fault_b, cmd_raw, vbus_v,
                           iq_ma, id_ma, mc_st);
  }

/* USER SECTION BEGIN PostMediumFrequencyHookM1 */

/* USER SECTION END PostMediumFrequencyHookM1 */
}

#else /* !BUILD_ESC — Debug / Motor Pilot build ----------------------------- */

/**
  * @brief Hook called at the end of MCboot() — Motor Pilot build.
  *
  * Registers the potentiometer ADC slot only; ESC UART layer is not started.
  * ASPEP owns USART2; Motor Pilot connects over ST-Link VCP.
  */
__weak void MC_APP_BootHook(void)
{
  (void)RCM_RegisterRegConv(&PotRegConv_M1);

/* USER CODE BEGIN BootHook */

/* USER CODE END BootHook */
}

/**
  * @brief Hook called every Medium Frequency task cycle — Motor Pilot build.
  *
  * Empty: Motor Pilot drives the motor directly via MCP/ASPEP.
  * No ESC state machine, no UART command processing.
  */
__weak void MC_APP_PostMediumFrequencyHook_M1(void)
{
/* USER SECTION BEGIN PostMediumFrequencyHookM1 */

/* USER SECTION END PostMediumFrequencyHookM1 */
}

#endif /* BUILD_ESC */

/** @} */
/** @} */
/** @} */

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
