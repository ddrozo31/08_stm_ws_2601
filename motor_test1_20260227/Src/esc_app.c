/**
 * @file    esc_app.c
 * @brief   ESC application layer — UART-controlled motor state machine.
 *
 * Runs at 1 kHz from SysTick (after CFOC_MediumFrequencyTask).
 * Reads commands from esc_comm, drives custom_foc, sends telemetry.
 *
 * State flow:
 *   BOOT → WAIT_NEUTRAL → READY ↔ FORWARD/REVERSE (via BRAKE) → FAULT
 *
 * Control mode: speed mode.
 *   During startup (ALIGNMENT → OPEN_LOOP → CROSSFADE), CFOC manages
 *   its own current references internally. Once CFOC reaches CLOSED_LOOP,
 *   ESC sets speed target: speed_cmd = u × ESC_MAX_SPEED_RPM (runtime configurable).
 *   The speed PI in CFOC regulates Iq automatically.
 */

#ifdef BUILD_ESC

#include "esc_app.h"
#include "esc_comm.h"
#include "custom_foc.h"
#include <math.h>

/* ── Constants (compile-time defaults, overridable via 0xCC config) ─────── */
#define ESC_NEUTRAL_DEADBAND    0.02f     /* |u| below this = neutral */
#define ESC_DEFAULT_MAX_SPD_RPM 5000.0f   /* |u|=1.0 maps to this speed [RPM] */
#define ESC_DEFAULT_IQ_LIMIT_A  10.0f     /* Speed PI Iq clamp [A] */
#define ESC_TIMEOUT_MS          500U      /* No-command timeout → stop */
#define ESC_RESTART_DELAY_MS    500U      /* Back-off between auto-restarts */
#define ESC_TELEMETRY_EVERY     100U      /* Telemetry rate: 1000/100 = 10 Hz */
#define ESC_WRONG_ANGLE_RPM     50.0f     /* Speed sign mismatch threshold */

/* ── State ──────────��───────────────────────────────────────────────────── */
static ESC_State_t  esc_state       = ESC_BOOT;
static float        esc_max_spd_rpm = ESC_DEFAULT_MAX_SPD_RPM;
static float        esc_iq_limit_a  = ESC_DEFAULT_IQ_LIMIT_A;
static uint16_t     esc_timeout_ctr = 0U;
static uint16_t     esc_tlm_ctr     = 0U;
static uint16_t     esc_restart_dly = 0U;
static int8_t       esc_direction   = 0;      /* +1 forward, -1 reverse */
static uint8_t      esc_cl_entered  = 0U;     /* 1 once CFOC reaches CLOSED_LOOP */

/** Send one telemetry frame with current state. */
static void esc_send_telemetry(float u)
{
  float iq, id;
  CFOC_GetIqd(&iq, &id);

  int16_t  spd_rpm = (int16_t)CFOC_GetSpeedRPM();
  uint8_t  esc_st  = (uint8_t)esc_state;
  uint8_t  fault_b = (CFOC_GetState() == CFOC_FAULT) ? 0x01U : 0x00U;
  int16_t  cmd_raw = ESC_COMM_GetCommand();
  uint16_t vbus_v  = (uint16_t)(CFOC_GetVbusV() * 10.0f + 0.5f); /* tenths of V, e.g. 126 = 12.6V */
  int16_t  iq_ma   = (int16_t)(iq * 1000.0f);
  int16_t  id_ma   = (int16_t)(id * 1000.0f);
  uint8_t  cfoc_st = (uint8_t)CFOC_GetState();

  ESC_COMM_SendTelemetry(spd_rpm, esc_st, fault_b, cmd_raw, vbus_v,
                         iq_ma, id_ma, cfoc_st);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void ESC_APP_Init(void)
{
  ESC_COMM_Init();
  esc_state = ESC_BOOT;
}

void ESC_APP_Tick(void)
{
  /* ── Apply runtime config (0xCC frames from RPi5) ─────────────────── */
  {
    float v;
    v = ESC_COMM_GetMaxSpeedRPM();
    if (v > 0.0f) esc_max_spd_rpm = v;

    v = ESC_COMM_GetIqLimitA();
    if (v > 0.0f) { esc_iq_limit_a = v; CFOC_SetIqLimit(v); }

    CFOC_SetStartupParams(ESC_COMM_GetOlIqA(), ESC_COMM_GetOlRampMs(),
                          ESC_COMM_GetAlignMs(), ESC_COMM_GetAlignIdA());
  }

  /* ── Read command ─────��─────────────────────────────��──────────────── */
  uint8_t new_cmd = ESC_COMM_HasNewCommand();
  int16_t raw     = ESC_COMM_GetCommand();
  float   u       = (float)raw / 32767.0f;

  /* Timeout counter (active in FORWARD/REVERSE only) */
  if (new_cmd)
    esc_timeout_ctr = 0U;
  else if (esc_timeout_ctr < ESC_TIMEOUT_MS)
    esc_timeout_ctr++;
  uint8_t timed_out = (esc_timeout_ctr >= ESC_TIMEOUT_MS) ? 1U : 0U;

  /* Restart delay countdown */
  if (esc_restart_dly > 0U)
    esc_restart_dly--;

  /* Fault check: if CFOC entered FAULT (e.g. overcurrent), go to ESC_FAULT */
  if (esc_state != ESC_FAULT && CFOC_GetState() == CFOC_FAULT)
  {
    esc_state = ESC_FAULT;
  }

  /* ── State machine ────────────────────────────────────────────────── */
  switch (esc_state)
  {
  /* ------------------------------------------------------------------ */
  case ESC_BOOT:
    esc_state = ESC_WAIT_NEUTRAL;
    break;

  /* ------------------------------------------------------------------ */
  case ESC_WAIT_NEUTRAL:
    if (fabsf(u) < ESC_NEUTRAL_DEADBAND)
      esc_state = ESC_READY;
    break;

  /* ------------------------------------------------------------------ */
  case ESC_READY:
    if (new_cmd && esc_restart_dly == 0U)
    {
      if (u > ESC_NEUTRAL_DEADBAND)
      {
        esc_direction  = 1;
        esc_cl_entered = 0U;

        CFOC_Start(1);
        esc_state = ESC_FORWARD;
      }
      else if (u < -ESC_NEUTRAL_DEADBAND)
      {
        esc_direction  = -1;
        esc_cl_entered = 0U;

        CFOC_Start(-1);
        esc_state = ESC_REVERSE;
      }
    }
    break;

  /* ------------------------------------------------------------------ */
  case ESC_FORWARD:
  {
    CFOC_State_t cfoc_st = CFOC_GetState();

    /* Motor stopped unexpectedly (observer loss) → auto-restart */
    if (cfoc_st == CFOC_IDLE && esc_cl_entered)
    {
      esc_restart_dly = ESC_RESTART_DELAY_MS;
      esc_state = ESC_READY;
      break;
    }

    /* Timeout or neutral → stop */
    if (timed_out || (new_cmd && fabsf(u) < ESC_NEUTRAL_DEADBAND))
    {
      CFOC_Stop();
      esc_state = ESC_READY;
      break;
    }

    /* Direction reversal → brake */
    if (new_cmd && u < -ESC_NEUTRAL_DEADBAND)
    {
      CFOC_Stop();
      esc_state = ESC_BRAKE;
      break;
    }

    /* Wrong-angle detection: speed sign mismatch */
    if (esc_cl_entered && CFOC_GetSpeedRPM() < -ESC_WRONG_ANGLE_RPM)
    {
      CFOC_Stop();
      esc_restart_dly = ESC_RESTART_DELAY_MS;
      esc_state = ESC_READY;
      break;
    }

    /* Speed control once CFOC reaches CLOSED_LOOP */
    if (cfoc_st == CFOC_CLOSED_LOOP)
    {
      if (!esc_cl_entered)
        esc_cl_entered = 1U;

      CFOC_SetSpeed(u * esc_max_spd_rpm);
    }
    break;
  }

  /* ------------------------------------------------------------------ */
  case ESC_REVERSE:
  {
    CFOC_State_t cfoc_st = CFOC_GetState();

    /* Motor stopped unexpectedly → auto-restart */
    if (cfoc_st == CFOC_IDLE && esc_cl_entered)
    {
      esc_restart_dly = ESC_RESTART_DELAY_MS;
      esc_state = ESC_READY;
      break;
    }

    /* Timeout or neutral → stop */
    if (timed_out || (new_cmd && fabsf(u) < ESC_NEUTRAL_DEADBAND))
    {
      CFOC_Stop();
      esc_state = ESC_READY;
      break;
    }

    /* Direction reversal → brake */
    if (new_cmd && u > ESC_NEUTRAL_DEADBAND)
    {
      CFOC_Stop();
      esc_state = ESC_BRAKE;
      break;
    }

    /* Wrong-angle detection: speed sign mismatch (reverse expects negative RPM) */
    if (esc_cl_entered && CFOC_GetSpeedRPM() > ESC_WRONG_ANGLE_RPM)
    {
      CFOC_Stop();
      esc_restart_dly = ESC_RESTART_DELAY_MS;
      esc_state = ESC_READY;
      break;
    }

    /* Speed control once CFOC reaches CLOSED_LOOP */
    if (cfoc_st == CFOC_CLOSED_LOOP)
    {
      if (!esc_cl_entered)
        esc_cl_entered = 1U;

      CFOC_SetSpeed(u * esc_max_spd_rpm);  /* u < 0 → negative speed */
    }
    break;
  }

  /* ------------------------------------------------------------------ */
  case ESC_BRAKE:
    /* Wait for motor to stop, then dispatch */
    if (!CFOC_IsRunning())
    {
      if (new_cmd && u > ESC_NEUTRAL_DEADBAND)
      {
        esc_direction  = 1;
        esc_cl_entered = 0U;

        CFOC_Start(1);
        esc_state = ESC_FORWARD;
      }
      else if (new_cmd && u < -ESC_NEUTRAL_DEADBAND)
      {
        esc_direction  = -1;
        esc_cl_entered = 0U;

        CFOC_Start(-1);
        esc_state = ESC_REVERSE;
      }
      else
      {
        esc_state = ESC_READY;
      }
    }
    break;

  /* ------------------------------------------------------------------ */
  case ESC_FAULT:
    /* Attempt auto-ack. CFOC_AckFault transitions FAULT→IDLE. */
    CFOC_AckFault();
    if (CFOC_GetState() == CFOC_IDLE)
    {
      esc_state = ESC_WAIT_NEUTRAL;
    }
    break;
  }

  /* ── Telemetry (10 Hz) ────────────────────────────────────────────── */
  if (++esc_tlm_ctr >= ESC_TELEMETRY_EVERY)
  {
    esc_tlm_ctr = 0U;
    esc_send_telemetry(u);
  }
}

#endif /* BUILD_ESC */
