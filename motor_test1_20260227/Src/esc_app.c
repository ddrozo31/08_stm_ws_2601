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
#include "drive_parameters.h"   /* USE_ADAPTIVE_R_EKF */
#include <math.h>

/* ── Constants (compile-time defaults, overridable via 0xCC config) ─────── */
#define ESC_NEUTRAL_DEADBAND    0.02f     /* |u| below this = neutral */
#define ESC_DEFAULT_MAX_SPD_RPM 5000.0f   /* |u|=1.0 maps to this speed [RPM] */
#define ESC_DEFAULT_IQ_LIMIT_A  10.0f     /* Speed PI Iq clamp [A] */
#define ESC_TIMEOUT_MS          500U      /* No-command timeout → stop */
#define ESC_RESTART_DELAY_MS    500U      /* Back-off between auto-restarts */
#define ESC_TELEMETRY_EVERY     100U      /* Telemetry rate: 1000/100 = 10 Hz */
#define ESC_WRONG_ANGLE_RPM     50.0f     /* Speed sign mismatch threshold */

/* ── State ───────────────────────────────────────────────────────────────── */
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
#if USE_ADAPTIVE_R_EKF
  /* Diagnostic: bit1 = swap_deferred, bits[2..7] = cl_hysteresis_ms (0-50) */
  if (CFOC_GetSwapDeferred()) fault_b |= 0x02U;
  {
    uint32_t hyst = CFOC_GetClHysteresisMs();
    if (hyst > 63U) hyst = 63U;
    fault_b |= (uint8_t)(hyst << 2);
  }
#endif
  int16_t  cmd_raw = ESC_COMM_GetCommand();
  uint16_t vbus_v  = (uint16_t)(CFOC_GetVbusV() * 10.0f + 0.5f); /* tenths of V, e.g. 126 = 12.6V */
  int16_t  iq_ma   = (int16_t)(iq * 1000.0f);
  int16_t  id_ma   = (int16_t)(id * 1000.0f);
  uint8_t  cfoc_st = (uint8_t)CFOC_GetState();
  float    innov_a = CFOC_GetInnovMag();
  float    inv_scaled = innov_a * 1000.0f;
  if (inv_scaled < 0.0f)       inv_scaled = 0.0f;
  if (inv_scaled > 65535.0f)   inv_scaled = 65535.0f;
  uint16_t innov_x1000 = (uint16_t)inv_scaled;

  /* Telemetry fields 16–19 have two semantics, selected at runtime:
   *   observer_mode == 0 (legacy)   : [kappa_x10000][res_x1000]
   *   observer_mode == 1 (adaptive) : [R_x1000      ][bemf_mag_x100]
   * The frame length and checksum layout are identical; only the meaning
   * changes. The node-side decoder picks by the same observer-mode bit
   * (or by inspecting whether the adaptive 0xCC params have been sent). */
  uint16_t tlm_f16f17;  /* frame bytes 16+17 LE: κ_x10000 or R_x1000 */
  uint16_t tlm_f18f19;  /* frame bytes 18+19 LE: r_x1000 or |e|_x100 */
#if USE_ADAPTIVE_R_EKF
  if (CFOC_GetObserverMode() == 1U)
  {
    float    R_now    = CFOC_GetEkfR();
    float    R_scaled = R_now * 1000.0f;
    if (R_scaled < 0.0f)     R_scaled = 0.0f;
    if (R_scaled > 65535.0f) R_scaled = 65535.0f;
    tlm_f16f17 = (uint16_t)R_scaled;

    float    emag     = CFOC_GetEkfBemfMag();
    float    e_scaled = emag * 100.0f;
    if (e_scaled < 0.0f)     e_scaled = 0.0f;
    if (e_scaled > 65535.0f) e_scaled = 65535.0f;
    tlm_f18f19 = (uint16_t)e_scaled;
  }
  else
#endif
  {
    float    kappa    = CFOC_GetLockKappa();
    float    k_scaled = kappa * 10000.0f;
    if (k_scaled < 0.0f)     k_scaled = 0.0f;
    if (k_scaled > 65535.0f) k_scaled = 65535.0f;
    tlm_f16f17 = (uint16_t)k_scaled;

    float    residual = CFOC_GetLockResidual();
    float    r_scaled = residual * 1000.0f;
    if (r_scaled < 0.0f)     r_scaled = 0.0f;
    if (r_scaled > 65535.0f) r_scaled = 65535.0f;
    tlm_f18f19 = (uint16_t)r_scaled;
  }

  ESC_COMM_SendTelemetry(spd_rpm, esc_st, fault_b, cmd_raw, vbus_v,
                         iq_ma, id_ma, cfoc_st, innov_x1000,
                         tlm_f16f17, tlm_f18f19);
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

    CFOC_SetCrossfadeParams(ESC_COMM_GetXfDurationMs(),
                            ESC_COMM_GetXfDwellMs(),
                            ESC_COMM_GetOlTargetRPM());

    CFOC_SetSpeedPIParams(ESC_COMM_GetSpdKp(),
                          ESC_COMM_GetSpdKi(),
                          ESC_COMM_GetSpdLpfAlpha());

#if USE_ADAPTIVE_R_EKF
    /* Step 8 adaptive-R EKF parameters (0xCC 0x14–0x18).
     * Host leaves unset → getters return 0xFF / -1 → CFOC setter keeps
     * its compile-time defaults (mode=0 = discrete/legacy behaviour). */
    CFOC_SetAdaptiveREkfParams(ESC_COMM_GetObserverMode(),
                               ESC_COMM_GetEkfOmegaThreshRad(),
                               ESC_COMM_GetEkfR0(),
                               ESC_COMM_GetEkfQe(),
                               ESC_COMM_GetEkfVfPriorLock());
#endif
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
