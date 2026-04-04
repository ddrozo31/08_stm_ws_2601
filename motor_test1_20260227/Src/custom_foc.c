/**
 * @file    custom_foc.c
 * @brief   Custom FOC motor control — replaces MCSDK motor control stack.
 *
 * Step 2: Open-loop startup with PI current control + SVM.
 *   HF (25 kHz): ADC read → Clarke → Park → PI(Iq,Id) → InvPark → SVM → PWM
 *   MF (1 kHz):  Open-loop angle ramp, current reference ramp, state machine
 *
 * The motor spins in open-loop at a commanded speed. No observer feedback yet
 * (EKF integration is Step 3).
 */

#include "custom_foc.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdbool.h>

/* ── ADC handles (declared in main.c) ───────────────────────────────────── */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

/* ── Constants ──────────────────────────────────────────────────────────── */
#define INV_SQRT3   0.57735026919f   /* 1/√3 */
#define SQRT3       1.73205080757f
#define TWO_PI      6.28318530718f
#define PI_F        3.14159265359f

/* RPM → electrical rad/s: ω_e = RPM × 2π/60 × pole_pairs */
#define RPM_TO_ERAD_S  (TWO_PI / 60.0f * (float)CFOC_POLE_PAIRS)

/* ── Private state ───────────────────────────────────────────────────────── */
static volatile CFOC_State_t cfoc_state = CFOC_IDLE;

/* ADC offset calibration (zero-current mid-point, 12-bit).
 * Float for continuous EMA tracking — avoids integer truncation drift. */
static volatile float adc_offset_a = 2048.0f;
static volatile float adc_offset_b = 2048.0f;

/* Self-calibration accumulators (run inside ISR for first N samples) */
static volatile uint32_t calib_count = 0U;
static volatile int32_t  calib_sum_a = 0;
static volatile int32_t  calib_sum_b = 0;
static volatile uint8_t  calib_done  = 0U;

/* Measured currents in α-β frame [A], written by HF ISR */
static volatile float isr_Ialpha = 0.0f;
static volatile float isr_Ibeta  = 0.0f;

/* ISR cycle counter for diagnostics */
static volatile uint32_t isr_count = 0U;

/* ── Open-loop state (written by MF task, read by HF task) ──────────────── */
static volatile float ol_theta_e  = 0.0f;   /* Electrical angle [rad] */
static volatile float ol_omega_e  = 0.0f;   /* Electrical speed [rad/s] */
static volatile float ol_Iq_ref   = 0.0f;   /* q-axis current reference [A] */
static volatile float ol_Id_ref   = 0.0f;   /* d-axis current reference [A] */
static volatile int8_t ol_direction = 1;     /* +1 forward, -1 reverse */
static volatile uint32_t ol_ramp_ms = 0U;    /* MF tick counter since OPEN_LOOP entry */
static volatile uint32_t align_ms   = 0U;    /* MF tick counter during ALIGNMENT */

/* ── Voltage outputs for telemetry / future EKF feed ────────────────────── */
static volatile float isr_Valpha = 0.0f;
static volatile float isr_Vbeta  = 0.0f;

/* ── Debug: PI outputs and d-q currents (add to Live Expressions) ──────── */
static volatile float dbg_Vq = 0.0f;
static volatile float dbg_Vd = 0.0f;
static volatile float dbg_Iq = 0.0f;
static volatile float dbg_Id = 0.0f;
static volatile uint32_t dbg_ccr1 = 0U;
static volatile uint32_t dbg_ccr2 = 0U;
static volatile uint32_t dbg_ccr3 = 0U;
static volatile int32_t  dbg_raw_a = 0;
static volatile int32_t  dbg_raw_b = 0;

/* ── Debug log buffer ──────────────────────────────────────────────────── */
volatile CFOC_LogEntry_t cfoc_log[CFOC_LOG_SIZE];
volatile uint32_t cfoc_log_idx     = 0U;
volatile uint8_t  cfoc_log_running = 0U;
static volatile uint32_t log_start_tick = 0U;

/* ── PI controllers ─────────────────────────────────────────────────────── */
static CFOC_PI_t pi_iq = {
  .Kp = CFOC_PI_IQ_KP, .Ki = CFOC_PI_IQ_KI,
  .integral = 0.0f, .out_min = -CFOC_PI_VMAX, .out_max = CFOC_PI_VMAX
};
static CFOC_PI_t pi_id = {
  .Kp = CFOC_PI_ID_KP, .Ki = CFOC_PI_ID_KI,
  .integral = 0.0f, .out_min = -CFOC_PI_VMAX, .out_max = CFOC_PI_VMAX
};

/* ── Inline helpers ─────────────────────────────────────────────────────── */

/** Run PI controller with conditional-integration antiwindup.
 *  Ki is already discretized (Ki × Ts).
 *  Integration is frozen when the output saturates AND the error
 *  would push it further into saturation (same sign). */
static inline float PI_Run(CFOC_PI_t *pi, float error)
{
  /* Tentative output (before integration update) */
  float out = pi->Kp * error + pi->integral;

  /* Only integrate when NOT (saturated and error drives further) */
  bool sat_high = (out >= pi->out_max);
  bool sat_low  = (out <= pi->out_min);
  if (!(sat_high && error > 0.0f) && !(sat_low && error < 0.0f))
  {
    pi->integral += pi->Ki * error;
    if (pi->integral > pi->out_max) pi->integral = pi->out_max;
    if (pi->integral < pi->out_min) pi->integral = pi->out_min;
    out = pi->Kp * error + pi->integral;
  }

  /* Output clamp */
  if (out > pi->out_max) out = pi->out_max;
  if (out < pi->out_min) out = pi->out_min;
  return out;
}

/** Reset PI controller state. */
static inline void PI_Reset(CFOC_PI_t *pi)
{
  pi->integral = 0.0f;
}

/**
 * Space Vector Modulation — standard 7-segment center-aligned.
 * Input:  Valpha, Vbeta in volts.
 * Output: writes TIM1 CCR1/2/3 directly.
 *
 * Normalization: duty = 0.5 + V / Vbus  (Vbus read from ADC or assumed)
 * For now, assume Vbus ≈ 12V (3S LiPo nominal).
 */
static void SVM_Apply(float Valpha, float Vbeta)
{
  /* TODO: read actual Vbus from ADC regular channel.
   * For Step 2, use nominal 12V. */
  const float Vbus = 12.0f;
  const float inv_Vbus = 1.0f / Vbus;
  const float half_period = (float)CFOC_PWM_HALF_PERIOD;

  /* Inverse Clarke to get phase voltages (balanced 3-phase):
   *   Va = Valpha
   *   Vb = -0.5·Valpha + (√3/2)·Vbeta
   *   Vc = -0.5·Valpha - (√3/2)·Vbeta
   */
  float Va = Valpha;
  float Vb = -0.5f * Valpha + (SQRT3 * 0.5f) * Vbeta;
  float Vc = -0.5f * Valpha - (SQRT3 * 0.5f) * Vbeta;

  /* Normalize to [-0.5, 0.5] relative to Vbus */
  Va *= inv_Vbus;
  Vb *= inv_Vbus;
  Vc *= inv_Vbus;

  /* Min-max injection (SVPWM equivalent — centers the waveform) */
  float vmin = Va;
  if (Vb < vmin) vmin = Vb;
  if (Vc < vmin) vmin = Vc;
  float vmax = Va;
  if (Vb > vmax) vmax = Vb;
  if (Vc > vmax) vmax = Vc;
  float voffset = -(vmax + vmin) * 0.5f;

  Va += voffset;
  Vb += voffset;
  Vc += voffset;

  /* Convert to timer compare values: CCRx = (0.5 + Vx) × ARR
   * Clamp to [1, ARR-1] to keep dead-time valid */
  int32_t ccr_a = (int32_t)((0.5f + Va) * half_period);
  int32_t ccr_b = (int32_t)((0.5f + Vb) * half_period);
  int32_t ccr_c = (int32_t)((0.5f + Vc) * half_period);

  /* Clamp */
  if (ccr_a < 1) ccr_a = 1;
  if (ccr_a > (int32_t)CFOC_PWM_HALF_PERIOD - 1) ccr_a = (int32_t)CFOC_PWM_HALF_PERIOD - 1;
  if (ccr_b < 1) ccr_b = 1;
  if (ccr_b > (int32_t)CFOC_PWM_HALF_PERIOD - 1) ccr_b = (int32_t)CFOC_PWM_HALF_PERIOD - 1;
  if (ccr_c < 1) ccr_c = 1;
  if (ccr_c > (int32_t)CFOC_PWM_HALF_PERIOD - 1) ccr_c = (int32_t)CFOC_PWM_HALF_PERIOD - 1;

  dbg_ccr1 = (uint32_t)ccr_a;
  dbg_ccr2 = (uint32_t)ccr_b;
  dbg_ccr3 = (uint32_t)ccr_c;

  LL_TIM_OC_SetCompareCH1(TIM1, (uint32_t)ccr_a);
  LL_TIM_OC_SetCompareCH2(TIM1, (uint32_t)ccr_b);
  LL_TIM_OC_SetCompareCH3(TIM1, (uint32_t)ccr_c);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void CFOC_Init(void)
{
  /* Start OPAMPs (current sense amplifiers) */
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);

  /* Calibrate ADCs (internal offset calibration, leaves ADC disabled) */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  /* ── Enable ADCs with LL (bypass HAL state machine) ────────────────── */
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) { /* wait */ }
  LL_ADC_ClearFlag_ADRDY(ADC1);

  LL_ADC_Enable(ADC2);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC2)) { /* wait */ }
  LL_ADC_ClearFlag_ADRDY(ADC2);

  /* ── Configure injected sequence + trigger via LL ──────────────────── */
  /* ADC1: 2 ranks — ch3 (Phase U via OPAMP1), ch12 (Phase W via OPAMP3) */
  LL_ADC_INJ_ConfigQueueContext(ADC1,
      LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
      LL_ADC_INJ_TRIG_EXT_RISING,
      LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
      LL_ADC_CHANNEL_3,
      LL_ADC_CHANNEL_12,
      LL_ADC_CHANNEL_0, LL_ADC_CHANNEL_0);

  /* ADC2: 2 ranks — VOPAMP3 (Phase W), ch3 (Phase V via OPAMP2) */
  LL_ADC_INJ_ConfigQueueContext(ADC2,
      LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
      LL_ADC_INJ_TRIG_EXT_RISING,
      LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
      LL_ADC_CHANNEL_VOPAMP3_ADC2,
      LL_ADC_CHANNEL_3,
      LL_ADC_CHANNEL_0, LL_ADC_CHANNEL_0);

  /* Arm both ADCs for external trigger (JADSTART) */
  LL_ADC_INJ_StartConversion(ADC1);
  LL_ADC_INJ_StartConversion(ADC2);

  /* ── TIM1 setup ────────────────────────────────────────────────────── */
  /* Set all duty cycles to 50% (zero voltage in center-aligned mode) */
  LL_TIM_OC_SetCompareCH1(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH2(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH3(TIM1, CFOC_PWM_HALF_PERIOD / 2U);

  /* CC4 triggers ADC via TRGO=OC4REF.
   * CH4 is PWM2 mode: OC4REF=1 when counter >= CC4.
   * In center-aligned mode, counter counts 0→ARR→0.
   * We want to sample at the CENTER of the low-side ON-time,
   * which is when the counter is near ARR (top of triangle).
   * Set CC4 to (ARR - 1) so OC4REF rising edge triggers ADC
   * at the top, where at least 2 of 3 low-side FETs are ON
   * and shunt current is measurable.
   *
   * NOTE: With extreme duty cycles (CCR near 0 or near ARR),
   * one phase may not have a measurable window — that's a
   * Step 6 optimization (sector-dependent sampling). */
  LL_TIM_OC_SetCompareCH4(TIM1, CFOC_PWM_HALF_PERIOD - 1U);

  /* Enable PWM channel outputs (high-side + low-side for 3 phases) */
  LL_TIM_CC_EnableChannel(TIM1,
      LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
      LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
      LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);

  /* Enable TIM1 master output (MOE bit in BDTR) */
  LL_TIM_EnableAllOutputs(TIM1);

  /* Enable TIM1 update interrupt */
  LL_TIM_EnableIT_UPDATE(TIM1);

  /* Enable JEOS interrupt on ADC2 — the 25 kHz FOC ISR */
  LL_ADC_EnableIT_JEOS(ADC2);

  /* Start TIM1 counter */
  LL_TIM_EnableCounter(TIM1);

  cfoc_state = CFOC_IDLE;
}

void CFOC_HighFrequencyTask(void)
{
  /* ── 1. Read phase currents from injected ADC ────────────────────────── */
  /* ADC is left-aligned (12-bit << 4); shift right to get 0..4095.
   *
   * B-G431B-ESC1 current sensing assignment (from .ioc):
   *   Phase U → ADC1 CH3  (OPAMP1 output on PA2)
   *   Phase V → ADC2 CH3  (OPAMP2 output on PA6) — different ADC!
   *   Phase W → ADC1 CH12 (OPAMP3 output on PB1)
   *
   * We read Phase U (Ia) from ADC1 rank 1 and Phase V (Ib) from ADC2 rank 1.
   * Phase W = -(Ia + Ib), reconstructed implicitly by Clarke transform. */
  int32_t raw_a = (int32_t)(LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) >> 4);
  int32_t raw_b = (int32_t)(LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) >> 4);
  dbg_raw_a = raw_a;
  dbg_raw_b = raw_b;

  /* ── Self-calibration: bootstrap first N samples at zero current ────── */
  if (!calib_done)
  {
    calib_sum_a += raw_a;
    calib_sum_b += raw_b;
    calib_count++;

    if (calib_count >= CFOC_CALIB_SAMPLES)
    {
      adc_offset_a = (float)calib_sum_a / (float)CFOC_CALIB_SAMPLES;
      adc_offset_b = (float)calib_sum_b / (float)CFOC_CALIB_SAMPLES;
      calib_done = 1U;
    }
    isr_count++;
    return;
  }

  /* ── Continuous offset tracking: EMA while IDLE (zero current) ─────── */
  if (cfoc_state == CFOC_IDLE)
  {
    adc_offset_a += CFOC_OFFSET_EMA_ALPHA * ((float)raw_a - adc_offset_a);
    adc_offset_b += CFOC_OFFSET_EMA_ALPHA * ((float)raw_b - adc_offset_b);
  }

  /* ── 2. Convert to Amps ─────────────────────────────────────────────── */
  float Ia = (adc_offset_a - (float)raw_a) * CFOC_ADC_TO_AMPS;
  float Ib = (adc_offset_b - (float)raw_b) * CFOC_ADC_TO_AMPS;

  /* ── 3. Clarke transform: (Ia, Ib) → (Iα, Iβ) ──────────────────────── */
  float Ialpha = Ia;
  float Ibeta  = (Ia + 2.0f * Ib) * INV_SQRT3;

  /* Store for telemetry */
  isr_Ialpha = Ialpha;
  isr_Ibeta  = Ibeta;

  /* ── 4. State-dependent control ─────────────────────────────────────── */
  if (cfoc_state == CFOC_IDLE || cfoc_state == CFOC_FAULT)
  {
    /* Zero voltage output */
    LL_TIM_OC_SetCompareCH1(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
    LL_TIM_OC_SetCompareCH2(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
    LL_TIM_OC_SetCompareCH3(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
    isr_Valpha = 0.0f;
    isr_Vbeta  = 0.0f;
    isr_count++;
    return;
  }

  /* ── Determine angle and current references based on state ─────────── */
  float theta;
  float Iq_ref;
  float Id_ref;

  if (cfoc_state == CFOC_ALIGNMENT)
  {
    /* Hold θ=0, push Id to lock rotor to d-axis.
     * Ramp Id from 0 to CFOC_ALIGN_ID over first 100ms to avoid current spike. */
    theta  = 0.0f;
    Iq_ref = 0.0f;
    float align_frac = (float)align_ms / 100.0f;
    if (align_frac > 1.0f) align_frac = 1.0f;
    Id_ref = CFOC_ALIGN_ID * align_frac;
  }
  else  /* CFOC_OPEN_LOOP (and future CROSSFADE / CLOSED_LOOP) */
  {
    /* Integrate angle at HF rate (25 kHz) for smooth rotor tracking.
     * ω_e is set by MF task at 1 kHz (slow-changing ramp). */
    float omega = ol_omega_e;
    theta = ol_theta_e + omega * CFOC_TS;
    if (theta > PI_F)       theta -= TWO_PI;
    else if (theta < -PI_F) theta += TWO_PI;
    ol_theta_e = theta;

    Iq_ref = ol_Iq_ref;
    Id_ref = ol_Id_ref;
  }

  /* ── 5. Park transform: (Iα, Iβ) → (Id, Iq) using θ_e ────────────── */
  float sin_th, cos_th;
  sin_th = sinf(theta);
  cos_th = cosf(theta);

  float Id =  cos_th * Ialpha + sin_th * Ibeta;
  float Iq = -sin_th * Ialpha + cos_th * Ibeta;

  /* ── 6. PI current controllers ──────────────────────────────────────── */
  float Vd = PI_Run(&pi_id, Id_ref - Id);
  float Vq = PI_Run(&pi_iq, Iq_ref - Iq);

  /* Debug snapshot */
  dbg_Vq = Vq;
  dbg_Vd = Vd;
  dbg_Iq = Iq;
  dbg_Id = Id;

  /* ── 7. Circle limitation — keep |V| ≤ Vmax ────────────────────────── */
  float Vsq = Vd * Vd + Vq * Vq;
  float Vmax_sq = CFOC_PI_VMAX * CFOC_PI_VMAX;
  if (Vsq > Vmax_sq)
  {
    float scale = CFOC_PI_VMAX / sqrtf(Vsq);
    Vd *= scale;
    Vq *= scale;
  }

  /* ── 8. Inverse Park: (Vd, Vq) → (Vα, Vβ) ─────────────────────────── */
  float Valpha = cos_th * Vd - sin_th * Vq;
  float Vbeta  = sin_th * Vd + cos_th * Vq;

  /* Store for telemetry / future EKF feed */
  isr_Valpha = Valpha;
  isr_Vbeta  = Vbeta;

  /* ── 9. SVM → TIM1 CCR1/2/3 ────────────────────────────────────────── */
  SVM_Apply(Valpha, Vbeta);

  isr_count++;
}

void CFOC_MediumFrequencyTask(void)
{
  /* ── ALIGNMENT phase: wait for rotor to lock, then transition ──────── */
  if (cfoc_state == CFOC_ALIGNMENT)
  {
    align_ms++;
    if (align_ms >= CFOC_ALIGN_MS)
    {
      /* Rotor is aligned to θ=0 — begin open-loop ramp */
      ol_theta_e = 0.0f;
      ol_omega_e = 0.0f;
      ol_Iq_ref  = 0.0f;
      ol_ramp_ms = 0U;
      cfoc_state = CFOC_OPEN_LOOP;
    }
    goto log_sample;
  }

  if (cfoc_state != CFOC_OPEN_LOOP)
    goto log_sample;

  ol_ramp_ms++;

  /* ── Speed ramp: 0 → target RPM over CFOC_OL_RAMP_MS ──────────────── */
  float speed_frac = (float)ol_ramp_ms / (float)CFOC_OL_RAMP_MS;
  if (speed_frac > 1.0f) speed_frac = 1.0f;

  float target_rpm = CFOC_OL_TARGET_RPM * speed_frac;
  ol_omega_e = target_rpm * RPM_TO_ERAD_S * (float)ol_direction;

  /* Angle is integrated at 25 kHz in HF task (smooth rotor tracking) */

  /* ── Current ramp: 0 → Iq_target over CFOC_OL_IQ_RAMP_MS ──────────── */
  float iq_frac = (float)ol_ramp_ms / (float)CFOC_OL_IQ_RAMP_MS;
  if (iq_frac > 1.0f) iq_frac = 1.0f;

  ol_Iq_ref = CFOC_OL_IQ_TARGET * iq_frac * (float)ol_direction;
  ol_Id_ref = CFOC_OL_ID_REF;

log_sample:
  /* ── Record one log entry per MF tick while active ─────────────────── */
  if (cfoc_log_running && cfoc_log_idx < CFOC_LOG_SIZE)
  {
    uint32_t idx = cfoc_log_idx;
    cfoc_log[idx].tick_ms   = (uint16_t)(HAL_GetTick() - log_start_tick);
    cfoc_log[idx].state     = (uint8_t)cfoc_state;
    cfoc_log[idx].Iq_x100   = (int16_t)(dbg_Iq * 100.0f);
    cfoc_log[idx].Id_x100   = (int16_t)(dbg_Id * 100.0f);
    cfoc_log[idx].Vq_x100   = (int16_t)(dbg_Vq * 100.0f);
    cfoc_log[idx].Vd_x100   = (int16_t)(dbg_Vd * 100.0f);
    cfoc_log[idx].theta_x10 = (int16_t)(ol_theta_e * (1800.0f / PI_F)); /* rad→deg×10 */
    cfoc_log_idx = idx + 1U;
  }
}

CFOC_State_t CFOC_GetState(void)
{
  return cfoc_state;
}

void CFOC_Start(int8_t direction)
{
  if (cfoc_state != CFOC_IDLE)
    return;

  /* Reset all state */
  ol_theta_e  = 0.0f;
  ol_omega_e  = 0.0f;
  ol_Iq_ref   = 0.0f;
  ol_Id_ref   = 0.0f;
  ol_ramp_ms  = 0U;
  align_ms    = 0U;
  ol_direction = (direction >= 0) ? 1 : -1;

  /* Reset PI integrators */
  PI_Reset(&pi_iq);
  PI_Reset(&pi_id);

  /* Start debug log (overwrites previous capture) */
  cfoc_log_idx     = 0U;
  log_start_tick   = HAL_GetTick();
  cfoc_log_running = 1U;

  /* Start with alignment: hold θ=0, push Id to lock rotor position */
  cfoc_state = CFOC_ALIGNMENT;
}

void CFOC_Stop(void)
{
  /* Force 50% duty (zero voltage) */
  LL_TIM_OC_SetCompareCH1(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH2(TIM1, CFOC_PWM_HALF_PERIOD / 2U);
  LL_TIM_OC_SetCompareCH3(TIM1, CFOC_PWM_HALF_PERIOD / 2U);

  /* Reset PI integrators */
  PI_Reset(&pi_iq);
  PI_Reset(&pi_id);

  cfoc_state = CFOC_IDLE;
}

void CFOC_GetCurrents(float *ia, float *ib)
{
  if (ia) *ia = isr_Ialpha;
  if (ib) *ib = isr_Ibeta;
}

float CFOC_GetAngle(void)
{
  return ol_theta_e;
}

float CFOC_GetSpeedRPM(void)
{
  return ol_omega_e / RPM_TO_ERAD_S;
}
