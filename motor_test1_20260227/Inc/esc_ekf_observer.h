/**
 ******************************************************************************
 * @file    esc_ekf_observer.h
 * @brief   Extended Kalman Filter sensorless observer for PMSM.
 *
 * State: x = [iα, iβ, eα, eβ]
 *   eα = -Ψf · ω_e · sin(θ_e),   eβ = Ψf · ω_e · cos(θ_e)
 *
 * Recovered outputs:
 *   θ_e = atan2f(-eα, eβ)
 *   ω_e = √(eα²+eβ²) / Ψf
 *
 * Design decisions (see docs/20260327_MCSDK_EKF_Plan.md §3-4):
 *   - Runs at 1 kHz (speed loop rate), NOT 25 kHz FOC rate.
 *   - float32 throughout (STM32G431 single-precision FPU).
 *   - F sparsity exploited: F×P avoids 48 zero-multiplies vs naive 4×4.
 *   - P stored as upper-triangle (10 floats), reducing writes.
 *   - Diagonal R: 2×2 S inverse costs 3 divides (not full matrix invert).
 *   - P update via P = P_pred − K×S×Kᵀ (numerically stable).
 *
 * Usage:
 *   EKF_Handle_t ekf;
 *   EKF_Init(&ekf, RS, LS, PSI_F, EKF_TS, q_i, q_e, r_i);
 *   // — call once per 1ms speed loop tick —
 *   EKF_Update(&ekf, Va, Vb, ia_measured, ib_measured);
 *   float theta = EKF_GetAngle(&ekf);       // electrical angle [rad]
 *   float rpm   = EKF_GetSpeedRPM(&ekf);    // mechanical RPM
 *
 * MCSDK integration (Step 5):
 *   Call EKF_Update() in the 1kHz speed loop hook in mc_app_hooks.c.
 *   Write EKF_GetAngle() → STO_PLL_M1.SPD_Handle.hElAngle (converted to
 *   MCSDK int16: hElAngle = (int16_t)(θ / (2π) * 65536)).
 *   Guard with USE_EKF_OBSERVER compile flag in drive_parameters.h.
 ******************************************************************************
 */

#ifndef ESC_EKF_OBSERVER_H
#define ESC_EKF_OBSERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Public handle ────────────────────────────────────────────────────────── */

/**
 * @brief EKF_Handle_t — full EKF state. Initialise with EKF_Init().
 *
 * P is stored as the upper triangle (10 elements) to halve memory writes.
 * Index mapping:  P[0..9] ↔ P00 P01 P02 P03 P11 P12 P13 P22 P23 P33
 */
typedef struct {
    float x[4];       /*!< State [iα, iβ, eα, eβ]                     */
    float P[10];      /*!< Covariance upper triangle (symmetric)        */
    float Rs;         /*!< Stator resistance [Ω]                        */
    float Ls;         /*!< Stator inductance [H]                        */
    float psi_f;      /*!< PM flux linkage [Wb]                         */
    float Ts;         /*!< EKF update period [s] (= 1e-3 at 1 kHz)     */
    /* Precomputed constants (computed in EKF_Init, zero runtime cost) */
    uint8_t p;        /*!< Pole pair count                              */
    float _a1;        /*!< 1 − Ts·Rs/Ls                                */
    float _b;         /*!< Ts/Ls                                        */
    float _inv_psi;   /*!< 1/Ψf                                         */
    float _inv_psi2;  /*!< 1/Ψf²                                        */
    float _rpm_scale; /*!< 60/(2π×p) — RPM conversion constant          */
    float _Q[4];      /*!< Process noise diagonal [q_i, q_i, q_e, q_e] */
    float _R;         /*!< Measurement noise (both channels equal) r_i  */
} EKF_Handle_t;

/* ── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  EKF_Init — initialise EKF handle.
 *
 * @param h     Pointer to uninitialised EKF_Handle_t.
 * @param Rs    Stator resistance [Ω]            (0.100 for 2852/3100KV)
 * @param Ls    Stator inductance [H]            (10e-6 for 2852/3100KV)
 * @param psi_f PM flux linkage [Wb]             (9.75e-4 for 2852/3100KV)
 * @param p     Pole pair count                  (2 for 2852/3100KV)
 * @param Ts    EKF update period [s]            (1e-3 at 1 kHz)
 * @param q_i   Process noise — current states  (3.33e-3 A², from noise_floor bag)
 * @param q_e   Process noise — BEMF states     (3.33e-2, 10× q_i)
 * @param r_i   Measurement noise — current     (3.33e-3 A², σ_I = 57 mA)
 */
void EKF_Init(EKF_Handle_t *h,
              float Rs, float Ls, float psi_f, uint8_t p, float Ts,
              float q_i, float q_e, float r_i);

/**
 * @brief  EKF_Update — one EKF predict+correct step.
 *
 * Call once per speed loop tick (1 kHz). Takes ~452 cycles on STM32G431
 * at 170 MHz with -O2 (analytical; measure with tests/ekf_timing_bench.c).
 *
 * @param h      EKF handle (must be initialised with EKF_Init).
 * @param Va     Applied stator α-axis voltage [V]
 * @param Vb     Applied stator β-axis voltage [V]
 * @param ia_m   Measured α-axis current [A]
 * @param ib_m   Measured β-axis current [A]
 */
void EKF_Update(EKF_Handle_t *h,
                float Va, float Vb,
                float ia_m, float ib_m);

/**
 * @brief  EKF_GetAngle — estimated rotor electrical angle.
 * @return θ_e in radians, range [−π, π].
 */
float EKF_GetAngle(const EKF_Handle_t *h);

/**
 * @brief  EKF_GetSpeedRPM — estimated mechanical speed.
 * @return Mechanical speed in RPM (always positive; sign from direction).
 */
float EKF_GetSpeedRPM(const EKF_Handle_t *h);

/**
 * @brief  EKF_GetAngleMCSdk — angle in MCSDK int16 format.
 *
 * Full circle = 65536 counts (S16_FULL_SCALE = 32767 = π rad).
 * Converts EKF_GetAngle() for direct write to hElAngle in SpeednPosFdbk_Handle_t.
 *
 * @return hElAngle value for STO_PLL_M1.SPD_Handle.hElAngle
 */
int16_t EKF_GetAngleMCSdk(const EKF_Handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* ESC_EKF_OBSERVER_H */
