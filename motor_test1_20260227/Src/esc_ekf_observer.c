/**
 ******************************************************************************
 * @file    esc_ekf_observer.c
 * @brief   Extended Kalman Filter sensorless observer for PMSM — Step 4.
 *
 * See Inc/esc_ekf_observer.h for API documentation and design decisions.
 * See docs/20260327_MCSDK_EKF_Plan.md for full mathematical derivation.
 * See tests/ekf_pmsm.py      for Python prototype (validated vs Luenberger).
 * See tests/ekf_opcount.py   for operation count and timing analysis.
 * See tests/ekf_timing_bench.c for DWT cycle measurement harness.
 ******************************************************************************
 */

#include "esc_ekf_observer.h"
#include <math.h>     /* sqrtf, atan2f */
#include <string.h>   /* memset */

/* ── Upper-triangle P index macros ────────────────────────────────────────── */
/* P[10] stores: P00 P01 P02 P03 P11 P12 P13 P22 P23 P33 */
#define IP00 0
#define IP01 1
#define IP02 2
#define IP03 3
#define IP11 4
#define IP12 5
#define IP13 6
#define IP22 7
#define IP23 8
#define IP33 9

/* ── EKF_Init ─────────────────────────────────────────────────────────────── */

void EKF_Init(EKF_Handle_t *h,
              float Rs, float Ls, float psi_f, uint8_t p, float Ts,
              float q_i, float q_e, float r_i)
{
    memset(h, 0, sizeof(*h));

    h->Rs    = Rs;
    h->Ls    = Ls;
    h->psi_f = psi_f;
    h->p     = p;
    h->Ts    = Ts;

    /* Precomputed scalars — computed once, zero runtime overhead in EKF_Update.
     *
     * Current dynamics: diα/dt = -(Rs/Ls)·iα + (Va-ea)/Ls
     * Time constant τ = Ls/Rs = 10μH/0.1Ω = 100 μs.
     *
     * Euler stability requires Ts·Rs/Ls < 2, i.e. Ts < 200 μs.
     * Run EKF at FOC rate (Ts=40μs): Ts·Rs/Ls = 0.4 → a1=0.6 (stable).
     * BEMF predict uses exact cosine rotation (ea_p = cos(ωTs)·ea − sin(ωTs)·eb)
     * to prevent magnitude drift: Euler grows ||BEMF|| by e^11 over 5s at 1600 RPM.
     *
     * Note: plan originally proposed 1kHz for timing conservatism.
     * Actual EKF cost: ~490 cycles (7.2% of 6800-cycle FOC budget) — feasible.
     * Integration point: call EKF_Update() inside the FOC ISR (mc_tasks_foc.c)
     * or in the 1kHz speed loop if τ_EKF << speed loop interval (not the case here). */
    h->_a1        = 1.0f - Ts * Rs / Ls;    /* Euler: valid for Ts << τ=Ls/Rs=100μs */
    h->_b         = Ts / Ls;
    h->_inv_psi   = 1.0f / psi_f;
    h->_inv_psi2  = 1.0f / (psi_f * psi_f);
    h->_rpm_scale = 60.0f / (2.0f * 3.14159265f * (float)p);

    /* Process noise */
    h->_Q[0] = q_i;
    h->_Q[1] = q_i;
    h->_Q[2] = q_e;
    h->_Q[3] = q_e;

    /* Measurement noise (both α and β channels equal) */
    h->_R = r_i;

    /* Initial covariance.
     * Current states (iα, iβ): ±1 A uncertainty at startup → P=1 A².
     * BEMF states (eα, eβ): completely unknown at startup (range ±~0.5 V),
     * set P=1e4 so Kalman gains for rows 2/3 are large → aggressive correction
     * from zero toward true BEMF within the first few FOC cycles.
     * Small P (1.0) for BEMF → gains near zero → filter stuck at eα=eβ=0 → no
     * convergence → EKF_GetSpeedRPM returns ~0 throughout Phase 5. */
    h->P[IP00] = 1.0f;
    h->P[IP11] = 1.0f;
    h->P[IP22] = 1.0e4f;  /* eα: large init uncertainty → fast convergence */
    h->P[IP33] = 1.0e4f;  /* eβ: large init uncertainty → fast convergence */
    /* Off-diagonal elements already zero from memset */
}

/* ── EKF_Update ───────────────────────────────────────────────────────────── */

void EKF_Update(EKF_Handle_t *h,
                float Va, float Vb,
                float ia_m, float ib_m)
{
    /* Load state */
    float ia = h->x[0], ib = h->x[1], ea = h->x[2], eb = h->x[3];

    /* Load precomputed scalars */
    const float a1       = h->_a1;
    const float b        = h->_b;
    const float inv_psi2 = h->_inv_psi2;
    const float inv_psi  = h->_inv_psi;

    /* ── 1. Electrical speed from current BEMF estimate ────────────────── */
    /* Floor omega_e at 10 rad/s (~48 mech RPM for p=2) to keep the
     * Jacobian term d = Ts/(Ψf²·ω_e) bounded.  Without this, omega_e ≈ 0
     * at startup makes d → Inf → NaN propagation through P and K. */
    float omega_e = sqrtf(ea * ea + eb * eb) * inv_psi;
    if (omega_e < 10.0f) omega_e = 10.0f;

    /* ── 2. Exact BEMF rotation (mandatory even at Ts=40μs) ─────────────────
     *
     * Euler BEMF: ||BEMF||² grows by (1+(ωTs)²) per step.
     * At 1600 RPM (ωe=335 rad/s), Ts=40μs:  ωTs=0.0134 rad.
     * Over 5s (125000 steps): total growth ≈ e^(8.98e-5 × 125000) = e^11.2 ≈ 74000× → NaN.
     * Use exact rotation: ||BEMF|| conserved exactly (no magnitude drift).
     * Cost: one sincosf call (~25 cycles on Cortex-M4) — still well within budget.
     */
    float oTs   = omega_e * h->Ts;
    float c_rot = cosf(oTs);
    float s_rot = sinf(oTs);

    /* ── 3. Predicted state ──────────────────────────────────────────────── */
    float ia_p = a1 * ia + b * (Va - ea);
    float ib_p = a1 * ib + b * (Vb - eb);
    float ea_p = c_rot * ea - s_rot * eb;   /* exact rotation — magnitude preserved */
    float eb_p = s_rot * ea + c_rot * eb;

    /* ── 4. Jacobian F (nonzero/non-one elements only) ──────────────────── */
    /*
     * F = [ a1   0   -b    0  ]
     *     [  0  a1    0   -b  ]
     *     [  0   0  f22  f23  ]
     *     [  0   0  f32  f33  ]
     *
     * Exact-rotation Jacobian (linearisation of ea_p = cos(ω(ea,eb)·Ts)·ea − sin(…)·eb):
     *   d = Ts / (Ψf² × ω_e)   — same factor as Euler case
     *   F[2,2] = c_rot − d·eα·eb_p
     *   F[2,3] = −s_rot − d·eβ·eb_p
     *   F[3,2] =  s_rot + d·eα·ea_p
     *   F[3,3] =  c_rot + d·eβ·ea_p
     * Reduces to Euler Jacobian when ωTs→0 and ea_p≈ea, eb_p≈eb.
     */
    float d   = h->Ts * inv_psi2 / omega_e;
    float f22 =  c_rot - d * ea * eb_p;
    float f33 =  c_rot + d * eb * ea_p;
    float f23 = -s_rot - d * eb * eb_p;
    float f32 =  s_rot + d * ea * ea_p;

    /* ── 5. P_pred = F × P × Fᵀ + Q  (exploit F sparsity) ──────────────── */
    /*
     * F sparsity means F×P (4×4) can be computed as linear combinations of
     * P rows using only 2 nonzero coefficients per row.
     * P is symmetric; load upper triangle.
     */
    float p00 = h->P[IP00], p01 = h->P[IP01], p02 = h->P[IP02], p03 = h->P[IP03];
    float                    p11 = h->P[IP11], p12 = h->P[IP12], p13 = h->P[IP13];
    float                                      p22 = h->P[IP22], p23 = h->P[IP23];
    float                                                          p33 = h->P[IP33];

    /* FP = F × P */
    /* Row 0: a1·P[0,:] − b·P[2,:] */
    float fp00 = a1 * p00 - b * p02;
    float fp01 = a1 * p01 - b * p12;
    float fp02 = a1 * p02 - b * p22;
    float fp03 = a1 * p03 - b * p23;
    /* Row 1: a1·P[1,:] − b·P[3,:]   (P symmetric: P[1,0]=p01, P[3,0]=p03, ...) */
    float fp10 = a1 * p01 - b * p03;
    float fp11 = a1 * p11 - b * p13;
    float fp12 = a1 * p12 - b * p23;
    float fp13 = a1 * p13 - b * p33;
    /* Row 2: f22·P[2,:] + f23·P[3,:] */
    float fp20 = f22 * p02 + f23 * p03;
    float fp21 = f22 * p12 + f23 * p13;
    float fp22 = f22 * p22 + f23 * p23;
    float fp23 = f22 * p23 + f23 * p33;
    /* Row 3: f32·P[2,:] + f33·P[3,:] */
    float fp30 = f32 * p02 + f33 * p03;
    float fp31 = f32 * p12 + f33 * p13;
    float fp32 = f32 * p22 + f33 * p23;
    float fp33 = f32 * p23 + f33 * p33;

    /*
     * P_pred = (FP) × Fᵀ + Q
     *
     * Fᵀ col j = F row j:
     *   col 0: [a1, 0,   0,   0 ]   col 1: [0, a1,  0,   0 ]
     *   col 2: [ 0, 0, f22, f23 ]   col 3: [0,  0, f32, f33]
     *
     * P_pred[i][j] = Σ_k (FP)[i][k] × Fᵀ[k][j] = Σ_k (FP)[i][k] × F[j][k]
     *
     * Result is symmetric — compute upper triangle only.
     * Off-diagonal fp rows (fp10, fp20, fp21, fp30, fp31) are not needed
     * because Fᵀ col 2 and col 3 have zeros in positions 0 and 1.
     */
    float pp00 = fp00 * a1 - fp02 * b                        + h->_Q[0];
    float pp01 = fp01 * a1 - fp03 * b;
    float pp02 =             fp02 * f22 + fp03 * f23;
    float pp03 =             fp02 * f32 + fp03 * f33;
    float pp11 = fp11 * a1 - fp13 * b                        + h->_Q[1];
    float pp12 =             fp12 * f22 + fp13 * f23;
    float pp13 =             fp12 * f32 + fp13 * f33;
    float pp22 =             fp22 * f22 + fp23 * f23         + h->_Q[2];
    float pp23 =             fp22 * f32 + fp23 * f33;
    float pp33 =             fp32 * f32 + fp33 * f33         + h->_Q[3];

    /* fp10, fp20, fp21, fp30, fp31 computed but not needed (Fᵀ col 2,3 have zeros there) */
    (void)fp10; (void)fp20; (void)fp21; (void)fp30; (void)fp31;
    (void)oTs;

    /* ── 6. Innovation ───────────────────────────────────────────────────── */
    float yi0 = ia_m - ia_p;
    float yi1 = ib_m - ib_p;

    /* ── 7. S = H×P_pred×Hᵀ + R  (H extracts rows 0,1 → top-left 2×2) ─── */
    float s00 = pp00 + h->_R;
    float s01 = pp01;           /* S is symmetric: s01 = s10 */
    float s11 = pp11 + h->_R;

    /* ── 8. S⁻¹ (2×2 symmetric) ─────────────────────────────────────────── */
    float det   = s00 * s11 - s01 * s01;
    float inv_d = 1.0f / det;
    float si00  =  s11 * inv_d;
    float si01  = -s01 * inv_d;
    float si11  =  s00 * inv_d;

    /* ── 9. Kalman gain K = P_pred×Hᵀ × S⁻¹  (4×2 matrix) ──────────────── */
    /* P_pred×Hᵀ extracts first 2 cols of P_pred (no multiply needed). */
    float k00 = pp00 * si00 + pp01 * si01;
    float k01 = pp00 * si01 + pp01 * si11;
    float k10 = pp01 * si00 + pp11 * si01;
    float k11 = pp01 * si01 + pp11 * si11;
    float k20 = pp02 * si00 + pp12 * si01;
    float k21 = pp02 * si01 + pp12 * si11;
    float k30 = pp03 * si00 + pp13 * si01;
    float k31 = pp03 * si01 + pp13 * si11;

    /* ── 10. State update: x = x_pred + K × y_innov ─────────────────────── */
    h->x[0] = ia_p + k00 * yi0 + k01 * yi1;
    h->x[1] = ib_p + k10 * yi0 + k11 * yi1;
    h->x[2] = ea_p + k20 * yi0 + k21 * yi1;
    h->x[3] = eb_p + k30 * yi0 + k31 * yi1;

    /* ── 11. Covariance update: P = P_pred − K × S × Kᵀ ─────────────────── */
    /*
     * Numerically stable form: P = P_pred − (KS)×Kᵀ
     * Avoids (I-KH)×P_pred which amplifies rounding errors.
     * K×S: 4×2 × 2×2 → 4×2
     */
    float ks00 = k00 * s00 + k01 * s01;   float ks01 = k00 * s01 + k01 * s11;
    float ks10 = k10 * s00 + k11 * s01;   float ks11 = k10 * s01 + k11 * s11;
    float ks20 = k20 * s00 + k21 * s01;   float ks21 = k20 * s01 + k21 * s11;
    float ks30 = k30 * s00 + k31 * s01;   float ks31 = k30 * s01 + k31 * s11;

    /* (KS)×Kᵀ upper triangle: [(KS)[i,:]] · [K[j,:]] for j ≥ i */
    h->P[IP00] = pp00 - (ks00 * k00 + ks01 * k01);
    h->P[IP01] = pp01 - (ks00 * k10 + ks01 * k11);
    h->P[IP02] = pp02 - (ks00 * k20 + ks01 * k21);
    h->P[IP03] = pp03 - (ks00 * k30 + ks01 * k31);
    h->P[IP11] = pp11 - (ks10 * k10 + ks11 * k11);
    h->P[IP12] = pp12 - (ks10 * k20 + ks11 * k21);
    h->P[IP13] = pp13 - (ks10 * k30 + ks11 * k31);
    h->P[IP22] = pp22 - (ks20 * k20 + ks21 * k21);
    h->P[IP23] = pp23 - (ks20 * k30 + ks21 * k31);
    h->P[IP33] = pp33 - (ks30 * k30 + ks31 * k31);

    /* Diagonal clamping — prevents floating-point round-off from making P
     * non-positive-definite in the simplified P=P_pred-KSKᵀ form.
     * Clamp to Q (process noise) as the minimum reasonable variance.
     * Standard practice in embedded EKF (used in PX4/ArduPilot EKF2). */
    if (h->P[IP00] < h->_Q[0]) h->P[IP00] = h->_Q[0];
    if (h->P[IP11] < h->_Q[1]) h->P[IP11] = h->_Q[1];
    if (h->P[IP22] < h->_Q[2]) h->P[IP22] = h->_Q[2];
    if (h->P[IP33] < h->_Q[3]) h->P[IP33] = h->_Q[3];
}

/* ── Output functions ─────────────────────────────────────────────────────── */

float EKF_GetAngle(const EKF_Handle_t *h)
{
    return atan2f(-h->x[2], h->x[3]);
}

float EKF_GetSpeedRPM(const EKF_Handle_t *h)
{
    float ea = h->x[2], eb = h->x[3];
    /* ω_e = √(eα²+eβ²)/Ψf [elec rad/s],  RPM_mech = ω_e × 60/(2π×p) */
    return sqrtf(ea * ea + eb * eb) * h->_inv_psi * h->_rpm_scale;
}

int16_t EKF_GetAngleMCSdk(const EKF_Handle_t *h)
{
    /* MCSDK convention: full circle = 65536 counts, S16_FULL_SCALE = 32767 = π */
    float theta = EKF_GetAngle(h);                  /* range [−π, π] */
    return (int16_t)(theta * (32767.0f / 3.14159265f));
}
