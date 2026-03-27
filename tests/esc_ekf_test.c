/**
 * @file  esc_ekf_test.c
 * @brief Step 4 unit tests for esc_ekf_observer.c
 *
 * Tests:
 *   T1 — EKF_Init: state and P initialised correctly, precomputed constants correct.
 *   T2 — No NaN: after 10 000 update steps with zero inputs, no NaN in x or P.
 *   T3 — Step response: from x=0, apply steady-state BEMF voltage → EKF converges.
 *   T4 — Speed recovery: EKF_GetSpeedRPM matches known ω after convergence.
 *   T5 — Angle recovery: EKF_GetAngle within ±15° of true angle after convergence.
 *   T6 — EKF_GetAngleMCSdk: int16 output consistent with float angle.
 *   T7 — P stays positive-definite throughout (all diagonal elements > 0).
 *   T8 — EKF is stable under large current noise (σ = 1.0 A, 10× normal).
 *
 * Build (host):
 *   gcc -O2 -lm -I../motor_test1_20260227/Inc \
 *       esc_ekf_test.c ../motor_test1_20260227/Src/esc_ekf_observer.c \
 *       -o /tmp/esc_ekf_test && /tmp/esc_ekf_test
 *
 * All tests must pass (exit 0). Each failure prints a diagnostic and exits 1.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "esc_ekf_observer.h"

/* ── Motor constants (2852/3100KV, from pmsm_motor_parameters.h) ─────────── */
#define M_RS        0.100f
#define M_LS        10e-6f
#define M_PSI_F     9.75e-4f
#define M_P         2
#define M_TS        40e-6f     /* EKF at FOC rate (25 kHz) — Euler stable for Ts << τ=100μs */
#define M_QI        3.33e-3f
#define M_QE        3.33e-2f
#define M_RI        3.33e-3f

/* ── Test framework ─────────────────────────────────────────────────────── */

static int g_tests_run = 0;
static int g_tests_pass = 0;

#define ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (!(cond)) { \
        printf("  FAIL [%s:%d] %s\n", __FILE__, __LINE__, msg); \
        exit(1); \
    } else { \
        g_tests_pass++; \
    } \
} while(0)

#define ASSERT_NEAR(a, b, tol, msg) \
    ASSERT(fabsf((a) - (b)) < (tol), msg)

#define TEST(name) printf("[%s]\n", name)
#define PASS(name) printf("  pass: %s\n\n", name)

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static float wrap_pi(float a) { return a - 2.0f * 3.14159265f * floorf((a + 3.14159265f) / (2.0f * 3.14159265f)); }
static float angle_err_deg(float est_rad, float true_rad) { return fabsf(wrap_pi(est_rad - true_rad)) * (180.0f / 3.14159265f); }

/**
 * Simple open-loop PMSM simulator (αβ forward Euler, Ts_foc=40μs).
 * Used to generate voltage/current pairs for the EKF to track.
 */
typedef struct { float theta_e, omega_m, ia, ib; } PMSM_t;

static void pmsm_step(PMSM_t *m, float Va, float Vb, float Ts_foc)
{
    float we = M_P * m->omega_m;
    float ea = -M_PSI_F * we * sinf(m->theta_e);
    float eb =  M_PSI_F * we * cosf(m->theta_e);
    m->ia     += Ts_foc * (Va - M_RS * m->ia - ea) / M_LS;
    m->ib     += Ts_foc * (Vb - M_RS * m->ib - eb) / M_LS;
    /* Simplified mechanics: fixed angular acceleration → ramp to target */
    m->omega_m += Ts_foc * 200.0f;    /* 200 rad/s² */
    float rpm_limit = 1600.0f;
    float wm_limit  = rpm_limit * 2.0f * 3.14159265f / 60.0f;
    if (m->omega_m > wm_limit) m->omega_m = wm_limit;
    m->theta_e = fmodf(m->theta_e + (float)M_P * m->omega_m * Ts_foc, 2.0f * 3.14159265f);
}

/* ── Tests ─────────────────────────────────────────────────────────────────  */

static void test_t1_init(void)
{
    TEST("T1: EKF_Init — state and precomputed constants");
    EKF_Handle_t h;
    EKF_Init(&h, M_RS, M_LS, M_PSI_F, M_P, M_TS, M_QI, M_QE, M_RI);

    /* State should be zero */
    ASSERT(h.x[0] == 0.0f && h.x[1] == 0.0f &&
           h.x[2] == 0.0f && h.x[3] == 0.0f, "initial state zero");

    /* P diagonal should be 1 (large initial uncertainty) */
    ASSERT(h.P[0] == 1.0f, "P[IP00] = 1");
    ASSERT(h.P[4] == 1.0f, "P[IP11] = 1");
    ASSERT(h.P[7] == 1.0f, "P[IP22] = 1");
    ASSERT(h.P[9] == 1.0f, "P[IP33] = 1");

    /* Precomputed scalars — Euler (valid at Ts=40μs << τ=Ls/Rs=100μs) */
    ASSERT_NEAR(h._a1,  1.0f - M_TS * M_RS / M_LS, 1e-4f, "_a1 = 1 - Ts*Rs/Ls");
    ASSERT_NEAR(h._b,   M_TS / M_LS,                1e-4f, "_b = Ts/Ls");
    ASSERT_NEAR(h._inv_psi,  1.0f / M_PSI_F,        1.0f,  "_inv_psi");
    ASSERT_NEAR(h._inv_psi2, 1.0f / (M_PSI_F * M_PSI_F), 1e4f, "_inv_psi2");
    float rpm_scale_expected = 60.0f / (2.0f * 3.14159265f * (float)M_P);
    ASSERT_NEAR(h._rpm_scale, rpm_scale_expected, 1e-3f, "_rpm_scale");
    ASSERT(h.p == M_P, "pole pairs");

    PASS("T1");
}

static void test_t2_no_nan(void)
{
    TEST("T2: No NaN after 10000 steps with near-zero inputs");
    EKF_Handle_t h;
    EKF_Init(&h, M_RS, M_LS, M_PSI_F, M_P, M_TS, M_QI, M_QE, M_RI);

    for (int i = 0; i < 10000; i++) {
        EKF_Update(&h, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    for (int i = 0; i < 4; i++) ASSERT(!isnanf(h.x[i]), "x[i] not NaN");
    for (int i = 0; i < 10; i++) ASSERT(!isnanf(h.P[i]), "P[i] not NaN");
    PASS("T2");
}

static void test_t3_step_response(void)
{
    TEST("T3: EKF converges from zero state under open-loop rev-up");
    EKF_Handle_t h;
    EKF_Init(&h, M_RS, M_LS, M_PSI_F, M_P, M_TS, M_QI, M_QE, M_RI);

    /* Simulate a simple open-loop ramp: impose stator field at 1600 RPM */
    float omega_e_ref = 1600.0f * 2.0f * 3.14159265f / 60.0f * (float)M_P;
    float theta_ref   = 0.0f;
    float ia_steady   = 8.0f;   /* A */
    int converged = 0;

    /* At Ts=40μs: 125000 steps = 5s total, check after 1s (25000 steps).
     * Require 12500 consecutive convergent steps = 0.5s sustained. */
    int total_steps = (int)(5.0f / M_TS);
    int check_after = (int)(1.0f / M_TS);
    int check_win   = (int)(0.5f / M_TS);
    for (int step = 0; step < total_steps; step++) {
        /* Advance reference angle at target speed */
        theta_ref = fmodf(theta_ref + omega_e_ref * M_TS, 2.0f * 3.14159265f);

        /* Steady-state αβ voltage for Id=0, Iq=ia_steady:
         *   Va = Rs·Iq·cos(θ) − ωe·(Ls·Iq + Ψf)·sin(θ)  ← includes BEMF
         *   Vb = Rs·Iq·sin(θ) + ωe·(Ls·Iq + Ψf)·cos(θ)
         * Omitting BEMF (−Ψf·ωe·sin(θ)) would make i/V inconsistent. */
        float Ls_Iq_psi = M_LS * ia_steady + M_PSI_F;
        float Va = M_RS * ia_steady * cosf(theta_ref) - omega_e_ref * Ls_Iq_psi * sinf(theta_ref);
        float Vb = M_RS * ia_steady * sinf(theta_ref) + omega_e_ref * Ls_Iq_psi * cosf(theta_ref);
        float ia_m = ia_steady * cosf(theta_ref);
        float ib_m = ia_steady * sinf(theta_ref);

        EKF_Update(&h, Va, Vb, ia_m, ib_m);

        if (step > check_after) {
            float err = angle_err_deg(EKF_GetAngle(&h), theta_ref);
            if (err < 15.0f) converged++;
        }
    }
    ASSERT(converged > check_win, "EKF converged within 15° for > 0.5s sustained");
    PASS("T3");
}

static void test_t4_speed_recovery(void)
{
    TEST("T4: EKF_GetSpeedRPM within 10% of true RPM after convergence");
    EKF_Handle_t h;
    EKF_Init(&h, M_RS, M_LS, M_PSI_F, M_P, M_TS, M_QI, M_QE, M_RI);

    float target_rpm  = 1600.0f;
    float omega_e_ref = target_rpm * 2.0f * 3.14159265f / 60.0f * (float)M_P;
    float theta_ref   = 0.0f;
    float ia_steady   = 8.0f;

    /* Run for 5 s = 125000 steps at Ts=40μs. Include BEMF in voltage. */
    int total_steps = (int)(5.0f / M_TS);
    for (int step = 0; step < total_steps; step++) {
        theta_ref = fmodf(theta_ref + omega_e_ref * M_TS, 2.0f * 3.14159265f);
        float Ls_Iq_psi = M_LS * ia_steady + M_PSI_F;
        float Va = M_RS * ia_steady * cosf(theta_ref) - omega_e_ref * Ls_Iq_psi * sinf(theta_ref);
        float Vb = M_RS * ia_steady * sinf(theta_ref) + omega_e_ref * Ls_Iq_psi * cosf(theta_ref);
        float ia_m = ia_steady * cosf(theta_ref);
        float ib_m = ia_steady * sinf(theta_ref);
        EKF_Update(&h, Va, Vb, ia_m, ib_m);
    }

    float rpm_est = EKF_GetSpeedRPM(&h);
    float rpm_err = fabsf(rpm_est - target_rpm) / target_rpm;
    printf("  RPM true=%.1f  est=%.1f  err=%.1f%%\n", target_rpm, rpm_est, rpm_err * 100.0f);
    ASSERT(rpm_err < 0.10f, "speed within 10% of truth");
    PASS("T4");
}

static void test_t5_angle_recovery(void)
{
    TEST("T5: EKF_GetAngle within 15° of true angle (PMSM simulator)");
    EKF_Handle_t h;
    EKF_Init(&h, M_RS, M_LS, M_PSI_F, M_P, M_TS, M_QI, M_QE, M_RI);

    PMSM_t plant;
    memset(&plant, 0, sizeof(plant));

    /* EKF now runs at FOC rate (Ts=40μs = M_TS), so one EKF per plant step.
     * 5s = 125000 steps. Check convergence after 1s (25000 steps). */
    int total_steps = (int)(5.0f / M_TS);
    int check_after = (int)(1.0f / M_TS);
    int check_win   = (int)(0.5f / M_TS);
    int converged = 0;

    for (int step = 0; step < total_steps; step++) {
        float omega_e = (float)M_P * plant.omega_m;
        float Iq = 8.0f;
        float Ls_Iq_psi = M_LS * Iq + M_PSI_F;
        float Va = M_RS * Iq * cosf(plant.theta_e) - omega_e * Ls_Iq_psi * sinf(plant.theta_e);
        float Vb = M_RS * Iq * sinf(plant.theta_e) + omega_e * Ls_Iq_psi * cosf(plant.theta_e);

        pmsm_step(&plant, Va, Vb, M_TS);
        EKF_Update(&h, Va, Vb, plant.ia, plant.ib);

        if (step > check_after) {
            float err = angle_err_deg(EKF_GetAngle(&h), plant.theta_e);
            if (err < 15.0f) converged++;
        }
    }
    printf("  converged steps (>1s): %d / %d\n", converged, total_steps - check_after);
    ASSERT(converged > check_win, "EKF angle within 15° for > 0.5s sustained");
    PASS("T5");
}

static void test_t6_mcsdk_angle(void)
{
    TEST("T6: EKF_GetAngleMCSdk consistent with EKF_GetAngle");
    EKF_Handle_t h;
    EKF_Init(&h, M_RS, M_LS, M_PSI_F, M_P, M_TS, M_QI, M_QE, M_RI);

    /* Force a known BEMF state: θ = π/4 */
    float theta = 3.14159265f / 4.0f;
    h.x[2] = -M_PSI_F * sinf(theta) * 1000.0f;  /* ea = -Ψf·ω·sin(θ), ω arbitrary */
    h.x[3] =  M_PSI_F * cosf(theta) * 1000.0f;  /* eb =  Ψf·ω·cos(θ) */

    float angle_float = EKF_GetAngle(&h);
    int16_t angle_int = EKF_GetAngleMCSdk(&h);

    /* Reconstruct float from int16 */
    float angle_recon = (float)angle_int / 32767.0f * 3.14159265f;

    printf("  theta_true=%.4f rad  EKF_GetAngle=%.4f  int16=%d  recon=%.4f\n",
           theta, angle_float, (int)angle_int, angle_recon);

    ASSERT_NEAR(angle_float, theta, 0.02f,           "float angle near π/4");
    ASSERT_NEAR(angle_recon, angle_float, 0.001f,    "int16 reconstruction < 0.001 rad error");
    PASS("T6");
}

static void test_t7_p_positive_definite(void)
{
    TEST("T7: P diagonal stays positive through 5000 update steps");
    EKF_Handle_t h;
    EKF_Init(&h, M_RS, M_LS, M_PSI_F, M_P, M_TS, M_QI, M_QE, M_RI);

    float omega_e_ref = 1600.0f * 2.0f * 3.14159265f / 60.0f * (float)M_P;
    float theta_ref   = 0.0f;
    float ia_steady   = 8.0f;

    int total_steps = (int)(5.0f / M_TS);   /* 5s = 125000 steps at 40μs */
    for (int step = 0; step < total_steps; step++) {
        theta_ref = fmodf(theta_ref + omega_e_ref * M_TS, 2.0f * 3.14159265f);
        float Ls_Iq_psi = M_LS * ia_steady + M_PSI_F;
        float Va = M_RS * ia_steady * cosf(theta_ref) - omega_e_ref * Ls_Iq_psi * sinf(theta_ref);
        float Vb = M_RS * ia_steady * sinf(theta_ref) + omega_e_ref * Ls_Iq_psi * cosf(theta_ref);
        float ia_m = ia_steady * cosf(theta_ref);
        float ib_m = ia_steady * sinf(theta_ref);
        EKF_Update(&h, Va, Vb, ia_m, ib_m);

        /* P diagonal: IP00=0, IP11=4, IP22=7, IP33=9 */
        if (h.P[0] <= 0.0f || h.P[4] <= 0.0f || h.P[7] <= 0.0f || h.P[9] <= 0.0f) {
            printf("  P diagonal non-positive at step %d: [%.2e %.2e %.2e %.2e]\n",
                   step, (double)h.P[0], (double)h.P[4], (double)h.P[7], (double)h.P[9]);
            ASSERT(0, "P diagonal positive definite");
            return;
        }
    }
    ASSERT(1, "P diagonal always positive");
    PASS("T7");
}

static void test_t8_high_noise(void)
{
    TEST("T8: EKF stable under high noise (σ=1.0A, 10× nominal)");
    EKF_Handle_t h;
    /* R scaled up 100× for high noise */
    float r_high = 1.0f * 1.0f;   /* σ=1.0A → R=1.0 A² */
    EKF_Init(&h, M_RS, M_LS, M_PSI_F, M_P, M_TS, M_QI, M_QE, r_high);

    float omega_e_ref = 1600.0f * 2.0f * 3.14159265f / 60.0f * (float)M_P;
    float theta_ref   = 0.0f;
    float ia_steady   = 8.0f;

    /* Simple LCG for deterministic noise */
    uint32_t rng = 12345;
    float noise_sigma = 1.0f;

    int total_steps = (int)(2.0f / M_TS);   /* 2s = 50000 steps at 40μs */
    for (int step = 0; step < total_steps; step++) {
        theta_ref = fmodf(theta_ref + omega_e_ref * M_TS, 2.0f * 3.14159265f);
        float Ls_Iq_psi = M_LS * ia_steady + M_PSI_F;
        float Va = M_RS * ia_steady * cosf(theta_ref) - omega_e_ref * Ls_Iq_psi * sinf(theta_ref);
        float Vb = M_RS * ia_steady * sinf(theta_ref) + omega_e_ref * Ls_Iq_psi * cosf(theta_ref);
        float ia_true = ia_steady * cosf(theta_ref);
        float ib_true = ia_steady * sinf(theta_ref);
        /* LCG noise */
        rng = 1664525u * rng + 1013904223u;
        float n0 = ((float)(int32_t)rng / 2147483648.0f) * noise_sigma;
        rng = 1664525u * rng + 1013904223u;
        float n1 = ((float)(int32_t)rng / 2147483648.0f) * noise_sigma;
        EKF_Update(&h, Va, Vb, ia_true + n0, ib_true + n1);

        for (int i = 0; i < 4; i++) ASSERT(!isnanf(h.x[i]), "no NaN in state under high noise");
        for (int i = 0; i < 10; i++) ASSERT(!isnanf(h.P[i]), "no NaN in P under high noise");
    }
    PASS("T8");
}

/* ─────────────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("esc_ekf_observer unit tests\n");
    printf("Motor: Rs=%.3f Ls=%.0fuH Psi_f=%.4e p=%d Ts=%.0fus\n\n",
           M_RS, M_LS * 1e6f, (double)M_PSI_F, M_P, M_TS * 1e6f);

    test_t1_init();
    test_t2_no_nan();
    test_t3_step_response();
    test_t4_speed_recovery();
    test_t5_angle_recovery();
    test_t6_mcsdk_angle();
    test_t7_p_positive_definite();
    test_t8_high_noise();

    printf("─────────────────────────────────────────\n");
    printf("Result: %d / %d tests passed\n", g_tests_pass, g_tests_run);
    return (g_tests_pass == g_tests_run) ? 0 : 1;
}
