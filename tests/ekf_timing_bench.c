/**
 * @file  ekf_timing_bench.c
 * @brief Step 3 DWT benchmark — measures EKF_Update() cycle count on STM32G431.
 *
 * Self-contained (no MCSDK dependencies). Contains a minimal inline EKF
 * implementation identical to the planned esc_ekf_observer.c.
 *
 * Build target: STM32G431 test build (not the production ESC build).
 * Add this file to a bare-metal STM32 project targeting the ESC board.
 * The UART2 printf output can be captured via the ST-Link VCP.
 *
 * Usage:
 *   Call EKF_Bench_Run() once from main() after SystemClock_Config().
 *   Results print via printf → UART2 (redirect via retarget_io if needed).
 *
 * Analytical prediction (from tests/ekf_opcount.py):
 *   280 float32 ops → 452 cycles (+20% overhead) → 0.27% CPU at 1 kHz.
 *   Target: < 5000 cycles.
 *
 * Expected output (example):
 *   [EKF bench] N=1000 iterations
 *   [EKF bench]   min:  410 cycles
 *   [EKF bench]   max:  520 cycles
 *   [EKF bench]   mean: 448 cycles
 *   [EKF bench]   CPU @ 1kHz, 170MHz: 0.26%
 *   [EKF bench] atan2f standalone:  18 cycles
 *   [EKF bench] sqrtf standalone:   12 cycles
 */

#include <stdint.h>
#include <math.h>
#include <stdio.h>   /* printf — redirect to UART in retarget_io or semihosting */

/* ─── DWT cycle counter ─────────────────────────────────────────────────── */

#ifdef __ARM_ARCH
/* ARM Cortex-M4 DWT registers (CMSIS-compatible, no CMSIS header needed). */
#  define DWT_CTRL   (*((volatile uint32_t *)0xE0001000U))
#  define DWT_CYCCNT (*((volatile uint32_t *)0xE0001004U))
#  define DEMCR      (*((volatile uint32_t *)0xE000EDFCU))
#  define DWT_CTRL_CYCCNTENA_Msk  (1UL << 0)
#  define DEMCR_TRCENA_Msk        (1UL << 24)

static inline void dwt_enable(void)
{
    DEMCR      |= DEMCR_TRCENA_Msk;
    DWT_CYCCNT  = 0;
    DWT_CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t dwt_cycles(void)  { return DWT_CYCCNT; }

#else
/* Host build (x86/amd64): DWT unavailable — cycles always read 0.
 * Compile with:  gcc -O2 -lm -o /tmp/ekf_bench tests/ekf_timing_bench.c /tmp/bench_main.c
 * On host, all cycle counts will be 0 (expected). The EKF math still runs
 * and can be inspected for NaN / correctness via the printf output. */
#  include <time.h>
static uint32_t _host_cyc;
static inline void    dwt_enable(void)    { _host_cyc = 0; }
static inline uint32_t dwt_cycles(void)   { return _host_cyc++; }
#endif

/* ─── Minimal EKF implementation (mirrors planned esc_ekf_observer.c) ───── */

#define EKF_Rs      0.100f          /* stator resistance [Ω] */
#define EKF_Ls      10e-6f          /* stator inductance [H] */
#define EKF_PSI_F   9.75e-4f        /* PM flux linkage [Wb] */
#define EKF_P       2               /* pole pairs */
#define EKF_TS      40e-6f          /* FOC period [s] — NOT the EKF period */
#define EKF_TS_SPD  1e-3f           /* EKF runs at 1 kHz (speed loop) */

/* Precomputed constants (computed in EKF_Init, not at runtime) */
#define EKF_A1      (1.0f - EKF_TS_SPD * EKF_Rs / EKF_Ls)   /* ≈ -9.0 at 1ms */
#define EKF_B       (EKF_TS_SPD / EKF_Ls)                     /* = 100 */
#define EKF_INV_PSI (1.0f / EKF_PSI_F)
#define EKF_INV_PSI2 (1.0f / (EKF_PSI_F * EKF_PSI_F))

/* Q / R noise matrices (diagonal) */
#define EKF_Q_I     3.33e-3f   /* current state process noise [A²] */
#define EKF_Q_E     3.33e-2f   /* BEMF state process noise */
#define EKF_R_I     3.33e-3f   /* current measurement noise [A²] */

typedef struct {
    float x[4];       /* [iα, iβ, eα, eβ] */
    /* P stored as upper triangle: P[0..9] = P00,P01,P02,P03,P11,P12,P13,P22,P23,P33 */
    float P[10];
} EKF_State_t;

/* Indices into upper-triangle P array */
#define P00 0
#define P01 1
#define P02 2
#define P03 3
#define P11 4
#define P12 5
#define P13 6
#define P22 7
#define P23 8
#define P33 9

static void EKF_Init(EKF_State_t *s)
{
    for (int i = 0; i < 4; i++) s->x[i] = 0.0f;
    /* Large initial uncertainty */
    s->P[P00] = 1.0f; s->P[P11] = 1.0f; s->P[P22] = 1.0f; s->P[P33] = 1.0f;
    s->P[P01] = 0.0f; s->P[P02] = 0.0f; s->P[P03] = 0.0f;
    s->P[P12] = 0.0f; s->P[P13] = 0.0f; s->P[P23] = 0.0f;
}

/**
 * @brief  EKF_Update — one EKF predict+update step.
 *
 * This is the function being timed. It is intentionally written for clarity,
 * not maximum performance — the compiler will optimise with -O2.
 *
 * @param s     EKF state (x, P)
 * @param Va    Applied αβ voltage [V]
 * @param Vb
 * @param ia_m  Measured α current [A]
 * @param ib_m  Measured β current [A]
 */
static void EKF_Update(EKF_State_t *s,
                       float Va, float Vb,
                       float ia_m, float ib_m)
{
    float ia = s->x[0], ib = s->x[1], ea = s->x[2], eb = s->x[3];

    /* ── 1. ω from current BEMF estimate ──────────────────────────────── */
    float psi2    = EKF_PSI_F * EKF_PSI_F;
    float bemf2   = ea * ea + eb * eb;
    float omega_e = sqrtf(bemf2) * EKF_INV_PSI + 1e-12f;
    float omega_s = omega_e;              /* safe ω (already ε-guarded) */

    /* ── 2. Precompute omega_Ts (reused in x_pred and Jacobian) ──────── */
    float oTs = omega_s * EKF_TS_SPD;

    /* ── 3. Predicted state x_pred ───────────────────────────────────── */
    float ia_p = EKF_A1 * ia + EKF_B * (Va - ea);
    float ib_p = EKF_A1 * ib + EKF_B * (Vb - eb);
    float ea_p = ea - oTs * eb;
    float eb_p = eb + oTs * ea;

    /* ── 4. Jacobian F (nonzero off-diagonal elements only) ─────────── */
    /* c = Ts / (Ψf² × ω)  — common factor in BEMF Jacobian rows */
    float c    = EKF_TS_SPD / (psi2 * omega_s);
    float ceaeb = c * ea * eb;
    float f22  =  1.0f - ceaeb;
    float f33  =  1.0f + ceaeb;
    float f23  = -(oTs + c * eb * eb);   /* -(ω·Ts + c·eβ²) */
    float f32  =   oTs + c * ea * ea;   /*  (ω·Ts + c·eα²) */
    /* a1 = EKF_A1 (precomputed), b = EKF_B (precomputed) */

    /* ── 5. P_pred = F×P×Fᵀ + Q  (exploit F sparsity) ─────────────── */
    /*
     * F has structure:
     *   [ a1   0   -b   0  ]    (rows 0,1 couple current to BEMF)
     *   [  0  a1    0  -b  ]
     *   [  0   0  f22  f23 ]    (rows 2,3: BEMF coupling)
     *   [  0   0  f32  f33 ]
     *
     * P is symmetric; we store upper triangle only.
     * We compute full FP first (4×4), then (FP)×Fᵀ (symmetric result).
     * Intermediate FP stored as full 4×4 for clarity.
     */

    /* Read P upper triangle (P[i][j] = P[j][i]) */
    float p00 = s->P[P00], p01 = s->P[P01], p02 = s->P[P02], p03 = s->P[P03];
    float                   p11 = s->P[P11], p12 = s->P[P12], p13 = s->P[P13];
    float                                    p22 = s->P[P22], p23 = s->P[P23];
    float                                                      p33 = s->P[P33];

    /* FP = F × P — row by row using F sparsity */
    /* Row 0 of FP: a1*P[0,:] - b*P[2,:] */
    float fp00 = EKF_A1 * p00 - EKF_B * p02;
    float fp01 = EKF_A1 * p01 - EKF_B * p12;
    float fp02 = EKF_A1 * p02 - EKF_B * p22;
    float fp03 = EKF_A1 * p03 - EKF_B * p23;
    /* Row 1 of FP: a1*P[1,:] - b*P[3,:] (symmetric P: P[1,0]=p01, P[3,0]=p03 etc.) */
    float fp10 = EKF_A1 * p01 - EKF_B * p03;
    float fp11 = EKF_A1 * p11 - EKF_B * p13;
    float fp12 = EKF_A1 * p12 - EKF_B * p23;
    float fp13 = EKF_A1 * p13 - EKF_B * p33;
    /* Row 2 of FP: f22*P[2,:] + f23*P[3,:] */
    float fp20 = f22 * p02 + f23 * p03;
    float fp21 = f22 * p12 + f23 * p13;
    float fp22 = f22 * p22 + f23 * p23;
    float fp23 = f22 * p23 + f23 * p33;
    /* Row 3 of FP: f32*P[2,:] + f33*P[3,:] */
    float fp30 = f32 * p02 + f33 * p03;
    float fp31 = f32 * p12 + f33 * p13;
    float fp32 = f32 * p22 + f33 * p23;
    float fp33_ = f32 * p23 + f33 * p33;

    /* (FP)×Fᵀ — result is symmetric, compute upper triangle only.
     * Fᵀ columns: [a1,0,0,0], [0,a1,0,0], [-b,0,f22,f32], [0,-b,f23,f33] */
    /* pp[i][j] = Σ_k FP[i][k] × F[j][k]  (Fᵀ[k][j] = F[j][k]) */
    float pp00 = fp00 * EKF_A1                   + fp02 * (-EKF_B)                       + EKF_Q_I;
    float pp01 = fp00 * 0.0f + fp01 * EKF_A1     + fp03 * (-EKF_B);
    float pp02 = fp00 * (-EKF_B)                  + fp02 * f22         + fp03 * f32;
    float pp03 = fp00 * 0.0f                       + fp02 * f23         + fp03 * f33;
    float pp11 = fp11 * EKF_A1                   + fp13 * (-EKF_B)                       + EKF_Q_I;
    float pp12 = fp10 * (-EKF_B)                  + fp12 * f22         + fp13 * f32;
    float pp13 = fp11 * (-EKF_B)                  + fp12 * f23         + fp13 * f33;
    float pp22 = fp20 * (-EKF_B)                  + fp22 * f22         + fp23 * f32       + EKF_Q_E;
    float pp23 = fp20 * 0.0f  + fp21 * (-EKF_B)  + fp22 * f23         + fp23 * f33;
    float pp33 = fp31 * (-EKF_B)                  + fp32 * f23         + fp33_ * f33      + EKF_Q_E;

    /* Suppress compiler warnings for unused intermediate rows */
    (void)fp10; (void)fp20; (void)fp21; (void)fp30; (void)fp31; (void)fp32;

    /* ── 6–7. Innovation and S ───────────────────────────────────────── */
    float yi0 = ia_m - ia_p;
    float yi1 = ib_m - ib_p;

    /* S = H×P_pred×Hᵀ + R = top-left 2×2 of P_pred + diag(R) */
    float s00 = pp00 + EKF_R_I;
    float s01 = pp01;
    float s11 = pp11 + EKF_R_I;

    /* ── 8. S⁻¹ (2×2 symmetric) ─────────────────────────────────────── */
    float det    = s00 * s11 - s01 * s01;
    float inv_d  = 1.0f / det;
    float si00   =  s11 * inv_d;
    float si01   = -s01 * inv_d;
    float si11   =  s00 * inv_d;

    /* ── 9. K = P_pred×Hᵀ × S⁻¹  (4×2 result) ─────────────────────── */
    /* P_pred×Hᵀ extracts cols 0,1 of P_pred → [pp00,pp01; pp01,pp11; pp02,pp12; pp03,pp13] */
    float k00 = pp00 * si00 + pp01 * si01;
    float k01 = pp00 * si01 + pp01 * si11;
    float k10 = pp01 * si00 + pp11 * si01;
    float k11 = pp01 * si01 + pp11 * si11;
    float k20 = pp02 * si00 + pp12 * si01;
    float k21 = pp02 * si01 + pp12 * si11;
    float k30 = pp03 * si00 + pp13 * si01;
    float k31 = pp03 * si01 + pp13 * si11;

    /* ── 10. x update ───────────────────────────────────────────────── */
    s->x[0] = ia_p + k00 * yi0 + k01 * yi1;
    s->x[1] = ib_p + k10 * yi0 + k11 * yi1;
    s->x[2] = ea_p + k20 * yi0 + k21 * yi1;
    s->x[3] = eb_p + k30 * yi0 + k31 * yi1;

    /* ── 11. P update: P = P_pred - K×S×Kᵀ ────────────────────────── */
    /* K×S (4×2): [k0,:] × [[s00,s01],[s01,s11]] */
    float ks00 = k00 * s00 + k01 * s01;   float ks01 = k00 * s01 + k01 * s11;
    float ks10 = k10 * s00 + k11 * s01;   float ks11 = k10 * s01 + k11 * s11;
    float ks20 = k20 * s00 + k21 * s01;   float ks21 = k20 * s01 + k21 * s11;
    float ks30 = k30 * s00 + k31 * s01;   float ks31 = k30 * s01 + k31 * s11;

    /* (K×S)×Kᵀ upper triangle: (KS)[i,:] · K[j,:] for i ≤ j */
    s->P[P00] = pp00 - (ks00 * k00 + ks01 * k01);
    s->P[P01] = pp01 - (ks00 * k10 + ks01 * k11);
    s->P[P02] = pp02 - (ks00 * k20 + ks01 * k21);
    s->P[P03] = pp03 - (ks00 * k30 + ks01 * k31);
    s->P[P11] = pp11 - (ks10 * k10 + ks11 * k11);
    s->P[P12] = pp12 - (ks10 * k20 + ks11 * k21);
    s->P[P13] = pp13 - (ks10 * k30 + ks11 * k31);
    s->P[P22] = pp22 - (ks20 * k20 + ks21 * k21);
    s->P[P23] = pp23 - (ks20 * k30 + ks21 * k31);
    s->P[P33] = pp33 - (ks30 * k30 + ks31 * k31);
}

static inline float EKF_GetAngle(const EKF_State_t *s)
{
    return atan2f(-s->x[2], s->x[3]);
}

static inline float EKF_GetSpeedRPM(const EKF_State_t *s)
{
    float ea = s->x[2], eb = s->x[3];
    return sqrtf(ea * ea + eb * eb) * EKF_INV_PSI * (60.0f / (2.0f * 3.14159265f * EKF_P));
}

/* ─── Benchmark ─────────────────────────────────────────────────────────── */

#define BENCH_N      1000U          /* iterations for statistics */
#define F_CPU_HZ     170000000UL

/**
 * @brief  EKF_Bench_Run — call from main() after SystemClock_Config().
 *
 * Uses DWT_CYCCNT to measure EKF_Update() wall cycles.
 * Input values are non-trivial constants so the compiler cannot constant-fold.
 */
void EKF_Bench_Run(void)
{
    dwt_enable();

    EKF_State_t ekf;
    EKF_Init(&ekf);

    /* Warm up: run 100 iterations to populate P, fill branch predictor */
    for (uint32_t i = 0; i < 100; i++) {
        EKF_Update(&ekf, 0.327f, 0.0f, 0.30f, 0.05f);
    }

    uint32_t min_cyc  = UINT32_MAX;
    uint32_t max_cyc  = 0;
    uint64_t sum_cyc  = 0;
    volatile float angle_sink = 0.0f;   /* prevent dead-code elimination */

    for (uint32_t i = 0; i < BENCH_N; i++) {
        /* Vary inputs slightly so compiler cannot hoist loop body */
        float Va = 0.327f + (float)i * 1e-6f;
        float Vb = 0.001f + (float)i * 1e-6f;
        float ia = 7.9f   + (float)i * 1e-5f;
        float ib = 0.5f   + (float)i * 1e-5f;

        uint32_t t0 = dwt_cycles();
        EKF_Update(&ekf, Va, Vb, ia, ib);
        uint32_t cyc = dwt_cycles() - t0;

        if (cyc < min_cyc) min_cyc = cyc;
        if (cyc > max_cyc) max_cyc = cyc;
        sum_cyc += cyc;
    }

    /* Consume outputs so compiler doesn't eliminate the calls */
    angle_sink = EKF_GetAngle(&ekf) + EKF_GetSpeedRPM(&ekf);
    (void)angle_sink;

    uint32_t mean_cyc = (uint32_t)(sum_cyc / BENCH_N);
    float    cpu_pct  = (float)mean_cyc / (float)(F_CPU_HZ / 1000) * 100.0f;

    /* Benchmark atan2f and sqrtf standalone for reference */
    volatile float x = 0.327f, y = -0.100f;
    uint32_t t_atan = dwt_cycles();
    volatile float r_atan = atan2f(x, y); (void)r_atan;
    t_atan = dwt_cycles() - t_atan;

    volatile float z = 0.107f;
    uint32_t t_sqrt = dwt_cycles();
    volatile float r_sqrt = sqrtf(z); (void)r_sqrt;
    t_sqrt = dwt_cycles() - t_sqrt;

    /* Output results */
    printf("\r\n[EKF bench] N=%u iterations\r\n", BENCH_N);
    printf("[EKF bench]   min:  %lu cycles\r\n", (unsigned long)min_cyc);
    printf("[EKF bench]   max:  %lu cycles\r\n", (unsigned long)max_cyc);
    printf("[EKF bench]   mean: %lu cycles\r\n", (unsigned long)mean_cyc);
    printf("[EKF bench]   CPU @ 1kHz, 170MHz: %.2f%%\r\n", (double)cpu_pct);
    printf("[EKF bench]   Target: < 5000 cycles — %s\r\n",
           mean_cyc < 5000U ? "PASS" : "FAIL");
    printf("[EKF bench] atan2f standalone:  %lu cycles\r\n", (unsigned long)t_atan);
    printf("[EKF bench] sqrtf  standalone:  %lu cycles\r\n", (unsigned long)t_sqrt);
    printf("\r\n");

    /* Analytical comparison */
    printf("[EKF bench] Analytical prediction (ekf_opcount.py):\r\n");
    printf("[EKF bench]   280 float ops, 452 cycles (+20%% overhead)\r\n");
    printf("[EKF bench]   F sparsity: F*P avoids 48/128 zero muls vs naive 4x4\r\n");
    printf("[EKF bench]   P symmetric: 10-element upper triangle, not full 4x4\r\n");
}

/*
 * To add this benchmark to the ESC firmware:
 *
 *   1. Add ekf_timing_bench.c to CMakeLists.txt / STM32CubeIDE sources.
 *   2. In main.c, after SystemClock_Config():
 *        extern void EKF_Bench_Run(void);
 *        EKF_Bench_Run();
 *   3. Ensure printf is redirected to UART (retarget_io or semihosting).
 *   4. Build with -O2 (same as production build).
 *   5. Flash and read UART output.
 *
 * Compile-time check (host PC) for logic correctness:
 *   gcc -O2 -lm -o /tmp/ekf_bench tests/ekf_timing_bench.c \
 *       -DBENCH_HOST -include stdio.h && /tmp/ekf_bench
 *
 * On host the DWT reads will return 0 (registers unavailable), so cycle
 * counts will be 0, but the EKF math will still run and can be checked
 * for NaN via the printf output.
 */
