#!/usr/bin/env python3
"""
ekf_opcount.py — Step 3: EKF floating-point operation count and STM32G431 timing estimate.

Counts the exact number of float32 operations in one EKF_Update() call, broken down
by operation type. Extrapolates to cycle count on STM32G431 (Cortex-M4 FPU, 170 MHz).

Usage:
    python3 tests/ekf_opcount.py

References:
    ARM Cortex-M4 TRM — FPU instruction latencies (most single-cycle)
    STM32G431 datasheet — 170 MHz, single-precision FPU, no double-precision HW
"""

# ─────────────────────────────────────────────────────────────────────────────
# Operation counter
# ─────────────────────────────────────────────────────────────────────────────

class OpCounter:
    """Tracks float32 operations symbolically."""
    def __init__(self):
        self.mul   = 0   # VMUL.F32   — 1 cycle
        self.add   = 0   # VADD.F32   — 1 cycle
        self.sub   = 0   # VSUB.F32   — 1 cycle
        self.div   = 0   # VDIV.F32   — 14 cycles
        self.sqrt  = 0   # VSQRT.F32  — 14 cycles
        self.atan2 = 0   # library call (libm, FPU-accelerated) — ~20 cycles
        self.madd  = 0   # VFMA.F32   — 1 cycle (fused multiply-add, cheaper than mul+add)

    def total_ops(self):
        return self.mul + self.add + self.sub + self.div + self.sqrt + self.atan2 + self.madd

    def cycles_g431(self):
        """Conservative cycle estimate for STM32G431 Cortex-M4 FPU."""
        return (self.mul   * 1  +
                self.add   * 1  +
                self.sub   * 1  +
                self.madd  * 1  +
                self.div   * 14 +
                self.sqrt  * 14 +
                self.atan2 * 20)

    def __repr__(self):
        return (f"mul={self.mul} add={self.add} sub={self.sub} "
                f"div={self.div} sqrt={self.sqrt} atan2={self.atan2} "
                f"madd={self.madd}")


# ─────────────────────────────────────────────────────────────────────────────
# EKF_Update operation breakdown
# ─────────────────────────────────────────────────────────────────────────────
#
# State: x = [iα, iβ, eα, eβ]    P = 4×4 symmetric covariance
# Input: u = [Vα, Vβ]            y = [iα_meas, iβ_meas]
#
# Precomputed constants (zero runtime cost, done in EKF_Init or compile-time):
#   a1 = 1 - Ts*Rs/Ls            (= 1 - 40e-6*0.1/10e-6 = 0.6)
#   b  = Ts/Ls                   (= 40e-6/10e-6 = 4.0)
#   inv_psi2 = 1/Ψf²             (= 1/(9.75e-4)² = 1.051e6)
#
# Diagonal R: S is also diagonal → 2×2 inversion costs 2 divs (no det needed).
#
# Sparse F structure:
#   Row 0: [a1, 0,  -b,  0 ]
#   Row 1: [0,  a1,  0, -b ]
#   Row 2: [0,  0,  f22, f23]
#   Row 3: [0,  0,  f32, f33]
#
# Joseph form for P update: P = (I-KH)P(I-KH)ᵀ + KRKᵀ
# Simplified for diagonal R: P = P_pred - K*S*Kᵀ
#   (numerically stable, fewer ops than (I-KH)*P_pred for our 4×4 case)

def count_ekf_update(verbose=True) -> OpCounter:
    c = OpCounter()

    # ── 1. ω from BEMF ───────────────────────────────────────────────────────
    # omega_e = sqrt(ea²+eb²) / psi_f
    # With 1/psi_f precomputed: omega_e = sqrt(ea²+eb²) * inv_psi_f
    c.mul  += 2   # ea*ea, eb*eb
    c.add  += 1   # ea²+eb²
    c.sqrt += 1   # sqrt(...)
    c.mul  += 1   # × inv_psi_f
    ops_omega = 5
    if verbose: print(f"  1. ω estimate:          {ops_omega} ops  (2 mul, 1 add, 1 sqrt, 1 mul)")

    # ── 2. omega_Ts = ω*Ts ───────────────────────────────────────────────────
    # (reused 4 times in x_pred and Jacobian)
    c.mul  += 1
    ops_omega_ts = 1
    if verbose: print(f"  2. omega_Ts precompute:  {ops_omega_ts} op   (1 mul)")

    # ── 3. x_pred ────────────────────────────────────────────────────────────
    # iα_next = a1*iα + b*(Vα - ea)        → 1 sub, 1 mul, 1 fma
    # iβ_next = a1*iβ + b*(Vβ - eb)        → 1 sub, 1 mul, 1 fma
    # eα_next = eα - omega_Ts * eβ         → 1 mul, 1 sub
    # eβ_next = eβ + omega_Ts * eα         → 1 mul, 1 add
    c.sub  += 2   # Vα-ea, Vβ-eb
    c.mul  += 2   # b*(Vα-ea), b*(Vβ-eb)
    c.madd += 2   # a1*iα + ..., a1*iβ + ...     (FMA: fused multiply-add = 1 cycle)
    c.mul  += 2   # omega_Ts*eβ, omega_Ts*eα
    c.sub  += 1   # eα - ...
    c.add  += 1   # eβ + ...
    ops_xpred = 10
    if verbose: print(f"  3. x_pred:              {ops_xpred} ops  (2 sub, 2 mul, 2 fma, 2 mul, 1 sub, 1 add)")

    # ── 4. Jacobian F (nonzero/non-one elements only) ────────────────────────
    # c_J = Ts * inv_psi2 / omega_e         → 1 mul, 1 div (or precompute inv_omega)
    # f22 = 1 - c_J*ea*eb                  → 2 mul, 1 sub
    # f23 = -(omega_Ts + c_J*Ts*eb*eb)     → 1 mul, 1 mul, 1 add, negate (no op)
    # f32 =   omega_Ts + c_J*Ts*ea*ea      → 1 mul, 1 mul, 1 add
    # f33 = 1 + c_J*ea*eb                  → reuse c_J*ea*eb → 1 add
    # Note: c_J*ea*eb computed once, reused for f22 and f33.
    c.mul  += 1   # Ts * inv_psi2
    c.div  += 1   # / omega_e
    c.mul  += 2   # c_J*ea, c_J*ea*eb
    c.sub  += 1   # 1 - c_J*ea*eb  (f22)
    c.mul  += 1   # c_J*eb*eb
    c.add  += 1   # omega_Ts + c_J*Ts*eb² ... wait need *Ts too
    # Actually c_J already has Ts factor: c_J = Ts/(psi2*omega)
    # f23 = -(Ts*omega + c_J*eb*eb) = -(omega_Ts + c_J*eb²)
    # f32 =   Ts*omega + c_J*ea*ea  = omega_Ts + c_J*ea²
    c.mul  += 1   # c_J*ea*ea  (for f32)
    c.add  += 2   # omega_Ts + c_J*eb² (f23), omega_Ts + c_J*ea² (f32)
    c.add  += 1   # 1 + c_J*ea*eb (f33)
    # a1, b, omega_Ts already computed
    ops_jac = 11
    if verbose: print(f"  4. Jacobian:            {ops_jac} ops  (1 mul, 1 div, 4 mul, 3 add, 1 sub)")

    # ── 5. P_pred = F×P×Fᵀ + Q ───────────────────────────────────────────────
    #
    # F is block-sparse (as above). P is 4×4 symmetric (10 unique values).
    # Exploit sparsity: compute F×P using only nonzero F entries.
    #
    # F×P: each row of F×P = linear combination of rows of P.
    #   (FP)[0,:] = a1*P[0,:] - b*P[2,:]      → 4 mul + 4 mul + 4 sub = 12
    #   (FP)[1,:] = a1*P[1,:] - b*P[3,:]      → 12
    #   (FP)[2,:] = f22*P[2,:] + f23*P[3,:]   → 4 mul + 4 mul + 4 add = 12 + 4 = 12 (with FMA: 8)
    #   (FP)[3,:] = f32*P[2,:] + f33*P[3,:]   → 8 FMA (if FMA available) or 12
    # Total F×P: 48 ops (without FMA) or 32 ops (with FMA on all)
    # Use conservative estimate: assume FMA available for a1*x + (-b*y) patterns.
    c.madd += 8   # FMA for rows 0,1: 4 each for a1*P[i,:] - b*P[j,:]
    c.sub  += 8   # subtract b*P[j,:] if not FMA... let's be conservative: all FMA
    # Actually let's just count all as explicit muls+adds/subs for clarity
    # Reset and recount cleanly:
    c.mul  -= 0   # already zero additions above for FP
    # Row 0: 4 muls (a1*P[0,0..3]) + 4 muls (b*P[2,0..3]) + 4 sub = 12
    # Row 1: same = 12
    # Row 2: 4 muls (f22*P[2,:]) + 4 muls (f23*P[3,:]) + 4 add = 12
    # Row 3: 4 muls (f32*P[2,:]) + 4 muls (f33*P[3,:]) + 4 add = 12
    c.madd = 0   # reset FMA (count separately)
    c.mul  += 32  # 8 muls/row × 4 rows
    c.sub  += 8   # rows 0,1: 4 sub each
    c.add  += 8   # rows 2,3: 4 add each
    # (FP)×Fᵀ: Fᵀ has same sparsity.
    # Col 0 of Fᵀ: [a1, 0, 0, 0]   → each of 4 rows of FP × a1 → 4 mul
    # Col 1 of Fᵀ: [0, a1, 0, 0]   → 4 mul
    # Col 2 of Fᵀ: [-b, 0, f22, f32] → 4×(2 mul + 1 sub + 1 add) = 16
    # Col 3 of Fᵀ: [0, -b, f23, f33] → 4×(2 mul + 1 sub + 1 add) = 16
    c.mul  += 4 + 4 + 16 + 16   # = 40 muls for (FP)Fᵀ
    c.sub  += 8                  # from b terms (negated)
    c.add  += 8                  # from f22/f23/f32/f33 cross terms
    # +Q: only diagonal adds
    c.add  += 4   # P_pred += Q (diagonal)
    ops_ppred = 32 + 8 + 8 + 40 + 8 + 8 + 4
    if verbose: print(f"  5. P_pred = F×P×Fᵀ+Q:  {ops_ppred} ops  (72 mul, 32 add, 16 sub, 4 add for Q)")

    # ── 6. Innovation y_innov = y - H×x_pred ─────────────────────────────────
    # H extracts rows 0,1 of x_pred → no multiply, just subtraction
    c.sub  += 2
    if verbose: print(f"  6. y_innov:              2 ops  (2 sub)")

    # ── 7. S = H×P_pred×Hᵀ + R ───────────────────────────────────────────────
    # H extracts top-left 2×2 of P_pred:
    # S[0,0] = P_pred[0,0] + R[0]
    # S[0,1] = P_pred[0,1]
    # S[1,0] = P_pred[1,0]
    # S[1,1] = P_pred[1,1] + R[1]
    # (R is diagonal, so S[0,1]=S[1,0]=P_pred[0,1])
    c.add  += 2   # +R diagonal
    if verbose: print(f"  7. S = H×P×Hᵀ+R:        2 ops  (2 add, H extracts 2×2 block)")

    # ── 8. S_inv (2×2, symmetric, diagonal R) ────────────────────────────────
    # S is symmetric: S = [[s00, s01],[s01, s11]]
    # det = s00*s11 - s01²
    # S_inv = [[s11, -s01],[-s01, s00]] / det
    c.mul  += 1   # s00*s11
    c.mul  += 1   # s01*s01
    c.sub  += 1   # det
    c.div  += 3   # s11/det, s01/det, s00/det  (3 unique values in S_inv for symmetric)
    if verbose: print(f"  8. S_inv:                6 ops  (2 mul, 1 sub, 3 div)")

    # ── 9. K = P_pred×Hᵀ×S_inv ───────────────────────────────────────────────
    # P_pred×Hᵀ: extracts first 2 cols of P_pred → P_H (4×2, no ops)
    # K = P_H × S_inv  (4×2 × 2×2 → 4×2)
    # Each of 4 rows of K: [P_H[i,0], P_H[i,1]] × S_inv
    #   K[i,0] = P_H[i,0]*S_inv[0,0] + P_H[i,1]*S_inv[1,0]  → 2 mul + 1 add
    #   K[i,1] = P_H[i,0]*S_inv[0,1] + P_H[i,1]*S_inv[1,1]  → 2 mul + 1 add
    # 4 rows × (4 mul + 2 add) = 24 ops
    c.mul  += 16
    c.add  += 8
    if verbose: print(f"  9. K = P_pred×Hᵀ×S_inv: 24 ops (16 mul, 8 add)")

    # ── 10. x update: x = x_pred + K×y_innov ────────────────────────────────
    # K×y_innov: 4×2 × 2×1 = 4×1
    #   [K[0,0]*y[0]+K[0,1]*y[1], ..., K[3,0]*y[0]+K[3,1]*y[1]]
    # 4 rows × (2 mul + 1 add) = 12 ops
    # x += K×y: 4 adds
    c.mul  += 8
    c.add  += 4 + 4
    if verbose: print(f" 10. x update:            12 ops  (8 mul, 4 add) + 4 add for x+=")

    # ── 11. P update: P = P_pred - K×S×Kᵀ ───────────────────────────────────
    # (Numerically stable simplified form for diagonal R)
    # K×S: 4×2 × 2×2 → 4×2
    #   Each row: [K[i,0]*S[0,0]+K[i,1]*S[1,0], K[i,0]*S[0,1]+K[i,1]*S[1,1]]
    #   = 4 × (2 mul + 1 add) + 4 × (2 mul + 1 add) = 24 ops
    # (K×S)×Kᵀ: 4×2 × 2×4 → 4×4
    #   Each of 16 elements: dot of length-2 row of (K×S) with col of Kᵀ = 2 mul + 1 add
    #   = 16 × (2 mul + 1 add) = 48 ops
    # P_pred - KSKᵀ: 16 subs (but P is symmetric → compute only upper triangle: 10 subs)
    c.mul  += 8 + 8       # K×S: 16 muls
    c.add  += 4 + 4       # K×S: 8 adds
    c.mul  += 32          # (KS)Kᵀ: 32 muls
    c.add  += 16          # (KS)Kᵀ: 16 adds
    c.sub  += 10          # P - KSKᵀ upper triangle only (symmetric)
    ops_pupdate = 16 + 8 + 32 + 16 + 10
    if verbose: print(f" 11. P update (P-K×S×Kᵀ): {ops_pupdate} ops  (48 mul, 24 add, 10 sub)")

    return c


def count_ekf_outputs() -> OpCounter:
    """Count ops for EKF_GetAngle and EKF_GetSpeedRPM (called once per ms)."""
    c = OpCounter()
    # EKF_GetAngle: atan2f(-ea, eb) → 1 negation (free) + 1 atan2
    c.atan2 += 1
    # EKF_GetSpeedRPM: sqrtf(ea²+eb²) * inv_psi_f * rpm_scale
    c.mul   += 2   # ea*ea, eb*eb
    c.add   += 1   # ea²+eb²
    c.sqrt  += 1   # sqrt
    c.mul   += 1   # * inv_psi_f
    c.mul   += 1   # * (60/(2π×p)) — precomputed constant
    return c


# ─────────────────────────────────────────────────────────────────────────────
# Cycle budget analysis
# ─────────────────────────────────────────────────────────────────────────────

F_CPU_HZ     = 170e6      # STM32G431 max clock
F_EKF_HZ     = 1000       # EKF runs at 1 kHz (speed loop rate, NOT 25 kHz FOC rate)
CYCLES_AVAIL = F_CPU_HZ / F_EKF_HZ   # cycles per EKF tick


def print_report():
    print("=" * 65)
    print("EKF_Update() — STM32G431 float32 operation count")
    print("=" * 65)
    print()
    print("Per-section breakdown:")

    c_update = count_ekf_update(verbose=True)
    c_output = count_ekf_outputs()

    total_ops    = c_update.total_ops() + c_output.total_ops()
    est_cycles   = c_update.cycles_g431() + c_output.cycles_g431()

    # Add loop overhead / register spills (conservatively 20%)
    est_cycles_w_overhead = int(est_cycles * 1.2)

    cpu_pct  = est_cycles_w_overhead / CYCLES_AVAIL * 100.0

    print()
    print(f" 12. Outputs (angle, RPM):  {c_output.total_ops()} ops  (6 mul, 1 add, 1 sqrt, 1 atan2)")
    print()
    print("-" * 65)
    print(f"  Totals:   {c_update}")
    print(f"  Outputs:  {c_output}")
    print()
    print(f"  Total float32 ops:       {total_ops}")
    print(f"  Cycles (no overhead):    {c_update.cycles_g431() + c_output.cycles_g431()}")
    print(f"  Cycles (+20% overhead):  {est_cycles_w_overhead}")
    print()
    print("=" * 65)
    print("STM32G431 timing budget (170 MHz, EKF at 1 kHz):")
    print("=" * 65)
    print(f"  Cycles available per tick:  {CYCLES_AVAIL:.0f}")
    print(f"  EKF estimated cycles:       {est_cycles_w_overhead}")
    print(f"  CPU utilisation:            {cpu_pct:.2f}%")
    print(f"  Wall time per EKF call:     {est_cycles_w_overhead/F_CPU_HZ*1e6:.1f} µs")
    print()
    if cpu_pct < 1.0:
        print(f"  ✓ FEASIBLE — EKF fits comfortably within 1 kHz budget (<1% CPU)")
    elif cpu_pct < 5.0:
        print(f"  ✓ FEASIBLE — EKF fits within 1 kHz budget")
    else:
        print(f"  ✗ TIGHT — consider optimisation or lower update rate")
    print()
    print("Instruction latency assumptions (ARM Cortex-M4 FPU):")
    print("  VMUL.F32 / VADD.F32 / VSUB.F32:  1 cycle")
    print("  VFMA.F32 (fused mul-add):         1 cycle")
    print("  VDIV.F32:                        14 cycles")
    print("  VSQRT.F32:                       14 cycles")
    print("  atan2f() (libm, FPU-accel):      ~20 cycles")
    print()
    print("Key optimisation notes:")
    print("  - Precompute a1=1-Ts*Rs/Ls, b=Ts/Ls, inv_psi_f, inv_psi2 in EKF_Init")
    print("  - Exploit F sparsity: F×P avoids 32/64 zero muls in naive 4×4 product")
    print("  - P is symmetric: store/update only upper triangle (10 floats vs 16)")
    print("  - Diagonal R: S is 2×2 symmetric → 3 divs instead of full matrix invert")
    print("  - omega_Ts = ω*Ts reused across x_pred and Jacobian (1 mul saved)")
    print()
    print("Comparison: DWT measurement target (Step 3 deliverable)")
    print(f"  Target:  < 5000 cycles  (plan specification)")
    print(f"  Estimate:  {est_cycles_w_overhead} cycles  (analytical)")
    print(f"  Margin:  {5000 - est_cycles_w_overhead} cycles headroom")
    print()


if __name__ == '__main__':
    print_report()
