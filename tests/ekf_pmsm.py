#!/usr/bin/env python3
"""
ekf_pmsm.py — PMSM EKF observer prototype (Step 2)

Simulates a PMSM open-loop rev-up and compares:
  1. PMSM simulator  — ground truth (true angle, true speed)
  2. Luenberger+PLL  — simplified MCSDK STO equivalent
  3. EKF             — proposed replacement

Motor parameters: 2852 3100KV SPM outrunner (Zulu HOSIM/AMORIL).
Drivetrain loads: HOSIM (heavy, high friction) and AMORIL (lighter).

Usage:
    python3 tests/ekf_pmsm.py               # runs both drivetrains, saves plot
    python3 tests/ekf_pmsm.py --sweep-q     # sweeps Q process noise, shows sensitivity
    python3 tests/ekf_pmsm.py --no-plot     # text output only

Output:
    /tmp/ekf_comparison.png  — speed estimate + angle error for both drivetrains
    Console summary table    — minimum speed at which each observer locks (< 15° error)

References:
    Chen et al. (1998) — extended BEMF state formulation [iα,iβ,eα,eβ]
    Bolognani et al. (2001) — SPM EKF at low speed
    pmsm_motor_parameters.h, drive_parameters.h — hardware constants
"""

import argparse
import numpy as np
import sys
from dataclasses import dataclass

# ─────────────────────────────────────────────────────────────────────────────
# Motor and drivetrain parameters
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class MotorParams:
    """2852 3100KV SPM outrunner — Motor Pilot identified values."""
    Rs:    float = 0.100      # stator resistance [Ω]
    Ls:    float = 10e-6      # stator inductance [H]
    psi_f: float = 9.75e-4   # PM flux linkage [Wb]  (derived from Ke=0.25 V_rms_LL/kRPM)
    p:     int   = 2          # pole pairs


@dataclass
class DrivetrainLoad:
    """
    Friction load at motor shaft.
    T_load = T_coulomb * sign(ω) + B_viscous * ω
    """
    T_coulomb: float   # Coulomb friction [N·m at motor shaft]
    B_viscous: float   # Viscous friction coefficient [N·m·s/rad]
    J:         float   # Total inertia [kg·m²] at motor shaft


# 3-phase torque: Te = (3/2)×p×Ψf×Iq.  Kt_3ph = 1.5×2×0.000975 = 0.00293 N·m/A.
#
# HOSIM: from mc_app_hooks.c — 4.8A needed on smooth floor to break stiction.
#   T_coulomb = Kt_3ph × 4.8 = 0.0141 N·m.
#   At 8A (iter3): T_max = 0.0234 N·m > T_coulomb → motor can spin.
HOSIM  = DrivetrainLoad(T_coulomb=0.014, B_viscous=5e-5, J=8e-6)

# AMORIL: calibrated from rosbag2_2026_03_18-09_47_04 (best RUN session).
#   RUN Iq mean = 3.3A → viscous load = Kt_3ph × 3.3 ≈ 0.0097 N·m at 4000 RPM.
#   SWITCH_OVER observed at 2000–2500 RPM → T_coulomb < Kt_3ph × 6A = 0.0176 N·m.
AMORIL = DrivetrainLoad(T_coulomb=0.005, B_viscous=2.5e-5, J=4e-6)

MOTOR  = MotorParams()
TS_FOC = 40e-6   # 25 kHz FOC rate


# ─────────────────────────────────────────────────────────────────────────────
# PMSM Simulator — ground truth
# ─────────────────────────────────────────────────────────────────────────────

class PMSMSimulator:
    """
    αβ-frame PMSM model with Coulomb + viscous load.

    State: θ_e (electrical angle, rad), ω_m (mech speed, rad/s), iα, iβ (A)
    Input: Vα, Vβ (phase voltages, V)
    Integration: forward Euler at Ts.
    """

    def __init__(self, motor: MotorParams, load: DrivetrainLoad, Ts: float):
        self.m  = motor
        self.ld = load
        self.Ts = Ts
        self.theta_e = 0.0
        self.omega_m = 0.0
        self.ia = 0.0
        self.ib = 0.0

    def step(self, Va: float, Vb: float):
        """Advance one Ts. Returns (θ_e, ω_m, iα, iβ)."""
        m, ld, Ts = self.m, self.ld, self.Ts
        ω_e = m.p * self.omega_m
        θ   = self.theta_e

        # BEMF components in αβ
        ea = -m.psi_f * ω_e * np.sin(θ)
        eb =  m.psi_f * ω_e * np.cos(θ)

        # Electrical dynamics
        dia = (Va - m.Rs * self.ia - ea) / m.Ls
        dib = (Vb - m.Rs * self.ib - eb) / m.Ls

        # Electromagnetic torque — 3/2 factor from αβ (2-phase) → 3-phase conversion.
        # Te = (3/2)×p×Ψf×Iq in dq frame; equivalent αβ form below.
        Te = 1.5 * m.p * m.psi_f * (self.ib * np.cos(θ) - self.ia * np.sin(θ))

        # Mechanical load
        sign_w = np.sign(self.omega_m) if abs(self.omega_m) > 1e-3 else 0.0
        T_load = ld.T_coulomb * sign_w + ld.B_viscous * self.omega_m
        dom    = (Te - T_load) / ld.J

        # Euler integration
        self.ia      += dia * Ts
        self.ib      += dib * Ts
        self.omega_m += dom * Ts
        self.theta_e  = (self.theta_e + m.p * self.omega_m * Ts) % (2 * np.pi)

        return self.theta_e, self.omega_m, self.ia, self.ib

    @property
    def rpm(self):
        return self.omega_m * 60.0 / (2 * np.pi)


# ─────────────────────────────────────────────────────────────────────────────
# Open-loop rev-up controller (MCSDK START phase equivalent)
# ─────────────────────────────────────────────────────────────────────────────

class RevUpController:
    """
    Imposes stator field angle during open-loop rev-up.

    Voltage commanded in dq frame (field-oriented):
      Vd = -ω_e·Ls·Iq            (cross-coupling compensation)
      Vq =  Rs·Iq + ω_e·Ψf       (steady-state current maintenance)
    then inverse-Park → αβ using the imposed θ_ref.
    """

    def __init__(self, motor: MotorParams, Ts: float,
                 I_phase_A: float, target_rpm: float, ramp_s: float):
        self.m         = motor
        self.Ts        = Ts
        self.I         = I_phase_A
        self.omega_max = target_rpm * 2 * np.pi / 60.0 * motor.p   # electrical rad/s
        self.alpha     = self.omega_max / ramp_s                    # electrical rad/s²
        self.theta_ref = 0.0
        self.omega_ref = 0.0   # electrical rad/s

    def step(self):
        """Returns (Vα, Vβ) and advances internal angle."""
        m = self.m
        self.omega_ref = min(self.omega_ref + self.alpha * self.Ts, self.omega_max)
        self.theta_ref = (self.theta_ref + self.omega_ref * self.Ts) % (2 * np.pi)

        ω = self.omega_ref
        Vd = -ω * m.Ls * self.I
        Vq =  m.Rs * self.I + ω * m.psi_f

        cos_t = np.cos(self.theta_ref)
        sin_t = np.sin(self.theta_ref)
        Va = Vd * cos_t - Vq * sin_t
        Vb = Vd * sin_t + Vq * cos_t
        return Va, Vb


# ─────────────────────────────────────────────────────────────────────────────
# Luenberger + PLL observer  (simplified MCSDK STO equivalent)
# ─────────────────────────────────────────────────────────────────────────────

class LuenbergerPLL:
    """
    BEMF Luenberger observer with PLL angle extractor.

    The Luenberger drives BEMF estimates from the current prediction error.
    The PLL tracks the angle of the estimated BEMF vector.

    Gains derived from pole placement in SI (continuous-time Luenberger):
        Observer poles at α = 20000 rad/s (2× current loop bandwidth Rs/Ls = 10000).
        g1 = -α²×Ls = -(20000)²×10e-6 = -4000 V/(A·s)  [BEMF driven by current error]
        g2 = 2α - Rs/Ls = 40000 - 10000 = 30000 s⁻¹   [current model correction]

    PLL gains in SI (rad/s per rad, rad/s² per rad):
        Target bandwidth ωn = 300 rad/s (≈ electrical freq at 1600 RPM), ζ = 0.75.
        pll_ki = ωn² = 90 000   [rad/s² per rad]
        pll_kp = 2ζωn = 450     [rad/s  per rad]
        MCSDK values (GAIN1/F1, GAIN2/F2, PLL_KP/KP_DIV) are fixed-point integers
        in an internal unit system — they cannot be used directly as SI gains.
    """

    def __init__(self, motor: MotorParams, Ts: float,
                 g1: float = -4000.0,
                 g2: float =  30000.0,
                 pll_kp: float = 450.0,
                 pll_ki: float = 90000.0):
        self.m      = motor
        self.Ts     = Ts
        self.g1     = g1
        self.g2     = g2
        self.kp     = pll_kp
        self.ki     = pll_ki
        # Observer state
        self.ia_hat = 0.0
        self.ib_hat = 0.0
        self.ea_hat = 0.0
        self.eb_hat = 0.0
        # PLL state
        self.theta_hat  = 0.0
        self.omega_hat  = 0.0   # electrical rad/s
        self._pll_int   = 0.0

    def update(self, Va: float, Vb: float, ia_m: float, ib_m: float):
        m, Ts = self.m, self.Ts

        # Current error (using previous-step estimates)
        ia_err = ia_m - self.ia_hat
        ib_err = ib_m - self.ib_hat

        # Snapshot BEMF before updating (consistent Euler step)
        ea_old, eb_old = self.ea_hat, self.eb_hat

        # Luenberger BEMF update (driven by current error)
        self.ea_hat += Ts * self.g1 * ia_err
        self.eb_hat += Ts * self.g1 * ib_err

        # Current model prediction + Luenberger correction (use OLD BEMF)
        self.ia_hat += Ts * ((Va - m.Rs * self.ia_hat - ea_old) / m.Ls
                              + self.g2 * ia_err)
        self.ib_hat += Ts * ((Vb - m.Rs * self.ib_hat - eb_old) / m.Ls
                              + self.g2 * ib_err)

        # PLL: phase error from BEMF vector
        theta_bemf = np.arctan2(-self.ea_hat, self.eb_hat)
        err = _wrap(theta_bemf - self.theta_hat)

        # PI integrator
        self._pll_int  += Ts * self.ki * err
        self.omega_hat  = self._pll_int + self.kp * err
        self.theta_hat  = (self.theta_hat + Ts * self.omega_hat) % (2 * np.pi)

    @property
    def angle_deg(self):
        return np.degrees(self.theta_hat)

    @property
    def speed_rpm(self):
        return self.omega_hat * 60.0 / (2 * np.pi * self.m.p)


# ─────────────────────────────────────────────────────────────────────────────
# EKF observer
# ─────────────────────────────────────────────────────────────────────────────

class PMSM_EKF:
    """
    Extended Kalman Filter for PMSM sensorless control.

    State x = [iα, iβ, eα, eβ]
      eα = -Ψf·ω_e·sin(θ_e),   eβ = Ψf·ω_e·cos(θ_e)
    Recovered:
      θ_e = atan2(-eα, eβ)
      ω_e = √(eα²+eβ²) / Ψf

    Measurement y = [iα_measured, iβ_measured]

    Noise matrices:
      Q = diag([q_i, q_i, q_e, q_e])   process noise (model uncertainty)
      R = diag([r_i, r_i])              measurement noise (current ADC)

    Starting point:
      r_i = σ_I² = (0.0577 A)² = 3.33e-3 A²  (from noise_floor rosbag)
      q_i = r_i   (trust model as much as sensor)
      q_e = 10×r_i (BEMF dynamics less certain than current dynamics)
    """

    def __init__(self, motor: MotorParams, Ts: float,
                 q_i: float = 3.33e-3,
                 q_e: float = 3.33e-2,
                 r_i: float = 3.33e-3):
        self.m  = motor
        self.Ts = Ts
        # State vector [iα, iβ, eα, eβ]
        self.x = np.zeros(4)
        # Error covariance — start large (unknown initial state)
        self.P = np.eye(4) * 1.0
        # Noise matrices
        self.Q = np.diag([q_i, q_i, q_e, q_e])
        self.R = np.diag([r_i, r_i])
        # Measurement matrix: y = H·x = [iα, iβ]
        self.H = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.]])

    # ── Core EKF step ──────────────────────────────────────────────────────

    def update(self, Va: float, Vb: float, ia_m: float, ib_m: float):
        m, Ts = self.m, self.Ts
        ia, ib, ea, eb = self.x
        psi2 = m.psi_f ** 2

        # Electrical speed from current BEMF estimate
        omega_e = np.sqrt(ea**2 + eb**2) / (m.psi_f + 1e-30)
        omega_s = max(omega_e, 1e-10)   # safe ω (avoids /0 in Jacobian)

        # ── Predict ────────────────────────────────────────────────────────
        # Nonlinear state transition f(x, u)
        x_pred = np.array([
            ia + Ts / m.Ls * (Va - m.Rs * ia - ea),
            ib + Ts / m.Ls * (Vb - m.Rs * ib - eb),
            ea - omega_e * eb * Ts,
            eb + omega_e * ea * Ts,
        ])

        # Jacobian F = ∂f/∂x  (4×4)
        # Rows 0,1: current dynamics (linear in x)
        # Rows 2,3: BEMF rotation (nonlinear — requires ∂ω/∂eα, ∂ω/∂eβ)
        c = Ts / (psi2 * omega_s)            # common factor in BEMF Jacobian rows
        F = np.array([
            [1 - Ts * m.Rs / m.Ls,  0.,              -Ts / m.Ls,                         0.                           ],
            [0.,  1 - Ts * m.Rs / m.Ls,               0.,                               -Ts / m.Ls                    ],
            [0.,  0.,   1 - c * ea * eb,              -Ts * (omega_s + c * eb * eb)                                    ],
            [0.,  0.,   Ts * (omega_s + c * ea * ea),  1 + c * ea * eb                                                 ],
        ])

        P_pred = F @ self.P @ F.T + self.Q

        # ── Update ─────────────────────────────────────────────────────────
        y_innov = np.array([ia_m - x_pred[0],
                            ib_m - x_pred[1]])

        S = self.H @ P_pred @ self.H.T + self.R   # 2×2 innovation covariance
        K = P_pred @ self.H.T @ np.linalg.inv(S)  # 4×2 Kalman gain

        self.x = x_pred + K @ y_innov
        self.P = (np.eye(4) - K @ self.H) @ P_pred

    # ── Outputs ────────────────────────────────────────────────────────────

    @property
    def angle_rad(self):
        return np.arctan2(-self.x[2], self.x[3])

    @property
    def angle_deg(self):
        return np.degrees(self.angle_rad)

    @property
    def omega_e(self):
        ea, eb = self.x[2], self.x[3]
        return np.sqrt(ea**2 + eb**2) / (self.m.psi_f + 1e-30)

    @property
    def speed_rpm(self):
        return self.omega_e * 60.0 / (2 * np.pi * self.m.p)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _wrap(angle_rad: float) -> float:
    """Wrap angle to [-π, π]."""
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def _angle_err_deg(est_deg: float, true_rad: float) -> float:
    """Absolute angle error in degrees, wrapped to [0, 180]."""
    true_deg = np.degrees(true_rad)
    d = (est_deg - true_deg + 180) % 360 - 180
    return abs(d)


def _rpm_from_omega_m(omega_m: float) -> float:
    return omega_m * 60.0 / (2 * np.pi)


# ─────────────────────────────────────────────────────────────────────────────
# Simulation runner
# ─────────────────────────────────────────────────────────────────────────────

def run_simulation(load: DrivetrainLoad,
                   label: str,
                   I_phase_A:    float = 8.0,
                   target_rpm:   float = 1600.0,
                   ramp_s:       float = 3.0,
                   duration_s:   float = 5.0,
                   noise_sigma:  float = 0.0577,
                   Ts:           float = TS_FOC,
                   ekf_q_i:      float = 3.33e-3,
                   ekf_q_e:      float = 3.33e-2,
                   ekf_r_i:      float = 3.33e-3,
                   seed:         int   = 42) -> dict:
    """
    Run open-loop rev-up simulation. Returns a result dict for plotting.

    Parameters
    ----------
    noise_sigma : float
        Current measurement noise std [A]. Default = σ_Iq from noise_floor bag.
    """
    motor = MOTOR
    total = int(duration_s / Ts)

    sim  = PMSMSimulator(motor, load, Ts)
    sto  = LuenbergerPLL(motor, Ts)
    ekf  = PMSM_EKF(motor, Ts, q_i=ekf_q_i, q_e=ekf_q_e, r_i=ekf_r_i)
    ctrl = RevUpController(motor, Ts, I_phase_A, target_rpm, ramp_s)
    rng  = np.random.default_rng(seed)

    # Log every 1 ms (25 FOC steps)
    log_stride = max(1, int(1e-3 / Ts))
    n_log      = total // log_stride
    t_log      = np.empty(n_log)
    rpm_true   = np.empty(n_log)
    rpm_sto    = np.empty(n_log)
    rpm_ekf    = np.empty(n_log)
    err_sto    = np.empty(n_log)
    err_ekf    = np.empty(n_log)

    li = 0
    for k in range(total):
        Va, Vb = ctrl.step()
        theta_e, omega_m, ia, ib = sim.step(Va, Vb)

        ia_n = ia + rng.normal(0.0, noise_sigma)
        ib_n = ib + rng.normal(0.0, noise_sigma)

        sto.update(Va, Vb, ia_n, ib_n)
        ekf.update(Va, Vb, ia_n, ib_n)

        if k % log_stride == 0 and li < n_log:
            t_log[li]    = k * Ts
            rpm_true[li] = _rpm_from_omega_m(omega_m)
            rpm_sto[li]  = sto.speed_rpm
            rpm_ekf[li]  = ekf.speed_rpm
            err_sto[li]  = _angle_err_deg(sto.angle_deg,  theta_e)
            err_ekf[li]  = _angle_err_deg(ekf.angle_deg,  theta_e)
            li += 1

    return dict(t=t_log[:li], rpm_true=rpm_true[:li],
                rpm_sto=rpm_sto[:li],  rpm_ekf=rpm_ekf[:li],
                err_sto=err_sto[:li],  err_ekf=err_ekf[:li],
                label=label)


# ─────────────────────────────────────────────────────────────────────────────
# Analysis helpers
# ─────────────────────────────────────────────────────────────────────────────

ERR_THRESHOLD_DEG = 15.0   # observer considered "locked" when |err| < this


def lock_rpm(result: dict, observer: str,
             rpm_tol: float = 0.25, min_rpm: float = 100.0) -> float:
    """
    RPM at which the observer sustainably locks.

    Criteria (must hold for 100 consecutive ms):
      1. Angle error < ERR_THRESHOLD_DEG  (15°)
      2. Speed estimate within ±rpm_tol of true RPM  (25%)
      3. True RPM > min_rpm  (avoids startup coincidence where both start at θ=0)

    This mirrors MCSDK's NB_CONSECUTIVE_TESTS consistency check which validates
    both BEMF amplitude (i.e. speed) and angle before firing SWITCH_OVER.
    """
    err      = result[f'err_{observer}']
    rpm_true = result['rpm_true']
    rpm_obs  = result[f'rpm_{observer}']
    win = 100   # consecutive 1ms samples
    for i in range(len(err) - win):
        window_rpm = rpm_true[i:i+win]
        if window_rpm.min() < min_rpm:
            continue
        if not np.all(err[i:i+win] < ERR_THRESHOLD_DEG):
            continue
        rpm_err_rel = np.abs(rpm_obs[i:i+win] - window_rpm) / (window_rpm + 1e-6)
        if np.all(rpm_err_rel < rpm_tol):
            return window_rpm[0]
    return float('nan')


def print_summary(results: list):
    print()
    print(f"{'Drivetrain':<20} {'Observer':<18} {'Lock RPM':>10}  {'(< 15° sustained)'}")
    print("-" * 65)
    for res in results:
        for obs, name in [('sto', 'Luenberger+PLL'), ('ekf', 'EKF')]:
            rpm = lock_rpm(res, obs)
            rpm_str = f"{rpm:.0f}" if not np.isnan(rpm) else "never locked"
            print(f"  {res['label']:<18} {name:<18} {rpm_str:>10}")
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Q sensitivity sweep
# ─────────────────────────────────────────────────────────────────────────────

def sweep_q(load: DrivetrainLoad, label: str):
    """Show how EKF lock RPM varies with q_e (process noise for BEMF states)."""
    q_e_values = [1e-4, 1e-3, 1e-2, 1e-1, 1.0]
    print(f"\nQ sensitivity sweep — {label}")
    print(f"  {'q_e':<12} {'EKF lock RPM':>14}")
    print("  " + "-" * 28)
    for q_e in q_e_values:
        res = run_simulation(load, label, ekf_q_e=q_e, duration_s=4.0)
        rpm = lock_rpm(res, 'ekf')
        rpm_str = f"{rpm:.0f}" if not np.isnan(rpm) else "never"
        print(f"  {q_e:<12.0e} {rpm_str:>14}")


# ─────────────────────────────────────────────────────────────────────────────
# Plot
# ─────────────────────────────────────────────────────────────────────────────

def plot_results(results: list, out_path: str = "/tmp/ekf_comparison.png"):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available — skipping plot")
        return

    n = len(results)
    fig, axes = plt.subplots(2, n, figsize=(7 * n, 8))
    if n == 1:
        axes = axes.reshape(2, 1)

    for col, res in enumerate(results):
        t   = res['t']
        ax0 = axes[0][col]
        ax1 = axes[1][col]

        # Speed
        ax0.plot(t, res['rpm_true'], 'k-',  lw=1.5, label='True RPM',        zorder=3)
        ax0.plot(t, res['rpm_sto'],  'r--', lw=1.2, label='Luenberger+PLL',  zorder=2)
        ax0.plot(t, res['rpm_ekf'],  'b-',  lw=1.2, label='EKF',             zorder=2)
        ax0.axhline(1600, color='#888', ls=':', lw=1, label='OBS_MIN (1600)')
        ax0.set_title(f'{res["label"]} — Speed estimate', fontsize=11)
        ax0.set_ylabel('RPM')
        ax0.legend(fontsize=8)
        ax0.set_ylim(-300, max(2000, res['rpm_true'].max() * 1.1))
        ax0.grid(True, alpha=0.3)

        # Angle error (log scale)
        ax1.semilogy(t, res['err_sto'] + 0.1, 'r--', lw=1.2, label='Luenberger+PLL')
        ax1.semilogy(t, res['err_ekf'] + 0.1, 'b-',  lw=1.2, label='EKF')
        ax1.axhline(ERR_THRESHOLD_DEG, color='#888', ls=':', lw=1,
                    label=f'{ERR_THRESHOLD_DEG}° lock threshold')
        ax1.set_title(f'{res["label"]} — Angle error', fontsize=11)
        ax1.set_ylabel('|angle error| (deg)')
        ax1.set_xlabel('Time (s)')
        ax1.legend(fontsize=8)
        ax1.set_ylim(0.05, 200)
        ax1.grid(True, alpha=0.3, which='both')

    plt.suptitle('PMSM EKF vs Luenberger+PLL — open-loop rev-up to 1600 RPM\n'
                 f'Rs={MOTOR.Rs}Ω  Ls={MOTOR.Ls*1e6:.0f}μH  Ψf={MOTOR.psi_f:.4e}Wb  '
                 f'noise σ=57mA', fontsize=10)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved → {out_path}")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="PMSM EKF prototype")
    parser.add_argument('--sweep-q',  action='store_true',
                        help='Sweep q_e values to show Q sensitivity')
    parser.add_argument('--no-plot',  action='store_true',
                        help='Skip plot output')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='Simulation duration in seconds (default 5.0)')
    args = parser.parse_args()

    print("Running HOSIM simulation  (heavy drivetrain, 8A, 1600 RPM)...")
    r_hosim  = run_simulation(HOSIM,  label='HOSIM (heavy)',
                              I_phase_A=8.0, target_rpm=1600.0,
                              duration_s=args.duration)

    print("Running AMORIL simulation (lighter drivetrain, 6A, 2500 RPM)...")
    r_amoril = run_simulation(AMORIL, label='AMORIL (lighter)',
                              I_phase_A=6.0, target_rpm=2500.0,
                              duration_s=args.duration)

    print_summary([r_hosim, r_amoril])

    if args.sweep_q:
        sweep_q(HOSIM,  'HOSIM')
        sweep_q(AMORIL, 'AMORIL')

    if not args.no_plot:
        plot_results([r_hosim, r_amoril])


if __name__ == '__main__':
    main()
