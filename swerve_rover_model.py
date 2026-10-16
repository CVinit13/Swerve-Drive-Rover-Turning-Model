"""Reproducible reduced-order model for a 4-wheel swerve-drive rover.

This is a research prototype, not a validated digital twin. Several values are
explicitly marked ILLUSTRATIVE because the source PDF omits chassis mass,
module spacing, yaw inertia, wheel/module inertias, tire stiffness, and exact
motor torque-speed curves. Do not present the illustrative simulation outputs
as measured rover performance.

Dependencies: numpy, scipy, matplotlib.
Run: python swerve_rover_model.py --outdir results
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

G = 9.81


def wrap_pi(angle: float) -> float:
    """Wrap angle into [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class Params:
    # Values extracted from the supplied design PDF unless marked ILLUSTRATIVE.
    wheel_radius: float = 0.095             # m; PDF
    drive_teeth_sun: int = 23              # PDF
    drive_teeth_ring: int = 69             # PDF
    steer_teeth_sun: int = 18              # PDF
    steer_teeth_ring: int = 90             # PDF
    eta_stage_1: float = 0.92              # PDF assumption
    eta_stage_2: float = 0.92              # PDF assumption
    eta_steering: float = 0.92             # PDF assumption
    mu_traction: float = 0.7               # PDF
    mu_scrub: float = 0.7                  # PDF
    rolling_resistance_coeff: float = 0.02 # PDF
    scrub_radius: float = 0.015            # m; PDF, approximation
    grade_angle: float = 0.0               # rad; demo uses level ground

    # Missing physical parameters: placeholders for reproducible code execution.
    total_mass: float = 120.0              # kg; payload-only lower-bound illustration
    module_spacing: float = 0.80           # m; ILLUSTRATIVE, measure from CAD
    yaw_inertia: float = 12.8              # kg m^2; ILLUSTRATIVE uniform square mass
    wheel_inertia: float = 0.025           # kg m^2; ILLUSTRATIVE equivalent inertia
    steering_inertia: float = 0.08         # kg m^2; ILLUSTRATIVE per module
    tire_longitudinal_coeff: float = 500.0 # N s/m; ILLUSTRATIVE regularized model
    tire_lateral_coeff: float = 800.0      # N s/m; ILLUSTRATIVE regularized model
    wheel_viscous_damping: float = 0.01    # N m s/rad; ILLUSTRATIVE
    steering_viscous_damping: float = 0.05 # N m s/rad; ILLUSTRATIVE
    drive_motor_cont_torque: float = 1.3   # N m; recommended in PDF, not selected motor
    drive_motor_cont_power: float = 250.0  # W; recommended in PDF, not selected motor
    steer_motor_cont_torque: float = 1.0   # N m; recommended in PDF, not selected motor
    drive_speed_kp: float = 0.50           # N m per (rad/s); ILLUSTRATIVE controller
    steer_kp: float = 20.0                 # N m/rad; output-side control gain, illustrative
    steer_kd: float = 2.0                  # N m s/rad; output-side control gain, illustrative
    steer_alignment_gate: float = 0.30     # rad; ILLUSTRATIVE
    slip_regularization_speed: float = 0.03# m/s; ILLUSTRATIVE
    scrub_regularization_rate: float = 0.02# rad/s; ILLUSTRATIVE

    @property
    def drive_ratio_signed(self) -> float:
        # Stage 1: +1/4; stage 2: -1/3 => overall -1/12 speed ratio.
        return -((self.drive_teeth_sun / (self.drive_teeth_sun + self.drive_teeth_ring))
                 * (self.drive_teeth_sun / self.drive_teeth_ring))

    @property
    def drive_reduction_magnitude(self) -> float:
        return 1.0 / abs(self.drive_ratio_signed)

    @property
    def drive_efficiency(self) -> float:
        return self.eta_stage_1 * self.eta_stage_2

    @property
    def steer_ratio(self) -> float:
        # Fixed ring, carrier output: omega_c/omega_s = Zs/(Zs+Zr)
        return (self.steer_teeth_sun + self.steer_teeth_ring) / self.steer_teeth_sun

    @property
    def normal_load_each(self) -> float:
        # Equal static load approximation; ignores load transfer and CG offset.
        return self.total_mass * G * math.cos(self.grade_angle) / 4.0

    @property
    def scrub_torque_each(self) -> float:
        # Uniform pressure over a circular patch: M = (2/3) mu N R.
        return (2.0 / 3.0) * self.mu_scrub * self.normal_load_each * self.scrub_radius

    def module_positions(self) -> np.ndarray:
        a = self.module_spacing / 2.0
        # body +x forward, +y left; order: front-left, front-right, rear-left, rear-right
        return np.array([[ a,  a], [ a, -a], [-a,  a], [-a, -a]], dtype=float)


def planetary_speed_ratios() -> Dict[str, float]:
    """Return signed ratios omega_output/omega_input from Willis equation."""
    # Drive stage 1: ring fixed, sun input, carrier output.
    r1 = 23.0 / (23.0 + 69.0)
    # Drive stage 2: carrier fixed, sun input, ring output.
    r2 = -23.0 / 69.0
    # Steering: ring fixed, sun input, carrier output.
    rs = 18.0 / (18.0 + 90.0)
    return {"drive_stage_1_carrier_per_sun": r1,
            "drive_stage_2_ring_per_sun": r2,
            "drive_overall_wheel_per_motor": r1 * r2,
            "steer_carrier_per_sun": rs}


def inverse_kinematics(vx: float, vy: float, yaw_rate: float,
                       positions: np.ndarray, wheel_radius: float,
                       current_angles: np.ndarray | None = None) -> Tuple[np.ndarray, np.ndarray]:
    """Ideal inverse swerve kinematics, with shortest steering rotation.

    Returns (steering angle target [rad], wheel angular speed target [rad/s]).
    Steering target is represented continuously relative to current angle.
    """
    if current_angles is None:
        current_angles = np.zeros(4)
    deltas = np.array(current_angles, dtype=float).copy()
    omegas = np.zeros(4, dtype=float)
    for i, (xi, yi) in enumerate(positions):
        vix = vx - yaw_rate * yi
        viy = vy + yaw_rate * xi
        speed = math.hypot(vix, viy)
        if speed < 1e-9:
            # Steering direction is undefined at zero local wheel velocity.
            deltas[i] = current_angles[i]
            omegas[i] = 0.0
            continue
        base = math.atan2(viy, vix)
        e_forward = wrap_pi(base - current_angles[i])
        e_reverse = wrap_pi(base + math.pi - current_angles[i])
        if abs(e_forward) <= abs(e_reverse):
            deltas[i] = current_angles[i] + e_forward
            sign = 1.0
        else:
            deltas[i] = current_angles[i] + e_reverse
            sign = -1.0
        # Projection avoids inconsistencies caused by angle wrapping.
        v_along = vix * math.cos(deltas[i]) + viy * math.sin(deltas[i])
        omegas[i] = v_along / wheel_radius
    return deltas, omegas


def operating_point(mass_per_module: float = 30.0, acceleration: float = 0.8,
                    grade_deg: float = 5.0) -> Dict[str, float]:
    """Source-style per-module longitudinal operating point, corrected for grade cosine in N."""
    alpha = math.radians(grade_deg)
    n = mass_per_module * G * math.cos(alpha)
    f_roll = 0.02 * n
    f_grade = mass_per_module * G * math.sin(alpha)
    f_accel = mass_per_module * acceleration
    f_required = f_roll + f_grade + f_accel
    f_traction_max = 0.7 * n
    r = 0.095
    eta = 0.92**2
    ratio = 12.0
    wheel_torque = f_required * r
    motor_torque = wheel_torque / (ratio * eta)
    wheel_omega = 1.2 / r
    motor_omega = ratio * wheel_omega
    motor_power = motor_torque * motor_omega
    return {"normal_load_per_module_N": n,
            "rolling_resistance_per_module_N": f_roll,
            "grade_force_per_module_N": f_grade,
            "acceleration_force_per_module_N": f_accel,
            "required_drive_force_per_module_N": f_required,
            "traction_limit_per_module_N": f_traction_max,
            "traction_utilization_fraction": f_required / f_traction_max,
            "wheel_torque_per_module_Nm": wheel_torque,
            "motor_torque_per_module_Nm": motor_torque,
            "wheel_speed_at_1p2mps_rad_s": 1.2 / r,
            "wheel_speed_at_1p2mps_rpm": (1.2 / r) * 60.0 / (2.0 * math.pi),
            "motor_speed_magnitude_rad_s": motor_omega,
            "motor_speed_magnitude_rpm": motor_omega * 60.0 / (2.0 * math.pi),
            "motor_power_per_module_W": motor_power,
            "four_module_motor_power_W": 4.0 * motor_power}


def contact_forces(u: float, v: float, yaw_rate: float, omegas: np.ndarray,
                   deltas: np.ndarray, p: Params) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute forces with a low-speed regularized creepage model and friction circle.

    Returns: body-frame wheel forces [4,2], gross longitudinal traction [4],
    lateral tire forces [4], and steering scrub moments [4].
    The tire coefficients are illustrative and require experimental identification.
    """
    pos = p.module_positions()
    N = p.normal_load_each
    body_forces = np.zeros((4, 2), dtype=float)
    gross_long = np.zeros(4, dtype=float)
    lateral = np.zeros(4, dtype=float)
    scrub = np.zeros(4, dtype=float)
    for i, (xi, yi) in enumerate(pos):
        vix = u - yaw_rate * yi
        viy = v + yaw_rate * xi
        c, s = math.cos(deltas[i]), math.sin(deltas[i])
        v_long = vix * c + viy * s
        v_lat = -vix * s + viy * c
        f_rr = p.rolling_resistance_coeff * N * math.tanh(v_long / p.slip_regularization_speed)
        f_long_raw = p.tire_longitudinal_coeff * (p.wheel_radius * omegas[i] - v_long)
        f_lat_raw = -p.tire_lateral_coeff * v_lat
        net_long_raw = f_long_raw - f_rr
        demand = math.hypot(net_long_raw, f_lat_raw)
        scale = min(1.0, (p.mu_traction * N / demand)) if demand > 1e-12 else 1.0
        f_net_long = scale * net_long_raw
        f_lat = scale * f_lat_raw
        f_traction = f_net_long + f_rr
        gross_long[i] = f_traction
        lateral[i] = f_lat
        # body transform of net contact force; longitudinal rolling resistance is included.
        body_forces[i, 0] = f_net_long * c - f_lat * s
        body_forces[i, 1] = f_net_long * s + f_lat * c
        scrub[i] = p.scrub_torque_each * math.tanh(
            0.0 / p.scrub_regularization_rate)  # overwritten with steering rate in RHS
    return body_forces, gross_long, lateral, scrub


def simulate(command: Tuple[float, float, float], duration: float = 4.0,
             p: Params | None = None, sample_dt: float = 0.01,
             steering_override: np.ndarray | None = None) -> Dict[str, object]:
    """Closed-loop numerical demonstration for a constant desired body twist.

    State: [X,Y,psi,u,v,r, wheel_omega(4), steer_delta(4), steer_rate(4)].
    The inverse kinematics generate targets; bounded motor-side torque commands
    drive the steering and wheel states. Tire model is deliberately reduced-order.
    """
    if p is None:
        p = Params()
    positions = p.module_positions()
    vd = np.asarray(command, dtype=float)
    y0 = np.zeros(18, dtype=float)
    ts = np.arange(0.0, duration + sample_dt * 0.5, sample_dt)

    def rhs(t: float, y: np.ndarray) -> np.ndarray:
        X, Y, psi, u, v, yaw = y[:6]
        omega = y[6:10]
        delta = y[10:14]
        delta_dot = y[14:18]
        if steering_override is None:
            delta_des, omega_des = inverse_kinematics(vd[0], vd[1], vd[2], positions,
                                                       p.wheel_radius, delta)
        else:
            # Explicit steering target permits stationary wheel pre-positioning.
            delta_des = np.asarray(steering_override, dtype=float)
            omega_des = np.zeros(4, dtype=float)
        delta_err = delta_des - delta
        # Steering motor torque: output-side PD + smooth Coulomb-friction compensation.
        tau_scrub = p.scrub_torque_each * np.tanh(delta_dot / p.scrub_regularization_rate)
        tau_steer_out_req = (p.steer_kp * delta_err - p.steer_kd * delta_dot
                             + tau_scrub * np.tanh(delta_err / 0.02))
        tau_sm_lim = p.steer_motor_cont_torque
        tau_sm = np.clip(tau_steer_out_req / (p.steer_ratio * p.eta_steering),
                         -tau_sm_lim, tau_sm_lim)
        tau_steer_out = p.steer_ratio * p.eta_steering * tau_sm
        delta_ddot = (tau_steer_out - tau_scrub - p.steering_viscous_damping * delta_dot) / p.steering_inertia

        # Reduce drive commands while steering direction is far from its target.
        alignment = np.exp(-0.5 * (delta_err / p.steer_alignment_gate) ** 2)
        omega_target = omega_des * alignment
        # Torque feed-forward covers level-ground rolling resistance and the grade component.
        frr_guess = p.rolling_resistance_coeff * p.normal_load_each
        grade_share = p.total_mass * G * math.sin(p.grade_angle) / 4.0
        tau_ff_wheel = p.wheel_radius * (frr_guess + grade_share)
        tau_wheel_req = tau_ff_wheel + p.drive_speed_kp * (omega_target - omega)
        motor_speed_abs = p.drive_reduction_magnitude * np.abs(omega)
        power_torque_limit = p.drive_motor_cont_power / np.maximum(motor_speed_abs, 1.0)
        motor_torque_limit = np.minimum(p.drive_motor_cont_torque, power_torque_limit)
        wheel_torque_limit = p.drive_reduction_magnitude * p.drive_efficiency * motor_torque_limit
        tau_wheel = np.clip(tau_wheel_req, -wheel_torque_limit, wheel_torque_limit)
        # Stage 2 reverses rotation; tau_w = -|i|*eta*tau_motor for common signed axes.
        tau_motor = -tau_wheel / (p.drive_reduction_magnitude * p.drive_efficiency)
        tau_wheel_actual = -p.drive_reduction_magnitude * p.drive_efficiency * tau_motor

        body_forces, gross_long, lateral, _ = contact_forces(u, v, yaw, omega, delta, p)
        # Coulomb-style external tire-patch torsional resistance during relative steering.
        tau_scrub_ext = p.scrub_torque_each * np.tanh(delta_dot / p.scrub_regularization_rate)
        # Chassis forces include rolling resistance already; grade is separate external force.
        Fx = float(np.sum(body_forces[:, 0]) - p.total_mass * G * math.sin(p.grade_angle))
        Fy = float(np.sum(body_forces[:, 1]))
        Mz_force = float(np.sum(positions[:, 0] * body_forces[:, 1]
                                - positions[:, 1] * body_forces[:, 0]))
        Mz_scrub = -float(np.sum(tau_scrub_ext))
        u_dot = Fx / p.total_mass + yaw * v
        v_dot = Fy / p.total_mass - yaw * u
        yaw_dot = (Mz_force + Mz_scrub) / p.yaw_inertia
        omega_dot = (tau_wheel_actual - p.wheel_radius * gross_long
                     - p.wheel_viscous_damping * omega) / p.wheel_inertia
        X_dot = u * math.cos(psi) - v * math.sin(psi)
        Y_dot = u * math.sin(psi) + v * math.cos(psi)
        psi_dot = yaw
        return np.concatenate(([X_dot, Y_dot, psi_dot, u_dot, v_dot, yaw_dot],
                               omega_dot, delta_dot, delta_ddot))

    sol = solve_ivp(rhs, (0.0, duration), y0, t_eval=ts, method="RK45",
                    rtol=2e-6, atol=1e-8, max_step=min(0.005, sample_dt))
    if not sol.success:
        raise RuntimeError(f"Simulation failed: {sol.message}")

    # Diagnostics are evaluated at saved output times, not at the solver's internal steps.
    state_out = sol.y.T
    friction_ratios = []
    motor_saturation = []
    for row in state_out:
        u, v, yaw = row[3], row[4], row[5]
        omega, delta = row[6:10], row[10:14]
        if steering_override is None:
            delta_des, omega_des = inverse_kinematics(vd[0], vd[1], vd[2], positions,
                                                       p.wheel_radius, delta)
        else:
            delta_des = np.asarray(steering_override, dtype=float)
            omega_des = np.zeros(4, dtype=float)
        delta_err = delta_des - delta
        alignment = np.exp(-0.5 * (delta_err / p.steer_alignment_gate) ** 2)
        omega_target = omega_des * alignment
        frr_guess = p.rolling_resistance_coeff * p.normal_load_each
        grade_share = p.total_mass * G * math.sin(p.grade_angle) / 4.0
        tau_ff_wheel = p.wheel_radius * (frr_guess + grade_share)
        tau_wheel_req = tau_ff_wheel + p.drive_speed_kp * (omega_target - omega)
        motor_speed_abs = p.drive_reduction_magnitude * np.abs(omega)
        power_torque_limit = p.drive_motor_cont_power / np.maximum(motor_speed_abs, 1.0)
        motor_torque_limit = np.minimum(p.drive_motor_cont_torque, power_torque_limit)
        wheel_torque_limit = p.drive_reduction_magnitude * p.drive_efficiency * motor_torque_limit
        motor_saturation.append(np.abs(tau_wheel_req) >= (wheel_torque_limit * (1.0 - 1e-5)))

        _, gross_long, lateral, _ = contact_forces(u, v, yaw, omega, delta, p)
        per_wheel = []
        for i, (xi, yi) in enumerate(positions):
            vix = u - yaw * yi
            viy = v + yaw * xi
            v_long = vix * math.cos(delta[i]) + viy * math.sin(delta[i])
            f_rr = p.rolling_resistance_coeff * p.normal_load_each * math.tanh(
                v_long / p.slip_regularization_speed)
            net_long = gross_long[i] - f_rr
            per_wheel.append(math.hypot(net_long, lateral[i]) /
                              max(p.mu_traction * p.normal_load_each, 1e-12))
        friction_ratios.append(per_wheel)

    friction_ratios = np.asarray(friction_ratios)
    motor_saturation = np.asarray(motor_saturation, dtype=bool)
    diagnostics = {
        "peak_friction_utilization": float(np.max(friction_ratios)),
        "fraction_saved_samples_any_wheel_motor_torque_saturated": float(np.mean(np.any(motor_saturation, axis=1))),
        "fraction_saved_samples_any_wheel_at_friction_limit": float(np.mean(np.any(friction_ratios >= 0.999, axis=1))),
    }
    return {"t": sol.t, "state": state_out, "command": np.asarray(command, dtype=float),
            "positions": positions, "params": p, "diagnostics": diagnostics}


def verify_analytical_checks() -> Dict[str, bool]:
    ratios = planetary_speed_ratios()
    checks = {
        "drive_stage1_is_plus_one_quarter": abs(ratios["drive_stage_1_carrier_per_sun"] - 0.25) < 1e-12,
        "drive_stage2_is_minus_one_third": abs(ratios["drive_stage_2_ring_per_sun"] + 1.0/3.0) < 1e-12,
        "combined_drive_is_minus_one_twelfth": abs(ratios["drive_overall_wheel_per_motor"] + 1.0/12.0) < 1e-12,
        "steering_is_plus_one_sixth": abs(ratios["steer_carrier_per_sun"] - 1.0/6.0) < 1e-12,
    }
    p = Params()
    # Straight translation: all local vectors point +x, all wheel angles 0.
    d, w = inverse_kinematics(1.0, 0.0, 0.0, p.module_positions(), p.wheel_radius, np.zeros(4))
    checks["straight_kinematics_zero_angles"] = bool(np.allclose(d, 0.0, atol=1e-12))
    checks["straight_kinematics_equal_speeds"] = bool(np.allclose(w, 1.0/p.wheel_radius, atol=1e-12))
    # In-place rotation: local speed at each corner = |yaw|*L/sqrt(2).
    _, wrot = inverse_kinematics(0.0, 0.0, 1.0, p.module_positions(), p.wheel_radius, np.zeros(4))
    expected = (p.module_spacing/math.sqrt(2.0))/p.wheel_radius
    checks["in_place_corner_speed_magnitude"] = bool(np.allclose(np.abs(wrot), expected, atol=1e-12))
    # Gearbox power check with source operating point.
    op = operating_point()
    wheel_power = op["required_drive_force_per_module_N"] * 1.2
    checks["drive_power_efficiency_consistent"] = abs(op["motor_power_per_module_W"] * p.drive_efficiency - wheel_power) < 1.0
    return checks


def make_figures(simulations: Dict[str, Dict[str, np.ndarray]], p: Params, outdir: Path) -> None:
    outdir.mkdir(parents=True, exist_ok=True)
    pos = p.module_positions()
    # Figure 1: square rover geometry and body-frame velocity vectors for in-place rotation.
    fig, ax = plt.subplots(figsize=(7, 6))
    a = p.module_spacing / 2.0
    square = np.array([[a,a], [a,-a], [-a,-a], [-a,a], [a,a]])
    ax.plot(square[:,0], square[:,1], linewidth=2)
    labels = ["FL", "FR", "RL", "RR"]
    for i, (x, y) in enumerate(pos):
        ax.scatter([x],[y],s=80)
        ax.text(x + 0.035, y + 0.035, labels[i], fontsize=10)
        vx = -0.5 * y
        vy = 0.5 * x
        ax.arrow(x, y, 0.23*vx, 0.23*vy, head_width=0.025, length_includes_head=True)
    ax.scatter([0],[0],marker="+",s=120)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Body x [m]")
    ax.set_ylabel("Body y [m]")
    ax.set_title("Square module geometry and local velocities for positive yaw")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(outdir / "fig1_rover_geometry.png", dpi=180)
    plt.close(fig)

    # Figure 5: conceptual planar free-body diagram and wheel-frame force decomposition.
    fig, (ax, axw) = plt.subplots(1, 2, figsize=(11.5, 6.0),
                                  gridspec_kw={"width_ratios": [1.15, 0.85]})
    a = p.module_spacing / 2.0
    square = np.array([[a,a], [a,-a], [-a,-a], [-a,a], [a,a]])
    ax.plot(square[:,0], square[:,1], color="black", linewidth=2)
    labels = ["FL", "FR", "RL", "RR"]
    for i, (x, y) in enumerate(pos):
        ax.scatter([x], [y], s=50, color="black", zorder=4)
        ax.text(x - (0.065 if x >= 0 else -0.065),
                y - (0.055 if y >= 0 else -0.055), labels[i], fontsize=9,
                ha="right" if x >= 0 else "left", va="top" if y >= 0 else "bottom")
        # Tangential force is a schematic positive-yaw example, not measured data.
        tangent = np.array([-y, x], dtype=float)
        tangent = tangent / np.linalg.norm(tangent)
        ax.arrow(x, y, 0.19*tangent[0], 0.19*tangent[1],
                 head_width=0.023, length_includes_head=True, color="tab:blue", zorder=3)
        ax.text(x + 0.225*tangent[0], y + 0.225*tangent[1], f"F_{labels[i]}",
                fontsize=8.5, color="tab:blue", ha="center", va="center")
    ax.scatter([0], [0], marker="+", s=140, color="black", zorder=5)
    ax.text(0.025, -0.035, "CG", fontsize=9)
    ax.annotate("", xy=(0.19,0), xytext=(0.04,0), arrowprops=dict(arrowstyle="->", color="black"))
    ax.annotate("", xy=(0,0.19), xytext=(0,0.04), arrowprops=dict(arrowstyle="->", color="black"))
    ax.text(0.19, -0.035, "+x", fontsize=8)
    ax.text(0.02, 0.20, "+y", fontsize=8)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-0.69, 0.69); ax.set_ylim(-0.69, 0.69)
    ax.set_xlabel("Body x coordinate [m]"); ax.set_ylabel("Body y coordinate [m]")
    ax.set_title("Planar chassis free body")
    ax.grid(True, alpha=0.2)

    axw.set_xlim(-0.15, 1.1); axw.set_ylim(-0.58, 1.0); axw.axis("off")
    axw.set_title("Wheel-aligned force decomposition")
    origin = np.array([0.2, 0.47])
    axw.annotate("", xy=(0.92,origin[1]), xytext=(origin[0],origin[1]),
                 arrowprops=dict(arrowstyle="->", color="black", lw=1.2))
    axw.text(0.79, origin[1]-0.11, "+x", fontsize=9)
    d = math.radians(32)
    e_t = np.array([math.cos(d), math.sin(d)])
    e_l = np.array([-math.sin(d), math.cos(d)])
    axw.plot([origin[0]-0.07*e_t[0], origin[0]+0.07*e_t[0]],
             [origin[1]-0.07*e_t[1], origin[1]+0.07*e_t[1]], color="0.3", linewidth=5)
    axw.arrow(origin[0], origin[1], 0.48*e_t[0], 0.48*e_t[1], head_width=0.035,
              length_includes_head=True, color="tab:blue")
    axw.arrow(origin[0], origin[1], 0.42*e_l[0], 0.42*e_l[1], head_width=0.035,
              length_includes_head=True, color="tab:orange")
    axw.text(origin[0]+0.50*e_t[0], origin[1]+0.50*e_t[1], r"$F_{ti}$", fontsize=10, color="tab:blue")
    axw.text(origin[0]+0.44*e_l[0]-0.02, origin[1]+0.44*e_l[1], r"$F_{li}$", fontsize=10, color="tab:orange")
    axw.text(0.32, 0.53, r"$\delta_i$", fontsize=10)
    axw.text(-0.02, 0.08, r"$F_{ix}=F_{ti}\cos\delta_i-F_{li}\sin\delta_i$", fontsize=9)
    axw.text(-0.02, -0.03, r"$F_{iy}=F_{ti}\sin\delta_i+F_{li}\cos\delta_i$", fontsize=9)
    axw.text(-0.02, -0.26, r"$M_z=\sum_i (x_iF_{iy}-y_iF_{ix})$", fontsize=11,
             bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="0.5"))
    axw.text(-0.02, -0.46, "Forces and dimensions are schematic; use measured wheel positions and forces.",
             fontsize=8, wrap=True)
    fig.tight_layout()
    fig.savefig(outdir / "fig5_rover_free_body.png", dpi=180)
    plt.close(fig)

    # Figure 2: model architecture, programmatically drawn.
    fig, ax = plt.subplots(figsize=(12, 3.8))
    ax.set_xlim(0, 12); ax.set_ylim(0, 4); ax.axis("off")
    blocks = [
        (0.2, 1.25, 1.8, 1.5, "Twist / torque\ncommands"),
        (2.35, 1.25, 1.8, 1.5, "Inverse swerve\nkinematics"),
        (4.5, 1.25, 1.8, 1.5, "Motor + signed\nplanetary ratios"),
        (6.65, 1.25, 1.8, 1.5, "Tire forces +\nfriction circle"),
        (8.8, 1.25, 1.3, 1.5, "Newton-Euler\ndynamics"),
        (10.45, 1.25, 1.3, 1.5, "Pose +\nvelocity")]
    for x,y,w,h,lab in blocks:
        rect=plt.Rectangle((x,y),w,h,fill=False,linewidth=1.5)
        ax.add_patch(rect); ax.text(x+w/2,y+h/2,lab,ha="center",va="center",fontsize=9)
    for i in range(len(blocks)-1):
        x1=blocks[i][0]+blocks[i][2]; x2=blocks[i+1][0]
        ax.annotate("", xy=(x2,2.0), xytext=(x1,2.0), arrowprops=dict(arrowstyle="->",lw=1.5))
    ax.annotate("feedback: wheel speeds, steering angles, body state",xy=(3.2,0.65),xytext=(9.6,0.65),
                ha="center",fontsize=9,arrowprops=dict(arrowstyle="->",connectionstyle="angle,angleA=0,angleB=90,rad=0.2"))
    ax.set_title("Reduced-order coupled model architecture", fontsize=12)
    fig.tight_layout()
    fig.savefig(outdir / "fig2_model_architecture.png", dpi=180)
    plt.close(fig)

    # Figure 3: signed gear speed map. Drive stages are connected; steering is separate.
    fig, ax = plt.subplots(figsize=(10, 4.6))
    ax.axis("off"); ax.set_xlim(0,10); ax.set_ylim(0,4.6)
    boxes = [(0.55,2.25,3.35,1.35,"Drive stage 1", "sun input; ring fixed; carrier output"),
             (6.05,2.25,3.35,1.35,"Drive stage 2", "sun input; carrier fixed; ring output"),
             (3.55,0.35,2.90,1.05,"Steering stage", "sun input; ring fixed; carrier output")]
    for x,y,w,h,title,subtitle in boxes:
        ax.add_patch(plt.Rectangle((x,y),w,h,fill=False,lw=1.5))
        ax.text(x+w/2,y+h*0.68,title,ha="center",va="center",weight="bold")
        ax.text(x+w/2,y+h*0.30,subtitle,ha="center",va="center",fontsize=8.5)
    ax.annotate("",xy=(6.05,2.91),xytext=(3.90,2.91),arrowprops=dict(arrowstyle="->",lw=1.5))
    ax.text(4.97,3.35,"ωc1 / ωm = +1/4",ha="center",fontsize=9.2)
    ax.text(5.02,2.47,"ωwheel / ωc1 = -1/3",ha="center",fontsize=9.2)
    ax.text(5.00,1.75,"Combined drive: ωwheel / ωmotor = -1/12 (magnitude 12:1)",
            ha="center",fontsize=9.5)
    ax.text(5.00,0.12,"Steering: ωcarrier / ωsun = +1/6",ha="center",fontsize=9)
    ax.set_title("Signed planetary speed relationships from the Willis equation", fontsize=12)
    fig.tight_layout()
    fig.savefig(outdir / "fig3_signed_gear_ratios.png", dpi=180)
    plt.close(fig)

    # Figure 4: command tracking (illustrative simulation only).
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for name in ("straight", "crab", "curve", "in_place"):
        if name not in simulations:
            continue
        sim = simulations[name]
        t = sim["t"]; state = sim["state"]
        axes[0].plot(t, state[:,3], label=f"{name}: u")
        axes[0].plot(t, state[:,4], linestyle="--", label=f"{name}: v")
        axes[1].plot(t, state[:,5], label=f"{name}: yaw rate")
    axes[0].set_ylabel("Body velocity [m/s]"); axes[0].grid(True,alpha=.3); axes[0].legend(ncol=2,fontsize=8)
    axes[1].set_ylabel("Yaw rate [rad/s]"); axes[1].set_xlabel("Time [s]"); axes[1].grid(True,alpha=.3); axes[1].legend(fontsize=8)
    fig.suptitle("Controller-in-the-loop demonstration (illustrative parameters, not validated rover results)")
    fig.tight_layout()
    fig.savefig(outdir / "fig4_demo_simulation.png", dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--outdir", default="results", help="Output folder")
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    p = Params()
    checks = verify_analytical_checks()
    if not all(checks.values()):
        failed = [k for k,v in checks.items() if not v]
        raise AssertionError(f"Analytical checks failed: {failed}")
    op = operating_point()
    (out / "analytical_checks.json").write_text(json.dumps(checks, indent=2))
    (out / "operating_point.json").write_text(json.dumps(op, indent=2))
    (out / "parameter_config.json").write_text(json.dumps(asdict(p), indent=2))
    # Stress tests deliberately alter placeholder values and are not claims about the real rover.
    tangential_steer = np.array([3*math.pi/4, math.pi/4, -3*math.pi/4, -math.pi/4])
    p_traction_limited = replace(p, mu_traction=0.10)
    p_motor_limited = replace(p, drive_motor_cont_torque=0.12, drive_motor_cont_power=60.0)
    cases = {
        "straight": ((0.6,0.0,0.0), p, None),
        "crab": ((0.0,0.3,0.0), p, None),
        "curve": ((0.5,0.0,0.25), p, None),
        "in_place": ((0.0,0.0,0.4), p, None),
        "steer_stationary": ((0.0,0.0,0.0), p, tangential_steer),
        "traction_limited_turn": ((0.7,0.0,1.5), p_traction_limited, None),
        "motor_torque_limited_turn": ((0.7,0.0,0.8), p_motor_limited, None),
    }
    simulations = {}
    summary_rows = []
    case_config_rows = {}
    for name, (command, case_p, steer_override) in cases.items():
        sim = simulate(command, duration=args.duration, p=case_p, steering_override=steer_override)
        simulations[name] = sim
        state=sim["state"]
        summary_rows.append({"case":name,"requested_u_m_s":command[0],"requested_v_m_s":command[1],
                             "requested_yaw_rad_s":command[2],"total_mass_kg":case_p.total_mass,
                             "mu_traction":case_p.mu_traction,"drive_motor_cont_torque_Nm":case_p.drive_motor_cont_torque,
                             "final_X_m":state[-1,0],"final_Y_m":state[-1,1],
                             "final_heading_rad":state[-1,2],"final_u_m_s":state[-1,3],"final_v_m_s":state[-1,4],
                             "final_yaw_rate_rad_s":state[-1,5],"peak_abs_yaw_rate_rad_s":float(np.max(np.abs(state[:,5]))),
                             "peak_abs_body_speed_m_s":float(np.max(np.hypot(state[:,3],state[:,4]))),
                             **sim["diagnostics"]})
        case_config_rows[name] = {"command_body_twist": list(command),
                                  "steering_override_rad": None if steer_override is None else steer_override.tolist(),
                                  "parameters": asdict(case_p),
                                  "interpretation": ("baseline illustrative case" if name in {"straight","crab","curve","in_place"}
                                                     else "deliberate placeholder stress test; not a claimed design operating point")}
        arr=np.column_stack((sim["t"],state))
        header="t,X,Y,psi,u,v,yaw_rate,omega_FL,omega_FR,omega_RL,omega_RR,delta_FL,delta_FR,delta_RL,delta_RR,delta_dot_FL,delta_dot_FR,delta_dot_RL,delta_dot_RR"
        np.savetxt(out/f"simulation_{name}.csv",arr,delimiter=",",header=header,comments="")
    (out / "case_configs.json").write_text(json.dumps(case_config_rows, indent=2))

    numerical_checks = {
        "all_saved_simulation_states_finite": bool(all(np.isfinite(sim["state"]).all() for sim in simulations.values())),
        "friction_circle_constraint_respected_at_saved_samples": bool(max(sim["diagnostics"]["peak_friction_utilization"] for sim in simulations.values()) <= 1.0 + 1e-6),
        "straight_line_baseline_has_no_unintended_yaw": bool(abs(simulations["straight"]["state"][-1,5]) < 1e-8),
        "stationary_steering_case_has_zero_requested_body_twist": bool(np.allclose(cases["steer_stationary"][0], (0.0,0.0,0.0))),
        "low_friction_stress_case_reaches_traction_boundary": bool(simulations["traction_limited_turn"]["diagnostics"]["peak_friction_utilization"] >= 0.999),
        "low_motor_torque_stress_case_activates_torque_limit": bool(simulations["motor_torque_limited_turn"]["diagnostics"]["fraction_saved_samples_any_wheel_motor_torque_saturated"] > 0.0),
    }
    (out / "numerical_checks.json").write_text(json.dumps(numerical_checks, indent=2))
    if not all(numerical_checks.values()):
        failed = [name for name, ok in numerical_checks.items() if not ok]
        raise AssertionError(f"Numerical simulation checks failed: {failed}")

    import csv
    with (out / "simulation_summary.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(summary_rows[0].keys()))
        writer.writeheader(); writer.writerows(summary_rows)
    make_figures(simulations,p,out.parent/"figures")
    print("ANALYTICAL CHECKS")
    print(json.dumps(checks,indent=2))
    print("PER-MODULE OPERATING POINT")
    print(json.dumps(op,indent=2))
    print("NUMERICAL SIMULATION CHECKS")
    print(json.dumps(numerical_checks, indent=2))
    print("SIMULATION SUMMARY (illustrative only)")
    print(json.dumps(summary_rows,indent=2))

if __name__ == "__main__":
    main()
