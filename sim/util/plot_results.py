#!/usr/bin/env python3
"""
Missile Simulation Result Plotter

Generates validation plots from HDF5 simulation output.
Handles both unguided and guided scenarios automatically.

Usage:
    python plot_results.py                              # default: sim_output.h5
    python plot_results.py guided_output.h5             # specific file
    python plot_results.py guided_output.h5 run_0001    # specific run
    python plot_results.py --save guided_output.h5      # save PNGs instead of show

Requires: pip install h5py matplotlib numpy
"""

import sys
import h5py
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


# ── Data loading ──────────────────────────────────────────────

def load_run(filepath: str, run_name: str = "run_0000") -> dict:
    """Load a simulation run from HDF5 into a dictionary of numpy arrays."""
    with h5py.File(filepath, "r") as f:
        if run_name not in f:
            available = list(f.keys())
            raise KeyError(f"Run '{run_name}' not found. Available: {available}")

        g = f[run_name]
        data = {}
        for key in g.keys():
            data[key] = np.array(g[key])

        data["attrs"] = dict(g.attrs)

    return data


def compute_derived(data: dict) -> dict:
    """Compute derived quantities for plotting."""
    pos = data["position"]
    data["north"] = pos[:, 0]
    data["east"] = pos[:, 1]
    data["down"] = pos[:, 2]
    data["alt"] = -pos[:, 2]

    vel = data["velocity_body"]
    data["u"] = vel[:, 0]
    data["v"] = vel[:, 1]
    data["w"] = vel[:, 2]

    euler = data["euler"]
    data["phi_deg"] = np.degrees(euler[:, 0])
    data["theta_deg"] = np.degrees(euler[:, 1])
    data["psi_deg"] = np.degrees(euler[:, 2])

    omega = data["omega_body"]
    data["p"] = omega[:, 0]
    data["q"] = omega[:, 1]
    data["r"] = omega[:, 2]

    data["alpha_deg"] = np.degrees(data["alpha"])
    data["beta_deg"] = np.degrees(data["beta"])

    return data


# ── Common plots (both guided and unguided) ───────────────────

def plot_trajectory_2d(data, target_data=None):
    """2D trajectory: North vs Altitude."""
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(data["north"], data["alt"], "b-", linewidth=1.5, label="Missile")

    if target_data is not None:
        ax.plot(target_data["north"], target_data["alt"],
                "r--", linewidth=1.5, label="Target")
        # Mark intercept (last point)
        ax.plot(data["north"][-1], data["alt"][-1],
                "k*", markersize=15, label="Intercept")

    ax.plot(data["north"][0], data["alt"][0], "go", markersize=10, label="Launch")
    ax.set_xlabel("North (m)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Engagement Geometry")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    return fig


def plot_trajectory_3d(data, target_data=None):
    """3D trajectory plot."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(data["north"], data["east"], data["alt"],
            "b-", linewidth=1.5, label="Missile")

    if target_data is not None:
        ax.plot(target_data["north"], target_data["east"], target_data["alt"],
                "r--", linewidth=1.5, label="Target")

    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("3D Trajectory")
    ax.legend()
    return fig


def plot_attitude(data):
    """Euler angles vs time."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(data["time"], data["phi_deg"], "b-")
    axes[0].set_ylabel("Roll φ (deg)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(data["time"], data["theta_deg"], "b-")
    axes[1].set_ylabel("Pitch θ (deg)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(data["time"], data["psi_deg"], "b-")
    axes[2].set_ylabel("Yaw ψ (deg)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)

    fig.suptitle("Euler Angles", fontsize=14)
    plt.tight_layout()
    return fig


def plot_body_rates(data):
    """Body angular rates vs time."""
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(data["time"], data["p"], label="p (roll)")
    ax.plot(data["time"], data["q"], label="q (pitch)")
    ax.plot(data["time"], data["r"], label="r (yaw)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Rate (rad/s)")
    ax.set_title("Body Angular Rates")
    ax.legend()
    ax.grid(True, alpha=0.3)
    return fig


def plot_speed_mach(data):
    """Speed and Mach number vs time."""
    fig, ax1 = plt.subplots(figsize=(12, 5))

    color1 = "tab:blue"
    ax1.plot(data["time"], data["speed"], color=color1, linewidth=1.5)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Speed (m/s)", color=color1)
    ax1.tick_params(axis="y", labelcolor=color1)

    ax2 = ax1.twinx()
    color2 = "tab:red"
    ax2.plot(data["time"], data["mach"], color=color2, linestyle="--")
    ax2.set_ylabel("Mach Number", color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)

    ax1.set_title("Speed and Mach Number")
    ax1.grid(True, alpha=0.3)
    return fig


def plot_alpha_beta(data):
    """Angle of attack and sideslip vs time."""
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(data["time"], data["alpha_deg"], label="α (AoA)")
    ax.plot(data["time"], data["beta_deg"], label="β (sideslip)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Angle of Attack and Sideslip")
    ax.legend()
    ax.grid(True, alpha=0.3)
    return fig


def plot_altitude_mass(data):
    """Altitude and mass vs time."""
    fig, ax1 = plt.subplots(figsize=(12, 5))

    color1 = "tab:blue"
    ax1.plot(data["time"], data["alt"], color=color1, linewidth=1.5)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Altitude (m)", color=color1)
    ax1.tick_params(axis="y", labelcolor=color1)

    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.plot(data["time"], data["mass"], color=color2, linestyle="--")
    ax2.set_ylabel("Mass (kg)", color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)

    ax1.set_title("Altitude and Mass")
    ax1.grid(True, alpha=0.3)
    return fig


def plot_dynamic_pressure(data):
    """Dynamic pressure vs time."""
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(data["time"], data["qbar"] / 1000.0, linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Dynamic Pressure (kPa)")
    ax.set_title("Dynamic Pressure")
    ax.grid(True, alpha=0.3)
    return fig


# ── Guided-specific plots ─────────────────────────────────────

def compute_target_trajectory(data):
    """Reconstruct target trajectory from missile data and engagement geometry.

    For now, uses target config from attributes if available.
    Returns a dict with north, east, alt arrays matching time vector.
    """
    t = data["time"]
    attrs = data.get("attrs", {})

    # Try to reconstruct from logged data
    # For a constant velocity target starting at (5000, 0, -500) with vel (-100, 0, 0):
    # This is a placeholder — ideally target state would be logged too
    return None


def compute_range(data, target_pos_0, target_vel):
    """Compute range to target over time."""
    t = data["time"]
    pos = data["position"]

    tgt_north = target_pos_0[0] + target_vel[0] * t
    tgt_east = target_pos_0[1] + target_vel[1] * t
    tgt_down = target_pos_0[2] + target_vel[2] * t

    dn = tgt_north - pos[:, 0]
    de = tgt_east - pos[:, 1]
    dd = tgt_down - pos[:, 2]

    return np.sqrt(dn**2 + de**2 + dd**2), tgt_north, tgt_east, -tgt_down


def plot_range(data, target_pos_0, target_vel):
    """Range to target vs time."""
    rng, _, _, _ = compute_range(data, target_pos_0, target_vel)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(data["time"], rng, "b-", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Range (m)")
    ax.set_title("Range to Target")
    ax.grid(True, alpha=0.3)

    # Mark closest approach
    idx_min = np.argmin(rng)
    ax.plot(data["time"][idx_min], rng[idx_min], "r*", markersize=15,
            label=f"CPA: {rng[idx_min]:.2f} m at t={data['time'][idx_min]:.2f}s")
    ax.legend(fontsize=12)
    return fig


def plot_engagement_2d(data, target_pos_0, target_vel):
    """2D engagement geometry: North vs Altitude with target track."""
    rng, tgt_n, tgt_e, tgt_alt = compute_range(data, target_pos_0, target_vel)

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(data["north"], data["alt"], "b-", linewidth=1.5, label="Missile")
    ax.plot(tgt_n, tgt_alt, "r--", linewidth=1.5, label="Target")

    # Mark launch and intercept
    ax.plot(data["north"][0], data["alt"][0], "go", markersize=10, label="Launch")

    idx_min = np.argmin(rng)
    ax.plot(data["north"][idx_min], data["alt"][idx_min],
            "b*", markersize=12, label=f"Missile at CPA")
    ax.plot(tgt_n[idx_min], tgt_alt[idx_min],
            "r*", markersize=12, label=f"Target at CPA")

    ax.set_xlabel("North (m)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title(f"Engagement Geometry — Miss: {rng[idx_min]:.2f} m")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    return fig


def plot_los_rate(data, target_pos_0, target_vel):
    """Reconstruct and plot LOS rate magnitude vs time.

    For an ideal seeker, LOS rate = (R × V_rel) / r².
    On a collision course, LOS rate → 0.
    """
    t = data["time"]
    pos = data["position"]
    vel_body = data["velocity_body"]
    euler = data["euler"]

    tgt_pos = np.column_stack([
        target_pos_0[0] + target_vel[0] * t,
        target_pos_0[1] + target_vel[1] * t,
        target_pos_0[2] + target_vel[2] * t
    ])

    # Relative position
    R = tgt_pos - pos
    rng = np.linalg.norm(R, axis=1)

    # Missile NED velocity (rotate body velocity through Euler DCM)
    msl_vel_ned = np.zeros_like(vel_body)
    for i in range(len(t)):
        phi, theta, psi = euler[i]
        cp, sp = np.cos(phi), np.sin(phi)
        ct, st = np.cos(theta), np.sin(theta)
        cy, sy = np.cos(psi), np.sin(psi)

        # Body to inertial DCM (3-2-1)
        L = np.array([
            [ct*cy, sp*st*cy - cp*sy, cp*st*cy + sp*sy],
            [ct*sy, sp*st*sy + cp*cy, cp*st*sy - sp*cy],
            [-st,   sp*ct,            cp*ct]
        ])
        msl_vel_ned[i] = L @ vel_body[i]

    # Relative velocity (target - missile)
    tgt_vel_array = np.tile(target_vel, (len(t), 1))
    V_rel = tgt_vel_array - msl_vel_ned

    # LOS rate: ω = (R × V_rel) / r²
    omega = np.cross(R, V_rel) / (rng[:, None]**2 + 1e-10)
    omega_mag = np.linalg.norm(omega, axis=1)

    # Range rate
    R_unit = R / (rng[:, None] + 1e-10)
    rdot = np.sum(V_rel * R_unit, axis=1)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # LOS rate magnitude
    axes[0].plot(t, np.degrees(omega_mag) * 1000, "b-", linewidth=1.0)
    axes[0].set_ylabel("LOS Rate (mdeg/s)")
    axes[0].set_title("Line-of-Sight Rate Magnitude")
    axes[0].grid(True, alpha=0.3)

    # LOS rate components
    axes[1].plot(t, omega[:, 0], label="ω_x (roll)")
    axes[1].plot(t, omega[:, 1], label="ω_y (pitch)")
    axes[1].plot(t, omega[:, 2], label="ω_z (yaw)")
    axes[1].set_ylabel("LOS Rate (rad/s)")
    axes[1].set_title("LOS Rate Components (NED)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # Range rate
    axes[2].plot(t, rdot, "r-", linewidth=1.0)
    axes[2].set_ylabel("Range Rate (m/s)")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_title("Range Rate (negative = closing)")
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_acceleration_cmd(data, target_pos_0, target_vel):
    """Reconstruct and plot guidance acceleration commands.

    Since we don't log guidance commands directly (yet), we reconstruct
    from the PN law: a_cmd = N * Vc * (ω_LOS × û_LOS).
    """
    t = data["time"]
    pos = data["position"]
    vel_body = data["velocity_body"]
    euler = data["euler"]

    tgt_pos = np.column_stack([
        target_pos_0[0] + target_vel[0] * t,
        target_pos_0[1] + target_vel[1] * t,
        target_pos_0[2] + target_vel[2] * t
    ])

    R = tgt_pos - pos
    rng = np.linalg.norm(R, axis=1)
    R_unit = R / (rng[:, None] + 1e-10)

    # Missile NED velocity
    msl_vel_ned = np.zeros_like(vel_body)
    for i in range(len(t)):
        phi, theta, psi = euler[i]
        cp, sp = np.cos(phi), np.sin(phi)
        ct, st = np.cos(theta), np.sin(theta)
        cy, sy = np.cos(psi), np.sin(psi)
        L = np.array([
            [ct*cy, sp*st*cy - cp*sy, cp*st*cy + sp*sy],
            [ct*sy, sp*st*sy + cp*cy, cp*st*sy - sp*cy],
            [-st,   sp*ct,            cp*ct]
        ])
        msl_vel_ned[i] = L @ vel_body[i]

    tgt_vel_array = np.tile(target_vel, (len(t), 1))
    V_rel = tgt_vel_array - msl_vel_ned

    omega = np.cross(R, V_rel) / (rng[:, None]**2 + 1e-10)
    rdot = np.sum(V_rel * R_unit, axis=1)
    Vc = -rdot

    # PN acceleration: a = N * Vc * (ω × û)
    N = 4.0
    omega_cross_u = np.cross(omega, R_unit)
    a_cmd = N * Vc[:, None] * omega_cross_u

    # Magnitude
    a_cmd_mag = np.linalg.norm(a_cmd, axis=1)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(t, a_cmd[:, 0], label="a_N (North)")
    axes[0].plot(t, a_cmd[:, 1], label="a_E (East)")
    axes[0].plot(t, a_cmd[:, 2], label="a_D (Down)")
    axes[0].set_ylabel("Acceleration (m/s²)")
    axes[0].set_title("Guidance Acceleration Commands (NED)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, a_cmd_mag / 9.80665, "b-", linewidth=1.5)
    axes[1].set_ylabel("Acceleration (g)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title("Commanded Acceleration Magnitude")
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_summary_dashboard(data, target_pos_0=None, target_vel=None):
    """Compact 2×3 summary dashboard."""
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    t = data["time"]

    # [0,0] Trajectory
    ax = axes[0, 0]
    ax.plot(data["north"], data["alt"], "b-", linewidth=1.5, label="Missile")
    if target_pos_0 is not None:
        rng, tgt_n, _, tgt_alt = compute_range(data, target_pos_0, target_vel)
        ax.plot(tgt_n, tgt_alt, "r--", linewidth=1.5, label="Target")
        idx = np.argmin(rng)
        ax.plot(data["north"][idx], data["alt"][idx], "b*", ms=12)
        ax.plot(tgt_n[idx], tgt_alt[idx], "r*", ms=12)
        ax.set_title(f"Engagement — Miss: {rng[idx]:.2f} m")
    else:
        ax.set_title("Trajectory")
    ax.set_xlabel("North (m)")
    ax.set_ylabel("Altitude (m)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # [0,1] Speed & Mach
    ax = axes[0, 1]
    ax.plot(t, data["speed"], "b-", label="Speed")
    ax.set_ylabel("Speed (m/s)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Speed")
    ax2 = ax.twinx()
    ax2.plot(t, data["mach"], "r--", alpha=0.7, label="Mach")
    ax2.set_ylabel("Mach")
    ax.grid(True, alpha=0.3)

    # [0,2] Pitch angle
    ax = axes[0, 2]
    ax.plot(t, data["theta_deg"], "b-")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pitch (deg)")
    ax.set_title("Pitch Angle")
    ax.grid(True, alpha=0.3)

    # [1,0] Alpha/Beta
    ax = axes[1, 0]
    ax.plot(t, data["alpha_deg"], label="α")
    ax.plot(t, data["beta_deg"], label="β")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("AoA and Sideslip")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # [1,1] Body rates
    ax = axes[1, 1]
    ax.plot(t, data["q"], label="q (pitch)")
    ax.plot(t, data["r"], label="r (yaw)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rate (rad/s)")
    ax.set_title("Body Rates")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # [1,2] Range (guided) or Altitude/Mass (unguided)
    ax = axes[1, 2]
    if target_pos_0 is not None:
        rng, _, _, _ = compute_range(data, target_pos_0, target_vel)
        ax.plot(t, rng, "b-", linewidth=1.5)
        idx = np.argmin(rng)
        ax.plot(t[idx], rng[idx], "r*", ms=15)
        ax.set_ylabel("Range (m)")
        ax.set_title("Range to Target")
    else:
        ax.plot(t, data["alt"], "b-")
        ax.set_ylabel("Altitude (m)")
        ax.set_title("Altitude")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)

    fig.suptitle("Simulation Summary", fontsize=16, fontweight="bold")
    plt.tight_layout()
    return fig


# ── Main entry point ──────────────────────────────────────────

def main():
    save_mode = False
    args = [a for a in sys.argv[1:] if a != "--save"]
    if "--save" in sys.argv:
        save_mode = True

    filepath = args[0] if len(args) > 0 else "sim_output.h5"
    run_name = args[1] if len(args) > 1 else "run_0000"

    if not Path(filepath).exists():
        print(f"Error: {filepath} not found")
        sys.exit(1)

    print(f"Loading {filepath}/{run_name}...")
    data = load_run(filepath, run_name)
    data = compute_derived(data)

    print(f"  Records: {len(data['time'])}")
    print(f"  Duration: {data['time'][-1]:.3f} s")
    if data["attrs"]:
        print(f"  Attributes:")
        for k, v in data["attrs"].items():
            print(f"    {k}: {v}")

    # Detect guided mode
    guided = data["attrs"].get("guided", 0) == 1

    # Target definition (hardcoded for now — later read from HDF5 attributes)
    # TODO: log target config as HDF5 attributes for automatic reconstruction
    target_pos_0 = None
    target_vel = None
    if guided:
        attrs = data["attrs"]
        target_pos_0 = np.array([
            attrs.get("target_pos_north", 0),
            attrs.get("target_pos_east", 0),
            attrs.get("target_pos_down", 0)
        ])
        target_vel = np.array([
            attrs.get("target_vel_north", 0),
            attrs.get("target_vel_east", 0),
            attrs.get("target_vel_down", 0)
        ])
        print(f"  Guided mode: target at [{target_pos_0}], vel=[{target_vel}]")

        rng, _, _, _ = compute_range(data, target_pos_0, target_vel)
        print(f"  Miss distance: {np.min(rng):.3f} m at t={data['time'][np.argmin(rng)]:.3f} s")

    # Generate plots
    figs = []

    # Summary dashboard (always first)
    figs.append(("summary", plot_summary_dashboard(data, target_pos_0, target_vel)))

    # Common plots
    figs.append(("attitude", plot_attitude(data)))
    figs.append(("body_rates", plot_body_rates(data)))
    figs.append(("speed_mach", plot_speed_mach(data)))
    figs.append(("alpha_beta", plot_alpha_beta(data)))
    figs.append(("alt_mass", plot_altitude_mass(data)))
    figs.append(("qbar", plot_dynamic_pressure(data)))

    # Guided-specific plots
    if guided and target_pos_0 is not None:
        figs.append(("engagement", plot_engagement_2d(data, target_pos_0, target_vel)))
        figs.append(("range", plot_range(data, target_pos_0, target_vel)))
        figs.append(("los_rate", plot_los_rate(data, target_pos_0, target_vel)))
        figs.append(("accel_cmd", plot_acceleration_cmd(data, target_pos_0, target_vel)))
        figs.append(("trajectory_3d", plot_trajectory_3d(
            data, {"north": target_pos_0[0] + target_vel[0] * data["time"],
                   "east": target_pos_0[1] + target_vel[1] * data["time"],
                   "alt": -(target_pos_0[2] + target_vel[2] * data["time"])})))

    if save_mode:
        out_dir = Path(filepath).stem + "_plots"
        Path(out_dir).mkdir(exist_ok=True)
        for name, fig in figs:
            outpath = f"{out_dir}/{name}.png"
            fig.savefig(outpath, dpi=150, bbox_inches="tight")
            print(f"  Saved: {outpath}")
        print(f"\nAll plots saved to {out_dir}/")
    else:
        plt.show()


if __name__ == "__main__":
    main()