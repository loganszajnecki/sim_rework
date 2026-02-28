#!/usr/bin/env python3
"""
Plot simulation results from HDF5 output.

Usage:
    python plot_results.py                          # default: sim_output.h5, run_0000
    python plot_results.py output.h5                # specific file
    python plot_results.py output.h5 run_0003       # specific run

Requires: pip install h5py matplotlib numpy
"""

import sys
import h5py
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


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

        # Load attributes
        data["attrs"] = dict(g.attrs)

    return data


def plot_trajectory_3d(data: dict):
    """3D trajectory plot."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    pos = data["position"]  # N x 3: North, East, Down
    north = pos[:, 0]
    east = pos[:, 1]
    alt = -pos[:, 2]  # Convert Down to altitude

    ax.plot(north, east, alt)
    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("3D Trajectory")
    return fig


def plot_position(data: dict):
    """Position vs time in NED."""
    fig, ax = plt.subplots(figsize=(10, 6))
    t = data["time"]
    pos = data["position"]

    ax.plot(t, pos[:, 0], label="North")
    ax.plot(t, pos[:, 1], label="East")
    ax.plot(t, pos[:, 2], label="Down")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Position (NED)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    return fig


def plot_attitude(data: dict):
    """Euler angles vs time."""
    fig, ax = plt.subplots(figsize=(10, 6))
    t = data["time"]
    euler = np.degrees(data["euler"])  # Convert to degrees

    ax.plot(t, euler[:, 0], label="φ (roll)")
    ax.plot(t, euler[:, 1], label="θ (pitch)")
    ax.plot(t, euler[:, 2], label="ψ (yaw)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Euler Angles")
    ax.legend()
    ax.grid(True, alpha=0.3)
    return fig


def plot_body_rates(data: dict):
    """Body angular rates vs time."""
    fig, ax = plt.subplots(figsize=(10, 6))
    t = data["time"]
    omega = data["omega_body"]

    ax.plot(t, omega[:, 0], label="p (roll rate)")
    ax.plot(t, omega[:, 1], label="q (pitch rate)")
    ax.plot(t, omega[:, 2], label="r (yaw rate)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Rate (rad/s)")
    ax.set_title("Body Angular Rates")
    ax.legend()
    ax.grid(True, alpha=0.3)
    return fig


def plot_speed_mach(data: dict):
    """Speed and Mach number vs time."""
    fig, ax1 = plt.subplots(figsize=(10, 6))
    t = data["time"]

    color1 = "tab:blue"
    ax1.plot(t, data["speed"], color=color1, label="Speed")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Speed (m/s)", color=color1)
    ax1.tick_params(axis="y", labelcolor=color1)

    ax2 = ax1.twinx()
    color2 = "tab:red"
    ax2.plot(t, data["mach"], color=color2, label="Mach", linestyle="--")
    ax2.set_ylabel("Mach Number", color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)

    ax1.set_title("Speed and Mach Number")
    ax1.grid(True, alpha=0.3)
    return fig


def plot_alpha_beta(data: dict):
    """Angle of attack and sideslip vs time."""
    fig, ax = plt.subplots(figsize=(10, 6))
    t = data["time"]

    ax.plot(t, np.degrees(data["alpha"]), label="α (AoA)")
    ax.plot(t, np.degrees(data["beta"]), label="β (sideslip)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Angle of Attack & Sideslip")
    ax.legend()
    ax.grid(True, alpha=0.3)
    return fig


def plot_altitude_mass(data: dict):
    """Altitude and mass vs time."""
    fig, ax1 = plt.subplots(figsize=(10, 6))
    t = data["time"]

    color1 = "tab:blue"
    ax1.plot(t, data["altitude"], color=color1)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Altitude (m)", color=color1)
    ax1.tick_params(axis="y", labelcolor=color1)

    ax2 = ax1.twinx()
    color2 = "tab:green"
    ax2.plot(t, data["mass"], color=color2, linestyle="--")
    ax2.set_ylabel("Mass (kg)", color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)

    ax1.set_title("Altitude and Mass")
    ax1.grid(True, alpha=0.3)
    return fig


def plot_dynamic_pressure(data: dict):
    """Dynamic pressure vs time."""
    fig, ax = plt.subplots(figsize=(10, 6))
    t = data["time"]

    ax.plot(t, data["qbar"] / 1000.0)  # Convert to kPa
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Dynamic Pressure (kPa)")
    ax.set_title("Dynamic Pressure")
    ax.grid(True, alpha=0.3)
    return fig


def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else "sim_output.h5"
    run_name = sys.argv[2] if len(sys.argv) > 2 else "run_0000"

    if not Path(filepath).exists():
        print(f"Error: {filepath} not found")
        sys.exit(1)

    print(f"Loading {filepath}/{run_name}...")
    data = load_run(filepath, run_name)

    print(f"  Records: {len(data['time'])}")
    print(f"  Duration: {data['time'][-1]:.3f} s")
    if data["attrs"]:
        print(f"  Attributes:")
        for k, v in data["attrs"].items():
            print(f"    {k}: {v}")

    # Generate all plots
    plot_trajectory_3d(data)
    plot_position(data)
    plot_attitude(data)
    plot_body_rates(data)
    plot_speed_mach(data)
    plot_alpha_beta(data)
    plot_altitude_mass(data)
    plot_dynamic_pressure(data)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()