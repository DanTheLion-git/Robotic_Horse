"""
Robotic Horse — main entry point.

Usage
-----
    python main.py              # run with GUI + force analysis plots
    python main.py --no-gui     # headless simulation (CI / server)
    python main.py --analysis   # static force analysis only (no simulation)
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt

from robot.gait.walk_gait import generate_gait_trajectory, LEG_NAMES
from robot.kinematics.force_calculator import analyse_gait_forces, M_BODY


def plot_force_analysis(gait: dict) -> None:
    """Plot kinematic force analysis for one gait cycle."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Robotic Horse — Ballscrew Force Analysis (one gait cycle)", fontsize=13)

    for ax, leg in zip(axes.flat, LEG_NAMES):
        result = analyse_gait_forces(
            gait[leg]["theta_hip"],
            gait[leg]["theta_knee"],
            body_mass=M_BODY,
        )
        phase = gait[leg]["phase"]

        ax.plot(phase, result["ballscrew_force"],  label="Ballscrew F [N]",   color="tab:blue")
        ax.plot(phase, result["tau_knee"],         label="Knee torque [N·m]", color="tab:orange", linestyle="--")
        ax2 = ax.twinx()
        ax2.plot(phase, result["motor_torque"],    label="Motor τ [N·m]",     color="tab:green",  linestyle=":")
        ax2.set_ylabel("Motor torque [N·m]", color="tab:green")

        ax.set_title(leg.upper())
        ax.set_xlabel("Gait phase")
        ax.set_ylabel("Force / Torque")
        ax.legend(loc="upper left", fontsize=7)
        ax2.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("force_analysis.png", dpi=150)
    print("Force analysis plot saved → force_analysis.png")
    plt.show()


def print_analysis_summary(gait: dict) -> None:
    print("\n" + "=" * 60)
    print("  KINEMATIC FORCE ANALYSIS — one gait cycle")
    print(f"  Body mass: {M_BODY} kg    Gravity: 9.81 m/s²")
    print("=" * 60)
    header = f"  {'Leg':<6} {'Max F_bs [N]':>14} {'Mean F_bs [N]':>14} {'Max τ_motor [N·m]':>18}"
    print(header)
    print("  " + "-" * 54)
    for leg in LEG_NAMES:
        r = analyse_gait_forces(
            gait[leg]["theta_hip"],
            gait[leg]["theta_knee"],
            body_mass=M_BODY,
        )
        print(
            f"  {leg.upper():<6} "
            f"{r['ballscrew_force'].max():>14.1f} "
            f"{r['ballscrew_force'].mean():>14.1f} "
            f"{r['motor_torque'].max():>18.3f}"
        )
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Robotic Horse simulator")
    parser.add_argument("--no-gui",    action="store_true", help="Run headless (no PyBullet GUI)")
    parser.add_argument("--analysis",  action="store_true", help="Static force analysis only")
    parser.add_argument("--no-plots",  action="store_true", help="Skip matplotlib plots")
    args = parser.parse_args()

    print("Generating gait trajectory...")
    gait = generate_gait_trajectory(n_steps=200)

    print_analysis_summary(gait)

    if not args.no_plots:
        plot_force_analysis(gait)

    if not args.analysis:
        from robot.simulation.pybullet_sim import run_simulation, print_force_summary
        print("\nLaunching PyBullet simulation...")
        results = run_simulation(gui=not args.no_gui)
        print_force_summary(results)


if __name__ == "__main__":
    main()
