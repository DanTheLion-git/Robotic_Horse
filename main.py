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
from robot.kinematics.force_calculator import (
    analyse_gait_forces, M_BODY, NUT_STROKE,
    BALLSCREW_LEAD, BALLSCREW_EFF, CARRIAGE_DRAG_N,
)
from robot.kinematics.leg_kinematics import R_ARM, L1, L2


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

        ax.plot(phase, result["nut_force"],    label="Nut force [N]",      color="tab:blue")
        ax.plot(phase, result["tau_knee"],     label="Knee torque [N·m]",  color="tab:orange", linestyle="--")
        ax.plot(phase, result["lever_arm"] * 1000, label="Lever arm [mm]", color="tab:red",    linestyle="-.")
        ax2 = ax.twinx()
        ax2.plot(phase, result["motor_torque"], label="Motor τ [N·m]",     color="tab:green",  linestyle=":")
        ax2.set_ylabel("Motor torque [N·m]", color="tab:green")

        ax.set_title(leg.upper())
        ax.set_xlabel("Gait phase")
        ax.set_ylabel("Force [N] / Torque [N·m] / Lever [mm]")
        ax.legend(loc="upper left", fontsize=7)
        ax2.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("force_analysis.png", dpi=150)
    print("Force analysis plot saved → force_analysis.png")
    plt.show()


def print_analysis_summary(gait: dict) -> None:
    print("\n" + "=" * 68)
    print("  ROBOTIC HORSE — BALLSCREW FORCE ANALYSIS")
    print(f"  Body mass : {M_BODY} kg      Gravity: 9.81 m/s²")
    print(f"  Leg dims  : L1={L1} m (thigh)  L2={L2} m (shank)")
    print(f"  Lever arm : R_ARM={R_ARM*1000:.0f} mm  (pivot → nut connection)")
    print(f"  Screw lead: {BALLSCREW_LEAD*1000:.1f} mm/rev   Efficiency: {BALLSCREW_EFF*100:.0f}%")
    print(f"  Nut stroke: {NUT_STROKE*1000:.1f} mm  (over working knee-angle range)")
    print(f"  Carriage drag: {CARRIAGE_DRAG_N:.1f} N (negligible)")
    print("=" * 68)
    header = (
        f"  {'Leg':<5} {'Peak F_nut [N]':>14} {'Mean F_nut [N]':>14} "
        f"{'Peak τ_motor [N·m]':>19} {'Nut stroke [mm]':>16}"
    )
    print(header)
    print("  " + "-" * 70)
    for leg in LEG_NAMES:
        r = analyse_gait_forces(
            gait[leg]["theta_hip"],
            gait[leg]["theta_knee"],
            body_mass=M_BODY,
        )
        nut_travel_mm = (r["nut_position_m"].max() - r["nut_position_m"].min()) * 1000
        print(
            f"  {leg.upper():<5} "
            f"{r['nut_force'].max():>14.1f} "
            f"{r['nut_force'].mean():>14.1f} "
            f"{r['motor_torque'].max():>19.3f} "
            f"{nut_travel_mm:>16.1f}"
        )
    print("=" * 68)


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
