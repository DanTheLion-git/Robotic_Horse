"""
Robotic Horse — main entry point.

Usage
-----
    python main.py              # run with GUI + force analysis plots
    python main.py --no-gui     # headless simulation (CI / server)
    python main.py --analysis   # static force analysis only (no simulation)
    python main.py --trot       # use trot gait instead of walk
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt

from robot.gait.walk_gait import generate_gait_trajectory, LEG_NAMES, WALK, TROT, FRONT_LEGS
from robot.kinematics.force_calculator import (
    analyse_gait_forces, M_BODY, QDD_SPECS, weight_per_leg,
)
from robot.kinematics.leg_kinematics import (
    L1_FRONT, L2_FRONT, L3_FRONT, L1_REAR, L2_REAR, L3_REAR,
    BODY_HEIGHT, ANKLE_HEIGHT, ANKLE_HEIGHT_FRONT, ANKLE_HEIGHT_REAR,
    FRONT_HIP_HEIGHT, REAR_HIP_HEIGHT,
)


def plot_force_analysis(gait_data: dict) -> None:
    """Plot QDD force analysis for one gait cycle."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Highland Cow — QDD Joint Torque Analysis (one gait cycle)", fontsize=13)

    for ax, leg in zip(axes.flat, LEG_NAMES):
        is_front = leg in FRONT_LEGS
        result = analyse_gait_forces(
            gait_data[leg]["theta_hip"],
            gait_data[leg]["theta_knee"],
            is_front=is_front,
        )
        phase = gait_data[leg]["phase"]

        ax.plot(phase, result["tau_hip"],  label="Hip torque [N·m]",  color="tab:blue")
        ax.plot(phase, result["tau_knee"], label="Knee torque [N·m]", color="tab:orange", linestyle="--")
        ax.plot(phase, result["tau_cannon"], label="Cannon torque [N·m]", color="tab:red", linestyle="-.")

        ax2 = ax.twinx()
        ax2.plot(phase, result["power_hip"],  label="Hip power [W]",  color="tab:green", linestyle=":", alpha=0.7)
        ax2.plot(phase, result["power_knee"], label="Knee power [W]", color="tab:purple", linestyle=":", alpha=0.7)
        ax2.set_ylabel("Power [W]")

        ax.set_title(f"{leg.upper()} ({'front' if is_front else 'rear'}, "
                     f"{'55%' if is_front else '45%'} weight)")
        ax.set_xlabel("Gait phase")
        ax.set_ylabel("Joint torque [N·m]")
        ax.legend(loc="upper left", fontsize=7)
        ax2.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("force_analysis.png", dpi=150)
    print("Force analysis plot saved → force_analysis.png")
    plt.show()


def print_analysis_summary(gait_data: dict) -> None:
    print("\n" + "=" * 72)
    print("  HIGHLAND COW — QDD FORCE ANALYSIS (Bovine Leg Mechanics)")
    print(f"  Body mass : {M_BODY} kg      Gravity: 9.81 m/s²")
    print(f"  Front leg : L1={L1_FRONT} m (humerus)  L2={L2_FRONT} m (radius)  L3={L3_FRONT} m (cannon)")
    print(f"  Rear leg  : L1={L1_REAR} m (femur)    L2={L2_REAR} m (tibia)   L3={L3_REAR} m (cannon)")
    print(f"  Front hip : {FRONT_HIP_HEIGHT:.3f} m   Ankle: {ANKLE_HEIGHT_FRONT:.3f} m")
    print(f"  Rear hip  : {REAR_HIP_HEIGHT:.3f} m   Ankle: {ANKLE_HEIGHT_REAR:.3f} m")
    print(f"  Weight distribution: 55% front / 45% rear (bovine)")
    print(f"  Front leg GRF: {weight_per_leg(True):.1f} N   Rear leg GRF: {weight_per_leg(False):.1f} N")
    print("=" * 72)

    header = (
        f"  {'Leg':<5} {'Peak τ_hip [N·m]':>16} {'Peak τ_knee [N·m]':>18} "
        f"{'Peak τ_cannon':>14} {'Peak P_hip [W]':>15} {'Peak P_knee [W]':>16}"
    )
    print(header)
    print("  " + "-" * 82)

    for leg in LEG_NAMES:
        is_front = leg in FRONT_LEGS
        r = analyse_gait_forces(
            gait_data[leg]["theta_hip"],
            gait_data[leg]["theta_knee"],
            is_front=is_front,
        )
        print(
            f"  {leg.upper():<5} "
            f"{r['tau_hip'].max():>16.1f} "
            f"{r['tau_knee'].max():>18.1f} "
            f"{r['tau_cannon'].max():>14.1f} "
            f"{r['power_hip'].max():>15.1f} "
            f"{r['power_knee'].max():>16.1f}"
        )

    print("=" * 72)
    print("\n  QDD Motor Budget:")
    total_cost = 0
    total_mass = 0
    for joint, spec in QDD_SPECS.items():
        count = 4
        print(f"    {joint:<12} {spec['name']:<14} × {count}  "
              f"peak={spec['peak_torque']:>5.0f} N·m  "
              f"cont={spec['cont_torque']:>5.0f} N·m  "
              f"mass={spec['mass']*count:.1f} kg  "
              f"cost=€{spec['price_eur']*count}")
        total_cost += spec['price_eur'] * count
        total_mass += spec['mass'] * count
    print(f"    {'':─<65}")
    print(f"    Total: {total_mass:.1f} kg motor mass, €{total_cost} estimated cost")
    print("=" * 72)


def main():
    parser = argparse.ArgumentParser(description="Highland Cow quadruped simulator")
    parser.add_argument("--no-gui",    action="store_true", help="Run headless (no PyBullet GUI)")
    parser.add_argument("--analysis",  action="store_true", help="Static force analysis only")
    parser.add_argument("--no-plots",  action="store_true", help="Skip matplotlib plots")
    parser.add_argument("--trot",      action="store_true", help="Use trot gait instead of walk")
    args = parser.parse_args()

    gait = TROT if args.trot else WALK
    print(f"Generating {gait['name']} gait trajectory...")
    gait_data = generate_gait_trajectory(n_steps=200, gait=gait)

    print_analysis_summary(gait_data)

    if not args.no_plots:
        plot_force_analysis(gait_data)

    if not args.analysis:
        from pybullet.pybullet_sim import run_simulation, print_force_summary
        print("\nLaunching PyBullet simulation...")
        results = run_simulation(gui=not args.no_gui, gait=gait)
        print_force_summary(results)


if __name__ == "__main__":
    main()
