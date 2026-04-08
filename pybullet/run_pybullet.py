"""
PyBullet entry point — run from the repository root:

    python pybullet/run_pybullet.py [--no-gui]
"""

import argparse
import sys
import os

# Ensure the repo root is on the path so 'robot.*' imports work
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pybullet.pybullet_sim import run_simulation, print_force_summary


def main():
    parser = argparse.ArgumentParser(description="Robotic Horse — PyBullet simulation")
    parser.add_argument("--no-gui", action="store_true", help="Headless mode (no window)")
    args = parser.parse_args()

    print("Launching PyBullet simulation...")
    results = run_simulation(gui=not args.no_gui)
    print_force_summary(results)


if __name__ == "__main__":
    main()
