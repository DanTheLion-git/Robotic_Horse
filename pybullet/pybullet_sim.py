"""
PyBullet simulation for the Highland Cow quadruped.

Run this from the repository root:
    python pybullet/run_pybullet.py

Loads the URDF, runs a bovine gait, applies joint position targets,
and logs joint torques every step.

Leg mechanics:
  Front legs: passive cannon linkage — cannon = CANNON_LEAN - (thigh + knee)
  Rear legs:  reciprocal apparatus  — cannon = RECIP_OFFSET - RECIP_RATIO * knee
"""

import os
import sys
import time
import numpy as np

# The local pybullet/ directory is loaded as a package before this module runs,
# leaving an empty stub in sys.modules. Evict it and reload from site-packages.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.modules.pop('pybullet', None)
sys.modules.pop('pybullet_data', None)
_saved_path = sys.path[:]
sys.path = [p for p in sys.path if p != _repo_root]
import pybullet as pb          # noqa: E402  — real C extension
import pybullet_data           # noqa: E402
sys.path = _saved_path
del _repo_root, _saved_path

from robot.gait.walk_gait import generate_gait_trajectory, LEG_NAMES, WALK, FRONT_LEGS
from robot.kinematics.leg_kinematics import (
    cannon_angle_front, cannon_angle_rear,
)
from robot.kinematics.force_calculator import M_BODY

URDF_PATH = os.path.join(
    os.path.dirname(__file__), "..", "robot", "urdf", "robotic_horse.urdf"
)

# Joint names in the URDF that we actively control (Highland Cow 3-segment legs)
JOINT_MAP = {
    "fl": {"hip": "fl_hip_joint", "thigh": "fl_thigh_joint", "knee": "fl_knee_joint", "cannon": "fl_cannon_joint"},
    "fr": {"hip": "fr_hip_joint", "thigh": "fr_thigh_joint", "knee": "fr_knee_joint", "cannon": "fr_cannon_joint"},
    "rl": {"hip": "rl_hip_joint", "thigh": "rl_thigh_joint", "knee": "rl_knee_joint", "cannon": "rl_cannon_joint"},
    "rr": {"hip": "rr_hip_joint", "thigh": "rr_thigh_joint", "knee": "rr_knee_joint", "cannon": "rr_cannon_joint"},
}

SIM_DT      = 1 / 240.0    # PyBullet default timestep
N_CYCLES    = 3             # how many gait cycles to simulate


def build_joint_index(robot_id: int) -> dict[str, int]:
    """Return a mapping {joint_name: joint_index} for all joints."""
    index = {}
    n = pb.getNumJoints(robot_id)
    for i in range(n):
        info = pb.getJointInfo(robot_id, i)
        index[info[1].decode()] = i
    return index


def run_simulation(gui: bool = True, record_forces: bool = True,
                   gait: dict = None) -> dict:
    """
    Run a bovine gait simulation.

    Parameters
    ----------
    gui            : If True, open the PyBullet GUI window.
    record_forces  : If True, record joint reaction forces each step.
    gait           : Gait parameters dict (defaults to WALK).

    Returns
    -------
    dict with force / torque logs per leg.
    """
    if gait is None:
        gait = WALK

    mode = pb.GUI if gui else pb.DIRECT
    physics_client = pb.connect(mode)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    pb.setTimeStep(SIM_DT)

    plane_id = pb.loadURDF("plane.urdf")

    # Spawn above ground so feet clear the floor
    start_pos = [0, 0, 1.20]
    start_orn = pb.getQuaternionFromEuler([0, 0, 0])
    robot_id  = pb.loadURDF(
        os.path.normpath(URDF_PATH),
        start_pos,
        start_orn,
        useFixedBase=False,
    )

    joint_index = build_joint_index(robot_id)

    # Pre-generate trajectory
    gait_data = generate_gait_trajectory(n_steps=200, gait=gait)
    gait_period = gait['period']
    steps_per_cycle = int(gait_period / SIM_DT)
    total_steps     = steps_per_cycle * N_CYCLES

    logs: dict[str, list] = {leg: [] for leg in LEG_NAMES}

    for step in range(total_steps):
        gait_phase = (step % steps_per_cycle) / steps_per_cycle
        traj_idx   = int(gait_phase * 200) % 200

        for leg in LEG_NAMES:
            th_hip  = gait_data[leg]["theta_hip"][traj_idx]
            th_knee = gait_data[leg]["theta_knee"][traj_idx]
            th_cannon = gait_data[leg]["theta_cannon"][traj_idx]

            hip_jnt    = JOINT_MAP[leg]["hip"]
            thigh_jnt  = JOINT_MAP[leg]["thigh"]
            knee_jnt   = JOINT_MAP[leg]["knee"]
            cannon_jnt = JOINT_MAP[leg]["cannon"]

            # Hip yaw stays at 0 (no turning in basic gait)
            if hip_jnt in joint_index:
                pb.setJointMotorControl2(
                    robot_id, joint_index[hip_jnt],
                    pb.POSITION_CONTROL, targetPosition=0.0,
                    force=200, positionGain=0.5, velocityGain=0.1,
                )
            if thigh_jnt in joint_index:
                pb.setJointMotorControl2(
                    robot_id, joint_index[thigh_jnt],
                    pb.POSITION_CONTROL, targetPosition=th_hip,
                    force=500, positionGain=0.5, velocityGain=0.1,
                )
            if knee_jnt in joint_index:
                pb.setJointMotorControl2(
                    robot_id, joint_index[knee_jnt],
                    pb.POSITION_CONTROL, targetPosition=th_knee,
                    force=800, positionGain=0.5, velocityGain=0.1,
                )
            if cannon_jnt in joint_index:
                pb.setJointMotorControl2(
                    robot_id, joint_index[cannon_jnt],
                    pb.POSITION_CONTROL, targetPosition=th_cannon,
                    force=100, positionGain=0.5, velocityGain=0.1,
                )

        pb.stepSimulation()

        if record_forces:
            for leg in LEG_NAMES:
                knee_jnt = JOINT_MAP[leg]["knee"]
                hip_jnt = JOINT_MAP[leg]["thigh"]
                if knee_jnt in joint_index and hip_jnt in joint_index:
                    k_state = pb.getJointState(robot_id, joint_index[knee_jnt])
                    h_state = pb.getJointState(robot_id, joint_index[hip_jnt])
                    logs[leg].append({
                        "step":        step,
                        "phase":       gait_phase,
                        "knee_angle":  k_state[0],
                        "knee_torque": k_state[3],
                        "hip_angle":   h_state[0],
                        "hip_torque":  h_state[3],
                    })

        if gui:
            time.sleep(SIM_DT)

    pb.disconnect()

    # Convert to numpy arrays
    results = {}
    for leg in LEG_NAMES:
        entries = logs[leg]
        if entries:
            results[leg] = {
                "phase":       np.array([e["phase"]       for e in entries]),
                "knee_angle":  np.array([e["knee_angle"]  for e in entries]),
                "knee_torque": np.array([e["knee_torque"] for e in entries]),
                "hip_angle":   np.array([e["hip_angle"]   for e in entries]),
                "hip_torque":  np.array([e["hip_torque"]  for e in entries]),
            }

    return results


def print_force_summary(sim_results: dict) -> None:
    """Print a summary of peak and mean joint torques from simulation."""
    print("\n" + "=" * 65)
    print("  SIMULATION FORCE SUMMARY — joint torques [N·m]")
    print("=" * 65)
    for leg, data in sim_results.items():
        tk = np.abs(data["knee_torque"])
        th = np.abs(data["hip_torque"])
        print(f"  {leg.upper()}  hip: peak={th.max():.1f} mean={th.mean():.1f}  "
              f"knee: peak={tk.max():.1f} mean={tk.mean():.1f}")
    print("=" * 65)
