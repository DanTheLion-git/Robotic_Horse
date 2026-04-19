"""
PyBullet simulation for the Robotic Horse quadruped.

Run this from the repository root:
    python pybullet/run_pybullet.py

Loads the URDF, runs a trot gait, applies joint position targets,
and logs joint torques every step.
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

from robot.gait.walk_gait import generate_gait_trajectory, LEG_NAMES
from robot.kinematics.force_calculator import (
    analyse_gait_forces,
    M_BODY,
    CARRIAGE_DRAG_N,
)

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

FRONT_LEGS = ("fl", "fr")
REAR_LEGS  = ("rl", "rr")

SIM_DT      = 1 / 240.0    # PyBullet default timestep
GAIT_PERIOD = 1.0           # seconds per gait cycle
N_CYCLES    = 3             # how many gait cycles to simulate

# Highland Cow cannon bone pantograph
CANNON_LEAN = 0.08


def build_joint_index(robot_id: int) -> dict[str, int]:
    """Return a mapping {joint_name: joint_index} for all joints."""
    index = {}
    n = pb.getNumJoints(robot_id)
    for i in range(n):
        info = pb.getJointInfo(robot_id, i)
        index[info[1].decode()] = i
    return index


def run_simulation(gui: bool = True, record_forces: bool = True) -> dict:
    """
    Run the trot gait simulation.

    Parameters
    ----------
    gui            : If True, open the PyBullet GUI window.
    record_forces  : If True, record joint reaction forces each step.

    Returns
    -------
    dict with force / torque logs per leg.
    """
    mode = pb.GUI if gui else pb.DIRECT
    physics_client = pb.connect(mode)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    pb.setTimeStep(SIM_DT)

    # Load ground plane
    plane_id = pb.loadURDF("plane.urdf")

    # Load robot — spawn above ground so feet clear the floor
    start_pos = [0, 0, 0.72]   # Highland Cow hip height ~0.464m + clearance
    start_orn = pb.getQuaternionFromEuler([0, 0, 0])
    robot_id  = pb.loadURDF(
        os.path.normpath(URDF_PATH),
        start_pos,
        start_orn,
        useFixedBase=False,
    )

    joint_index = build_joint_index(robot_id)

    # Pre-generate trajectory
    gait = generate_gait_trajectory(n_steps=200)
    steps_per_cycle = int(GAIT_PERIOD / SIM_DT)
    total_steps     = steps_per_cycle * N_CYCLES

    # Storage
    logs: dict[str, list] = {leg: [] for leg in LEG_NAMES}

    for step in range(total_steps):
        gait_phase = (step % steps_per_cycle) / steps_per_cycle
        traj_idx   = int(gait_phase * 200)

        for leg in LEG_NAMES:
            th_hip  = gait[leg]["theta_hip"][traj_idx]
            th_knee = gait[leg]["theta_knee"][traj_idx]

            # Cannon bone angle via pantograph linkage
            th_cannon = CANNON_LEAN - (th_hip + th_knee)

            hip_jnt    = JOINT_MAP[leg]["hip"]
            thigh_jnt  = JOINT_MAP[leg]["thigh"]
            knee_jnt   = JOINT_MAP[leg]["knee"]
            cannon_jnt = JOINT_MAP[leg]["cannon"]

            # Hip yaw stays at 0 (no turning in basic trot)
            if hip_jnt in joint_index:
                pb.setJointMotorControl2(
                    robot_id,
                    joint_index[hip_jnt],
                    pb.POSITION_CONTROL,
                    targetPosition=0.0,
                    force=200,
                    positionGain=0.5,
                    velocityGain=0.1,
                )
            if thigh_jnt in joint_index:
                pb.setJointMotorControl2(
                    robot_id,
                    joint_index[thigh_jnt],
                    pb.POSITION_CONTROL,
                    targetPosition=th_hip,
                    force=500,
                    positionGain=0.5,
                    velocityGain=0.1,
                )
            if knee_jnt in joint_index:
                pb.setJointMotorControl2(
                    robot_id,
                    joint_index[knee_jnt],
                    pb.POSITION_CONTROL,
                    targetPosition=th_knee,
                    force=800,
                    positionGain=0.5,
                    velocityGain=0.1,
                )
            if cannon_jnt in joint_index:
                pb.setJointMotorControl2(
                    robot_id,
                    joint_index[cannon_jnt],
                    pb.POSITION_CONTROL,
                    targetPosition=th_cannon,
                    force=100,
                    positionGain=0.5,
                    velocityGain=0.1,
                )

        pb.stepSimulation()

        if record_forces:
            for leg in LEG_NAMES:
                knee_jnt = JOINT_MAP[leg]["knee"]
                if knee_jnt in joint_index:
                    state = pb.getJointState(robot_id, joint_index[knee_jnt])
                    # state[3] is the applied joint motor torque
                    logs[leg].append(
                        {
                            "step":       step,
                            "phase":      gait_phase,
                            "knee_angle": state[0],
                            "knee_torque": state[3],
                        }
                    )

        if gui:
            time.sleep(SIM_DT)

    pb.disconnect()

    # Convert lists to numpy arrays
    results = {}
    for leg in LEG_NAMES:
        entries = logs[leg]
        results[leg] = {
            "phase":       np.array([e["phase"]       for e in entries]),
            "knee_angle":  np.array([e["knee_angle"]  for e in entries]),
            "knee_torque": np.array([e["knee_torque"] for e in entries]),
        }

    return results


def print_force_summary(sim_results: dict) -> None:
    """Print a summary of peak and mean knee torques from simulation."""
    print("\n" + "=" * 55)
    print("  SIMULATION FORCE SUMMARY — knee joint torques [N·m]")
    print("=" * 55)
    for leg, data in sim_results.items():
        t = np.abs(data["knee_torque"])
        print(f"  {leg.upper()}  peak={t.max():.1f}  mean={t.mean():.1f}  rms={np.sqrt(np.mean(t**2)):.1f}")
    print("=" * 55)
