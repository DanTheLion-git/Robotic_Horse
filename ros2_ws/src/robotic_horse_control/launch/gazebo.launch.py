"""
gazebo.launch.py  —  Spawn the Robotic Horse in Gazebo Harmonic and start
all ros2_control controllers.

Adapted for ROS2 Jazzy + Gazebo Harmonic (gz sim) on Ubuntu 24.04.

Usage:
    ros2 launch robotic_horse_control gazebo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description = FindPackageShare('robotic_horse_description')
    pkg_control     = FindPackageShare('robotic_horse_control')

    # ── Robot description ────────────────────────────────────────────
    urdf_file = PathJoinSubstitution([pkg_description, 'urdf', 'robotic_horse.urdf.xacro'])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str
    )

    # ── Controller param files ───────────────────────────────────────
    jsb_params   = PathJoinSubstitution([pkg_control, 'config', 'joint_state_broadcaster.yaml'])
    leg_params   = PathJoinSubstitution([pkg_control, 'config', 'leg_controller.yaml'])
    ctrl_params  = PathJoinSubstitution([pkg_control, 'config', 'ros2_control.yaml'])

    # ── World file ───────────────────────────────────────────────────
    world_file = PathJoinSubstitution([pkg_control, 'worlds', 'robotic_horse.world'])

    # ── Launch Gazebo Harmonic (gz sim) ───────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Publish TF from URDF ──────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # ── Spawn the robot into Gazebo ───────────────────────────────────
    # base_link is now the root. Spawn at z=1.33m (body_center 1.27m + 6cm clearance).
    # With neutral leg angles (56-deg knee), feet just touch the ground at z=1.27m.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robotic_horse',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '1.33',
        ],
        output='screen',
    )

    # ── Bridge gz clock + IMU → ROS2 ─────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    # ── Start joint_state_broadcaster ────────────────────────────────
    # -p jsb_params  : declares the controller type (must be top-level key)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '-p', jsb_params,
        ],
        output='screen',
    )

    # ── Start the leg position controller ────────────────────────────
    # -p leg_params  : declares the controller type
    # -p ctrl_params : supplies joints, PID gains, interfaces
    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'leg_controller',
            '--controller-manager', '/controller_manager',
            '-p', leg_params,
            '-p', ctrl_params,
        ],
        output='screen',
    )

    # Delay controller spawners until gz_ros_control has registered hardware
    # interfaces (~2 s after entity spawn; 6 s is a safe margin).
    start_controllers = TimerAction(
        period=6.0,
        actions=[joint_state_broadcaster_spawner, leg_controller_spawner],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        gz_bridge,
        spawn_robot,
        start_controllers,
    ])
