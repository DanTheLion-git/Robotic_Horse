"""
gazebo.launch.py  —  Spawn the Robotic Horse in Gazebo Classic and start
all ros2_control controllers.

Usage:
    ros2 launch robotic_horse_control gazebo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description = FindPackageShare('robotic_horse_description')
    pkg_control     = FindPackageShare('robotic_horse_control')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')

    # ── Robot description ────────────────────────────────────────────
    urdf_file = PathJoinSubstitution([pkg_description, 'urdf', 'robotic_horse.urdf'])
    robot_description = Command([FindExecutable(name='cat'), ' ', urdf_file])

    # ── World file ───────────────────────────────────────────────────
    world_file = PathJoinSubstitution([pkg_control, 'worlds', 'robotic_horse.world'])

    # ── Launch Gazebo ─────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
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
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robotic_horse',
            '-x', '0.0', '-y', '0.0', '-z', '1.15',  # spawn above ground
        ],
        output='screen',
    )

    # ── Start joint_state_broadcaster (reads joint states from Gazebo) ─
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ── Start the leg position controller ────────────────────────────
    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Start controllers only after the robot has been spawned
    start_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, leg_controller_spawner],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        start_controllers,
    ])
