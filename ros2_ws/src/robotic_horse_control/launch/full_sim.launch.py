"""
full_sim.launch.py  —  Single entry-point for the complete Robotic Horse simulation.

Starts (in order):
  1. Gazebo Harmonic GUI  + robot spawn + ros2_control controllers
  2. RViz2 with the Robotic Horse config  (after 4 s, display is ready)
  3. Gait node  (after 9 s, controllers are active)
  4. Force / ballscrew marker node  (together with gait node)

Usage:
    ros2 launch robotic_horse_control full_sim.launch.py

To stop: Ctrl-C  (Gazebo exit triggers full shutdown via on_exit_shutdown).
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_control     = FindPackageShare('robotic_horse_control')
    pkg_description = FindPackageShare('robotic_horse_description')

    # ── 1. Gazebo + controllers ──────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, 'launch', 'gazebo.launch.py'])
        )
    )

    # ── 2. RViz2 ────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution(
        [pkg_description, 'rviz', 'robotic_horse.rviz']
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 3. Blendspace node ───────────────────────────────────────────
    blendspace_node = Node(
        package='robotic_horse_control',
        executable='blendspace_node',
        name='blendspace_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 4. Force / marker node ────────────────────────────────────────
    force_node = Node(
        package='robotic_horse_control',
        executable='force_node',
        name='force_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        # Gazebo starts immediately
        gazebo_launch,

        # RViz2 after 4 s (Gazebo window is up, robot description available)
        TimerAction(period=4.0, actions=[rviz2]),

        # Blendspace + force nodes after 9 s (controllers active by then)
        TimerAction(period=9.0, actions=[blendspace_node, force_node]),
    ])
