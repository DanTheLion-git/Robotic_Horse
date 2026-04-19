"""
full_sim.launch.py  —  Single entry-point for the complete Robotic Horse simulation.

Starts (in order):
  1. Gazebo Harmonic GUI  + robot spawn + ros2_control controllers
  2. Gait node  (after 9 s, controllers are active)

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


def generate_launch_description():
    pkg_control     = FindPackageShare('robotic_horse_control')

    # ── 1. Gazebo + controllers ──────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, 'launch', 'gazebo.launch.py'])
        )
    )

    # ── 2. Blendspace node ───────────────────────────────────────────
    blendspace_node = Node(
        package='robotic_horse_control',
        executable='blendspace_node',
        name='blendspace_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # (force_node removed — obsolete ballscrew model)
    # (RViz2 removed — broken display, not needed for simulation)

    return LaunchDescription([
        # Gazebo starts immediately
        gazebo_launch,

        # Blendspace controller after 9 s (controllers active by then)
        TimerAction(period=9.0, actions=[blendspace_node]),
    ])
