"""
display.launch.py  — Load the robot URDF into RViz2 for visual inspection.

Usage:
    ros2 launch robotic_horse_description display.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = FindPackageShare('robotic_horse_description')

    urdf_file = PathJoinSubstitution([pkg_description, 'urdf', 'robotic_horse.urdf'])

    robot_description = Command([FindExecutable(name='cat'), ' ', urdf_file])

    return LaunchDescription([
        # Publish TF transforms from URDF joint angles
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # GUI slider panel to manually move joints — great for inspecting the URDF
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2 visualiser
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
