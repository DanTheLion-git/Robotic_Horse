"""
blendspace.launch.py  —  Start the blendspace node instead of gait_node.

Usage (after gazebo.launch.py is running):
    ros2 launch robotic_horse_control blendspace.launch.py

Then control the robot by publishing to /cmd_vel:
    ros2 topic pub /cmd_vel geometry_msgs/Twist \
        "{linear: {x: 1.0}, angular: {z: 0.0}}"
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'max_speed',
            default_value='1.0',
            description='Linear speed [m/s] that maps to normalised speed=1',
        ),
        DeclareLaunchArgument(
            'max_angular',
            default_value='1.5',
            description='Angular rate [rad/s] that maps to normalised rate=1',
        ),

        # Blendspace node — replaces gait_node when using animation files
        Node(
            package='robotic_horse_control',
            executable='blendspace_node',
            name='blendspace_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'max_speed':    LaunchConfiguration('max_speed'),
                'max_angular':  LaunchConfiguration('max_angular'),
                'publish_rate_hz': 50.0,
            }],
        ),

        # Force reporter still runs in parallel
        Node(
            package='robotic_horse_control',
            executable='force_node',
            name='force_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
