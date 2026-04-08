"""
robot_control.launch.py  —  Start the gait node and force reporter on top
of a running Gazebo session.

Usage (in a separate terminal after gazebo.launch.py):
    ros2 launch robotic_horse_control robot_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Publishes joint trajectory commands from the trot gait planner
        Node(
            package='robotic_horse_control',
            executable='gait_node',
            name='gait_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Subscribes to /joint_states, computes and publishes ballscrew forces
        Node(
            package='robotic_horse_control',
            executable='force_node',
            name='force_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
