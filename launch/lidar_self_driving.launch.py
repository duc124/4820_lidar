import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='articubot_one',  
            executable='lidar_self_driving.py',
            name='lidar_navigation',
            output='screen'
        )
    ])
