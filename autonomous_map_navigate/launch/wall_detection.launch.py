#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    wall_detection_node = Node(
        package='autonomous_map_navigate',
        executable='wall_detection',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        wall_detection_node
    ])
