#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('toolhead_interface'),
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='toolhead_interface',
            executable='toolhead_server',
            name='toolhead_service_node',
            parameters=[params]
        ),
    ])