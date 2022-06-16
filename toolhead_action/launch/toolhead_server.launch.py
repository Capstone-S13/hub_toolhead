from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('toolhead_action'),
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='toolhead_action',
            executable='toolhead_action_server',
            name='toolhead_action_server_node',
            parameters=[params]
        ),
    ])