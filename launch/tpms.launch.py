import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = '{}/../conf/tpms.yaml'.format(
        os.path.abspath(os.path.dirname(os.path.realpath(__file__))))
    return LaunchDescription([
        Node(
            package='ble_pkg',
            executable='tpms_monitor',
            parameters=[parameters_file_path]
        ),
    ])
