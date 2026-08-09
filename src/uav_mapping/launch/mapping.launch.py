from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory(
            "uav_mapping"
        ),
        "config",
        "mapping.yaml"
    )

    mapping_node = Node(
        package="uav_mapping",
        executable="mapping_node",
        name="uav_mapping",
        output="screen",
        parameters=[
            config_file
        ]
    )

    return LaunchDescription([
        mapping_node
    ])