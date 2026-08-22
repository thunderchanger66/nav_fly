import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory(
            "px4_controller"
        ),
        "config",
        "px4_controller.yaml"
    )

    controller_node = Node(
        package="px4_controller",
        executable="px4_controller_node",
        name="px4_controller",
        output="screen",
        parameters=[
            config_file
        ]
    )

    return LaunchDescription([
        controller_node
    ])
