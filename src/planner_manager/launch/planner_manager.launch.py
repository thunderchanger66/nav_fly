import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory(
            "planner_manager"
        ),
        "config",
        "planner_manager.yaml"
    )

    manager_node = Node(
        package="planner_manager",
        executable="planner_manager_node",
        name="planner_manager",
        output="screen",
        parameters=[
            config_file
        ]
    )

    return LaunchDescription([
        manager_node
    ])
