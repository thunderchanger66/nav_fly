from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)

import os


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory(
            "astar_planner"
        ),
        "config",
        "astar.yaml"
    )


    astar_node = Node(
        package="astar_planner",
        executable="astar_node",
        name="astar_planner",
        output="screen",
        parameters=[
            config_file
        ]
    )


    return LaunchDescription([
        astar_node
    ])