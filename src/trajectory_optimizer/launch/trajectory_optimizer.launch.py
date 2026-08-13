import os

from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory(
            "trajectory_optimizer"
        ),
        "config",
        "trajectory_optimizer.yaml"
    )


    optimizer_node = Node(
        package="trajectory_optimizer",
        executable="trajectory_optimizer_node",
        name="trajectory_optimizer",
        output="screen",
        parameters=[
            config_file
        ]
    )


    return LaunchDescription([
        optimizer_node
    ])