import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动RViz2"
    )


    # =========================================================
    # uav_mapping
    # =========================================================

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("uav_mapping"),
                "launch",
                "mapping.launch.py"
            )
        )
    )


    # =========================================================
    # astar_planner
    # =========================================================

    astar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astar_planner"),
                "launch",
                "astar.launch.py"
            )
        )
    )

    optimizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    "trajectory_optimizer"
                ),
                "launch",
                "trajectory_optimizer.launch.py"
            )
        )
    )

    # =========================================================
    # RViz
    # =========================================================

    rviz_config = os.path.join(
        get_package_share_directory(
            "bringup"
        ),
        "rviz",
        "navigation.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            rviz_config
        ],
        parameters=[
            {
                "use_sim_time": True
            }
        ],
        output="screen",
        condition=IfCondition(use_rviz)
    )


    return LaunchDescription([
        declare_use_rviz,

        mapping_launch,
        astar_launch,
        optimizer_launch,
        rviz
    ])