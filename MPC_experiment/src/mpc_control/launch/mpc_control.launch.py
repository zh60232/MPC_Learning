import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("mpc_control")
    rviz_config = os.path.join(package_share, "config", "mpc_visualization.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz with MPC path visualization enabled.",
        ),
        DeclareLaunchArgument(
            "simulate",
            default_value="true",
            description="Run the built-in closed-loop vehicle simulation.",
        ),
        Node(
            package="mpc_control",
            executable="mpc_control",
            name="mpc_control",
            output="screen",
            parameters=[{"simulate": LaunchConfiguration("simulate")}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="mpc_rviz",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])