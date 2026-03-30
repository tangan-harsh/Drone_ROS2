import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 包路径
    uart_to_stm32_pkg_share = FindPackageShare(package="uart_to_stm32").find("uart_to_stm32")
    pid_control_pkg_share = FindPackageShare(package="pid_control_pkg").find("pid_control_pkg")
    activity_control_pkg_share = FindPackageShare(package="activity_control_pkg").find("activity_control_pkg")


    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_pkg_share, "launch", "uart_to_stm32.launch.py")
        )
    )

    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, "launch", "position_pid_controller.launch.py")
        )
    )

    route_test_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, "launch", "route_target_publisher.launch.py")
        )
    )

    return LaunchDescription(
        [
            uart_to_stm32_launch,
            position_pid_controller_launch,
            route_test_node,
        ]
    )
