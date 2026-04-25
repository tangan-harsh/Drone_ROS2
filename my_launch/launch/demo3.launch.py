import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    namespace = LaunchConfiguration("namespace", default="a")
    drone_id = LaunchConfiguration("drone_id", default="0")
    odom_topic = LaunchConfiguration('odom_topic', default='Odometry')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=namespace,
        description="Namespace for the drone"
    )
    drone_id_arg = DeclareLaunchArgument(
        "drone_id",
        default_value=drone_id,
        description="Drone ID"
    )
    # 包路径
    uart_to_stm32_pkg_share = FindPackageShare(package="uart_to_stm32").find("uart_to_stm32")
    pid_control_pkg_share = FindPackageShare(package="pid_control_pkg").find("pid_control_pkg")
    activity_control_pkg_share = FindPackageShare(package="activity_control_pkg").find("activity_control_pkg")


    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_pkg_share, "launch", "uart_to_stm32.launch.py")
        ),
        launch_arguments=[
            ("namespace", namespace),
            ("drone_id", drone_id),
            ("odom_topic", odom_topic)
        ]
    )

    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, "launch", "position_pid_controller.launch.py")
        ),
        launch_arguments=[
            ("namespace", namespace),
            ("drone_id", drone_id)
        ]
    )

    route_test_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, "launch", "route_target_publisher.launch.py")
        ),
        launch_arguments=[
            ("namespace", namespace),
            ("drone_id", drone_id)
        ]
    )

    return LaunchDescription(
        [
            namespace_arg,
            drone_id_arg,
            odom_topic_cmd,
            uart_to_stm32_launch,
            position_pid_controller_launch,
            route_test_node,
        ]
    )
