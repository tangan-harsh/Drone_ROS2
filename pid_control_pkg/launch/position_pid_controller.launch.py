from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace", default="a")
    drone_id = LaunchConfiguration("drone_id", default="0")
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value=namespace, description="Namespace for the drone"
    )
    drone_id_arg = DeclareLaunchArgument("drone_id",default_value=drone_id,description="Drone ID")
    return LaunchDescription([
        namespace_arg,
        drone_id_arg,
        Node(
            package="pid_control_pkg",
            executable="position_pid_controller",
            name="position_pid_controller",
            output="screen",
            namespace=namespace,
            parameters=[
                {
                    "control_frequency": 50.0,
                    "map_frame": "a/camera_init",
                    "laser_link_frame": "a/body",
                    "position_tolerance": 6.0,
                    "yaw_tolerance": 5.0,
                    "height_tolerance": 6.0,
                    "kp_xy": 0.8,
                    "ki_xy": 0.0,
                    "kd_xy": 0.2,
                    "kp_yaw": 1.0,
                    "ki_yaw": 0.0,
                    "kd_yaw": 0.2,
                    "kp_z": 1.0,
                    "ki_z": 0.0,
                    "kd_z": 0.2,
                    "max_linear_velocity": 33.0,
                    "max_angular_velocity": 30.0,
                    "max_vertical_velocity": 30.0,
                    "max_slow_velocity": 20.0,
                }
            ],
        )
    ])
