from importlib import import_module
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover
    LaunchDescription = Any
    Node = Any


def generate_launch_description():
    namespace = LaunchConfiguration("namespace", default="a")
    drone_id = LaunchConfiguration("drone_id", default="0")
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value=namespace, description="Namespace for the drone"
    )
    drone_id_arg = DeclareLaunchArgument("drone_id", default_value=drone_id, description="Drone ID")
    launch_module = import_module("launch")
    launch_ros_actions = import_module("launch_ros.actions")
    LaunchDescription = getattr(launch_module, "LaunchDescription")
    Node = getattr(launch_ros_actions, "Node")

    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="route_target_publisher_node",
            name="route_target_publisher",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "map_frame": "a/camera_init",
                    "laser_link_frame": "a/body",
                    "output_topic": "target_position",
                    "position_tolerance_cm": 6.0,
                    "yaw_tolerance_deg": 5.0,
                    "height_tolerance_cm": 6.0,
                    "waypoint_preset": "simple_4",  # Options: "test_19", "simple_4", "none"
                }
            ],
        )
    ])
