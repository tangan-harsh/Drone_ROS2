import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 包路径
    my_carto_pkg_share = FindPackageShare(package="my_carto_pkg").find("my_carto_pkg")
    uart_to_stm32_pkg_share = FindPackageShare(package="uart_to_stm32").find("uart_to_stm32")
    pid_control_pkg_share = FindPackageShare(package="pid_control_pkg").find("pid_control_pkg")
    activity_control_pkg_share = FindPackageShare(package="activity_control_pkg").find("activity_control_pkg")

    # === demo1 原有部分 ===
    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_carto_pkg_share, "launch", "fly_carto.launch.py")
        )
    )

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

    # 航点（内部会起 route_target_publisher + route_test_node）
    # 通过 arguments 传参给进程内节点：开启视觉接管与微调发布频率
    route_test_node = Node(
        package="activity_control_pkg",
        executable="route_test_node",
        name="route_test_node",
        output="screen",
        arguments=[
            "--ros-args",
            "-p",
            "enable_visual_takeover:=true",
            "-p",
            "visual_takeover_distance_cm:=10.0",
            "-p",
            "fine_offset_limit_cm:=15.0",
            "-p",
            "fine_target_publish_hz:=10.0",
            "-p",
            "laser_hold_sec:=0.5",
        ],
    )

    # === 摄像头二维码识别（左右相机） ===
    qr_decoder_right = Node(
        package="opencv01",
        executable="qr_decoder_right",
        name="qr_decoder_right",
        output="screen",
        parameters=[
            {"camera_device": "/dev/video0"},
            {"eps_x": 0.40},
            {"eps_y": 0.40},
            {"eps_x_laser":0.25},
            {"stable_frames": 1},
            {"enable_debug_image": False},
            {"enable_gui": False},  # 禁用GUI窗口，避免阻塞（飞行时建议关闭）
            {"decode_interval": 3},  # 每5帧解码一次，降低CPU负载避免卡死
            {"laser_pin": 10},  # 右侧激光引脚（实际连接在 Pin 10）
        ],
    )

    qr_decoder_left = Node(
        package="opencv01",
        executable="qr_decoder_left",
        name="qr_decoder_left",
        output="screen",
        parameters=[
            {"camera_device": "/dev/video2"},
            {"eps_x": 0.40},
            {"eps_y": 0.40},
            {"eps_x_laser":0.25},
            {"stable_frames": 1},
            {"enable_debug_image": False},
            {"enable_gui": False},  # 禁用GUI窗口，避免阻塞（飞行时建议关闭）
            {"decode_interval": 3},  # 每5帧解码一次，降低CPU负载避免卡死
            {"laser_pin": 13},  # 左侧激光引脚（实际连接在 Pin 13）
        ],
    )

    # === 微调节点（左右各一份，按前缀订阅） ===
    qr_fine_tune_right = Node(
        package="opencv01",
        executable="qr_fine_tune",
        name="qr_fine_tune_right",
        output="screen",
        parameters=[
            {"input_prefix": "/qr_right"},
            {"output_topic": "/qr_right/fine_offset_body_cm"},
            {"publish_hz": 5.0},
            {"ema_alpha": 0.9},
            {"deadband_ex": 0.05},
            {"deadband_ey": 0.05},
            {"max_step_cm": 1.0},
            {"k_body_x_cm": 20.0},
            {"k_body_y_cm": 0.0},
            {"k_body_z_cm": 20.0},
            {"max_cm": 15.0},
            # 右相机：图像左侧 -> 机身x增大（dx = -ex * k）
            {"invert_body_x": True},
            {"invert_body_y": False},
            # 图像上方(ey<0) -> z增大（dz = -ey * k）
            {"invert_body_z": True},
            {"publish_zero_when_aligned": True},
        ],
    )

    qr_fine_tune_left = Node(
        package="opencv01",
        executable="qr_fine_tune",
        name="qr_fine_tune_left",
        output="screen",
        parameters=[
            {"input_prefix": "/qr_left"},
            {"output_topic": "/qr_left/fine_offset_body_cm"},
            {"publish_hz": 5.0},
            {"ema_alpha": 0.9},
            {"deadband_ex": 0.05},
            {"deadband_ey": 0.05},
            {"max_step_cm": 1.0},
            {"k_body_x_cm": 20.0},
            {"k_body_y_cm": 0.0},
            {"k_body_z_cm": 20.0},
            {"max_cm": 15.0},
            # 左相机：图像左侧 -> 机身x减小（dx = ex * k）
            {"invert_body_x": False},
            {"invert_body_y": False},
            {"invert_body_z": True},
            {"publish_zero_when_aligned": True},
        ],
    )

    # 启动顺序：先系统，再相机/微调（给 TF/串口一点启动时间）
    return LaunchDescription(
        [
            fly_carto_launch,
            uart_to_stm32_launch,
            position_pid_controller_launch,
            route_test_node,
            TimerAction(
                period=1.0,
                actions=[
                    qr_decoder_right,
                    qr_decoder_left,
                    qr_fine_tune_right,
                    qr_fine_tune_left,
                ],
            ),
        ]
    )
