import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # === 右侧摄像头二维码识别 ===
        Node(
            package="opencv01",
            executable="qr_decoder_right",
            name="qr_decoder_right",
            output="screen",
            parameters=[
                {"camera_device": "/dev/video0"},
                {"eps_x": 0.4},
                {"eps_y": 0.4},
                {"eps_x_laser":0.25},
                {"stable_frames": 1},
                {"enable_debug_image": True},
                {"enable_gui": True},  # 可以通过 ros2 param set 动态开启
                {"decode_interval": 3},
                {"laser_pin": 10},  # 右侧激光引脚（实际连接在 Pin 10）
            ],
        ),

        # # === 左侧摄像头二维码识别 ===
        # Node(
        #     package="opencv01",
        #     executable="qr_decoder_left",
        #     name="qr_decoder_left",
        #     output="screen",
        #     parameters=[
        #         {"camera_device": "/dev/video2"},
        #         {"eps_x": 0.4},
        #         {"eps_y": 0.4},
        #         {"eps_x_laser":0.25},
        #         {"stable_frames": 1},
        #         {"enable_debug_image": True},
        #         {"enable_gui": False},  # 可以通过 ros2 param set 动态开启
        #         {"decode_interval": 3},
        #         {"laser_pin": 13},  # 左侧激光引脚（实际连接在 Pin 13）
        #     ],
        # ),

        # # === 右侧微调节点（可选，如果需要微调功能） ===
        # Node(
        #     package="opencv01",
        #     executable="qr_fine_tune",
        #     name="qr_fine_tune_right",
        #     output="screen",
        #     parameters=[
        #         {"input_prefix": "/qr_right"},
        #         {"output_topic": "/qr_right/fine_offset_body_cm"},
        #         {"publish_hz": 5.0},
        #         {"ema_alpha": 0.9},
        #         {"deadband_ex": 0.05},
        #         {"deadband_ey": 0.05},
        #         {"max_step_cm": 1.0},
        #         {"k_body_x_cm": 20.0},
        #         {"k_body_y_cm": 0.0},
        #         {"k_body_z_cm": 20.0},
        #         {"max_cm": 15.0},
        #         {"invert_body_x": True},
        #         {"invert_body_y": False},
        #         {"invert_body_z": True},
        #         {"publish_zero_when_aligned": True},
        #     ],
        # ),

        # # === 左侧微调节点（可选，如果需要微调功能） ===
        # Node(
        #     package="opencv01",
        #     executable="qr_fine_tune",
        #     name="qr_fine_tune_left",
        #     output="screen",
        #     parameters=[
        #         {"input_prefix": "/qr_left"},
        #         {"output_topic": "/qr_left/fine_offset_body_cm"},
        #         {"publish_hz": 5.0},
        #         {"ema_alpha": 0.9},
        #         {"deadband_ex": 0.05},
        #         {"deadband_ey": 0.05},
        #         {"max_step_cm": 1.0},
        #         {"k_body_x_cm": 20.0},
        #         {"k_body_y_cm": 0.0},
        #         {"k_body_z_cm": 20.0},
        #         {"max_cm": 15.0},
        #         {"invert_body_x": False},
        #         {"invert_body_y": False},
        #         {"invert_body_z": True},
        #         {"publish_zero_when_aligned": True},
        #     ],
        # ),
    ])
