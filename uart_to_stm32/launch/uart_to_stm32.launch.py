"""
Launch file for UART to STM32 bridge node.

This node provides serial communication between ROS 2 and STM32 flight controller.
It subscribes to odometry and target velocity topics and forwards data to STM32,
while also receiving sensor data (height, status) from STM32.

Parameters:
    serial_port (str): Serial port device path (default: "/dev/ttyS4")
    baud_rate (int): Serial port baud rate (default: 115200)
    log_throttle_interval (int): Log throttle interval in milliseconds (default: 5000)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for UART to STM32 node."""
    return LaunchDescription([
        Node(
            package='uart_to_stm32',
            executable='uart_to_stm32_node',
            name='uart_to_stm32',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyS4',
                'baud_rate': 115200,
                'log_throttle_interval': 5000,
            }]
        )
    ])
