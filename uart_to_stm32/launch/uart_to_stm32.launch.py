"""
UART 到 STM32 桥接节点的启动文件。

该节点提供 ROS 2 和 STM32 飞控之间的串行通信。
它订阅里程计和目标速度话题并将数据转发到 STM32，
同时也从 STM32 接收传感器数据（高度、状态）。

参数：
    serial_port (str): 串行端口设备路径（默认："/dev/ttyS4"）
    baud_rate (int): 串行端口波特率（默认：115200）
    log_throttle_interval (int): 日志节流间隔（毫秒）（默认：5000）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """生成 UART 到 STM32 节点的启动描述。"""
    use_target_velocity_topic = LaunchConfiguration('use_target_velocity_topic')
    namespace = LaunchConfiguration('namespace', default='a')
    drone_id = LaunchConfiguration('drone_id', default='0')
    odom_topic = LaunchConfiguration('odom_topic', default='Odometry')
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=namespace, description='Namespace for the drone'
    )
    drone_id_arg = DeclareLaunchArgument("drone_id",default_value=drone_id,description="Drone ID")
    odom_topic_arg = DeclareLaunchArgument("odom_topic",default_value=odom_topic,description="Odometry topic")
    return LaunchDescription([
        namespace_arg,
        drone_id_arg,
        odom_topic_arg,
        DeclareLaunchArgument(
            'use_target_velocity_topic',
            default_value='false',
            description='true: subscribe /target_velocity; false: subscribe /pos_cmd'
        ),
        Node(
            package='uart_to_stm32',
            executable='uart_to_stm32_node',
            name='uart_to_stm32',
            output='screen',
            namespace=namespace,
            remappings=[
                ('Odometry', ['drone_', drone_id, '_', odom_topic]),
                ('pos_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
                ('Odom_high_fre',['drone_', drone_id, '_', 'Odom_high_fre'])
            ],
            parameters=[{
                'serial_port': '/dev/ttyS4',
                'baud_rate': 921600,
                'log_throttle_interval': 5000,
                'use_target_velocity_topic': use_target_velocity_topic,
            }]
        )
    ])
