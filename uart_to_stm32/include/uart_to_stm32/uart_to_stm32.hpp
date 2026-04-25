#ifndef UART_TO_STM32__UART_TO_STM32_HPP_
#define UART_TO_STM32__UART_TO_STM32_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <serial_comm/serial_comm.h>

namespace uart_to_stm32
{

/**
 * @class UartToStm32
 * @brief 在 STM32 飞控和 ROS 2 系统之间桥接串行通信的 ROS 2 节点
 * 
 * 该节点订阅里程计和目标速度话题，然后通过串行端口将数据转发到 STM32。
 * 它还从 STM32 接收传感器数据（高度、状态）并发布到 ROS 2 话题。
 * 
 * 支持的协议帧 ID：
 * - 0x32: 速度反馈（机体坐标系，cm/s）
 * - 0x31: 目标速度指令（cm/s, deg/s）
 * - 0x05: 高度数据（cm）
 * - 0xF1: ST 就绪查询/响应
 * - 0xA2: A2 就绪响应
 * - 0xB1: 飞控的目标速度反馈
 */
class UartToStm32
{
public:
  /**
   * @brief 构造 UartToStm32 对象
   * @param node ROS 2 节点的共享指针
   */
  explicit UartToStm32(rclcpp::Node::SharedPtr node);

  /**
   * @brief 销毁 UartToStm32 对象
   * 停止串行通信并关闭端口
   */
  ~UartToStm32();

  /**
   * @brief 初始化 UART 到 STM32 桥接器
   * @details 执行以下步骤：
   * 1. 从 ROS 参数加载串行配置
   * 2. 初始化串行端口
   * 3. 创建里程计和目标速度的订阅者
   * 4. 创建高度、ST 就绪状态和任务步骤的发布者
   * 5. 启动协议接收线程
   * @return 初始化成功返回 true，否则返回 false
   */
  bool initialize();

private:
  /**
   * @brief 里程计回调函数
   * @param msg 来自 DCL-SLAM 的里程计消息
   * @details 提取线速度和偏航角，然后发送到 STM32
   */
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void droneCommandCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  /**
   * @brief 目标速度回调函数
   * @param msg 来自 PID 控制器的目标速度指令
   * @details 期望 4 个浮点值：[vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]
   */
  void targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  /**
   * @brief EGO 目标速度回调函数
   * @param msg 来自 EGO 规划器的位置控制指令
   * @details 从 /position_cmd 中提取速度和 yaw_dot，并发送到 STM32
   */
  void ego_targetVelocityCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);

  /**
   * @brief 速度来源回调函数
   * @param msg 速度来源标识
   * @details 接收 /speed_source 并转发到 STM32（帧 ID: 0x34）
   */
  void sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity);

  /**
   * @brief 发送目标速度指令到 STM32
   * @param vx_cm_per_s X 轴速度指令（cm/s）
   * @param vy_cm_per_s Y 轴速度指令（cm/s）
   * @param vz_cm_per_s Z 轴速度指令（cm/s）
   * @param vyaw_deg_per_s 偏航角速度指令（deg/s）
   */
  void sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s);

  /**
   * @brief 发送 A2 就绪响应到 STM32
   * @details 响应 ST 就绪查询（0xF1）的就绪状态
   */
  void sendA2ReadyResponse();

  /**
   * @brief 协议数据处理回调
   * @param id 协议帧 ID
   * @param data 协议帧数据
   * @details 处理来自 STM32 的串行数据：
   * - 0xF1: ST 就绪查询
   * - 0x05: 高度数据
   * - 0xB1: 目标速度反馈
   */
  void protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data);

  /// ROS 2 节点共享指针
  rclcpp::Node::SharedPtr node_;

  /// 里程计订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr drone_command_sub_;

  /// 目标速度订阅者
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_sub_;

  /// EGO 位置控制命令订阅者
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;

  /// 速度来源订阅者
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr speed_source_sub_;

  /// 串行通信接口
  std::unique_ptr<serial_comm::SerialComm> serial_comm_;

  /// 高度发布者（来自 STM32 气压计/激光雷达）
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub_;

  /// ST 就绪状态发布者（瞬态本地）
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_st_ready_pub_;

  /// 任务步骤发布者
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mission_step_pub_;

  uint8_t drone_command_ = 0;

  /// 标志表示是否已发布 ST 就绪状态
  bool has_st_ready_pub_;

  /// 串行端口设备路径（可通过 ROS 参数配置）
  std::string serial_port_;

  /// 串行端口波特率（可通过 ROS 参数配置）
  int baud_rate_;

  /// 日志节流间隔（毫秒）
  int log_throttle_interval_;

  /// 目标速度输入来源选择：true=/target_velocity, false=/position_cmd
  bool use_target_velocity_topic_;

  /// Latest yaw from odometry (rad), used for world->body conversion.
  double current_yaw_rad_{0.0};

  /// Whether odometry yaw has been received at least once.
  bool has_odom_{false};

  /// Protect shared yaw state across callbacks.
  std::mutex odom_mutex_;

  // Protocol frame IDs 
  static constexpr uint8_t VELOCITY_FRAME_ID = 0x32;
  static constexpr uint8_t TARGET_VELOCITY_FRAME_ID = 0x31;
  static constexpr uint8_t HEIGHT_FRAME_ID = 0x05;
  static constexpr uint8_t ST_READY_QUERY_ID = 0xF1;
  static constexpr uint8_t A2_READY_RESP_ID = 0xA2;

  // Default configuration values
  static constexpr char DEFAULT_SERIAL_PORT[] = "/dev/ttyS4";
  static constexpr int DEFAULT_BAUD_RATE = 115200;
  static constexpr int DEFAULT_LOG_THROTTLE_INTERVAL = 2000;
  static constexpr bool DEFAULT_USE_TARGET_VELOCITY_TOPIC = true;
};

}  // namespace uart_to_stm32

#endif  // UART_TO_STM32__UART_TO_STM32_HPP_
