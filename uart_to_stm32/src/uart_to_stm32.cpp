#include "uart_to_stm32/uart_to_stm32.hpp"
#include <iostream>
  
#include <chrono>
#include <cmath>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/Geometry_msgs.hpp>

namespace uart_to_stm32
{

using namespace std::chrono_literals;

/*
  构造函数：
  初始化节点，将传入的 shared_ptr 节点句柄保存到成员变量 node_ 中，
  以便后续使用这个节点来创建发布者、订阅者、定时器和打印日志。
  并将成员变量初始化为默认值。
 */
UartToStm32::UartToStm32(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  update_rate_(0.0),
  current_yaw_(0.0),
  yaw_valid_(false),
  velocity_valid_(false),
  has_st_ready_pub_(false)
{
  RCLCPP_INFO(node_->get_logger(), "UartToStm32 created");
}

/*
  析构函数：
  这是为了安全退出。
  当对象被销毁时，停止串口协议接收线程，并关闭串口链接，释放资源。
 */
UartToStm32::~UartToStm32()
{
  if (serial_comm_) {
    serial_comm_->stop_protocol_receive();
    serial_comm_->close();
  }
}

/*
  初始化函数：
  这是一个非常核心的函数，相当于整个模块的 "Start" 按钮。
  1. 保存配置参数：更新频率。
  2. 初始化串口：尝试打开 /dev/ttyS6 串口，波特率 921600。如果失败会报错返回。
  3. 订阅话题：
     - /a/Odometry: DCL-SLAM 输出的里程计话题，包含位姿和速度。
     - /target_velocity: PID 控制器算出来的目标控制速度。
  4. 创建发布者：
     - /height: 发布高度信息（从串口读上来的气压计或激光测距数据）。
     - /is_st_ready: 发布底层 STM32 的就绪状态。
     - /mission_step: 发布任务步骤。
  5. 开启串口接收：注册回调函数 protocolDataHandler，只要串口收到数据就丢给它处理。
 */
bool UartToStm32::initialize(double update_rate, const std::string & source_frame, const std::string & target_frame)
{
  (void)source_frame;
  (void)target_frame;
  
  try {
    update_rate_ = update_rate;

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized with update rate: %.1f Hz", update_rate_);

    serial_comm_ = std::make_unique<serial_comm::SerialComm>();
    if (!serial_comm_->initialize("/dev/ttyS6", 921600)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize serial port /dev/ttyS6 at 921600 baudrate");
      RCLCPP_ERROR(node_->get_logger(), "Serial error: %s", serial_comm_->get_last_error().c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Serial port /dev/ttyS6 initialized at 921600 baudrate");

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/a/Odometry", 10,
      std::bind(&UartToStm32::odometryCallback, this, std::placeholders::_1));

    target_velocity_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_velocity", 10,
      std::bind(&UartToStm32::targetVelocityCallback, this, std::placeholders::_1));
    
    height_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/height", 10);
    is_st_ready_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/is_st_ready", rclcpp::QoS(10).transient_local());
    mission_step_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/mission_step", 10);

    has_st_ready_pub_ = false;

    serial_comm_->start_protocol_receive(
      [this](uint8_t id, const std::vector<uint8_t> & data) { protocolDataHandler(id, data); },
      [this](const std::string & err) {
        RCLCPP_WARN(node_->get_logger(), "Serial protocol error: %s", err.c_str());
      });

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized successfully");
    RCLCPP_INFO(node_->get_logger(), "Subscribed to /a/Odometry and /target_velocity topics");
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize topic subscriber: %s", e.what());
    return false;
  }
}

/*
  里程计回调函数 (/a/Odometry)：
  这个回调接收 DCL-SLAM 发布的里程计数据，包含：
  - pose.pose.position: 位置 (x, y, z)
  - pose.pose.orientation: 姿态四元数
  - twist.twist.linear: 线速度
  - twist.twist.angular: 角速度
  
  处理流程：
  1. 从 Odometry 消息中提取线速度。
  2. 从姿态四元数中提取 yaw 角。
  3. 将速度从 camera_init 坐标系转换到 body 坐标系。
  4. 通过串口发送给 STM32 底层。
 */
void UartToStm32::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_velocity_.linear = msg->twist.twist.linear;
  current_velocity_.angular = msg->twist.twist.angular;
  velocity_valid_ = true;

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
  yaw_valid_ = true;

  const double linear_x = msg->twist.twist.linear.x;
  const double linear_y = msg->twist.twist.linear.y;
  const double linear_z = msg->twist.twist.linear.z;
  const double angular_x = msg->twist.twist.angular.x;
  const double angular_y = msg->twist.twist.angular.y;
  const double angular_z = msg->twist.twist.angular.z;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Odometry: linear(%.3f, %.3f, %.3f) angular(%.3f, %.3f, %.3f) yaw=%.2f deg",
    linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, yaw * 180.0 / M_PI);

  if (yaw_valid_ && velocity_valid_) {
    Eigen::Vector3d linear_vel(linear_x, linear_y, linear_z);
    const Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
    sendVelocityToSerial(transformed_vel);
  }
}

// void UartToStm32::bluetoothCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
// {
//   if(msg->data.empty()) {
//     RCLCPP_WARN(node_->get_logger(), "Received empty bluetooth_data message.");
//     return;
//   }
//   constexpr uint8_t BLUETOOTH_FRAME_ID = 0x34;
//   if(serial_comm_ && serial_comm_->is_open()) {
//     if(serial_comm_->send_protocol_data(BLUETOOTH_FRAME_ID, static_cast<uint8_t>(msg->data.size()), msg->data)) {
//       RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
//         "Sent bluetooth data %d",msg->data[0]);
//     } else {
//       RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
//         "Failed to send bluetooth data: %s", serial_comm_->get_last_error().c_str());
//     }  } 
//     else {
//       RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
//         "Serial port is not open, cannot send bluetooth data");
//     }
// }

/*
  速度转换函数 (Map -> Body)：
  这是一个核心数学函数。它负责把全局地图坐标系下的速度矢量，旋转到机体坐标系下。
  使用了一个二维旋转矩阵 Rz（绕Z轴旋转）：
  |  cos(yaw)  sin(yaw)  0 |
  | -sin(yaw)  cos(yaw)  0 |
  |     0         0      1 |
  如果你想验证 "Turn 180" 时控制会不会反，秘密就在这个矩阵里。
  cos(180)=-1, sin(180)=0。
  x_body = -1 * x_map
  y_body = -1 * y_map
  这证明了：如果我们给的是地图速度，必须经过这个转换，才能变成正确的机体控制量。
 */
Eigen::Vector3d UartToStm32::transformVelocity(const Eigen::Vector3d & linear, double yaw)
{
  Eigen::Matrix3d Rz;
  Rz << std::cos(yaw), std::sin(yaw), 0.0,
    -std::sin(yaw), std::cos(yaw), 0.0,
    0.0, 0.0, 1.0;

  const Eigen::Vector3d transformed = Rz * linear;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Velocity transform: yaw=%.3f deg, original(%.3f,%.3f,%.3f) -> transformed(%.3f,%.3f,%.3f)",
    yaw * 180.0 / M_PI,
    linear.x(), linear.y(), linear.z(),
    transformed.x(), transformed.y(), transformed.z());

  return transformed;
}

/*
  发送 实际速度 反馈到底层串口 (ID: VELOCITY_FRAME_ID / 0x32)：
  将计算好的 Body 系下的实际速度 (x, y, z)，放大 100 倍转成 int16 发送。
  这通常是给飞控做观测用的反馈，不是控制指令。
 */
void UartToStm32::sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send velocity data");
    return;
  }

  try {
    constexpr double scale_factor = 100.0;

    const int16_t vel_x = static_cast<int16_t>(transformed_velocity.x() * scale_factor);
    const int16_t vel_y = static_cast<int16_t>(transformed_velocity.y() * scale_factor);
    const int16_t vel_z = static_cast<int16_t>(transformed_velocity.z() * scale_factor);

    std::vector<uint8_t> data(6);
    data[0] = static_cast<uint8_t>(vel_x & 0xFF);
    data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(vel_y & 0xFF);
    data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(vel_z & 0xFF);
    data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);

    if (serial_comm_->send_protocol_data(VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Sent velocity data: x=%d, y=%d, z=%d (cm/s)", vel_x, vel_y, vel_z);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to send velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendVelocityToSerial: %s", e.what());
  }
}

/*
  目标速度回调函数 (/target_velocity)：
  这是**真正的控制指令入口**。
  PID 控制器计算出期望速度，发到这里。
  函数提取 vx, vy, vz, vyaw，然后调用 sendTargetVelocityToSerial 发给底层。
 */
void UartToStm32::targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(node_->get_logger(),
      "Target velocity message should contain 4 float values [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]");
    return;
  }

  const float vx_cm_per_s = msg->data[0];
  const float vy_cm_per_s = msg->data[1];
  const float vz_cm_per_s = msg->data[2];
  const float vyaw_deg_per_s = msg->data[3];

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Target Velocity: linear(%.1f, %.1f, %.1f)cm/s angular(%.1f)deg/s",
    vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);

  sendTargetVelocityToSerial(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);
}

/*
  发送 目标速度 到底层串口 (ID: TARGET_VELOCITY_FRAME_ID / 0x31)：
  将 PID 算出的目标速度直接由浮点转整型，打包发给 STM32。
  
  **重要观察**：
  这里**直接**发送了传入的 vx, vy，没有在这个函数里做任何旋转变换！
  这意味着：如果传入的 /target_velocity 是 Map 坐标系的，这里直接发出去，那就是发了 Map 速度。
  如果 STM32 傻，那就真的反了。
  所以，你在 PID 控制器里加的那段 "Map -> Body" 的旋转代码是**绝对必要且关键的**！
 */
void UartToStm32::sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send target velocity data");
    return;
  }

  try {
    const int16_t vel_x = static_cast<int16_t>(std::lround(vx_cm_per_s));
    const int16_t vel_y = static_cast<int16_t>(std::lround(vy_cm_per_s));
    const int16_t vel_z = static_cast<int16_t>(std::lround(vz_cm_per_s));
    const int16_t vel_yaw = static_cast<int16_t>(std::lround(vyaw_deg_per_s));

    std::vector<uint8_t> data(8);
    data[0] = static_cast<uint8_t>(vel_x & 0xFF);
    data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(vel_y & 0xFF);
    data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(vel_z & 0xFF);
    data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>(vel_yaw & 0xFF);
    data[7] = static_cast<uint8_t>((vel_yaw >> 8) & 0xFF);

    if (serial_comm_->send_protocol_data(TARGET_VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "Sent target velocity data: x=%d, y=%d, z=%d, yaw=%d",
        vel_x, vel_y, vel_z, vel_yaw);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to send target velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendTargetVelocityToSerial: %s", e.what());
  }
}

/*
  串口协议数据接收回调：
  解析 STM32 发上来的数据包。
  - ST_READY_QUERY_ID (0xF1): 飞控询问 "Ready?"。我们如果准备好了就回个包。
    同时如果不为 1，说明底层还没准备好。
  - 0x05: 接收高度数据，发布到 /height。
  - 0xB1: 接收飞控当前的目标速度反馈（调试用）。
 */
void UartToStm32::protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data)
{
  switch (id) {
    case ST_READY_QUERY_ID: {
      if (data.size() < 9) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0xF1 data too short, len=%zu", data.size());
        break;
      }
      const uint8_t first = data[0];
      if (mission_step_pub_) {
        std_msgs::msg::UInt8 msg;
        msg.data = first;
        mission_step_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "Published /mission_step: %u (from 0xF1 frame)", static_cast<unsigned>(first));
      }
      if (has_st_ready_pub_) {
        break;
      }
      const uint8_t second = data[1];
      if (second == 1) {
        if (is_st_ready_pub_) {
          std_msgs::msg::UInt8 msg;
          msg.data = 1;
          is_st_ready_pub_->publish(msg);
          RCLCPP_INFO(node_->get_logger(), "Published /is_st_ready: 1 (from 0xF1 frame)");
        }
        // sendA2ReadyResponse();
        has_st_ready_pub_ = true;
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "0xF1 frame second byte != 1 (%u), ignoring", static_cast<unsigned>(second));
      }
      break;
    }
    case 0x05: {
      if (data.size() < 2) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0x05 data too short");
        break;
      }
      const int16_t value = static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8));
      std_msgs::msg::Int16 msg;
      msg.data = value;
      if (height_pub_) {
        height_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Published /height: %d", value);
      } else {
        RCLCPP_WARN(node_->get_logger(), "Height publisher not initialized");
      }
      break;
    }
    case 0xB1: {  // 飞控发送的目标速度数据
      if (data.size() < 8) {
        RCLCPP_WARN(node_->get_logger(),
                    "protocolDataHandler: ID 0xB1 data too short (expected 8, got %zu)", data.size());
        break;
      }

      int16_t vel_x = static_cast<int16_t>(data[0] | (data[1] << 8));
      int16_t vel_y = static_cast<int16_t>(data[2] | (data[3] << 8));
      int16_t vel_z = static_cast<int16_t>(data[4] | (data[5] << 8));
      int16_t yaw   = static_cast<int16_t>(data[6] | (data[7] << 8));

      // 每秒打印两次日志（500ms一次）
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "[0xB1] Target Speed -> X:%d, Y:%d, Z:%d, Yaw:%d",
        vel_x, vel_y, vel_z, yaw);

      break;
    }
    default: {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
        "Unhandled protocol ID: 0x%02X, len=%zu", id, data.size());
      break;
    }
  }
}

void UartToStm32::sendA2ReadyResponse()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send A2 ready response");
    return;
  }

  std::vector<uint8_t> data(9, 0x00);
  data[0] = 0x01;
  if (serial_comm_->send_protocol_data(A2_READY_RESP_ID, static_cast<uint8_t>(data.size()), data)) {
    RCLCPP_INFO(node_->get_logger(), "Sent A2 ready response (len=9, first=0x01)");
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to send A2 ready response: %s", serial_comm_->get_last_error().c_str());
  }
}

}  // namespace uart_to_stm32
