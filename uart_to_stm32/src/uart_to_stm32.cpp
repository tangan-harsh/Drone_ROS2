#include "uart_to_stm32/uart_to_stm32.hpp"

#include <chrono>
#include <cmath>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace uart_to_stm32
{

using namespace std::chrono_literals;

UartToStm32::UartToStm32(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  has_st_ready_pub_(false),
  serial_port_(DEFAULT_SERIAL_PORT),
  baud_rate_(DEFAULT_BAUD_RATE),
  log_throttle_interval_(DEFAULT_LOG_THROTTLE_INTERVAL)
{
  RCLCPP_INFO(node_->get_logger(), "UartToStm32 created");
}

UartToStm32::~UartToStm32()
{
  if (serial_comm_) {
    serial_comm_->stop_protocol_receive();
    serial_comm_->close();
  }
}

bool UartToStm32::initialize()
{
  try {
    serial_port_ = node_->declare_parameter<std::string>("serial_port", DEFAULT_SERIAL_PORT);
    baud_rate_ = node_->declare_parameter<int>("baud_rate", DEFAULT_BAUD_RATE);
    log_throttle_interval_ = node_->declare_parameter<int>("log_throttle_interval", DEFAULT_LOG_THROTTLE_INTERVAL);

    RCLCPP_INFO(node_->get_logger(), "Serial configuration: port=%s, baud_rate=%d",
                serial_port_.c_str(), baud_rate_);

    serial_comm_ = std::make_unique<serial_comm::SerialComm>();
    if (!serial_comm_->initialize(serial_port_, baud_rate_)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize serial port %s at %d baudrate",
                   serial_port_.c_str(), baud_rate_);
      RCLCPP_ERROR(node_->get_logger(), "Serial error: %s", serial_comm_->get_last_error().c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Serial port %s initialized at %d baudrate",
                serial_port_.c_str(), baud_rate_);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/a/Odometry", 10,
      std::bind(&UartToStm32::odometryCallback, this, std::placeholders::_1));

    target_velocity_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_velocity", 10,
      std::bind(&UartToStm32::targetVelocityCallback, this, std::placeholders::_1));

    height_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/height", 10);
    is_st_ready_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
      "/is_st_ready", rclcpp::QoS(10).transient_local());
    mission_step_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/mission_step", 10);

    serial_comm_->start_protocol_receive(
      [this](uint8_t id, const std::vector<uint8_t> & data) { protocolDataHandler(id, data); },
      [this](const std::string & err) {
        RCLCPP_WARN(node_->get_logger(), "Serial protocol error: %s", err.c_str());
      });

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized successfully");
    RCLCPP_INFO(node_->get_logger(), "Subscribed to /a/Odometry and /target_velocity topics");
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize: %s", e.what());
    return false;
  }
}

/**
 * @brief Odometry callback - receives odometry data from DCL-SLAM
 * @param msg Odometry message containing pose and velocity
 * 
 * Extracts linear velocity and yaw angle from odometry message,
 * then sends velocity data to STM32 via serial port.
 */
void UartToStm32::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  const double linear_x = msg->twist.twist.linear.x;
  const double linear_y = msg->twist.twist.linear.y;
  const double linear_z = msg->twist.twist.linear.z;
  const double angular_x = msg->twist.twist.angular.x;
  const double angular_y = msg->twist.twist.angular.y;
  const double angular_z = msg->twist.twist.angular.z;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
    "Odometry: linear(%.3f, %.3f, %.3f) angular(%.3f, %.3f, %.3f) yaw=%.2f deg",
    linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, yaw * 180.0 / M_PI);

    Eigen::Vector3d body_vel(linear_x, linear_y, linear_z);
    sendVelocityToSerial(body_vel);
}

/**
 * @brief Send velocity feedback to STM32 via serial port (Frame ID: 0x32)
 * @param transformed_velocity Velocity in body frame (m/s)
 * 
 * Scales velocity by 100 and sends as int16 values in cm/s.
 * This is velocity feedback for the flight controller, not a control command.
 */
void UartToStm32::sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
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
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
        "Sent velocity data: x=%d, y=%d, z=%d (cm/s)", vel_x, vel_y, vel_z);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
        "Failed to send velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendVelocityToSerial: %s", e.what());
  }
}

/**
 * @brief Target velocity callback - receives velocity commands from PID controller
 * @param msg Target velocity command [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]
 * 
 * This is the main control command entry point. Extracts velocity components
 * and forwards them to STM32 via serial port.
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
    node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
    "Target Velocity: linear(%.1f, %.1f, %.1f)cm/s angular(%.1f)deg/s",
    vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);

  sendTargetVelocityToSerial(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);
}

/**
 * @brief Send target velocity command to STM32 (Frame ID: 0x31)
 * @param vx_cm_per_s X-axis velocity command (cm/s)
 * @param vy_cm_per_s Y-axis velocity command (cm/s)
 * @param vz_cm_per_s Z-axis velocity command (cm/s)
 * @param vyaw_deg_per_s Yaw angular velocity command (deg/s)
 * 
 * Converts float velocity commands to int16 and sends to STM32.
 * Note: This function sends the values directly without any coordinate transformation.
 * Coordinate transformation (Map -> Body) should be done by the PID controller before publishing.
 */
void UartToStm32::sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
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
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
        "Sent target velocity data: x=%d, y=%d, z=%d, yaw=%d",
        vel_x, vel_y, vel_z, vel_yaw);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
        "Failed to send target velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendTargetVelocityToSerial: %s", e.what());
  }
}

/**
 * @brief Protocol data handler - parses incoming data from STM32
 * @param id Protocol frame ID
 * @param data Protocol frame data
 * 
 * Handles the following protocol frames:
 * - 0xF1 (ST_READY_QUERY_ID): ST ready query/response
 * - 0x05 (HEIGHT_FRAME_ID): Height data from barometer/lidar
 * - 0xB1: Target velocity feedback from flight controller
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
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
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
        sendA2ReadyResponse();
        has_st_ready_pub_ = true;
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "0xF1 frame second byte != 1 (%u), ignoring", static_cast<unsigned>(second));
      }
      break;
    }
    case HEIGHT_FRAME_ID: {
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
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
          "Published /height: %d", value);
      } else {
        RCLCPP_WARN(node_->get_logger(), "Height publisher not initialized");
      }
      break;
    }
    default: {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
        "Unhandled protocol ID: 0x%02X, len=%zu", id, data.size());
      break;
    }
  }
}

/**
 * @brief Send A2 ready response to STM32 (Frame ID: 0xA2)
 * 
 * Responds to ST ready query (0xF1) when STM32 is ready.
 * Sends a 9-byte response with first byte set to 0x01.
 */
void UartToStm32::sendA2ReadyResponse()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), log_throttle_interval_,
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
