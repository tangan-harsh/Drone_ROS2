#ifndef UART_TO_STM32__UART_TO_STM32_HPP_
#define UART_TO_STM32__UART_TO_STM32_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <serial_comm/serial_comm.h>

namespace uart_to_stm32
{

/**
 * @class UartToStm32
 * @brief ROS 2 node that bridges serial communication between STM32 flight controller and ROS 2 system
 * 
 * This node subscribes to odometry and target velocity topics, then forwards the data
 * to the STM32 via serial port. It also receives sensor data (height, status) from STM32
 * and publishes them to ROS 2 topics.
 * 
 * Supported protocol frame IDs:
 * - 0x32: Velocity feedback (body frame, cm/s)
 * - 0x31: Target velocity command (cm/s, deg/s)
 * - 0x05: Height data (cm)
 * - 0xF1: ST ready query/response
 * - 0xA2: A2 ready response
 * - 0xB1: Target velocity feedback from flight controller
 */
class UartToStm32
{
public:
  /**
   * @brief Construct a new UartToStm32 object
   * @param node Shared pointer to ROS 2 node
   */
  explicit UartToStm32(rclcpp::Node::SharedPtr node);

  /**
   * @brief Destroy the UartToStm32 object
   * Stops serial communication and closes the port
   */
  ~UartToStm32();

  /**
   * @brief Initialize the UART to STM32 bridge
   * @details Performs the following steps:
   * 1. Load serial configuration from ROS parameters
   * 2. Initialize serial port
   * 3. Create subscribers for odometry and target velocity
   * 4. Create publishers for height, ST ready status, and mission step
   * 5. Start protocol receive thread
   * @return true if initialization successful, false otherwise
   */
  bool initialize();

private:
  /**
   * @brief Odometry callback function
   * @param msg Odometry message from DCL-SLAM
   * @details Extracts linear velocity and yaw angle, then sends to STM32
   */
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Target velocity callback function
   * @param msg Target velocity command from PID controller
   * @details Expects 4 float values: [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]
   */
  void targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  /**
   * @brief Send velocity data to STM32 via serial port
   * @param transformed_velocity Velocity in body frame (m/s)
   * @details Scales velocity by 100 and sends as int16 values (cm/s)
   */
  void sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity);

  /**
   * @brief Send target velocity command to STM32
   * @param vx_cm_per_s X-axis velocity command (cm/s)
   * @param vy_cm_per_s Y-axis velocity command (cm/s)
   * @param vz_cm_per_s Z-axis velocity command (cm/s)
   * @param vyaw_deg_per_s Yaw angular velocity command (deg/s)
   */
  void sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s);

  /**
   * @brief Send A2 ready response to STM32
   * @details Responds to ST ready query (0xF1) with ready status
   */
  void sendA2ReadyResponse();

  /**
   * @brief Protocol data handler callback
   * @param id Protocol frame ID
   * @param data Protocol frame data
   * @details Handles incoming serial data from STM32:
   * - 0xF1: ST ready query
   * - 0x05: Height data
   * - 0xB1: Target velocity feedback
   */
  void protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data);

  /// ROS 2 node shared pointer
  rclcpp::Node::SharedPtr node_;

  /// Odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  /// Target velocity subscriber
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_sub_;

  /// Serial communication interface
  std::unique_ptr<serial_comm::SerialComm> serial_comm_;

  /// Height publisher (from STM32 barometer/lidar)
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub_;

  /// ST ready status publisher (transient local)
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_st_ready_pub_;

  /// Mission step publisher
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mission_step_pub_;

  /// Flag indicating if ST ready status has been published
  bool has_st_ready_pub_;

  /// Serial port device path (configurable via ROS parameter)
  std::string serial_port_;

  /// Serial port baud rate (configurable via ROS parameter)
  int baud_rate_;

  /// Log throttle interval in milliseconds
  int log_throttle_interval_;

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
};

}  // namespace uart_to_stm32

#endif  // UART_TO_STM32__UART_TO_STM32_HPP_
