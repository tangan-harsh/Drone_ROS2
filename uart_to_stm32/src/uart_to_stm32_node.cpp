/**
 * @file uart_to_stm32_node.cpp
 * @brief ROS 2 node entry point for UART to STM32 bridge
 * 
 * This node serves as a communication bridge between ROS 2 and STM32 flight controller.
 * It subscribes to odometry and target velocity topics, then forwards the data to STM32
 * via serial port. It also receives sensor data from STM32 and publishes to ROS 2 topics.
 * 
 * Supported protocol frame IDs:
 * - 0x32: Velocity feedback (body frame, cm/s)
 * - 0x31: Target velocity command (cm/s, deg/s)
 * - 0x05: Height data (cm)
 * - 0xF1: ST ready query/response
 * - 0xA2: A2 ready response
 * - 0xB1: Target velocity feedback from flight controller
 */

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "uart_to_stm32/uart_to_stm32.hpp"

/**
 * @brief Main entry point for the UART to STM32 node
 * @param argc Argument count
 * @param argv Argument vector
 * @return EXIT_SUCCESS on success, EXIT_FAILURE on failure
 * 
 * Initializes ROS 2, creates the UartToStm32 instance, and starts the event loop.
 * Handles initialization failures and exceptions gracefully.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("uart_to_stm32_node");

  try {
    auto app = std::make_shared<uart_to_stm32::UartToStm32>(node);
    if (!app->initialize()) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize UartToStm32");
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }

    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Exception in main: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
