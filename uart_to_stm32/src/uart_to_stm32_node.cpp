/**
 * @file uart_to_stm32_node.cpp
 * @brief UART 到 STM32 桥接器的 ROS 2 节点入口
 * 
 * 该节点作为 ROS 2 和 STM32 飞控之间的通信桥接器。
 * 它订阅里程计和目标速度话题，然后通过串行端口将数据转发到 STM32。
 * 它还从 STM32 接收传感器数据并发布到 ROS 2 话题。
 * 
 * 支持的协议帧 ID：
 * - 0x32: 速度反馈（机体坐标系，cm/s）
 * - 0x31: 目标速度指令（cm/s, deg/s）
 * - 0x05: 高度数据（cm）
 * - 0xF1: ST 就绪查询/响应
 * - 0xA2: A2 就绪响应
 * - 0xB1: 飞控的目标速度反馈
 */

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "uart_to_stm32/uart_to_stm32.hpp"

/**
 * @brief UART 到 STM32 节点的主入口点
 * @param argc 参数个数
 * @param argv 参数向量
 * @return 成功返回 EXIT_SUCCESS，失败返回 EXIT_FAILURE
 * 
 * 初始化 ROS 2，创建 UartToStm32 实例，并启动事件循环。
 * 优雅地处理初始化失败和异常。
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
