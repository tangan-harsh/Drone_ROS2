#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

/**
 * @struct Target
 * @brief 航点目标结构体
 * 
 * 用于存储无人机的飞行航点信息，所有单位均为厘米 (cm) 或度 (deg)
 */
struct Target
{
  double x_cm;      ///< X 轴坐标 (cm)，地图坐标系
  double y_cm;      ///< Y 轴坐标 (cm)，地图坐标系
  double z_cm;      ///< 高度 (cm)，相对于起飞点
  double yaw_deg;   ///< 偏航角 (deg)，逆时针为正
};

/**
 * @class RouteTargetPublisherNode
 * @brief 航点目标发布节点
 * 
 * 核心功能：
 * 1. 管理航点队列，按顺序发布目标位置
 * 2. 通过 TF 获取机器人当前位置和姿态
 * 3. 判断机器人是否到达当前航点
 * 4. 到达后自动切换到下一个航点
 * 5. 发布控制器激活信号（无人机/小车模式切换）
 * 
 * 数据流：
 *   航点队列 → getCurrentPose() → isReached() → publishTarget()
 *                                    ↓
 *                              /target_position (发布)
 *                              /active_controller (发布)
 *                              /height (订阅)
 */
class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   * @param options ROS2 节点选项
   * 
   * 初始化内容：
   * - 加载参数：容差、坐标系名称、话题名称
   * - 创建发布者：/target_position, /active_controller
   * - 创建订阅者：/height
   * - 初始化 TF 监听器
   */
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief 添加航点到队列
   * @param target 航点目标
   * 
   * 如果是首个航点，会立即发布并开始执行
   */
  void addTarget(const Target & target);

  /**
   * @brief 获取当前正在执行的航点索引
   * @return 当前航点索引，如果队列为空返回 max()
   */
  std::size_t currentIndex() const;

  /**
   * @brief 获取航点队列大小
   * @return 航点总数
   */
  std::size_t size() const;

  /**
   * @brief 加载预定义航点序列
   * @param preset_name 预设名称 ("test_19", "simple_5", "none")
   * @return true 加载成功，false 未知预设名称
   * 
   * 支持的预设：
   * - "test_19": 19 个测试航点（起飞、水平飞行、旋转、降落）
   * - "simple_5": 5 个简单航点（快速测试用）
   * - "none": 不加载任何预设航点
   */
  bool loadPresetWaypoints(const std::string & preset_name);

 private:
  /**
   * @brief 发布当前航点
   * 
   * 调用 publishTarget() 发布 currentIndex() 对应的航点
   */
  void publishCurrent();

  /**
   * @brief 发布目标位置和控制器信号
   * @param target 目标航点
   * @param init_flag 是否为首个航点标志
   * 
   * 发布内容：
   * - /target_position: [x_cm, y_cm, z_cm, yaw_deg]
   * - /active_controller: 2 (无人机模式)
   */
  void publishTarget(const Target & target, bool init_flag);

  /**
   * @brief 通过 TF 获取机器人当前位置和姿态
   * @param x_cm 输出：X 坐标 (cm)
   * @param y_cm 输出：Y 坐标 (cm)
   * @param z_cm 输出：高度 (cm)，来自 /height 话题
   * @param yaw_deg 输出：偏航角 (deg)
   * @return true 获取成功，false 失败
   * 
   * 坐标系变换：map → laser_link
   * 注意：Z 轴使用高度计数据，XY 使用 TF，Yaw 使用 TF
   */
  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);

  /**
   * @brief 判断机器人是否到达目标点
   * @param target 目标航点
   * @param x_cm 当前 X 坐标 (cm)
   * @param y_cm 当前 Y 坐标 (cm)
   * @param z_cm 当前高度 (cm)
   * @param yaw_deg 当前偏航角 (deg)
   * @return true 已到达，false 未到达
   * 
   * 判断条件（全部满足）：
   * - XY 距离误差 ≤ position_tolerance_cm (默认 9cm)
   * - 高度误差 ≤ height_tolerance_cm (默认 12cm)
   * - 偏航角误差 ≤ yaw_tolerance_deg (默认 5°)
   */
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;

  /**
   * @brief 定时器回调：监控航点执行状态
   * 
   * 执行流程：
   * 1. 检查是否完成所有航点 → 发送停止信号
   * 2. 获取当前位姿
   * 3. 判断是否到达当前航点
   * 4. 已到达 → 切换到下一个航点并发布
   * 
   * 频率：20Hz (0.05s)
   */
  void monitorTimerCallback();

  /**
   * @brief 高度数据回调
   * @param msg 高度消息 (cm)
   * 
   * 来源：底层 STM32 通过串口发送的气压计/激光测距数据
   */
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  
  /**
   * @brief 米转厘米
   * @param value_m 米单位的值
   * @return 厘米单位的值
   */
  static double meterToCm(double value_m);

  /**
   * @brief 弧度转角度
   * @param value_rad 弧度值
   * @return 角度值
   */
  static double radToDeg(double value_rad);

  /**
   * @brief 归一化角度到 [-180°, 180°]
   * @param angle_deg 输入角度 (度)
   * @return 归一化后的角度
   * 
   * 用途：计算偏航角误差时避免 179°→-179° 跳变
   */
  double normalizeAngleDeg(double angle_deg) const;

  // ==================== ROS 接口 ====================
  
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;        ///< 发布目标位置 [x,y,z,yaw]
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_controller_pub_;         ///< 发布控制器激活信号 (2=无人机，3=停止)
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;                 ///< 订阅高度数据 (cm)

  rclcpp::TimerBase::SharedPtr monitor_timer_;                                       ///< 航点监控定时器 (20Hz)

  // ==================== TF 变换 ====================
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                       ///< TF 缓冲区
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                          ///< TF 监听器

  // ==================== 状态管理 ====================
  
  mutable std::mutex mutex_;                                                         ///< 保护航点队列的互斥锁
  std::vector<Target> targets_;                                                      ///< 航点队列
  std::size_t current_idx_;                                                          ///< 当前航点索引

  // ==================== 传感器数据 ====================
  
  bool has_height_;                                                                  ///< 是否收到高度数据
  double current_height_cm_;                                                         ///< 当前高度 (cm)

  // ==================== 控制参数 ====================
  
  double pos_tol_cm_;                                                                ///< 位置容差 (cm)，默认 9.0
  double yaw_tol_deg_;                                                               ///< 偏航角容差 (deg)，默认 5.0
  double height_tol_cm_;                                                             ///< 高度容差 (cm)，默认 12.0

  // ==================== 坐标系配置 ====================
  
  std::string map_frame_;                                                            ///< 地图坐标系名称，默认 "map"
  std::string laser_link_frame_;                                                     ///< 激光雷达坐标系名称，默认 "laser_link"
  std::string output_topic_;                                                         ///< 目标位置话题名称，默认 "/target_position"
};

/**
 * @class RouteTestNode
 * @brief 航点测试节点
 * 
 * 用途：自动生成测试航点序列，验证航点执行功能
 * 
 * 功能：
 * 1. 启动时自动添加首个航点（起飞）
 * 2. 每秒添加一个预设航点
 * 3. 共 19 个航点，包含：
 *    - 起飞/降落
 *    - 水平飞行
 *    - 高度变化
 *    - 偏航旋转
 * 
 * 测试流程：
 *   启动 → 添加航点 1(起飞) → 每秒添加下一个 → 添加完毕 (19 个)
 */
class RouteTestNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   * @param route_node 航点发布节点指针
   * @param options ROS2 节点选项
   */
  explicit RouteTestNode(
    const std::shared_ptr<RouteTargetPublisherNode> & route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief 定时器回调：添加预设航点
   * 
   * 从预定义的航点数组中按顺序添加，共 19 个航点
   */
  void addTimerCallback();

  std::shared_ptr<RouteTargetPublisherNode> route_node_;                             ///< 航点发布节点指针
  rclcpp::TimerBase::SharedPtr add_timer_;                                           ///< 添加航点定时器 (1Hz)

  bool started_;                                                                     ///< 是否已启动
  int next_target_index_;                                                            ///< 下一个要添加的航点索引
};

}  // namespace activity_control_pkg
