/**
 * @file route_target_publisher.cpp
 * @brief 航点目标发布节点实现
 * 
 * 功能概述：
 * 1. 航点管理：维护航点队列，按顺序执行
 * 2. 位姿获取：通过 TF 获取机器人当前位置和姿态
 * 3. 到达判断：基于位置、高度、偏航角容差判断是否到达
 * 4. 自动切换：到达后自动发布下一个航点
 * 5. 控制信号：发布控制器激活信号（无人机/小车模式）
 * 
 * 系统架构：
 * 
 *   ┌─────────────────────────────────────────────────────────┐
 *   │              RouteTargetPublisherNode                   │
 *   ├─────────────────────────────────────────────────────────┤
 *   │  输入：                                                  │
 *   │    - /height (订阅)          高度数据 (cm)                │
 *   │    - TF: map→laser_link      位置和偏航角                 │
 *   │                                                         │
 *   │  输出：                                                  │
 *   │    - /target_position (发布) 目标位置 [x,y,z,yaw]         │
 *   │    - /active_controller (发布) 控制器激活信号              │
 *   │                                                         │
 *   │  核心流程：                                               |
 *   │    getCurrentPose() → isReached() → publishTarget()     │ 
 *   │         ↓                ↓              ↓               │
 *   │       TF 查询          容差判断        发布话题             │
 *   └─────────────────────────────────────────────────────────┘
 * 
 * @author Your Name
 * @date 2024
 */

#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>
#include <array>
#include <clocale>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

// ==================== RouteTargetPublisherNode 实现 ====================

/**
 * @brief 构造函数
 * 
 * 初始化流程：
 * 1. 声明和加载 ROS 参数
 * 2. 初始化 TF 缓冲区和监听器
 * 3. 创建发布者和订阅者
 * 4. 打印初始化信息
 */
RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),  // 初始无航点
  has_height_(false),
  current_height_cm_(0.0)
{
  // 1. 加载控制参数
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  
  // 2. 加载坐标系配置
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");

  // 3. 初始化 TF 监听器
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 4. 创建发布者 (使用 transient_local QoS，保证新节点能收到最后一条消息)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/active_controller", qos);  // 对 STM32 发送应答帧的判断符
  
  // 5. 创建订阅者
  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));

  // 6. 打印初始化信息
  RCLCPP_INFO(
    get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s",
    map_frame_.c_str(), laser_link_frame_.c_str(), output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_, yaw_tol_deg_, height_tol_cm_);
}

/**
 * @brief 添加航点到队列
 * 
 * 线程安全：使用互斥锁保护
 * 
 * @param target 航点目标
 */
void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(target);
  
  // 如果是首个航点，立即开始执行
  if (was_empty) {
    current_idx_ = 0;
    publishCurrent();
  }
}

/**
 * @brief 获取当前航点索引
 * @return 当前航点索引，队列为空时返回 max()
 */
std::size_t RouteTargetPublisherNode::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

/**
 * @brief 获取航点队列大小
 * @return 航点总数
 */
std::size_t RouteTargetPublisherNode::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

/**
 * @brief 发布当前航点
 * 
 * 条件检查：
 * - 队列非空
 * - 索引有效
 */
void RouteTargetPublisherNode::publishCurrent()
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() &&
    current_idx_ < targets_.size())
  {
    publishTarget(targets_[current_idx_], current_idx_ == 0);
  }
}

/**
 * @brief 发布目标位置和控制信号
 * 
 * 发布内容：
 * 1. /target_position: Float32MultiArray[x_cm, y_cm, z_cm, yaw_deg]
 * 2. /active_controller: UInt8(2=无人机模式)
 * 
 * @param target 目标航点
 * @param init_flag 是否为首个航点
 */
void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag)
{
  // 1. 构建目标位置消息
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  target_pub_->publish(message);

  // 2. 发布控制器激活信号 (2=无人机模式)
  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;  // Drone
  active_controller_pub_->publish(active_msg);

  // 3. 打印日志
  RCLCPP_INFO(
    get_logger(),
    "发布目标：x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg%s",
    target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
    init_flag ? " (首个)" : "");
}

/**
 * @brief 高度数据回调
 * 
 * 来源：底层 STM32 通过串口发送
 * 用途：用于到达判断的 Z 轴误差计算
 * 
 * @param msg 高度消息 (cm)
 */
void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

/**
 * @brief 通过 TF 获取机器人位姿
 * 
 * 坐标系变换：map → laser_link
 * 
 * 数据来源：
 * - X, Y: TF 变换的平移分量
 * - Z: /height 话题（气压计/激光测距）
 * - Yaw: TF 变换的四元数转欧拉角
 * 
 * @param x_cm 输出：X 坐标 (cm)
 * @param y_cm 输出：Y 坐标 (cm)
 * @param z_cm 输出：高度 (cm)
 * @param yaw_deg 输出：偏航角 (deg)
 * @return true 成功，false 失败
 */
bool RouteTargetPublisherNode::getCurrentPose(
  double & x_cm, double & y_cm, double & z_cm, double & yaw_deg)
{
  try {
    // 1. 查询 TF 变换
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    
    // 2. 提取位置信息 (m → cm)
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    
    // 3. 使用高度计数据（更准确）
    z_cm = has_height_ ? current_height_cm_ : 0.0;
    
    // 4. 四元数转欧拉角
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    
    return true;
  } catch (const tf2::TransformException & ex) {
    // TF 查询失败（可能是坐标系未发布）
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF 查询失败 (%s->%s): %s",
      map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

/**
 * @brief 判断是否到达目标点
 * 
 * 判断条件（全部满足）：
 * 1. XY 平面距离误差 ≤ pos_tol_cm_ (默认 9cm)
 * 2. 高度误差 ≤ height_tol_cm_ (默认 12cm)
 * 3. 偏航角误差 ≤ yaw_tol_deg_ (默认 5°)
 * 
 * @param target 目标航点
 * @param x_cm 当前 X 坐标 (cm)
 * @param y_cm 当前 Y 坐标 (cm)
 * @param z_cm 当前高度 (cm)
 * @param yaw_deg 当前偏航角 (deg)
 * @return true 已到达，false 未到达
 */
bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  // 1. 计算各轴误差
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);  // XY 平面距离
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  // 2. 提取容差参数
  const double z_tol = height_tol_cm_;
  const double xy_tol = pos_tol_cm_;

  // 3. 判断是否满足所有条件
  const bool z_ok = (std::fabs(dz) <= z_tol);
  const bool xy_ok = (dxy <= xy_tol);
  const bool yaw_ok = (std::fabs(dyaw) <= yaw_tol_deg_);

  return z_ok && xy_ok && yaw_ok;
}

/**
 * @brief 监控定时器回调 (20Hz)
 * 
 * 执行流程：
 * 
 * ┌─────────────────────────────────────────────────────────────┐
 * │  1. 检查是否完成所有航点                                     │
 *     ↓ 是 → 发送停止信号 (3)，持续广播                           │
 *     ↓ 否                                                        │
 *  ┌───────────────────────────────────────────────────────────┐│
 *  │  2. 检查是否有有效航点                                      ││
 *     ↓ 否 → 返回                                               ││
 *     ↓ 是                                                      ││
 *  ┌───────────────────────────────────────────────────────────┐│
 *  │  3. 获取当前位姿                                            ││
 *     ↓ 失败 → 返回                                             ││
 *     ↓ 成功                                                    ││
 *  ┌───────────────────────────────────────────────────────────┐│
 *  │  4. 调用 isReached() 判断                                   ││
 *     ↓ 未到达 → 返回                                           ││
 *     ↓ 已到达                                                  ││
 *  ┌───────────────────────────────────────────────────────────┐│
 *  │  5. 切换到下一个航点                                        ││
 *     - current_idx_++                                          ││
 *     - 如果有下一个 → publishCurrent()                         ││
 *     - 如果没有 → 发送停止信号 (3)                             ││
 *     └───────────────────────────────────────────────────────────┘│
 *     └─────────────────────────────────────────────────────────────┘
 */
void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 1. 检查是否已经完成所有目标
  if (current_idx_ != std::numeric_limits<std::size_t>::max() &&
    current_idx_ >= targets_.size())
  {
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;  // Drone Stop
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "所有目标已完成，持续发送停止信号 (3)");
    return;
  }

  // 2. 检查是否有有效航点
  if (current_idx_ == std::numeric_limits<std::size_t>::max()) {
    return;
  }

  // 3. 获取当前位姿
  double x_cm = 0.0, y_cm = 0.0, z_cm = 0.0, yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  // 4. 获取当前目标并打印状态
  const Target & target = targets_[current_idx_];
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "当前目标 %zu: x=%.1f,y=%.1f,z=%.1f,yaw=%.1f",
    current_idx_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg);

  // 5. 目标点判断逻辑
  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    current_idx_++;
    if (current_idx_ < targets_.size()) {
      publishCurrent();  // 发布新目标
    } else {
      current_idx_ = targets_.size();
      RCLCPP_INFO(get_logger(), "所有目标已完成");
      std_msgs::msg::UInt8 active_msg;
      active_msg.data = 3;  // Drone Stop
      active_controller_pub_->publish(active_msg);
    }
  }
}

/**
 * @brief 米转厘米
 * @param value_m 米单位的值
 * @return 厘米单位的值
 */
double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

/**
 * @brief 弧度转角度
 * @param value_rad 弧度值
 * @return 角度值
 */
double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

/**
 * @brief 归一化角度到 [-180°, 180°]
 * 
 * 用途：避免 179° → -179° 的跳变导致误差计算错误
 * 
 * 示例：
 * - 185° → -175°
 * - -190° → 170°
 * 
 * @param angle_deg 输入角度 (度)
 * @return 归一化后的角度
 */
double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}

// ==================== RouteTestNode 实现 ====================

/**
 * @brief 构造函数
 * 
 * 初始化流程：
 * 1. 创建定时器（1Hz）
 * 2. 添加首个航点（起飞：0,0,130cm,0°）
 * 3. 启动定时器
 * 
 * @param route_node 航点发布节点指针
 * @param options ROS2 节点选项
 */
RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node),
  started_(false),
  next_target_index_(1)
{
  std::setlocale(LC_ALL, "");
  
  // 1. 创建定时器（每秒添加一个航点）
  add_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&RouteTestNode::addTimerCallback, this));
  add_timer_->cancel();  // 先暂停，添加首个航点后再启动

  RCLCPP_INFO(get_logger(), "Route test node 启动，自动添加第一个航点...");

  // 2. 添加首个航点（起飞）
  Target first{0.0, 0.0, 130.0, 0.0};
  route_node_->addTarget(first);

  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(
    get_logger(),
    "添加首个目标：x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
    first.x_cm, first.y_cm, first.z_cm, first.yaw_deg,
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

  // 3. 启动定时器
  add_timer_->reset();
  started_ = true;
}

/**
 * @brief 添加预设航点定时器回调
 * 
 * 预设航点序列（共 19 个）：
 * 
 * | 序号 | X(cm)  | Y(cm)  | Z(cm) | Yaw(°) | 动作说明      |
 * |------|--------|--------|-------|--------|---------------|
 * | 1    | 0      | 0      | 130   | 0      | 起飞          |
 * | 2    | 103    | -145   | 130   | 0      | 水平飞行      |
 * | 3    | 153    | -145   | 130   | 0      | 水平飞行      |
 * | 4    | 203    | -145   | 130   | 0      | 水平飞行      |
 * | 5    | 203    | -145   | 83    | 0      | 下降          |
 * | 6    | 153    | -145   | 83    | 0      | 水平飞行      |
 * | 7    | 103    | -145   | 83    | 0      | 水平飞行      |
 * | 8    | -30    | -145   | 83    | 0      | 长距离飞行    |
 * | 9    | -30    | -260   | 83    | 0      | 水平飞行      |
 * | 10   | -30    | -260   | 83    | 90     | 原地旋转 90°  |
 * | 11   | -30    | -260   | 83    | 180    | 原地旋转 180° |
 * | 12   | 103    | -260   | 83    | 180    | 水平飞行      |
 * | 13   | 153    | -260   | 83    | 180    | 水平飞行      |
 * | 14   | 203    | -260   | 83    | 180    | 水平飞行      |
 * | 15   | 203    | -260   | 130   | 180    | 上升          |
 * | 16   | 153    | -260   | 130   | 180    | 水平飞行      |
 * | 17   | 103    | -260   | 130   | 180    | 水平飞行      |
 * | 18   | 280    | -380   | 130   | 180    | 长距离飞行    |
 * | 19   | 280    | -380   | 4     | 180    | 降落          |
 * 
 * 测试覆盖：
 * ✅ 起飞/降落
 * ✅ 水平飞行（X/Y轴）
 * ✅ 高度变化（上升/下降）
 * ✅ 偏航旋转（90°/180°）
 * ✅ 长距离飞行
 */
void RouteTestNode::addTimerCallback()
{
  if (!started_) {
    return;
  }

  // 预定义航点数组
  static constexpr std::array<Target, 19> kWaypoints = {{
    {0.0, 0.0, 130.0, 0.0},           // 1: 起飞
    {103.0, -145.0, 130.0, 0.0},      // 2
    {153.0, -145.0, 130.0, 0.0},      // 3
    {203.0, -145.0, 130.0, 0.0},      // 4
    {203.0, -145.0, 83.0, 0.0},       // 5: 下降
    {153.0, -145.0, 83.0, 0.0},       // 6
    {103.0, -145.0, 83.0, 0.0},       // 7
    {-30.0, -145.0, 83.0, 0.0},       // 8
    {-30.0, -260.0, 83.0, 0.0},       // 9
    {-30.0, -260.0, 83.0, 90.0},      // 10: 旋转 90
    {-30.0, -260.0, 83.0, 180.0},     // 11: 旋转 180
    {103.0, -260.0, 83.0, 180.0},     // 12
    {153.0, -260.0, 83.0, 180.0},     // 13
    {203.0, -260.0, 83.0, 180.0},     // 14
    {203.0, -260.0, 130.0, 180.0},    // 15: 上升
    {153.0, -260.0, 130.0, 180.0},    // 16
    {103.0, -260.0, 130.0, 180.0},    // 17
    {280.0, -380.0, 130.0, 180.0},    // 18
    {280.0, -380.0, 4.0, 180.0},      // 19: 降落
  }};

  // 按顺序添加航点
  if (next_target_index_ <= static_cast<int>(kWaypoints.size())) {
    const Target & target = kWaypoints[next_target_index_ - 1];
    route_node_->addTarget(target);

    const auto current = route_node_->currentIndex();
    RCLCPP_INFO(
      get_logger(),
      "追加目标 idx=%d: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
      next_target_index_,
      target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
      (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

    ++next_target_index_;
  } else {
    // 所有航点添加完毕
    add_timer_->cancel();
    RCLCPP_INFO(get_logger(), "预设目标全部添加完毕");
  }
}

}  // namespace activity_control_pkg
