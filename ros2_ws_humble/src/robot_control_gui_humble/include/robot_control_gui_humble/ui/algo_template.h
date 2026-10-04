// SPDX-License-Identifier: MIT
//
// algo_template.h —— 自定义 Nav2 插件的模板(规划器 / PID 控制器)。
// 两个模板都已在本项目容器里 colcon build 验证过,能直接编译。
// 规划器 = 直线插值示例 + 一个自定义参数(my_gain);
// 控制器 = PID 路径跟踪(kp/ki/kd 等参数可在 GUI 里查/改)。
#pragma once
#include <string>
#include <vector>

namespace rcj {

struct AlgoTemplateFile { const char* path; const char* content; };

inline const std::vector<AlgoTemplateFile>& kPlannerTemplate() {
  static const std::vector<AlgoTemplateFile> v = {
    { R"ALGO(package.xml)ALGO",
      R"ALGO(<?xml version="1.0"?>
<package format="3">
  <name>my_planner</name>
  <version>0.0.1</version>
  <description>自定义 Nav2 全局规划器插件</description>
  <maintainer email="me@example.com">me</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>nav2_core</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>pluginlib</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
)ALGO" },
    { R"ALGO(CMakeLists.txt)ALGO",
      R"ALGO(cmake_minimum_required(VERSION 3.16)
project(my_planner)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
add_library(${PROJECT_NAME} SHARED src/my_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav_msgs geometry_msgs pluginlib)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav_msgs geometry_msgs pluginlib)
ament_package()
)ALGO" },
    { R"ALGO(plugins.xml)ALGO",
      R"ALGO(<library path="my_planner">
  <class name="my_planner/MyPlanner" type="my_planner::MyPlanner"
         base_class_type="nav2_core::GlobalPlanner">
    <description>自定义全局规划器</description>
  </class>
</library>
)ALGO" },
    { R"ALGO(include/@NAME@/@NAME@.hpp)ALGO",
      R"ALGO(#pragma once
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace my_planner {

class MyPlanner : public nav2_core::GlobalPlanner {
public:
  MyPlanner() = default;
  ~MyPlanner() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // ★ 你的算法写在这里:给起点和终点,返回一条 nav_msgs/Path
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                                 const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_{nullptr};
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  double my_gain_{1.0};   // 自定义参数示例,可在 GUI 里改
};

}  // namespace my_planner
)ALGO" },
    { R"ALGO(src/@NAME@.cpp)ALGO",
      R"ALGO(#include "my_planner/my_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace my_planner {

void MyPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                          std::string name, std::shared_ptr<tf2_ros::Buffer>,
                          std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  // 声明自己的参数:这样它会出现在 GUI 的「🔧 算法参数」里,并能在线修改。
  // 参数全名会变成 GridBased.my_gain(前缀 = 插件实例名)。
  const std::string full = name_ + ".my_gain";
  if (!node_->has_parameter(full)) node_->declare_parameter(full, my_gain_);
  node_->get_parameter(full, my_gain_);

  RCLCPP_INFO(node_->get_logger(), "MyPlanner '%s' 已配置 (my_gain=%.2f)", name_.c_str(), my_gain_);
}

void MyPlanner::cleanup()   { RCLCPP_INFO(node_->get_logger(), "MyPlanner cleanup"); }
void MyPlanner::activate()  { RCLCPP_INFO(node_->get_logger(), "MyPlanner activate"); }
void MyPlanner::deactivate(){ RCLCPP_INFO(node_->get_logger(), "MyPlanner deactivate"); }

nav_msgs::msg::Path MyPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start,
                                          const geometry_msgs::msg::PoseStamped & goal) {
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  path.header.stamp = node_->now();

  // ===== 示例:直线插值(把它换成你的算法: A* / Dijkstra / RRT ...)=====
  // 可用:costmap_->getSizeInCellsX()/Y(), costmap_->getCost(mx,my),
  //       costmap_->worldToMap(wx,wy,mx,my)
  const int N = std::max(2, static_cast<int>(20 * my_gain_));   // my_gain 影响点数
  for (int i = 0; i <= N; ++i) {
    const double t = static_cast<double>(i) / N;
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    // 用一下自定义参数(示例):my_gain 会整体缩放插值密度
    p.pose.position.x = start.pose.position.x + t * (goal.pose.position.x - start.pose.position.x);
    p.pose.position.y = start.pose.position.y + t * (goal.pose.position.y - start.pose.position.y);
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  return path;
}

}  // namespace my_planner

PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav2_core::GlobalPlanner)
)ALGO" },
  };
  return v;
}

inline const std::vector<AlgoTemplateFile>& kControllerTemplate() {
  static const std::vector<AlgoTemplateFile> v = {
    { R"ALGO(package.xml)ALGO",
      R"ALGO(<?xml version="1.0"?>
<package format="3">
  <name>my_pid_controller</name>
  <version>0.0.1</version>
  <description>自定义 Nav2 路径跟踪控制器插件(PID)</description>
  <maintainer email="me@example.com">me</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend><depend>rclcpp_lifecycle</depend>
  <depend>nav2_core</depend><depend>nav2_costmap_2d</depend>
  <depend>nav_msgs</depend><depend>geometry_msgs</depend><depend>pluginlib</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
)ALGO" },
    { R"ALGO(CMakeLists.txt)ALGO",
      R"ALGO(cmake_minimum_required(VERSION 3.16)
project(my_pid_controller)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
add_library(${PROJECT_NAME} SHARED src/my_pid_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav_msgs geometry_msgs pluginlib)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav_msgs geometry_msgs pluginlib)
ament_package()
)ALGO" },
    { R"ALGO(plugins.xml)ALGO",
      R"ALGO(<library path="my_pid_controller">
  <class name="my_pid_controller/MyPidController" type="my_pid_controller::MyPidController"
         base_class_type="nav2_core::Controller">
    <description>PID 路径跟踪控制器</description>
  </class>
</library>
)ALGO" },
    { R"ALGO(include/@NAME@/@NAME@.hpp)ALGO",
      R"ALGO(#pragma once
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace my_pid_controller {

// PID 路径跟踪控制器:
//   横向:朝"前瞻点"的航向误差 -> 角速度 PID
//   纵向:航向偏差越大越减速 -> 线速度
// 所有 PID 参数都能在 GUI「🔧 算法参数」里查/改(FollowPath.*)。
class MyPidController : public nav2_core::Controller {
public:
  MyPidController() = default;
  ~MyPidController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // ---- PID 参数(都能在 GUI 里查/改)----
  double kp_linear_{0.6}, ki_linear_{0.0}, kd_linear_{0.05};
  double kp_angular_{1.2}, ki_angular_{0.0}, kd_angular_{0.15};
  double max_linear_{0.25};      // m/s
  double max_angular_{0.8};      // rad/s
  double lookahead_dist_{0.45};  // m,前瞻距离
  double goal_tolerance_{0.15};  // m,到这个距离就停

  // ---- PID 状态 ----
  double e_ang_int_{0.0}, e_ang_prev_{0.0};
  double e_lin_int_{0.0}, e_lin_prev_{0.0};

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_{nullptr};
  nav_msgs::msg::Path global_plan_;
  std::string name_;
  rclcpp::Time last_time_;
};

}  // namespace my_pid_controller
)ALGO" },
    { R"ALGO(src/@NAME@.cpp)ALGO",
      R"ALGO(#include "my_pid_controller/my_pid_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace my_pid_controller {

void MyPidController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                                std::string name, std::shared_ptr<tf2_ros::Buffer>,
                                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;

  // 声明参数:这样它才会出现在 GUI 的「🔧 算法参数」里,并能在线修改。
  // 名字会变成 FollowPath.kp_angular 这种(前缀 = 插件实例名)。
  auto d = [&](const std::string & k, double def) {
    const std::string full = name_ + "." + k;
    if (!node_->has_parameter(full)) node_->declare_parameter(full, def);
    double v = def;
    node_->get_parameter(full, v);
    return v;
  };
  kp_linear_    = d("kp_linear",    kp_linear_);
  ki_linear_    = d("ki_linear",    ki_linear_);
  kd_linear_    = d("kd_linear",    kd_linear_);
  kp_angular_   = d("kp_angular",   kp_angular_);
  ki_angular_   = d("ki_angular",   ki_angular_);
  kd_angular_   = d("kd_angular",   kd_angular_);
  max_linear_   = d("max_linear",   max_linear_);
  max_angular_  = d("max_angular",  max_angular_);
  lookahead_dist_ = d("lookahead_dist", lookahead_dist_);
  goal_tolerance_ = d("goal_tolerance", goal_tolerance_);

  RCLCPP_INFO(node_->get_logger(),
              "MyPidController '%s' 已配置 (kp_ang=%.2f ki_ang=%.2f kd_ang=%.2f)",
              name_.c_str(), kp_angular_, ki_angular_, kd_angular_);
}

void MyPidController::cleanup()    { RCLCPP_INFO(node_->get_logger(), "cleanup"); }
void MyPidController::activate()   { RCLCPP_INFO(node_->get_logger(), "activate"); }
void MyPidController::deactivate() { RCLCPP_INFO(node_->get_logger(), "deactivate"); }
void MyPidController::setPlan(const nav_msgs::msg::Path & path) {
  global_plan_ = path;
  e_ang_int_ = e_lin_int_ = 0.0;      // 新路径 -> 清积分,避免旧积分捣乱
  e_ang_prev_ = e_lin_prev_ = 0.0;
}
void MyPidController::setSpeedLimit(const double &, const bool &) {}

geometry_msgs::msg::TwistStamped MyPidController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker * /*goal_checker*/) {
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = node_->now();
  if (global_plan_.poses.empty()) return cmd;      // 没路径 -> 停

  // 1) 找前瞻点:路径上第一个离车 >= lookahead_dist 的点(找不到就用终点)
  const auto & p0 = pose.pose.position;
  geometry_msgs::msg::PoseStamped target = global_plan_.poses.back();
  for (const auto & ps : global_plan_.poses) {
    const double dx = ps.pose.position.x - p0.x, dy = ps.pose.position.y - p0.y;
    if (std::hypot(dx, dy) >= lookahead_dist_) { target = ps; break; }
  }

  // 2) 两个误差
  const double dx = target.pose.position.x - p0.x;
  const double dy = target.pose.position.y - p0.y;
  const auto & q = pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  double e_ang = std::atan2(dy, dx) - yaw;
  while (e_ang >  M_PI) e_ang -= 2.0 * M_PI;       // 归一化到 [-pi, pi]
  while (e_ang < -M_PI) e_ang += 2.0 * M_PI;
  const double dist_goal = std::hypot(global_plan_.poses.back().pose.position.x - p0.x,
                                      global_plan_.poses.back().pose.position.y - p0.y);
  const double e_lin = dist_goal;                  // 纵向误差 = 离终点距离

  // 3) dt(两次调用的时间差,用来做积分/微分)
  const rclcpp::Time now = node_->now();
  double dt = (last_time_.nanoseconds() == 0) ? 0.1 : (now - last_time_).seconds();
  if (dt <= 0.0 || dt > 1.0) dt = 0.1;
  last_time_ = now;

  // 4) 角速度 PID(占位里的 PID 就是这三行,改这里换你的控制律)
  e_ang_int_ += e_ang * dt;
  const double e_ang_der = (e_ang - e_ang_prev_) / dt;
  e_ang_prev_ = e_ang;
  double omega = kp_angular_ * e_ang + ki_angular_ * e_ang_int_ + kd_angular_ * e_ang_der;

  // 5) 线速度 PID(用"离终点距离"当误差;转得急就减速)
  e_lin_int_ += e_lin * dt;
  const double e_lin_der = (e_lin - e_lin_prev_) / dt;
  e_lin_prev_ = e_lin;
  double v = kp_linear_ * e_lin + ki_linear_ * e_lin_int_ + kd_linear_ * e_lin_der;
  v *= std::max(0.0, 1.0 - std::fabs(e_ang) / M_PI);   // 航向偏差大 -> 减速

  // 6) 限幅 + 到点停
  cmd.twist.linear.x  = std::max(-max_linear_,  std::min(max_linear_,  v));
  cmd.twist.angular.z = std::max(-max_angular_, std::min(max_angular_, omega));
  if (dist_goal < goal_tolerance_) { cmd.twist.linear.x = 0.0; }
  return cmd;
}

}  // namespace my_pid_controller

PLUGINLIB_EXPORT_CLASS(my_pid_controller::MyPidController, nav2_core::Controller)
)ALGO" },
  };
  return v;
}

}  // namespace rcj
