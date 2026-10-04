#pragma once
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

namespace my_pid {

// PID 路径跟踪控制器:
//   横向:朝"前瞻点"的航向误差 -> 角速度 PID
//   纵向:航向偏差越大越减速 -> 线速度
// 所有 PID 参数都能在 GUI「🔧 算法参数」里查/改(FollowPath.*)。
class MyPid : public nav2_core::Controller {
public:
  MyPid() = default;
  ~MyPid() override = default;

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

}  // namespace my_pid
