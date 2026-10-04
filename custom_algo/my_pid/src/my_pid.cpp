#include "my_pid/my_pid.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace my_pid {

void MyPid::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
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
              "MyPid '%s' 已配置 (kp_ang=%.2f ki_ang=%.2f kd_ang=%.2f)",
              name_.c_str(), kp_angular_, ki_angular_, kd_angular_);
}

void MyPid::cleanup()    { RCLCPP_INFO(node_->get_logger(), "cleanup"); }
void MyPid::activate()   { RCLCPP_INFO(node_->get_logger(), "activate"); }
void MyPid::deactivate() { RCLCPP_INFO(node_->get_logger(), "deactivate"); }
void MyPid::setPlan(const nav_msgs::msg::Path & path) {
  global_plan_ = path;
  e_ang_int_ = e_lin_int_ = 0.0;      // 新路径 -> 清积分,避免旧积分捣乱
  e_ang_prev_ = e_lin_prev_ = 0.0;
}
void MyPid::setSpeedLimit(const double &, const bool &) {}

geometry_msgs::msg::TwistStamped MyPid::computeVelocityCommands(
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

}  // namespace my_pid

PLUGINLIB_EXPORT_CLASS(my_pid::MyPid, nav2_core::Controller)
