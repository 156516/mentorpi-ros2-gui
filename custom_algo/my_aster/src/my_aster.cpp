#include "my_aster/my_aster.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace my_aster {

void MyPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                          std::string name, std::shared_ptr<tf2_ros::Buffer>,
                          std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  RCLCPP_INFO(node_->get_logger(), "MyPlanner '%s' 已配置", name_.c_str());
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
  const int N = 20;
  for (int i = 0; i <= N; ++i) {
    const double t = static_cast<double>(i) / N;
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = start.pose.position.x + t * (goal.pose.position.x - start.pose.position.x);
    p.pose.position.y = start.pose.position.y + t * (goal.pose.position.y - start.pose.position.y);
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  return path;
}

}  // namespace my_aster

PLUGINLIB_EXPORT_CLASS(my_aster::MyPlanner, nav2_core::GlobalPlanner)
