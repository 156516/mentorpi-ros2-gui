// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ros/slam_param_client.h"

#include "robot_control_gui_humble/ros/topic_names.h"

#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <sstream>

namespace rcj {

SlamParamClient::SlamParamClient(std::shared_ptr<rclcpp::Node> node)
    : node_(std::move(node)) {}

bool SlamParamClient::ensureParamClient() {
  if (param_client_) return true;
  if (!node_) return false;
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, topics::slamToolboxNode().toStdString());
  return true;
}

bool SlamParamClient::getParams(const std::vector<std::string>& names,
                                std::map<std::string, double>& out,
                                std::string* err) {
  if (!ensureParamClient()) { if (err) *err = "未连接 ROS"; return false; }
  if (!param_client_->wait_for_service(std::chrono::seconds(2))) {
    if (err) *err = "slam_toolbox 参数服务不可用(节点没在跑?)";
    return false;
  }
  auto future = param_client_->get_parameters(names);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    if (err) *err = "get_parameters 超时";
    return false;
  }
  for (const auto& p : future.get()) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      out[p.get_name()] = p.as_double();
    } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      out[p.get_name()] = static_cast<double>(p.as_int());
    }  // NOT_SET / other types: omit
  }
  return true;
}

bool SlamParamClient::setParams(const std::map<std::string, double>& values,
                                std::map<std::string, bool>& per_param_ok,
                                std::string* err) {
  if (!ensureParamClient()) { if (err) *err = "未连接 ROS"; return false; }
  if (!param_client_->wait_for_service(std::chrono::seconds(2))) {
    if (err) *err = "slam_toolbox 参数服务不可用(节点没在跑?)";
    return false;
  }
  std::vector<rclcpp::Parameter> ps;
  std::vector<std::string> names;
  ps.reserve(values.size());
  for (const auto& kv : values) {
    ps.emplace_back(kv.first, kv.second);
    names.push_back(kv.first);
  }
  auto future = param_client_->set_parameters(ps);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    if (err) *err = "set_parameters 超时";
    return false;
  }
  const auto results = future.get();
  bool all_ok = true;
  for (size_t i = 0; i < results.size() && i < names.size(); ++i) {
    per_param_ok[names[i]] = results[i].successful;
    if (!results[i].successful) all_ok = false;
  }
  return all_ok;
}

bool SlamParamClient::applyAndRestart(const std::map<std::string, double>& values,
                                      std::string* err) {
  if (!node_) { if (err) *err = "未连接 ROS"; return false; }

  // 1) stage values as a tiny hand-rolled JSON dict on /slam_params
  auto pub = node_->create_publisher<std_msgs::msg::String>(
      topics::slamParamsTopic().toStdString(), 10);
  std::ostringstream os;
  os << "{";
  bool first = true;
  for (const auto& kv : values) {
    if (!first) os << ",";
    first = false;
    os << "\"" << kv.first << "\":" << kv.second;
  }
  os << "}";
  std_msgs::msg::String m;
  m.data = os.str();
  // brief discovery grace so the robot-side subscriber is matched
  for (int i = 0; i < 20 && pub->get_subscription_count() == 0; ++i) {
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
  pub->publish(m);
  rclcpp::sleep_for(std::chrono::milliseconds(150));

  // 2) ask the robot to write its params YAML and restart SLAM
  auto client = node_->create_client<std_srvs::srv::Trigger>(
      topics::slamApplyParamsService().toStdString());
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    if (err) *err =
        "小车端不支持参数下发:/apply_slam_params 不存在(需更新 slam_controller.py)";
    return false;
  }
  auto future = client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  if (future.wait_for(std::chrono::seconds(20)) != std::future_status::ready) {
    if (err) *err = "/apply_slam_params 超时";
    return false;
  }
  const auto resp = future.get();
  if (err) *err = resp->message;
  return resp->success;
}

}  // namespace rcj
