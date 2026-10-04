// SPDX-License-Identifier: MIT
#include "robot_control_gui_jazzy/ros/diagnostics_watcher.h"
#include "robot_control_gui_jazzy/ros/topic_names.h"

namespace rcj {

DiagnosticsWatcher::DiagnosticsWatcher(std::shared_ptr<rclcpp::Node> node, QObject* parent)
    : QObject(parent) {
  sub_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      topics::diagnostics(),
      rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        emit updated(*msg);
      });
}

}  // namespace rcj
