// SPDX-License-Identifier: MIT
#pragma once

#include <QObject>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace rcj {

// Thin wrapper that subscribes to /diagnostic_agg and surfaces
// human-readable summaries (battery, wifi, motor temperature) via Qt signals.
class DiagnosticsWatcher : public QObject {
  Q_OBJECT
public:
  explicit DiagnosticsWatcher(
    std::shared_ptr<rclcpp::Node> node, QObject* parent = nullptr);

signals:
  void updated(const diagnostic_msgs::msg::DiagnosticArray& msg);

private:
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
};

}  // namespace rcj
