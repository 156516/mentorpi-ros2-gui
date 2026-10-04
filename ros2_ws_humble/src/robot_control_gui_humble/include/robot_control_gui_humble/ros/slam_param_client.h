// SPDX-License-Identifier: MIT
#pragma once
//
// SlamParamClient — read/write slam_toolbox parameters from the GUI, and ask
// the robot-side slam_controller to apply launch-time params + restart SLAM.
//
// Uses rclcpp::AsyncParametersClient on /slam_toolbox plus a small custom
// interface the robot side exposes:
//   /slam_params         (std_msgs/String, JSON dict)  -> stage values
//   /apply_slam_params   (std_srvs/Trigger)            -> write yaml + restart
//
// IMPORTANT: the node is spun by a global MultiThreadedExecutor, so we must
// NEVER spin here — use future.wait_for() like SlamServiceClient does.
//
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace rcj {

class SlamParamClient {
 public:
  explicit SlamParamClient(std::shared_ptr<rclcpp::Node> node);

  // Read the named parameters. Missing/unset params are simply omitted.
  bool getParams(const std::vector<std::string>& names,
                 std::map<std::string, double>& out,
                 std::string* err);

  // Best-effort dynamic set. per_param_ok reports slam_toolbox's own
  // SetParametersResult for each name (launch-time params report false).
  bool setParams(const std::map<std::string, double>& values,
                 std::map<std::string, bool>& per_param_ok,
                 std::string* err);

  // Stage values on /slam_params, then call /apply_slam_params so the robot
  // writes its params YAML and restarts SLAM. Fails cleanly if the robot-side
  // service is absent (older slam_controller).
  bool applyAndRestart(const std::map<std::string, double>& values, std::string* err);

 private:
  bool ensureParamClient();
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
};

}  // namespace rcj
