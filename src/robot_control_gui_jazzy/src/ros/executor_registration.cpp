// SPDX-License-Identifier: MIT
//
// P11: lazy ROS connection lifecycle.
// Until the user clicks "Connect", the GUI is just a Qt window that
// does not touch DDS. Connecting initialises rclcpp, builds a
// MultiThreadedExecutor, and starts a std::thread to spin it. Disconnecting
// tears all of that down.
//
// This avoids the mentorpi robot dying whenever a stray DDS participant
// joins the network.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <memory>
#include <thread>
#include <unistd.h>      // getpid

namespace rcj {

namespace {
rclcpp::executors::MultiThreadedExecutor::SharedPtr g_executor;
std::shared_ptr<rclcpp::Node> g_node;
std::thread* g_ros_thread = nullptr;
bool g_ros_initialised = false;
}  // namespace

void register_executor(rclcpp::executors::MultiThreadedExecutor::SharedPtr exec) {
  g_executor = exec;
}

std::shared_ptr<rclcpp::Node> rcj_connect() {
  if (g_ros_initialised) return g_node;
  int argc = 0;
  char** argv = nullptr;
  rclcpp::init(argc, argv);

  g_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  g_node = rclcpp::Node::make_shared(
      "robot_control_gui_jazzy_" + std::to_string(::getpid()));
  g_executor->add_node(g_node);
  g_ros_thread = new std::thread([]() {
    g_executor->spin();
  });
  g_ros_initialised = true;
  return g_node;
}

void rcj_disconnect() {
  if (!g_ros_initialised) return;
  g_executor->cancel();
  if (g_ros_thread) {
    if (g_ros_thread->joinable()) g_ros_thread->join();
    delete g_ros_thread;
    g_ros_thread = nullptr;
  }
  g_executor.reset();
  g_node.reset();
  rclcpp::shutdown();
  g_ros_initialised = false;
}

}  // namespace rcj
