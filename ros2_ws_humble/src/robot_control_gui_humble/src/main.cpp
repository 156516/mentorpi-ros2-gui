// SPDX-License-Identifier: MIT
//
// Entry point: P11 "safe mode".
//
// ROS2 + QApplication share the same process. To avoid the GUI's mere
// presence disturbing the mentorpi robot (which appears to die whenever
// any extra DDS participant joins the network), we DON'T initialise ROS
// until the user clicks the "Connect" button. Until then the GUI is a
// plain Qt window that does not touch DDS at all.
//
// The ROS thread + executor are also created lazily on connect and
// destroyed on disconnect.

#include <QApplication>
#include <QMessageBox>
#include <QTimer>
#include <csignal>

#include "robot_control_gui_humble/ui/main_window.h"
#include "robot_control_gui_humble/ros/robot_controller.h"
#include "robot_control_gui_humble/ros/robot_profile.h"

// Register Qt metatypes so queued cross-thread signals work once ROS is up.
#include <QMetaType>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace {
struct MetaRegister {
  MetaRegister() {
    qRegisterMetaType<nav_msgs::msg::Odometry>("nav_msgs::msg::Odometry");
    qRegisterMetaType<geometry_msgs::msg::PoseArray>("geometry_msgs::msg::PoseArray");
    qRegisterMetaType<QImage>("QImage");
    qRegisterMetaType<nav_msgs::msg::OccupancyGrid>("nav_msgs::msg::OccupancyGrid");
    qRegisterMetaType<nav_msgs::msg::Path>("nav_msgs::msg::Path");
    qRegisterMetaType<sensor_msgs::msg::BatteryState>("sensor_msgs::msg::BatteryState");
    qRegisterMetaType<sensor_msgs::msg::LaserScan>("sensor_msgs::msg::LaserScan");
    qRegisterMetaType<diagnostic_msgs::msg::DiagnosticArray>("diagnostic_msgs::msg::DiagnosticArray");
  }
};
static MetaRegister g_meta_register;
}  // namespace

// Warn if a previous GUI instance is still alive (the user closed the
// terminal without clicking "Disconnect" first). This is the most common
// reason the mentorpi robot dies on second connect.
void check_for_zombie_gui_instance() {
  // Use ps to find any zombie robot_control_gui_humble_node processes.
  // (We can't easily call into our own session from here.)
  // Skip: just log to stderr so the user can see it in console.
  std::fprintf(stderr,
      "[gui] Tip: if you closed a previous terminal without clicking "
      "'Disconnect',\n"
      "     kill the leftover process: pkill -f robot_control_gui_humble_node\n");
}

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("mentorpi");
  QCoreApplication::setApplicationName("robot_control_gui_humble");

  std::signal(SIGINT, [](int) {
    rcj::rcj_disconnect();
    QCoreApplication::quit();
  });

  // 加载机器人配置档(话题名/坐标系/电池类型…),换车不用改代码
  rcj::RobotProfile::loadSelected(qEnvironmentVariable("RCJ_ROBOTS_DIR", "/robots"));
  std::fprintf(stderr, "[profile] 使用配置: %s (cmd_vel=%s scan=%s)\n",
               rcj::RobotProfile::cur().file.toUtf8().constData(),
               rcj::RobotProfile::cur().cmd_vel.toUtf8().constData(),
               rcj::RobotProfile::cur().scan.toUtf8().constData());

  rcj::MainWindow w;
  check_for_zombie_gui_instance();
  w.show();

  // 自动化测试:设了 RCJ_AUTOCONNECT_IP 就自动连接(跳过弹框)
  if (!qEnvironmentVariable("RCJ_AUTOCONNECT_IP").isEmpty())
    QTimer::singleShot(500, &w, &rcj::MainWindow::onConnectClicked);
  if (!qEnvironmentVariable("RCJ_SHOW_HELP").isEmpty())
    QTimer::singleShot(600, &w, &rcj::MainWindow::showHelp);

  const int rc = app.exec();

  rcj::rcj_disconnect();
  return rc;
}
