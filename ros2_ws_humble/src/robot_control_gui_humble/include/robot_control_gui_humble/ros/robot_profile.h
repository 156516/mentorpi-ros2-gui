// SPDX-License-Identifier: MIT
//
// robot_profile.h —— 机器人配置档:话题名 / 坐标系 / 电池类型 / 能力开关。
// 让 GUI 能连不同的车而不改代码:在 robots/ 目录放一个 yaml,设置页选它即可。
//
#pragma once

#include <QString>
#include <QStringList>

namespace rcj {

struct RobotProfile {
  // 默认值 = mentorpi(保持原行为不变)
  QString file;                 // yaml 文件名(不含路径)
  QString name            = "MentorPi (默认)";
  QString robot_ip         = "192.168.149.1";

  QString cmd_vel          = "/controller/cmd_vel";
  QString scan             = "/scan_raw";
  QString odom             = "/odom";
  QString map              = "/map";
  QString plan             = "/plan";
  QString battery          = "/ros_robot_controller/battery";
  QString battery_type     = "uint16";          // uint16 | battery_state
  QString camera_topic     = "/ascamera/camera_publisher/rgb0/image_compressed";

  QString map_frame        = "map";
  QString odom_frame       = "odom";
  QString base_frame       = "base_footprint";
  QString lidar_frame      = "lidar_frame";

  bool has_nav2            = true;
  bool has_slam_controller = true;
  bool has_camera          = true;

  // 当前生效的配置(启动时由 loadSelected() 填)
  static const RobotProfile& cur();
  static RobotProfile& mutableCur();

  // 从 yaml 文件读(flat 的 `key: value` + `#` 注释)
  static bool loadFile(const QString& path, RobotProfile& out, QString* err);
  // 列出目录下所有 *.yaml(不含 _ 开头的)
  static QStringList listProfiles(const QString& dir);
  // 按 QSettings 里记的选择加载;目录不存在就保持默认
  static void loadSelected(const QString& dir);
};

}  // namespace rcj
