// SPDX-License-Identifier: MIT
//
// Centralised topic / service / action names.
// 速度/雷达/电池/坐标系全部来自「机器人配置档」(robots/*.yaml),换车不用改代码。
// 默认值仍是 mentorpi 的 /controller/cmd_vel (not the de-facto /cmd_vel),
// and the init_pose is an action, not a topic. Keep the project
// re-targetable by reading everything through this header.
#pragma once

#include <QString>

#include "robot_control_gui_humble/ros/robot_profile.h"
#include <string>

namespace rcj::topics {

// Commands & feedback
// mentorpi uses /controller/cmd_vel as the chassis input (it gets remapped
// internally to whatever the chassis driver subscribes). /cmd_vel is also
// subscribed by some nodes but the mentorpi bringup accepts commands via
// /controller/cmd_vel — that was the topic that moved the robot during
// initial testing, so keep it as the default.
inline QString cmdVel()           { return RobotProfile::cur().cmd_vel; }
inline QString odom()             { return RobotProfile::cur().odom; }
// mentorpi's lidar.launch publishes /scan_raw but NOT a filtered /scan;
// we subscribe to /scan_raw directly so the GUI works out of the box.
inline QString scan()             { return RobotProfile::cur().scan; }
// mentorpi publishes battery on its custom topic; fall back to the standard
// name if a downstream stack republishes it.
inline QString batteryState()     { return RobotProfile::cur().battery; }
inline std::string diagnostics()  { return std::string("/diagnostic_agg"); }
inline QString globalPath()       { return RobotProfile::cur().plan; }
inline QString localPath()        { return QStringLiteral("/local_plan"); }

// Map & TF
// mentorpi's slam_toolbox online_sync_launch.py publishes /map in the root
// namespace. We subscribe directly to /map. rtabmap uses /rtabmap/map
// (with namespace), but on mentorpi we use slam_toolbox instead because
// rtabmap needs RGBD cameras and mentorpi ships a 2D lidar (MS200).
inline QString mapTopic()         { return RobotProfile::cur().map; }
inline QString mapFrame()         { return RobotProfile::cur().map_frame; }
inline QString odomFrame()        { return RobotProfile::cur().odom_frame; }
inline QString baseFrame()        { return RobotProfile::cur().base_frame; }
inline QString robotBaseFrame()   { return RobotProfile::cur().base_frame; }

// Sensors
inline QString depthImage()       { return QStringLiteral("/depth_cam/depth/image_raw"); }
inline QString rgbImage()         { return QStringLiteral("/depth_cam/rgb/image_raw"); }
// mentorpi 自带相机(ascamera)的压缩图(JPEG),Qt 直接能解
inline QString cameraCompressedImage() { return RobotProfile::cur().camera_topic; }

// Nav2 actions & services
inline QString navigateToPose()   { return QStringLiteral("/navigate_to_pose"); }
inline QString followPath()       { return QStringLiteral("/follow_path"); }
inline QString computePath()      { return QStringLiteral("/compute_path_to_pose"); }
inline QString clearCostmapGlobal() { return QStringLiteral("/global_costmap/clear_entirely"); }
inline QString clearCostmapLocal()  { return QStringLiteral("/local_costmap/clear_entirely"); }
inline QString saveMapService()   { return QStringLiteral("/map_saver/save_map"); }
inline QString loadMapService()   { return QStringLiteral("/map_server/load_map"); }

// SLAM: robot-side slam_controller node + slam_toolbox
inline QString startSlamService()        { return QStringLiteral("/start_slam"); }
inline QString stopSlamService()         { return QStringLiteral("/stop_slam"); }
inline QString isMappingService()        { return QStringLiteral("/is_mapping"); }
inline QString slamRequestMethodTopic()  { return QStringLiteral("/slam_request_method"); }
inline QString slamStatusTopic()         { return QStringLiteral("/slam_status"); }
inline QString slamApplyParamsService()  { return QStringLiteral("/apply_slam_params"); }
inline QString slamParamsTopic()         { return QStringLiteral("/slam_params"); }
inline QString slamToolboxNode()         { return QStringLiteral("/slam_toolbox"); }
// The robot exposes slam_toolbox's OWN save service (slam_toolbox/srv/SaveMap,
// request {name}), NOT nav2's /map_saver/save_map (nav2_msgs/srv/SaveMap).
inline QString slamToolboxSaveMapService() { return QStringLiteral("/slam_toolbox/save_map"); }

// Nav2 参数管理(小车端 nav_controller 节点)
inline QString navParamsTopic()        { return QStringLiteral("/nav_params"); }
inline QString navApplyParamsService() { return QStringLiteral("/apply_nav_params"); }
inline QString navListPluginsService() { return QStringLiteral("/list_nav_plugins"); }

// mentorpi custom
inline QString initPoseAction()   { return QStringLiteral("/init_pose"); }

}  // namespace rcj::topics
