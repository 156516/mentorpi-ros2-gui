// SPDX-License-Identifier: MIT
//
// Centralised topic / service / action names.
// mentorpi's controller exposes /controller/cmd_vel (not the de-facto /cmd_vel),
// and the init_pose is an action, not a topic. Keep the project
// re-targetable by reading everything through this header.
#pragma once

#include <QString>
#include <string>

namespace rcj::topics {

// Commands & feedback
// mentorpi uses /controller/cmd_vel as the chassis input (it gets remapped
// internally to whatever the chassis driver subscribes). /cmd_vel is also
// subscribed by some nodes but the mentorpi bringup accepts commands via
// /controller/cmd_vel — that was the topic that moved the robot during
// initial testing, so keep it as the default.
inline QString cmdVel()           { return QStringLiteral("/controller/cmd_vel"); }
inline QString odom()             { return QStringLiteral("/odom"); }
// mentorpi's lidar.launch publishes /scan_raw but NOT a filtered /scan;
// we subscribe to /scan_raw directly so the GUI works out of the box.
inline QString scan()             { return QStringLiteral("/scan_raw"); }
// mentorpi publishes battery on its custom topic; fall back to the standard
// name if a downstream stack republishes it.
inline QString batteryState()     { return QStringLiteral("/ros_robot_controller/battery"); }
inline std::string diagnostics()  { return std::string("/diagnostic_agg"); }
inline QString globalPath()       { return QStringLiteral("/plan"); }
inline QString localPath()        { return QStringLiteral("/local_plan"); }

// Map & TF
// mentorpi's slam_toolbox online_sync_launch.py publishes /map in the root
// namespace. We subscribe directly to /map. rtabmap uses /rtabmap/map
// (with namespace), but on mentorpi we use slam_toolbox instead because
// rtabmap needs RGBD cameras and mentorpi ships a 2D lidar (MS200).
inline QString mapTopic()         { return QStringLiteral("/map"); }
inline QString mapFrame()         { return QStringLiteral("map"); }
inline QString odomFrame()        { return QStringLiteral("odom"); }
inline QString baseFrame()        { return QStringLiteral("base_footprint"); }
inline QString robotBaseFrame()   { return QStringLiteral("base_link"); }

// Sensors
inline QString depthImage()       { return QStringLiteral("/depth_cam/depth/image_raw"); }
inline QString rgbImage()         { return QStringLiteral("/depth_cam/rgb/image_raw"); }

// Nav2 actions & services
inline QString navigateToPose()   { return QStringLiteral("/navigate_to_pose"); }
inline QString followPath()       { return QStringLiteral("/follow_path"); }
inline QString computePath()      { return QStringLiteral("/compute_path_to_pose"); }
inline QString clearCostmapGlobal() { return QStringLiteral("/global_costmap/clear_entirely"); }
inline QString clearCostmapLocal()  { return QStringLiteral("/local_costmap/clear_entirely"); }
inline QString saveMapService()   { return QStringLiteral("/map_saver/save_map"); }
inline QString loadMapService()   { return QStringLiteral("/map_server/load_map"); }

// mentorpi custom
inline QString initPoseAction()   { return QStringLiteral("/init_pose"); }

}  // namespace rcj::topics
