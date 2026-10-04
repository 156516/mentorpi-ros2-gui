// SPDX-License-Identifier: MIT
//
// Tests for the static helpers on RobotController. The helpers are defined
// inline in robot_controller.h so the test compiles WITHOUT linking the full
// GUI library (which would require rclcpp init).

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include <QImage>
#include <QPointF>
#include <QVector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Including the lightweight header pulls in the inline helpers directly.
// We must NOT include robot_controller.h here, or we'd pull rclcpp into
// a test process and need to init it.
#include "robot_control_gui_jazzy/ros/ros_to_qt.h"

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(OccupancyGrid, FreeIsWhite) {
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = 2; g.info.height = 2; g.info.resolution = 0.05;
  g.data = {0, 0, 0, 0};  // all free
  const auto img = rcj::occupancyGridToImage(g);
  ASSERT_EQ(img.width(), 2);
  ASSERT_EQ(img.height(), 2);
  // QImage origin is upper-left; OccupancyGrid is lower-left → row order
  // is flipped. Bottom-left of map = (0,0) of grid.
  EXPECT_EQ(img.pixel(0, 1), qRgb(255, 255, 255));
  EXPECT_EQ(img.pixel(1, 0), qRgb(255, 255, 255));
}

TEST(OccupancyGrid, OccupiedIsBlack) {
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = 1; g.info.height = 1; g.info.resolution = 0.05;
  g.data = {100};
  const auto img = rcj::occupancyGridToImage(g);
  EXPECT_EQ(img.pixel(0, 0), qRgb(0, 0, 0));
}

TEST(OccupancyGrid, UnknownIsMidGrey) {
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = 1; g.info.height = 1; g.info.resolution = 0.05;
  g.data = {-1};
  const auto img = rcj::occupancyGridToImage(g);
  EXPECT_EQ(img.pixel(0, 0), qRgb(180, 180, 180));
}

TEST(LaserScan, ForwardOneMetre) {
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.0;
  scan.range_min = 0.05;
  scan.range_max = 10.0;
  scan.ranges = {1.0f};
  // robot at origin facing +x
  const auto pts = rcj::laserScanToPoints(scan, 0.0, 0.0, 0.0);
  ASSERT_EQ(pts.size(), 1);
  EXPECT_NEAR(pts[0].x(), 1.0, 1e-6);
  EXPECT_NEAR(pts[0].y(), 0.0, 1e-6);
}

TEST(LaserScan, FiltersOutOfRange) {
  // helper subsamples with optional step; default 1 = inspect every ray.
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.0;
  scan.range_min = 0.05;
  scan.range_max = 10.0;
  scan.ranges = {0.01f, 1.0f, 20.0f, std::numeric_limits<float>::quiet_NaN()};
  const auto pts = rcj::laserScanToPoints(scan, 0.0, 0.0, 0.0);
  // Only the second ray (range 1.0) is valid.
  ASSERT_EQ(pts.size(), 1);
  EXPECT_NEAR(pts[0].x(), 1.0, 1e-6);
}

TEST(LaserScan, BodyToWorldRotation) {
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.0;
  scan.range_min = 0.05;
  scan.range_max = 10.0;
  scan.ranges = {1.0f};
  // robot at (1,1), facing +y (yaw = π/2)
  const auto pts = rcj::laserScanToPoints(scan, 1.0, 1.0, M_PI / 2);
  ASSERT_EQ(pts.size(), 1);
  EXPECT_NEAR(pts[0].x(), 1.0, 1e-6);
  EXPECT_NEAR(pts[0].y(), 2.0, 1e-6);
}

TEST(Path, MapsPoses) {
  nav_msgs::msg::Path p;
  p.poses.resize(3);
  p.poses[0].pose.position.x = 0.0; p.poses[0].pose.position.y = 0.0;
  p.poses[1].pose.position.x = 1.0; p.poses[1].pose.position.y = 0.0;
  p.poses[2].pose.position.x = 1.0; p.poses[2].pose.position.y = 1.0;
  const auto pts = rcj::pathToPoints(p);
  ASSERT_EQ(pts.size(), 3);
  EXPECT_EQ(pts[0], QPointF(0.0, 0.0));
  EXPECT_EQ(pts[2], QPointF(1.0, 1.0));
}

TEST(Path, EmptyReturnsEmpty) {
  nav_msgs::msg::Path p;
  EXPECT_TRUE(rcj::pathToPoints(p).isEmpty());
}
