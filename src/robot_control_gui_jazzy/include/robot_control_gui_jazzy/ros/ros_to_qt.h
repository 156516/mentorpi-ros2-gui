// SPDX-License-Identifier: MIT
#pragma once
//
// Pure helpers for converting ROS messages → Qt drawing primitives.
// These are inline and depend ONLY on Qt + nav/sensor/geometry_msgs types,
// so unit tests can use them without linking rclcpp.
// (No Q_OBJECT / Qt MOC macros needed — keeps the translation unit small.)
//
#include <QImage>
#include <QPointF>
#include <QVector>
#include <cmath>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace rcj {

inline QImage occupancyGridToImage(const nav_msgs::msg::OccupancyGrid& grid) {
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  if (w <= 0 || h <= 0) return QImage();

  QImage img(w, h, QImage::Format_RGB32);
  // nav_msgs convention: 0=free, 100=occupied, -1=unknown
  // ROS map origin is lower-left; QImage origin is upper-left.
  // Flip rows so the resulting image aligns with the map frame.
  for (int y = 0; y < h; ++y) {
    const int row_from_top = h - 1 - y;
    const auto* row = grid.data.data() + row_from_top * w;
    for (int x = 0; x < w; ++x) {
      const int8_t v = row[x];
      if (v < 0) {
        img.setPixel(x, y, qRgb(180, 180, 180));
      } else {
        const int g = 255 - (v * 255 / 100);
        img.setPixel(x, y, qRgb(g, g, g));
      }
    }
  }
  return img;
}

inline QVector<QPointF> laserScanToPoints(const sensor_msgs::msg::LaserScan& scan,
                                          double rx, double ry, double r_yaw,
                                          int step = 1) {
  QVector<QPointF> pts;
  pts.reserve(scan.ranges.size() / std::max(1, step));
  const double c = std::cos(r_yaw), s = std::sin(r_yaw);
  for (size_t i = 0; i < scan.ranges.size(); i += step) {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double lx = r * std::cos(a);
    const double ly = r * std::sin(a);
    const double wx = lx * c - ly * s + rx;
    const double wy = lx * s + ly * c + ry;
    pts << QPointF(wx, wy);
  }
  return pts;
}

inline QVector<QPointF> pathToPoints(const nav_msgs::msg::Path& path) {
  QVector<QPointF> pts;
  pts.reserve(path.poses.size());
  for (const auto& p : path.poses) {
    pts << QPointF(p.pose.position.x, p.pose.position.y);
  }
  return pts;
}

}  // namespace rcj
