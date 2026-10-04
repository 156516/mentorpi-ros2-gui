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

#include <geometry_msgs/msg/pose_array.hpp>
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

// nav_msgs/OccupancyGrid(Nav2 代价地图) → RViz/nav2 默认色带(蓝→青→绿→黄→红),致命黑。
// 可走/未知 = 透明 / 代价 = 青→黄→红 / 致命障碍 = 纯黑。
inline QImage costmapToImage(const nav_msgs::msg::OccupancyGrid& grid) {
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  if (w <= 0 || h <= 0 || static_cast<int>(grid.data.size()) < w * h) return QImage();
  QImage img(w, h, QImage::Format_ARGB32);
  img.fill(Qt::transparent);
  for (int y = 0; y < h; ++y) {
    const int row_from_top = h - 1 - y;          // 和 occupancyGridToImage 一致的翻转
    const int8_t* row = grid.data.data() + static_cast<size_t>(row_from_top) * w;
    for (int x = 0; x < w; ++x) {
      const int c = row[x];
      if (c < 0) continue;                       // -1 = 未知 → 透明(透出底图)
      // 标准约定(实测确认):0 = 空闲, 1..98 = 膨胀代价,
      // 99 = INSCRIBED(内切/贴墙), 100 = LETHAL(致命障碍)。和普通
      // OccupancyGrid 一致 —— 不是反的!(之前搞反了,把空闲画成了黑)
      const int cost = std::min(c, 100);
      if (cost <= 0) continue;                   // 空闲 → 透明(透出底图)
      if (cost >= 100) {                         // 致命障碍 → 黑(RViz 惯例)
        img.setPixel(x, y, qRgba(0, 0, 0, 255));
        continue;
      }
      // RViz/nav2 默认代价地图色带(蓝→青→绿→黄→红),和 hiwonder 文档一致
      const int alpha = std::min(240, 60 + cost * 180 / 100);
      struct Stop { double t; int r, g, b; };
      static const Stop kStops[] = {
        { 0.00,  30,  50, 145 },   // 深蓝 = 代价很低
        { 0.25,  40, 190, 190 },   // 青(略柔)
        { 0.50,  70, 190,  90 },   // 绿(略柔)
        { 0.75, 226, 214,  86 },   // 柔和的黄
        { 1.00, 170,  85,  75 },   // 暗砖红 = 代价很高(刻意的暗,不和雷达的鲜红撞)
      };
      const double t = cost / 100.0;
      int r = 0, g = 0, b = 0;
      for (size_t i = 1; i < sizeof(kStops)/sizeof(kStops[0]); ++i) {
        if (t <= kStops[i].t || i + 1 == sizeof(kStops)/sizeof(kStops[0])) {
          const Stop& a = kStops[i-1];
          const Stop& c = kStops[i];
          const double f = (c.t > a.t) ? (t - a.t) / (c.t - a.t) : 0.0;
          r = static_cast<int>(a.r + f * (c.r - a.r));
          g = static_cast<int>(a.g + f * (c.g - a.g));
          b = static_cast<int>(a.b + f * (c.b - a.b));
          break;
        }
      }
      img.setPixel(x, y, qRgba(r, g, b, alpha));
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

// AMCL 粒子云(PoseArray)→ 平面点集,用来画"定位有多确定"。
inline QVector<QPointF> poseArrayToPoints(const geometry_msgs::msg::PoseArray& pa) {
  QVector<QPointF> pts;
  pts.reserve(static_cast<int>(pa.poses.size()));
  for (const auto& p : pa.poses)
    pts << QPointF(p.position.x, p.position.y);
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
