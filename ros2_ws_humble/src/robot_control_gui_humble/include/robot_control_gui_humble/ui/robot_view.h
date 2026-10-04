// SPDX-License-Identifier: MIT
#pragma once

#include <QGraphicsView>
#include <QHash>
#include <QPointF>
#include <QPointer>

class QGraphicsPixmapItem;
class QGraphicsEllipseItem;
class QGraphicsPolygonItem;
class QGraphicsPathItem;
class QGraphicsLineItem;
class QImage;

namespace rcj {

// Self-painted map view:
//   - map (nav_msgs/OccupancyGrid) → grayscale QPixmap aligned to map frame
//   - robot pose (odom)            → green triangle
//   - laser scan                   → cyan fan
//   - planned path                 → yellow polyline
//   - goal / initial-pose markers
//
// Interaction:
//   wheel        = zoom (anchor under mouse)
//   middle drag  = pan
//   right drag   = rotate about the viewport centre  (NOT in edit mode)
//   left click   = set goal (Shift+left = set initial pose)  (NOT in edit mode)
//   left/right drag = paint brush  (edit mode only)
//
// Coordinate convention: scene is metres in the map frame, with y flipped up
// (see fitWithYUp) so the robot's forward points up on screen.
class RobotView : public QGraphicsView {
  Q_OBJECT
public:
  explicit RobotView(QWidget* parent = nullptr);

  bool editMode() const { return edit_mode_; }
  // Brush value written while dragging in edit mode: 100 = occupied, 0 = free.
  int  brushValue() const { return brush_value_; }
  void setBrushValue(int v) { brush_value_ = v; }

signals:
  void goalSelected(double x, double y, double yaw);   // 兼容保留
  // 地图上"左键按下并拖动"= 起草一个目标(位置+朝向),不立即下发。
  void goalDrafted(double x, double y, double yaw);
  void initialPoseSelected(double x, double y, double yaw);
  void viewRotated(double degrees);
  void editStrokeBegan();
  void editStroke(double world_x, double world_y, int value);
  void editStrokeFinished();

public slots:
  void setMap(const QImage& img,
              double resolution,
              double origin_x, double origin_y, double origin_yaw);
  // Update the existing pixmap in place — NO auto-fit (used by the map editor).
  void setMapImage(const QImage& img);
  void setRobotPose(double x, double y, double yaw);
  void setLaserScan(const QVector<QPointF>& ranges_m);
  void setPath(const QVector<QPointF>& path_m);
  void setGoal(double x, double y, double yaw);
  void setInitialPose(double x, double y, double yaw);
  void clearOverlays();
  void resetView();
  void setEditMode(bool on);
  // Nav2 代价地图覆盖层(local=true 为局部代价地图)
  void setCostmapImage(const QImage& img, double resolution,
                       double origin_x, double origin_y, double origin_yaw,
                       bool local);
  void setCostmapVisible(bool global_on, bool local_on);
  // 目标草稿(点"开始导航"前只是一个待发的标记)
  bool hasDraftGoal() const { return has_draft_goal_; }
  bool takeDraftGoal(double& x, double& y, double& yaw);
  void clearDraftGoal();
  // AMCL 粒子云(定位不确定性可视化)
  void setParticleCloud(const QVector<QPointF>& pts);
  void setParticleVisible(bool on);

protected:
  void wheelEvent(QWheelEvent* e) override;
  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;
  void resizeEvent(QResizeEvent* e) override;

private:
  // fitInView 之后把 view 的 y 轴翻过来(Qt 场景默认 y 向下,而世界坐标 y 向上)
  void fitWithYUp();
  void rebuildRobotPolygon();

  QGraphicsPixmapItem*  map_item_{nullptr};
  QGraphicsPolygonItem* robot_poly_{nullptr};
  QGraphicsEllipseItem* goal_marker_{nullptr};
  QGraphicsLineItem*    goal_arrow_{nullptr};
  QGraphicsEllipseItem* initial_marker_{nullptr};
  QGraphicsPathItem*    path_item_{nullptr};
  QGraphicsPixmapItem*  global_costmap_item_{nullptr};
  QGraphicsPixmapItem*  local_costmap_item_{nullptr};
  bool global_costmap_on_{false};
  bool local_costmap_on_{false};
  QGraphicsPathItem*    particle_item_{nullptr};
  bool particles_on_{true};
  bool has_draft_goal_{false};
  bool goal_dragging_{false};
  double draft_x_{0.0}, draft_y_{0.0}, draft_yaw_{0.0};

  // laser fan is recreated every update for simplicity
  QList<QGraphicsItem*> laser_items_;

  double map_resolution_{0.05};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  double map_origin_yaw_{0.0};
  int    map_height_px_{0};
  // Auto-fit only the FIRST map. Otherwise the ~1 Hz /map refresh during live
  // mapping would reset the user's zoom / rotation / pan on every message.
  bool   map_fitted_{false};

  double robot_x_{0.0}, robot_y_{0.0}, robot_yaw_{0.0};
  double robot_length_{0.30};
  double robot_width_{0.30};

  QPointF last_drag_pos_;
  bool    panning_{false};
  bool    rotating_{false};
  double  rotation_deg_{0.0};
  bool    edit_mode_{false};
  int     brush_value_{100};   // 100 = draw wall, 0 = erase
};

}  // namespace rcj
