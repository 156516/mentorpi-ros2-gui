// SPDX-License-Identifier: MIT
#include "robot_control_gui_jazzy/ui/robot_view.h"

#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPathItem>
#include <QGraphicsLineItem>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QTransform>
#include <QtMath>
#include <cmath>

namespace rcj {

RobotView::RobotView(QWidget* parent) : QGraphicsView(parent) {
  setScene(new QGraphicsScene(this));
  setRenderHint(QPainter::Antialiasing);
  setBackgroundBrush(QColor(40, 40, 40));
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setDragMode(QGraphicsView::NoDrag);
  // default scene: 20x20 m centred on origin
  scene()->setSceneRect(-10, -10, 20, 20);

  // initial grid
  setRobotPose(0, 0, 0);
}

// Qt 的 QGraphicsScene 默认 y 轴向下,而地图/机器人/雷达点/路径都按
// "世界 y 向上" 摆放。fit 之后把 view 的 y 翻转,世界 +y 才显示在上方。
// 必须 setTransform(单位阵):否则每次 fit 的 scale(1,-1) 会互相抵消。
void RobotView::fitWithYUp() {
  setTransform(QTransform());
  fitInView(scene()->sceneRect(), Qt::KeepAspectRatio);
  scale(1.0, -1.0);
}

void RobotView::resizeEvent(QResizeEvent* e) {
  QGraphicsView::resizeEvent(e);
  if (!map_item_) {
    fitWithYUp();
  }
}

void RobotView::wheelEvent(QWheelEvent* e) {
  const double factor = std::pow(1.2, e->angleDelta().y() / 240.0);
  scale(factor, factor);
}

void RobotView::mousePressEvent(QMouseEvent* e) {
  if (e->button() == Qt::MiddleButton) {
    panning_ = true;
    last_drag_pos_ = e->pos();
    setCursor(Qt::ClosedHandCursor);
    return;
  }
  if (e->button() == Qt::RightButton) {
    if (edit_mode_) {                       // edit mode: right = erase stroke
      emit editStrokeBegan();
      const QPointF w = mapToScene(e->pos());
      emit editStroke(w.x(), w.y(), 0);
      return;
    }
    rotating_ = true;                       // otherwise: right drag = rotate
    last_drag_pos_ = e->pos();
    setCursor(Qt::SizeAllCursor);
    return;
  }
  if (e->button() == Qt::LeftButton) {
    if (edit_mode_) {                       // edit mode: left = paint stroke
      emit editStrokeBegan();
      const QPointF w = mapToScene(e->pos());
      emit editStroke(w.x(), w.y(), brush_value_);
      return;
    }
    const QPointF world = mapToScene(e->pos());
    const double yaw = 0.0;
    if (e->modifiers() & Qt::ShiftModifier) {
      setInitialPose(world.x(), world.y(), yaw);
      emit initialPoseSelected(world.x(), world.y(), yaw);
    } else {
      // compute heading from robot to click
      const double dx = world.x() - robot_x_;
      const double dy = world.y() - robot_y_;
      const double heading = std::atan2(dy, dx);
      setGoal(world.x(), world.y(), heading);
      emit goalSelected(world.x(), world.y(), heading);
    }
  }
}

void RobotView::mouseMoveEvent(QMouseEvent* e) {
  if (panning_) {
    const QPointF delta = e->pos() - last_drag_pos_;
    last_drag_pos_ = e->pos();
    // translate() works in device pixels and composes with whatever zoom /
    // rotation is active, so it stays 1:1 with the cursor. The old code
    // divided by transform().m11()/m22(), which assumes an axis-aligned scale
    // and is simply wrong once the view is rotated.
    translate(delta.x(), delta.y());
    return;
  }
  if (rotating_) {
    const QPointF c(viewport()->rect().center());
    const QPointF a = last_drag_pos_ - c;
    const QPointF b = e->pos()         - c;
    const double cross = a.x() * b.y() - a.y() * b.x();
    const double dot   = a.x() * b.x() + a.y() * b.y();
    const double deg   = qRadiansToDegrees(std::atan2(cross, dot));
    last_drag_pos_ = e->pos();
    if (deg != 0.0) {
      // rotate() left-multiplies in device space, so the y-flip from
      // fitWithYUp() is preserved. It rotates about the viewport origin, which
      // would swing the scene; re-centring on the same anchor makes the net
      // effect a rotation about that anchor.
      const QPointF anchor = mapToScene(viewport()->rect().center());
      rotate(deg);
      centerOn(anchor);
      rotation_deg_ = std::fmod(rotation_deg_ + deg, 360.0);
      emit viewRotated(rotation_deg_);
    }
    return;
  }
  if (edit_mode_) {
    if (e->buttons() & Qt::LeftButton) {
      const QPointF w = mapToScene(e->pos());
      emit editStroke(w.x(), w.y(), brush_value_);
    } else if (e->buttons() & Qt::RightButton) {
      const QPointF w = mapToScene(e->pos());
      emit editStroke(w.x(), w.y(), 0);
    }
  }
}

void RobotView::mouseReleaseEvent(QMouseEvent* e) {
  // BUGFIX: without this, panning_ latched true and every subsequent
  // mouse-move kept panning even with no button held.
  if (e->button() == Qt::MiddleButton) {
    panning_ = false;
    if (!edit_mode_) unsetCursor();
  }
  if (e->button() == Qt::RightButton) {
    if (rotating_) { rotating_ = false; if (!edit_mode_) unsetCursor(); }
    else if (edit_mode_) { emit editStrokeFinished(); }
  }
  if (e->button() == Qt::LeftButton && edit_mode_) {
    emit editStrokeFinished();
  }
}

void RobotView::setMap(const QImage& img,
                       double resolution,
                       double origin_x, double origin_y, double origin_yaw) {
  if (map_item_) {
    scene()->removeItem(map_item_);
    delete map_item_;
    map_item_ = nullptr;
  }
  map_resolution_ = resolution;
  map_origin_x_   = origin_x;
  map_origin_y_   = origin_y;
  map_origin_yaw_ = origin_yaw;
  map_height_px_  = img.height();

  QPixmap pm = QPixmap::fromImage(img);
  map_item_ = scene()->addPixmap(pm);
  map_item_->setZValue(-10);
  // Scale pixmap pixels to metres (1 px = resolution m) AND flip Y in ONE
  // transform: ROS map origin is lower-left, pixmap origin is upper-left.
  // NOTE: do NOT also call setScale(resolution) here -- setTransform() with
  // combine=true MULTIPLIES with the existing transform, so the two together
  // squared the scale (0.05 -> 0.0025), drawing the map 20x too small with the
  // robot marker landing outside it.
  map_item_->setTransform(QTransform::fromScale(resolution, -resolution));

  // The pixmap's top-left after transform sits at (origin_x, origin_y + height*res).
  // We want its bottom-left at (origin_x, origin_y). Apply translation.
  const double h = img.height() * resolution;
  map_item_->setPos(origin_x, origin_y + h);
  // yaw: rotate around the map origin
  map_item_->setTransformOriginPoint(0, 0);
  map_item_->setRotation(qRadiansToDegrees(origin_yaw));

  scene()->setSceneRect(scene()->itemsBoundingRect().adjusted(-2, -2, 2, 2));
  // Auto-fit ONLY the first map — afterwards the user's zoom / rotation / pan
  // must survive the map refresh (live mapping publishes ~1 Hz).
  if (!map_fitted_) {
    fitWithYUp();
    map_fitted_ = true;
  }
}

void RobotView::setMapImage(const QImage& img) {
  // Update the pixmap in place without touching the view transform.
  if (!map_item_) {
    setMap(img, map_resolution_, map_origin_x_, map_origin_y_, map_origin_yaw_);
    return;
  }
  map_item_->setPixmap(QPixmap::fromImage(img));
}

void RobotView::resetView() {
  rotation_deg_ = 0.0;
  map_fitted_   = true;
  fitWithYUp();
  emit viewRotated(0.0);
}

void RobotView::setEditMode(bool on) {
  edit_mode_ = on;
  setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
}

void RobotView::rebuildRobotPolygon() {
  if (robot_poly_) {
    scene()->removeItem(robot_poly_);
    delete robot_poly_;
  }
  const double L = robot_length_, W = robot_width_;
  // isoceles triangle pointing along +x
  QPolygonF poly;
  poly << QPointF( L / 2.0, 0.0)
       << QPointF(-L / 2.0,  W / 2.0)
       << QPointF(-L / 2.0, -W / 2.0);
  // build a copy rotated by yaw and translated to pose
  QPolygonF rotated;
  const double c = std::cos(robot_yaw_), s = std::sin(robot_yaw_);
  for (const auto& p : poly) {
    rotated << QPointF(p.x() * c - p.y() * s + robot_x_,
                       p.x() * s + p.y() * c + robot_y_);
  }
  robot_poly_ = scene()->addPolygon(rotated, QPen(Qt::green, 0.02),
                                    QBrush(QColor(0, 200, 0, 80)));
  robot_poly_->setZValue(2);
}

void RobotView::setRobotPose(double x, double y, double yaw) {
  robot_x_ = x; robot_y_ = y; robot_yaw_ = yaw;
  rebuildRobotPolygon();
}

void RobotView::setLaserScan(const QVector<QPointF>& ranges_m) {
  for (auto* it : laser_items_) { scene()->removeItem(it); delete it; }
  laser_items_.clear();
  for (const auto& p : ranges_m) {
    auto* line = scene()->addLine(robot_x_, robot_y_, p.x(), p.y(),
                                  QPen(QColor(0, 255, 255, 120), 0.01));
    line->setZValue(0);
    laser_items_ << line;
  }
}

void RobotView::setPath(const QVector<QPointF>& path_m) {
  if (path_item_) { scene()->removeItem(path_item_); delete path_item_; }
  if (path_m.size() < 2) return;
  QPainterPath pp;
  pp.moveTo(path_m.first());
  for (int i = 1; i < path_m.size(); ++i) pp.lineTo(path_m[i]);
  path_item_ = scene()->addPath(pp, QPen(Qt::yellow, 0.04));
  path_item_->setZValue(1);
}

void RobotView::setGoal(double x, double y, double /*yaw*/) {
  if (goal_marker_) { scene()->removeItem(goal_marker_); delete goal_marker_; }
  goal_marker_ = scene()->addEllipse(x - 0.1, y - 0.1, 0.2, 0.2,
                                     QPen(Qt::red, 0.02), QBrush(Qt::red));
  goal_marker_->setZValue(3);
}

void RobotView::setInitialPose(double x, double y, double /*yaw*/) {
  if (initial_marker_) { scene()->removeItem(initial_marker_); delete initial_marker_; }
  initial_marker_ = scene()->addEllipse(x - 0.15, y - 0.15, 0.3, 0.3,
                                        QPen(Qt::cyan, 0.02), QBrush(Qt::cyan));
  initial_marker_->setZValue(3);
}

void RobotView::clearOverlays() {
  for (auto* it : laser_items_) { scene()->removeItem(it); delete it; }
  laser_items_.clear();
  if (path_item_) { scene()->removeItem(path_item_); delete path_item_; }
  if (goal_marker_) { scene()->removeItem(goal_marker_); delete goal_marker_; }
  if (initial_marker_) { scene()->removeItem(initial_marker_); delete initial_marker_; }
  path_item_ = nullptr; goal_marker_ = nullptr; initial_marker_ = nullptr;
}

}  // namespace rcj
