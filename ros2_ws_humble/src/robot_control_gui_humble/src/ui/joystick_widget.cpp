// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ui/joystick_widget.h"

#include <QPainter>
#include <QMouseEvent>
#include <cmath>

namespace rcj {

JoystickWidget::JoystickWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(160, 160);
}

void JoystickWidget::resizeEvent(QResizeEvent*) {
  center_ = QPointF(width() / 2.0, height() / 2.0);
  base_radius_ = std::min(width(), height()) / 2 - 10;
  stick_radius_ = base_radius_ / 4;
  stick_pos_ = center_;
}

void JoystickWidget::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(230, 230, 230));
  p.drawEllipse(center_, base_radius_, base_radius_);
  p.setBrush(QColor(180, 180, 180));
  p.drawEllipse(center_, 6, 6);
  p.setBrush(QColor(70, 130, 200));
  p.drawEllipse(stick_pos_, stick_radius_, stick_radius_);
}

void JoystickWidget::updateFromMouse(const QPoint& p) {
  QPointF v = QPointF(p) - center_;
  const double r = std::hypot(v.x(), v.y());
  if (r > base_radius_) {
    v *= base_radius_ / r;
  }
  stick_pos_ = center_ + v;
  const double nx = v.x() / base_radius_;
  const double ny = v.y() / base_radius_;
  emit moved(nx, ny);
  update();
}

void JoystickWidget::mousePressEvent(QMouseEvent* e) {
  pressed_ = true;
  updateFromMouse(e->pos());
}
void JoystickWidget::mouseMoveEvent(QMouseEvent* e) {
  if (pressed_) updateFromMouse(e->pos());
}
void JoystickWidget::mouseReleaseEvent(QMouseEvent*) {
  pressed_ = false;
  stick_pos_ = center_;
  emit moved(0, 0);
  update();
}

}  // namespace rcj
