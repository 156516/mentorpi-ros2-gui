// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ui/speed_dashboard.h"

#include <QPainter>
#include <QtMath>

namespace rcj {

SpeedDashboard::SpeedDashboard(QWidget* parent) : QWidget(parent) {
  setMinimumSize(180, 180);
}

void SpeedDashboard::setLinearSpeed(double v)  { lin_ = v; update(); }
void SpeedDashboard::setLateralSpeed(double v) { lat_ = v; update(); }
void SpeedDashboard::setAngularSpeed(double w) { ang_ = w; update(); }

void SpeedDashboard::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const QPointF c(width() / 2.0, height() / 2.0);
  const double r = std::min(width(), height()) / 2.0 - 8;

  // Linear (top half)
  p.setPen(Qt::black);
  p.drawArc(QRectF(c.x() - r, c.y() - r, r * 2, r * 2), 225 * 16, -270 * 16);

  const double linAng = -45.0 + (lin_ / 1.0) * -270.0;  // -1..1 m/s
  p.setPen(QPen(Qt::blue, 3));
  p.drawLine(c, QPointF(c.x() + r * std::cos(qDegreesToRadians(linAng)),
                        c.y() - r * std::sin(qDegreesToRadians(linAng))));

  // Lateral vy (麦轮左右平移)—— 绿色虚线指针
  const double latAng = -45.0 + (lat_ / 1.0) * -270.0;
  p.setPen(QPen(QColor(0, 160, 0), 2, Qt::DashLine));
  p.drawLine(c, QPointF(c.x() + r * std::cos(qDegreesToRadians(latAng)),
                        c.y() - r * std::sin(qDegreesToRadians(latAng))));

  // Angular (bottom half)
  p.setPen(Qt::blue);
  p.drawText(int(c.x()) - 40, int(c.y()) + 8,  tr("v =%1 m/s").arg(lin_, 0, 'f', 2));
  p.setPen(QColor(0, 140, 0));
  p.drawText(int(c.x()) - 40, int(c.y()) + 24, tr("vy=%1 m/s").arg(lat_, 0, 'f', 2));
  p.setPen(Qt::darkGray);
  p.drawText(int(c.x()) - 40, int(c.y()) + 40, tr("ω =%1 rad/s").arg(ang_, 0, 'f', 2));
}

}  // namespace rcj
