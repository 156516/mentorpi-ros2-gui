// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ui/robot_status_panel.h"

#include <sensor_msgs/msg/battery_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.h>          // for getYaw

// (intentionally include the full headers here, not just fwd-decl in the .h)

#include <QProgressBar>
#include <QLabel>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <cmath>
#include <algorithm>

namespace rcj {

RobotStatusPanel::RobotStatusPanel(QWidget* parent) : QWidget(parent) {
  auto* root = new QVBoxLayout(this);

  // Battery block
  auto* bat_box = new QGroupBox(tr("电池"), this);
  auto* bat_form = new QFormLayout(bat_box);
  battery_bar_ = new QProgressBar(bat_box);
  battery_bar_->setRange(0, 100);
  battery_bar_->setFormat("%p%");
  voltage_label_  = new QLabel("--", bat_box);
  current_label_  = new QLabel("--", bat_box);
  temp_label_     = new QLabel("--", bat_box);
  bat_form->addRow(battery_bar_);
  bat_form->addRow(tr("电压"),  voltage_label_);
  bat_form->addRow(tr("电流"),  current_label_);
  bat_form->addRow(tr("温度"),  temp_label_);
  root->addWidget(bat_box);

  // WiFi block
  auto* wifi_box = new QGroupBox(tr("WiFi"), this);
  auto* wifi_lay = new QVBoxLayout(wifi_box);
  wifi_bar_ = new QProgressBar(wifi_box);
  wifi_bar_->setRange(0, 100);
  wifi_lay->addWidget(wifi_bar_);
  root->addWidget(wifi_box);

  // Pose block
  auto* pose_box = new QGroupBox(tr("里程计"), this);
  auto* pose_form = new QFormLayout(pose_box);
  pose_label_ = new QLabel(tr("(无数据)"), pose_box);
  pose_label_->setWordWrap(true);
  pose_form->addRow(pose_label_);
  root->addWidget(pose_box);

  status_ = new QLabel(tr("等待数据..."), this);
  root->addWidget(status_);
  root->addStretch();
}

void RobotStatusPanel::onBattery(const sensor_msgs::msg::BatteryState& b) {
  // percentage = 0..100; some platforms don't publish it, fall back to voltage
  float pct = b.percentage * 100.0f;
  if (pct <= 0.0f && b.voltage > 0.0f) {
    // naive 3S LiPo approximation
    pct = std::clamp((b.voltage - 9.0f) / (12.6f - 9.0f) * 100.0f, 0.0f, 100.0f);
  }
  battery_bar_->setValue(static_cast<int>(pct));
  voltage_label_->setText(QString::number(b.voltage, 'f', 2) + " V");
  current_label_->setText(QString::number(b.current, 'f', 2) + " A");
  temp_label_->setText(QString::number(b.temperature, 'f', 1) + " °C");
  if (status_) status_->setText(tr("电池: %1V").arg(b.voltage, 0, 'f', 2));
}

void RobotStatusPanel::onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray& d) {
  for (const auto& s : d.status) {
    if (s.name.find("wifi") != std::string::npos ||
        s.name.find("WiFi") != std::string::npos) {
      // crude: read first integer in message
      for (const auto& kv : s.values) {
        if (kv.key == "signal" || kv.key == "Signal" || kv.key == "level") {
          last_wifi_ = QString::fromStdString(kv.value).toInt();
          updateWifi(last_wifi_);
        }
      }
    }
  }
}

void RobotStatusPanel::onOdom(const nav_msgs::msg::Odometry& o) {
  const auto& p = o.pose.pose.position;
  const auto& q = o.pose.pose.orientation;
  // yaw from quaternion
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  pose_label_->setText(
    QString("x=%1 m  y=%2 m  yaw=%3 deg\nv=%4 m/s  w=%5 rad/s")
      .arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2).arg(yaw * 180.0 / M_PI, 0, 'f', 1)
      .arg(o.twist.twist.linear.x, 0, 'f', 2).arg(o.twist.twist.angular.z, 0, 'f', 2));
  if (status_) status_->setText(tr("odom: x=%1 y=%2").arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2));
}

void RobotStatusPanel::updateBattery(float pct, float voltage) {
  battery_bar_->setValue(static_cast<int>(pct));
  voltage_label_->setText(QString::number(voltage, 'f', 2) + " V");
}
void RobotStatusPanel::updateWifi(int strength) { wifi_bar_->setValue(strength); }

}  // namespace rcj
