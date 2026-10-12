// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ui/control_panel.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QGroupBox>
#include <QImage>
#include <QTimer>
#include <cmath>

#include "robot_control_gui_humble/ui/joystick_widget.h"
#include "robot_control_gui_humble/ui/speed_dashboard.h"
#include "robot_control_gui_humble/ui/mjpeg_stream.h"
#include "robot_control_gui_humble/ros/robot_controller.h"

namespace rcj {

ControlPanel::ControlPanel(std::shared_ptr<RobotController> c, QWidget* parent)
    : QWidget(parent), controller_(std::move(c)) {
  setupUi();
}

void ControlPanel::setupUi() {
  auto* root = new QHBoxLayout(this);

  auto* joy_box = new QGroupBox(tr("双摇杆控制"), this);
  auto* joy_lay = new QVBoxLayout(joy_box);
  lin_joy_ = new JoystickWidget(joy_box);
  ang_joy_ = new JoystickWidget(joy_box);
  joy_lay->addWidget(lin_joy_);
  joy_lay->addWidget(ang_joy_);

  auto* mid_box = new QGroupBox(tr("速度仪表 / 急停"), this);
  auto* mid_lay = new QVBoxLayout(mid_box);
  speed_ = new SpeedDashboard(mid_box);
  auto* estop = new QPushButton(tr("⚠ 紧急停止"), mid_box);
  estop->setMinimumHeight(48);
  mid_lay->addWidget(speed_);
  mid_lay->addWidget(estop);

  auto* cam_box = new QGroupBox(tr("摄像头"), this);
  auto* cam_lay = new QVBoxLayout(cam_box);
  camera_ = new MjpegStream(cam_box);
  // 直接用车上的 ROS 压缩图话题(不再依赖 web_video_server)
  if (controller_) {
    connect(controller_.get(), &RobotController::cameraImageUpdated, this,
            [this](const QImage& img) { if (camera_) camera_->setImage(img); });
  }
  cam_lay->addWidget(camera_);

  root->addWidget(joy_box);
  root->addWidget(mid_box, 1);
  root->addWidget(cam_box, 1);

  // Left joystick: linear.x via Y, linear.y via X.
  connect(lin_joy_, &JoystickWidget::moved, this, &ControlPanel::onLinJoyMoved);
  // Right joystick: angular.z via X.
  connect(ang_joy_, &JoystickWidget::moved, this, &ControlPanel::onAngJoyMoved);

  // 急停必须走 emitCombined():直接调 emergencyStop() 的话,重发定时器
  // 下一拍又会把「最后那个非零速度」发出去,车停不下来。
  connect(estop, &QPushButton::clicked, this, [this]() {
    key_lin_x_ = key_lin_y_ = key_ang_z_ = 0.0;
    joy_lin_x_ = joy_lin_y_ = joy_ang_z_ = 0.0;
    controller_->emergencyStop();   // 立刻补一条 0
    emitCombined();                 // 状态归零 → 停掉重发
  });

  // 遥操作重发:10 Hz 重发最近一次合成速度,直到归零。
  stream_timer_ = new QTimer(this);
  stream_timer_->setInterval(100);
  connect(stream_timer_, &QTimer::timeout, this, [this]() {
    controller_->publishFullVelocity(last_lin_x_, last_lin_y_, last_ang_z_);
  });
}

void ControlPanel::onLinJoyMoved(double x, double y) {
  joy_lin_x_ = -y;   // forward = -y (push up to go forward)
  joy_lin_y_ = -x;   // strafe  = -x (push left → move left)
  joy_ang_z_ =  0.0;
  emitCombined();
}

void ControlPanel::onAngJoyMoved(double x, double /*y*/) {
  joy_lin_x_ = 0.0;
  joy_lin_y_ = 0.0;
  joy_ang_z_ = -x;
  emitCombined();
}

void ControlPanel::setKeyLinearX(double v) { key_lin_x_ = v; emitCombined(); }
void ControlPanel::setKeyLinearY(double v) { key_lin_y_ = v; emitCombined(); }
void ControlPanel::setKeyAngularZ(double v) { key_ang_z_ = v; emitCombined(); }

void ControlPanel::emitCombined() {
  if (!controller_) return;
  // Sum the keyboard + joystick contributions, capped at max_*
  const double lin_x = std::clamp(key_lin_x_ + joy_lin_x_, -1.0, 1.0) * max_lin_;
  const double lin_y = std::clamp(key_lin_y_ + joy_lin_y_, -1.0, 1.0) * max_lin_;
  const double ang_z = std::clamp(key_ang_z_ + joy_ang_z_, -1.0, 1.0) * max_ang_;

  last_lin_x_ = lin_x;
  last_lin_y_ = lin_y;
  last_ang_z_ = ang_z;

  controller_->publishFullVelocity(lin_x, lin_y, ang_z);

  // 非零 → 起重发;归零 → 停发(刚发出去的那条 0 就是停止指令)。
  if (stream_timer_) {
    const bool active = std::abs(lin_x) + std::abs(lin_y) + std::abs(ang_z) > 1e-6;
    if (active && !stream_timer_->isActive())       stream_timer_->start();
    else if (!active && stream_timer_->isActive())  stream_timer_->stop();
  }

  if (speed_) {
    speed_->setLinearSpeed(lin_x);
    speed_->setAngularSpeed(ang_z);
  }
}

}  // namespace rcj