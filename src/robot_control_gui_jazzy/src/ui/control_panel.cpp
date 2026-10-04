// SPDX-License-Identifier: MIT
#include "robot_control_gui_jazzy/ui/control_panel.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QGroupBox>

#include "robot_control_gui_jazzy/ui/joystick_widget.h"
#include "robot_control_gui_jazzy/ui/speed_dashboard.h"
#include "robot_control_gui_jazzy/ui/mjpeg_stream.h"
#include "robot_control_gui_jazzy/ros/robot_controller.h"

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

  auto* cam_box = new QGroupBox(tr("摄像头 (MJPEG)"), this);
  auto* cam_lay = new QVBoxLayout(cam_box);
  camera_ = new MjpegStream(cam_box);
  camera_->setUrl(QStringLiteral("http://192.168.149.1:8080/stream?topic=/depth_cam/rgb/image_raw"));
  cam_lay->addWidget(camera_);

  root->addWidget(joy_box);
  root->addWidget(mid_box, 1);
  root->addWidget(cam_box, 1);

  // Left joystick: linear.x via Y, linear.y via X.
  connect(lin_joy_, &JoystickWidget::moved, this, &ControlPanel::onLinJoyMoved);
  // Right joystick: angular.z via X.
  connect(ang_joy_, &JoystickWidget::moved, this, &ControlPanel::onAngJoyMoved);

  connect(estop, &QPushButton::clicked, controller_.get(),
          &RobotController::emergencyStop);
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

  controller_->publishFullVelocity(lin_x, lin_y, ang_z);
  if (speed_) {
    speed_->setLinearSpeed(lin_x);
    speed_->setAngularSpeed(ang_z);
  }
}

}  // namespace rcj