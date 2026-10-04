// SPDX-License-Identifier: MIT
#include "robot_control_gui_jazzy/ui/navigation_panel.h"
#include "robot_control_gui_jazzy/ui/robot_view.h"
#include "robot_control_gui_jazzy/ros/robot_controller.h"

#include <cmath>

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QMessageBox>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rcj {

NavigationPanel::NavigationPanel(std::shared_ptr<RobotController> c,
                                 RobotView* view, QWidget* parent)
    : QWidget(parent), controller_(std::move(c)), view_(view) {
  auto* root = new QVBoxLayout(this);

  // -- Initial pose --
  auto* init_box = new QGroupBox(tr("初始位姿"), this);
  auto* init_form = new QFormLayout(init_box);
  init_x_   = new QDoubleSpinBox(init_box);  init_x_->setRange(-50, 50); init_x_->setValue(0);
  init_y_   = new QDoubleSpinBox(init_box);  init_y_->setRange(-50, 50); init_y_->setValue(0);
  init_yaw_ = new QDoubleSpinBox(init_box);  init_yaw_->setRange(-3.14, 3.14); init_yaw_->setSingleStep(0.1);
  auto* init_btn = new QPushButton(tr("设置初始位姿"), init_box);
  init_form->addRow(tr("X"),   init_x_);
  init_form->addRow(tr("Y"),   init_y_);
  init_form->addRow(tr("Yaw"), init_yaw_);
  init_form->addRow(init_btn);
  root->addWidget(init_box);

  // -- Goal --
  auto* goal_box = new QGroupBox(tr("导航目标"), this);
  auto* goal_form = new QFormLayout(goal_box);
  goal_x_   = new QDoubleSpinBox(goal_box); goal_x_->setRange(-50, 50); goal_x_->setValue(1);
  goal_y_   = new QDoubleSpinBox(goal_box); goal_y_->setRange(-50, 50); goal_y_->setValue(0);
  goal_yaw_ = new QDoubleSpinBox(goal_box); goal_yaw_->setRange(-3.14, 3.14); goal_yaw_->setSingleStep(0.1);
  auto* nav_btn  = new QPushButton(tr("开始导航"),   goal_box);
  auto* cnl_btn  = new QPushButton(tr("取消导航"),   goal_box);
  auto* map_goal_btn = new QPushButton(tr("从视图取点"), goal_box);
  goal_form->addRow(tr("X"),   goal_x_);
  goal_form->addRow(tr("Y"),   goal_y_);
  goal_form->addRow(tr("Yaw"), goal_yaw_);
  goal_form->addRow(map_goal_btn);
  goal_form->addRow(nav_btn, cnl_btn);
  root->addWidget(goal_box);

  // -- Status --
  auto* st_box = new QGroupBox(tr("状态"), this);
  auto* st_lay = new QFormLayout(st_box);
  state_label_ = new QLabel(tr("空闲"), st_box);
  dist_label_  = new QLabel(tr("-- m"), st_box);
  st_lay->addRow(tr("状态:"),     state_label_);
  st_lay->addRow(tr("剩余距离:"), dist_label_);
  root->addWidget(st_box);
  root->addStretch();

  // -- Wire up --
  connect(init_btn, &QPushButton::clicked, this, &NavigationPanel::setInitialPose);
  connect(nav_btn,  &QPushButton::clicked, this, &NavigationPanel::setGoal);
  connect(cnl_btn,  &QPushButton::clicked, controller_.get(), &RobotController::cancelNavigation);
  connect(map_goal_btn, &QPushButton::clicked, this, [this]() {
    if (view_) {
      QMessageBox::information(this, tr("提示"),
        tr("请直接在右侧视图上点击目标点（Shift+点击 = 设置初始位姿）"));
    }
  });

  // Connect controller signals
  connect(controller_.get(), &RobotController::navigationStateChanged,
          this, &NavigationPanel::onNavigationStateChanged);
  connect(controller_.get(), &RobotController::navigationDistanceRemaining,
          this, &NavigationPanel::onNavigationDistanceRemaining);
}

void NavigationPanel::setInitialPose() {
  geometry_msgs::msg::PoseWithCovarianceStamped p;
  p.header.frame_id = "map";
  p.header.stamp = rclcpp::Clock().now();
  p.pose.pose.position.x = init_x_->value();
  p.pose.pose.position.y = init_y_->value();
  const double yaw = init_yaw_->value();
  p.pose.pose.orientation.w = std::cos(yaw / 2.0);
  p.pose.pose.orientation.z = std::sin(yaw / 2.0);
  // 6x6 covariance: large for x,y,yaw; small for everything else
  for (int i = 0; i < 36; ++i) p.pose.covariance[i] = 0.0;
  p.pose.covariance[0]  = 0.25;
  p.pose.covariance[7]  = 0.25;
  p.pose.covariance[35] = 0.07;
  controller_->sendInitialPose(p);
  if (view_) view_->setInitialPose(init_x_->value(), init_y_->value(), yaw);
}

void NavigationPanel::setGoal() {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.header.stamp = rclcpp::Clock().now();
  p.pose.position.x = goal_x_->value();
  p.pose.position.y = goal_y_->value();
  const double yaw = goal_yaw_->value();
  p.pose.orientation.w = std::cos(yaw / 2.0);
  p.pose.orientation.z = std::sin(yaw / 2.0);
  controller_->sendNavigateToPose(p);
  if (view_) view_->setGoal(goal_x_->value(), goal_y_->value(), yaw);
}

void NavigationPanel::onNavigationStateChanged(int state) {
  using S = RobotController::NavigationState;
  switch (static_cast<S>(state)) {
    case S::IDLE:       state_label_->setText(tr("空闲"));     break;
    case S::ACTIVE:     state_label_->setText(tr("导航中…")); break;
    case S::PAUSED:     state_label_->setText(tr("已暂停"));   break;
    case S::SUCCEEDED:  state_label_->setText(tr("✓ 已到达")); break;
    case S::FAILED:     state_label_->setText(tr("✗ 失败"));   break;
    case S::CANCELLED:  state_label_->setText(tr("已取消"));   break;
    case S::STOPPED:    state_label_->setText(tr("已停止"));   break;
  }
}

void NavigationPanel::onNavigationDistanceRemaining(double d) {
  dist_label_->setText(QString::number(d, 'f', 2) + " m");
}

}  // namespace rcj
