// SPDX-License-Identifier: MIT
#include "robot_control_gui_jazzy/ui/teleop_panel.h"
#include "robot_control_gui_jazzy/ros/robot_controller.h"

#include <QGridLayout>
#include <QPushButton>
#include <QAbstractButton>

namespace rcj {

TeleopPanel::TeleopPanel(std::shared_ptr<RobotController> c, QWidget* parent)
    : QWidget(parent), controller_(std::move(c)) {
  auto* g = new QGridLayout(this);
  auto* fwd = new QPushButton(tr("↑"), this);
  auto* back= new QPushButton(tr("↓"), this);
  auto* lft = new QPushButton(tr("←"), this);
  auto* rgt = new QPushButton(tr("→"), this);
  auto* stp = new QPushButton(tr("停"), this);

  g->addWidget(fwd, 0, 1);
  g->addWidget(lft, 1, 0);
  g->addWidget(stp, 1, 1);
  g->addWidget(rgt, 1, 2);
  g->addWidget(back,2, 1);

  auto press = [this](double v, double w) {
    return [this, v, w]() { controller_->publishVelocity(v, w); };
  };
  connect(fwd, &QAbstractButton::pressed, controller_.get(), press( 0.3,  0.0));
  connect(back,&QAbstractButton::pressed, controller_.get(), press(-0.3,  0.0));
  connect(lft, &QAbstractButton::pressed, controller_.get(), press( 0.0,  0.6));
  connect(rgt, &QAbstractButton::pressed, controller_.get(), press( 0.0, -0.6));
  for (auto* b : {fwd, back, lft, rgt}) {
    connect(b, &QAbstractButton::released, controller_.get(),
            &RobotController::emergencyStop);
  }
  connect(stp, &QPushButton::clicked, controller_.get(),
          &RobotController::emergencyStop);
}

}  // namespace rcj
