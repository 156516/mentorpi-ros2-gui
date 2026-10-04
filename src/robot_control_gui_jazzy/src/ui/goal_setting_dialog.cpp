// SPDX-License-Identifier: MIT
#include "robot_control_gui_jazzy/ui/goal_setting_dialog.h"

#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>

namespace rcj {
GoalSettingDialog::GoalSettingDialog(QWidget* parent) : QDialog(parent) {
  setWindowTitle(tr("目标点"));
  auto* form = new QFormLayout(this);
  auto* x = new QDoubleSpinBox(this);
  auto* y = new QDoubleSpinBox(this);
  auto* yaw = new QDoubleSpinBox(this);
  x->setRange(-50, 50); y->setRange(-50, 50); yaw->setRange(-3.14, 3.14);
  yaw->setSingleStep(0.1);
  form->addRow(tr("X"),   x);
  form->addRow(tr("Y"),   y);
  form->addRow(tr("Yaw"), yaw);
  form->addRow(new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this));
  // Stub: not wired to result yet
}
double GoalSettingDialog::x()   const { return 0.0; }
double GoalSettingDialog::y()   const { return 0.0; }
double GoalSettingDialog::yaw() const { return 0.0; }
}  // namespace rcj
