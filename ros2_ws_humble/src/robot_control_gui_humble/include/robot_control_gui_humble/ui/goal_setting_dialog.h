#pragma once
#include <QDialog>

namespace rcj {
class GoalSettingDialog : public QDialog {
  Q_OBJECT
public:
  explicit GoalSettingDialog(QWidget* parent = nullptr);

  double x() const;
  double y() const;
  double yaw() const;
};
}  // namespace rcj
