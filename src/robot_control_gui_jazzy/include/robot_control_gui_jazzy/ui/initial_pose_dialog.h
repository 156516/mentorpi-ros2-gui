#pragma once
#include <QDialog>

namespace rcj {
class InitialPoseDialog : public QDialog {
  Q_OBJECT
public:
  explicit InitialPoseDialog(QWidget* parent = nullptr);

  double x() const;
  double y() const;
  double yaw() const;
};
}  // namespace rcj
