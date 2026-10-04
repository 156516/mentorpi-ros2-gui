#pragma once
#include <QDialog>

namespace rcj {
class PlannerSettingsDialog : public QDialog {
  Q_OBJECT
public:
  explicit PlannerSettingsDialog(QWidget* parent = nullptr);
};
}  // namespace rcj
