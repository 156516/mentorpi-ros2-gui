#pragma once
#include <QWidget>
#include <memory>

class QSpinBox;
class QLineEdit;
class QPushButton;
class QDoubleSpinBox;
class QLabel;

namespace rcj {
class RobotController;

class SettingsPanel : public QWidget {
  Q_OBJECT
public:
  explicit SettingsPanel(std::shared_ptr<RobotController> c, QWidget* parent = nullptr);

public slots:
  void onSave();
  void onTestConnection(const QString& ip);
  void onTestSsh(const QString& host);

private:
  std::shared_ptr<RobotController> controller_;
  QSpinBox*      domain_id_{nullptr};
  QLineEdit*     pi_host_edit_{nullptr};
  QDoubleSpinBox* max_lin_{nullptr};
  QDoubleSpinBox* max_ang_{nullptr};
  QPushButton*   save_btn_{nullptr};
  QLabel*        status_label_{nullptr};
};

}  // namespace rcj
