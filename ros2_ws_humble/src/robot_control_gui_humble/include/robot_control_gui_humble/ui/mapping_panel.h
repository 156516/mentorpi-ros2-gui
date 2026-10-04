#pragma once

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>

class QComboBox;
class QPushButton;
class QLineEdit;
class QLabel;

namespace rcj {

class SlamServiceClient;
class SlamParamsPanel;
class RobotController;

class MappingPanel : public QWidget
{
  Q_OBJECT
public:
  explicit MappingPanel(std::shared_ptr<RobotController> c, QWidget* parent = nullptr);
  ~MappingPanel() override;

private:
  std::shared_ptr<RobotController> controller_;
  std::unique_ptr<SlamServiceClient> client_;

  QComboBox*  method_combo_{nullptr};
  QPushButton* start_btn_{nullptr};
  QPushButton* stop_btn_{nullptr};
  QPushButton* save_btn_{nullptr};
  QLineEdit*  save_path_edit_{nullptr};
  QLabel*     status_label_{nullptr};
  QLabel*     method_desc_{nullptr};
  SlamParamsPanel* params_panel_{nullptr};
};

}  // namespace rcj