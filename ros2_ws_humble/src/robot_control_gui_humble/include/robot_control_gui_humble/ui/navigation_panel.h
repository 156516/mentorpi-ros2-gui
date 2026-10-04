#pragma once
#include <QWidget>
#include <memory>

class QPushButton;
class QLabel;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;

namespace rcj {
class RobotController;
class RobotView;
class NavParamClient;

// 导航页 —— 精简版:只保留 状态 + 取消导航 + 操作提示。
// 设点不再用数字输入框,统一走地图交互(左键=目标点,Shift+左键=初始位姿),
// 避免和右侧视图的点击功能重复。
class NavigationPanel : public QWidget {
  Q_OBJECT
public:
  explicit NavigationPanel(std::shared_ptr<RobotController> c,
                           RobotView* view, QWidget* parent = nullptr);

  ~NavigationPanel() override;   // unique_ptr<NavParamClient> 需要

public slots:
  void onNavigationStateChanged(int state);
  void onNavigationDistanceRemaining(double remaining);

private:
  std::shared_ptr<RobotController> controller_;
  RobotView* view_{nullptr};
  QLabel*    state_label_{nullptr};
  QLabel*    dist_label_{nullptr};
  QLabel*    locate_label_{nullptr};

  // Nav2 算法/参数
  QComboBox*      planner_combo_{nullptr};
  QComboBox*      controller_combo_{nullptr};
  QLineEdit*      map_path_edit_{nullptr};
  QDoubleSpinBox* robot_radius_{nullptr};
  QDoubleSpinBox* inflation_radius_{nullptr};
  QPushButton*    upload_btn_{nullptr};
  QPushButton*    start_btn_{nullptr};
  QPushButton*    clear_btn_{nullptr};
  QPushButton*    read_btn_{nullptr};
  QPushButton*    apply_btn_{nullptr};
  QLabel*         np_status_{nullptr};
  std::unique_ptr<NavParamClient> nav_client_;
};

}  // namespace rcj
