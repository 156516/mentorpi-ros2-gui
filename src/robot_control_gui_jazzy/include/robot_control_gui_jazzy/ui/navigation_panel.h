#pragma once
#include <QWidget>
#include <memory>

class QLabel;
class QProgressBar;
class QDoubleSpinBox;

namespace rcj {
class RobotController;
class RobotView;

class NavigationPanel : public QWidget {
  Q_OBJECT
public:
  explicit NavigationPanel(std::shared_ptr<RobotController> c,
                           RobotView* view, QWidget* parent = nullptr);

public slots:
  void setInitialPose();
  void setGoal();
  void onNavigationStateChanged(int state);
  void onNavigationDistanceRemaining(double remaining);

private:
  std::shared_ptr<RobotController> controller_;
  RobotView* view_{nullptr};
  QLabel*      state_label_{nullptr};
  QLabel*      dist_label_{nullptr};
  QDoubleSpinBox* goal_x_{nullptr};
  QDoubleSpinBox* goal_y_{nullptr};
  QDoubleSpinBox* goal_yaw_{nullptr};
  QDoubleSpinBox* init_x_{nullptr};
  QDoubleSpinBox* init_y_{nullptr};
  QDoubleSpinBox* init_yaw_{nullptr};
};

}  // namespace rcj
