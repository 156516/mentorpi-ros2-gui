#pragma once
#include <QWidget>
#include <memory>

namespace rcj {
class RobotController;
class JoystickWidget;
class SpeedDashboard;
class MjpegStream;

class ControlPanel : public QWidget {
  Q_OBJECT
public:
  explicit ControlPanel(std::shared_ptr<RobotController> c, QWidget* parent = nullptr);

public slots:
  // Keyboard-driven slots. Each axis is independent so joystick and keyboard
  // contributions coexist (joystick "moved(0,0)" releases only its axes).
  void setKeyLinearX(double v);
  void setKeyLinearY(double v);
  void setKeyAngularZ(double v);

private slots:
  // Joystick slots
  void onLinJoyMoved(double x, double y);
  void onAngJoyMoved(double x, double y);
  void emitCombined();

private:
  void setupUi();
  std::shared_ptr<RobotController> controller_;

  double key_lin_x_{0.0};
  double key_lin_y_{0.0};
  double key_ang_z_{0.0};
  double joy_lin_x_{0.0};
  double joy_lin_y_{0.0};
  double joy_ang_z_{0.0};

  JoystickWidget* lin_joy_{nullptr};
  JoystickWidget* ang_joy_{nullptr};
  SpeedDashboard* speed_{nullptr};
  MjpegStream*    camera_{nullptr};

  double max_lin_{0.5};    // m/s — comfortable indoor floor speed
  double max_ang_{1.0};    // rad/s — full turning rate
};

}  // namespace rcj