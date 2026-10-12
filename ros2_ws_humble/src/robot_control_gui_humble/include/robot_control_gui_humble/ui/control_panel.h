#pragma once
#include <QWidget>
#include <QTimer>
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

  // 遥操作重发。原来每一路输入都只在「事件发生的那一刻」发一条 cmd_vel
  // (键盘按下/松开各一条,摇杆拖动时才有),跨城链路 RTT 数百毫秒还会丢包,
  // 于是「松开」那一条 0 一旦迟到或丢失,小车就按最后速度一直跑 —— 实测出现过。
  // 改成:速度非零时以 10 Hz 重发,归零立刻停发。
  // 不能常驻发零:Nav2 导航时发 /cmd_vel,我们发 controller/cmd_vel,
  // odom_publisher 两条都订阅,常驻的 0 会把导航指令打断。
  QTimer* stream_timer_{nullptr};
  double last_lin_x_{0.0};
  double last_lin_y_{0.0};
  double last_ang_z_{0.0};
};

}  // namespace rcj