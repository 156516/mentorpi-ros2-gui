#pragma once
#include <QWidget>
#include <sensor_msgs/msg/battery_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
class QProgressBar;
class QLabel;

namespace rcj {
class RobotStatusPanel : public QWidget {
  Q_OBJECT
public:
  explicit RobotStatusPanel(QWidget* parent = nullptr);

public slots:
  void onBattery(const sensor_msgs::msg::BatteryState& b);
  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray& d);
  void onOdom(const nav_msgs::msg::Odometry& o);
  void updateBattery(float pct, float voltage);
  void updateWifi(int strength);

private:
  QProgressBar* battery_bar_{nullptr};
  QProgressBar* wifi_bar_{nullptr};
  QLabel*       status_{nullptr};
  QLabel*       pose_label_{nullptr};
  QLabel*       voltage_label_{nullptr};
  QLabel*       current_label_{nullptr};
  QLabel*       temp_label_{nullptr};
  int           last_wifi_{0};
};

}  // namespace rcj
