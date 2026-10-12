#pragma once
#include <QWidget>
#include <QVector>
#include <QPair>
#include <QString>
#include <QStringList>
#include <memory>

class QSpinBox;
class QComboBox;
class QLineEdit;
class QPushButton;
class QDoubleSpinBox;
class QLabel;
class QTimer;

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

public:
  // --- 回归自测用(见 main_window.cpp 的 RCJ_SELFTEST_RECONNECT)---
  // 点不了界面的环境里,靠这两个接口验证:
  //   · onSave 保存的是「机器人 IP」框还是 SSH 主机框(曾经的串写 bug)
  //   · onTestConnection 之后状态栏有没有卡在「正在 ping ...」
  void setTestIpFields(const QString& robot_ip, const QString& pi_host);
  QString statusText() const;

private:
  // 「测试 ROS 连接」的轮询状态:刚连上/DDS 还没发现完时,一次查询必然是 0,
  // 所以隔 300ms 复查一次,最多 ~3 秒,期间发现到哪个算哪个。
  void tickConnectionProbe();
  void finishConnectionProbe(const QString& msg, bool ok);

  std::shared_ptr<RobotController> controller_;
  QTimer*      probe_timer_{nullptr};
  QString      probe_ip_;
  // 探哪些话题、以及该往哪个方向探:
  //   小车→GUI 的话题(里程计/雷达/相机)要查「发布者在不在」;
  //   GUI→小车 的话题(速度指令)要查「订阅者在不在」——它的发布者就是 GUI 自己,
  //   查发布者永远是 0(踩过)。
  struct Probe {
    QString label;
    QString topic;
    bool want_subscriber{false};
  };
  QVector<Probe> probe_topics_;
  QStringList  probe_seen_;                          // 已经探到对端的那些
  int          probe_tries_{0};
  QComboBox*     profile_combo_{nullptr};   // 机器人配置档(robots/*.yaml)
  QComboBox*     ip_combo_{nullptr};        // 机器人 IP(保存/测试都要用,不能是局部变量)
  QSpinBox*      domain_id_{nullptr};
  QLineEdit*     pi_host_edit_{nullptr};
  QDoubleSpinBox* max_lin_{nullptr};
  QDoubleSpinBox* max_ang_{nullptr};
  QPushButton*   save_btn_{nullptr};
  QLabel*        status_label_{nullptr};
};

}  // namespace rcj
