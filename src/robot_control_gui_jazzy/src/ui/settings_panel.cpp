// SPDX-License-Identifier: MIT
#include "robot_control_gui_jazzy/ui/settings_panel.h"
#include "robot_control_gui_jazzy/ros/robot_controller.h"

#include <QFormLayout>
#include <QSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QGroupBox>
#include <QSettings>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QMessageBox>
#include <QProcess>
#include <QTextStream>

namespace rcj {

namespace {
QString localIpv4() {
  for (const auto& iface : QNetworkInterface::allInterfaces()) {
    if (!(iface.flags() & QNetworkInterface::IsUp)) continue;
    if (!(iface.flags() & QNetworkInterface::IsRunning)) continue;
    if (iface.flags() & QNetworkInterface::IsLoopBack) continue;
    for (const auto& entry : iface.addressEntries()) {
      const auto ip = entry.ip();
      if (ip.protocol() == QAbstractSocket::IPv4Protocol &&
          !ip.isInSubnet(QHostAddress("169.254.0.0"), 16)) {
        return ip.toString();
      }
    }
  }
  return QStringLiteral("127.0.0.1");
}
}  // namespace

SettingsPanel::SettingsPanel(std::shared_ptr<RobotController> c, QWidget* parent)
    : QWidget(parent), controller_(std::move(c)) {
  QSettings s;

  auto* root = new QFormLayout(this);

  // -- Network --
  auto* net_box = new QGroupBox(tr("网络"), this);
  auto* net_lay = new QFormLayout(net_box);

  domain_id_ = new QSpinBox(net_box);
  domain_id_->setRange(0, 232);
  // Default to 0 because mentorpi's bringup runs on ROS_DOMAIN_ID=0.
  // If you've set ROS_DOMAIN_ID in your shell, that value takes precedence
  // on first launch; thereafter the saved value sticks.
  domain_id_->setValue(s.value("ros/domain_id",
                                qEnvironmentVariableIntValue("ROS_DOMAIN_ID", 0)).toInt());

  auto* ip_combo = new QComboBox(net_box);
  ip_combo->setEditable(true);
  ip_combo->addItem(localIpv4());
  ip_combo->addItems({"192.168.149.1", "192.168.1.100", "10.0.0.100"});
  ip_combo->setCurrentText(s.value("network/robot_ip", "192.168.149.1").toString());

  pi_host_edit_ = new QLineEdit(net_box);
  pi_host_edit_->setText(s.value("network/pi_host", "pi@192.168.149.1").toString());

  net_lay->addRow(tr("ROS_DOMAIN_ID:"), domain_id_);
  net_lay->addRow(tr("机器人 IP:"),     ip_combo);
  net_lay->addRow(tr("SSH 用户@主机:"), pi_host_edit_);

  auto* hint = new QLabel(tr("(DOMAIN_ID 修改需重启 GUI 生效)"), net_box);
  net_lay->addRow(hint);

  auto* test_btn = new QPushButton(tr("测试 ROS 连接"), net_box);
  auto* ssh_btn  = new QPushButton(tr("测试 SSH"),       net_box);
  auto* btn_row  = new QHBoxLayout;
  btn_row->addWidget(test_btn);
  btn_row->addWidget(ssh_btn);
  net_lay->addRow(btn_row);

  root->addWidget(net_box);

  // -- Speed limits --
  auto* speed_box = new QGroupBox(tr("速度上限"), this);
  auto* speed_lay = new QFormLayout(speed_box);
  max_lin_ = new QDoubleSpinBox(speed_box);  max_lin_->setRange(0.05, 2.0);  max_lin_->setSingleStep(0.05);
  max_lin_->setValue(s.value("control/max_linear", 0.5).toDouble());
  max_ang_ = new QDoubleSpinBox(speed_box);  max_ang_->setRange(0.1, 5.0);   max_ang_->setSingleStep(0.1);
  max_ang_->setValue(s.value("control/max_angular", 1.0).toDouble());
  speed_lay->addRow(tr("线速度 (m/s)"),  max_lin_);
  speed_lay->addRow(tr("角速度 (rad/s)"), max_ang_);
  root->addWidget(speed_box);

  // -- Save button --
  save_btn_ = new QPushButton(tr("💾 保存"), this);
  root->addWidget(save_btn_);

  // -- Status --
  status_label_ = new QLabel(tr("就绪"), this);
  root->addWidget(status_label_);

  // Wire
  connect(save_btn_, &QPushButton::clicked, this, &SettingsPanel::onSave);
  connect(test_btn, &QPushButton::clicked, this, [this, ip_combo]() {
    onTestConnection(ip_combo->currentText());
  });
  connect(ssh_btn, &QPushButton::clicked, this, [this]() {
    onTestSsh(pi_host_edit_->text());
  });
}

void SettingsPanel::onSave() {
  QSettings s;
  s.setValue("ros/domain_id",      domain_id_->value());
  s.setValue("network/pi_host",    pi_host_edit_->text());
  s.setValue("network/robot_ip",   pi_host_edit_->text().section('@', -1));
  s.setValue("control/max_linear", max_lin_->value());
  s.setValue("control/max_angular",max_ang_->value());
  status_label_->setText(tr("已保存到 ~/.config/mentorpi/robot_control_gui_jazzy.conf"));
}

void SettingsPanel::onTestConnection(const QString& ip) {
  status_label_->setText(tr("正在 ping %1 ...").arg(ip));
  QProcess* p = new QProcess(this);
  p->start("ping", {"-c", "2", "-W", "1", ip});
  connect(p, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this, p, ip](int code, QProcess::ExitStatus) {
    if (code == 0) status_label_->setText(tr("✓ %1 可达").arg(ip));
    else            status_label_->setText(tr("✗ %1 不可达 (ping exit %2)").arg(ip).arg(code));
    p->deleteLater();
  });
}

void SettingsPanel::onTestSsh(const QString& host) {
  status_label_->setText(tr("ssh %1 ...").arg(host));
  QProcess* p = new QProcess(this);
  p->start("ssh", {"-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
                   host, "true"});
  connect(p, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this, p, host](int code, QProcess::ExitStatus) {
    if (code == 0) status_label_->setText(tr("✓ SSH %1 OK").arg(host));
    else            status_label_->setText(tr("✗ SSH %1 失败 (exit %2, 是否已 ssh-copy-id?)").arg(host).arg(code));
    p->deleteLater();
  });
}

}  // namespace rcj
