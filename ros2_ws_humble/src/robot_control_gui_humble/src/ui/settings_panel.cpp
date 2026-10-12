// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ui/settings_panel.h"
#include "robot_control_gui_humble/ros/robot_controller.h"

#include <QFormLayout>
#include <QSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QDir>
#include "robot_control_gui_humble/ros/robot_profile.h"
#include <QLabel>
#include <QGroupBox>
#include <QSettings>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QMessageBox>
#include <QProcess>
#include <QTextStream>
#include <QTcpSocket>
#include <QTimer>
#include <QPair>
#include <QVector>

#include "robot_control_gui_humble/ros/topic_names.h"

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

// 一次同步的 TCP 端口探测。用它而不是 ping/ssh 这类外部程序,是因为
// GUI 跑在精简容器里,那些程序不一定存在(存在了失败也只发 errorOccurred)。
bool probeTcp(const QString& host, quint16 port, int timeout_ms, QString* why) {
  if (host.isEmpty()) {
    if (why) *why = QStringLiteral("IP 为空");
    return false;
  }
  QTcpSocket sock;
  sock.connectToHost(host, port);
  const bool ok = sock.waitForConnected(timeout_ms);
  if (ok) {
    sock.disconnectFromHost();
  } else if (why) {
    *why = sock.errorString();
  }
  return ok;
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

  ip_combo_ = new QComboBox(net_box);
  ip_combo_->setEditable(true);
  ip_combo_->addItem(localIpv4());
  ip_combo_->addItems({"10.132.156.106", "192.168.1.112", "192.168.149.1"});
  ip_combo_->setCurrentText(s.value("network/robot_ip", "192.168.149.1").toString());

  pi_host_edit_ = new QLineEdit(net_box);
  pi_host_edit_->setText(s.value("network/pi_host", "pi@192.168.149.1").toString());

  // ---- 机器人配置档:换车不用改代码 ----
  const QString robots_dir = qEnvironmentVariable("RCJ_ROBOTS_DIR", "/robots");
  profile_combo_ = new QComboBox(net_box);
  const QStringList profiles = RobotProfile::listProfiles(robots_dir);
  for (const QString& f : profiles) profile_combo_->addItem(f);
  if (profiles.isEmpty()) profile_combo_->addItem(tr("(没找到 robots/ 目录)")); 
  const QString curp = QSettings().value("robot/profile", "mentorpi.yaml").toString();
  const int pi = profile_combo_->findText(curp);
  if (pi >= 0) profile_combo_->setCurrentIndex(pi);
  profile_combo_->setToolTip(tr("话题名/坐标系/电池类型都按这个文件来。\n"
      "加新车:复制 robots/_template.yaml 改名改值,重启 GUI 就能选。"));
  net_lay->addRow(tr("机器人配置:"), profile_combo_);
  auto* prof_hint = new QLabel(tr("（换配置档后需重启 GUI 生效）"));
  prof_hint->setStyleSheet("color:#6c757d;font-size:11px;");
  net_lay->addRow(prof_hint);

  net_lay->addRow(tr("ROS_DOMAIN_ID:"), domain_id_);
  net_lay->addRow(tr("机器人 IP:"),     ip_combo_);
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
  connect(test_btn, &QPushButton::clicked, this, [this]() {
    onTestConnection(ip_combo_->currentText());
  });
  connect(ssh_btn, &QPushButton::clicked, this, [this]() {
    onTestSsh(pi_host_edit_->text());
  });
}

void SettingsPanel::setTestIpFields(const QString& robot_ip, const QString& pi_host) {
  if (ip_combo_)       ip_combo_->setCurrentText(robot_ip);
  if (pi_host_edit_)   pi_host_edit_->setText(pi_host);
}

QString SettingsPanel::statusText() const {
  return status_label_ ? status_label_->text() : QString();
}

void SettingsPanel::onSave() {
  QSettings s;
  s.setValue("ros/domain_id",      domain_id_->value());
  s.setValue("network/pi_host",    pi_host_edit_->text());
  // 保存的是「机器人 IP」这个框的值。以前这里误从 pi_host 里 section('@',-1)
  // 截出来,而 ip_combo 是个局部变量根本读不到 —— 于是无论你填什么 IP,存进去的
  // 永远是 SSH 主机那段。现改成读 ip_combo_(已提升为成员)。
  const QString ip = ip_combo_->currentText().trimmed();
  s.setValue("network/robot_ip",   ip.isEmpty() ? pi_host_edit_->text().section('@', -1) : ip);
  s.setValue("control/max_linear", max_lin_->value());
  s.setValue("control/max_angular",max_ang_->value());
  status_label_->setText(tr("已保存到 ~/.config/mentorpi/robot_control_gui_humble.conf"));
}

void SettingsPanel::onTestConnection(const QString& ip) {
  // 这个按钮测的是「ROS 能不能看见小车」,而不是「这个 IP 能不能 ping 通」。
  // 旧实现跑 `ping -c 2`,两个坑:
  //   ① 容器镜像里没有 ping,QProcess 启动失败只发 errorOccurred、不发 finished,
  //      状态就永远停在「正在 ping ...」(点多少次都一样);
  //   ② 就算 ping 通了也说明不了 DDS 发现正常 —— 跨城走 Discovery Server,不是组播。
  // 现改成用本机节点直接问 DDS:数一数小车那几个话题有没有发布者。
  if (!controller_ || !controller_->isInitialized()) {
    status_label_->setStyleSheet("color:#dc3545;");
    status_label_->setText(tr("✗ 还没连接:先点左上角「🔌 连接小车」"));
    return;
  }
  // 隔 300ms 复查一次、最多约 3 秒,而不是查一次就下结论 —— 刚连上时
  // DDS 发现还没走完(跨城要走 Discovery Server,更慢),一次查询必然是 0,
  // 直接报「看不到任何话题」会把人吓一跳。
  probe_ip_ = ip;
  probe_tries_ = 0;
  // 四个都选「小车发、GUI 订」的话题:只有当 GUI 真的订阅了,双方端点才会在
  // DDS 里匹配上,这个节点才能在图中看到对方的发布者。反方向(GUI 发、小车订,
  // 如 speed 指令)不行 —— 没发过之前压根没有匹配,查出来恒为 0(踩过:
  // 看门狗明明在跑,却报「速度指令没起」)。
  probe_topics_ = {
      {tr("里程计"), topics::odom(),                  false},
      {tr("雷达"),   topics::scan(),                  false},
      {tr("相机"),   topics::cameraCompressedImage(), false},
      {tr("电池"),   topics::batteryState(),          false},
  };
  probe_seen_.clear();
  if (!probe_timer_) {
    probe_timer_ = new QTimer(this);
    probe_timer_->setInterval(300);
    connect(probe_timer_, &QTimer::timeout, this, &SettingsPanel::tickConnectionProbe);
  }
  status_label_->setStyleSheet("color:#fd7e14;");
  status_label_->setText(tr("正在检查 ROS 发现 %1 …(最多 3 秒)").arg(ip));
  probe_timer_->start();
  tickConnectionProbe();   // 先立刻查一次:大部分情况第一次就齐了
}

void SettingsPanel::tickConnectionProbe() {
  constexpr int kMaxTries = 10;   // 10 x 300ms ≈ 3s
  ++probe_tries_;
  auto node = controller_ ? controller_->node() : nullptr;
  if (!node) { finishConnectionProbe(tr("✗ ROS 节点不可用"), false); return; }

  for (const auto& pr : probe_topics_) {
    if (probe_seen_.contains(pr.label)) continue;
    const size_t n = pr.want_subscriber
                         ? node->count_subscribers(pr.topic.toStdString())
                         : node->count_publishers(pr.topic.toStdString());
    if (n > 0) probe_seen_ << pr.label;
  }

  QStringList unseen;
  for (const auto& pr : probe_topics_)
    if (!probe_seen_.contains(pr.label)) unseen << pr.label;

  if (unseen.isEmpty()) {
    finishConnectionProbe(tr("✓ ROS 发现正常 %1/%2:%3")
                              .arg(probe_seen_.size()).arg(probe_topics_.size())
                              .arg(probe_seen_.join("、")), true);
    return;
  }
  if (probe_tries_ >= kMaxTries) {
    // 一个都没看到 → 再探一下 IP 的 22 端口,用来区分「网络不通」和「网络通但
    // DDS/服务没起来」。这是唯一会阻塞 UI 的地方(800ms),只在失败路径上跑。
    if (probe_seen_.isEmpty()) {
      QString why;
      const bool ip_up = probeTcp(probe_ip_, 22, 800, &why);
      finishConnectionProbe(
          tr("✗ 看不到小车任何话题 (0/%1) ·IP %2:%3 → %4")
              .arg(probe_topics_.size()).arg(probe_ip_)
              .arg(ip_up ? tr("22 端口通") : tr("不可达"))
              .arg(ip_up ? tr("网络通,多半是 Discovery Server 或小车端服务没起")
                         : tr("网络就不通,先查 ZeroTier / 网线 / IP")),
          false);
    } else {
      finishConnectionProbe(tr("△ 只发现 %1/%2:%3 ·未发现:%4(该功能可能没起)")
                                .arg(probe_seen_.size()).arg(probe_topics_.size())
                                .arg(probe_seen_.join("、")).arg(unseen.join("、")),
                            true);
    }
  }
  // 还没到次数上限:什么都不做,等下一个 timeout 再查
}

void SettingsPanel::finishConnectionProbe(const QString& msg, bool ok) {
  if (probe_timer_) probe_timer_->stop();
  status_label_->setStyleSheet(ok ? "color:#198754;" : "color:#dc3545;");
  status_label_->setText(msg);
}

void SettingsPanel::onTestSsh(const QString& host) {
  // 和上面同一类坑:容器里没装 ssh 客户端时 QProcess 同样只发 errorOccurred。
  // 这里把这条分支补上,并且退化成一次端口探测,至少能回答「22 通不通」。
  status_label_->setText(tr("ssh %1 ...").arg(host));
  QProcess* p = new QProcess(this);
  connect(p, &QProcess::errorOccurred, this,
          [this, p, host](QProcess::ProcessError e) {
    if (e != QProcess::FailedToStart) return;  // 其它错误交给 finished 分支
    const QString ip = host.section('@', -1);
    QString why;
    const bool up = probeTcp(ip, 22, 1200, &why);
    status_label_->setStyleSheet(up ? "color:#198754;" : "color:#dc3545;");
    status_label_->setText(
        up ? tr("容器里没有 ssh 客户端,但 %1:22 可达(SSH 服务在线)").arg(ip)
           : tr("容器里没有 ssh 客户端,%1:22 也不可达(%2)").arg(ip, why));
    p->deleteLater();
  });
  connect(p, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, [this, p, host](int code, QProcess::ExitStatus) {
    if (code == 0) {
      status_label_->setStyleSheet("color:#198754;");
      status_label_->setText(tr("✓ SSH %1 OK").arg(host));
    } else {
      status_label_->setStyleSheet("color:#dc3545;");
      status_label_->setText(tr("✗ SSH %1 失败 (exit %2, 是否已 ssh-copy-id?)").arg(host).arg(code));
    }
    p->deleteLater();
  });
  p->start("ssh", {"-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
                   host, "true"});
}

}  // namespace rcj
