// SPDX-License-Identifier: MIT
//
// log_panel.cpp — 「日志」页:小车 /rosout + GUI 自身事件,可导出。
#include "robot_control_gui_humble/ui/log_panel.h"
#include "robot_control_gui_humble/ros/robot_controller.h"
#include "robot_control_gui_humble/ros/topic_names.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDateTimeEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QDebug>
#include <QTimer>
#include <cmath>
#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTextStream>
#include <QVBoxLayout>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdio>

namespace rcj {

namespace {
constexpr int kMaxRows = 3000;
LogPanel* g_panel = nullptr;
QtMessageHandler g_prev_handler = nullptr;

// 分类:靠节点名(来源)里的关键字归类
struct Cat { const char* label; const char* needles; };  // needles 逗号分隔
const Cat kCats[] = {
  { "全部",                 "" },
  { "定位 (AMCL)",          "amcl" },
  { "导航 (规划/控制/BT)",   "planner_server,controller_server,bt_navigator,behavior_server,waypoint,velocity_smoother,navigate" },
  { "建图 (SLAM)",          "slam_toolbox,slam_controller,mapping" },
  { "参数管理",             "nav_controller" },
  { "位姿 / 里程计",         "odom,ekf,robot_state_publisher,tf,map_server" },
  { "传感器 (雷达/相机/底盘)", "scan,lidar,MS200,ascamera,camera,ros_robot_controller,imu" },
  { "GUI 本程序",            "GUI" },
};
bool catMatch(const QString& src, const QString& needles) {
  if (needles.isEmpty()) return true;                 // 全部
  for (const QString& n : needles.split(',')) {
    if (!n.trimmed().isEmpty() && src.contains(n.trimmed(), Qt::CaseInsensitive)) return true;
  }
  return false;
}

const char* levelName(int lv) {
  if (lv >= 50) return "FATAL";
  if (lv >= 40) return "ERROR";
  if (lv >= 30) return "WARN";
  if (lv >= 20) return "INFO";
  return "DEBUG";
}
}  // namespace

// 把 GUI 自己的 qInfo/qWarning/... 也收进日志页
static void rcjQtMessageHandler(QtMsgType type, const QMessageLogContext& ctx, const QString& msg) {
  if (g_panel) {
    const int lv = (type == QtDebugMsg) ? 10 : (type == QtInfoMsg) ? 20
                 : (type == QtWarningMsg) ? 30 : 40;
    const QString file = ctx.file ? QString::fromUtf8(ctx.file) : QString("gui");
    QMetaObject::invokeMethod(g_panel, [lv, msg, file]() {
      if (g_panel) g_panel->addRow("GUI", lv, file.section('/', -1), msg);
    }, Qt::QueuedConnection);
  }
  if (g_prev_handler) g_prev_handler(type, ctx, msg);
}

LogPanel::LogPanel(std::shared_ptr<RobotController> c, QWidget* parent)
    : QWidget(parent), controller_(std::move(c)) {
  buildUi();
  g_panel = this;
  if (!g_prev_handler) g_prev_handler = qInstallMessageHandler(rcjQtMessageHandler);

  auto node = controller_ ? controller_->node() : nullptr;
  if (node) {
    rosout_sub_ = node->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", rclcpp::QoS(200),
        [this](const rcl_interfaces::msg::Log::SharedPtr m) {
          // executor 线程 -> GUI 线程
          const int lv = static_cast<int>(m->level);
          const QString node_name = QString::fromStdString(m->name);
          const QString text = QString::fromStdString(m->msg);
          QMetaObject::invokeMethod(this, [this, lv, node_name, text]() {
            addRow("ROS", lv, node_name, text);
          }, Qt::QueuedConnection);
        });
  }
  addRow("GUI", 20, "gui", tr("已连接,开始记录日志(小车 /rosout + GUI 事件)"));

  // 位姿:定时记一条(节点名写 odom,好归到「位姿 / 里程计」分类)
  auto* pose_timer = new QTimer(this);
  connect(pose_timer, &QTimer::timeout, this, [this]() {
    if (!controller_) return;
    const auto p = controller_->currentPose();
    const auto& q = p.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double mx = 0, my = 0, myaw = 0;
    if (controller_->tryGetRobotPoseInMap(mx, my, myaw)) {
      addRow("GUI", 20, "odom", tr("位姿 map(%1, %2, %3°) / odom(%4, %5, %6°)")
             .arg(mx, 0, 'f', 2).arg(my, 0, 'f', 2).arg(myaw * 180.0 / M_PI, 0, 'f', 0)
             .arg(p.position.x, 0, 'f', 2).arg(p.position.y, 0, 'f', 2)
             .arg(yaw * 180.0 / M_PI, 0, 'f', 0));
    } else {
      addRow("GUI", 20, "odom", tr("位姿 odom(%1, %2, %3°) —— 未定位")
             .arg(p.position.x, 0, 'f', 2).arg(p.position.y, 0, 'f', 2)
             .arg(yaw * 180.0 / M_PI, 0, 'f', 0));
    }
  });
  pose_timer->start(2000);
}

LogPanel::~LogPanel() {
  if (g_panel == this) { g_panel = nullptr; qInstallMessageHandler(g_prev_handler); }
}

void LogPanel::note(const QString& text) { qInfo().noquote() << text; }

void LogPanel::noteAs(const QString& node, const QString& text) {
  if (!g_panel) return;
  QMetaObject::invokeMethod(g_panel, [node, text]() {
    if (g_panel) g_panel->addRow("GUI", 20, node, text);
  }, Qt::QueuedConnection);
}

void LogPanel::buildUi() {
  auto* root = new QVBoxLayout(this);

  auto* top = new QHBoxLayout;
  top->addWidget(new QLabel(tr("分类:"), this));
  category_filter_ = new QComboBox(this);
  for (const auto& c : kCats) category_filter_->addItem(tr(c.label), QString::fromUtf8(c.needles));
  category_filter_->setToolTip(tr("按节点归类看日志:定位/导航/建图/参数/位姿/传感器/GUI"));
  top->addWidget(category_filter_);

  top->addWidget(new QLabel(tr("级别:"), this));
  level_filter_ = new QComboBox(this);
  level_filter_->addItem(tr("全部"), 0);
  level_filter_->addItem(tr("INFO 以上"), 20);
  level_filter_->addItem(tr("WARN 以上"), 30);
  level_filter_->addItem(tr("ERROR 以上"), 40);
  level_filter_->setCurrentIndex(1);      // 默认 INFO+,避免被 DEBUG 淹
  top->addWidget(level_filter_);
  autoscroll_ = new QCheckBox(tr("自动滚到底"), this);
  autoscroll_->setChecked(true);
  top->addWidget(autoscroll_);
  top->addStretch();
  count_label_ = new QLabel(tr("0 条"), this);
  count_label_->setStyleSheet("color:#6c757d;");
  top->addWidget(count_label_);
  auto* clear_btn = new QPushButton(tr("🗑 清空"), this);
  auto* export_btn = new QPushButton(tr("💾 导出日志…"), this);
  top->addWidget(clear_btn);
  top->addWidget(export_btn);
  root->addLayout(top);

  table_ = new QTableWidget(this);
  table_->setColumnCount(4);
  table_->setHorizontalHeaderLabels({tr("时间"), tr("级别"), tr("来源"), tr("内容")});
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->verticalHeader()->setVisible(false);
  root->addWidget(table_);

  auto* hint = new QLabel(tr(
      "「ROS」= 小车上的节点日志(/rosout);「GUI」= 本程序自己的事件。\n"
      "导出为 CSV,可直接用 Excel/WPS 打开。"), this);
  hint->setStyleSheet("color:#6c757d;font-size:11px;");
  hint->setWordWrap(true);
  root->addWidget(hint);

  connect(clear_btn, &QPushButton::clicked, this, [this]() { clearLog(); });
  connect(export_btn, &QPushButton::clicked, this, [this]() { exportLog(); });
  auto apply_filters = [this]() {
    const int minlv = level_filter_->currentData().toInt();
    const QString needles = category_filter_->currentData().toString();
    for (int r = 0; r < table_->rowCount(); ++r) {
      const int lv = table_->item(r, 1)->data(Qt::UserRole).toInt();
      const QString src = table_->item(r, 2)->text();
      table_->setRowHidden(r, lv < minlv || !catMatch(src, needles));
    }
  };
  connect(level_filter_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [apply_filters](int) { apply_filters(); });
  connect(category_filter_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [apply_filters](int) { apply_filters(); });
}

void LogPanel::addRow(const QString& source, int level, const QString& node, const QString& text) {
  if (table_->rowCount() >= kMaxRows) table_->removeRow(0);   // 环形上限
  const int r = table_->rowCount();
  table_->insertRow(r);
  const QDateTime now = QDateTime::currentDateTime();
  auto* t = new QTableWidgetItem(now.toString("HH:mm:ss.zzz"));
  t->setData(Qt::UserRole, now);
  auto* l = new QTableWidgetItem(QString::fromUtf8(levelName(level)));
  l->setData(Qt::UserRole, level);
  auto* s = new QTableWidgetItem(source + (node.isEmpty() ? QString() : " /" + node));
  auto* m = new QTableWidgetItem(text);
  if (level >= 40) { l->setForeground(Qt::red); m->setForeground(Qt::red); }
  else if (level >= 30) { l->setForeground(QColor(200,120,0)); }
  table_->setItem(r, 0, t);
  table_->setItem(r, 1, l);
  table_->setItem(r, 2, s);
  table_->setItem(r, 3, m);
  if (level < level_filter_->currentData().toInt() ||
      !catMatch(source + (node.isEmpty() ? QString() : " /" + node),
                category_filter_->currentData().toString()))
    table_->setRowHidden(r, true);
  rows_ = table_->rowCount();
  count_label_->setText(tr("%1 条").arg(rows_));
  if (autoscroll_->isChecked()) table_->scrollToBottom();
}

void LogPanel::clearLog() {
  table_->setRowCount(0);
  count_label_->setText(tr("0 条"));
}

void LogPanel::exportLog() {
  if (table_->rowCount() == 0) { QMessageBox::information(this, tr("导出日志"), tr("没有日志")); return; }

  // ---- 选时间范围 ----
  const QDateTime first = table_->item(0, 0)->data(Qt::UserRole).toDateTime();
  const QDateTime last  = table_->item(table_->rowCount() - 1, 0)->data(Qt::UserRole).toDateTime();
  QDialog dlg(this);
  dlg.setWindowTitle(tr("选择导出时间段"));
  auto* lay = new QVBoxLayout(&dlg);
  auto* form = new QFormLayout;
  auto* from_edit = new QDateTimeEdit(first, &dlg);
  auto* to_edit   = new QDateTimeEdit(last, &dlg);
  from_edit->setDisplayFormat("HH:mm:ss");
  to_edit->setDisplayFormat("HH:mm:ss");
  from_edit->setCalendarPopup(false);
  to_edit->setCalendarPopup(false);
  form->addRow(tr("从:"), from_edit);
  form->addRow(tr("到:"), to_edit);
  lay->addLayout(form);

  auto* quick = new QHBoxLayout;
  auto add_quick = [&](const QString& label, int secs) {
    auto* b = new QPushButton(label, &dlg);
    connect(b, &QPushButton::clicked, &dlg, [from_edit, to_edit, last, secs]() {
      from_edit->setDateTime(last.addSecs(-secs)); to_edit->setDateTime(last);
    });
    quick->addWidget(b);
  };
  add_quick(tr("全部"),        static_cast<int>(first.secsTo(last)) + 1);
  add_quick(tr("最近 1 分钟"), 60);
  add_quick(tr("最近 5 分钟"), 300);
  add_quick(tr("最近 1 小时"), 3600);
  quick->addStretch();
  lay->addLayout(quick);

  auto* bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
  connect(bb, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
  connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
  lay->addWidget(bb);
  if (dlg.exec() != QDialog::Accepted) return;
  const QDateTime t_from = from_edit->dateTime();
  const QDateTime t_to   = to_edit->dateTime();

  QString path = QFileDialog::getSaveFileName(this, tr("导出日志"),
      QStringLiteral("/maps/robot_log_%1.csv")
          .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")),
      tr("CSV (*.csv);;文本 (*.txt)"));
  if (path.isEmpty()) return;
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(this, tr("导出失败"), tr("写不了 %1").arg(path));
    return;
  }
  QTextStream os(&f);
  os.setCodec("UTF-8");
  os << "time,level,source,message\n";
  int written = 0;
  for (int r = 0; r < table_->rowCount(); ++r) {
    const QDateTime ts = table_->item(r, 0)->data(Qt::UserRole).toDateTime();
    if (!ts.isValid() || ts < t_from || ts > t_to) continue;   // 只导范围内
    ++written;
    auto esc = [](const QString& s) {
      QString x = s; x.replace('"', "\"\""); return "\"" + x + "\"";
    };
    os << esc(table_->item(r,0)->text()) << ","
       << esc(table_->item(r,1)->text()) << ","
       << esc(table_->item(r,2)->text()) << ","
       << esc(table_->item(r,3)->text()) << "\n";
  }
  f.close();
  QMessageBox::information(this, tr("导出成功"),
      tr("已导出 %1 行(%2 ~ %3)到:\n%4")
      .arg(written).arg(t_from.toString("HH:mm:ss")).arg(t_to.toString("HH:mm:ss")).arg(path));
}

}  // namespace rcj
