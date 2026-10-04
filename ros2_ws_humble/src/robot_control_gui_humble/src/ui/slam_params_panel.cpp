// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ui/slam_params_panel.h"

#include "robot_control_gui_humble/ros/robot_controller.h"
#include "robot_control_gui_humble/ros/slam_param_client.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSettings>
#include <QVBoxLayout>

#include <set>
#include <string>
#include <vector>

namespace rcj {

namespace {
// Params that slam_toolbox only reads at configure time -> restart required.
const std::set<std::string> kLaunchTimeOnly = {
  "map_update_interval", "resolution", "max_laser_range", "transform_publish_period"
};
}  // namespace

SlamParamsPanel::SlamParamsPanel(std::shared_ptr<RobotController> c, QWidget* parent)
    : QWidget(parent), controller_(std::move(c)) {
  buildUi();
}

SlamParamsPanel::~SlamParamsPanel() = default;

void SlamParamsPanel::buildUi() {
  auto* lay = new QVBoxLayout(this);
  lay->setContentsMargins(0, 0, 0, 0);

  auto* form = new QFormLayout;
  auto mk = [this](double lo, double hi, double step, int dec, double def) {
    auto* s = new QDoubleSpinBox(this);
    s->setRange(lo, hi);
    s->setSingleStep(step);
    s->setDecimals(dec);
    s->setValue(def);
    return s;
  };
  QSettings qs;
  spin_interval_   = mk(0.1, 30.0, 0.5, 1, qs.value("slam/map_update_interval", 5.0).toDouble());
  spin_resolution_ = mk(0.010, 0.25, 0.005, 3, qs.value("slam/resolution", 0.05).toDouble());
  spin_laser_range_= mk(0.5, 30.0, 0.5, 1, qs.value("slam/max_laser_range", 20.0).toDouble());
  spin_tf_period_  = mk(0.001, 1.0, 0.01, 3, qs.value("slam/transform_publish_period", 0.02).toDouble());
  spin_min_travel_ = mk(0.0, 2.0, 0.05, 2, qs.value("slam/minimum_travel_distance", 0.5).toDouble());

  spin_interval_->setToolTip(tr(
      "地图刷新间隔(秒):slam_toolbox 多久把地图重算一遍发到 /map。\n"
      "调小=地图更跟手,但小车 CPU 更忙。默认 5.0;1.0~2.0 已明显更流畅。"));
  spin_resolution_->setToolTip(tr(
      "分辨率(米/格):地图每个格子代表多少米。\n"
      "调小=更精细但地图更大、更费 CPU。默认 0.05(5cm)。改它要重启。"));
  spin_laser_range_->setToolTip(tr(
      "雷达最大距离(米):超过这个距离的点当作无效、不参与建图。\n"
      "调小可滤掉远处噪声/玻璃反射,调大能覆盖更大范围。默认 20.0。"));
  spin_tf_period_->setToolTip(tr(
      "位姿发布周期(秒):map→odom 坐标变换多久发一次。\n"
      "调小=小车在地图里的位置更新更平滑(代价是消息更多)。默认 0.02(=50Hz)。"));
  spin_min_travel_->setToolTip(tr(
      "最小移动距离(米):小车至少移动这么远,才处理一帧新雷达。\n"
      "用来过滤原地抖动;调大省 CPU 但会丢细节,调小更灵敏。默认 0.5。"));

  form->addRow(tr("map_update_interval (s) *"), spin_interval_);
  form->addRow(tr("resolution (m) *"),          spin_resolution_);
  form->addRow(tr("max_laser_range (m) *"),     spin_laser_range_);
  form->addRow(tr("transform_publish_period (s) *"), spin_tf_period_);
  form->addRow(tr("minimum_travel_distance (m)"),    spin_min_travel_);
  lay->addLayout(form);

  auto* desc = new QLabel(tr(
      "各参数作用(鼠标悬停也有提示):\n"
      "· map_update_interval —— 地图刷新间隔。越小地图越跟手,越费小车 CPU\n"
      "· resolution —— 分辨率。每格多少米,越小越精细、地图文件越大\n"
      "· max_laser_range —— 雷达有效距离。超出此距离的点不参与建图\n"
      "· transform_publish_period —— map→odom 发送周期。越小位置更新越平滑\n"
      "· minimum_travel_distance —— 最小移动距离。车走够这么远才处理新一帧"), this);
  desc->setWordWrap(true);
  desc->setStyleSheet("color:#495057;font-size:11px;");
  lay->addWidget(desc);

  auto* hint = new QLabel(tr(
      "* 标记的参数 slam_toolbox 只在启动时读取,必须用「应用并重启 SLAM」;"
      "未标记的可「应用(动态)」即时生效。"), this);
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#6c757d;");
  lay->addWidget(hint);

  auto* row = new QHBoxLayout;
  read_btn_    = new QPushButton(tr("读取当前值"), this);
  apply_btn_   = new QPushButton(tr("应用(动态)"), this);
  restart_btn_ = new QPushButton(tr("应用并重启 SLAM"), this);
  row->addWidget(read_btn_);
  row->addWidget(apply_btn_);
  row->addWidget(restart_btn_);
  lay->addLayout(row);

  status_ = new QLabel(tr("—"), this);
  status_->setWordWrap(true);
  lay->addWidget(status_);

  connect(read_btn_,    &QPushButton::clicked, this, [this]() { readValues(); });
  connect(apply_btn_,   &QPushButton::clicked, this, [this]() { applyDynamic(); });
  connect(restart_btn_, &QPushButton::clicked, this, [this]() { applyRestart(); });
}

std::map<std::string, double> SlamParamsPanel::currentValues() const {
  return {
    {"map_update_interval",     spin_interval_->value()},
    {"resolution",              spin_resolution_->value()},
    {"max_laser_range",         spin_laser_range_->value()},
    {"transform_publish_period",spin_tf_period_->value()},
    {"minimum_travel_distance", spin_min_travel_->value()},
  };
}

void SlamParamsPanel::readValues() {
  auto node = controller_ ? controller_->node() : nullptr;
  if (!node) { status_->setText(tr("未连接 ROS")); return; }
  if (!client_) client_ = std::make_unique<SlamParamClient>(node);

  std::vector<std::string> names;
  std::map<std::string, double> out;
  for (const auto& kv : currentValues()) names.push_back(kv.first);
  std::string err;
  if (!client_->getParams(names, out, &err)) {
    status_->setText(tr("读取失败:%1").arg(QString::fromStdString(err)));
    return;
  }
  auto put = [&](const char* k, QDoubleSpinBox* s) {
    auto it = out.find(k);
    if (it != out.end()) s->setValue(it->second);
  };
  put("map_update_interval", spin_interval_);
  put("resolution", spin_resolution_);
  put("max_laser_range", spin_laser_range_);
  put("transform_publish_period", spin_tf_period_);
  put("minimum_travel_distance", spin_min_travel_);
  status_->setText(tr("已读取 %1 个参数").arg(static_cast<int>(out.size())));
}

void SlamParamsPanel::applyDynamic() {
  auto node = controller_ ? controller_->node() : nullptr;
  if (!node) { status_->setText(tr("未连接 ROS")); return; }
  if (!client_) client_ = std::make_unique<SlamParamClient>(node);

  QStringList failed, ok;
  const auto vals = currentValues();
  std::map<std::string, bool> per_ok;
  std::string err;
  const bool rpc_ok = client_->setParams(vals, per_ok, &err);
  if (!rpc_ok && per_ok.empty()) {
    status_->setText(tr("应用失败:%1").arg(QString::fromStdString(err)));
    return;
  }
  for (const auto& kv : vals) {
    const bool good = per_ok.count(kv.first) ? per_ok[kv.first] : false;
    (good ? ok : failed) << QString::fromStdString(kv.first);
  }
  QString msg = tr("RPC 成功=%1;接受:%2").arg(rpc_ok ? "是" : "否").arg(ok.join(", "));
  if (!failed.isEmpty()) msg += tr(";被拒:%1").arg(failed.join(", "));
  msg += tr("\n注意:带 * 的参数即使被接受,也要重启 SLAM 才会生效。");
  status_->setText(msg);
}

void SlamParamsPanel::applyRestart() {
  auto node = controller_ ? controller_->node() : nullptr;
  if (!node) { status_->setText(tr("未连接 ROS")); return; }
  if (!client_) client_ = std::make_unique<SlamParamClient>(node);

  QSettings qs;
  for (const auto& kv : currentValues())
    qs.setValue(("slam/" + kv.first).c_str(), kv.second);

  std::string err;
  const bool ok = client_->applyAndRestart(currentValues(), &err);
  status_->setText(ok ? tr("已下发并重启:%1").arg(QString::fromStdString(err))
                      : tr("失败:%1").arg(QString::fromStdString(err)));
}

}  // namespace rcj
