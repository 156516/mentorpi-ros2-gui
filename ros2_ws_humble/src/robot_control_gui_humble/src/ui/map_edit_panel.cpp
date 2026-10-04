// SPDX-License-Identifier: MIT
//
// map_edit_panel.cpp — "地图编辑" tab.
//
// Flow: 冻结当前地图(或导入 PGM+YAML) → 用 RobotView 当画布擦除/补墙 → 另存。
// 编辑期间通过 editModeChanged(true) 让 MainWindow 锁住 /map,避免实时建图
// 刷新把编辑内容冲掉。
#include "robot_control_gui_humble/ui/map_edit_panel.h"
#include "robot_control_gui_humble/ros/robot_controller.h"
#include "robot_control_gui_humble/ros/topic_names.h"
#include "robot_control_gui_humble/ui/robot_view.h"

#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QRegularExpression>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace rcj {

MapEditPanel::MapEditPanel(std::shared_ptr<RobotController> c, RobotView* view, QWidget* parent)
    : QWidget(parent), controller_(std::move(c)), view_(view) {
  buildUi();
}

MapEditPanel::~MapEditPanel() = default;

void MapEditPanel::buildUi() {
  auto* root = new QVBoxLayout(this);

  // --- 来源 ---
  auto* src_box = new QGroupBox(tr("编辑来源"), this);
  auto* src_lay = new QVBoxLayout(src_box);
  auto* src_row = new QHBoxLayout;
  enter_btn_  = new QPushButton(tr("📌 编辑当前地图"), src_box);
  import_btn_ = new QPushButton(tr("📂 导入地图 (PGM+YAML)…"), src_box);
  src_row->addWidget(enter_btn_);
  src_row->addWidget(import_btn_);
  src_lay->addLayout(src_row);
  auto* src_hint = new QLabel(tr(
      "编辑前请先停止建图(SLAM 运行中不允许编辑)。导入需选 .yaml,同名 .pgm 会被自动读取。"), src_box);
  src_hint->setWordWrap(true);
  src_hint->setStyleSheet("color:#6c757d;");
  src_lay->addWidget(src_hint);
  root->addWidget(src_box);

  // --- 工具 ---
  auto* tool_box = new QGroupBox(tr("工具"), this);
  auto* tool_lay = new QVBoxLayout(tool_box);
  auto* tool_row = new QHBoxLayout;
  draw_radio_  = new QRadioButton(tr("补画墙 (占用)"), tool_box);
  erase_radio_ = new QRadioButton(tr("擦除 (空闲)"), tool_box);
  draw_radio_->setChecked(true);
  tool_row->addWidget(draw_radio_);
  tool_row->addWidget(erase_radio_);
  tool_lay->addLayout(tool_row);

  auto* brush_row = new QHBoxLayout;
  brush_row->addWidget(new QLabel(tr("笔刷半径 (m):"), tool_box));
  brush_spin_ = new QDoubleSpinBox(tool_box);
  brush_spin_->setRange(0.02, 2.0);
  brush_spin_->setSingleStep(0.05);
  brush_spin_->setDecimals(2);
  brush_spin_->setValue(0.15);
  brush_row->addWidget(brush_spin_);
  brush_row->addStretch();
  tool_lay->addLayout(brush_row);
  root->addWidget(tool_box);

  // --- 操作 ---
  auto* op_row = new QHBoxLayout;
  undo_btn_ = new QPushButton(tr("↶ 撤销"), this);
  redo_btn_ = new QPushButton(tr("↷ 重做"), this);
  save_btn_ = new QPushButton(tr("💾 另存为…"), this);
  auto* up_btn = new QPushButton(tr("📤 上传给小车用于导航"), this);
  up_btn->setToolTip(tr("把这张(编辑后的)图直接发给小车当导航地图,不用拷文件"));
  op_row->addWidget(up_btn);
  exit_btn_ = new QPushButton(tr("⏏ 退出编辑"), this);
  op_row->addWidget(undo_btn_);
  op_row->addWidget(redo_btn_);
  op_row->addWidget(save_btn_);
  op_row->addWidget(exit_btn_);
  root->addLayout(op_row);

  status_label_ = new QLabel(tr("未进入编辑模式"), this);
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);
  root->addStretch();

  // --- 交互 ---
  auto sync_tool = [this]() {
    // 左键画笔 = 当前工具;右键始终擦除(view 内部处理)
    if (view_) view_->setBrushValue(draw_radio_->isChecked() ? 100 : 0);
  };
  connect(draw_radio_,  &QRadioButton::toggled, this, sync_tool);
  connect(erase_radio_, &QRadioButton::toggled, this, sync_tool);

  connect(enter_btn_,  &QPushButton::clicked, this, [this]() { beginEditingFromCurrentMap(); });
  connect(import_btn_, &QPushButton::clicked, this, [this]() { importFromFile(); });
  connect(exit_btn_,   &QPushButton::clicked, this, [this]() { setEditing(false); });
  connect(undo_btn_,   &QPushButton::clicked, this, [this]() {
    if (buffer_.undo()) renderBuffer();
  });
  connect(redo_btn_,   &QPushButton::clicked, this, [this]() {
    if (buffer_.redo()) renderBuffer();
  });
  connect(save_btn_,   &QPushButton::clicked, this, [this]() { saveToFile(); });
  connect(up_btn, &QPushButton::clicked, this, [this]() {
    if (!controller_ || buffer_.empty()) { status_label_->setText(tr("没有地图可上传")); return; }
    std::string err;
    const bool ok = controller_->uploadNavMap(buffer_.grid(), &err);
    status_label_->setText(ok ? tr("✅ 已上传给小车;到导航页点\"应用并重启导航\"即可用")
                              : tr("❌ 上传失败:%1").arg(QString::fromStdString(err)));
  });

  if (view_) {
    connect(view_, &RobotView::editStrokeBegan, this, [this]() {
      buffer_.beginStroke();
    });
    connect(view_, &RobotView::editStroke, this, [this](double x, double y, int value) {
      buffer_.applyBrush(x, y, brush_spin_->value(), static_cast<int8_t>(value));
      renderBuffer();
    });
    connect(view_, &RobotView::editStrokeFinished, this, [this]() {
      buffer_.endStroke();
      undo_btn_->setEnabled(buffer_.canUndo());
      redo_btn_->setEnabled(buffer_.canRedo());
    });
  }

  setEditing(false);
}

bool MapEditPanel::queryIsMapping() {
  auto node = controller_ ? controller_->node() : nullptr;
  if (!node) return false;   // not connected -> nothing is mapping
  auto client = node->create_client<std_srvs::srv::Trigger>(
      topics::isMappingService().toStdString());
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    // Service absent (synthetic harness / slam_controller not running) =>
    // treat as "not mapping" so editing is allowed.
    return false;
  }
  auto future = client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) return false;
  return future.get()->success;
}

void MapEditPanel::beginEditingFromCurrentMap() {
  if (!controller_) { status_label_->setText(tr("未连接 ROS")); return; }
  if (queryIsMapping()) {
    QMessageBox::warning(this, tr("地图编辑"),
        tr("正在建图中,请先停止 SLAM 再编辑地图。"));
    return;
  }
  const auto grid = controller_->latestMap();
  if (grid.data.empty() || grid.info.width == 0) {
    QMessageBox::warning(this, tr("地图编辑"),
        tr("还没有地图可编辑(先建一张图,或改用\"导入地图\")。"));
    return;
  }
  buffer_.reset(grid);
  renderBuffer();
  setEditing(true);
  status_label_->setText(tr("已冻结当前地图:%1 x %2 @ %3 m")
      .arg(grid.info.width).arg(grid.info.height).arg(grid.info.resolution));
}

void MapEditPanel::importFromFile() {
  const QString yaml_path = QFileDialog::getOpenFileName(
      this, tr("选择地图 YAML"), QStringLiteral("/maps"),
      tr("Map YAML (*.yaml *.yml);;All files (*)"));
  if (yaml_path.isEmpty()) return;

  QFile yf(yaml_path);
  if (!yf.open(QIODevice::ReadOnly)) {
    QMessageBox::warning(this, tr("导入失败"), tr("打不开 %1").arg(yaml_path));
    return;
  }
  const std::string yaml_text = yf.readAll().toStdString();
  yf.close();

  std::string image_name = "map.pgm";
  {
    double res, ox, oy, oyaw, occ_t, free_t; std::string err;
    parseMapYaml(yaml_text, res, ox, oy, oyaw, occ_t, free_t, image_name, &err);
  }
  // image path is relative to the yaml
  const QFileInfo yfi(yaml_path);
  const QString pgm_path = yfi.dir().filePath(QString::fromStdString(image_name));
  QFile pf(pgm_path);
  if (!pf.open(QIODevice::ReadOnly)) {
    QMessageBox::warning(this, tr("导入失败"),
        tr("找不到地图图片:%1\n(YAML 里 image: %2)").arg(pgm_path, QString::fromStdString(image_name)));
    return;
  }
  const QByteArray raw = pf.readAll();
  pf.close();
  const std::vector<uint8_t> pgm(raw.begin(), raw.end());

  nav_msgs::msg::OccupancyGrid grid;
  std::string err;
  if (!loadPgmYaml(yaml_text, pgm, grid, &err)) {
    QMessageBox::warning(this, tr("导入失败"), QString::fromStdString(err));
    return;
  }
  buffer_.reset(grid);
  renderBuffer();
  setEditing(true);
  status_label_->setText(tr("已导入:%1 (%2 x %3)").arg(yaml_path)
      .arg(grid.info.width).arg(grid.info.height));
}

void MapEditPanel::renderBuffer() {
  if (!view_ || buffer_.empty()) return;
  const QImage img = RobotController::occupancyGridToImage(buffer_.grid());
  view_->setMapImage(img);
}

void MapEditPanel::saveToFile() {
  if (buffer_.empty()) {
    QMessageBox::warning(this, tr("保存"), tr("没有可保存的地图。"));
    return;
  }
  QString path = QFileDialog::getSaveFileName(
      this, tr("保存地图 (base name)"), QStringLiteral("/maps/my_map"),
      tr("Map base name (*);;PGM (*.pgm)"));
  if (path.isEmpty()) return;
  path.remove(QRegularExpression("\\.(pgm|yaml|yml)$", QRegularExpression::CaseInsensitiveOption));
  const QString pgm_path  = path + ".pgm";
  const QString yaml_path = path + ".yaml";
  const QString base = QFileInfo(path).fileName() + ".pgm";

  const auto pgm = gridToPgm(buffer_.grid());
  QFile pf(pgm_path);
  if (!pf.open(QIODevice::WriteOnly)) {
    QMessageBox::warning(this, tr("保存失败"), tr("写不了 %1").arg(pgm_path));
    return;
  }
  pf.write(reinterpret_cast<const char*>(pgm.data()), static_cast<qint64>(pgm.size()));
  pf.close();

  const std::string yaml = gridToYaml(buffer_.grid(), base.toStdString());
  QFile yf(yaml_path);
  if (!yf.open(QIODevice::WriteOnly)) {
    QMessageBox::warning(this, tr("保存失败"), tr("写不了 %1").arg(yaml_path));
    return;
  }
  yf.write(yaml.data(), static_cast<qint64>(yaml.size()));
  yf.close();

  status_label_->setText(tr("已保存:%1 + %2").arg(pgm_path, yaml_path));
}

void MapEditPanel::setEditing(bool on) {
  editing_ = on;
  if (view_) {
    view_->setEditMode(on);
    if (on) view_->setBrushValue(draw_radio_->isChecked() ? 100 : 0);
  }
  enter_btn_->setEnabled(!on);
  import_btn_->setEnabled(!on);
  exit_btn_->setEnabled(on);
  draw_radio_->setEnabled(on);
  erase_radio_->setEnabled(on);
  brush_spin_->setEnabled(on);
  save_btn_->setEnabled(on);
  undo_btn_->setEnabled(on && buffer_.canUndo());
  redo_btn_->setEnabled(on && buffer_.canRedo());
  if (!on) status_label_->setText(tr("未进入编辑模式"));
  emit editModeChanged(on);
}

}  // namespace rcj
