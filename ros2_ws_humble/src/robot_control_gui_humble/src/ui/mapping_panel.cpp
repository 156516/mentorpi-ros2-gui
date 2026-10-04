// SPDX-License-Identifier: MIT
//
// mapping_panel.cpp - UI for SLAM start/stop and map saving.
//
// P15: SLAM is started via ROS2 service call to a long-lived
// slam_controller node on the robot. No SSH is used from this GUI.
//
// We use a two-step protocol to avoid needing a custom .srv definition
// (which would not be cross-ROS-distro compatible):
//   1. Publish /slam_request_method (std_msgs/String) with the desired method
//   2. Call /start_slam (std_srvs/Trigger) - the controller remembers the
//      most recently published method and launches it.
#include "robot_control_gui_humble/ui/mapping_panel.h"
#include "robot_control_gui_humble/ros/robot_controller.h"
#include "robot_control_gui_humble/ros/topic_names.h"
#include "robot_control_gui_humble/ui/slam_params_panel.h"
#include "robot_control_gui_humble/ros/map_editor.h"

#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QFile>
#include <QFileDialog>
#include <QTimer>
#include <QFileInfo>
#include <QRegularExpression>
#include <QMessageBox>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <functional>
#include <string>

class SlamServiceClient;

namespace rcj {

// Service + topic client wrapping rclcpp.
class SlamServiceClient {
 public:
  explicit SlamServiceClient(std::shared_ptr<rclcpp::Node> node)
      : node_(std::move(node)) {}

  bool startSlam(const std::string& method, std::string* msg) {
    if (!method_pub_) {
      method_pub_ = node_->create_publisher<std_msgs::msg::String>(
          topics::slamRequestMethodTopic().toStdString(), 10);
    }
    if (!start_client_) {
      start_client_ = node_->create_client<std_srvs::srv::Trigger>(
          topics::startSlamService().toStdString());
    }
    if (!start_client_->wait_for_service(std::chrono::seconds(2))) {
      if (msg) *msg = "service /start_slam not available (is slam_controller running on the robot?)";
      return false;
    }

    std_msgs::msg::String msg_out;
    msg_out.data = method;
    method_pub_->publish(msg_out);
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    auto future = start_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    // Use wait_for() rather than spin_until_future_complete() because the
    // node is already being spun on the executor's own thread; spinning
    // it here would cause "already added to an executor" aborts.
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      if (msg) *msg = "service call timed out";
      return false;
    }
    const auto& resp = future.get();
    if (msg) *msg = resp->message;
    return resp->success;
  }

  bool isMapping(bool* running, std::string* msg) {
    if (!is_map_client_) {
      is_map_client_ = node_->create_client<std_srvs::srv::Trigger>(
          topics::isMappingService().toStdString());
    }
    if (!is_map_client_->wait_for_service(std::chrono::milliseconds(400))) {
      if (msg) *msg = "service /is_mapping not available";
      return false;
    }
    auto future = is_map_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) return false;
    const auto& r = future.get();
    if (running) *running = r->success;
    if (msg) *msg = r->message;
    return true;
  }

  // 非阻塞:服务已发现才发,结果经回调(在 executor 线程)返回。
  void isMappingAsync(std::function<void(bool)> cb) {
    if (!is_map_client_) {
      is_map_client_ = node_->create_client<std_srvs::srv::Trigger>(
          topics::isMappingService().toStdString());
    }
    if (!is_map_client_->service_is_ready()) return;   // 下一刻再试
    is_map_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [cb](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) { cb(f.get()->success); });
  }

  bool stopSlam(std::string* msg) {
    if (!stop_client_) {
      stop_client_ = node_->create_client<std_srvs::srv::Empty>(
          topics::stopSlamService().toStdString());
    }
    if (!stop_client_->wait_for_service(std::chrono::seconds(2))) {
      if (msg) *msg = "service /stop_slam not available";
      return false;
    }
    auto future = stop_client_->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>());
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      if (msg) *msg = "service call timed out";
      return false;
    }
    if (msg) *msg = "stopped";
    return true;
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr method_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr   stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr is_map_client_;
};

MappingPanel::MappingPanel(std::shared_ptr<RobotController> c, QWidget* parent)
    : QWidget(parent), controller_(std::move(c)) {
  auto* root = new QVBoxLayout(this);

  auto* method_box = new QGroupBox(tr("SLAM"), this);
  auto* method_lay = new QVBoxLayout(method_box);
  auto* method_row = new QHBoxLayout;
  method_combo_ = new QComboBox(method_box);
  // mentorpi 只有 slam_toolbox 可用(rtabmap 需 RGBD 相机),设为首项即默认值。
  method_combo_->addItems({"slam_toolbox", "rtabmap", "hector", "cartographer"});
  method_combo_->setItemData(0, tr("2D 激光 SLAM:只需激光雷达,输出 /map 栅格地图。\nmentorpi 用这个(唯一实测可用)。"), Qt::ToolTipRole);
  method_combo_->setItemData(1, tr("RGB-D 视觉 SLAM:需要深度相机。\nmentorpi 只有 2D 雷达 → 启动后收不到数据,建不出图。"), Qt::ToolTipRole);
  method_combo_->setItemData(2, tr("纯激光 SLAM:不依赖里程计,只靠激光匹配。\n适合平地快速移动;mentorpi 有里程计,用它精度不如 slam_toolbox,未验证。"), Qt::ToolTipRole);
  method_combo_->setItemData(3, tr("Google Cartographer:2D/3D SLAM,功能强但配置复杂。\n需要额外的 lua 配置文件,mentorpi 上未验证。"), Qt::ToolTipRole);
  method_row->addWidget(new QLabel(tr("Method:"), method_box));
  method_row->addWidget(method_combo_);
  method_lay->addLayout(method_row);

  method_desc_ = new QLabel(method_box);
  method_desc_->setWordWrap(true);
  method_desc_->setStyleSheet("color:#495057;font-size:11px;");
  auto update_method_desc = [this](int idx) {
    static const char* kDesc[] = {
      "slam_toolbox — 2D 激光 SLAM。只要激光雷达就能跑,输出栅格地图 /map。mentorpi 唯一实测可用,推荐。",
      "rtabmap — RGB-D 视觉 SLAM,需要深度相机。mentorpi 没有 → 启动后收不到数据,建不出图。",
      "hector — 纯激光 SLAM,不需要里程计。适合平地快速移动;本车有里程计,用它精度不如 slam_toolbox,且未验证。",
      "cartographer — Google 的 2D/3D SLAM,功能强但配置复杂(要额外的 lua 配置),本车未验证。"
    };
    method_desc_->setText(tr(kDesc[idx < 0 || idx > 3 ? 0 : idx]));
  };
  connect(method_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, update_method_desc);
  update_method_desc(method_combo_->currentIndex());
  method_lay->addWidget(method_desc_);

  auto* hint = new QLabel(tr(
    "SLAM is started via service call to a slam_controller node on the robot.\n"
    "No SSH is used. The robot pulls up the launch file; the GUI subscribes to /map."), method_box);
  hint->setWordWrap(true);
  hint->setStyleSheet("color: #198754;");
  method_lay->addWidget(hint);

  auto* btn_row = new QHBoxLayout;
  start_btn_ = new QPushButton(tr("Start SLAM"), method_box);
  start_btn_->setToolTip(tr("按住下拉框选的方法启动 SLAM(mentorpi 只有 slam_toolbox 可用)"));
  stop_btn_  = new QPushButton(tr("Stop"), method_box);
  stop_btn_->setToolTip(tr("停止当前 SLAM;编辑地图前必须先停"));
  btn_row->addWidget(start_btn_);
  btn_row->addWidget(stop_btn_);
  method_lay->addLayout(btn_row);

  status_label_ = new QLabel(tr("Status: idle"), method_box);
  method_lay->addWidget(status_label_);

  root->addWidget(method_box);

  auto* save_box = new QGroupBox(tr("Save map(存到 PC 的 maps/ 目录)"), this);
  auto* save_lay = new QVBoxLayout(save_box);
  save_path_edit_ = new QLineEdit(save_box);
  save_path_edit_->setPlaceholderText("/maps/my_map");
  auto* browse_btn = new QPushButton(tr("Browse..."), save_box);
  browse_btn->setToolTip(tr("选择保存的基础路径(不带扩展名)"));
  save_btn_ = new QPushButton(tr("Save"), save_box);
  save_btn_->setToolTip(tr("把当前地图存成 PGM+YAML 到 PC 的 maps/ 目录"));

  auto* path_row = new QHBoxLayout;
  path_row->addWidget(save_path_edit_);
  path_row->addWidget(browse_btn);
  save_lay->addLayout(path_row);
  save_lay->addWidget(save_btn_);

  root->addWidget(save_box);

  auto* params_box = new QGroupBox(tr("SLAM 参数"), this);
  auto* params_lay = new QVBoxLayout(params_box);
  params_panel_ = new SlamParamsPanel(controller_, params_box);
  params_lay->addWidget(params_panel_);
  root->addWidget(params_box);

  root->addStretch();

  stop_btn_->setEnabled(false);

  connect(start_btn_, &QPushButton::clicked, this, [this]() {
    auto node = controller_ ? controller_->node() : nullptr;
    if (!node) {
      status_label_->setText(tr("Status: not connected to ROS"));
      return;
    }
    if (!client_) client_ = std::make_unique<SlamServiceClient>(node);
    const std::string method = method_combo_->currentText().toStdString();
    status_label_->setText(tr("Status: sending /start_slam (%1) ...").arg(method_combo_->currentText()));
    start_btn_->setEnabled(false);

    std::string msg;
    const bool ok = client_->startSlam(method, &msg);
    if (ok) {
      status_label_->setText(tr("Status: started - %1").arg(QString::fromStdString(msg)));
      stop_btn_->setEnabled(true);
    } else {
      status_label_->setText(tr("Status: failed - %1").arg(QString::fromStdString(msg)));
      start_btn_->setEnabled(true);
    }
  });

  connect(stop_btn_, &QPushButton::clicked, this, [this]() {
    auto node = controller_ ? controller_->node() : nullptr;
    if (!node) return;
    if (!client_) client_ = std::make_unique<SlamServiceClient>(node);
    status_label_->setText(tr("Status: sending /stop_slam ..."));
    std::string msg;
    const bool ok = client_->stopSlam(&msg);
    status_label_->setText(tr("Status: %1").arg(QString::fromStdString(msg)));
    stop_btn_->setEnabled(false);
    start_btn_->setEnabled(true);
    Q_UNUSED(ok);
  });

  connect(browse_btn, &QPushButton::clicked, this, [this]() {
    const QString p = QFileDialog::getSaveFileName(this, tr("Choose map path"),
                                                   save_path_edit_->text(),
                                                   tr("Map base name (*);;All files (*)"));
    if (!p.isEmpty()) {
      QString base = p;
      base.remove(".pgm", Qt::CaseInsensitive);
      base.remove(".yaml", Qt::CaseInsensitive);
      save_path_edit_->setText(base);
    }
  });

  connect(save_btn_, &QPushButton::clicked, this, [this]() {
    const QString p = save_path_edit_->text().trimmed();
    if (p.isEmpty()) {
      QMessageBox::warning(this, tr("Save map"), tr("Please enter a path"));
      return;
    }
    // 存到 PC 本地:直接用 GUI 手里的 /map 快照写 <name>.pgm + <name>.yaml
    const auto grid = controller_->latestMap();
    if (grid.data.empty() || grid.info.width == 0) {
      QMessageBox::warning(this, tr("Save map"), tr("还没有地图可保存(先建一张图)。"));
      return;
    }
    QString base_path = p;
    base_path.remove(QRegularExpression("\\.(pgm|yaml|yml)$",
                                        QRegularExpression::CaseInsensitiveOption));
    const QString pgm_path  = base_path + ".pgm";
    const QString yaml_path = base_path + ".yaml";
    const QString image_name = QFileInfo(base_path).fileName() + ".pgm";

    const auto pgm = rcj::gridToPgm(grid);
    QFile pf(pgm_path);
    if (!pf.open(QIODevice::WriteOnly)) {
      QMessageBox::warning(this, tr("Save map"), tr("写不了 %1").arg(pgm_path));
      return;
    }
    pf.write(reinterpret_cast<const char*>(pgm.data()), static_cast<qint64>(pgm.size()));
    pf.close();

    const std::string yaml = rcj::gridToYaml(grid, image_name.toStdString());
    QFile yf(yaml_path);
    if (!yf.open(QIODevice::WriteOnly)) {
      QMessageBox::warning(this, tr("Save map"), tr("写不了 %1").arg(yaml_path));
      return;
    }
    yf.write(yaml.data(), static_cast<qint64>(yaml.size()));
    yf.close();

    status_label_->setText(tr("Status: 已保存 %1 + .yaml").arg(pgm_path));
  });

  // 让「开始/停止」按钮反映真实状态。slam_controller 的 /slam_status 只在
  // 状态变化时发(volatile),后连上来的 GUI 收不到当前值 —— 所以改成定时
  // 主动查 /is_mapping(异步,不阻塞界面)。无论谁启动/停止 SLAM 都能同步。
  auto* state_timer = new QTimer(this);
  connect(state_timer, &QTimer::timeout, this, [this]() {
    auto node = controller_ ? controller_->node() : nullptr;
    if (!node) return;
    if (!client_) client_ = std::make_unique<SlamServiceClient>(node);
    client_->isMappingAsync([this](bool running) {
      QMetaObject::invokeMethod(this, [this, running]() {
        start_btn_->setEnabled(!running);
        stop_btn_->setEnabled(running);
        status_label_->setText(running ? tr("Status: 建图中(外部启动也会显示)")
                                       : tr("Status: 空闲"));
      }, Qt::QueuedConnection);
    });
  });
  state_timer->start(3000);
}

MappingPanel::~MappingPanel() = default;

}  // namespace rcj