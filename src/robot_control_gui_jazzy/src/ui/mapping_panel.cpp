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
#include "robot_control_gui_jazzy/ui/mapping_panel.h"
#include "robot_control_gui_jazzy/ros/robot_controller.h"

#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QFileDialog>
#include <QMessageBox>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
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
          "/slam_request_method", 10);
    }
    if (!start_client_) {
      start_client_ = node_->create_client<std_srvs::srv::Trigger>("/start_slam");
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

  bool stopSlam(std::string* msg) {
    if (!stop_client_) {
      stop_client_ = node_->create_client<std_srvs::srv::Empty>("/stop_slam");
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
  method_row->addWidget(new QLabel(tr("Method:"), method_box));
  method_row->addWidget(method_combo_);
  method_lay->addLayout(method_row);

  auto* hint = new QLabel(tr(
    "SLAM is started via service call to a slam_controller node on the robot.\n"
    "No SSH is used. The robot pulls up the launch file; the GUI subscribes to /map."), method_box);
  hint->setWordWrap(true);
  hint->setStyleSheet("color: #198754;");
  method_lay->addWidget(hint);

  auto* btn_row = new QHBoxLayout;
  start_btn_ = new QPushButton(tr("Start SLAM"), method_box);
  stop_btn_  = new QPushButton(tr("Stop"), method_box);
  btn_row->addWidget(start_btn_);
  btn_row->addWidget(stop_btn_);
  method_lay->addLayout(btn_row);

  status_label_ = new QLabel(tr("Status: idle"), method_box);
  method_lay->addWidget(status_label_);

  root->addWidget(method_box);

  auto* save_box = new QGroupBox(tr("Save map"), this);
  auto* save_lay = new QVBoxLayout(save_box);
  save_path_edit_ = new QLineEdit(save_box);
  save_path_edit_->setPlaceholderText("/home/robot/maps/my_map");
  auto* browse_btn = new QPushButton(tr("Browse..."), save_box);
  save_btn_ = new QPushButton(tr("Save"), save_box);

  auto* path_row = new QHBoxLayout;
  path_row->addWidget(save_path_edit_);
  path_row->addWidget(browse_btn);
  save_lay->addLayout(path_row);
  save_lay->addWidget(save_btn_);

  root->addWidget(save_box);
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
    controller_->requestSaveMap(p);
    status_label_->setText(tr("Status: save requested at %1").arg(p));
  });
}

MappingPanel::~MappingPanel() = default;

}  // namespace rcj