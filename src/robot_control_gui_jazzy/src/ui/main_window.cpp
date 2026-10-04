// SPDX-License-Identifier: MIT
#include "robot_control_gui_jazzy/ui/main_window.h"

#include <cmath>

#include <QTabWidget>
#include <QDockWidget>
#include <QToolBar>
#include <QAction>
#include <QStackedWidget>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QStatusBar>
#include <QKeyEvent>
#include <QTimerEvent>
#include <QInputDialog>
#include <QMessageBox>
#include <QSettings>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <QInputDialog>

#include "robot_control_gui_jazzy/ui/control_panel.h"
#include "robot_control_gui_jazzy/ui/navigation_panel.h"
#include "robot_control_gui_jazzy/ui/mapping_panel.h"
#include "robot_control_gui_jazzy/ui/settings_panel.h"
#include "robot_control_gui_jazzy/ui/robot_status_panel.h"
#include "robot_control_gui_jazzy/ui/teleop_panel.h"
#include "robot_control_gui_jazzy/ui/robot_view.h"
#include "robot_control_gui_jazzy/ui/speed_dashboard.h"
#include "robot_control_gui_jazzy/ros/robot_controller.h"

namespace rcj {

// Helpers wired up in main.cpp — connect/disconnect to ROS network.
extern std::shared_ptr<rclcpp::Node> rcj_connect();
extern void rcj_disconnect();

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle(tr("MentorPi 控制面板 (ROS2 Jazzy) — 未连接"));
  resize(1280, 800);

  setupUi();
  ui_tick_timer_ = startTimer(100);
}

MainWindow::~MainWindow() = default;

void MainWindow::setupUi() {
  statusBar()->showMessage(tr("未连接 — 点击右上角 [连接小车] 开始"));

  // --- Tool bar with connect/disconnect + status ---
  auto* tb = addToolBar(tr("连接"));
  tb->setMovable(false);

  tb->addWidget(new QLabel(tr("  Robot IP: "), tb));
  ip_label_ = new QLabel("192.168.149.1", tb);
  ip_label_->setStyleSheet("font-family: monospace;");
  tb->addWidget(ip_label_);

  tb->addSeparator();

  connect_btn_ = new QPushButton(tr("🔌 连接小车"), tb);
  connect_btn_->setStyleSheet("background:#198754;color:white;padding:6px 14px;font-weight:bold;");
  tb->addWidget(connect_btn_);

  disconnect_btn_ = new QPushButton(tr("⏏ 断开"), tb);
  disconnect_btn_->setStyleSheet("background:#dc3545;color:white;padding:6px 14px;");
  disconnect_btn_->setEnabled(false);
  tb->addWidget(disconnect_btn_);

  auto* spacer = new QWidget(tb);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  tb->addWidget(spacer);

  tb->addWidget(new QLabel(tr("状态: "), tb));
  status_label_ = new QLabel(tr("未连接"), tb);
  status_label_->setStyleSheet("color:#dc3545;font-weight:bold;");
  tb->addWidget(status_label_);

  // --- Stacked body: placeholder (disconnected) <-> tabs+view (connected) ---
  body_stack_ = new QStackedWidget(this);

  // Placeholder page
  placeholder_ = new QWidget(body_stack_);
  auto* ph_lay = new QVBoxLayout(placeholder_);
  ph_lay->setContentsMargins(40, 40, 40, 40);
  auto* ph_title = new QLabel(tr("🚗 MentorPi 控制面板"), placeholder_);
  ph_title->setStyleSheet("font-size: 32px; font-weight: bold;");
  ph_title->setAlignment(Qt::AlignCenter);

  auto* ph_hint = new QLabel(tr(
    "当前未连接小车。\n\n"
    "点击右上角 [🔌 连接小车] 启动 ROS2 节点、订阅传感器话题。\n\n"
    "连接前 GUI 完全静默，不会对小车产生任何影响。\n"
    "连接后可点击 [⏏ 断开] 立即停止所有 ROS 通信。"), placeholder_);
  ph_hint->setAlignment(Qt::AlignCenter);
  ph_hint->setStyleSheet("color: #555; font-size: 14px;");
  ph_hint->setWordWrap(true);

  auto* ph_btn = new QPushButton(tr("连接小车"), placeholder_);
  ph_btn->setStyleSheet("padding:12px 32px; font-size:16px; background:#198754; color:white;");
  ph_btn->setMaximumWidth(220);
  ph_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  auto* ph_col = new QWidget(placeholder_);
  auto* ph_col_lay = new QVBoxLayout(ph_col);
  ph_col_lay->addWidget(ph_title);
  ph_col_lay->addSpacing(20);
  ph_col_lay->addWidget(ph_hint);
  ph_col_lay->addSpacing(30);
  ph_col_lay->addWidget(ph_btn, 0, Qt::AlignCenter);
  ph_col_lay->addStretch();
  ph_lay->addWidget(ph_col);

  body_stack_->addWidget(placeholder_);

  // Connected page (will be filled in by wireUpController)
  auto* connected_page = new QWidget(body_stack_);
  auto* cp_lay = new QHBoxLayout(connected_page);
  cp_lay->setContentsMargins(0, 0, 0, 0);

  auto* left_col = new QWidget(connected_page);
  auto* left_lay = new QVBoxLayout(left_col);
  left_lay->setContentsMargins(0, 0, 0, 0);
  left_lay->setSpacing(2);

  tabs_ = new QTabWidget(left_col);
  // Placeholders — wireUpController will replace them.
  for (const auto& name : {tr("控制"), tr("导航"), tr("建图"), tr("遥控"), tr("状态"), tr("设置")}) {
    tabs_->addTab(new QWidget(tabs_), name);
  }

  footer_dashboard_ = new SpeedDashboard(left_col);
  footer_dashboard_->setMaximumHeight(120);
  left_lay->addWidget(tabs_, 1);
  left_lay->addWidget(footer_dashboard_);

  cp_lay->addWidget(left_col, 1);
  cp_lay->addWidget(new QWidget(connected_page), 2);  // view placeholder
  body_stack_->addWidget(connected_page);

  setCentralWidget(body_stack_);

  // --- Wire up buttons (always live, no ROS) ---
  connect(connect_btn_, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
  connect(disconnect_btn_, &QPushButton::clicked, this, &MainWindow::onDisconnectClicked);
  connect(ph_btn, &QPushButton::clicked, this, &MainWindow::onConnectClicked);

  // P7: robot selector (informational only)
  tb->addWidget(new QLabel(tr("  Robot: "), tb));
  robot_selector_ = new QComboBox(tb);
  robot_selector_->addItems({"mentorpi", "robot_01", "robot_02"});
  robot_selector_->setEditable(true);
  tb->addWidget(robot_selector_);
  tb->addWidget(new QLabel(tr("  Namespace: "), tb));
  namespace_edit_ = new QLineEdit("/", tb);
  namespace_edit_->setMaximumWidth(120);
  tb->addWidget(namespace_edit_);
}

void MainWindow::onConnectClicked() {
  if (connected_) return;

  // Ask for IP (default to mentorpi).
  bool ok = false;
  const QString ip = QInputDialog::getText(this, tr("连接小车"),
      tr("小车 IP (热点模式默认 192.168.149.1):"),
      QLineEdit::Normal, ip_label_->text(), &ok);
  if (!ok || ip.isEmpty()) return;
  ip_label_->setText(ip);
  QSettings().setValue("network/robot_ip", ip);

  status_label_->setText(tr("正在连接..."));
  status_label_->setStyleSheet("color:#fd7e14;font-weight:bold;");

  if (!rcj_connect()) {
    status_label_->setText(tr("连接失败"));
    status_label_->setStyleSheet("color:#dc3545;font-weight:bold;");
    QMessageBox::critical(this, tr("连接失败"), tr("无法初始化 ROS2，请检查环境"));
    return;
  }

  // rcj_connect() returned true and is keeping the shared node alive
  // in its module globals. Recover it by re-issuing the same call —
  // it's a no-op the second time and returns the cached node.
  auto shared_node = rcj_connect();
  if (!shared_node) {
    status_label_->setText(tr("未获取到共享节点"));
    return;
  }
  // Create the controller sharing that node (no double-add to executor).
  controller_ = std::make_shared<RobotController>(shared_node);
  if (!controller_->isInitialized()) {
    status_label_->setText(tr("RobotController 创建失败"));
    return;
  }

  // Build panels under their tabs.
  // Clear placeholder tabs and add real panels.
  while (tabs_->count() > 0) tabs_->removeTab(0);
  control_panel_  = std::make_unique<ControlPanel>(controller_, tabs_);
  nav_panel_      = std::make_unique<NavigationPanel>(controller_, /*view*/nullptr, tabs_);
  mapping_panel_  = std::make_unique<MappingPanel>(controller_, tabs_);
  settings_panel_ = std::make_unique<SettingsPanel>(controller_, tabs_);
  status_panel_   = std::make_unique<RobotStatusPanel>(tabs_);
  teleop_panel_   = std::make_unique<TeleopPanel>(controller_, tabs_);
  tabs_->addTab(control_panel_.get(),  tr("控制"));
  tabs_->addTab(nav_panel_.get(),      tr("导航"));
  tabs_->addTab(mapping_panel_.get(),  tr("建图"));
  tabs_->addTab(teleop_panel_.get(),   tr("遥控"));
  tabs_->addTab(status_panel_.get(),   tr("状态"));
  tabs_->addTab(settings_panel_.get(), tr("设置"));

  wireUpController();

  // Replace connected page placeholder view with the real RobotView.
  auto* cp = qobject_cast<QWidget*>(body_stack_->widget(1));
  if (cp) {
    auto* lay = qobject_cast<QHBoxLayout*>(cp->layout());
    if (lay && lay->count() >= 2) {
      auto* old = lay->itemAt(1)->widget();
      lay->removeWidget(old);
      delete old;
      view_ = std::make_unique<RobotView>(cp);
      lay->addWidget(view_.get(), 2);
    }
  }

  // Show the connected page.
  body_stack_->setCurrentIndex(1);

  // UI state
  connect_btn_->setEnabled(false);
  disconnect_btn_->setEnabled(true);
  connected_ = true;
  status_label_->setText(tr("已连接"));
  status_label_->setStyleSheet("color:#198754;font-weight:bold;");
  statusBar()->showMessage(tr("已连接 ROS_DOMAIN_ID=%1 @ %2")
      .arg(qEnvironmentVariable("ROS_DOMAIN_ID", "0")).arg(ip_label_->text()));
  setWindowTitle(tr("MentorPi 控制面板 — 已连接 %1").arg(ip_label_->text()));
}

void MainWindow::onDisconnectClicked() {
  if (!connected_) return;
  rcj_disconnect();
  controller_.reset();
  control_panel_.reset();
  nav_panel_.reset();
  mapping_panel_.reset();
  settings_panel_.reset();
  status_panel_.reset();
  teleop_panel_.reset();
  view_.reset();
  while (tabs_->count() > 0) tabs_->removeTab(0);
  for (const auto& name : {tr("控制"), tr("导航"), tr("建图"), tr("遥控"), tr("状态"), tr("设置")}) {
    tabs_->addTab(new QWidget(tabs_), name);
  }
  body_stack_->setCurrentIndex(0);
  connect_btn_->setEnabled(true);
  disconnect_btn_->setEnabled(false);
  connected_ = false;
  status_label_->setText(tr("未连接"));
  status_label_->setStyleSheet("color:#dc3545;font-weight:bold;");
  statusBar()->showMessage(tr("未连接"));
  setWindowTitle(tr("MentorPi 控制面板 — 未连接"));
}

void MainWindow::wireUpController() {
  // Status
  connect(controller_.get(), &RobotController::batteryUpdated,
          status_panel_.get(), &RobotStatusPanel::onBattery);
  connect(controller_.get(), &RobotController::diagnosticsUpdated,
          status_panel_.get(), &RobotStatusPanel::onDiagnostics);
  connect(controller_.get(), &RobotController::odomUpdated,
          status_panel_.get(), &RobotStatusPanel::onOdom);

  // Footer dashboard + cached pose for TF fallback
  connect(controller_.get(), &RobotController::odomUpdated, this,
          [this](const nav_msgs::msg::Odometry& o) {
            footer_dashboard_->setLinearSpeed(o.twist.twist.linear.x);
            footer_dashboard_->setAngularSpeed(o.twist.twist.angular.z);
            const auto& p = o.pose.pose.position;
            const auto& q = o.pose.pose.orientation;
            const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            cur_x_   = p.x;
            cur_y_   = p.y;
            cur_yaw_ = std::atan2(siny_cosp, cosy_cosp);
            if (view_) view_->setRobotPose(cur_x_, cur_y_, cur_yaw_);
          });

  // Map
  connect(controller_.get(), &RobotController::mapUpdated, this,
          [this](const nav_msgs::msg::OccupancyGrid& grid) {
            if (!view_) return;
            const QImage img = RobotController::occupancyGridToImage(grid);
            std::fprintf(stderr, "[MW] /map recv: %dx%d cells, res=%f m, img=%dx%d\n",
                         grid.info.width, grid.info.height, grid.info.resolution,
                         img.width(), img.height());
            std::fflush(stderr);
            view_->setMap(img, grid.info.resolution,
                          grid.info.origin.position.x,
                          grid.info.origin.position.y,
                          std::atan2(2 * (grid.info.origin.orientation.w * grid.info.origin.orientation.z),
                                     1 - 2 * std::pow(grid.info.origin.orientation.z, 2)));
          });

  // Laser scan: prefer TF, fall back to odom pose.
  connect(controller_.get(), &RobotController::scanUpdated, this,
          [this](const sensor_msgs::msg::LaserScan& scan) {
            if (!view_) return;
            QVector<QPointF> pts;
            if (controller_) pts = controller_->laserScanToMap(scan);
            if (pts.isEmpty()) {
              pts = RobotController::laserScanToPoints(scan, cur_x_, cur_y_, cur_yaw_);
            }
            view_->setLaserScan(pts);
          });

  // Path
  connect(controller_.get(), &RobotController::pathUpdated, this,
          [this](const nav_msgs::msg::Path& p) {
            if (!view_) return;
            view_->setPath(RobotController::pathToPoints(p));
          });

  // Click-to-set goal / initial pose (from view back into controller)
  if (view_) {
    connect(view_.get(), &RobotView::goalSelected,
            [this](double x, double y, double yaw) {
              geometry_msgs::msg::PoseStamped p;
              p.header.frame_id = "map";
              p.header.stamp = rclcpp::Clock().now();
              p.pose.position.x = x;
              p.pose.position.y = y;
              p.pose.orientation.w = std::cos(yaw / 2.0);
              p.pose.orientation.z = std::sin(yaw / 2.0);
              controller_->sendNavigateToPose(p);
            });
  }
}

void MainWindow::keyPressEvent(QKeyEvent* e) {
  if (!e->isAutoRepeat()) {
    key_states_[e->key()] = true;
    updateKeyboardControl();
  }
}

void MainWindow::keyReleaseEvent(QKeyEvent* e) {
  if (!e->isAutoRepeat()) {
    key_states_[e->key()] = false;
    updateKeyboardControl();
    if (e->key() == Qt::Key_Space && controller_) {
      controller_->emergencyStop();
      if (control_panel_) {
        control_panel_->setKeyLinearX(0);
        control_panel_->setKeyLinearY(0);
        control_panel_->setKeyAngularZ(0);
      }
      footer_dashboard_->setLinearSpeed(0);
      footer_dashboard_->setAngularSpeed(0);
    }
  }
}

void MainWindow::timerEvent(QTimerEvent* /*e*/) {}

void MainWindow::updateKeyboardControl() {
  if (!controller_ || !control_panel_) return;
  // Decoupled axes — each key sets one axis on the control panel; the panel
  // merges keyboard + joystick contributions.
  //   W/S     → linear.x  (forward / back)
  //   A/D     → linear.y  (sideways / strafe)
  //   Q/E     → angular.z (rotate left / right)
  bool up    = key_states_.value(Qt::Key_W) || key_states_.value(Qt::Key_Up);
  bool down  = key_states_.value(Qt::Key_S) || key_states_.value(Qt::Key_Down);
  bool a_left  = key_states_.value(Qt::Key_A);
  bool d_right = key_states_.value(Qt::Key_D);
  // Z / C  for rotation (Q/E may be intercepted by desktop shortcuts).
  bool z_left  = key_states_.value(Qt::Key_Z);
  bool c_right = key_states_.value(Qt::Key_C);

  const double lin_x = (up ?  max_linear_ : 0.0) - (down ? max_linear_ : 0.0);
  const double lin_y = (a_left ?  max_linear_ : 0.0) - (d_right ? max_linear_ : 0.0);
  const double ang_z = (z_left ?  max_angular_ : 0.0) - (c_right ? max_angular_ : 0.0);

  control_panel_->setKeyLinearX(lin_x);
  control_panel_->setKeyLinearY(lin_y);
  control_panel_->setKeyAngularZ(ang_z);

  // Footer dashboard still wants the *combined* speed (use it as a display
  // for the linear.x and angular.z the user actually commanded).
  footer_dashboard_->setLinearSpeed(lin_x);
  footer_dashboard_->setAngularSpeed(ang_z);
}

}  // namespace rcj
