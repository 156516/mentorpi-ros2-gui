// SPDX-License-Identifier: MIT
#include "robot_control_gui_humble/ui/main_window.h"

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
#include <QDialog>
#include <QTextBrowser>
#include <QSettings>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <QInputDialog>

#include "robot_control_gui_humble/ui/control_panel.h"
#include "robot_control_gui_humble/ui/navigation_panel.h"
#include "robot_control_gui_humble/ui/mapping_panel.h"
#include "robot_control_gui_humble/ui/settings_panel.h"
#include "robot_control_gui_humble/ui/robot_status_panel.h"
#include "robot_control_gui_humble/ui/teleop_panel.h"
#include "robot_control_gui_humble/ui/robot_view.h"
#include "robot_control_gui_humble/ui/map_edit_panel.h"
#include "robot_control_gui_humble/ui/log_panel.h"
#include "robot_control_gui_humble/ui/speed_dashboard.h"
#include "robot_control_gui_humble/ros/robot_controller.h"

namespace rcj {

// Helpers wired up in main.cpp — connect/disconnect to ROS network.
extern std::shared_ptr<rclcpp::Node> rcj_connect();
extern void rcj_disconnect();

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle(tr("MentorPi 控制面板 (ROS2 Humble) — 未连接"));
  resize(1280, 800);

  setupUi();
  ui_tick_timer_ = startTimer(100);
}

MainWindow::~MainWindow() = default;

void MainWindow::showHelp() {
  QDialog dlg(this);
  dlg.setWindowTitle(tr("使用说明"));
  dlg.resize(680, 640);
  auto* lay = new QVBoxLayout(&dlg);
  auto* tb  = new QTextBrowser(&dlg);
  tb->setOpenExternalLinks(false);
  tb->setHtml(tr(R"HTML(
<style>h3{margin:10px 0 4px} li{margin:2px 0} code{background:#f2f2f2}</style>
<h3>① 连接</h3>
<ul>
<li>点左上角 <b>🔌 连接小车</b> → 输入小车 IP(热点模式默认 <code>192.168.149.1</code>)→ 确定。</li>
<li>连接前 GUI <b>完全静默</b>,不会影响小车;退出前先点 <b>⏏ 断开</b>。</li>
<li>断开后可以随时再连,不需要冷启小车。</li>
</ul>

<h3>② 视图操作(右侧地图)</h3>
<ul>
<li><b>滚轮</b> = 缩放 · <b>中键拖动</b> = 平移 · <b>右键拖动</b> = 旋转</li>
<li>点 <b>🎯 重置视图</b> 回到正北朝上、自适应大小;工具栏会显示当前视角角度。</li>
<li><b>左键点地图</b> = 设导航目标点 · <b>Shift+左键</b> = 设初始位姿。</li>
</ul>

<h3>③ 控制页</h3>
<ul><li>双摇杆 / <b>W A S D</b> 开车,<b>空格</b> 急停。Camera 面板目前是占位(未接摄像头)。</li></ul>

<h3>④ 建图页</h3>
<ul>
<li>方法选 <b>slam_toolbox</b>(默认;其他方法在 mentorpi 上不可用)。</li>
<li>点 <b>开始建图</b> → 切到控制页开车绕一圈 → 回来点 <b>停止</b>。</li>
<li><b>SLAM 参数</b>:点 <b>读取当前值</b> 看车上实际值;改了以后 ——
  带 <b>*</b> 的(如 map_update_interval / resolution / max_laser_range /
  transform_publish_period)slam_toolbox <b>只在启动时读</b>,必须点 <b>应用并重启 SLAM</b>;
  没带 * 的(如 minimum_travel_distance)可点 <b>应用(动态)</b> 即时生效。</li>
<li><b>Save map</b> 一栏是让<b>小车端</b>存图(需要小车上有 map_saver);要自己编辑/带走地图请用下面的「地图编辑」页。</li>
</ul>

<h3>⑤ 地图编辑页(改建好的图)</h3>
<ul>
<li><b>先停止建图</b>(SLAM 运行中不允许编辑)。</li>
<li><b>📌 编辑当前地图</b>:把当前这张图冻结成可编辑副本。<br>
    <b>📂 导入地图</b>:选一个 <code>.yaml</code>,同名 <code>.pgm</code> 会自动一起读进来。</li>
<li>笔刷:<b>左键拖动</b> = 当前工具(补墙/擦除),<b>右键拖动</b> = 直接擦除;</li>
<li>调 <b>笔刷半径</b>;用 <b>↶ 撤销 / ↷ 重做</b> 反悔。</li>
<li><b>💾 另存为</b> → 写出 <code>&lt;名字&gt;.pgm</code> + <code>&lt;名字&gt;.yaml</code> 到 <code>/maps/</code>(宿主目录
    <code>ros2_gui/robot_control_gui_ros2/maps/</code>)。</li>
<li>编辑期间实时地图被冻结,退出编辑后自动恢复。</li>
</ul>

<h3>⑥ 导航页</h3>
<ul><li>需要小车端先起 Nav2(bringup + AMCL)。之后在地图上左键点目标点即可下发。</li></ul>



<h3>⑩ 自定义算法(全局规划器 / 路径跟踪控制器)</h3>
<ul>
<li><b>①</b> 导航页 → <b>➕ 添加自定义算法</b> → 选类型(全局规划器/控制器)+ 起名(小写字母/数字/下划线)<br>
    → 模板生成到 <code>custom_algo/&lt;名字&gt;/</code></li>
<li><b>②</b> 用 VS Code 改 <code>custom_algo/&lt;名字&gt;/src/&lt;名字&gt;.cpp</code>:<br>
    · 规划器 → 改 <code>createPlan()</code>(给起点终点,返回一条 nav_msgs/Path)<br>
    · 控制器 → 改 <code>computeVelocityCommands()</code>(给当前位姿,返回速度指令)<br>
    模板里已有<b>能跑的示例</b>(规划器=直线插值,控制器=纯跟踪),替换成你的算法即可。</li>
<li><b>③</b> 在终端跑 <code>bash deploy_algo.sh &lt;名字&gt;</code><br>
    → 传到小车、在 <b>ARM64 上编译</b>并部署(必须在小车上编,PC 编的用不了)。</li>
<li><b>④</b> 回 GUI 导航页 → <b>读取当前值</b> → 下拉里出现 <b>「★ 自定义: &lt;包名&gt;/&lt;类名&gt;」</b>
    → 选它 → <b>应用并重启导航</b>。</li>
<li><b>改已有算法</b>:重复 ②③,再点「应用并重启导航」—— <b>Nav2 重启才会加载新的 .so</b>,这步不能省。</li>
</ul>

<h3>⑦ 状态 / 设置</h3>
<ul><li>状态页看 odom / 电池 / IMU;设置页可存小车 IP 与速度上限。</li></ul>

<h3>⑧ SLAM 方法怎么选(建图页的 Method)</h3>
<ul>
<li><b>slam_toolbox</b> — 2D 激光 SLAM,只要激光雷达就能跑,输出栅格地图 <code>/map</code>。<b>mentorpi 用这个</b>(唯一实测可用)。</li>
<li><b>rtabmap</b> — RGB-D 视觉 SLAM,<b>需要深度相机</b>。mentorpi 只有 2D 雷达 → 启动后收不到数据、建不出图。</li>
<li><b>hector</b> — 纯激光 SLAM、<b>不依赖里程计</b>,适合平地快速移动;本车有里程计,用它精度不如 slam_toolbox,且未验证。</li>
<li><b>cartographer</b> — Google 的 2D/3D SLAM,功能强但配置复杂(要额外的 lua 配置),本车未验证。</li>
</ul>

<h3>⑨ SLAM 参数各是什么(建图页的 SLAM 参数)</h3>
<ul>
<li><b>map_update_interval</b> — 地图刷新间隔(秒)。slam_toolbox 多久把地图重算一遍发到 /map。
    <b>越小地图越跟手,越费小车 CPU</b>;默认 5.0,调到 1.0~2.0 已明显更流畅。<b>需重启</b>。</li>
<li><b>resolution</b> — 分辨率(米/格)。每个格子代表多少米。<b>越小越精细,地图文件越大、越费 CPU</b>;
    默认 0.05(5cm)。<b>需重启</b>。</li>
<li><b>max_laser_range</b> — 雷达有效距离(米)。超过这个距离的点当作无效、不参与建图。
    调小可滤掉远处噪声/玻璃反射,调大覆盖更大范围;默认 20.0。<b>需重启</b>。</li>
<li><b>transform_publish_period</b> — map→odom 坐标变换的发布周期(秒)。<b>越小,车在地图里的位置更新越平滑</b>;
    默认 0.02(=50Hz)。<b>需重启</b>。</li>
<li><b>minimum_travel_distance</b> — 最小移动距离(米)。车至少移动这么远才处理一帧新雷达,用来过滤原地抖动;
    调大省 CPU 但丢细节,调小更灵敏;默认 0.5。<b>可动态生效</b>。</li>
</ul>
)HTML"));
  lay->addWidget(tb);
  auto* ok = new QPushButton(tr("知道了"), &dlg);
  connect(ok, &QPushButton::clicked, &dlg, &QDialog::accept);
  lay->addWidget(ok);
  dlg.exec();
}

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
  connect_btn_->setToolTip(tr("连接前 GUI 完全静默;点此输入小车 IP 建立 ROS 连接"));
  connect_btn_->setStyleSheet("background:#198754;color:white;padding:6px 14px;font-weight:bold;");
  tb->addWidget(connect_btn_);

  disconnect_btn_ = new QPushButton(tr("⏏ 断开"), tb);
  disconnect_btn_->setToolTip(tr("断开 ROS、停止订阅(建议退出前先点这个)"));
  disconnect_btn_->setStyleSheet("background:#dc3545;color:white;padding:6px 14px;");
  disconnect_btn_->setEnabled(false);
  tb->addWidget(disconnect_btn_);

  // View controls: wheel = zoom, middle-drag = pan, right-drag = rotate.
  reset_view_btn_ = new QPushButton(tr("🎯 重置视图"), tb);
  reset_view_btn_->setEnabled(false);
  reset_view_btn_->setToolTip(tr("滚轮缩放 · 中键拖动平移 · 右键拖动旋转"));
  connect(reset_view_btn_, &QPushButton::clicked, this,
          [this]() { if (view_) view_->resetView(); });
  tb->addWidget(reset_view_btn_);

  rotation_label_ = new QLabel(tr("视角 0°"), tb);
  rotation_label_->setStyleSheet("color:#6c757d;");
  tb->addWidget(rotation_label_);

  help_btn_ = new QPushButton(tr("❓ 使用说明"), tb);
  help_btn_->setToolTip(tr("怎么看各个页面、鼠标怎么用"));
  connect(help_btn_, &QPushButton::clicked, this, [this]() { showHelp(); });
  tb->addWidget(help_btn_);

  // 底部常驻操作提示 —— 鼠标怎么用一眼可见,不用翻文档
  auto* nav_hint = new QLabel(tr(
      "视图:滚轮缩放 · 中键拖动平移 · 右键拖动旋转 | 地图:左键设目标 · Shift+左键设初始位姿"));
  nav_hint->setStyleSheet("color:#6c757d;");
  statusBar()->addPermanentWidget(nav_hint);

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
  for (const auto& name : {tr("控制"), tr("导航"), tr("建图"), tr("地图编辑"), tr("状态"), tr("设置")}) {
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

  // 自动化测试入口:RCJ_AUTOCONNECT_IP 非空则跳过弹框直接连
  QString ip = qEnvironmentVariable("RCJ_AUTOCONNECT_IP");
  if (ip.isEmpty()) {
    bool ok = false;
    ip = QInputDialog::getText(this, tr("连接小车"),
        tr("小车 IP (热点模式默认 192.168.149.1):"),
        QLineEdit::Normal, ip_label_->text(), &ok);
    if (!ok || ip.isEmpty()) return;
  }
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

  // BUGFIX: create the RobotView FIRST. wireUpController() used to run
  // before view_ existed, so its `if (view_)` block was skipped and the
  // goalSelected / initialPoseSelected connections were never made
  // (click-to-set-goal silently did nothing).
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

  // Build panels under their tabs.
  // Clear placeholder tabs and add real panels.
  while (tabs_->count() > 0) tabs_->removeTab(0);
  control_panel_  = std::make_unique<ControlPanel>(controller_, tabs_);
  nav_panel_      = std::make_unique<NavigationPanel>(controller_, view_.get(), tabs_);
  mapping_panel_  = std::make_unique<MappingPanel>(controller_, tabs_);
  settings_panel_ = std::make_unique<SettingsPanel>(controller_, tabs_);
  status_panel_   = std::make_unique<RobotStatusPanel>(tabs_);
  // 「遥控」页已删除:它的方向键功能和控制页的摇杆/WASD 完全重复。
  map_edit_panel_ = std::make_unique<MapEditPanel>(controller_, view_.get(), tabs_);
  log_panel_      = std::make_unique<LogPanel>(controller_, tabs_);
  tabs_->addTab(control_panel_.get(),  tr("控制"));
  tabs_->addTab(nav_panel_.get(),      tr("导航"));
  tabs_->addTab(mapping_panel_.get(),  tr("建图"));
  tabs_->addTab(map_edit_panel_.get(), tr("地图编辑"));
  tabs_->addTab(log_panel_.get(),      tr("日志"));
  tabs_->addTab(status_panel_.get(),   tr("状态"));
  tabs_->addTab(settings_panel_.get(), tr("设置"));

  // 地图编辑期间冻结实时 /map
  connect(map_edit_panel_.get(), &MapEditPanel::editModeChanged, this,
          [this](bool on) { map_locked_ = on; });

  wireUpController();

  if (reset_view_btn_) reset_view_btn_->setEnabled(true);
  if (view_) {
    connect(view_.get(), &RobotView::viewRotated, this,
            [this](double d) {
              if (rotation_label_)
                rotation_label_->setText(tr("视角 %1°").arg(d, 0, 'f', 0));
            });
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
  map_edit_panel_.reset();
  log_panel_.reset();
  map_locked_ = false;
  view_.reset();
  while (tabs_->count() > 0) tabs_->removeTab(0);
  for (const auto& name : {tr("控制"), tr("导航"), tr("建图"), tr("地图编辑"), tr("状态"), tr("设置")}) {
    tabs_->addTab(new QWidget(tabs_), name);
  }
  body_stack_->setCurrentIndex(0);
  connect_btn_->setEnabled(true);
  disconnect_btn_->setEnabled(false);
  if (reset_view_btn_) reset_view_btn_->setEnabled(false);
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
            footer_dashboard_->setLateralSpeed(o.twist.twist.linear.y);
            footer_dashboard_->setAngularSpeed(o.twist.twist.angular.z);
            const auto& p = o.pose.pose.position;
            const auto& q = o.pose.pose.orientation;
            const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            // 优先用 map 帧位姿(TF map→base_footprint),这样小车三角和地图
            // 在同一坐标系(map→odom 实测有 ~16° 偏航,只用 odom 会错位)。
            // TF 不可用(没起 SLAM/Nav2)时回退到 odom。
            double mx = 0, my = 0, myaw = 0;
            if (controller_->tryGetRobotPoseInMap(mx, my, myaw)) {
              cur_x_ = mx; cur_y_ = my; cur_yaw_ = myaw;
            } else {
              cur_x_ = p.x;
              cur_y_ = p.y;
              cur_yaw_ = std::atan2(siny_cosp, cosy_cosp);
            }
            if (view_) view_->setRobotPose(cur_x_, cur_y_, cur_yaw_);
          });

  // Map
  connect(controller_.get(), &RobotController::mapUpdated, this,
          [this](const nav_msgs::msg::OccupancyGrid& grid) {
            if (!view_) return;
            if (map_locked_) return;  // 地图编辑中:不吃实时 /map
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

  // AMCL 粒子云(定位准不准,一眼可见)
  connect(controller_.get(), &RobotController::particleCloudUpdated, this,
          [this](const geometry_msgs::msg::PoseArray& pa) {
            if (!view_) return;
            view_->setParticleCloud(RobotController::poseArrayToPoints(pa));
          });

  // Nav2 代价地图(全局/局部)—— 半透明彩色覆盖层
  connect(controller_.get(), &RobotController::globalCostmapUpdated, this,
          [this](const nav_msgs::msg::OccupancyGrid& g) {
            if (!view_ || map_locked_) return;
            const QImage img = RobotController::costmapToImage(g);
            if (img.isNull()) return;
            const auto& q = g.info.origin.orientation;
            const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            view_->setCostmapImage(img, g.info.resolution,
                                   g.info.origin.position.x, g.info.origin.position.y,
                                   yaw, /*local=*/false);
          });
  connect(controller_.get(), &RobotController::localCostmapUpdated, this,
          [this](const nav_msgs::msg::OccupancyGrid& g) {
            if (!view_ || map_locked_) return;
            const QImage img = RobotController::costmapToImage(g);
            if (img.isNull()) return;
            const auto& q = g.info.origin.orientation;
            const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            view_->setCostmapImage(img, g.info.resolution,
                                   g.info.origin.position.x, g.info.origin.position.y,
                                   yaw, /*local=*/true);
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
              p.header.stamp = controller_->node()->get_clock()->now();
              p.pose.position.x = x;
              p.pose.position.y = y;
              p.pose.orientation.w = std::cos(yaw / 2.0);
              p.pose.orientation.z = std::sin(yaw / 2.0);
              controller_->sendNavigateToPose(p);
            });

    // BUGFIX: initialPoseSelected was emitted but never connected, so
    // Shift+click ("set initial pose") silently did nothing.
    connect(view_.get(), &RobotView::initialPoseSelected,
            [this](double x, double y, double yaw) {
              geometry_msgs::msg::PoseWithCovarianceStamped p;
              p.header.frame_id = "map";
              p.header.stamp = controller_->node()->get_clock()->now();
              p.pose.pose.position.x = x;
              p.pose.pose.position.y = y;
              p.pose.pose.orientation.w = std::cos(yaw / 2.0);
              p.pose.pose.orientation.z = std::sin(yaw / 2.0);
              p.pose.covariance[0]  = 0.25;    // x variance
              p.pose.covariance[7]  = 0.25;    // y variance
              p.pose.covariance[35] = 0.0685;  // yaw variance
              controller_->sendInitialPose(p);
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
      footer_dashboard_->setLateralSpeed(0);
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
  footer_dashboard_->setLateralSpeed(lin_y);
  footer_dashboard_->setAngularSpeed(ang_z);
}

}  // namespace rcj
