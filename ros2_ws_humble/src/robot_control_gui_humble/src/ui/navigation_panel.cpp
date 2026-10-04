// SPDX-License-Identifier: MIT
//
// navigation_panel.cpp — 导航页。
//   设点统一走右侧地图交互(左键=目标点,Shift+左键=初始位姿)。
//   本页提供:怎么导航的说明、状态、Nav2 算法/参数、代价地图开关、取消导航。
#include "robot_control_gui_humble/ui/navigation_panel.h"
#include "robot_control_gui_humble/ui/robot_view.h"
#include "robot_control_gui_humble/ros/robot_controller.h"
#include "robot_control_gui_humble/ros/topic_names.h"
#include "robot_control_gui_humble/ui/algo_template.h"
#include "robot_control_gui_humble/ui/log_panel.h"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QFileInfo>
#include <QRegularExpression>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QDir>
#include <QFile>
#include <QInputDialog>
#include <QTextStream>
#include <QPushButton>
#include <cmath>
#include <QTimer>
#include <QVBoxLayout>
#include <QTableWidget>
#include <QHeaderView>
#include <QDialog>
#include <QDialogButtonBox>

#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <sstream>
#include <vector>
#include <array>
#include <cstdio>
#include <string>

namespace rcj {

namespace {
// 算法 key → 显示名。key 会原样发给小车端 nav_controller.py(它持有真正的插件映射)。
struct Algo { const char* key; const char* label; };
const Algo kPlanners[] = {
  {"navfn",        "NavFn(经典,快而稳)"},
  {"smac2d",       "Smac 2D(A* 路径更平滑)"},
  {"smac_hybrid",  "Smac Hybrid-A*(考虑车体运动学)"},
  {"smac_lattice", "Smac State Lattice(状态栅格)"},
  {"theta_star",   "Theta*(任意角度,路径更直)"},
};
const Algo kControllers[] = {
  {"dwb",  "DWB 动态窗口(经典,默认)"},
  {"rpp",  "Regulated Pure Pursuit(纯跟踪,平滑)"},
  {"mppi", "MPPI 模型预测(最先进,吃 CPU)"},
};

const char* pluginToKey(const std::string& cls, bool planner) {
  struct P { const char* frag; const char* key; };
  static const P pl[] = {
    {"navfn", "navfn"}, {"SmacPlanner2D", "smac2d"},
    {"SmacPlannerHybrid", "smac_hybrid"}, {"SmacPlannerLattice", "smac_lattice"},
    {"theta_star", "theta_star"},
  };
  static const P ct[] = {
    {"DWBLocalPlanner", "dwb"}, {"RegulatedPurePursuit", "rpp"}, {"MPPI", "mppi"},
  };
  const P* t = planner ? pl : ct;
  const size_t n = planner ? sizeof(pl)/sizeof(pl[0]) : sizeof(ct)/sizeof(ct[0]);
  for (size_t i = 0; i < n; ++i)
    if (cls.find(t[i].frag) != std::string::npos) return t[i].key;
  return nullptr;
}

}  // namespace

// 和小车端 nav_controller 打交道:读运行中的 Nav2 参数 / 下发配置并重启。
// (放在匿名命名空间外,好让头文件能前向声明)
class NavParamClient {
 public:
  explicit NavParamClient(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)) {}

  bool readRunning(std::string& planner_key, std::string& controller_key,
                   double& robot_radius, double& inflation_radius, std::string* err) {
    if (!node_) { if (err) *err = "未连接 ROS"; return false; }
    if (!pc_planner_) pc_planner_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/planner_server");
    if (!pc_controller_) pc_controller_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/controller_server");
    if (!pc_costmap_) pc_costmap_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/global_costmap/global_costmap");

    if (!pc_planner_->wait_for_service(std::chrono::seconds(2)) ||
        !pc_controller_->wait_for_service(std::chrono::seconds(2))) {
      if (err) *err = "Nav2 没在跑(/planner_server 或 /controller_server 不存在)";
      return false;
    }
    auto fp = pc_planner_->get_parameters({"GridBased.plugin"});
    auto fc = pc_controller_->get_parameters({"FollowPath.plugin"});
    if (fp.wait_for(std::chrono::seconds(3)) != std::future_status::ready ||
        fc.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      if (err) *err = "读参数超时"; return false;
    }
    std::string pcls, ccls;
    for (auto& p : fp.get()) if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) pcls = p.as_string();
    for (auto& p : fc.get()) if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) ccls = p.as_string();
    planner_key    = pluginToKey(pcls, true)  ? pluginToKey(pcls, true)  : pcls;
    controller_key = pluginToKey(ccls, false) ? pluginToKey(ccls, false) : ccls;

    if (pc_costmap_->wait_for_service(std::chrono::milliseconds(600))) {
      auto fr = pc_costmap_->get_parameters({"robot_radius", "inflation_layer.inflation_radius"});
      if (fr.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        for (auto& p : fr.get()) {
          if (p.get_name() == "robot_radius" &&
              p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) robot_radius = p.as_double();
          if (p.get_name() == "inflation_layer.inflation_radius" &&
              p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) inflation_radius = p.as_double();
        }
      }
    }
    return true;
  }

  // 列出某个算法插件自己的参数(Nav2 里叫 GridBased.* / FollowPath.*)
  // out: (名字, 类型名, 当前值字符串)
  bool listAlgoParams(bool planner,
                      std::vector<std::array<std::string, 3>>& out, std::string* err) {
    if (!ensure(planner)) { if (err) *err = "参数服务不可用(Nav2 没在跑?)"; return false; }
    auto pc = planner ? pc_planner_ : pc_controller_;
    const std::string prefix = planner ? "GridBased" : "FollowPath";
    auto lf = pc->list_parameters({prefix}, 4);
    if (lf.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      if (err) *err = "list_parameters 超时"; return false;
    }
    const auto names = lf.get().names;
    if (names.empty()) { if (err) *err = "该算法没有可读参数"; return false; }
    auto gf = pc->get_parameters(names);
    if (gf.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      if (err) *err = "get_parameters 超时"; return false;
    }
    for (const auto& p : gf.get()) {
      std::string val;
      const char* ty = "?";
      switch (p.get_type()) {
        case rclcpp::ParameterType::PARAMETER_DOUBLE:  val = std::to_string(p.as_double()); ty = "double"; break;
        case rclcpp::ParameterType::PARAMETER_INTEGER: val = std::to_string(p.as_int());    ty = "int";    break;
        case rclcpp::ParameterType::PARAMETER_BOOL:    val = p.as_bool() ? "true" : "false"; ty = "bool";  break;
        case rclcpp::ParameterType::PARAMETER_STRING:  val = p.as_string();                 ty = "string"; break;
        default: continue;   // 数组等先不列
      }
      // 去掉数组默认值的显示噪音:double 只保留 4 位
      if (ty == std::string("double")) { char b[32]; snprintf(b, sizeof(b), "%.4g", p.as_double()); val = b; }
      out.push_back({p.get_name(), ty, val});
    }
    return true;
  }

  // 改一个参数(按类型构造,尽量让它成功)
  bool setAlgoParam(bool planner, const std::string& name, const std::string& type,
                    const std::string& value, std::string* err) {
    if (!ensure(planner)) { if (err) *err = "参数服务不可用"; return false; }
    auto pc = planner ? pc_planner_ : pc_controller_;
    rclcpp::Parameter p;
    try {
      if (type == "double")       p = rclcpp::Parameter(name, std::stod(value));
      else if (type == "int")     p = rclcpp::Parameter(name, static_cast<int64_t>(std::stoll(value)));
      else if (type == "bool")    p = rclcpp::Parameter(name, value == "true" || value == "1");
      else                        p = rclcpp::Parameter(name, value);
    } catch (...) { if (err) *err = "值格式不对(" + type + ")"; return false; }
    auto f = pc->set_parameters({p});
    if (f.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      if (err) *err = "set_parameters 超时"; return false;
    }
    const auto r = f.get();
    if (!r.empty() && !r[0].successful) { if (err) *err = r[0].reason; return false; }
    if (err) *err = "已设置";
    return true;
  }

  // 问小车端有哪些可用插件(含它扫到的自定义算法)
  bool ensure(bool planner) {
    if (!node_) return false;
    if (!pc_planner_) pc_planner_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/planner_server");
    if (!pc_controller_) pc_controller_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/controller_server");
    auto pc = planner ? pc_planner_ : pc_controller_;
    return pc->wait_for_service(std::chrono::seconds(2));
  }

  bool listPlugins(std::vector<std::string>& planners,
                   std::vector<std::string>& controllers, std::string* err) {
    if (!node_) { if (err) *err = "未连接 ROS"; return false; }
    if (!list_client_) list_client_ = node_->create_client<std_srvs::srv::Trigger>(
        topics::navListPluginsService().toStdString());
    if (!list_client_->wait_for_service(std::chrono::seconds(2))) {
      if (err) *err = "小车端没有 /list_nav_plugins(需更新 nav_controller.py)";
      return false;
    }
    auto f = list_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (f.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      if (err) *err = "超时"; return false;
    }
    const std::string js = f.get()->message;
    auto grab = [&js](const char* key, std::vector<std::string>& out) {
      const size_t k = js.find(key);
      if (k == std::string::npos) return;
      const size_t b = js.find('[', k), e = js.find(']', k);
      if (b == std::string::npos || e == std::string::npos) return;
      std::string inner = js.substr(b + 1, e - b - 1);
      size_t pos = 0;
      while (pos < inner.size()) {
        size_t q1 = inner.find('"', pos);
        if (q1 == std::string::npos) break;
        size_t q2 = inner.find('"', q1 + 1);
        if (q2 == std::string::npos) break;
        out.push_back(inner.substr(q1 + 1, q2 - q1 - 1));
        pos = q2 + 1;
      }
    };
    grab("planners", planners);
    grab("controllers", controllers);
    return true;
  }

  bool apply(const std::string& planner, const std::string& controller,
             const std::string& map_path, double robot_radius, double inflation_radius,
             std::string* err) {
    if (!node_) { if (err) *err = "未连接 ROS"; return false; }
    if (!params_pub_) params_pub_ = node_->create_publisher<std_msgs::msg::String>(
        topics::navParamsTopic().toStdString(), 10);
    std::ostringstream os;
    os << "{\"planner\":\"" << planner << "\",\"controller\":\"" << controller
       << "\",\"map\":\"" << map_path << "\",\"robot_radius\":" << robot_radius
       << ",\"inflation_radius\":" << inflation_radius << "}";
    std_msgs::msg::String m; m.data = os.str();
    for (int i = 0; i < 20 && params_pub_->get_subscription_count() == 0; ++i)
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    params_pub_->publish(m);
    rclcpp::sleep_for(std::chrono::milliseconds(150));

    if (!apply_client_) apply_client_ = node_->create_client<std_srvs::srv::Trigger>(
        topics::navApplyParamsService().toStdString());
    if (!apply_client_->wait_for_service(std::chrono::seconds(3))) {
      if (err) *err = "小车端没有 /apply_nav_params(需要部署 nav_controller.py)";
      return false;
    }
    auto f = apply_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (f.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
      if (err) *err = "应用超时(Nav2 重启较慢)"; return false;
    }
    const auto r = f.get();
    if (err) *err = r->message;
    return r->success;
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::AsyncParametersClient::SharedPtr pc_planner_, pc_controller_, pc_costmap_;
 public:
  rclcpp::AsyncParametersClient::SharedPtr pc_planner_pub() { return pc_planner_; }
  rclcpp::AsyncParametersClient::SharedPtr pc_controller_pub() { return pc_controller_; }
 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr apply_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr list_client_;
};

NavigationPanel::NavigationPanel(std::shared_ptr<RobotController> c,
                                 RobotView* view, QWidget* parent)
    : QWidget(parent), controller_(std::move(c)), view_(view) {
  auto* root = new QVBoxLayout(this);

  // -- 怎么导航 --
  auto* how_box = new QGroupBox(tr("怎么导航"), this);
  auto* how_lay = new QVBoxLayout(how_box);
  auto* how = new QLabel(tr(
      "1. 在右侧地图上 <b>左键按下并拖动</b> → 拖的方向就是<b>目标朝向</b>,松手后只是"
      "<b>放一个待发的目标</b>(车不会动)。\n"
      "2. 确认位置/朝向没问题 → 点下面的 <b>▶ 开始导航</b>,车才出发。\n"
      "3. <b>Shift+左键</b> 点地图 = 告诉 AMCL 小车当前在哪(设初始位姿)。\n"
      "4. 导航中可点「取消导航」。"), how_box);
  how->setWordWrap(true);
  how->setTextFormat(Qt::RichText);
  how->setStyleSheet("color:#495057;");
  how_lay->addWidget(how);
  root->addWidget(how_box);

  // -- 状态 --
  auto* st_box = new QGroupBox(tr("状态"), this);
  auto* st_lay = new QVBoxLayout(st_box);
  state_label_ = new QLabel(tr("状态:空闲"), st_box);
  state_label_->setStyleSheet("font-weight:bold;");
  dist_label_  = new QLabel(tr("剩余距离: -- m"), st_box);
  st_lay->addWidget(state_label_);
  st_lay->addWidget(dist_label_);
  locate_label_ = new QLabel(tr("定位:检查中…"), st_box);
  locate_label_->setStyleSheet("font-weight:bold;");
  st_lay->addWidget(locate_label_);
  auto* go_row = new QHBoxLayout;
  start_btn_ = new QPushButton(tr("▶ 开始导航"), st_box);
  clear_btn_ = new QPushButton(tr("✕ 清除目标"), st_box);
  start_btn_->setEnabled(false);
  clear_btn_->setEnabled(false);
  start_btn_->setStyleSheet("background:#198754;color:white;font-weight:bold;padding:6px;");
  start_btn_->setToolTip(tr("把地图上那个待发目标下发出去,车开始走"));
  clear_btn_->setToolTip(tr("清掉地图上的待发目标"));
  go_row->addWidget(start_btn_);
  go_row->addWidget(clear_btn_);
  st_lay->addLayout(go_row);

  auto* cur_pose_btn = new QPushButton(tr("📍 用当前位置设为初始位姿"), st_box);
  cur_pose_btn->setToolTip(tr("把小车当前 odom 位姿直接当作地图上的初始位姿。\n"
                              "只在「刚建完图、没断电、接着导航」时正确;导入旧图/重启过请用 Shift+点击。"));
  connect(cur_pose_btn, &QPushButton::clicked, this, [this]() {
    if (!controller_ || !controller_->node()) return;
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.frame_id = "map";
    m.header.stamp = controller_->node()->get_clock()->now();
    m.pose.pose = controller_->currentPose();     // odom 位姿
    m.pose.covariance[0]  = 0.25;
    m.pose.covariance[7]  = 0.25;
    m.pose.covariance[35] = 0.0685;
    controller_->sendInitialPose(m);
    LogPanel::noteAs("amcl", tr("设初始位姿 (%1, %2)")
        .arg(m.pose.pose.position.x,0,'f',2).arg(m.pose.pose.position.y,0,'f',2));
    if (view_) view_->setInitialPose(m.pose.pose.position.x, m.pose.pose.position.y, 0.0);
  });
  st_lay->addWidget(cur_pose_btn);
  root->addWidget(st_box);

  // -- 导航参数(算法 + 常用参数)--
  auto* np_box = new QGroupBox(tr("导航参数(算法需重启 Nav2)"), this);
  auto* np_lay = new QVBoxLayout(np_box);
  auto* np_form = new QFormLayout;

  planner_combo_ = new QComboBox(np_box);
  for (const auto& a : kPlanners) planner_combo_->addItem(tr(a.label), QString(a.key));
  planner_combo_->setToolTip(tr("全局规划器:算从车到目标的路径。切换后需重启 Nav2。"));
  controller_combo_ = new QComboBox(np_box);
  for (const auto& a : kControllers) controller_combo_->addItem(tr(a.label), QString(a.key));
  controller_combo_->setToolTip(tr("控制器:沿路径控制车速。切换后需重启 Nav2。"));

  map_path_edit_ = new QLineEdit(np_box);
  map_path_edit_->setPlaceholderText("/home/ubuntu/ros2_ws/maps/test2");
  map_path_edit_->setToolTip(tr("留空 = 用下面「上传当前地图」推给小车的图"));
  map_path_edit_->setPlaceholderText(tr("(留空:用上传的图)"));
  robot_radius_ = new QDoubleSpinBox(np_box);
  robot_radius_->setRange(0.05, 1.0); robot_radius_->setSingleStep(0.05);
  robot_radius_->setDecimals(2); robot_radius_->setValue(0.20);
  robot_radius_->setToolTip(tr("车体半径(米):代价地图按它给车留空间"));
  inflation_radius_ = new QDoubleSpinBox(np_box);
  inflation_radius_->setRange(0.05, 2.0); inflation_radius_->setSingleStep(0.05);
  inflation_radius_->setDecimals(2); inflation_radius_->setValue(0.55);
  inflation_radius_->setToolTip(tr("膨胀半径(米):离墙至少留这么远才算代价低;调大更保守"));

  np_form->addRow(tr("全局规划器"), planner_combo_);
  np_form->addRow(tr("控制器"), controller_combo_);
  np_form->addRow(tr("地图路径"), map_path_edit_);
  np_form->addRow(tr("车体半径 (m)"), robot_radius_);
  np_form->addRow(tr("膨胀半径 (m)"), inflation_radius_);
  np_lay->addLayout(np_form);

  upload_btn_ = new QPushButton(tr("📤 上传当前地图给小车"), np_box);
  upload_btn_->setToolTip(tr("把 GUI 里当前的图(=建图刚建好的 / 编辑过的)\n"
                             "通过 ROS 发给小车,存成导航地图。\n"
                             "这样就不用把文件拷到小车、也不用填路径了。"));
  np_lay->addWidget(upload_btn_);
  auto* np_btns = new QHBoxLayout;
  read_btn_  = new QPushButton(tr("读取当前值"), np_box);
  apply_btn_ = new QPushButton(tr("应用并重启导航"), np_box);
  apply_btn_->setToolTip(tr("算法切换必须重启 Nav2(插件是启动时定的),约 10~20 秒"));
  np_btns->addWidget(read_btn_);
  np_btns->addWidget(apply_btn_);
  np_lay->addLayout(np_btns);
  auto* add_algo_btn = new QPushButton(tr("➕ 添加自定义算法(生成模板)"), np_box);
  add_algo_btn->setToolTip(tr("生成一个可编译的 Nav2 插件模板到 custom_algo/,\n"
                              "你用 VS Code 改完,再跑 deploy_algo.sh 编译部署到小车"));
  np_lay->addWidget(add_algo_btn);
  auto* algo_param_btn = new QPushButton(tr("🔧 算法参数(查询 / 修改)"), np_box);
  algo_param_btn->setToolTip(tr("看当前规划器/控制器有哪些参数、当前值是多少,并可直接改。\n"
                                "(改的是运行中的 Nav2,多数参数即时生效)"));
  np_lay->addWidget(algo_param_btn);
  np_status_ = new QLabel(tr("—"), np_box);
  np_status_->setWordWrap(true);
  np_status_->setStyleSheet("color:#495057;");
  np_lay->addWidget(np_status_);
  root->addWidget(np_box);

  // -- 代价地图 --
  auto* cm_box = new QGroupBox(tr("代价地图"), this);
  auto* cm_lay = new QVBoxLayout(cm_box);
  auto* g_chk = new QCheckBox(tr("显示全局代价地图(含墙边膨胀层)"), cm_box);
  auto* l_chk = new QCheckBox(tr("显示局部代价地图(车周围实时障碍)"), cm_box);
  g_chk->setChecked(true);
  g_chk->setToolTip(tr("RViz costmap 配色:青→黄→红 = 代价;黑 = 致命障碍;透明 = 可走/未知"));
  auto* cm_note = new QLabel(tr("RViz costmap 配色:青→黄→红 = 代价越高;黑 = 致命障碍;透明 = 可走。"), cm_box);
  cm_note->setWordWrap(true);
  cm_note->setStyleSheet("color:#6c757d;font-size:11px;");
  auto* p_chk = new QCheckBox(tr("显示 AMCL 粒子云(定位准不准一眼可见)"), cm_box);
  p_chk->setChecked(true);
  p_chk->setToolTip(tr("AMCL 的粒子:聚成一小团=定位准;散一大片=没定位准;跑到别处=收错了"));
  cm_lay->addWidget(g_chk);
  cm_lay->addWidget(l_chk);
  cm_lay->addWidget(p_chk);
  cm_lay->addWidget(cm_note);
  root->addWidget(cm_box);
  auto sync_cm = [this, g_chk, l_chk]() {
    if (view_) view_->setCostmapVisible(g_chk->isChecked(), l_chk->isChecked());
  };
  connect(g_chk, &QCheckBox::toggled, this, [sync_cm](bool) { sync_cm(); });
  connect(l_chk, &QCheckBox::toggled, this, [sync_cm](bool) { sync_cm(); });
  sync_cm();   // 应用初始勾选状态(否则默认勾了也不会显示)
  auto sync_pc = [this, p_chk]() { if (view_) view_->setParticleVisible(p_chk->isChecked()); };
  connect(p_chk, &QCheckBox::toggled, this, [sync_pc](bool) { sync_pc(); });
  sync_pc();

  // -- 取消 --
  auto* cnl_btn = new QPushButton(tr("✗ 取消导航"), this);
  cnl_btn->setToolTip(tr("取消当前导航目标"));
  connect(cnl_btn, &QPushButton::clicked, controller_.get(),
          &RobotController::cancelNavigation);
  root->addWidget(cnl_btn);

  root->addStretch();

  if (view_) {
    connect(view_, &RobotView::goalDrafted, this, [this](double x, double y, double yaw) {
      start_btn_->setEnabled(true);
      clear_btn_->setEnabled(true);
      np_status_->setText(tr("已放好目标:(%1, %2) 朝向 %3°。确认后点「▶ 开始导航」")
          .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw * 180.0 / M_PI, 0, 'f', 0));
    });
  }
  connect(start_btn_, &QPushButton::clicked, this, [this]() {
    if (!view_ || !controller_) return;
    double x = 0, y = 0, yaw = 0;
    if (!view_->takeDraftGoal(x, y, yaw)) { np_status_->setText(tr("没有待发目标")); return; }
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = controller_->node()->get_clock()->now();
    p.pose.position.x = x; p.pose.position.y = y;
    p.pose.orientation.w = std::cos(yaw / 2.0);
    p.pose.orientation.z = std::sin(yaw / 2.0);
    controller_->sendNavigateToPose(p);
    LogPanel::noteAs("bt_navigator", tr("下发导航目标 (%1, %2, %3°)")
        .arg(x,0,'f',2).arg(y,0,'f',2).arg(yaw*180.0/M_PI,0,'f',0));
    start_btn_->setEnabled(false);
    clear_btn_->setEnabled(false);
    np_status_->setText(tr("已下发导航目标,车开始移动…"));
  });
  connect(clear_btn_, &QPushButton::clicked, this, [this]() {
    if (view_) view_->clearDraftGoal();
    start_btn_->setEnabled(false);
    clear_btn_->setEnabled(false);
    np_status_->setText(tr("已清除目标"));
  });

  // 每 2 秒查一次 TF:有没有 map→odom(即 AMCL 有没有定位)
  auto* loc_timer = new QTimer(this);
  connect(loc_timer, &QTimer::timeout, this, [this]() {
    if (!controller_ || !locate_label_) return;
    double x = 0, y = 0, yaw = 0;
    if (controller_->tryGetRobotPoseInMap(x, y, yaw)) {
      locate_label_->setText(tr("定位:✅ 已定位  (%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
      locate_label_->setStyleSheet("color:#198754;font-weight:bold;");
    } else {
      locate_label_->setText(tr("定位:❌ 未定位 —— 请在图上 Shift+左键点小车位置(或 📍 按钮)"));
      locate_label_->setStyleSheet("color:#dc3545;font-weight:bold;");
    }
  });
  loc_timer->start(2000);

  connect(controller_.get(), &RobotController::navigationStateChanged,
          this, &NavigationPanel::onNavigationStateChanged);
  connect(controller_.get(), &RobotController::navigationDistanceRemaining,
          this, &NavigationPanel::onNavigationDistanceRemaining);

  connect(algo_param_btn, &QPushButton::clicked, this, [this]() {
    auto node = controller_ ? controller_->node() : nullptr;
    if (!node) { np_status_->setText(tr("未连接 ROS")); return; }
    if (!nav_client_) nav_client_ = std::make_unique<NavParamClient>(node);

    QDialog dlg(this);
    dlg.setWindowTitle(tr("算法参数"));
    dlg.resize(760, 520);
    auto* lay = new QVBoxLayout(&dlg);

    auto* head = new QHBoxLayout;
    head->addWidget(new QLabel(tr("看哪个:"), &dlg));
    auto* which = new QComboBox(&dlg);
    which->addItem(tr("全局规划器"), 1);
    which->addItem(tr("控制器"), 0);
    which->setCurrentIndex(0);
    head->addWidget(which);
    auto* reload = new QPushButton(tr("🔄 重新读取"), &dlg);
    head->addWidget(reload);
    head->addStretch();
    lay->addLayout(head);

    auto* tbl = new QTableWidget(&dlg);
    tbl->setColumnCount(3);
    tbl->setHorizontalHeaderLabels({tr("参数"), tr("类型"), tr("当前值(双击可改)")});
    tbl->horizontalHeader()->setStretchLastSection(true);
    tbl->setSelectionBehavior(QAbstractItemView::SelectRows);
    lay->addWidget(tbl);
    auto* st = new QLabel(tr("双击「当前值」单元格修改 → 点「应用修改」"), &dlg);
    st->setStyleSheet("color:#6c757d;");
    lay->addWidget(st);

    auto* do_apply = new QPushButton(tr("✅ 应用修改"), &dlg);
    auto* bb = new QDialogButtonBox(QDialogButtonBox::Close, &dlg);
    connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto* foot = new QHBoxLayout;
    foot->addWidget(do_apply);
    foot->addWidget(bb);
    lay->addLayout(foot);

    auto load = [this, tbl, which, st, do_apply]() {
      const bool planner = which->currentData().toInt() == 1;
      std::vector<std::array<std::string, 3>> rows;
      std::string err;
      if (!nav_client_->listAlgoParams(planner, rows, &err)) {
        st->setText(tr("读取失败:%1").arg(QString::fromStdString(err)));
        tbl->setRowCount(0); return;
      }
      tbl->setRowCount(0);
      for (const auto& r : rows) {
        const int i = tbl->rowCount();
        tbl->insertRow(i);
        tbl->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(r[0])));
        tbl->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(r[1])));
        auto* v = new QTableWidgetItem(QString::fromStdString(r[2]));
        v->setData(Qt::UserRole, QString::fromStdString(r[2]));   // 原始值,用来判断有没有改过
        tbl->setItem(i, 2, v);
      }
      st->setText(tr("读到 %1 个参数(改完点「应用修改」)").arg(rows.size()));
      do_apply->setEnabled(true);
    };
    connect(reload, &QPushButton::clicked, &dlg, load);
    connect(which, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg,
            [load](int) { load(); });

    connect(do_apply, &QPushButton::clicked, &dlg, [this, tbl, which, st]() {
      const bool planner = which->currentData().toInt() == 1;
      int changed = 0, ok = 0;
      for (int r = 0; r < tbl->rowCount(); ++r) {
        const QString name = tbl->item(r, 0)->text();
        const QString ty   = tbl->item(r, 1)->text();
        const QString val  = tbl->item(r, 2)->text();
        if (val == tbl->item(r, 2)->data(Qt::UserRole).toString()) continue;  // 没改
        ++changed;
        std::string err;
        if (nav_client_->setAlgoParam(planner, name.toStdString(), ty.toStdString(),
                                      val.toStdString(), &err)) ++ok;
        else st->setText(tr("❌ %1 失败:%2").arg(name, QString::fromStdString(err)));
      }
      if (changed == 0) st->setText(tr("没有改动"));
      else st->setText(tr("已应用 %1/%2 个改动").arg(ok).arg(changed));
    });

    load();
    dlg.exec();
  });

  connect(add_algo_btn, &QPushButton::clicked, this, [this]() {
    const QStringList types = { tr("全局规划器 (GlobalPlanner)"), tr("控制器 (Controller)") };
    bool ok = false;
    const QString ty = QInputDialog::getItem(this, tr("添加自定义算法"),
        tr("类型:"), types, 0, false, &ok);
    if (!ok) return;
    const QString name = QInputDialog::getText(this, tr("添加自定义算法"),
        tr("算法名(英文,建议小写+下划线,会成为 ROS 包名):"),
        QLineEdit::Normal, QStringLiteral("my_planner"), &ok).trimmed();
    if (!ok || name.isEmpty()) return;
    if (!QRegularExpression("^[a-z][a-z0-9_]*$").match(name).hasMatch()) {
      QMessageBox::warning(this, tr("名字不合法"),
          tr("只能用 小写字母/数字/下划线,且以字母开头(如 my_planner)"));
      return;
    }
    const bool is_planner = (ty == types[0]);
    const QString root = QStringLiteral("/algo") + "/" + name;   // 容器内,挂到宿主 custom_algo/
    QDir().mkpath(root + "/include/" + name);
    QDir().mkpath(root + "/src");
    const auto& files = is_planner ? kPlannerTemplate() : kControllerTemplate();
    QStringList written;
    for (const auto& f : files) {
      QString rel = QString::fromUtf8(f.path);
      rel.replace("@NAME@", name);
      const QString full = root + "/" + rel;
      QDir().mkpath(QFileInfo(full).absolutePath());
      QFile out(full);
      if (!out.open(QIODevice::WriteOnly | QIODevice::Text)) continue;
      QString body = QString::fromUtf8(f.content);
      // 驼峰类名:my_astar → MyAstar(这样类名和包名一致,不再是 MyPlanner)
      QString camel;
      bool up = true;
      for (const QChar c : name) {
        if (c == '_') { up = true; continue; }
        camel += up ? c.toUpper() : c;
        up = false;
      }
      body.replace("MyPidController", camel).replace("MyPlanner", camel)
          .replace("MyController", camel)
          .replace("my_pid_controller", name).replace("my_planner", name)
          .replace("my_controller", name);
      QTextStream(&out) << body;
      out.close();
      written << rel;
    }
    QMessageBox::information(this, tr("模板已生成"),
        tr("已生成到(容器 /algo/%1,宿主 custom_algo/%1):\n  %2\n\n"
           "下一步:\n"
           "1. 用 VS Code 打开 ros2_gui/robot_control_gui_ros2/custom_algo/%1 改算法\n"
           "2. 在终端跑:  bash deploy_algo.sh %1\n"
           "3. 回来在下面的下拉里就能选到它")
        .arg(name, written.join(QStringLiteral("\n  "))));
  });

  connect(upload_btn_, &QPushButton::clicked, this, [this]() {
    if (!controller_) return;
    const auto g = controller_->latestMap();
    if (g.data.empty()) { np_status_->setText(tr("没有地图可上传(先建图,或用地图编辑页导入)")); return; }
    std::string err;
    const bool ok = controller_->uploadNavMap(g, &err);
    if (ok) {
      map_path_edit_->clear();   // 用上传的图,不用填路径
      np_status_->setText(tr("✅ 已上传 %1x%2 的图给小车;点\"应用并重启导航\"即可用它")
                          .arg(g.info.width).arg(g.info.height));
    } else {
      np_status_->setText(tr("❌ 上传失败:%1").arg(QString::fromStdString(err)));
    }
  });

  connect(read_btn_, &QPushButton::clicked, this, [this]() {
    auto node = controller_ ? controller_->node() : nullptr;
    if (!node) { np_status_->setText(tr("未连接 ROS")); return; }
    if (!nav_client_) nav_client_ = std::make_unique<NavParamClient>(node);
    std::string pk, ck, err; double rr = -1, ir = -1;
    // ① 先刷新可用插件(含小车扫到的自定义算法)—— 不依赖 Nav2 在跑
    {
      std::vector<std::string> pls, cts;
      std::string e2;
      if (nav_client_->listPlugins(pls, cts, &e2)) {
        int added = 0;
        for (const auto& s : pls) {
          const QString q = QString::fromStdString(s);
          if (planner_combo_->findData(q) < 0) {
            planner_combo_->addItem(tr("★ 自定义: %1").arg(q), q); ++added;
          }
        }
        for (const auto& s : cts) {
          const QString q = QString::fromStdString(s);
          if (controller_combo_->findData(q) < 0) {
            controller_combo_->addItem(tr("★ 自定义: %1").arg(q), q); ++added;
          }
        }
        if (added) np_status_->setText(tr("已加入 %1 个自定义算法到下拉").arg(added));
      }
    }
    // ② 再读运行中的 Nav2(没跑/没定位会失败,给出提示但不影响上面)
    if (!nav_client_->readRunning(pk, ck, rr, ir, &err)) {
      np_status_->setText(tr("Nav2 状态读不到:%1\n"
          "(自定义算法已加进下拉了;要读当前值/切算法请先点「应用并重启导航」。\n"
          " 若 Nav2 刚起,还需先在地图上设初始位姿。)")
          .arg(QString::fromStdString(err)));
      return;
    }
    int i = planner_combo_->findData(QString::fromStdString(pk));
    if (i >= 0) planner_combo_->setCurrentIndex(i);
    i = controller_combo_->findData(QString::fromStdString(ck));
    if (i >= 0) controller_combo_->setCurrentIndex(i);
    if (rr > 0) robot_radius_->setValue(rr);
    if (ir > 0) inflation_radius_->setValue(ir);
    np_status_->setText(tr("当前运行:规划器=%1 控制器=%2").arg(
        QString::fromStdString(pk), QString::fromStdString(ck)));
  });

  connect(apply_btn_, &QPushButton::clicked, this, [this]() {
    auto node = controller_ ? controller_->node() : nullptr;
    if (!node) { np_status_->setText(tr("未连接 ROS")); return; }
    // 路径可留空:留空 = 用之前「上传当前地图」推给小车的图
    const QString map_path = map_path_edit_->text().trimmed();
    if (!nav_client_) nav_client_ = std::make_unique<NavParamClient>(node);
    apply_btn_->setEnabled(false);
    np_status_->setText(tr("正在下发并重启 Nav2…(约 10~20 秒)"));
    std::string err;
    const bool ok = nav_client_->apply(
        planner_combo_->currentData().toString().toStdString(),
        controller_combo_->currentData().toString().toStdString(),
        map_path.toStdString(), robot_radius_->value(), inflation_radius_->value(), &err);
    apply_btn_->setEnabled(true);
    np_status_->setText((ok ? tr("✅ 已应用:%1") : tr("❌ 失败:%1")).arg(QString::fromStdString(err)));
    if (ok) {
      QMessageBox::information(this, tr("导航已重启"),
          tr("Nav2 已按新算法重启。\n\n"
             "下一步:先在地图上设初始位姿(Shift+左键点小车位置,或用 📍按钮),\n"
             "等导航页「定位」那行变成 ✅ 已定位(或粒子云聚拢),\n"
             "再拖目标 → 点「开始导航」。"));
    }
  });
}

NavigationPanel::~NavigationPanel() = default;

void NavigationPanel::onNavigationStateChanged(int state) {
  using S = RobotController::NavigationState;
  switch (static_cast<S>(state)) {
    case S::IDLE:       state_label_->setText(tr("状态:空闲"));     break;
    case S::ACTIVE:     state_label_->setText(tr("状态:导航中…")); break;
    case S::PAUSED:     state_label_->setText(tr("状态:已暂停"));   break;
    case S::SUCCEEDED:  state_label_->setText(tr("状态:✓ 已到达")); break;
    case S::FAILED:     state_label_->setText(tr("状态:✗ 失败"));   break;
    case S::CANCELLED:  state_label_->setText(tr("状态:已取消"));   break;
    case S::STOPPED:    state_label_->setText(tr("状态:已停止"));   break;
  }
}

void NavigationPanel::onNavigationDistanceRemaining(double d) {
  dist_label_->setText(tr("剩余距离: %1 m").arg(d, 0, 'f', 2));
}

}  // namespace rcj
