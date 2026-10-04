// SPDX-License-Identifier: MIT
#include "robot_control_gui_jazzy/ros/robot_controller.h"

#include <QMetaObject>
#include <cmath>
#include <limits>
#include <unistd.h>      // getpid()

#include "robot_control_gui_jazzy/ros/topic_names.h"
#include "robot_control_gui_jazzy/ros/diagnostics_watcher.h"

namespace rcj {

RobotController::RobotController(std::shared_ptr<rclcpp::Node> node, QObject* parent)
    : QObject(parent), node_(std::move(node)) {
  // Reuse the node passed in (it's already added to the global executor).
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // SAFETY: only set up SUBSCRIBERS in the constructor.
  // Publishers are created lazily on first use, so just starting the GUI
  // never publishes anything to the robot — including /cmd_vel=0 which
  // can otherwise latch "stopped" commands to chassis controllers.
  setupSubscribersOnly();
  diag_watcher_ = std::make_shared<DiagnosticsWatcher>(node_);
  connect(diag_watcher_.get(), &DiagnosticsWatcher::updated,
          this, &RobotController::diagnosticsUpdated);
}

RobotController::~RobotController() = default;

void RobotController::setupSubscribersOnly() {
  using std::placeholders::_1;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      topics::odom().toStdString(), rclcpp::SensorDataQoS(),
      std::bind(&RobotController::onOdom, this, _1));

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      topics::scan().toStdString(), rclcpp::SensorDataQoS(),
      std::bind(&RobotController::onScan, this, _1));

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topics::mapTopic().toStdString(), rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      std::bind(&RobotController::onMap, this, _1));

  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      topics::globalPath().toStdString(), qos,
      std::bind(&RobotController::onPath, this, _1));

  // mentorpi's /ros_robot_controller/battery publishes std_msgs/UInt16
  // (voltage * 10); the standard BatteryState type is not used on this
  // stack. Subscribe to UInt16 and synthesise a BatteryState in onBatteryU16.
  bat_u16_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      topics::batteryState().toStdString(), rclcpp::SensorDataQoS(),
      std::bind(&RobotController::onBatteryU16, this, _1));

  diag_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      topics::diagnostics(), qos,
      std::bind(&RobotController::onDiagnostics, this, _1));

  // Publishers (cmd_vel, initial_pose) are NOT created here.
  // See publishVelocity() / sendInitialPose() for lazy creation.
}

void RobotController::setupActionClients() {
  // nav_client is harmless: it just discovers the action server.
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node_, topics::navigateToPose().toStdString());

  // save_map_client is created lazily (only when user clicks "Save Map").
}

void RobotController::publishVelocity(double linear, double angular) {
  publishFullVelocity(linear, 0.0, angular);
}

void RobotController::publishFullVelocity(double lin_x, double lin_y, double ang_z) {
  // Lazy-create the cmd_vel publisher on first use.
  // RELIABLE is required: mentorpi's chassis subscribes RELIABLE,
  // and ROS2 silently drops messages if QoS doesn't match. (Confirmed
  // by a "New subscription discovered ... incompatible QoS" warning.)
  if (!cmd_vel_pub_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        topics::cmdVel().toStdString(), rclcpp::QoS(rclcpp::KeepLast(10)));
    RCLCPP_INFO(node_->get_logger(), "cmd_vel publisher lazily created on first use");
  }
  geometry_msgs::msg::Twist t;
  t.linear.x  = lin_x;
  t.linear.y  = lin_y;
  t.angular.z = ang_z;
  cmd_vel_pub_->publish(t);
}

void RobotController::setLinearVelocity(double v)  { linear_  = v;  publishVelocity(linear_, angular_); }
void RobotController::setAngularVelocity(double w) { angular_ = w;  publishVelocity(linear_, angular_); }
void RobotController::emergencyStop()              { linear_ = angular_ = 0.0; publishVelocity(0, 0); }

void RobotController::sendNavigateToPose(const geometry_msgs::msg::PoseStamped& goal) {
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_->get_logger(), "NavigateToPose action server not available");
    return;
  }
  nav2_msgs::action::NavigateToPose::Goal g;
  g.pose = goal;
  nav_state_ = NavigationState::ACTIVE;
  emit navigationStateChanged(static_cast<int>(nav_state_));

  using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  opts.goal_response_callback =
    [this](const std::shared_ptr<GoalHandle>& gh) {
      if (!gh) {
        RCLCPP_WARN(node_->get_logger(), "Nav goal rejected");
        nav_state_ = NavigationState::FAILED;
        emit navigationStateChanged(static_cast<int>(nav_state_));
      } else {
        RCLCPP_INFO(node_->get_logger(), "Nav goal accepted");
        current_goal_handle_ = gh;
      }
    };
  opts.feedback_callback =
    [this](std::shared_ptr<GoalHandle> /*gh*/,
           std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> fb) {
      emit navigationDistanceRemaining(fb->distance_remaining);
    };
  opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& r) {
      switch (r.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          nav_state_ = NavigationState::SUCCEEDED; break;
        case rclcpp_action::ResultCode::ABORTED:
          nav_state_ = NavigationState::FAILED; break;
        case rclcpp_action::ResultCode::CANCELED:
          nav_state_ = NavigationState::CANCELLED; break;
        default:
          nav_state_ = NavigationState::IDLE; break;
      }
      emit navigationStateChanged(static_cast<int>(nav_state_));
    };
  nav_client_->async_send_goal(g, opts);
}

void RobotController::cancelNavigation() {
  nav_client_->async_cancel_all_goals();
  nav_state_ = NavigationState::CANCELLED;
  emit navigationStateChanged(static_cast<int>(nav_state_));
}

void RobotController::sendInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
  if (!initial_pose_pub_) {
    initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(rclcpp::KeepLast(10)));
    RCLCPP_INFO(node_->get_logger(), "initialpose publisher lazily created");
  }
  initial_pose_pub_->publish(pose);
  RCLCPP_INFO(node_->get_logger(), "Initial pose published to /initialpose");
  clearAllCostmaps();
}

void RobotController::clearAllCostmaps() {
  auto clear_one = [this](const QString& name) {
    auto client = node_->create_client<std_srvs::srv::Empty>(name.toStdString());
    if (!client->wait_for_service(std::chrono::milliseconds(200))) return;
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    client->async_send_request(req);
  };
  clear_one(topics::clearCostmapGlobal());
  clear_one(topics::clearCostmapLocal());
}

void RobotController::requestSaveMap(const QString& path) {
  if (!save_map_client_) {
    save_map_client_ = node_->create_client<nav2_msgs::srv::SaveMap>(
        topics::saveMapService().toStdString());
  }
  if (!save_map_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_->get_logger(), "/map_saver/save_map service not available");
    return;
  }
  auto req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  req->map_url      = path.toStdString();
  req->image_format = "pgm";
  req->map_mode     = "trinary";
  save_map_client_->async_send_request(req,
    [this](rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedFuture f) {
      if (f.get()->result) RCLCPP_INFO(node_->get_logger(), "Map saved");
      else                RCLCPP_ERROR(node_->get_logger(), "Map save failed");
    });
}

geometry_msgs::msg::Pose RobotController::currentPose() const {
  std::lock_guard<std::mutex> g(mtx_);
  return current_pose_;
}
nav_msgs::msg::OccupancyGrid RobotController::latestMap() const {
  std::lock_guard<std::mutex> g(mtx_);
  return latest_map_;
}
sensor_msgs::msg::BatteryState RobotController::latestBattery() const {
  std::lock_guard<std::mutex> g(mtx_);
  return latest_battery_;
}

void RobotController::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> g(mtx_);
    current_pose_ = msg->pose.pose;
  }
  emit odomUpdated(*msg);
}
void RobotController::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  emit scanUpdated(*msg);
}
void RobotController::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> g(mtx_);
    latest_map_ = *msg;
  }
  emit mapUpdated(*msg);
}
void RobotController::onPath(const nav_msgs::msg::Path::SharedPtr msg) {
  emit pathUpdated(*msg);
}
void RobotController::onBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> g(mtx_);
    latest_battery_ = *msg;
  }
  emit batteryUpdated(*msg);
}

void RobotController::onBatteryU16(const std_msgs::msg::UInt16::SharedPtr msg) {
  // mentorpi publishes battery voltage in MILLIVOLTS as UInt16
  // (e.g. 7422 -> 7.422 V). Synthesise a BatteryState for the UI.
  sensor_msgs::msg::BatteryState b;
  b.header.stamp = rclcpp::Clock().now();
  b.header.frame_id = "base_link";
  b.voltage = static_cast<float>(msg->data) / 1000.0f;
  b.current = 0.0f;
  b.charge  = std::numeric_limits<float>::quiet_NaN();
  b.capacity = std::numeric_limits<float>::quiet_NaN();
  b.design_capacity = std::numeric_limits<float>::quiet_NaN();
  b.percentage = std::numeric_limits<float>::quiet_NaN();
  b.power_supply_status = b.POWER_SUPPLY_STATUS_DISCHARGING;
  b.power_supply_health = b.POWER_SUPPLY_HEALTH_GOOD;
  b.power_supply_technology = b.POWER_SUPPLY_TECHNOLOGY_LIPO;
  b.present = true;
  {
    std::lock_guard<std::mutex> g(mtx_);
    latest_battery_ = b;
  }
  emit batteryUpdated(b);
}
void RobotController::onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
  emit diagnosticsUpdated(*msg);
}

// ---------------------------------------------------------------------------
// Static helpers: forward to the inline implementations in ros_to_qt.h so
// callers using robot_controller.h still get the static methods. The actual
// code is in ros_to_qt.h so unit tests don't need to link this TU.
// ---------------------------------------------------------------------------

#include "robot_control_gui_jazzy/ros/ros_to_qt.h"
// Trampoline static class methods → namespaced inline free functions.
// Free functions live in `rcj::`, class statics live in `rcj::RobotController::`,
// so the names are distinct and we just delegate.
QImage RobotController::occupancyGridToImage(const nav_msgs::msg::OccupancyGrid& g) {
  return rcj::occupancyGridToImage(g);
}
QVector<QPointF> RobotController::laserScanToPoints(const sensor_msgs::msg::LaserScan& s,
                                                    double x, double y, double yw, int step) {
  return rcj::laserScanToPoints(s, x, y, yw, step);
}
QVector<QPointF> RobotController::pathToPoints(const nav_msgs::msg::Path& p) {
  return rcj::pathToPoints(p);
}

// ---------------------------------------------------------------------------
// TF-based pose lookup + laser scan projection
// (uses only tf2 core, not tf2_geometry_msgs, so the `using namespace`
//  inside that header doesn't pollute our namespace.)
// ---------------------------------------------------------------------------

bool RobotController::tryGetRobotPoseInMap(double& x, double& y, double& yaw) {
  if (!tf_buffer_) return false;
  const auto target_frame = topics::mapFrame().toStdString();
  const auto source_frame = topics::baseFrame().toStdString();
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "TF map->%s failed: %s", source_frame.c_str(), ex.what());
    return false;
  }
  x   = tf.transform.translation.x;
  y   = tf.transform.translation.y;
  const auto& q = tf.transform.rotation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
  return true;
}

QVector<QPointF> RobotController::laserScanToMap(const sensor_msgs::msg::LaserScan& scan) {
  QVector<QPointF> out;
  if (!tf_buffer_) return out;

  const auto target = topics::mapFrame().toStdString();
  const auto source = topics::baseFrame().toStdString();
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(target, source,
                                     tf2::TimePoint(std::chrono::nanoseconds(
                                         rclcpp::Time(scan.header.stamp).nanoseconds())));
  } catch (const tf2::TransformException&) {
    try {
      tf = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
    } catch (...) {
      return out;
    }
  }

  const double tx = tf.transform.translation.x;
  const double ty = tf.transform.translation.y;
  const auto& q = tf.transform.rotation;
  tf2::Quaternion q_tf;
  q_tf.setW(q.w); q_tf.setX(q.x); q_tf.setY(q.y); q_tf.setZ(q.z);
  const tf2::Matrix3x3 rot(q_tf);

  out.reserve(scan.ranges.size() / 4);
  for (size_t i = 0; i < scan.ranges.size(); i += 4) {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double lx = r * std::cos(a);
    const double ly = r * std::sin(a);
    tf2::Vector3 p(lx, ly, 0.0);
    tf2::Vector3 w = rot * p;
    out << QPointF(w.x() + tx, w.y() + ty);
  }
  return out;
}

}  // namespace rcj
