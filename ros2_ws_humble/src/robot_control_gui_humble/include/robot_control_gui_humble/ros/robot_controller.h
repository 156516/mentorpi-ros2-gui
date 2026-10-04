// SPDX-License-Identifier: MIT
#pragma once

#include <QObject>
#include <QString>
#include <QImage>
#include <QPointF>
#include <QVector>
#include <memory>
#include <vector>
#include <mutex>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rcj {
// Set by main.cpp so RobotController can hand its node to the global executor.
void register_node_with_executor(std::shared_ptr<rclcpp::Node> node);
void register_executor(rclcpp::executors::MultiThreadedExecutor::SharedPtr exec);

// P11 lazy connect lifecycle
// rcj_connect() builds a single ROS node, adds it to a MultiThreadedExecutor
// and starts spinning on a std::thread. Returns that node so callers can
// use it for their subscribers (no double-add).
std::shared_ptr<rclcpp::Node> rcj_connect();
void rcj_disconnect();
}  // namespace rcj

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace rcj {

class DiagnosticsWatcher;

class RobotController : public QObject {
  Q_OBJECT
public:
  enum class NavigationState {
    IDLE, ACTIVE, PAUSED, STOPPED, CANCELLED, SUCCEEDED, FAILED
  };
  Q_ENUM(NavigationState)

  // The RobotController needs a node to subscribe on. Callers should pass
  // a node that's been added to the global executor (see main_window).
  explicit RobotController(std::shared_ptr<rclcpp::Node> node, QObject* parent = nullptr);
  ~RobotController() override;

  bool isInitialized() const { return node_ != nullptr; }
  bool isNavigating() const  { return nav_state_ == NavigationState::ACTIVE; }
  bool isMapping()    const  { return mapping_; }
  std::shared_ptr<rclcpp::Node> node() const { return node_; }

  // Helpers (used by MainWindow to bridge into QGraphicsView).
  // Definitions live in ros_to_qt.h so they're testable without rclcpp.
  static QImage occupancyGridToImage(const nav_msgs::msg::OccupancyGrid& grid);
  // 代价地图 → 半透明热力图(0/-1 透明,越大越红)
  static QImage costmapToImage(const nav_msgs::msg::OccupancyGrid& grid);
  static QVector<QPointF> poseArrayToPoints(const geometry_msgs::msg::PoseArray& pa);
  static QVector<QPointF> laserScanToPoints(const sensor_msgs::msg::LaserScan& scan,
                                            double robot_x, double robot_y, double robot_yaw,
                                            int step = 1);
  static QVector<QPointF> pathToPoints(const nav_msgs::msg::Path& path);

  // TF-based laser scan projection (preferred when TF is available).
  // Returns world-frame (map) points for the scan.
  // If TF lookup fails, returns empty (caller may fall back to odom-based pose).
  QVector<QPointF> laserScanToMap(const sensor_msgs::msg::LaserScan& scan);

  // Snapshot of last-known robot pose in map frame (via TF).
  bool tryGetRobotPoseInMap(double& x, double& y, double& yaw);

  // Movement
  void publishVelocity(double linear, double angular);
  void publishFullVelocity(double lin_x, double lin_y, double ang_z);
  void setLinearVelocity(double v);
  void setAngularVelocity(double w);
  void emergencyStop();

  // Navigation
  void sendNavigateToPose(const geometry_msgs::msg::PoseStamped& goal);
  void cancelNavigation();
  void sendInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
  void clearAllCostmaps();

  // 把一张地图通过 /nav_map_upload 发给小车的 nav_controller
  // (它存成文件并作为导航地图)。用于"GUI 里建好/编辑好的图直接用于导航"。
  bool uploadNavMap(const nav_msgs::msg::OccupancyGrid& g, std::string* err);

  // Mapping (P4 stubs)
  void requestSaveMap(const QString& path);

  // Latest snapshot (thread-safe)
  geometry_msgs::msg::Pose      currentPose() const;
  nav_msgs::msg::OccupancyGrid  latestMap()   const;
  sensor_msgs::msg::BatteryState latestBattery() const;

signals:
  void odomUpdated(const nav_msgs::msg::Odometry& odom);
  void scanUpdated(const sensor_msgs::msg::LaserScan& scan);
  void mapUpdated (const nav_msgs::msg::OccupancyGrid& map);
  // Nav2 代价地图(全局/局部)。Nav2 没跑时不会发。
  void globalCostmapUpdated(const nav_msgs::msg::OccupancyGrid& map);
  void localCostmapUpdated (const nav_msgs::msg::OccupancyGrid& map);
  // AMCL 粒子云(/particlecloud)。只在 AMCL 在跑时才有。
  void particleCloudUpdated(const geometry_msgs::msg::PoseArray& cloud);
  // 摄像头画面(已解码)。收到一帧就发一次。
  void cameraImageUpdated(const QImage& image);
  void pathUpdated(const nav_msgs::msg::Path& path);
  void batteryUpdated(const sensor_msgs::msg::BatteryState& bat);
  void diagnosticsUpdated(const diagnostic_msgs::msg::DiagnosticArray& diag);

  void navigationStateChanged(int state);
  void navigationFeedback(const geometry_msgs::msg::PoseStamped& pose);
  void navigationDistanceRemaining(double remaining);

  void connectionStateChanged(bool connected);

private:
  void setupSubscribersOnly();
  void setupActionClients();

  // ROS callbacks
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onMap (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onLocalCostmap (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onParticleCloud(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void onCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void onPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void onBatteryU16(const std_msgs::msg::UInt16::SharedPtr msg);
  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<DiagnosticsWatcher> diag_watcher_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr particle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr bat_u16_sub_;  // battery_type=uint16
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bat_sub_;  // battery_type=battery_state
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;

  // Clients (P4 / P5)
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr nav_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_pub_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
      current_goal_handle_;

  // TF (P10): used for accurate scan → map projection and pose lookup.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  using NavToPose = nav2_msgs::action::NavigateToPose;
  rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;

  NavigationState nav_state_{NavigationState::IDLE};
  double linear_{0.0};
  double angular_{0.0};
  bool   mapping_{false};

  mutable std::mutex mtx_;
  geometry_msgs::msg::Pose       current_pose_;
  nav_msgs::msg::OccupancyGrid   latest_map_;
  sensor_msgs::msg::BatteryState latest_battery_;
};

}  // namespace rcj
