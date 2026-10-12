#pragma once
#include <QMainWindow>
#include <QMap>
#include <memory>

class QTabWidget;
class QDockWidget;
class QComboBox;
class QLineEdit;
class QPushButton;
class QLabel;
class QStackedWidget;
class QWidget;
class QHBoxLayout;
#include "robot_control_gui_humble/ui/speed_dashboard.h"

namespace rcj {
class RobotController;
class ControlPanel;
class NavigationPanel;
class MappingPanel;
class SettingsPanel;
class RobotView;
class RobotStatusPanel;
class TeleopPanel;
class MapEditPanel;
class LogPanel;

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

  // Wire up the panel signal/slot connections. Called after the controller
  // exists (so it runs only once connect() succeeds).
  void wireUpController();

  // 内置使用说明(也用于自动化测试:设 RCJ_SHOW_HELP 即可弹出)
  void showHelp();

protected:
  void keyPressEvent(QKeyEvent* e) override;
  void keyReleaseEvent(QKeyEvent* e) override;
  void timerEvent(QTimerEvent* e) override;

public slots:
  // 公开给 main.cpp:设了 RCJ_AUTOCONNECT_IP 时自动连接(便于自动化测试)
  void onConnectClicked();

private slots:
  void onDisconnectClicked();

private:
  void setupUi();           // builds window with placeholder + tabs (disabled)
  void setupToolBar();
  void updateKeyboardControl();

  // Cached pose for laser scan → world transform
  double cur_x_{0.0}, cur_y_{0.0}, cur_yaw_{0.0};

  // Connection state
  bool connected_{false};
  // Set while the map editor is active: freezes live /map updates so
  // incoming OccupancyGrid messages can't clobber the edit buffer.
  bool map_locked_{false};
  QPushButton* connect_btn_{nullptr};
  QPushButton* disconnect_btn_{nullptr};
  QPushButton* reset_view_btn_{nullptr};
  QPushButton* help_btn_{nullptr};
  QLabel*      rotation_label_{nullptr};
  QLabel*      status_label_{nullptr};
  QLabel*      ip_label_{nullptr};
  QWidget*     placeholder_{nullptr};   // shown when disconnected
  QStackedWidget* body_stack_{nullptr}; // placeholder <-> tabs+view
  // 连接页中央那格(页签列 : 地图视图 = 3 : 2)。存成成员是为了断开→重连时
  // 能可靠地换掉地图视图 —— 以前用「布局 item 数 >= 2」判断,断开删视图后
  // Qt 会自动摘掉该 item,count 变 1,重连时就再也不建视图了(右侧空白)。
  QWidget*     center_page_{nullptr};
  QHBoxLayout* center_lay_{nullptr};
  QWidget*     view_placeholder_{nullptr};

  // Live robot controller (null until connected)
  std::shared_ptr<RobotController> controller_;
  std::unique_ptr<RobotView>      view_;
  std::unique_ptr<ControlPanel>   control_panel_;
  std::unique_ptr<NavigationPanel> nav_panel_;
  std::unique_ptr<MappingPanel>   mapping_panel_;
  std::unique_ptr<SettingsPanel>  settings_panel_;
  std::unique_ptr<RobotStatusPanel> status_panel_;
  std::unique_ptr<TeleopPanel>    teleop_panel_;
  std::unique_ptr<MapEditPanel>   map_edit_panel_;
  std::unique_ptr<LogPanel>       log_panel_;

  SpeedDashboard* footer_dashboard_{nullptr};
  QDockWidget*    display_options_dock_{nullptr};
  QComboBox*      robot_selector_{nullptr};
  QLineEdit*      namespace_edit_{nullptr};
  int             ui_tick_timer_{0};

  // 图层保鲜:代价地图/粒子云是 Nav2 和 AMCL 在「持续」发布的,停了就再也没有
  // 下一帧 —— 于是最后一帧永远糊在地图上,看着像 Nav2 还在跑。
  // 这里记下每类消息最后一次到达的时刻,超时(见 kLayerStaleMs)就当作对端没了,
  // 把该层清掉。0 = 从没收到过,不用清。
  // 注:路径(/plan)不参与保鲜 —— Nav2 只在规划的那一下发一次,不是周期发布,
  // 按超时清会把正在走的绿线在中途抹掉。
  qint64 last_global_costmap_ms_{0};
  qint64 last_local_costmap_ms_{0};
  qint64 last_particle_ms_{0};

  QMap<int, bool> key_states_;
  double max_linear_{0.5};
  double max_angular_{1.0};

  QTabWidget* tabs_{nullptr};
};

}  // namespace rcj
