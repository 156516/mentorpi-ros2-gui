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
#include "robot_control_gui_jazzy/ui/speed_dashboard.h"

namespace rcj {
class RobotController;
class ControlPanel;
class NavigationPanel;
class MappingPanel;
class SettingsPanel;
class RobotView;
class RobotStatusPanel;
class TeleopPanel;

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

  // Wire up the panel signal/slot connections. Called after the controller
  // exists (so it runs only once connect() succeeds).
  void wireUpController();

protected:
  void keyPressEvent(QKeyEvent* e) override;
  void keyReleaseEvent(QKeyEvent* e) override;
  void timerEvent(QTimerEvent* e) override;

private slots:
  void onConnectClicked();
  void onDisconnectClicked();

private:
  void setupUi();           // builds window with placeholder + tabs (disabled)
  void setupToolBar();
  void updateKeyboardControl();

  // Cached pose for laser scan → world transform
  double cur_x_{0.0}, cur_y_{0.0}, cur_yaw_{0.0};

  // Connection state
  bool connected_{false};
  QPushButton* connect_btn_{nullptr};
  QPushButton* disconnect_btn_{nullptr};
  QLabel*      status_label_{nullptr};
  QLabel*      ip_label_{nullptr};
  QWidget*     placeholder_{nullptr};   // shown when disconnected
  QStackedWidget* body_stack_{nullptr}; // placeholder <-> tabs+view

  // Live robot controller (null until connected)
  std::shared_ptr<RobotController> controller_;
  std::unique_ptr<RobotView>      view_;
  std::unique_ptr<ControlPanel>   control_panel_;
  std::unique_ptr<NavigationPanel> nav_panel_;
  std::unique_ptr<MappingPanel>   mapping_panel_;
  std::unique_ptr<SettingsPanel>  settings_panel_;
  std::unique_ptr<RobotStatusPanel> status_panel_;
  std::unique_ptr<TeleopPanel>    teleop_panel_;

  SpeedDashboard* footer_dashboard_{nullptr};
  QDockWidget*    display_options_dock_{nullptr};
  QComboBox*      robot_selector_{nullptr};
  QLineEdit*      namespace_edit_{nullptr};
  int             ui_tick_timer_{0};

  QMap<int, bool> key_states_;
  double max_linear_{0.5};
  double max_angular_{1.0};

  QTabWidget* tabs_{nullptr};
};

}  // namespace rcj
