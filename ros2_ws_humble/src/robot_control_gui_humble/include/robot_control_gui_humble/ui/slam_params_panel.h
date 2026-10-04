// SPDX-License-Identifier: MIT
#pragma once

#include <QWidget>

#include <map>
#include <memory>
#include <string>

class QDoubleSpinBox;
class QPushButton;
class QLabel;

namespace rcj {

class RobotController;
class SlamParamClient;

// Curated slam_toolbox parameter editor, embedded in the 建图 tab.
//
// Only `minimum_travel_distance` is honoured at runtime by slam_toolbox; the
// rest are read once at configure time, so changing them needs a SLAM restart.
// "应用并重启 SLAM" sends the values to the robot's slam_controller (which
// writes a params YAML and relaunches); "应用(动态)" is the best-effort
// runtime set with a per-parameter result.
class SlamParamsPanel : public QWidget {
  Q_OBJECT
public:
  explicit SlamParamsPanel(std::shared_ptr<RobotController> c, QWidget* parent = nullptr);
  ~SlamParamsPanel() override;

private:
  void buildUi();
  std::map<std::string, double> currentValues() const;
  void readValues();
  void applyDynamic();
  void applyRestart();

  std::shared_ptr<RobotController> controller_;
  std::unique_ptr<SlamParamClient> client_;

  QDoubleSpinBox* spin_interval_{nullptr};
  QDoubleSpinBox* spin_resolution_{nullptr};
  QDoubleSpinBox* spin_laser_range_{nullptr};
  QDoubleSpinBox* spin_tf_period_{nullptr};
  QDoubleSpinBox* spin_min_travel_{nullptr};

  QPushButton* read_btn_{nullptr};
  QPushButton* apply_btn_{nullptr};
  QPushButton* restart_btn_{nullptr};
  QLabel*      status_{nullptr};
};

}  // namespace rcj
