// SPDX-License-Identifier: MIT
#pragma once

#include <QWidget>

#include <memory>

#include "robot_control_gui_humble/ros/map_editor.h"

class QRadioButton;
class QDoubleSpinBox;
class QPushButton;
class QLabel;

namespace rcj {

class RobotController;
class RobotView;

// "地图编辑" tab: snapshot the CURRENT map (or import a saved PGM+YAML),
// paint erase/obstacle strokes on it with the shared RobotView, and save.
// Deliberately NOT interleaved with live mapping: entering edit mode is
// refused while SLAM is running, and MainWindow latches map_locked_ so an
// incoming /map can't clobber the edits.
class MapEditPanel : public QWidget {
  Q_OBJECT
public:
  explicit MapEditPanel(std::shared_ptr<RobotController> c,
                        RobotView* view,
                        QWidget* parent = nullptr);
  ~MapEditPanel() override;

signals:
  // MainWindow uses this to freeze live /map updates while editing.
  void editModeChanged(bool on);

private:
  void buildUi();
  bool queryIsMapping();          // true if SLAM is running (service absent => false)
  void beginEditingFromCurrentMap();
  void importFromFile();
  void renderBuffer();
  void saveToFile();
  void setEditing(bool on);

  std::shared_ptr<RobotController> controller_;
  RobotView* view_{nullptr};
  MapEditBuffer buffer_;

  QRadioButton*   draw_radio_{nullptr};
  QRadioButton*   erase_radio_{nullptr};
  QDoubleSpinBox* brush_spin_{nullptr};
  QPushButton*    enter_btn_{nullptr};
  QPushButton*    import_btn_{nullptr};
  QPushButton*    undo_btn_{nullptr};
  QPushButton*    redo_btn_{nullptr};
  QPushButton*    save_btn_{nullptr};
  QPushButton*    exit_btn_{nullptr};
  QLabel*         status_label_{nullptr};
  bool            editing_{false};
};

}  // namespace rcj
