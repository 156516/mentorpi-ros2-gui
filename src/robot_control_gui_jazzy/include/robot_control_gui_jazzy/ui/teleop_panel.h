#pragma once
#include <QWidget>
#include <memory>

namespace rcj {
class RobotController;

class TeleopPanel : public QWidget {
  Q_OBJECT
public:
  explicit TeleopPanel(std::shared_ptr<RobotController> c, QWidget* parent = nullptr);

private:
  std::shared_ptr<RobotController> controller_;
};

}  // namespace rcj
