#pragma once
#include <QWidget>

namespace rcj {
class SpeedDashboard : public QWidget {
  Q_OBJECT
public:
  explicit SpeedDashboard(QWidget* parent = nullptr);

public slots:
  void setLinearSpeed(double mps);    // 前进/后退 v
  void setLateralSpeed(double mps);   // 左右平移 vy(麦轮)

  void setAngularSpeed(double radps);

protected:
  void paintEvent(QPaintEvent* e) override;

private:
  double lin_{0.0};
  double lat_{0.0};
  double ang_{0.0};
};

}  // namespace rcj
