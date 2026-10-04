#pragma once
#include <QWidget>

namespace rcj {
class SpeedDashboard : public QWidget {
  Q_OBJECT
public:
  explicit SpeedDashboard(QWidget* parent = nullptr);

public slots:
  void setLinearSpeed(double mps);
  void setAngularSpeed(double radps);

protected:
  void paintEvent(QPaintEvent* e) override;

private:
  double lin_{0.0};
  double ang_{0.0};
};

}  // namespace rcj
