#pragma once
#include <QWidget>
#include <QPointF>

namespace rcj {
class JoystickWidget : public QWidget {
  Q_OBJECT
public:
  explicit JoystickWidget(QWidget* parent = nullptr);

signals:
  void moved(double x, double y);   // normalised [-1, 1]

protected:
  void paintEvent(QPaintEvent* e) override;
  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;
  void resizeEvent(QResizeEvent* e) override;

private:
  void updateFromMouse(const QPoint& p);
  QPointF stick_pos_;
  QPointF center_;
  int base_radius_{80};
  int stick_radius_{20};
  bool pressed_{false};
};

}  // namespace rcj
