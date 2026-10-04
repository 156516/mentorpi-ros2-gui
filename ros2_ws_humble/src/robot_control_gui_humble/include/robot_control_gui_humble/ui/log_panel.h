// SPDX-License-Identifier: MIT
#pragma once

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <memory>

class QTableWidget;
class QComboBox;
class QCheckBox;
class QLabel;

namespace rcj {

class RobotController;

// 「日志」页 —— 两块日志合在一起看 + 导出:
//   1. 小车的节点日志:订阅 /rosout(rcl_interfaces/Log)
//   2. GUI 自己的事件:装 Qt message handler(qInfo/qWarning/qCritical 都会进来)
class LogPanel : public QWidget {
  Q_OBJECT
public:
  explicit LogPanel(std::shared_ptr<RobotController> c, QWidget* parent = nullptr);
  ~LogPanel() override;

  // 供其它面板写 GUI 日志(线程安全:内部转成 Qt 消息)
  static void note(const QString& text);
  // 按"节点名"归类写一条(节点名决定它落到哪个分类),供其它面板调用
  static void noteAs(const QString& node, const QString& text);

  // 给 Qt 消息处理器用(必须在 GUI 线程调用)
  void addRow(const QString& source, int level, const QString& node, const QString& text);

private:
  void buildUi();
  void exportLog();
  void clearLog();

  std::shared_ptr<RobotController> controller_;
  // 必须存住!rclcpp 的 create_subscription 返回的 SharedPtr 一旦丢弃,订阅就被销毁
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  QTableWidget* table_{nullptr};
  QComboBox*    level_filter_{nullptr};
  QComboBox*    category_filter_{nullptr};
  QCheckBox*    autoscroll_{nullptr};
  QLabel*       count_label_{nullptr};
  int           rows_{0};
};

}  // namespace rcj
