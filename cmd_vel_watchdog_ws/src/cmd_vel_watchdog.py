#!/usr/bin/env python3
# cmd_vel_watchdog.py —— 小车端速度看门狗(安全网,最后一道防线)。
#
# 为什么需要它:
#   遥操作是「按下发一条 cmd_vel、松开发一条 cmd_vel=0」。
#   跨城链路(ZeroTier + 手机热点,RTT 数百 ms 且会丢包)上,
#   「松开」那条 0 一旦迟到或丢失,小车就按最后收到的速度一直跑 —— 实测出现过
#   (2026-10-10「控制页延迟很高,而且停不下来」)。
#   链路抖动、GUI 卡死/崩溃、进程被 OOM 杀掉,都会造成同样的后果。
#   所以停止不能只依赖「那条消息能送到」,必须由小车自己在超时后刹车。
#
# 做法:
#   订阅 controller/cmd_vel。只要收到过非零速度,之后静默超过 timeout 秒
#   (没有任何新指令),就补发一条零速度并告警。补完就进入「已刹车」状态,
#   不再重复发 —— 直到下一次收到非零指令。
#
# 为什么这样不会干扰别的:
#   - Nav2 导航走 /cmd_vel(另一条话题),这里只订 controller/cmd_vel,
#     导航期间本节点一直处于「未武装」状态,完全不插手。
#   - 速度归零后不再重复发 0,所以不会用常驻的零速度把别人的指令顶掉
#     (odom_publisher 对 /app/cmd_vel、/cmd_vel、/controller/cmd_vel 都订阅,
#      常驻 0 会把导航指令打断)。
#
# 用法(容器内):
#   python3 /home/ubuntu/ros2_ws/src/cmd_vel_watchdog.py \
#       --ros-args -p timeout:=0.8
#
# 调参:timeout 要明显大于 GUI 的重发周期(100 ms)。0.8 s 留了 8 拍余量,
#       即使连续丢几包也不会误刹;真出问题时 0.8 s × 0.3 m/s ≈ 24 cm。

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

DEFAULT_TIMEOUT = 0.8      # 秒
DEFAULT_TOPIC = 'controller/cmd_vel'
TICK = 0.05                # 检查周期,秒


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('timeout', DEFAULT_TIMEOUT)
        self.declare_parameter('topic', DEFAULT_TOPIC)
        self.timeout = float(self.get_parameter('timeout').value)
        topic = str(self.get_parameter('topic').value)

        # 必须 RELIABLE:GUI 那边 rclcpp::QoS(KeepLast(10)) 默认就是 RELIABLE,
        # QoS 不匹配会静默收不到消息(ROS2 只在发现时给一条 warning)。
        qos = QoSProfile(depth=10,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)

        self.pub = self.create_publisher(Twist, topic, qos)
        self.sub = self.create_subscription(Twist, topic, self.on_cmd, qos)

        self.last_rx = None     # 最近一次收到指令的单调时刻
        self.armed = False      # 收到过非零速度、还没刹车
        self.create_timer(TICK, self.tick)

        self.get_logger().info(
            '速度看门狗已启动: 话题=%s 静默阈值=%.2fs' % (topic, self.timeout))

    def on_cmd(self, msg):
        self.last_rx = time.monotonic()
        self.armed = (abs(msg.linear.x) + abs(msg.linear.y)
                      + abs(msg.angular.z)) > 1e-6

    def tick(self):
        if not self.armed or self.last_rx is None:
            return
        if time.monotonic() - self.last_rx > self.timeout:
            self.pub.publish(Twist())   # 全零 = 刹车
            self.armed = False          # 只补一条,之后等新指令
            self.get_logger().warn(
                'cmd_vel 静默超过 %.2fs,已补发零速度(看门狗刹车)' % self.timeout)


def main():
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
