#!/usr/bin/env python3
# 看门狗逻辑自测(安全):跑在假话题 /wd_test + ROS_DOMAIN_ID=42,
# 与小车真实话题 controller/cmd_vel(domain 0)完全隔离。
# 发一条非零速度,然后什么都不发,看门狗应该在 timeout 后补一条零速度,
# 且只补一条(不重复刷)。
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

TOPIC = '/wd_test'


class Tester(Node):
    def __init__(self):
        super().__init__('wd_test_probe')
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, TOPIC, qos)
        self.create_subscription(Twist, TOPIC, self.cb, qos)
        self.rx = []

    def cb(self, m):
        self.rx.append((time.monotonic(), m.linear.x, m.linear.y, m.angular.z))


def main():
    rclpy.init()
    n = Tester()

    # 1) 先 spin 3 秒,等看门狗那边的订阅建立(不然发的消息没人收)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.1)

    # 2) 发一条非零速度,之后彻底静默
    msg = Twist()
    msg.linear.x = 0.3
    n.pub.publish(msg)
    t1 = time.monotonic()
    print('已发非零速度 linear.x=0.3,之后静默 3 秒 ...')

    while time.monotonic() - t1 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.1)

    print('共收到 %d 条:' % len(n.rx))
    zeros = 0
    for t, lx, ly, az in n.rx:
        tag = '  <== 看门狗补发的零速度' if (lx == 0 and ly == 0 and az == 0) else ''
        if tag:
            zeros += 1
        print('  t+%.2fs  linear.x=%.3f linear.y=%.3f angular.z=%.3f%s'
              % (t - t1, lx, ly, az, tag))

    print()
    if zeros == 1:
        print('✅ 结论:看门狗在静默后补了【1】条零速度,且不重复 —— 符合预期')
    elif zeros == 0:
        print('❌ 结论:没收到零速度 —— 看门狗没工作(或 QoS 不匹配),检查日志')
    else:
        print('⚠ 结论:收到 %d 条零速度 —— 应该只发一条,检查 armed 标志' % zeros)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
