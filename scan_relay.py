#!/usr/bin/env python3
"""把小车发的 /scan_raw 转发成 slam_toolbox 要的 /scan。

【为什么不能只是转发】
mentorpi 的雷达驱动每帧点数不固定(实测在 445~448 之间逐帧乱跳),而且
angle_increment 是驱动按「这一帧收到多少点」重算的,也跟着变。

slam_toolbox 用的 OpenKarto 在**注册雷达时按第一帧把点数定死**,之后每帧都
必须等于那个数,否则 LaserRangeFinder::Validate 校验失败、**整帧被拒收**:

    [slam_toolbox]: LaserRangeScan contains 447 range readings, expected 446

实测 12 帧里只有 2 帧点数正好等于 446 → 约 83% 的帧被丢掉,图基本建不出来
(症状:雷达在发、TF 全正常、odom 在动,/map 却一动不动;日志刷屏上面那句)。
参见 slam_toolbox issue #789 / #702 / #63 —— 维护者的结论是「扫描元数据不一致
属于畸形数据」,只能在数据源侧修。

【怎么修】
把每帧按它自己声明的 (angle_min, angle_increment) 重采样到一个**固定的**
角网格上输出。这样:输出点数恒定、angle_min/angle_increment 也恒定,
每一帧的元数据完全一致,Validate 必过。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

# 固定输出点数。446 是实测里出现最多的值,也是本车雷达的名义线数。
OUT_N = 446
# 输出网格锚定在一个常量角上(而不是沿用每帧抖动的 angle_min),
# 这样每帧的元数据逐字节一致;各帧的读数仍按它们自己声明的角度摆放,
# 所以不会引入额外误差。
OUT_ANGLE_MIN = 0.0
OUT_ANGLE_INCREMENT = 2.0 * math.pi / OUT_N


class Relay(Node):
    def __init__(self):
        super().__init__('scan_raw_to_scan_relay')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(LaserScan, '/scan', qos)
        self.create_subscription(LaserScan, '/scan_raw', self.cb, qos)
        self.n = 0
        self.fixed = 0
        self.seen = {}
        self.get_logger().info(
            'relay /scan_raw -> /scan started (归一化到 %d 点)' % OUT_N)

    def cb(self, m):
        out = self.normalize(m)
        if out is None:          # 元数据不可用,原样转发,至少不让数据断流
            self.pub.publish(m)
            self.fixed += 1
        else:
            self.pub.publish(out)
        self.n += 1
        n_in = len(m.ranges)
        self.seen[n_in] = self.seen.get(n_in, 0) + 1
        if self.n % 50 == 0:
            self.get_logger().info(
                'relayed %d scans (归一化失败 %d 帧; 输入点数分布 %s)'
                % (self.n, self.fixed, sorted(self.seen.items())))

    def normalize(self, m):
        """按输入自己的角度元数据重采样到固定网格。"""
        rng = m.ranges
        n = len(rng)
        inc_in = m.angle_increment
        if n < 8 or not (inc_in > 0.0) or not math.isfinite(inc_in):
            return None

        out = LaserScan()
        out.header = m.header
        out.angle_min = OUT_ANGLE_MIN
        out.angle_increment = OUT_ANGLE_INCREMENT
        out.angle_max = OUT_ANGLE_MIN + (OUT_N - 1) * OUT_ANGLE_INCREMENT
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max

        # 输入只覆盖一圈,目标角可能落在输入起点之前 → 折回(n 个读数首尾相接)。
        base = m.angle_min
        vals = []
        for i in range(OUT_N):
            a = OUT_ANGLE_MIN + i * OUT_ANGLE_INCREMENT
            j = int(round((a - base) / inc_in)) % n
            v = rng[j]
            # REP-117:无效读数应该是 +inf。驱动会用 0.0 表示"没测到",
            # 原样留着会被当成"传感器原点处有障碍",必须在建图前抹掉。
            vals.append(v if (math.isfinite(v) and v > 0.0) else float('inf'))
        out.ranges = vals

        if len(m.intensities) == n:
            out.intensities = [m.intensities[int(round(
                (OUT_ANGLE_MIN + i * OUT_ANGLE_INCREMENT - base) / inc_in)) % n]
                for i in range(OUT_N)]
        return out


rclpy.init()
rclpy.spin(Relay())
