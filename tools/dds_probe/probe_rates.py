#!/usr/bin/env python3
# 上位机侧速率探针 v2:跨 ZeroTier 订阅小车话题。
# 每个话题同时挂三套 QoS(BEST_EFFORT/默认RELIABLE/TRANSIENT_LOCAL),
# 谁收到算谁的 —— ROS2 的默认 QoS 是 RELIABLE+VOLATILE,上一版就是漏了它。
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

TYPES = {
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/scan_raw': 'sensor_msgs/msg/LaserScan',
    '/odom': 'nav_msgs/msg/Odometry',
    '/map': 'nav_msgs/msg/OccupancyGrid',
    '/amcl_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    '/particle_cloud': 'nav2_msgs/msg/ParticleCloud',
    '/local_costmap/costmap': 'nav_msgs/msg/OccupancyGrid',
    '/global_costmap/costmap': 'nav_msgs/msg/OccupancyGrid',
    '/plan': 'nav_msgs/msg/Path',
    '/controller/cmd_vel': 'geometry_msgs/msg/Twist',
    '/ascamera/camera_publisher/rgb0/image_compressed': 'sensor_msgs/msg/CompressedImage',
    '/nav_status': 'std_msgs/msg/String',
    '/slam_status': 'std_msgs/msg/String',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}

PROFILES = [
    ('BE ', QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)),
    ('REL', QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)),
    ('TL ', QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)),
]

COUNTS = {}


def load_type(name):
    mod, cls = TYPES[name].split('/msg/')
    return getattr(__import__(mod.replace('/', '.') + '.msg', fromlist=[cls]), cls)


class Probe(Node):
    def __init__(self):
        super().__init__('rate_probe')
        for t, tn in TYPES.items():
            COUNTS[t] = {p[0]: 0 for p in PROFILES}
            try:
                T = load_type(t)
            except Exception as e:
                print('  !! %s 类型加载失败: %s' % (t, e))
                continue
            for label, qos in PROFILES:
                def cb(msg, t=t, label=label):
                    COUNTS[t][label] += 1
                try:
                    self.create_subscription(T, t, cb, qos)
                except Exception as e:
                    print('  !! %s[%s] 订阅失败: %s' % (t, label, e))


def main():
    rclpy.init()
    n = Probe()
    dur = 12.0
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.1)

    print()
    print('  %-50s %6s %6s %6s %8s' % ('话题', 'BE', 'REL', 'TL', '速率'))
    print('  ' + '-' * 82)
    for t in TYPES:
        c = COUNTS[t]
        best = max(c.values())
        hz = best / dur
        mark = '' if best else '   <<< 完全收不到'
        print('  %-50s %6d %6d %6d %7.1fHz%s'
              % (t, c['BE '], c['REL'], c['TL '], hz, mark))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
