#!/usr/bin/env python3
# 单话题聚焦探针:一次只订一个话题,跑长窗口,看端点匹配到底要多久。
# 用法: python3 probe_one.py <话题> <秒数>
import sys
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
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
    '/controller/cmd_vel': 'geometry_msgs/msg/Twist',
    '/ascamera/camera_publisher/rgb0/image_compressed': 'sensor_msgs/msg/CompressedImage',
}

PROFILES = {
    'BE': QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE),
    'REL': QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE),
    'TL': QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL),
}

topic = sys.argv[1]
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

mod, cls = TYPES[topic].split('/msg/')
T = getattr(__import__(mod.replace('/', '.') + '.msg', fromlist=[cls]), cls)

rclpy.init()
n = Node('one_probe')
counts = {}
first = {}


def mk(label):
    def cb(msg):
        counts[label] = counts.get(label, 0) + 1
        if label not in first:
            first[label] = time.time() - t0
    return cb


for label, qos in PROFILES.items():
    counts[label] = 0
    n.create_subscription(T, topic, mk(label), qos)

t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)

print('  %-46s' % topic, end='')
for label in PROFILES:
    f = first.get(label)
    print('  %s=%d(首帧%.1fs)' % (label, counts[label], f if f is not None else -1), end='')
print()
n.destroy_node()
rclpy.shutdown()
