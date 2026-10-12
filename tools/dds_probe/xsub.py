#!/usr/bin/env python3
# 通用接收端: python3 xsub.py <话题> [秒数]
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

topic = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0

rclpy.init()
n = Node('xsub')
got = []
n.create_subscription(String, topic, lambda m: got.append(m.data), 10)
print('  [SUB] topic=%s domain=%s rmw=%s' % (
    topic, os.environ.get('ROS_DOMAIN_ID', '0'),
    os.environ.get('RMW_IMPLEMENTATION', '?')), flush=True)

t0 = time.time()
first = None
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.05)
    if got and first is None:
        first = time.time() - t0
print('  [SUB] 收到 %d 条, 首帧 %s' % (
    len(got), ('%.1fs' % first) if first is not None else '无'), flush=True)
n.destroy_node()
rclpy.shutdown()
