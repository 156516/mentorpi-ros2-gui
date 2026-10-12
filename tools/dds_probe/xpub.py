#!/usr/bin/env python3
# 通用发送端: python3 xpub.py <话题> [秒数]
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

topic = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

rclpy.init()
n = Node('xpub')
p = n.create_publisher(String, topic, 10)
print('  [PUB] topic=%s domain=%s rmw=%s' % (
    topic, os.environ.get('ROS_DOMAIN_ID', '0'),
    os.environ.get('RMW_IMPLEMENTATION', '?')), flush=True)

t0 = time.time()
i = 0
while time.time() - t0 < secs:
    m = String()
    m.data = '%d' % i
    p.publish(m)
    i += 1
    time.sleep(0.2)
    rclpy.spin_once(n, timeout_sec=0.001)
print('  [PUB] 共发出 %d 条' % i, flush=True)
n.destroy_node()
rclpy.shutdown()
