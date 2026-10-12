#!/usr/bin/env python3
# 小车容器内探针:不用 ros2 cli(会卡死),用 rclpy 直接读图。
import time

import rclpy
from rclpy.node import Node

rclpy.init()
n = Node('probe_nodes')

t0 = time.time()
while time.time() - t0 < 6.0:
    time.sleep(0.3)

nodes = sorted(n.get_node_names())
topics = n.get_topic_names_and_types()

print('=== 节点 %d 个 ===' % len(nodes))
for x in nodes:
    print('   ', x)
print()
print('=== 话题 %d 个 ===' % len(topics))
for name, types in sorted(topics):
    print('   %-42s %s' % (name, ','.join(types)))

n.destroy_node()
rclpy.shutdown()
