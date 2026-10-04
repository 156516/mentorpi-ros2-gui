import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

class Relay(Node):
    def __init__(self):
        super().__init__('scan_raw_to_scan_relay')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(LaserScan, '/scan', qos)
        self.create_subscription(LaserScan, '/scan_raw', self.cb, qos)
        self.n = 0
        self.get_logger().info('relay /scan_raw -> /scan started')
    def cb(self, m):
        self.pub.publish(m)
        self.n += 1
        if self.n % 50 == 0:
            self.get_logger().info(f'relayed {self.n} scans')

rclpy.init(); rclpy.spin(Relay())
