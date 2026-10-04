# 判定渲染轴向的假场景:地图 10x7m,原点(-5,-3)
#  - 左下角方块(数据行10-25,列10-25)  -> 世界 x[-4.5,-3.75] y[-2.5,-1.75]  = 左下
#  - 右上角方块(数据行115-130,列175-190)-> 世界 x[3.75,4.5] y[2.75,3.5]    = 右上
#  - 沿底边的一条黑线(数据行2)                                             = 底边
#  - 机器人 odom 放在 (3.0, 2.0) = 偏右偏上
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry

W,H,RES,OX,OY = 200,140,0.05,-5.0,-3.0

class Fake(Node):
    def __init__(self):
        super().__init__('fake_axis')
        mq=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        q=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        self.mp=self.create_publisher(OccupancyGrid,'/map',mq)
        self.op=self.create_publisher(Odometry,'/odom',q)
        self.pub_map(); self.create_timer(0.2,self.tick)
        self.get_logger().info('axis scene ready')
    def pub_map(self):
        m=OccupancyGrid(); i=m.info
        i.resolution=RES; i.width=W; i.height=H
        i.origin.position.x=OX; i.origin.position.y=OY; i.origin.orientation.w=1.0
        d=[0]*(W*H)
        def put(r,c,v):
            if 0<=r<H and 0<=c<W: d[r*W+c]=v
        for r in range(10,26):
            for c in range(10,26): put(r,c,100)          # 左下
        for r in range(115,131):
            for c in range(175,191): put(r,c,100)        # 右上
        for c in range(W): put(2,c,100)                  # 底边横线
        m.data=d; self.mp.publish(m)
    def tick(self):
        self.pub_map()
        o=Odometry(); o.header.frame_id='odom'; o.child_frame_id='base_footprint'
        o.pose.pose.position.x=3.0; o.pose.pose.position.y=2.0
        o.pose.pose.orientation.w=1.0
        self.op.publish(o)

rclpy.init(); rclpy.spin(Fake())
