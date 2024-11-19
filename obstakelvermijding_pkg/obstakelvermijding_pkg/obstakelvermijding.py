import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random

class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstakelvermijding')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.motion)

        self.laser_forward = 1.0 
        self.cmd = Twist()

    def laser_callback(self, msg):
        self.laser_forward = min(msg.ranges[345:359] + msg.ranges[0:15])

    def motion(self):
        self.get_logger().info(f"Forward distance: {self.laser_forward:.2f} m")

        if self.laser_forward < 0.7: 
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = random.choice([0.5, -0.5])  
        else:
            self.cmd.linear.x = 0.2  
            self.cmd.angular.z = random.uniform(-1.0, 1.0)  

        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
