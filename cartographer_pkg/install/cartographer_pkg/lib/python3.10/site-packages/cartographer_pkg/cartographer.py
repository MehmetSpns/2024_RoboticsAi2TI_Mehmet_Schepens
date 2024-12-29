import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class ObstacleAvoidanceAndMapping(Node):
    def __init__(self):
        super().__init__('cartographer_node')

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for LaserScan data
        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Timer for sending velocity commands
        self.timer = self.create_timer(0.1, self.motion)

        # Parameters for motion and obstacle avoidance
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.cmd = Twist()

    def laser_callback(self, msg):
        # Extract relevant distance ranges
        self.front_distance = min(msg.ranges[345:359] + msg.ranges[0:15])
        self.left_distance = min(msg.ranges[75:105])
        self.right_distance = min(msg.ranges[255:285])

    def motion(self):
        if self.front_distance < 0.5:  # Obstacle detected
            self.cmd.linear.x = 0.0
            if self.left_distance > self.right_distance:
                self.cmd.angular.z = 0.5  # Turn left
            else:
                self.cmd.angular.z = -0.5  # Turn right
        else:
            self.cmd.linear.x = 0.2  # Move forward
            self.cmd.angular.z = 0.0  # Go straight

        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceAndMapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
