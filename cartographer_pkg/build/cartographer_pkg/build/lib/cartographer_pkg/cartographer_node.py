import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

class CartographerNode(Node):
    def __init__(self):
        super().__init__('cartographer_node')
        self.get_logger().info("Starting Cartographer Node...")
        
        # Publisher and subscriber initialization
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # Timer for motion
        self.timer = self.create_timer(0.1, self.motion_callback)
        
        # State variables
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.current_position = None
        
        self.get_logger().info("Initialization complete!")

    def scan_callback(self, msg):
        self.front_distance = min(msg.ranges[345:359] + msg.ranges[0:15])
        self.left_distance = min(msg.ranges[75:105])
        self.right_distance = min(msg.ranges[255:285])
        self.get_logger().info(
            f"LaserScan -> Front: {self.front_distance:.2f}, "
            f"Left: {self.left_distance:.2f}, Right: {self.right_distance:.2f}"
        )

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_position = (position.x, position.y)
        self.get_logger().info(f"Odometry -> Position: x={position.x:.2f}, y={position.y:.2f}")

    def motion_callback(self):
        if self.front_distance < 0.5:  # Obstacle ahead
            self.get_logger().warning("Obstacle detected ahead. Avoiding...")
            cmd = Twist()
            if self.left_distance > self.right_distance:
                cmd.angular.z = 0.5  # Turn left
            else:
                cmd.angular.z = -0.5  # Turn right
        else:  # No obstacle
            self.get_logger().info("Path clear. Moving forward...")
            cmd = Twist()
            cmd.linear.x = 0.2  # Move forward
            cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().debug(f"Published velocity -> linear.x: {cmd.linear.x}, angular.z: {cmd.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = CartographerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
