import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.timer_period = 0.1  
        self.timer = self.create_timer(self.timer_period, self.motion)

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.cmd = Twist()

    def laser_callback(self, msg):
        self.front_distance = min(msg.ranges[345:359] + msg.ranges[0:15])
        self.left_distance = min(msg.ranges[75:105])
        self.right_distance = min(msg.ranges[255:285])

    def motion(self):
        self.get_logger().info(
            f"Distances -> Front: {self.front_distance:.2f} m, "
            f"Left: {self.left_distance:.2f} m, Right: {self.right_distance:.2f} m"
        )

        
        if self.front_distance < 0.5:   
            self.cmd.linear.x = 0.0
            if self.left_distance > self.right_distance:
                self.cmd.angular.z = 0.3  
            else:
                self.cmd.angular.z = -0.3
        else:
            self.cmd.linear.x = 0.2  
            if self.left_distance < 0.3:  
                self.cmd.angular.z = -0.2 
            elif self.right_distance < 0.3: 
                self.cmd.angular.z = 0.2  
            else:
                self.cmd.angular.z = 0.0 

        self.publisher_.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
