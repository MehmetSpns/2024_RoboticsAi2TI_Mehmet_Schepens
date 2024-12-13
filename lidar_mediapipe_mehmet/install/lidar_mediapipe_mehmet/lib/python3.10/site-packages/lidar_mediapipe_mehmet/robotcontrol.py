import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

maxspeed = 0.2
turn_speed = 0.5
min_distance = 0.5  # Minimum distance to consider obstacle detected
safe_distance = 0.4  # Distance to maintain for sides

class RobotControl(Node):
    def __init__(self):
        super().__init__('lidar_navigation_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.create_subscription(String, 'hand_gesture', self.gesture_callback, 10)

        self.obstacle_detected = False
        self.current_gesture = "stop"

    def lidar_callback(self, msg):
        # Calculate distances for front, left, and right sections
        front = min(msg.ranges[0:10] + msg.ranges[-10:])
        left = min(msg.ranges[45:90])
        right = min(msg.ranges[-90:-45])

        self.obstacle_detected = front < 0.7

        cmd = Twist()

        if self.current_gesture == "start":
            if self.obstacle_detected:
                self.get_logger().info("Obstacle detected in front! Turning left.")
                cmd.linear.x = 0.0  # Stop moving forward
                cmd.angular.z = turn_speed  # Turn left
            elif left < safe_distance:
                self.get_logger().info("Obstacle detected on the left. Turning right.")
                cmd.linear.x = maxspeed  # Move forward
                cmd.angular.z = -turn_speed  # Turn right
            elif right < safe_distance:
                self.get_logger().info("Obstacle detected on the right. Turning left.")
                cmd.linear.x = maxspeed  # Move forward
                cmd.angular.z = turn_speed  # Turn left
            else:
                cmd.linear.x = maxspeed  # Move forward
                cmd.angular.z = 0.0  # Go straight
        else:
            cmd.linear.x = 0.0  # Stop moving
            cmd.angular.z = 0.0  # Stop turning

        self.publisher_.publish(cmd)

    def gesture_callback(self, msg):
        self.current_gesture = msg.data
        self.get_logger().info(f"Received gesture: {self.current_gesture}")

        if self.current_gesture == "thumb_up":
            self.spin_robot()

    def spin_robot(self):
        cmd = Twist()
        cmd.angular.z = 10.0  
        self.publisher_.publish(cmd)
        self.get_logger().info("Spinning robot.")

def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControl()
    rclpy.spin(robot_control_node)
    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
