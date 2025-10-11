#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class DriveToPoint(Node):
    def __init__(self):
        super().__init__('limo_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Target (x,y) location in meters
        self.target_x = 2.0
        self.target_y = 1.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.reached = False

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def control_loop(self):
        if self.reached:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        twist = Twist()

        if distance > 0.1:  # keep moving until within 10 cm
            # simple proportional control towards target
            angle_to_goal = math.atan2(dy, dx)
            twist.linear.x = 0.2
            twist.angular.z = 0.5 * angle_to_goal
        else:
            self.get_logger().info("Target reached!")
            self.reached = True

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DriveToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()