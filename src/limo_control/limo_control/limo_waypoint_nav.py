#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Waypoints (X then Y test)
        self.waypoints = [(0, 1.8), (1.8, 0)]
        self.current_index = 0
        self.pose = None
        self.timer = self.create_timer(0.1, self.navigate)

        # Gains
        self.Kp_linear = 0.3
        self.Kp_angular = 1.2

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def navigate(self):
        if self.pose is None:
            return

        if self.current_index >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            self.get_logger().info("All waypoints reached. Stopping.")
            self.destroy_node()
            return

        # --- Current and target ---
        target_x, target_y = self.waypoints[self.current_index]
        px, py = self.pose.position.x, self.pose.position.y
        dx, dy = target_x - px, target_y - py
        distance = math.hypot(dx, dy)

        # --- Heading ---
        yaw = self.yaw_from_quat(self.pose.orientation)
        target_theta = math.atan2(dy, dx)
        angle_err = math.atan2(math.sin(target_theta - yaw),
                               math.cos(target_theta - yaw))

        # --- Smooth control ---
        cmd = Twist()

        # reduce forward speed as angular error grows
        forward_scale = max(0.1, math.cos(angle_err))  # slow if large angle error
        cmd.linear.x = self.Kp_linear * distance * forward_scale

        # angular correction stronger for bigger angles
        cmd.angular.z = self.Kp_angular * angle_err

        # --- Speed clamps ---
        cmd.linear.x = max(min(cmd.linear.x, 0.5), 0.0)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

        # --- Publish ---
        self.cmd_pub.publish(cmd)

        # --- Logs (once per second) ---
        if int(self.get_clock().now().nanoseconds / 1e8) % 10 == 0:
            self.get_logger().info(
                f"Target({target_x:.2f},{target_y:.2f}) | "
                f"Pos({px:.2f},{py:.2f}) | D={distance:.2f} | "
                f"Yaw={math.degrees(yaw):.1f}° | θerr={math.degrees(angle_err):.1f}° | "
                f"v={cmd.linear.x:.2f} | ω={cmd.angular.z:.2f}"
            )

        # --- Goal reached ---
        if distance < 0.15:
            self.get_logger().info(f" Reached waypoint {self.current_index + 1}")
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.2)
            self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

