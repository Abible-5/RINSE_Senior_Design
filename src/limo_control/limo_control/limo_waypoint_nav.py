#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math, time


class WaypointNav(Node):
    def __init__(self):
        super().__init__("waypoint_nav")

        # --- publishers / subscribers ---
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # --- waypoints ---
        self.waypoints = [(1.0, 0.0), (2.0, 0.0), (3.0, 1.0)]
        self.current_index = 0
        self.pose = None

        # --- tuning ---
        self.max_lin = 0.25
        self.max_ang = 0.8
        self.goal_tolerance = 0.10
        self.pause_duration = 2.0
        self.maneuver_time = 1.2

        # --- control parameters ---
        self.heading_smooth = 0.85
        self.angular_rate_limit = 0.12

        # --- yaw correction factor (tune this) ---
        # If robot thinks it turns MORE than it actually does, use <1.0 (e.g., 0.78)
        # If robot thinks it turns LESS than it actually does, use >1.0
        self.yaw_correction = 0.65

        # --- controller state ---
        self.stage = "FORWARD"
        self.stage_start = None
        self.turn_dir = 0.0
        self.filtered_err = 0.0
        self.last_ang_cmd = 0.0
        self.prev_err = 0.0

        # run at 20 Hz
        self.timer = self.create_timer(0.05, self.loop)

    # ================================================================

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def heading_error(self, tx, ty):
        px, py = self.pose.position.x, self.pose.position.y
        yaw = self.yaw_from_quat(self.pose.orientation) * self.yaw_correction
        dx, dy = tx - px, ty - py
        desired = math.atan2(dy, dx)
        return math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))

    # ================================================================

    def loop(self):
        if self.pose is None or self.current_index >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            return

        target_x, target_y = self.waypoints[self.current_index]
        distance = math.hypot(target_x - self.pose.position.x,
                              target_y - self.pose.position.y)

        # --- waypoint reached ---
        if distance < self.goal_tolerance:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"Reached waypoint {self.current_index + 1}/{len(self.waypoints)}")
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.cmd_pub.publish(Twist())
                return
            self.stage = "PAUSE"
            self.stage_start = time.time()
            return

        # --- pause between waypoints ---
        if self.stage == "PAUSE":
            self.cmd_pub.publish(Twist())
            if time.time() - self.stage_start >= self.pause_duration:
                next_x, next_y = self.waypoints[self.current_index]
                err = abs(self.heading_error(next_x, next_y))
                if err > math.radians(15):
                    self.turn_dir = math.copysign(1.0, self.heading_error(next_x, next_y))
                    self.stage = "REVERSE"
                    self.stage_start = time.time()
                else:
                    self.stage = "FORWARD"
                    self.stage_start = None
            return

        # --- diagnostics ---
        if self.stage in ["REVERSE", "FORWARD_TURN", "FORWARD"]:
            next_x, next_y = self.waypoints[self.current_index]
            px, py = self.pose.position.x, self.pose.position.y
            yaw = self.yaw_from_quat(self.pose.orientation) * self.yaw_correction
            dx, dy = next_x - px, next_y - py
            desired = math.atan2(dy, dx)
            err = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
            self.get_logger().info(
                f"[{self.stage}] pos=({px:.2f},{py:.2f}) yaw={math.degrees(yaw):.1f}° desired={math.degrees(desired):.1f}° err={math.degrees(err):.1f}°"
            )

            odom_yaw = self.yaw_from_quat(self.pose.orientation)
            self.get_logger().info(f"raw_odom_yaw={math.degrees(odom_yaw):.1f}°  corrected_yaw={math.degrees(odom_yaw * self.yaw_correction):.1f}°")


        cmd = Twist()

        # --- reverse arc ---
        if self.stage == "REVERSE":
            cmd.linear.x = -0.12
            cmd.angular.z = self.max_ang * 0.40 * self.turn_dir
            self.cmd_pub.publish(cmd)

            if time.time() - self.stage_start >= self.maneuver_time:
                self.stage = "FORWARD_TURN"
                self.stage_start = time.time()
            return

        # --- forward counter-arc ---
        if self.stage == "FORWARD_TURN":
            err_now = self.heading_error(*self.waypoints[self.current_index])
            self.turn_dir = math.copysign(1.0, err_now)

            cmd.linear.x = 0.18
            cmd.angular.z = self.max_ang * 0.50 * self.turn_dir
            self.cmd_pub.publish(cmd)

            if time.time() - self.stage_start >= self.maneuver_time:
                next_x, next_y = self.waypoints[self.current_index]
                err = abs(self.heading_error(next_x, next_y))
                if err > math.radians(20):
                    self.turn_dir = math.copysign(1.0, self.heading_error(next_x, next_y))
                    self.stage = "REVERSE"
                    self.stage_start = time.time()
                else:
                    self.stage = "FORWARD"
                    self.stage_start = None
            return

        # --- forward steering ---
        if self.stage == "FORWARD":
            next_x, next_y = self.waypoints[self.current_index]
            raw_err = self.heading_error(next_x, next_y)
            self.filtered_err = 0.6 * self.filtered_err + 0.4 * raw_err
            err = self.filtered_err

            # PD steering
            Kp = 4.0
            Kd = 0.15
            dt = 0.05
            derivative = (err - self.prev_err) / dt
            self.prev_err = err

            ang_cmd = Kp * err + Kd * derivative
            ang_cmd = max(min(ang_cmd, self.max_ang), -self.max_ang)

            turn_scale = max(0.3, 1 - 0.8 * abs(err))
            cmd.linear.x = self.max_lin * turn_scale
            cmd.angular.z = ang_cmd
            self.cmd_pub.publish(cmd)


# ================================================================

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
