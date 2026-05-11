#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def fmt_twist(msg):
    if msg is None:
        return "none"
    return f"vx={msg.linear.x:.3f} wz={msg.angular.z:.3f}"


class NavDebugLogger(Node):
    def __init__(self):
        super().__init__('nav_debug_logger')

        self.declare_parameter('log_period', 1.0)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('wheelbase', 0.175)
        self.declare_parameter('rear_steering_ratio', -0.25)
        self.declare_parameter('linear_sign', -1.0)
        self.declare_parameter('steering_sign', 1.0)
        self.declare_parameter('omega_deadband', 1.0e-4)
        self.declare_parameter('min_steering_speed', 0.01)
        self.declare_parameter('min_steering_angle', 0.0)
        self.declare_parameter('odom_topic', '/odom')

        self.log_period = self.get_parameter('log_period').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.rear_steering_ratio = self.get_parameter('rear_steering_ratio').value
        self.linear_sign = self.get_parameter('linear_sign').value
        self.steering_sign = self.get_parameter('steering_sign').value
        self.omega_deadband = self.get_parameter('omega_deadband').value
        self.min_steering_speed = self.get_parameter('min_steering_speed').value
        self.min_steering_angle = self.get_parameter('min_steering_angle').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.goal = None
        self.goal_time = 0.0
        self.odom = None
        self.cmd_nav = None
        self.cmd_smoothed = None
        self.steering = None
        self.wheels = None

        self.create_subscription(
            PoseStamped,
            '/navigate_to_pose_approach/current_goal',
            self.goal_cb,
            10,
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_nav_cb, 20)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_smoothed_cb, 20)
        self.create_subscription(
            Float64MultiArray,
            '/steering_controller/commands',
            self.steering_cb,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            '/wheel_controller/commands',
            self.wheels_cb,
            20,
        )
        self.create_timer(self.log_period, self.log_state)
        self.get_logger().info(
            f"Nav debug logger listo, odom_topic={self.odom_topic}"
        )

    def goal_cb(self, msg):
        self.goal = msg
        self.goal_time = time.time()
        goal_yaw = yaw_from_quaternion(msg.pose.orientation)
        self.get_logger().info(
            "nav_goal "
            f"frame={msg.header.frame_id} "
            f"pos=({msg.pose.position.x:.3f},{msg.pose.position.y:.3f}) "
            f"yaw={math.degrees(goal_yaw):.1f}deg"
        )

    def odom_cb(self, msg):
        self.odom = msg

    def cmd_nav_cb(self, msg):
        self.cmd_nav = msg

    def cmd_smoothed_cb(self, msg):
        self.cmd_smoothed = msg

    def steering_cb(self, msg):
        self.steering = list(msg.data)

    def wheels_cb(self, msg):
        self.wheels = list(msg.data)

    def expected_front_steering(self):
        if self.cmd_smoothed is None:
            return None
        v_exec = self.linear_sign * self.cmd_smoothed.linear.x
        omega = self.cmd_smoothed.angular.z
        if abs(v_exec) < self.min_steering_speed or abs(omega) < self.omega_deadband:
            return 0.0
        steering_gain = max(0.1, 1.0 - self.rear_steering_ratio)
        delta = math.atan(
            (self.wheelbase * omega) / (abs(v_exec) * steering_gain)
        )
        if self.min_steering_angle > 0.0 and abs(delta) < self.min_steering_angle:
            delta = math.copysign(self.min_steering_angle, delta)
        return self.steering_sign * delta

    def log_state(self):
        if self.odom is None:
            self.get_logger().info("nav_diag esperando /odom")
            return

        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        odom_twist = self.odom.twist.twist

        goal_age = time.time() - self.goal_time
        goal_expired = self.goal_timeout > 0.0 and goal_age > self.goal_timeout
        if self.goal is None or goal_expired:
            self.get_logger().info(
                "nav_diag idle "
                f"pose=({pose.position.x:.3f},{pose.position.y:.3f}) "
                f"yaw={math.degrees(yaw):.1f}deg "
                f"cmd_nav=({fmt_twist(self.cmd_nav)}) "
                f"cmd=({fmt_twist(self.cmd_smoothed)})"
            )
            return

        dx = self.goal.pose.position.x - pose.position.x
        dy = self.goal.pose.position.y - pose.position.y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        heading_error = normalize_angle(bearing - yaw)
        goal_yaw = yaw_from_quaternion(self.goal.pose.orientation)
        goal_yaw_error = normalize_angle(goal_yaw - yaw)

        steering_txt = "none"
        if self.steering is not None and len(self.steering) >= 2:
            steering_txt = (
                f"front={math.degrees(self.steering[0]):.1f}deg "
                f"rear={math.degrees(self.steering[1]):.1f}deg"
            )

        wheels_txt = "none"
        if self.wheels is not None:
            wheels_txt = "[" + ",".join(f"{v:.2f}" for v in self.wheels) + "]"

        expected_delta = self.expected_front_steering()
        expected_txt = "none"
        if expected_delta is not None:
            expected_txt = f"{math.degrees(expected_delta):.1f}deg"

        self.get_logger().info(
            "nav_diag "
            f"pose=({pose.position.x:.3f},{pose.position.y:.3f}) "
            f"yaw={math.degrees(yaw):.1f}deg "
            f"goal=({self.goal.pose.position.x:.3f},{self.goal.pose.position.y:.3f}) "
            f"dist={dist:.3f} "
            f"bearing={math.degrees(bearing):.1f}deg "
            f"heading_err={math.degrees(heading_error):.1f}deg "
            f"goal_yaw_err={math.degrees(goal_yaw_error):.1f}deg "
            f"odom_twist=(vx={odom_twist.linear.x:.3f} wz={odom_twist.angular.z:.3f}) "
            f"cmd_nav=({fmt_twist(self.cmd_nav)}) "
            f"cmd=({fmt_twist(self.cmd_smoothed)}) "
            f"expected_front={expected_txt} "
            f"steering=({steering_txt}) "
            f"wheels={wheels_txt}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NavDebugLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
