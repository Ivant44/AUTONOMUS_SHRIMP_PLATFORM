#!/usr/bin/env python3

import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray


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


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class NavMetricsLogger(Node):
    def __init__(self):
        super().__init__('nav_metrics_logger')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter(
            'goal_topic',
            '/navigate_to_pose_approach/current_goal',
        )
        self.declare_parameter('cmd_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_exec_topic', '/cmd_vel')
        self.declare_parameter(
            'wheel_commands_topic',
            '/wheel_controller/commands',
        )
        self.declare_parameter(
            'steering_commands_topic',
            '/steering_controller/commands',
        )
        self.declare_parameter('publish_period', 0.1)
        self.declare_parameter('max_path_length', 5000)
        self.declare_parameter('csv_enabled', True)
        self.declare_parameter('csv_directory', '')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_nav_topic = self.get_parameter('cmd_nav_topic').value
        self.cmd_exec_topic = self.get_parameter('cmd_exec_topic').value
        self.wheel_commands_topic = (
            self.get_parameter('wheel_commands_topic').value
        )
        self.steering_commands_topic = (
            self.get_parameter('steering_commands_topic').value
        )
        self.publish_period = self.get_parameter('publish_period').value
        self.max_path_length = self.get_parameter('max_path_length').value
        self.csv_enabled = self.get_parameter('csv_enabled').value
        self.csv_directory = self.get_parameter('csv_directory').value

        self.odom = None
        self.goal = None
        self.cmd_nav = None
        self.cmd_exec = None
        self.wheel_commands = []
        self.steering_commands = []
        self.path = Path()
        self.csv_file = None
        self.csv_writer = None

        self.path_pub = self.create_publisher(
            Path,
            '/shrimp_metrics/trajectory',
            10,
        )
        self.distance_error_pub = self.create_publisher(
            Float32,
            '/shrimp_metrics/distance_error',
            10,
        )
        self.heading_error_pub = self.create_publisher(
            Float32,
            '/shrimp_metrics/heading_error',
            10,
        )
        self.goal_yaw_error_pub = self.create_publisher(
            Float32,
            '/shrimp_metrics/goal_yaw_error',
            10,
        )
        self.motor_effort_pub = self.create_publisher(
            Float32,
            '/shrimp_metrics/motor_effort_proxy',
            10,
        )
        self.motor_rms_pub = self.create_publisher(
            Float32,
            '/shrimp_metrics/motor_effort_rms',
            10,
        )
        self.steering_effort_pub = self.create_publisher(
            Float32,
            '/shrimp_metrics/steering_effort_proxy',
            10,
        )
        self.cmd_speed_pub = self.create_publisher(
            Float32,
            '/shrimp_metrics/executed_speed',
            10,
        )
        self.cmd_yaw_rate_pub = self.create_publisher(
            Float32,
            '/shrimp_metrics/executed_yaw_rate',
            10,
        )

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        self.create_subscription(Twist, self.cmd_nav_topic, self.cmd_nav_cb, 20)
        self.create_subscription(Twist, self.cmd_exec_topic, self.cmd_exec_cb, 20)
        self.create_subscription(
            Float64MultiArray,
            self.wheel_commands_topic,
            self.wheel_cb,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            self.steering_commands_topic,
            self.steering_cb,
            20,
        )

        if self.csv_enabled:
            self.open_csv()

        self.create_timer(self.publish_period, self.publish_metrics)
        self.get_logger().info(
            "Nav metrics logger listo: "
            f"odom={self.odom_topic}, goal={self.goal_topic}, "
            f"csv={self.csv_path if self.csv_enabled else 'disabled'}"
        )

    def open_csv(self):
        directory = self.csv_directory
        if not directory:
            directory = os.path.join(
                os.path.expanduser('~'),
                '.ros',
                'shrimp_metrics',
            )
        os.makedirs(directory, exist_ok=True)

        filename = time.strftime('nav_metrics_%Y%m%d_%H%M%S.csv')
        self.csv_path = os.path.join(directory, filename)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'stamp',
            'x',
            'y',
            'yaw',
            'goal_x',
            'goal_y',
            'goal_yaw',
            'distance_error',
            'heading_error',
            'goal_yaw_error',
            'odom_vx',
            'odom_wz',
            'cmd_nav_vx',
            'cmd_nav_wz',
            'cmd_exec_vx',
            'cmd_exec_wz',
            'motor_effort_proxy',
            'motor_effort_rms',
            'steering_effort_proxy',
            'front_steering_cmd',
            'rear_steering_cmd',
        ])

    def odom_cb(self, msg):
        self.odom = msg

    def goal_cb(self, msg):
        self.goal = msg
        self.path = Path()

    def cmd_nav_cb(self, msg):
        self.cmd_nav = msg

    def cmd_exec_cb(self, msg):
        self.cmd_exec = msg

    def wheel_cb(self, msg):
        self.wheel_commands = list(msg.data)

    def steering_cb(self, msg):
        self.steering_commands = list(msg.data)

    def publish_float(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def command_mean_abs(self, values):
        if not values:
            return 0.0
        return sum(abs(value) for value in values) / len(values)

    def command_rms(self, values):
        if not values:
            return 0.0
        return math.sqrt(sum(value * value for value in values) / len(values))

    def append_path_pose(self):
        pose = PoseStamped()
        pose.header = self.odom.header
        pose.pose = self.odom.pose.pose

        self.path.header = self.odom.header
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length:]
        self.path_pub.publish(self.path)

    def compute_errors(self):
        if self.odom is None or self.goal is None:
            return 0.0, 0.0, 0.0, 0.0

        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        goal = self.goal.pose
        goal_yaw = yaw_from_quaternion(goal.orientation)

        dx = goal.position.x - pose.position.x
        dy = goal.position.y - pose.position.y
        distance_error = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        heading_error = normalize_angle(bearing - yaw)
        goal_yaw_error = normalize_angle(goal_yaw - yaw)
        return distance_error, heading_error, goal_yaw_error, goal_yaw

    def publish_metrics(self):
        if self.odom is None:
            return

        self.append_path_pose()

        distance_error, heading_error, goal_yaw_error, goal_yaw = (
            self.compute_errors()
        )
        motor_effort = self.command_mean_abs(self.wheel_commands)
        motor_rms = self.command_rms(self.wheel_commands)
        steering_effort = self.command_mean_abs(self.steering_commands)

        self.publish_float(self.distance_error_pub, distance_error)
        self.publish_float(self.heading_error_pub, heading_error)
        self.publish_float(self.goal_yaw_error_pub, goal_yaw_error)
        self.publish_float(self.motor_effort_pub, motor_effort)
        self.publish_float(self.motor_rms_pub, motor_rms)
        self.publish_float(self.steering_effort_pub, steering_effort)
        self.publish_float(
            self.cmd_speed_pub,
            self.odom.twist.twist.linear.x,
        )
        self.publish_float(
            self.cmd_yaw_rate_pub,
            self.odom.twist.twist.angular.z,
        )

        if self.csv_writer is not None:
            self.write_csv_row(
                distance_error,
                heading_error,
                goal_yaw_error,
                goal_yaw,
                motor_effort,
                motor_rms,
                steering_effort,
            )

    def write_csv_row(
        self,
        distance_error,
        heading_error,
        goal_yaw_error,
        goal_yaw,
        motor_effort,
        motor_rms,
        steering_effort,
    ):
        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        goal_x = ''
        goal_y = ''
        if self.goal is not None:
            goal_x = self.goal.pose.position.x
            goal_y = self.goal.pose.position.y

        cmd_nav_vx = self.cmd_nav.linear.x if self.cmd_nav is not None else ''
        cmd_nav_wz = self.cmd_nav.angular.z if self.cmd_nav is not None else ''
        cmd_exec_vx = (
            self.cmd_exec.linear.x if self.cmd_exec is not None else ''
        )
        cmd_exec_wz = (
            self.cmd_exec.angular.z if self.cmd_exec is not None else ''
        )
        front_steering = (
            self.steering_commands[0] if len(self.steering_commands) > 0 else ''
        )
        rear_steering = (
            self.steering_commands[1] if len(self.steering_commands) > 1 else ''
        )

        self.csv_writer.writerow([
            stamp_to_sec(self.odom.header.stamp),
            pose.position.x,
            pose.position.y,
            yaw,
            goal_x,
            goal_y,
            goal_yaw if self.goal is not None else '',
            distance_error,
            heading_error,
            goal_yaw_error,
            self.odom.twist.twist.linear.x,
            self.odom.twist.twist.angular.z,
            cmd_nav_vx,
            cmd_nav_wz,
            cmd_exec_vx,
            cmd_exec_wz,
            motor_effort,
            motor_rms,
            steering_effort,
            front_steering,
            rear_steering,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NavMetricsLogger()
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
