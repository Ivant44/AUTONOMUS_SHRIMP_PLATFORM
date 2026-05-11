#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import SetParametersResult
import math
import time
import numpy as np


class ShrimpController(Node):
    def __init__(self):
        super().__init__('shrimp_controller')

        # =========================
        # PARAMETROS
        # =========================
        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('track_width', 0.37)
        self.declare_parameter('max_steering_angle', 0.0)
        self.declare_parameter('linear_sign', -1.0)
        self.declare_parameter('steering_sign', 1.0)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('wheelbase', 0.175)
        self.declare_parameter('rear_steering_ratio', -0.4)
        self.declare_parameter('max_steering_rate', 1.5)
        self.declare_parameter('omega_deadband', 0.03)
        self.declare_parameter('min_steering_speed', 0.01)
        self.declare_parameter('min_steering_angle', 0.0)
        self.declare_parameter('debug_logging', False)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.linear_sign = self.get_parameter('linear_sign').value
        self.steering_sign = self.get_parameter('steering_sign').value
        self.timeout = self.get_parameter('timeout').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.rear_steering_ratio = self.get_parameter('rear_steering_ratio').value
        self.max_steering_rate = self.get_parameter('max_steering_rate').value
        self.omega_deadband = self.get_parameter('omega_deadband').value
        self.min_steering_speed = self.get_parameter('min_steering_speed').value
        self.min_steering_angle = self.get_parameter('min_steering_angle').value
        self.debug_logging = self.get_parameter('debug_logging').value
        self.add_on_set_parameters_callback(self.on_parameter_update)

        # =========================
        # VARIABLES
        # =========================
        self.last_cmd_time = time.time()
        self.v = 0.0
        self.omega = 0.0
        self.delta_f = 0.0
        self.last_update_time = time.time()
        self.target_delta_f = 0.0
        self.debug_count = 0

        # =========================
        # SUBSCRIBERS
        # =========================
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # =========================
        # PUBLISHERS
        # =========================
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_controller/commands',
            10
        )
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )

        # =========================
        # TIMER (control loop)
        # =========================
        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

        self.get_logger().info("Shrimp controller listo 🚀")

    def on_parameter_update(self, params):
        for param in params:
            if param.name == 'linear_sign':
                self.linear_sign = float(param.value)
            elif param.name == 'steering_sign':
                self.steering_sign = float(param.value)
            elif param.name == 'max_steering_angle':
                self.max_steering_angle = float(param.value)
            elif param.name == 'rear_steering_ratio':
                self.rear_steering_ratio = float(param.value)
            elif param.name == 'max_steering_rate':
                self.max_steering_rate = float(param.value)
            elif param.name == 'omega_deadband':
                self.omega_deadband = float(param.value)
            elif param.name == 'min_steering_speed':
                self.min_steering_speed = float(param.value)
            elif param.name == 'min_steering_angle':
                self.min_steering_angle = float(param.value)
            elif param.name == 'debug_logging':
                self.debug_logging = bool(param.value)
        return SetParametersResult(successful=True)

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z
        self.last_cmd_time = time.time()

    def clamp_steering(self, angle):
        if self.max_steering_angle <= 0.0:
            return float(angle)
        return float(
            np.clip(angle, -self.max_steering_angle, self.max_steering_angle)
        )

    def update(self):
        # =========================
        # FAIL-SAFE (timeout)
        # =========================
        now = time.time()

        if now - self.last_cmd_time > self.timeout:
            v = 0.0
            omega = 0.0
        else:
            v = self.linear_sign * self.v
            omega = self.omega

        dt = max(now - self.last_update_time, 1.0e-3)
        self.last_update_time = now

        if abs(omega) < self.omega_deadband:
            target_delta_f = 0.0
        elif abs(v) < self.min_steering_speed:
            target_delta_f = 0.0
        else:
            steering_gain = max(0.1, 1.0 - self.rear_steering_ratio)
            target_delta_f = math.atan((self.wheelbase * omega) / (abs(v) * steering_gain))
            if (
                self.min_steering_angle > 0.0 and
                abs(target_delta_f) < self.min_steering_angle
            ):
                target_delta_f = math.copysign(self.min_steering_angle, target_delta_f)

        target_delta_f = self.clamp_steering(target_delta_f)
        self.target_delta_f = target_delta_f
        if abs(v) < self.min_steering_speed and abs(omega) < self.omega_deadband:
            self.delta_f = 0.0
        else:
            max_delta_step = self.max_steering_rate * dt
            self.delta_f += float(
                np.clip(target_delta_f - self.delta_f, -max_delta_step, max_delta_step)
            )
        delta_f = self.clamp_steering(self.delta_f)

        delta_r = self.rear_steering_ratio * delta_f
        delta_r = self.clamp_steering(delta_r)

        # La traccion se mantiene simetrica; la direccion se calcula desde /cmd_vel.
        v_left = v
        v_right = v

        wl = v_left / self.wheel_radius
        wr = v_right / self.wheel_radius

        # Orden del ForwardCommandController:
        # [atras_derecha, atras_izquierda, delantera_derecha, delantera_izquierda].
        # Los joints derechos e izquierdos tienen ejes opuestos en el URDF.
        wheels = [-wr, wl, -wr, wl]

        steering = [
            self.steering_sign * delta_f,
            self.steering_sign * delta_r,
        ]

        # =========================
        # PUBLICAR
        # =========================
        wheel_msg = Float64MultiArray()
        wheel_msg.data = wheels

        steering_msg = Float64MultiArray()
        steering_msg.data = steering

        self.wheel_pub.publish(wheel_msg)
        self.steering_pub.publish(steering_msg)

        self.debug_count += 1
        if self.debug_logging and self.debug_count % 10 == 0:
            self.get_logger().info(
                "ctrl_debug "
                f"cmd_v={self.v:.3f} cmd_omega={self.omega:.3f} "
                f"v_exec={v:.3f} omega_exec={omega:.3f} "
                f"target_delta_f={self.target_delta_f:.3f} "
                f"delta_f={delta_f:.3f} delta_r={delta_r:.3f} "
                f"steering_sign={self.steering_sign:.1f} "
                f"steering_cmd=[{steering[0]:.3f}, {steering[1]:.3f}] "
                f"wl={wl:.3f} wr={wr:.3f} "
                f"wheels={wheels}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ShrimpController()
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
