#!/usr/bin/env python3
#
# Nodo legacy de odometria experimental.
# El flujo actual usa /odom_custom publicado por gazebo_ros_p3d desde el URDF.
# Este archivo se conserva solo como referencia/debug y no forma parte del flujo normal.

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
    
class ShrimpOdometry(Node):

    def __init__(self):
        super().__init__('shrimp_odometry')

        self.declare_parameter('model_name', 'shrimp')
        self.declare_parameter('model_states_topic', '')
        self.model_name = self.get_parameter('model_name').value
        self.model_states_topic = self.get_parameter('model_states_topic').value

        self.debug_count = 0
        self.model_index = None
        self.model_warning_count = 0
        self.model_states_count = 0
        self.last_topic_seen = None

        model_states_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # =========================
        # SUBSCRIBERS
        # =========================
        model_states_topics = (
            [self.model_states_topic]
            if self.model_states_topic
            else ['/model_states', '/gazebo/model_states']
        )
        for topic in model_states_topics:
            self.create_subscription(
                ModelStates,
                topic,
                lambda msg, topic_name=topic: self.model_states_callback(msg, topic_name),
                model_states_qos
            )

        # =========================
        # PUBLISHER
        # =========================
        self.odom_pub = self.create_publisher(Odometry, '/odom_custom', 10)
        self.create_timer(1.0, self.watch_model_states)

        self.get_logger().info(
            "Odometry SHRIMP usando ground truth de Gazebo "
            f"para el modelo: {self.model_name} en {', '.join(model_states_topics)}"
        )

    def model_states_callback(self, msg, topic_name):
        self.model_states_count += 1
        if self.last_topic_seen != topic_name:
            self.last_topic_seen = topic_name
            self.get_logger().info(f"Recibiendo ModelStates desde {topic_name}")
        if self.model_index is None:
            self.model_index = self.find_model_index(msg.name)
            if self.model_index is None:
                self.model_warning_count += 1
                if self.model_warning_count % 20 == 1:
                    available = ", ".join(msg.name[:10])
                    self.get_logger().warning(
                        f"No encuentro el modelo `{self.model_name}` en {topic_name}. "
                        f"Modelos visibles: {available}"
                    )
                return

        if self.model_index >= len(msg.pose) or self.model_index >= len(msg.twist):
            return

        pose = msg.pose[self.model_index]
        twist = msg.twist[self.model_index]

        q = pose.orientation
        theta = np.arctan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        self.debug_count += 1
        if self.debug_count % 10 == 0:
            self.get_logger().info(
                "odom_debug "
                f"v={twist.linear.x:.3f} theta_dot={twist.angular.z:.3f} "
                f"pose=({pose.position.x:.3f},{pose.position.y:.3f},{theta:.3f})"
            )

        # =========================
        # PUBLICAR ODOM
        # =========================
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose = pose
        odom.twist.twist = twist

        self.odom_pub.publish(odom)

    def watch_model_states(self):
        if self.model_states_count == 0:
            self.get_logger().warning(
                "No estoy recibiendo mensajes de ModelStates. "
                "Revisa `ros2 topic list | grep model_states`."
            )

    def find_model_index(self, model_names):
        if self.model_name in model_names:
            return model_names.index(self.model_name)

        target = self.model_name.lower()
        for i, name in enumerate(model_names):
            if target in name.lower():
                self.get_logger().info(
                    f"Usando modelo `{name}` como coincidencia para `{self.model_name}`"
                )
                return i

        return None


def main(args=None):
    rclpy.init(args=args)
    node = ShrimpOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
