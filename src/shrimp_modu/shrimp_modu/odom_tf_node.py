import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def covariance_from_diagonal(diagonal):
    covariance = [0.0] * 36
    for index, value in enumerate(diagonal[:6]):
        covariance[index * 6 + index] = float(value)
    return covariance


class OdomTF(Node):
    def __init__(self):
        super().__init__('odom_tf_node')

        self.declare_parameter('input_odom_topic', '/odom_custom')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('override_covariance', False)
        self.declare_parameter(
            'pose_covariance_diagonal',
            [0.0025, 0.0025, 1000000.0, 1000000.0, 1000000.0, 0.0009],
        )
        self.declare_parameter(
            'twist_covariance_diagonal',
            [0.0004, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.0009],
        )

        self.input_odom_topic = self.get_parameter('input_odom_topic').value
        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.override_covariance = self.get_parameter('override_covariance').value
        self.pose_covariance = covariance_from_diagonal(
            self.get_parameter('pose_covariance_diagonal').value
        )
        self.twist_covariance = covariance_from_diagonal(
            self.get_parameter('twist_covariance_diagonal').value
        )

        self.br = TransformBroadcaster(self) if self.publish_tf else None
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.latest_odom = None
        self.create_subscription(Odometry, self.input_odom_topic, self.cb, 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_latest)

        tf_status = "publicando TF" if self.publish_tf else "sin publicar TF"
        covariance_status = (
            "con covarianzas configuradas"
            if self.override_covariance else
            "preservando covarianzas originales"
        )
        self.get_logger().info(
            f"Odom bridge {self.input_odom_topic} -> {self.output_odom_topic}, "
            f"{tf_status} {self.odom_frame} -> {self.base_frame}, "
            f"{covariance_status}"
        )

    def publish_latest(self):
        if self.latest_odom is None:
            return

        self.publish_odom_and_tf(self.latest_odom)

    def cb(self, msg):
        self.latest_odom = msg
        self.publish_odom_and_tf(msg)

    def publish_odom_and_tf(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        if self.br is not None:
            self.br.sendTransform(t)

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose = msg.pose
        odom.twist = msg.twist
        if self.override_covariance:
            odom.pose.covariance = self.pose_covariance
            odom.twist.covariance = self.twist_covariance
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdomTF()
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
