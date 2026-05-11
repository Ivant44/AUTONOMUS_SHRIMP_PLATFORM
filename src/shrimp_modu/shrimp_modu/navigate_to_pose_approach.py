#!/usr/bin/env python3

import math

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


class NavigateToPoseApproach(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_approach')

        self.declare_parameter('public_action_name', '/navigate_to_pose')
        self.declare_parameter('nav2_action_name', '/navigate_to_pose_raw')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('approach_frame', 'map')

        public_action_name = self.get_parameter('public_action_name').value
        nav2_action_name = self.get_parameter('nav2_action_name').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.approach_frame = self.get_parameter('approach_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/navigate_to_pose_approach/current_goal',
            1,
        )
        self.client = ActionClient(self, NavigateToPose, nav2_action_name)
        self.server = ActionServer(
            self,
            NavigateToPose,
            public_action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.active_nav2_goal = None
        self.get_logger().info(
            f"Approach wrapper listo: {public_action_name} -> {nav2_action_name}"
        )

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        if self.active_nav2_goal is not None:
            self.active_nav2_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def compute_approach_yaw(self, goal):
        frame_id = goal.pose.header.frame_id or self.approach_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"No pude calcular approach yaw con TF {frame_id}->{self.robot_frame}: {exc}. "
                "Uso la orientación original del goal."
            )
            return None

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        goal_x = goal.pose.pose.position.x
        goal_y = goal.pose.pose.position.y
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        if math.hypot(dx, dy) < 0.05:
            return None
        return math.atan2(dy, dx)

    def build_approach_goal(self, goal_request):
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = goal_request.pose
        nav2_goal.behavior_tree = goal_request.behavior_tree

        yaw = self.compute_approach_yaw(nav2_goal)
        if yaw is not None:
            nav2_goal.pose.pose.orientation = yaw_to_quaternion(yaw)
            self.get_logger().info(
                "Goal con approach yaw "
                f"{math.degrees(yaw):.1f} deg hacia "
                f"({nav2_goal.pose.pose.position.x:.2f}, {nav2_goal.pose.pose.position.y:.2f})"
            )
        self.goal_pub.publish(nav2_goal.pose)
        return nav2_goal

    async def execute_callback(self, goal_handle):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action interno no disponible: /navigate_to_pose_raw")
            goal_handle.abort()
            return NavigateToPose.Result()

        nav2_goal = self.build_approach_goal(goal_handle.request)
        send_future = self.client.send_goal_async(
            nav2_goal,
            feedback_callback=lambda feedback_msg: goal_handle.publish_feedback(
                feedback_msg.feedback
            ),
        )
        nav2_goal_handle = await send_future
        if not nav2_goal_handle.accepted:
            goal_handle.abort()
            return NavigateToPose.Result()

        self.active_nav2_goal = nav2_goal_handle
        result_future = nav2_goal_handle.get_result_async()
        wrapped_result = await result_future
        self.active_nav2_goal = None

        if wrapped_result.status == 4:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return wrapped_result.result


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NavigateToPoseApproach()
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
