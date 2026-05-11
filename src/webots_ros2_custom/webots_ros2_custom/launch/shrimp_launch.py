from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='webots_ros2_driver',
            executable='webots-controller',
            output='screen',
            parameters=[
                {'robot_description': ''},
                {'use_sim_time': True},
                {'set_robot_state_publisher': True},
                {'protocol': 'tcp'},
                {'ip_address': '172.17.208.1'},   # tu valor detectado
                {'port': 1234},
            ],
        )
    ])

