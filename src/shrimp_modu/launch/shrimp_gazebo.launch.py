import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('shrimp_modu')

    urdf_file = os.path.join(pkg, 'urdf', 'shrimp.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace(
        'package://shrimp_modu/',
        'file://' + pkg + '/'
    )

    # =========================
    # GAZEBO
    # =========================
    gazebo_world = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'worlds',
        'empty.world'
    )

    gazebo_share = get_package_share_directory('gazebo_ros')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': gazebo_world,
            'init': 'true',
            'factory': 'true',
            'force_system': 'false',
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # =========================
    # SPAWN ROBOT
    # =========================
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'shrimp',
            '-topic', 'robot_description',
            '-z', '0.2'
        ],
        output='screen'
    )

    # =========================
    # CONTROLLERS
    # =========================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "120"
        ],
        output="screen"
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "120"
        ],
        output="screen"
    )

    steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "steering_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "120"
        ],
        output="screen"
    )

    # =========================
    # NODOS PRINCIPALES
    # =========================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    shrimp_controller = Node(
        package='shrimp_modu',
        executable='shrimp_controller',
        name='shrimp_controller',
        parameters=[{
            'linear_sign': -1.0,
            'steering_sign': 1.0,
            'max_steering_angle': 0.6981317008,
            'rear_steering_ratio': -0.25,
            'max_steering_rate': 2.2,
            'omega_deadband': 0.001,
            'min_steering_speed': 0.002,
            'min_steering_angle': 0.035,
            'debug_logging': False,
        }],
        output='screen'
    )

    def spawn_exit_actions(event, context):
        if event.returncode == 0:
            return [
                joint_state_broadcaster_spawner,
                shrimp_controller,
                TimerAction(period=0.5, actions=[gzclient]),
            ]

        return [
            LogInfo(
                msg=(
                    "No se cargan controllers porque spawn_entity fallo. "
                    "Cierra cualquier Gazebo viejo antes de relanzar."
                )
            ),
            Shutdown(reason='spawn_entity failed'),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run Gazebo without gzclient.'
        ),

        gzserver,

        robot_state_publisher,

        spawn_robot,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=spawn_exit_actions
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[wheel_controller_spawner, steering_controller_spawner]
            )
        ),

    ])
