import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('shrimp_modu')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    default_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    default_ekf_params_file = os.path.join(pkg, 'config', 'ekf.yaml')
    ekf_params_file = LaunchConfiguration('ekf_params_file')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]
    action_endpoints = [
        'send_goal',
        'cancel_goal',
        'get_result',
        'feedback',
        'status',
    ]
    navigate_to_pose_remaps = [
        (
            f'/navigate_to_pose/_action/{endpoint}',
            f'/navigate_to_pose_raw/_action/{endpoint}',
        )
        for endpoint in action_endpoints
    ]
    navigate_through_poses_remaps = [
        (
            f'/navigate_through_poses/_action/{endpoint}',
            f'/navigate_through_poses_raw/_action/{endpoint}',
        )
        for endpoint in action_endpoints
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    odom_tf_node = Node(
        package='shrimp_modu',
        executable='odom_tf_node',
        name='odom_tf_node',
        output='screen',
        parameters=[{
            'input_odom_topic': '/odom_custom',
            'output_odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_tf': False,
            'override_covariance': True,
            'pose_covariance_diagonal': [
                0.0025, 0.0025, 1000000.0,
                1000000.0, 1000000.0, 0.0009,
            ],
            'twist_covariance_diagonal': [
                0.0004, 0.01, 1000000.0,
                1000000.0, 1000000.0, 0.0009,
            ],
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=remappings + [
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'map', 'odom'
        ],
        output='screen',
    )

    navigate_to_pose_approach = Node(
        package='shrimp_modu',
        executable='navigate_to_pose_approach',
        name='navigate_to_pose_approach',
        output='screen',
        parameters=[{
            'public_action_name': '/navigate_to_pose',
            'nav2_action_name': '/navigate_to_pose_raw',
            'robot_frame': 'base_footprint',
            'approach_frame': 'map',
        }],
    )

    nav_debug_logger = Node(
        package='shrimp_modu',
        executable='nav_debug_logger',
        name='nav_debug_logger',
        output='screen',
        parameters=[{
            'log_period': 1.0,
            'goal_timeout': 0.0,
            'wheelbase': 0.175,
            'rear_steering_ratio': -0.25,
            'linear_sign': -1.0,
            'steering_sign': 1.0,
            'omega_deadband': 0.001,
            'min_steering_speed': 0.002,
            'min_steering_angle': 0.035,
            'odom_topic': '/odometry/filtered',
        }],
    )

    nav_metrics_logger = Node(
        package='shrimp_modu',
        executable='nav_metrics_logger',
        name='nav_metrics_logger',
        output='screen',
        parameters=[{
            'odom_topic': '/odometry/filtered',
            'goal_topic': '/navigate_to_pose_approach/current_goal',
            'cmd_nav_topic': '/cmd_vel_nav',
            'cmd_exec_topic': '/cmd_vel',
            'wheel_commands_topic': '/wheel_controller/commands',
            'steering_commands_topic': '/steering_controller/commands',
            'publish_period': 0.1,
            'max_path_length': 5000,
            'csv_enabled': True,
        }],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [
            ('navigate_to_pose', 'navigate_to_pose_raw'),
            ('navigate_through_poses', 'navigate_through_poses_raw'),
        ] + navigate_to_pose_remaps + navigate_through_poses_remaps,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
        ],
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace superior para Nav2'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar el reloj de Gazebo'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Activar automaticamente los lifecycle nodes de Nav2'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Archivo de parametros para Nav2'
        ),
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=default_ekf_params_file,
            description='Archivo de parametros para robot_localization EKF'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nivel de logging de Nav2'
        ),
        map_to_odom_tf,
        odom_tf_node,
        ekf_node,
        navigate_to_pose_approach,
        nav_debug_logger,
        nav_metrics_logger,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        TimerAction(
            period=3.0,
            actions=[lifecycle_manager],
        ),
    ])
