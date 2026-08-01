from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('ros2_motion_planning')

    # Arguments
    map_name = LaunchConfiguration('map')
    map_path = LaunchConfiguration('map_path')
    algorithm = LaunchConfiguration('algorithm')
    animate = LaunchConfiguration('animate')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_theta = LaunchConfiguration('start_theta')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_theta = LaunchConfiguration('goal_theta')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_map_name = DeclareLaunchArgument(
        'map',
        default_value='terrain_open.pgm',
        description='Name of the PGM map file inside the package maps/ directory'
    )
    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value=PathJoinSubstitution([pkg_share, 'maps', map_name]),
        description='Full path to the PGM terrain map file'
    )
    declare_algorithm = DeclareLaunchArgument(
        'algorithm',
        default_value='astar',
        description="Planning algorithm to use ('astar' or 'rrt')"
    )
    declare_animate = DeclareLaunchArgument(
        'animate',
        default_value='false',
        description='Whether to enable live search exploration visualization in RViz'
    )
    declare_start_x = DeclareLaunchArgument(
        'start_x', default_value='-20.0', description='Start position X (m)'
    )
    declare_start_y = DeclareLaunchArgument(
        'start_y', default_value='0.0', description='Start position Y (m)'
    )
    declare_start_theta = DeclareLaunchArgument(
        'start_theta', default_value='0.0', description='Start heading angle (rad)'
    )
    declare_goal_x = DeclareLaunchArgument(
        'goal_x', default_value='20.0', description='Goal position X (m)'
    )
    declare_goal_y = DeclareLaunchArgument(
        'goal_y', default_value='0.0', description='Goal position Y (m)'
    )
    declare_goal_theta = DeclareLaunchArgument(
        'goal_theta', default_value='0.0', description='Goal heading angle (rad)'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to start RViz2 for visualization'
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz', 'motion_planning.rviz']),
        description='Full path to the RViz configuration file'
    )

    # Server Node
    server_node = Node(
        package='ros2_motion_planning',
        executable='motion_planning_server',
        name='motion_planning_server',
        output='screen'
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    # Client Node (delayed by 2.0 seconds so server and RViz are ready before planning starts)
    client_node = Node(
        package='ros2_motion_planning',
        executable='motion_planning_client',
        name='motion_planning_client',
        output='screen',
        arguments=[
            '--map', map_path,
            '--algorithm', algorithm,
            '--animate', animate,
            '--start-x', start_x,
            '--start-y', start_y,
            '--start-theta', start_theta,
            '--goal-x', goal_x,
            '--goal-y', goal_y,
            '--goal-theta', goal_theta,
        ]
    )
    delayed_client_node = TimerAction(
        period=2.0,
        actions=[client_node]
    )

    return LaunchDescription([
        declare_map_name,
        declare_map_path,
        declare_algorithm,
        declare_animate,
        declare_start_x,
        declare_start_y,
        declare_start_theta,
        declare_goal_x,
        declare_goal_y,
        declare_goal_theta,
        declare_use_rviz,
        declare_rviz_config,
        server_node,
        rviz_node,
        delayed_client_node,
    ])
