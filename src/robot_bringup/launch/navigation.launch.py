import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup  = get_package_share_directory('robot_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_params  = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_bringup, 'maps', 'tb3_world.yaml'),
    )

    # ── Topic relay: /diff_drive_controller/odom → /odom ──────────
    # Needed because Nav2 expects /odom but diff_drive publishes elsewhere
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        output='screen',
        arguments=[
            '/diff_drive_controller/odom',
            '/odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Map Server ─────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file},
        ],
    )

    # ── AMCL ───────────────────────────────────────────────────────
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'amcl_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Nav2 Stack ─────────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    # ── Lifecycle Manager ──────────────────────────────────────────
    # ✅ Delayed 5s so Gazebo TF (map→odom→CHASSIS) is available
    # before lifecycle manager tries to activate costmaps.
    # Fixes: "Timed out waiting for transform from CHASSIS to map"
    lifecycle_manager = TimerAction(
        period=5.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ]},
            ],
        )],
    )

    # ── RViz2 ─────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_bringup, 'maps', 'tb3_world.yaml'),
            description='Full path to map yaml',
        ),
        odom_relay,         # start immediately — needed by AMCL
        map_server,         # start immediately — needed by AMCL
        amcl,               # start immediately — begins localizing as soon as map arrives
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,  # ✅ delayed 5s — waits for Gazebo TF to stabilize
        rviz2,
    ])