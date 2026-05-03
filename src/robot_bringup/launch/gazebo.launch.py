import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_urdf(base_path: str, control_path: str, controllers_yaml: str) -> str:
    with open(base_path, 'r') as f:
        base = f.read()
    with open(control_path, 'r') as f:
        control = f.read()

    def resolve_package_uri(match):
        pkg_name = match.group(1)
        rel_path = match.group(2)
        try:
            pkg_share = get_package_share_directory(pkg_name)
        except Exception:
            return match.group(0)
        return 'file://' + os.path.join(pkg_share, rel_path)

    base = re.sub(
        r'package://([^/]+)/([^\s"\'<>]+)',
        resolve_package_uri,
        base,
    )
    control = control.replace('__CONTROLLERS_YAML__', controllers_yaml)
    inner = re.sub(r'<\?xml[^>]+\?>\s*', '', control)
    inner = re.sub(r'<!--.*?-->\s*', '', inner, flags=re.DOTALL)
    inner = re.sub(r'<robot[^>]+>\s*', '', inner)
    inner = inner.replace('</robot>', '').strip()
    return base.replace('</robot>', inner + '\n</robot>')


def _build_rsp_urdf(base_path: str, control_path: str, controllers_yaml: str) -> str:
    with open(base_path, 'r') as f:
        base = f.read()
    with open(control_path, 'r') as f:
        control = f.read()
    control = control.replace('__CONTROLLERS_YAML__', controllers_yaml)
    inner = re.sub(r'<\?xml[^>]+\?>\s*', '', control)
    inner = re.sub(r'<!--.*?-->\s*', '', inner, flags=re.DOTALL)
    inner = re.sub(r'<robot[^>]+>\s*', '', inner)
    inner = inner.replace('</robot>', '').strip()
    return base.replace('</robot>', inner + '\n</robot>')


def generate_launch_description():
    pkg_desc        = get_package_share_directory('robot_description')
    pkg_controllers = get_package_share_directory('robot_controllers')
    pkg_bringup     = get_package_share_directory('robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    controllers_yaml = os.path.join(
        pkg_controllers, 'config', 'ros2_controllers.yaml'
    )
    base_urdf    = os.path.join(pkg_desc, 'urdf', 'Wheeled_Base.urdf')
    control_urdf = os.path.join(pkg_desc, 'urdf', 'ros2_control.urdf')

    gz_urdf  = _build_urdf(base_urdf, control_urdf, controllers_yaml)
    rsp_urdf = _build_rsp_urdf(base_urdf, control_urdf, controllers_yaml)

    # ── World & Model paths ────────────────────────────────────────
    world_path = os.path.join(pkg_bringup, 'worlds', 'turtlebot3_world.world')

    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_bringup, 'models') + ':' +
              '/opt/ros/jazzy/share/turtlebot3_gazebo/models'
    )

    # ── Gazebo Harmonic ────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': '-r ' + world_path,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Robot State Publisher ──────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': rsp_urdf},
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Bridges ───────────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'subscription_heartbeat_period_ms': 100},
        ],
    )

    # ── Spawn Robot ───────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name',   'wheeled_base',
            '-string', gz_urdf,
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.15',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    # ── Controllers ───────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    delayed_jsb = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner],
    )

    load_diff_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # ── Twist → TwistStamped converter ───────────────────────────
    cmd_vel_converter = Node(
        package='robot_bringup',
        executable='twist_to_twiststamped',
        name='twist_to_twiststamped',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        clock_bridge,
        lidar_bridge,
        spawn_robot,
        delayed_jsb,
        load_diff_after_jsb,
        cmd_vel_converter,
    ])