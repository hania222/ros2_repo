import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _merge_urdfs(base_path: str, control_path: str) -> str:
    with open(base_path, 'r') as f:
        base = f.read()
    with open(control_path, 'r') as f:
        control = f.read()
    inner = re.sub(r'<\?xml[^>]+\?>\s*', '', control)
    inner = re.sub(r'<!--.*?-->\s*', '', inner, flags=re.DOTALL)
    inner = re.sub(r'<robot[^>]+>\s*', '', inner)
    inner = inner.replace('</robot>', '').strip()
    return base.replace('</robot>', inner + '\n</robot>')


def generate_launch_description():
    pkg_desc = get_package_share_directory('robot_description')

    merged_urdf = _merge_urdfs(
        os.path.join(pkg_desc, 'urdf', 'Wheeled_Base.urdf'),
        os.path.join(pkg_desc, 'urdf', 'ros2_control.urdf'),
    )

    robot_description = {'robot_description': merged_urdf}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])