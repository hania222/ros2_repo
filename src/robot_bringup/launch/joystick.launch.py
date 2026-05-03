from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_bringup',
            executable='joystick_ws_node',
            name='joystick_ws_node',
            output='screen',
        )
    ])