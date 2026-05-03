import os #to join directories and file names 
import re #to use regular expressions for processing the URDF files
from ament_index_python.packages import get_package_share_directory #finds where a package is installed, so we can access its files like URDFs and RViz configurations
from launch import LaunchDescription  
from launch_ros.actions import Node


def _merge_urdfs(base_path: str, control_path: str) -> str:
    with open(base_path, 'r') as f:
        base = f.read()
    with open(control_path, 'r') as f:
        control = f.read()
    #remove XML header and comments from the control URDF, and extract only the inner content of the <robot> tag. This allows us to merge the two URDFs without conflicts.
    inner = re.sub(r'<\?xml[^>]+\?>\s*', '', control)
    inner = re.sub(r'<!--.*?-->\s*', '', inner, flags=re.DOTALL)
    inner = re.sub(r'<robot[^>]+>\s*', '', inner)
    inner = inner.replace('</robot>', '').strip()
    #insert the inner content of the control URDF into the base URDF just before the closing </robot> tag, effectively merging the two URDFs into one. This allows us to use the combined robot description in our launch file.
    return base.replace('</robot>', inner + '\n</robot>') 

#main function that generates the launch description for RViz. It merges the base URDF and the control URDF, creates a parameter dictionary with the merged URDF, and then defines the nodes to be launched: robot_state_publisher to publish the robot's state, joint_state_publisher_gui to allow interactive manipulation of joint states, and rviz2 to visualize the robot in RViz. Each node is configured with its package, executable, name, output settings, and parameters as needed.
def generate_launch_description():
    pkg_desc = get_package_share_directory('robot_description')

    merged_urdf = _merge_urdfs(
        os.path.join(pkg_desc, 'urdf', 'Wheeled_Base.urdf'), #base URDF
        os.path.join(pkg_desc, 'urdf', 'ros2_control.urdf'), #control URDF
    )
    #create a dictionary with the key 'robot_description'
    # and the value being the merged URDF string. 
    # This dictionary will be passed as parameters to the robot_state_publisher node, allowing it to publish the robot's state based on the combined URDF description.
    robot_description = {'robot_description': merged_urdf}

    return LaunchDescription([  #list of nodes to launch
        
        #robot_state_publisher nodes reads URDF + joint_states and build TF tree
        Node(
            package='robot_state_publisher',    #ros package
            executable='robot_state_publisher', #program inside it 
            name='robot_state_publisher',       #node name
            output='screen',                    #print output to screen         
            parameters=[robot_description],
        ),
        
        #fake joint movement using sliders to rotate wheels manually
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