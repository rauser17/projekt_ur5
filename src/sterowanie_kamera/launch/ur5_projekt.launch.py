import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    ur_pkg_path = get_package_share_directory('ur_description')    
    xacro_file = os.path.join(ur_pkg_path, 'urdf', 'ur.urdf.xacro')
 
    robot_description_config = xacro.process_file(
        xacro_file, 
        mappings={
            'name': 'ur', 
            'ur_type': 'ur5',
            'joint_limit_params': os.path.join(ur_pkg_path, 'config', 'ur5', 'joint_limits.yaml'),
            'kinematics_params': os.path.join(ur_pkg_path, 'config', 'ur5', 'default_kinematics.yaml'),
            'physical_params': os.path.join(ur_pkg_path, 'config', 'ur5', 'physical_parameters.yaml'),
            'visual_params': os.path.join(ur_pkg_path, 'config', 'ur5', 'visual_parameters.yaml')
        }
    )
    robot_desc = robot_description_config.toxml()

    img_path = os.path.expanduser('~/ros2_ws/test_obraz.png')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='symulator_kamery',
            arguments=[img_path],
            remappings=[('/image_raw', '/burger/image')]
        ),
        Node(
            package='sterowanie_kamera',
            executable='sterownik', 
            output='screen'
        ),
    ])
