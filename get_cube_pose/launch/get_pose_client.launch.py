from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'get_cube_pose'
    pkg_share_path = get_package_share_directory(package_name)
    rviz_config_file = os.path.join(pkg_share_path, 'config', 'sim.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        Node(
            package='simple_grasping',
            executable='basic_grasping_perception_node',
            name='basic_grasping_perception_node',
            output='screen',
            parameters=[{'debug_topics': True}]
        ),
        Node(
            package='get_cube_pose',
            executable='get_pose_client',
            name='get_pose_client',
            output='screen'
        ),
    ])