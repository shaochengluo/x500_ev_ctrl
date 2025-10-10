from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('cpsl_px4_vicon_controller')
    default_params = os.path.join(pkg_share, 'config', 'mission_example.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Path to a YAML file with node parameters.'),
        Node(
            package='cpsl_px4_vicon_controller',
            executable='waypoint_offboard',
            name='waypoint_offboard',
            output='screen',
            parameters=[params_file]
        )
    ])
