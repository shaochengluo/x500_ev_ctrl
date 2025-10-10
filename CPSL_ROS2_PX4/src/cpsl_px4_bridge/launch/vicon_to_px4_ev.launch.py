from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='cpsl_uav_7'),
        Node(
            package='cpsl_px4_bridge',
            executable='vicon_to_px4_ev',
            name='vicon_to_px4_ev',
            namespace=ns,
            output='screen',
            parameters=[{
                'vicon_topic': '/vicon/x500_7/x500_7',
                'use_fd_velocity': True,
                'pose_var': 0.0004,
                'vel_var': 0.0025,
                'rate_hz': 30.0,
            }]
        )
    ])
