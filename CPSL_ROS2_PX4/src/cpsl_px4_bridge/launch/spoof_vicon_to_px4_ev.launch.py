from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpsl_px4_bridge',
            executable='spoof_vicon_to_px4_ev',
            name='spoof_vicon_to_px4_ev',
            output='screen',
            parameters=[{
                'vicon_topic': '/vicon/x500_4/x500_4',
                'ev_topic_out': '/cpsl_uav_4/fmu/in/vehicle_visual_odometry',
                'spoof_pos_topic': '/spoofing_position_cmd_vicon',
                'spoof_vel_topic': '/spoofing_velocity_cmd_vicon',
                'spoof_enable_topic': '/redirection/spoofing_enable',
                'use_fd_velocity': True,
                'pos_var': 0.0004,
                'vel_var': 0.0025,
                'ori_var': 0.0001,
                # optional timing controls
                'start_after_s': 20.0,   # set >=0 to auto-start after N sec
                'stop_after_s': 80.0,    # set >=0 to auto-stop after N sec
                'enable_on_start': False # True = spoof active immediately
            }]
        )
    ])
