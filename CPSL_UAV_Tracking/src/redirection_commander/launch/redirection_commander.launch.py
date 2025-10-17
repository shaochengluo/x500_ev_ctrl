from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # pose_topic   = LaunchConfiguration('pose_topic')
    # vel_topic    = LaunchConfiguration('vel_topic')
    # target_topic = LaunchConfiguration('target_topic')

    return LaunchDescription([
        # DeclareLaunchArgument('pose_topic',   default_value='/tracked_uav_pose_vicon'),
        # DeclareLaunchArgument('vel_topic',    default_value='/tracked_uav_velocity_vicon'),
        # DeclareLaunchArgument('target_topic', default_value='/redirection_commander/target'),

        Node(
            package='redirection_commander',
            executable='redirection_commander',
            name='redirection_commander',
            output='screen',
            parameters=[{
                'lambda_a': 0.5,
                'v_ref_max': 0.2,
                'v_spoof_max': 0.2,  # [m/s]
                'deadband_radius': 0.3,
                'control_rate_hz': 10.0,
                'frame_id': 'vicon/world',
                'attack_start_delay_s': 0.0,
                'base_Kp_pos': 0.5, 'Ki_pos': 0.005, 'Kd_pos': 0.1, 'pos_gain_decay': 0.1,
                'Kp_vel': 0.05, 'Ki_vel': 0.002, 'Kd_vel': 0.001,
            }],
            # remappings=[
            #     # ('tracked_uav_pose', pose_topic),
            #     # ('tracked_uav_velocity', vel_topic),
            #     # ('target', target_topic),
            #     ('spoofing_accel_cmd', '/spoofing_accel_cmd'),
            #     ('spoofing_velocity_cmd', '/spoofing_velocity_cmd'),
            #     ('v_ref', '/redirection_commander/v_ref'),
            #     ('delta_s_pid', '/redirection_commander/delta_s_pid'),
            #     ('state', '/redirection_commander/state'),
            # ],
        )
    ])