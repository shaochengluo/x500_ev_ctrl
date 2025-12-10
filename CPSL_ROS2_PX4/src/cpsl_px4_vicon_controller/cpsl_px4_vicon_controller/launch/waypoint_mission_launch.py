from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('cpsl_px4_vicon_controller')
    default_params = os.path.join(pkg_share, 'config', 'mission_example.yaml')

    # Launch args (DON'T call .perform() here)
    params_file   = LaunchConfiguration('params_file')
    ns            = LaunchConfiguration('ns')
    log_level     = LaunchConfiguration('log_level')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    respawn       = LaunchConfiguration('respawn')
    respawn_delay = LaunchConfiguration('respawn_delay')
    emulate_tty   = LaunchConfiguration('emulate_tty')

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
            default_value=default_params,
            description='Path to YAML with node parameters.'
        ),
        DeclareLaunchArgument('ns',
            default_value='/cpsl_uav_10',
            description='Vehicle namespace used by PX4 uXRCE topics.'
        ),
        DeclareLaunchArgument('log_level',
            default_value='info',
            description='Logging level (debug|info|warn|error|fatal).'
        ),
        DeclareLaunchArgument('use_sim_time',
            default_value='false',
            description='Use /clock if provided (Gazebo, etc.).'
        ),
        DeclareLaunchArgument('respawn',
            default_value='true',
            description='Respawn node if it crashes.'
        ),
        DeclareLaunchArgument('respawn_delay',
            default_value='2.0',
            description='Seconds to wait before respawn.'
        ),
        DeclareLaunchArgument('emulate_tty',
            default_value='true',
            description='Force flushed, colorized logs.'
        ),

        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),

        PushRosNamespace(ns),

        Node(
            package='cpsl_px4_vicon_controller',
            executable='waypoint_offboard',
            name='waypoint_offboard',
            output='screen',
            emulate_tty=emulate_tty,            # pass substitution
            arguments=['--ros-args', '--log-level', log_level],
            respawn=respawn,                     # pass substitution
            respawn_delay=respawn_delay,         # pass substitution
            parameters=[
                params_file,                     # YAML file
                {
                    'ns': ns,
                    'use_sim_time': use_sim_time,
                    # Optional quick overrides while tuning:
                    # 'xy_accept': 0.25,
                    # 'z_accept': 0.15,
                    # 'hold_time': 1.0,
                    # 'takeoff_hold_s': 2.0,
                    # 'speed_gate_enable': True,
                    # 'speed_gate_max': 0.30,
                }
            ]
        )
    ])
