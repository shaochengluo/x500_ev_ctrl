from setuptools import setup

package_name = 'cpsl_px4_vicon_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/waypoint_mission_launch.py']),
        ('share/' + package_name + '/config', ['config/mission_example.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='PX4 Offboard waypoint controller for Vicon room',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_offboard = cpsl_px4_vicon_controller.waypoint_offboard_node:main',
            'ros2_streaming_ts = cpsl_px4_vicon_controller.ros2_streaming_ts:main',
            'ros2_streaming_ts_temp = cpsl_px4_vicon_controller.ros2_streaming_ts_temp:main',
            'ros2_streaming_ts_dual_imu = cpsl_px4_vicon_controller.ros2_streaming_ts_dual_imu:main',
            'ros2_streaming_ts_dual_imu_icm = cpsl_px4_vicon_controller.ros2_streaming_ts_dual_imu_icm:main',
        ],
    },
)
