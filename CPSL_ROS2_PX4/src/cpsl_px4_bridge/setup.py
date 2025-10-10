from setuptools import setup

package_name = 'cpsl_px4_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # make sure cpsl_px4_bridge/__init__.py exists
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install BOTH launch files
        ('share/' + package_name + '/launch', [
            'launch/vicon_to_px4_ev.launch.py',
            'launch/spoof_vicon_to_px4_ev.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpsl',
    maintainer_email='cpsl@example.com',
    description='Vicon → PX4 External Vision bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # original bridge
            'vicon_to_px4_ev = cpsl_px4_bridge.vicon_to_px4_ev:main',
            # NEW spoofer bridge (matches your new filename)
            'spoof_vicon_to_px4_ev = cpsl_px4_bridge.spoof_vicon_to_px4_ev:main',
        ],
    },
)
