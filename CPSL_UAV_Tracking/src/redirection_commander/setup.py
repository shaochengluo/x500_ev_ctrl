from setuptools import setup

package_name = 'redirection_commander'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/redirection_commander.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CPSL',
    maintainer_email='you@example.com',
    description='Closed-loop redirection controller for UAV spoofing acceleration commands.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'redirection_commander = redirection_commander.redirection_commander_node:main',
        ],
    },
)
