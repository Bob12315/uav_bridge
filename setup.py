from setuptools import setup

package_name = 'uav_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/camera_mavlink.launch.py',
            'launch/mavlink_tx.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='level6',
    maintainer_email='your_email@example.com',
    description='MAVLink <-> ROS2 bridge (ArduPilot SITL)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mavlink_bridge = uav_bridge.mavlink_bridge:main',
            'mavlink_dump = uav_bridge.mavlink_dump:main',
            'mavlink_tx = uav_bridge.mavlink_tx:main',
        ],
    },
)
