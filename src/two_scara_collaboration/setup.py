from setuptools import setup
from glob import glob
import os

package_name = 'two_scara_collaboration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Two SCARA robots collaboration package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_robots = two_scara_collaboration.spawn_robots:main',
            'scara_left_motion_planner = two_scara_collaboration.scara_left_motion_planner:main',
            'scara_right_motion_planner = two_scara_collaboration.scara_right_motion_planner:main',
            'block_spawner = two_scara_collaboration.block_spawner:main',
            'block_publisher = two_scara_collaboration.block_publisher:main',
            'gripper_action_server = two_scara_collaboration.gripper_action_server:main',
            'automation_manager = two_scara_collaboration.automation_manager:main',
            'conveyor_driver = two_scara_collaboration.conveyor_driver:main',
        ],
    },
)
