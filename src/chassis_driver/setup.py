from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chassis_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='myhoney',
    maintainer_email='myhoney@todo.todo',
    description='Chassis driver and arm-bridge nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'chassis_driver_node = chassis_driver.chassis_driver_node:main',
            'chassis_arm_bridge = chassis_driver.chassis_arm_bridge:main',
        ],
    },
)