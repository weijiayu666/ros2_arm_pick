from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_joint_node = arm_pkg.arm_joint_node:main',
            'arm_catch_node = arm_pkg.arm_catch_node:main',
            'test_goal_publisher = arm_pkg.test_goal_publisher:main'
        ],
    },
)
