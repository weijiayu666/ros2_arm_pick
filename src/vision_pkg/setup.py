from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob(os.path.join('vision_pkg', '*.xml'))),
        (os.path.join('share', package_name), glob(os.path.join('vision_pkg', '*.yaml'))),
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
            'camera_native_node = vision_pkg.camera_native_node:main',
            'opencv_camera_node = vision_pkg.opencv_camera_node:main',
            'ball_detector_node = vision_pkg.ball_detector_node:main',
            'hsv_calibrator = vision_pkg.hsv_calibrator:main',
        ],
    },
)
