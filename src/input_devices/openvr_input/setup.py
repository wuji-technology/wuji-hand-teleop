from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'openvr_input'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wuji',
    maintainer_email='wuji@example.com',
    description='ROS2 input device for OpenVR Trackers (HTC Vive Trackers)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openvr_input = openvr_input.openvr_input_node:main',
        ],
    },
)
