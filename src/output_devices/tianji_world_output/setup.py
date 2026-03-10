from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'tianji_world_output'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('tianji_world_output/config/*.MvKDCfg')),
    ],
    package_data={
        package_name: [
            'config/*.MvKDCfg',   # Robot configuration files
        ],
    },
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
        'scipy>=1.8.0',
    ],
    zip_safe=False,
    maintainer='wuji',
    maintainer_email='support@wuji.tech',
    description='Tianji World Output - ROS REP 103 compliant output node for PICO teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tianji_world_output_node = tianji_world_output.tianji_world_output_node:main',
        ],
    },
)
