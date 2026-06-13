from setuptools import setup, find_packages
import os
from glob import glob

# ROS2 package name must match package.xml
ros2_package_name = 'camera'

setup(
    name=ros2_package_name,  # Must match <name> in package.xml
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + ros2_package_name]),  # Use ROS2 package name instead of Python package name
        ('share/' + ros2_package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', ros2_package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files (including stereo_head subdirectory)
        (os.path.join('share', ros2_package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.yaml.template')),
        (os.path.join('share', ros2_package_name, 'config', 'stereo_head'),
            glob('config/stereo_head/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wuji Tech',
    maintainer_email='support@wuji.tech',
    description='Camera integration for Wuji teleoperation system (RealSense/USB/Stereo Head)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_stereo = stereocamera.unified_stereo:main',
        ],
    },
)
