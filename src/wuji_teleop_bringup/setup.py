from setuptools import setup
import os
from glob import glob

package_name = 'wuji_teleop_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Wentao Zhang',
    maintainer_email='zhangwt20011015@gmail.com',
    description='Launch files for Wuji Hand and Tianji Arm teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
