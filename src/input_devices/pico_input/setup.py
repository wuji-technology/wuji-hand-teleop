from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'pico_input'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install rviz files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # Install test data files
        (os.path.join('share', package_name, 'test'),
            glob('test/trackingData*.txt')),
        # Install record data files
        (os.path.join('share', package_name, 'record'),
            glob('record/trackingData*.txt')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.21.0',
        'scipy>=1.8.0',
        # xrobotoolkit_sdk 需要单独安装: pip install ~/Desktop/XRoboToolkit-PC-Service-Pybind
    ],
    zip_safe=True,
    maintainer='Wuji Tech',
    maintainer_email='support@wuji.tech',
    description='PICO VR input node for Wuji teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pico_input_node = pico_input.pico_input_node:main',
        ],
    },
)
