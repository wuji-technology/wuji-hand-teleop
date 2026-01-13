from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'avp_input'

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
            glob('avp_input/config/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install rviz config files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
        'pyyaml>=6.0',
        'scipy>=1.9.0',
        # Note: The following packages need manual installation:
        # - avp-stream: pip install avp-stream
    ],
    zip_safe=True,
    maintainer='Wuji Tech',
    maintainer_email='support@wuji.tech',
    description='Apple Vision Pro input node for Wuji Hand teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avp_input = avp_input.avp_input_node:main',
        ],
    },
)
