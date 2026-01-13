from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'wujihand_ik'

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
            glob('wujihand_ik/config/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('wujihand_ik/launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=2.0.0',
        'pyyaml>=6.0',
        # Note: The following packages need manual installation:
        # - wujihandpy: pip install wujihandpy
        # - wuji_retargeting: Install from GitHub
        #   git clone --recurse-submodules https://github.com/Wuji-Technology-Co-Ltd/wuji_retargeting.git
        #   cd wuji_retargeting && pip install -e .
    ],
    zip_safe=True,
    maintainer='Wentao Zhang',
    maintainer_email='zhangwt20011015@gmail.com',
    description='IK and retargeting nodes for Wuji Hand teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wujihand_retargeting = wujihand_ik.ik_node:main',
            'ik_node = wujihand_ik.ik_node:main',
        ],
    },
)

