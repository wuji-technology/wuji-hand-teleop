from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'wuji_glove'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.yaml.template')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wuji Robotics',
    maintainer_email='dev@wuji.com',
    description='Wuji Glove input device — config-only package for SN binding.',
    license='MIT',
)
