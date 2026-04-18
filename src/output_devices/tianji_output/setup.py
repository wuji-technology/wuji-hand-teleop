from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'tianji_output'

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
            glob('tianji_output/config/*.MvKDCfg') + glob('tianji_output/config/*.yaml')),
    ],
    package_data={
        package_name: [
            '_internal/lib/*.so',  # Linux shared libraries
            '_internal/lib/*.dll', # Windows DLLs (if needed)
            'config/*.MvKDCfg',    # Robot configuration files
            'data/**/*',           # All data files recursively
        ],
    },
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
    ],
    zip_safe=False,  # Must be False because of shared libraries
    maintainer='wuji',
    maintainer_email='your_email@example.com',
    description='Tianji arm hardware interface library - provides low-level control interfaces such as CartesianController',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Tool scripts
            'analyze_recording = tianji_output.analyze_recording:main',
            'debug_arm_axis = tianji_output.debug_arm_axis:main',
        ],
    },
)
