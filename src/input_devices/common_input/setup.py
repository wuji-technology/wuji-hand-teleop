from setuptools import setup, find_packages

package_name = 'common_input'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
    ],
    zip_safe=True,
    maintainer='Wuji Tech',
    maintainer_email='support@wuji.tech',
    description='Common input utilities for Wuji Hand teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
