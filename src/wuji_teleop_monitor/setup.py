from setuptools import setup

package_name = 'wuji_teleop_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wuji',
    maintainer_email='932851972@qq.com',
    description='Qt-based status monitor for Wuji teleop system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = wuji_teleop_monitor.monitor_node:main',
        ],
    },
)
