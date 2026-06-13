from setuptools import setup

package_name = 'wuji_teleop_monitor'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name, package_name + '.ui'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wuji',
    maintainer_email='dev@wuji.tech',
    description='Qt-based teleop launcher and status monitor for the Wuji Hand + Tianji Arm teleoperation stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = wuji_teleop_monitor.ui.run_teleop:main',
            'brake = wuji_teleop_monitor.ui.run_brake:main',
            'camera = wuji_teleop_monitor.ui.run_camera:main',
        ],
    },
)
