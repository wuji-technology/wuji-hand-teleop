from setuptools import setup, find_packages

package_name = 'controller'

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
    maintainer='Wuji Robotics',
    maintainer_email='dev@wuji.com',
    description='Unified controller node - state machine with mode switching support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tianji_arm_controller = controller.tianji_arm_node:main',
            'wujihand_controller = controller.wujihand_node:main',
        ],
    },
)
