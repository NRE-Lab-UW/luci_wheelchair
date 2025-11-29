from setuptools import find_packages, setup
import glob
import os

package_name = 'uwnre_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = uwnre_base.odom_node:main',
            'twist2joy_node = uwnre_base.twist2joy:main',
            'wheel_angle_scaler = uwnre_base.wheel_angle_scaler:main',
            'joystick_odom_logger = uwnre_base.joystick_odom_logger:main',
        ],
    },
)
