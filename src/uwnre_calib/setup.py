from setuptools import find_packages, setup
import glob
import os

package_name = 'uwnre_calib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob.glob(os.path.join('launch', '*.launch.py')),
        ),
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
            'test_cmd_vel_pub = uwnre_calib.test_cmd_vel_pub:main',
            'joystick_speed_calib = uwnre_calib.joystick_speed_calib:main',
            'direct_joystick_odom_logger = uwnre_calib.direct_joystick_odom_logger:main',
        ],
    },
)
