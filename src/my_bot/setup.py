from setuptools import setup
import os
from glob import glob

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ✅ install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # ✅ install config files (เผื่อใช้ slam config)
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='My robot package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_odom = my_bot.encoder_odom:main',
            'cmd_vel_to_motor = my_bot.cmd_vel_to_motor:main',
            'robot_driver = my_bot.robot_driver:main',
            'joy_button_teleop = my_bot.joy_button_teleop:main',
        ],
    },
)
