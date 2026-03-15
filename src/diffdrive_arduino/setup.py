from setuptools import setup

package_name = 'diffdrive_arduino'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Diff drive Arduino bridge (cmd_vel -> serial L/R PWM)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_odom  = diffdrive_arduino.cmd_vel_to_odom:main',
            'cmd_vel_to_motor = diffdrive_arduino.cmd_vel_to_motor:main',
        ],
    },
)
