from setuptools import setup

package_name = 'slam_all'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/all.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_async.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Run lidar + fake odom + slam_toolbox + rviz in one launch',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_odom = slam_all.fake_odom:main',
        ],
    },
)
