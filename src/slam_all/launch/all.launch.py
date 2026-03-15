from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1) RPLIDAR
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': False,
        }],
    )

    # 2) static TF: base_link -> laser_frame
    # ใส่ x,y,z, roll,pitch,yaw ให้ตรงตำแหน่งจริง (ตอนนี้ให้ 0 ไปก่อน)
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # 3) fake odom: odom -> base_link (จาก cmd_vel)
    fake_odom = Node(
        package='slam_all',
        executable='fake_odom',
        name='fake_odom',
        output='screen',
        parameters=[{'rate_hz': 30.0}]
    )

    # 4) slam_toolbox async
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=['/home/ubuntu/ros2_ws/src/slam_all/config/slam_async.yaml'],        remappings=[('scan', '/scan')]
    )

    return LaunchDescription([rplidar, laser_tf, fake_odom, slam])
