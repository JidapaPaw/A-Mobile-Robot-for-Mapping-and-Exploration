from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    motor_port = LaunchConfiguration('motor_port')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_frame = LaunchConfiguration('lidar_frame')

    # static TF base_link -> laser (ปรับค่า xyz ตามตำแหน่งจริง)
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')

    return LaunchDescription([

        # -------- Launch args --------
        DeclareLaunchArgument('motor_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),

        DeclareLaunchArgument('laser_x', default_value='0.10'),
        DeclareLaunchArgument('laser_y', default_value='0.00'),
        DeclareLaunchArgument('laser_z', default_value='0.10'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),

        # 1) Motor driver (serial_motor_demo)
        Node(
            package='serial_motor_demo',
            executable='driver',
            name='motor_driver',
            output='screen',
            parameters=[{
                'serial_port': motor_port
            }]
        ),

        # 2) Odom publisher (ของคุณ)  << ต้องมีตัวนี้ตัวเดียว >>
        Node(
            package='my_bot',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen'
        ),

        # 3) RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': lidar_port,
                'frame_id': lidar_frame
            }]
        ),

        # 4) static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            output='screen',
            arguments=[
                laser_x, laser_y, laser_z,
                laser_roll, laser_pitch, laser_yaw,
                'base_link', lidar_frame
            ]
        ),

        # 5) SLAM toolbox (online async)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'use_scan_matching': True
            }]
        ),
    ])
