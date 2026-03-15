#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    lidar_port  = LaunchConfiguration('lidar_port')
    lidar_frame = LaunchConfiguration('lidar_frame')
    laser_x     = LaunchConfiguration('laser_x')
    laser_y     = LaunchConfiguration('laser_y')
    laser_z     = LaunchConfiguration('laser_z')
    laser_roll  = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw   = LaunchConfiguration('laser_yaw')

    return LaunchDescription([

        DeclareLaunchArgument('lidar_port',  default_value='/dev/rplidar'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        DeclareLaunchArgument('laser_x',     default_value='0.10'),
        DeclareLaunchArgument('laser_y',     default_value='0.00'),
        DeclareLaunchArgument('laser_z',     default_value='0.10'),
        DeclareLaunchArgument('laser_roll',  default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw',   default_value='0.0'),

        # 1) Robot Driver (ทำหน้าที่ motor + encoder + odom ครบในตัวเดียว)
        Node(
            package='my_bot',
            executable='robot_driver',
            name='robot_driver',
            output='screen',
            parameters=[{
                'port':             '/dev/arduino',
                'baud':             115200,
                'wheel_radius':     0.035,
                'wheel_separation': 0.175,
                'ticks_per_rev':    2445.0,
                'pwm_max':          255,
                'pwm_min':          80,
                'max_linear_vel':   0.3,
                'poll_rate_hz':     20.0,
                'invert_left':      False,
                'invert_right':     False,
                'debug_serial':     False,
            }]
        ),

        # ❌ encoder_odom ถูกลบออก — แย่ง /dev/arduino กับ robot_driver

        # 2) RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port':      lidar_port,
                'serial_baudrate':  115200,
                'frame_id':         lidar_frame,
                'angle_compensate': True,
                'scan_mode':        'Standard',
            }]
        ),

        # 3) Static TF: base_link -> laser
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

        # 4) SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time':             False,
                'odom_frame':               'odom',
                'base_frame':               'base_link',
                'map_frame':                'map',
                'scan_topic':               '/scan',
                'mode':                     'mapping',
                'max_laser_range':          6.0,
                'throttle_scans':           1,
                'resolution':               0.04,
                'minimum_time_interval':    3.0,
                'map_update_interval':      3.0,
                'transform_timeout':        1.0,
                'transform_publish_period': 0.02,
                'minimum_travel_distance':  0.0,
                'minimum_travel_heading':   0.0,
                'use_scan_matching':        True,
                'use_scan_barycenter':      True,
                'do_loop_closing':          True,
                'loop_search_maximum_distance':         6.0,
                'correlation_search_space_dimension':   0.5,
                'correlation_search_space_resolution':  0.01,
                'correlation_search_space_smear_deviation': 0.1,
                'distance_variance_penalty': 1.0,
                'angle_variance_penalty':    1.5,
                'minimum_angle_penalty':     0.9,
                'minimum_distance_penalty':  0.5,
                'use_response_expansion':    True,
            }]
        ),

        # 5) Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # 6) Joy teleop
        Node(
            package='my_bot',
            executable='joy_button_teleop',
            name='joy_button_teleop',
            output='screen',
            parameters=[{
                'linear_speed':  0.25,
                'angular_speed': 1.5,
                'btn_forward':   0,
                'btn_backward':  3,
                'btn_left':      2,
                'btn_right':     1,
                'btn_stop':      7,
            }]
        ),
    ])
