#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from serial_motor_demo_msgs.msg import EncoderVals
import tf2_ros
import math
import time

WHEEL_RADIUS = 0.03      # เมตร
WHEEL_BASE   = 0.18      # ระยะล้อซ้าย-ขวา (เมตร)
TICKS_PER_REV = 600

class DiffDriveOdom(Node):
    def __init__(self):
        super().__init__('diff_drive_odom')

        self.sub = self.create_subscription(
            EncoderVals, '/encoder_vals', self.encoder_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left = None
        self.last_right = None
        self.last_time = time.time()

    def encoder_cb(self, msg):
        if self.last_left is None:
            self.last_left = msg.left_encoder
            self.last_right = msg.right_encoder
            return

        dl = msg.left_encoder - self.last_left
        dr = msg.right_encoder - self.last_right

        self.last_left = msg.left_encoder
        self.last_right = msg.right_encoder

        dl_m = 2 * math.pi * WHEEL_RADIUS * dl / TICKS_PER_REV
        dr_m = 2 * math.pi * WHEEL_RADIUS * dr / TICKS_PER_REV

        dc = (dl_m + dr_m) / 2.0
        dtheta = (dr_m - dl_m) / WHEEL_BASE

        self.theta += dtheta
        self.x += dc * math.cos(self.theta)
        self.y += dc * math.sin(self.theta)

        now = self.get_clock().now().to_msg()

        # Odom msg
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta/2)
        odom.pose.pose.orientation.w = math.cos(self.theta/2)
        self.odom_pub.publish(odom)

        # TF
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = odom.pose.pose.orientation.z
        tf.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(tf)

def main():
    rclpy.init()
    node = DiffDriveOdom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
