#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math
import time


class CmdVelToMotor(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        # ===== Robot Parameters =====
        self.wheel_radius = 0.05      # ปรับตามจริง (เมตร)
        self.wheel_base = 0.20        # ระยะล้อ (เมตร)
        self.ticks_per_rev = 2445      # ปรับตาม encoder จริง

        # ===== Pose =====
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left = 0
        self.last_right = 0

        # ===== Serial =====
        try:
            self.serial_port = serial.Serial('/dev/arduino', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info("Connected to Arduino")
        except:
            self.get_logger().error("Serial error")
            exit()

        # ===== ROS Interface =====
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer 50Hz
        self.create_timer(0.02, self.update_odometry)

    # =========================
    # SEND MOTOR COMMAND
    # =========================
    def cmd_callback(self, msg):

        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        pwm_left = int(v_left * 200)
        pwm_right = int(v_right * 200)

        self.serial_port.write(f"{pwm_left} {pwm_right}\n".encode())

    # =========================
    # READ ENCODER + CALC ODOM
    # =========================
    def update_odometry(self):

        if self.serial_port.in_waiting == 0:
            return

        line = self.serial_port.readline().decode().strip()

        try:
            left_enc, right_enc = map(int, line.split())
        except:
            return

        d_left = left_enc - self.last_left
        d_right = right_enc - self.last_right

        self.last_left = left_enc
        self.last_right = right_enc

        # convert tick → meter
        dist_left = (2 * math.pi * self.wheel_radius) * (d_left / self.ticks_per_rev)
        dist_right = (2 * math.pi * self.wheel_radius) * (d_right / self.ticks_per_rev)

        d_center = (dist_left + dist_right) / 2.0
        d_theta = (dist_right - dist_left) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        now = self.get_clock().now()

        # ===== Publish Odom =====
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)

        # ===== TF Broadcast =====
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
