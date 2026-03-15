#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.time import Time

def yaw_to_quat(yaw):
    # z,w only (planar)
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate_hz', 30.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.w = 0.0

        self.last = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br = TransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(f'Fake odom running: {self.odom_frame} -> {self.base_frame}')

    def cmd_cb(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last = now

        # integrate
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        # TF odom->base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_br.sendTransform(t)

        # Odometry msg
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = FakeOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
