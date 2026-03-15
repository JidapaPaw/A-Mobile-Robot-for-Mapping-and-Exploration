#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class CmdVelToOdom(Node):
    def __init__(self):
        super().__init__('cmdvel_to_odom')

        # ---- params (ปรับทีหลังได้) ----
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('rate', 30.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.rate = float(self.get_parameter('rate').value)

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.br = TransformBroadcaster(self)

        self.v = 0.0
        self.w = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last = self.get_clock().now()
        self.timer = self.create_timer(1.0/self.rate, self.update)

        self.get_logger().info('Publishing /odom from /cmd_vel (approx)')

    def cb(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        if dt <= 0.0:
            return

        # integrate
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        # quaternion from yaw
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        # Odometry msg
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.pub.publish(odom)

        # TF odom->base_link
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.br.sendTransform(t)

def main():
    rclpy.init()
    node = CmdVelToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
