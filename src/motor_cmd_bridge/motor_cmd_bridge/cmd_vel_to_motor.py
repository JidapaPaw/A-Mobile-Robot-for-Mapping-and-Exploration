#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from serial_motor_demo_msgs.msg import MotorCommand


class CmdVelToMotor(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        # publisher → MotorCommand
        self.motor_pub = self.create_publisher(
            MotorCommand,
            '/motor_cmd',
            10
        )

        # subscriber → cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('CmdVel → MotorCommand bridge started')

    def cmd_vel_callback(self, msg: Twist):
        motor_msg = MotorCommand()

        # ตัวอย่าง mapping แบบง่าย
        linear = msg.linear.x
        angular = msg.angular.z

        # ปรับ scale ตามมอเตอร์จริง
        motor_msg.left  = int(100 * (linear - angular))
        motor_msg.right = int(100 * (linear + angular))

        self.motor_pub.publish(motor_msg)

        self.get_logger().info(
            f'Publish MotorCommand: left={motor_msg.left}, right={motor_msg.right}'
        )


def main():
    rclpy.init()
    node = CmdVelToMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
