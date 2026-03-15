#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial_motor_demo_msgs.msg import MotorCommand

class CmdVelToMotorCommand(Node):
    def __init__(self):
        super().__init__('cmdvel_to_motorcommand')

        # ===== parameters =====
        self.declare_parameter('max_pwm', 200)
        self.declare_parameter('k_lin', 120.0)
        self.declare_parameter('k_ang', 80.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.invL = bool(self.get_parameter('invert_left').value)
        self.invR = bool(self.get_parameter('invert_right').value)

        self.pub = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

        self.get_logger().info('cmd_vel -> motor_command bridge started')

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        left = self.k_lin * v - self.k_ang * w
        right = self.k_lin * v + self.k_ang * w

        left = max(-self.max_pwm, min(self.max_pwm, int(left)))
        right = max(-self.max_pwm, min(self.max_pwm, int(right)))

        if self.invL:
            left = -left
        if self.invR:
            right = -right

        out = MotorCommand()
        out.is_pwm = True
        out.mot_1_req_rad_sec = float(left)
        out.mot_2_req_rad_sec = float(right)
        self.pub.publish(out)

def main():
    rclpy.init()
    node = CmdVelToMotorCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
