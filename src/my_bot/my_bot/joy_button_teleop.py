#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyButtonTeleop(Node):
    def __init__(self):
        super().__init__('joy_button_teleop')

        # ===== Parameters =====
        self.declare_parameter('linear_speed',  0.25)
        self.declare_parameter('angular_speed', 1.2)

        # Button index (เปลี่ยนได้ถ้า mapping ไม่ตรง)
        self.declare_parameter('btn_forward',   0)   # A
        self.declare_parameter('btn_backward',  3)   # Y
        self.declare_parameter('btn_left',      2)   # X
        self.declare_parameter('btn_right',     1)   # B
        self.declare_parameter('btn_stop',      7)   # START

        self.lin  = self.get_parameter('linear_speed').value
        self.ang  = self.get_parameter('angular_speed').value
        self.fwd  = self.get_parameter('btn_forward').value
        self.bwd  = self.get_parameter('btn_backward').value
        self.lft  = self.get_parameter('btn_left').value
        self.rgt  = self.get_parameter('btn_right').value
        self.stp  = self.get_parameter('btn_stop').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self.get_logger().info("joy_button_teleop ready!")
        self.get_logger().info(f"A=forward | Y=backward | X=left | B=right | START=stop")

    def _joy_cb(self, msg: Joy):
        twist = Twist()
        btns  = msg.buttons

        def pressed(i):
            return len(btns) > i and btns[i] == 1

        if pressed(self.stp):
            pass  # ทุกค่าเป็น 0 อยู่แล้ว
        elif pressed(self.fwd):
            twist.linear.x  =  self.lin
        elif pressed(self.bwd):
            twist.linear.x  = -self.lin
        elif pressed(self.lft):
            twist.angular.z =  self.ang
        elif pressed(self.rgt):
            twist.angular.z = -self.ang

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyButtonTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
