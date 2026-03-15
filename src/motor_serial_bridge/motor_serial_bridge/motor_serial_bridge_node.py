import rclpy
from rclpy.node import Node

import serial
import time

from serial_motor_demo_msgs.msg import MotorCommand


class MotorSerialBridge(Node):
    def __init__(self):
        super().__init__('motor_serial_bridge')

        # ===== Parameters =====
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.get_logger().info(f'Opening serial port: {port} @ {baud}')

        # เปิด serial
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        time.sleep(1.0)  # ให้ Arduino รีเซ็ต/พร้อม

        # subscribe topic
        self.sub = self.create_subscription(
            MotorCommand,
            '/motor_cmd',   # <<<<< ต้องตรงกับที่คุณ publish อยู่
            self.cb,
            10
        )

        self.get_logger().info('MotorSerialBridge READY (sub: /motor_cmd)')

    def cb(self, msg: MotorCommand):
        # ส่งเป็น "L R\n" เช่น "50 50\n"
        line = f"{int(msg.left)} {int(msg.right)}\n"
        try:
            self.ser.write(line.encode('ascii'))
            # debug
            # self.get_logger().info(f"TX: {line.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
