#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from serial import Serial
from serial.serialutil import SerialException

from serial_motor_demo_msgs.msg import MotorCommand


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


class MotorCmdToArduino(Node):
    def __init__(self):
        super().__init__('motor_cmd_to_arduino')

        # ====== params ======
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('deadband', 0)     # เช่น 5 ถ้าจะตัดสัญญาณเล็กๆ
        self.declare_parameter('topic', '/motor_cmd')
        self.declare_parameter('send_rate_hz', 30.0)
        self.declare_parameter('watchdog_ms', 300)  # ถ้าไม่รับคำสั่งเกินนี้ -> stop

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.deadband = int(self.get_parameter('deadband').value)
        self.topic = self.get_parameter('topic').value
        self.send_rate = float(self.get_parameter('send_rate_hz').value)
        self.watchdog_ms = int(self.get_parameter('watchdog_ms').value)

        self.last_left = 0
        self.last_right = 0
        self.last_cmd_time = self.get_clock().now()

        self.ser = None
        self._open_serial()

        self.sub = self.create_subscription(
            MotorCommand,
            self.topic,
            self.cb_motor,
            10
        )

        self.timer = self.create_timer(1.0 / self.send_rate, self.tick)

        self.get_logger().info(
            f"Listening {self.topic} -> Serial {self.port}@{self.baud}, "
            f"max_pwm={self.max_pwm}, watchdog={self.watchdog_ms}ms"
        )

    def _open_serial(self):
        try:
            self.ser = Serial(self.port, self.baud, timeout=0.02)
            self.get_logger().info(f"Serial opened: {self.port} @ {self.baud}")
        except SerialException as e:
            self.ser = None
            self.get_logger().error(f"Serial open failed: {e}")

    def cb_motor(self, msg: MotorCommand):
        left = int(msg.left)
        right = int(msg.right)

        # deadband
        if abs(left) < self.deadband:
            left = 0
        if abs(right) < self.deadband:
            right = 0

        # clamp
        left = clamp(left, -self.max_pwm, self.max_pwm)
        right = clamp(right, -self.max_pwm, self.max_pwm)

        self.last_left = left
        self.last_right = right
        self.last_cmd_time = self.get_clock().now()

    def _write_line(self, line: str):
        if self.ser is None:
            self._open_serial()
            return
        try:
            self.ser.write(line.encode('ascii'))
        except SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def tick(self):
        # watchdog: ถ้าไม่มีคำสั่งนานเกิน -> ส่ง E (stop)
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e6
        if dt > self.watchdog_ms:
            self._write_line("E\n")
            return

        # ส่ง L.. R.. ทุก tick
        self._write_line(f"L{self.last_left} R{self.last_right}\n")


def main():
    rclpy.init()
    node = MotorCmdToArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser:
                node.ser.write(b"E\n")
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
