import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import time


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class CmdVelToMotor(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        # Parameters
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_separation', 0.17)  # meters (distance between wheels)
        self.declare_parameter('max_linear', 0.4)         # m/s (for scaling)
        self.declare_parameter('max_angular', 2.0)        # rad/s (for scaling)
        self.declare_parameter('max_pwm', 200)            # limit to protect L298N (0-255)
        self.declare_parameter('deadband', 0.02)          # ignore tiny cmd_vel
        self.declare_parameter('timeout_sec', 0.5)        # stop if no cmd_vel

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.L = float(self.get_parameter('wheel_separation').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        # Serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2.0)  # give Arduino time to reset
            self.get_logger().info(f'Opened serial {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().error(f'Cannot open serial {self.port}: {e}')
            raise

        self.last_cmd_time = time.time()
        self.last_sent = (None, None)

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.timer = self.create_timer(0.1, self.cb_watchdog)

    def cb_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # deadband
        if abs(v) < self.deadband:
            v = 0.0
        if abs(w) < self.deadband:
            w = 0.0

        # Convert v,w to wheel velocities (simple diff drive)
        # v_left = v - w*L/2, v_right = v + w*L/2
        v_l = v - (w * self.L / 2.0)
        v_r = v + (w * self.L / 2.0)

        # Scale to PWM
        # Normalize by max_linear (and allow turning scaling too)
        # This is a practical mapping (tune max_linear / max_angular for your robot)
        pwm_l = int(clamp((v_l / self.max_linear) * self.max_pwm, -self.max_pwm, self.max_pwm)) if self.max_linear > 0 else 0
        pwm_r = int(clamp((v_r / self.max_linear) * self.max_pwm, -self.max_pwm, self.max_pwm)) if self.max_linear > 0 else 0

        self.last_cmd_time = time.time()
        self.send_pwm(pwm_l, pwm_r)

    def cb_watchdog(self):
        if (time.time() - self.last_cmd_time) > self.timeout_sec:
            self.send_pwm(0, 0)

    def send_pwm(self, left: int, right: int):
        if self.last_sent == (left, right):
            return

        line = f"{left} {right}\n"  # <-- ASCII protocol: "L R\n"
        try:
            self.ser.write(line.encode('utf-8'))
            self.last_sent = (left, right)
            # Uncomment for debugging:
            # self.get_logger().info(f"TX: {line.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send_pwm(0, 0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
