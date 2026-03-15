#!/usr/bin/env python3
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToMotor(Node):
    """
    Subscribes:  /cmd_vel   (geometry_msgs/Twist)
    Sends to Arduino over serial: "L=<pwm> R=<pwm>\n"

    Safety:
      - stop on startup (send 0,0)
      - stop on shutdown (send 0,0)
      - ROS-side timeout: if no cmd_vel for N sec -> send 0,0 continuously
    """

    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        # ---- params ----
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/cmd_vel')

        # robot params (tune later)
        self.declare_parameter('wheel_separation', 0.17)   # meters
        self.declare_parameter('wheel_radius', 0.035)      # meters
        self.declare_parameter('pwm_max', 255)
        self.declare_parameter('pwm_min_move', 35)         # deadzone overcome
        self.declare_parameter('cmd_timeout', 0.35)        # seconds
        self.declare_parameter('send_hz', 20.0)            # send rate

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.L = float(self.get_parameter('wheel_separation').value)
        self.R = float(self.get_parameter('wheel_radius').value)
        self.pwm_max = int(self.get_parameter('pwm_max').value)
        self.pwm_min_move = int(self.get_parameter('pwm_min_move').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.send_hz = float(self.get_parameter('send_hz').value)

        # ---- state ----
        self.last_cmd_time = time.time()
        self.target_left = 0
        self.target_right = 0

        # ---- open serial ----
        self.get_logger().info(f"Opening serial port={self.port} baud={self.baud}")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
        time.sleep(0.25)

        # STOP on startup
        self._send_lr(0, 0)

        # subscriber
        self.sub = self.create_subscription(Twist, self.topic, self.cb_cmd, 10)

        # timer for sending
        period = 1.0 / max(self.send_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(f"Subscribed to {self.topic}")
        self.get_logger().info("READY: cmd_vel_to_motor running")

    def cb_cmd(self, msg: Twist):
        # Store last cmd time
        self.last_cmd_time = time.time()

        v = float(msg.linear.x)     # m/s
        w = float(msg.angular.z)    # rad/s

        # diff drive wheel linear velocities
        v_l = v - (w * self.L / 2.0)
        v_r = v + (w * self.L / 2.0)

        # convert to wheel angular speed (rad/s) if needed:
        # omega = v / R
        # Here we map linear wheel speed -> PWM (simple proportional)
        # NOTE: tune k_v for your robot
        k_v = 180.0  # PWM per (m/s)  <-- ปรับทีหลังได้
        left_pwm = int(v_l * k_v)
        right_pwm = int(v_r * k_v)

        left_pwm = self._apply_deadzone_and_clamp(left_pwm)
        right_pwm = self._apply_deadzone_and_clamp(right_pwm)

        self.target_left = left_pwm
        self.target_right = right_pwm

    def _apply_deadzone_and_clamp(self, pwm: int) -> int:
        pwm = max(-self.pwm_max, min(self.pwm_max, pwm))
        if pwm == 0:
            return 0
        # deadzone overcome
        if 0 < abs(pwm) < self.pwm_min_move:
            return self.pwm_min_move if pwm > 0 else -self.pwm_min_move
        return pwm

    def on_timer(self):
        # ROS-side timeout -> stop
        if (time.time() - self.last_cmd_time) > self.cmd_timeout:
            l = 0
            r = 0
        else:
            l = self.target_left
            r = self.target_right

        self._send_lr(l, r)

    def _send_lr(self, left: int, right: int):
        try:
            line = f"L={left} R={right}\n"
            self.ser.write(line.encode('ascii'))
            # (optional log) comment out if too noisy
            # self.get_logger().info(f"TX: {left},{right}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def destroy_node(self):
        # STOP on shutdown
        try:
            for _ in range(3):
                self._send_lr(0, 0)
                time.sleep(0.02)
        except Exception:
            pass

        try:
            if hasattr(self, 'ser') and self.ser:
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelToMotor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
