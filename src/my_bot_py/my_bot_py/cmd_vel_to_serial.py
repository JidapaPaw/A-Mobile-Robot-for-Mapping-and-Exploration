#!/usr/bin/env python3
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # ---- Parameters ----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        # scale cmd_vel -> pwm
        self.declare_parameter('k_lin', 800.0)     # m/s -> pwm
        self.declare_parameter('k_ang', 300.0)     # rad/s -> pwm

        # safety / limits
        self.declare_parameter('max_pwm', 160)     # เริ่มต่ำๆก่อน (ค่อยเพิ่ม)
        self.declare_parameter('timeout_sec', 0.6) # ไม่มี cmd_vel นาน -> หยุด

        # smoothing
        self.declare_parameter('accel_step', 15)   # จำกัดการเปลี่ยน PWM ต่อ tick
        self.declare_parameter('rate_hz', 20.0)    # tick rate (Hz)

        # optional invert/swap (ไว้แก้ทิศทางง่ายๆ)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('swap_lr', False)

        # ---- Read params ----
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)

        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.accel_step = int(self.get_parameter('accel_step').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.invert_linear = bool(self.get_parameter('invert_linear').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)
        self.swap_lr = bool(self.get_parameter('swap_lr').value)

        # ---- State ----
        self.last_cmd_time = time.time()

        self.targetL = 0
        self.targetR = 0
        self.currL = 0
        self.currR = 0

        # ---- Serial open ----
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2.0)  # Arduino reboot time
        self.get_logger().info(f'Opened {self.port} @ {self.baud}')

        # Drain one line (READY/HB etc.)
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                self.get_logger().info(f'arduino: {line}')
        except Exception:
            pass

        # ---- ROS ----
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

        # tick loop (smooth output)
        self.timer_cmd = self.create_timer(1.0 / self.rate_hz, self.tick)

    # --- helpers ---
    def approach(self, cur: int, tgt: int, step: int) -> int:
        if cur < tgt:
            return min(cur + step, tgt)
        if cur > tgt:
            return max(cur - step, tgt)
        return cur

    def send_pwm(self, L: int, R: int):
        L = int(clamp(L, -self.max_pwm, self.max_pwm))
        R = int(clamp(R, -self.max_pwm, self.max_pwm))
        self.ser.write(f"{L} {R}\n".encode())

    # --- ROS callback ---
    def cb(self, msg: Twist):
        self.last_cmd_time = time.time()

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        if self.invert_linear:
            v = -v
        if self.invert_angular:
            w = -w

        # cmd_vel -> pwm target (differential)
        pwmL = v * self.k_lin - w * self.k_ang
        pwmR = v * self.k_lin + w * self.k_ang

        pwmL = int(clamp(pwmL, -self.max_pwm, self.max_pwm))
        pwmR = int(clamp(pwmR, -self.max_pwm, self.max_pwm))

        if self.swap_lr:
            pwmL, pwmR = pwmR, pwmL

        self.targetL = pwmL
        self.targetR = pwmR

    # --- tick loop ---
    def tick(self):
        # timeout -> stop target
        if time.time() - self.last_cmd_time > self.timeout_sec:
            self.targetL = 0
            self.targetR = 0

        # smooth towards target
        self.currL = self.approach(self.currL, self.targetL, self.accel_step)
        self.currR = self.approach(self.currR, self.targetR, self.accel_step)

        # send to Arduino
        try:
            self.send_pwm(self.currL, self.currR)
        except Exception as e:
            self.get_logger().error(f"serial write failed: {e}")


def main():
    rclpy.init()
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # stop motors on exit
    try:
        node.send_pwm(0, 0)
        node.ser.close()
    except Exception:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
