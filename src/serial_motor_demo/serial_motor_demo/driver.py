#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time
import math
import serial
from threading import Lock

from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals


class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # ---------------- Parameters ----------------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('encoder_cpr', 0)
        self.declare_parameter('loop_rate', 0)
        self.declare_parameter('serial_debug', False)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.loop_rate = self.get_parameter('loop_rate').value
        self.debug_serial = self.get_parameter('serial_debug').value

        if self.encoder_cpr == 0:
            self.get_logger().warn('ENCODER CPR SET TO 0!! (encoder disabled)')

        if self.loop_rate == 0:
            self.get_logger().warn('LOOP RATE SET TO 0!! (feedback disabled)')

        # ---------------- Topics ----------------
        self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10
        )

        self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)
        self.encoder_pub = self.create_publisher(EncoderVals, 'encoder_vals', 10)

        # ---------------- State ----------------
        self.last_enc_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0

        self.mutex = Lock()

        # ---------------- Serial ----------------
        self.get_logger().info(
            f'Connecting to port {self.serial_port} at {self.baud_rate}'
        )

        self.conn = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=1.0
        )

        self.get_logger().info('Serial connected')

        # Timer for encoder polling (2 Hz is enough)
        self.timer = self.create_timer(0.5, self.check_encoders)

    # ============================================================
    # Motor command handling
    # ============================================================

    def motor_command_callback(self, msg: MotorCommand):
        if msg.is_pwm:
            self.send_pwm(msg.mot_1_req_rad_sec, msg.mot_2_req_rad_sec)
        else:
            # Feedback mode (only if encoder configured)
            if self.encoder_cpr == 0 or self.loop_rate == 0:
                return

            scaler = (1.0 / (2.0 * math.pi)) * self.encoder_cpr * (1.0 / self.loop_rate)
            m1 = msg.mot_1_req_rad_sec * scaler
            m2 = msg.mot_2_req_rad_sec * scaler
            self.send_feedback(m1, m2)

    # ============================================================
    # Encoder handling
    # ============================================================

    def check_encoders(self):
        if self.encoder_cpr == 0:
            return

        resp = self.send_command('e')
        if resp is None:
            return

        try:
            parts = resp.split()
            if len(parts) != 2:
                return

            enc1 = int(parts[0])
            enc2 = int(parts[1])
        except ValueError:
            return

        now = time.time()
        dt = now - self.last_enc_time
        self.last_enc_time = now
        if dt <= 0.0:
            return

        d1 = enc1 - self.last_m1_enc
        d2 = enc2 - self.last_m2_enc
        self.last_m1_enc = enc1
        self.last_m2_enc = enc2

        rads_per_ct = (2.0 * math.pi) / self.encoder_cpr
        spd1 = d1 * rads_per_ct / dt
        spd2 = d2 * rads_per_ct / dt

        vel_msg = MotorVels()
        vel_msg.mot_1_rad_sec = spd1
        vel_msg.mot_2_rad_sec = spd2
        self.speed_pub.publish(vel_msg)

        enc_msg = EncoderVals()
        enc_msg.mot_1_enc_val = enc1
        enc_msg.mot_2_enc_val = enc2
        self.encoder_pub.publish(enc_msg)

    # ============================================================
    # Serial helpers
    # ============================================================

    def send_pwm(self, m1_pwm, m2_pwm):
        self.send_command(f'o {int(m1_pwm)} {int(m2_pwm)}')

    def send_feedback(self, m1_ct, m2_ct):
        self.send_command(f'm {int(m1_ct)} {int(m2_ct)}')

    def send_command(self, cmd: str):
        with self.mutex:
            try:
                cmd = cmd.strip() + '\r'
                self.conn.write(cmd.encode('utf-8'))

                if self.debug_serial:
                    self.get_logger().info(f'Sent: {cmd.strip()}')

                line = ''
                while True:
                    ch = self.conn.read(1)
                    if ch == b'':
                        self.get_logger().warn(f'Serial timeout on command: {cmd.strip()}')
                        return None

                    try:
                        c = ch.decode('utf-8')
                    except UnicodeDecodeError:
                        continue

                    if c == '\r':
                        break

                    line += c

                if self.debug_serial:
                    self.get_logger().info(f'Received: {line}')

                return line.strip()

            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
                return None

    def destroy(self):
        if self.conn and self.conn.is_open:
            self.conn.close()


# ============================================================
# Main
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
