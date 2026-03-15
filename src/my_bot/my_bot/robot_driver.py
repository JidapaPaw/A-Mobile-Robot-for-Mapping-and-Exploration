#!/usr/bin/env python3
import math
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None


def quat_from_yaw(yaw: float):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))


class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')

        if serial is None:
            raise RuntimeError("pyserial not installed: pip install pyserial")

        self.declare_parameter('port',             '/dev/arduino')
        self.declare_parameter('baud',             115200)
        self.declare_parameter('wheel_radius',     0.035)
        self.declare_parameter('wheel_separation', 0.17)
        self.declare_parameter('ticks_per_rev',    2445.0)
        self.declare_parameter('pwm_max',          255)
        self.declare_parameter('pwm_min',          80)
        self.declare_parameter('max_linear_vel',   0.5)
        self.declare_parameter('poll_rate_hz',     20.0)
        self.declare_parameter('odom_frame',       'odom')
        self.declare_parameter('base_frame',       'base_link')
        self.declare_parameter('invert_left',      False)
        self.declare_parameter('invert_right',     False)
        self.declare_parameter('left_pwm_trim',    0)
        self.declare_parameter('cmd_timeout',      0.5)
        self.declare_parameter('use_pid_mode',     False)
        self.declare_parameter('debug_serial',     False)

        self.port          = self.get_parameter('port').value
        self.baud          = self.get_parameter('baud').value
        self.wheel_radius  = self.get_parameter('wheel_radius').value
        self.wheel_sep     = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.pwm_max       = self.get_parameter('pwm_max').value
        self.pwm_min       = self.get_parameter('pwm_min').value
        self.max_lin_vel   = self.get_parameter('max_linear_vel').value
        self.poll_rate_hz  = self.get_parameter('poll_rate_hz').value or 20.0
        self.odom_frame    = self.get_parameter('odom_frame').value
        self.base_frame    = self.get_parameter('base_frame').value
        self.inv_l         = self.get_parameter('invert_left').value
        self.inv_r         = self.get_parameter('invert_right').value
        self.left_trim     = self.get_parameter('left_pwm_trim').value
        self.cmd_timeout   = self.get_parameter('cmd_timeout').value
        self.use_pid_mode  = self.get_parameter('use_pid_mode').value
        self.debug_serial  = self.get_parameter('debug_serial').value

        self.create_subscription(Twist, '/cmd_vel', self._cmd_callback, 10)
        self.odom_pub       = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x         = 0.0
        self.y         = 0.0
        self.yaw       = 0.0
        self.last_time = self.get_clock().now()
        self.last_l    = None
        self.last_r    = None
        self._re       = re.compile(r'(-?\d+)[,\s]+(-?\d+)')

        self._pending_pl    = 0
        self._pending_pr    = 0
        self._last_cmd_time = time.time()

        self.ser = None
        self._connect()

        self.create_timer(1.0 / self.poll_rate_hz, self._timer)

        self.get_logger().info(
            f"robot_driver ready | port={self.port} baud={self.baud} "
            f"rate={self.poll_rate_hz}Hz "
            f"mode={'PID(m)' if self.use_pid_mode else 'RAW_PWM(o)'}"
        )

    def _connect(self):
        try:
            self.ser = serial.Serial(
                self.port, self.baud, timeout=0.1, write_timeout=0.2
            )
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'r\r')
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Serial OK: {self.port}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Serial failed: {e}")

    def _send_motor(self, pl: int, pr: int) -> bool:
        prefix = b'm' if self.use_pid_mode else b'o'
        cmd = prefix + f" {pl} {pr}\r".encode()
        if self.debug_serial:
            self.get_logger().info(f"[TX motor] {cmd!r}")
        try:
            self.ser.write(cmd)
            return True
        except Exception as e:
            self.get_logger().warn(f"Write(motor) error: {e}")
            return False

    def _send_read_encoders(self) -> bool:
        if self.debug_serial:
            self.get_logger().info("[TX enc] b'e\\r'")
        try:
            self.ser.write(b'e\r')
            return True
        except Exception as e:
            self.get_logger().warn(f"Write(enc) error: {e}")
            return False

    def _read_line(self):
        try:
            raw  = self.ser.readline()
            line = raw.decode('utf-8', errors='ignore').strip()
            if self.debug_serial and line:
                self.get_logger().info(f"[RX] '{line}'")
            if line in ('OK', 'Invalid Command', 'READY', ''):
                return None
            return line
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")
            return None

    def _vel_to_pwm(self, vel: float) -> int:
        if abs(vel) < 1e-4:
            return 0
        ratio = abs(vel) / max(self.max_lin_vel, 1e-6)
        pwm   = int(ratio * self.pwm_max)
        pwm   = max(self.pwm_min, min(self.pwm_max, pwm))
        return pwm if vel > 0 else -pwm

    def _trim_left(self, pwm: int) -> int:
        if pwm == 0:
            return 0
        return (min(self.pwm_max,  pwm + self.left_trim) if pwm > 0
                else max(-self.pwm_max, pwm - self.left_trim))

    def _cmd_callback(self, msg: Twist):
        v  = msg.linear.x
        w  = msg.angular.z
        vl = v - w * self.wheel_sep / 2.0
        vr = v + w * self.wheel_sep / 2.0

        pl = self._vel_to_pwm(vl)
        pr = self._vel_to_pwm(vr)

        self._pending_pl    = self._trim_left(pl) * (-1 if self.inv_l else 1)
        self._pending_pr    = pr                  * (-1 if self.inv_r else 1)
        self._last_cmd_time = time.time()

    def _timer(self):
        if not self.ser or not self.ser.is_open:
            self.ser = None
            self._connect()
            return

        if time.time() - self._last_cmd_time > self.cmd_timeout:
            self._pending_pl = 0
            self._pending_pr = 0

        if not self._send_motor(self._pending_pl, self._pending_pr):
            self.ser = None
            return
        self._read_line()

        if not self._send_read_encoders():
            self.ser = None
            return

        line = self._read_line()
        if not line:
            return

        m = self._re.search(line)
        if not m:
            self.get_logger().debug(f"parse fail: '{line}'")
            return

        lt = int(m.group(1))
        rt = int(m.group(2))

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1.0 / self.poll_rate_hz

        if self.last_l is None:
            self.last_l    = lt
            self.last_r    = rt
            self.last_time = now
            self._publish(now.to_msg(), 0.0, 0.0)
            return

        dl = lt - self.last_l
        dr = rt - self.last_r
        self.last_l    = lt
        self.last_r    = rt
        self.last_time = now

        c   = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
        dl *= c
        dr *= c
        ds  = 0.5 * (dl + dr)
        dth = (dr - dl) / max(self.wheel_sep, 1e-6)

        self.x   += ds * math.cos(self.yaw + 0.5 * dth)
        self.y   += ds * math.sin(self.yaw + 0.5 * dth)
        self.yaw  = self._wrap(self.yaw + dth)

        self._publish(now.to_msg(), ds / dt, dth / dt)

    def _publish(self, stamp, vx: float, wz: float):
        qx, qy, qz, qw = quat_from_yaw(self.yaw)

        cov_p     = [0.0] * 36
        cov_p[0]  = 0.05
        cov_p[7]  = 0.05
        cov_p[35] = 0.1

        cov_t     = [0.0] * 36
        cov_t[0]  = 0.05
        cov_t[35] = 0.1

        odom = Odometry()
        odom.header.stamp            = stamp
        odom.header.frame_id         = self.odom_frame
        odom.child_frame_id          = self.base_frame
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance         = cov_p
        odom.twist.twist.linear.x    = vx
        odom.twist.twist.angular.z   = wz
        odom.twist.covariance        = cov_t
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp            = stamp
        t.header.frame_id         = self.odom_frame
        t.child_frame_id          = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x    = qx
        t.transform.rotation.y    = qy
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _wrap(a: float) -> float:
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = RobotDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node.ser and node.ser.is_open:
                node.ser.write(b'o 0 0\r')
                time.sleep(0.1)
                node.ser.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
