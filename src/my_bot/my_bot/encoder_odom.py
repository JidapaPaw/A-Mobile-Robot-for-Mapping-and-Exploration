#!/usr/bin/env python3
import math
import re
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

try:
    import serial  # pyserial
except ImportError:
    serial = None


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return quaternion (x,y,z,w) from yaw (rad)."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class EncoderOdomPolling(Node):
    """
    Polling-mode odometry:
      - every cycle: send 'e\\r' to Arduino
      - read a line that contains two integers: left_ticks right_ticks
      - publish /odom and broadcast TF odom->base_link (DYNAMIC)
    """

    def __init__(self):
        super().__init__('encoder_odom')

        if serial is None:
            raise RuntimeError("pyserial not installed. Install with: sudo apt install python3-serial")

        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/arduino')          # your symlink -> /dev/ttyUSB0
        self.declare_parameter('baud', 115200)
        self.declare_parameter('poll_cmd', 'e')                # command to request encoder
        self.declare_parameter('poll_rate_hz', 20.0)           # how fast we poll encoders

        # Robot geometry (EDIT THESE 3 ให้ตรงหุ่นคุณ)
        self.declare_parameter('wheel_radius', 0.0325)         # meters (e.g. 65mm dia -> r=0.0325)
        self.declare_parameter('wheel_separation', 0.163)       # meters (distance between left-right wheels)
        self.declare_parameter('ticks_per_rev', 2445.0)         # ticks per wheel revolution (encoder resolution)

        # Frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # If Arduino sends cumulative ticks (recommended), keep true
        # If Arduino sends delta ticks per request, set false
        self.declare_parameter('ticks_are_cumulative', True)

        # Optional sign flips if wheel directions are opposite
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        # ---------- Read Parameters ----------
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baud').get_parameter_value().integer_value or 115200)
        self.poll_cmd = self.get_parameter('poll_cmd').get_parameter_value().string_value
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').get_parameter_value().double_value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').get_parameter_value().double_value)
        self.wheel_sep = float(self.get_parameter('wheel_separation').get_parameter_value().double_value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').get_parameter_value().double_value)

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.ticks_are_cum = self.get_parameter('ticks_are_cumulative').get_parameter_value().bool_value
        self.inv_l = self.get_parameter('invert_left').get_parameter_value().bool_value
        self.inv_r = self.get_parameter('invert_right').get_parameter_value().bool_value

        if self.poll_rate_hz <= 0.0:
            self.poll_rate_hz = 20.0

        # ---------- ROS pubs ----------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- State ----------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()
        self.last_left_ticks: Optional[int] = None
        self.last_right_ticks: Optional[int] = None

        # Encoder tick regex: find two ints anywhere in line
        self._rx = re.compile(r'(-?\d+)\s+(-?\d+)')

        # ---------- Serial connect ----------
        self.ser = None
        self._connect_serial()

        # Timer
        period = 1.0 / self.poll_rate_hz
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"encoder_odom started (polling mode). port={self.port}, baud={self.baud}, rate={self.poll_rate_hz}Hz")
        self.get_logger().info(f"wheel_radius={self.wheel_radius} m, wheel_separation={self.wheel_sep} m, ticks_per_rev={self.ticks_per_rev}")
        self.get_logger().info(f"ticks_are_cumulative={self.ticks_are_cum}, invert_left={self.inv_l}, invert_right={self.inv_r}")

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.05,        # non-blocking-ish
                write_timeout=0.2
            )
            # give Arduino time to reset
            time.sleep(1.5)

            # flush junk
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

            self.get_logger().info(f"Connected serial: {self.port} @ {self.baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Serial connect failed: {e}. Will retry...")

    def _write_poll(self):
        if not self.ser:
            return
        cmd = (self.poll_cmd + "\r").encode('utf-8')
        self.ser.write(cmd)

    def _read_line(self) -> Optional[str]:
        """Read one line if available."""
        if not self.ser:
            return None
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                return line
            return None
        except Exception:
            return None

    def _parse_ticks(self, line: str) -> Optional[Tuple[int, int]]:
        m = self._rx.search(line)
        if not m:
            return None
        l = int(m.group(1))
        r = int(m.group(2))
        if self.inv_l:
            l = -l
        if self.inv_r:
            r = -r
        return (l, r)

    def _ticks_to_distance(self, ticks: int) -> float:
        # distance = (ticks / ticks_per_rev) * (2*pi*R)
        return (ticks / self.ticks_per_rev) * (2.0 * math.pi * self.wheel_radius)

    def _publish_odom_tf(self, stamp, vx, wz):
        # Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quat_from_yaw(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # TF odom -> base_link (DYNAMIC)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def _on_timer(self):
        # Reconnect if needed
        if self.ser is None:
            self._connect_serial()
            return
        if not self.ser.is_open:
            self.ser = None
            return

        # poll encoder
        try:
            self._write_poll()
        except Exception:
            self.ser = None
            return

        # read line (try a few quick attempts)
        line = None
        for _ in range(3):
            line = self._read_line()
            if line:
                break

        if not line:
            # no data this cycle
            return

        ticks = self._parse_ticks(line)
        if ticks is None:
            # uncomment to debug bad lines
            # self.get_logger().warn(f"Unparsed line: {line}")
            return

        left_ticks, right_ticks = ticks

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1.0 / self.poll_rate_hz

        # compute delta ticks
        if self.last_left_ticks is None or self.last_right_ticks is None:
            # first sample -> just set baseline, publish zero twist
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_time = now
            self._publish_odom_tf(now.to_msg(), 0.0, 0.0)
            return

        if self.ticks_are_cum:
            dl_ticks = left_ticks - self.last_left_ticks
            dr_ticks = right_ticks - self.last_right_ticks
        else:
            # line already gives delta ticks
            dl_ticks = left_ticks
            dr_ticks = right_ticks

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = now

        # convert to distances
        dl = self._ticks_to_distance(dl_ticks)
        dr = self._ticks_to_distance(dr_ticks)

        # differential drive integration
        ds = 0.5 * (dl + dr)
        dtheta = (dr - dl) / max(self.wheel_sep, 1e-6)

        # update pose (midpoint integration)
        mid_yaw = self.yaw + 0.5 * dtheta
        self.x += ds * math.cos(mid_yaw)
        self.y += ds * math.sin(mid_yaw)
        self.yaw = self._wrap_pi(self.yaw + dtheta)

        # velocities
        vx = ds / dt
        wz = dtheta / dt

        self._publish_odom_tf(now.to_msg(), vx, wz)

    @staticmethod
    def _wrap_pi(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main():
    rclpy.init()
    node = EncoderOdomPolling()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
