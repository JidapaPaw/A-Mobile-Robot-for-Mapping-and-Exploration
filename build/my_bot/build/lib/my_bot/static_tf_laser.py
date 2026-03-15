import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class StaticTFLaser(Node):
    def __init__(self):
        super().__init__('static_tf_laser')
        self.declare_parameter('parent', 'base_link')
        self.declare_parameter('child', 'laser_frame')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.10)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('parent').value
        t.child_frame_id = self.get_parameter('child').value

        t.transform.translation.x = float(self.get_parameter('x').value)
        t.transform.translation.y = float(self.get_parameter('y').value)
        t.transform.translation.z = float(self.get_parameter('z').value)

        # Keep rotation identity (simple) - if you need rpy->quat, tell me
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static TF: {t.header.frame_id} -> {t.child_frame_id}")


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFLaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
