import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

class FixedFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('fixed_frame_tf2_broadcaster')
        self.br = TransformBroadcaster(self)
        period = 0.1
        self.timer = self.create_timer(period, self.broadcastTimerCallback)

        self.declare_parameter('parent_frame_name', 'world')
        self.declare_parameter('child_frame_name', 'link1')

    def broadcastTimerCallback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('parent_frame_name').get_parameter_value().string_value
        t.child_frame_id = self.get_parameter('child_frame_name').get_parameter_value().string_value

        # Vary the transformation with some math manipulation
        seconds, _ = self.get_clock().now().seconds_nanoseconds()
        ang = seconds * math.pi

        t.transform.translation.x = 10 * math.sin(ang)
        t.transform.translation.y = 10 * math.cos(ang)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = FixedFrameBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
