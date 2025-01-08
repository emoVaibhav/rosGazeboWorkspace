from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')

        # Declare target_frame parameter
        self.declare_parameter('from_frame', 'world')
        self.fromFrame = self.get_parameter('from_frame').get_parameter_value().string_value

        self.declare_parameter('to_frame', 'link1')
        self.toFrame = self.get_parameter('to_frame').get_parameter_value().string_value

        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        # Call function in a loop
        period = 1
        self.timer = self.create_timer(period, self.onTimer)

    def onTimer(self):
        fromFrame = self.fromFrame
        toFrame = self.toFrame

        now = rclpy.time.Time()
        try:
            tfStamped = self.tfBuffer.lookup_transform(
                toFrame, fromFrame, now
            )
        except TransformException as e:
            self.get_logger().error(
                'could not transform {} to {}: {}'.format(toFrame, fromFrame, e)
            )
            return
        
        self.get_logger().info(
            """transform of {} to {}
            translation:
            ------------
            x: {}
            y: {}
            z: {}
            rotation:
            ---------
            qx: {}
            qy: {}
            qz: {}
            qw: {}
            """.format(
                toFrame, fromFrame,
                tfStamped.transform.translation.x, 
                tfStamped.transform.translation.y, 
                tfStamped.transform.translation.z,
                tfStamped.transform.rotation.x,
                tfStamped.transform.rotation.y,
                tfStamped.transform.rotation.z,
                tfStamped.transform.rotation.w,
            )
        )

def main():
    rclpy.init()

    node = FrameListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
