import sys

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import tf_transformations

class StaticFramePublisher(Node):
    def __init__(self, transformation):
        super().__init__('static_tf2_broadcaster')

        self._tf_publisher = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.makeTransforms(transformation)

    def makeTransforms(self, transformation):
        stStamped = TransformStamped()
        stStamped.header.stamp = self.get_clock().now().to_msg()
        # set parent frame name
        stStamped.header.frame_id = 'world'
        # set child frame name
        stStamped.child_frame_id = sys.argv[1]
        stStamped.transform.translation.x = float(sys.argv[2])
        stStamped.transform.translation.y = float(sys.argv[3])
        stStamped.transform.translation.z = float(sys.argv[4])

        quat = tf_transformations.quaternion_from_euler(
            float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])
        )

        stStamped.transform.rotation.x = quat[0]
        stStamped.transform.rotation.y = quat[1]
        stStamped.transform.rotation.z = quat[2]
        stStamped.transform.rotation.w = quat[3]

        # Publish static transforms once at startup
        self._tf_publisher.sendTransform(stStamped)

def main():
    logger = rclpy.logging.get_logger('logger')

    # validation
    if len(sys.argv) < 8:
        logger.error(
            'Invalid number of parameters. Usage: \n',
            '$ ros2 run learning_tf2_py static_turtle_tf2_broadcaster',
            'child_frame_name x y z roll pitch yaw'
        )
        sys.exit(0)

    if sys.argv[1] == 'world':
        logger.error('static frame cannot be named "world"')
        sys.exit(0)

    # initialize node
    rclpy.init()
    node = StaticFramePublisher(sys.argv)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
    #/model/simple_car/pose

