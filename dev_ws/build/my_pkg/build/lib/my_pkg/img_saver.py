import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, HistoryPolicy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image


class ImageSub(Node):

    def __init__(self):
        super().__init__('image_saver')

        self.declare_parameter(
            'img_topic', '/camera'
        )

        imgTopic = self.get_parameter('img_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            imgTopic,
            self.listener_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ))
        self.subscription  # prevent unused variable warning
        self.image = None
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('got image...')
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            cv2.imwrite('camera_image.jpeg', self.image)


def main(args=None):
    rclpy.init(args=args)

    image_sub = ImageSub()

    rclpy.spin(image_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
