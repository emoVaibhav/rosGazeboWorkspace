import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'information',
            self.listenerCallback,
            10,
        )
        self.subscription

    def listenerCallback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    mySubscriber = MySubscriber()

    try:
        rclpy.spin(mySubscriber)
    except KeyboardInterrupt:
        pass

    mySubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
