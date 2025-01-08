import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')

        self.declare_parameter('model', 'simple_car')
        fromTopic = '/model/' + self.get_parameter('model').get_parameter_value().string_value + '/pose'

        self.br = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            TransformStamped,
            fromTopic,
            self.listenerCallback,
            10,
        )
        self.subscription
        self.get_logger().info('Listening on: "%s"' % fromTopic)

    def listenerCallback(self, msg):
        #  broadcast transform to TF2
        self.br.sendTransform(msg)

def main(args=None):
    rclpy.init(args=args)

    mySubscriber = PoseSubscriber()

    try:
        rclpy.spin(mySubscriber)
    except KeyboardInterrupt:
        pass

    mySubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
