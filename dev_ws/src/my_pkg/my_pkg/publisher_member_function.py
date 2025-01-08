import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import String

class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')

        paramDescriptor = ParameterDescriptor(description='Your name')
        # parameters must be declared before they can be set or get
        self.declare_parameter('name', 'Faaizz', paramDescriptor)

        paramDescriptor = ParameterDescriptor(description='Your age')
        self.declare_parameter('age', '26', paramDescriptor)

        paramDescriptor = ParameterDescriptor(description='Your profession')
        self.declare_parameter('profession', 'Software Developer', paramDescriptor)

        paramTimerPeriod = ParameterDescriptor(description='Timer Period')
        self.declare_parameter('timer_period', 5, paramTimerPeriod)
        myTimerPeriod = self.get_parameter('timer_period').get_parameter_value().integer_value


        self.publisher_ = self.create_publisher(String, 'information', 10)
        # timerPeriod = 2
        self.timer = self.create_timer(myTimerPeriod, self.timerCallback)
        self.i = 0

    def timerCallback(self):
        myName = self.get_parameter('name').get_parameter_value().string_value
        myAge = self.get_parameter('age').get_parameter_value().string_value
        myJob = self.get_parameter('profession').get_parameter_value().string_value

        msg = String()
        msg.data = 'I am %s, I am %s years old, I am a %s. Index: %d' % (myName, myAge, myJob, self.i)
        self.publisher_.publish(msg)
        self.get_logger().info('publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    myPublisher = MyPublisher()

    try:
        rclpy.spin(myPublisher)
    except KeyboardInterrupt:
        pass

    # Explucitly destroy node
    myPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
