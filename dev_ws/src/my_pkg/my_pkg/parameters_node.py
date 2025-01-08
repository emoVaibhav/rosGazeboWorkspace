import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class MyParam(Node):
    def __init__(self):
        super().__init__('my_param_node')
        timerPeriod = 2
        self.timer = self.create_timer(timerPeriod, self.timerCallback)

        paramDescriptor = ParameterDescriptor(description='Parameter Description')
        # parameters must be declared before they can be set or get
        self.declare_parameter('my_parameter', 'world', paramDescriptor)

    def timerCallback(self):
        myParam = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % myParam)

        newParam = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world',
        )

        allNewParameters = [newParam]
        self.set_parameters(allNewParameters)

def main(args=None):
    rclpy.init(args=args)
    myParam = MyParam()

    try:
        rclpy.spin(myParam)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
