from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class MyService(Node):
    def __init__(self):
        super().__init__('my_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.addTwoIntsCallback,
        )
    
    def addTwoIntsCallback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Request: \na: %d b: %d \nResponse: \nsum: %d' % (request.a, request.b, response.sum))

        return response

def main(args=None):
    rclpy.init(args=args)

    myService = MyService()

    try:
        rclpy.spin(myService)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
