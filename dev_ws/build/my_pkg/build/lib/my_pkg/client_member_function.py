import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MyClientAsync(Node):
    def __init__(self):
        super().__init__('my_client_async')
        self.cli = self.create_client(
            AddTwoInts,
            'add_two_ints',
        )
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        
        self.req = AddTwoInts.Request()

    def sendRequest(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    myClient = MyClientAsync()
    myClient.sendRequest()

    while rclpy.ok():
        rclpy.spin_once(myClient)

        if myClient.future.done():
            try:
                response = myClient.future.result()
            except Exception as e:
                myClient.get_logger().error(
                    'Service call failed %f' % (e,),
                ) 
            else:
                myClient.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (myClient.req.a, myClient.req.b, response.sum)
                )
            break

    myClient.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
