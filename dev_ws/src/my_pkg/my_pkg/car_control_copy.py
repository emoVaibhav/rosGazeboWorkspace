import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.twist = Twist()
        
        self.max_velocity = 1.0 

    def move(self, velocity_x=0.0, angular_z=0.0):
        self.twist.linear.x = velocity_x
        self.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(self.twist)

    def run(self):
       
        try:
            print("Control the car by entering x,and angular velocity (-1 to 1):")
            print("Format: <x_velocity> <angular_velocity>, e.g., 0.5 0.2")
            print("Enter 'q' to quit.")

            while rclpy.ok():
                user_input = input("Enter velocities: ").strip().lower()

                if user_input == 'q':
                    print("Exiting...")
                    break

                try:
                    velocities = list(map(float, user_input.split()))
                    if len(velocities) != 2:
                        print("Invalid input. Provide exactly two values: <x_velocity> <angular_velocity>.")
                        continue

                    velocity_x, angular_z = velocities

                    if not (-1.0 <= velocity_x <= 1.0 and -1.0 <= angular_z <= 1.0):
                        print("All velocities must be in the range -1 to 1.")
                        continue

                    self.move(velocity_x,angular_z)

                except ValueError:
                    print("Invalid input. Ensure all values are numbers.")
        except KeyboardInterrupt:
            pass
        finally:
            self.move()

def main(args=None):
    rclpy.init(args=args)
    controller = CarController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
