

#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from tf_transformations import euler_from_quaternion
import tf_transformations
from my_pkg import posn_msg

def quaternion_to_euler(quaternion):
    """
    Converts a quaternion to Euler angles (roll, pitch, yaw).
    Args:
        quaternion: A geometry_msgs/Quaternion object or [x, y, z, w] list.
    Returns:
        tuple: Roll, pitch, and yaw angles in radians.
    """
    x, y, z, w = quaternion
    roll, pitch, yaw = tf_transformations.euler_from_quaternion([x, y, z, w])
    return roll, pitch, yaw

    # #Example: Extracting position and orientation
    # pose = msg.pose[index]  # Replace 'index' with the model index
    # position = pose.position
    # orientation = pose.orientation
    # x = position.x
    # y= position.y
    # z = position.z
    # rpy = quaternion_to_euler([orientation.x, orientation.y, orientation.z, orientation.w])
    # roll, pitch, yaw = rpy
    # print(f"Position: x={x}, y={y}, z={z}")
    # print(f"Orientation: roll={roll}, pitch={pitch}, yaw={yaw}")

class RobotLocationSubscriber(Node):
    def __init__(self):
        super().__init__('robot_location_subscriber')
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'calc_posn', 10)

        self.target_robot_name = 'simple_car'                                        # Replace with your robot's name!!!!!!!!!
        self.get_logger().info('Robot Location Subscriber Node has been started.')

    def listener_callback(self, msg):

        for i, name in enumerate(msg.name):
            
            position = msg.pose[i].position
            x, y, z = position.x, position.y, position.z
            orientation = msg.pose[i].orientation
            qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
            r, p, y = euler_from_quaternion([qx, qy, qz, qw])

            loc=posn_msg()
            loc.x =x
            loc.y =y
            loc.z =z
            loc.r =r
            loc.p =p
            loc.y =y

            # Extract linear and angular velocity
            linear = msg.twist[i].linear
            angular = msg.twist[i].angular
            vx, vy, vz = linear.x, linear.y, linear.z
            wx, wy, wz = angular.x, angular.y, angular.z
            # Print all values for the current model
            self.get_logger().info(f"Model: {name}")
            self.get_logger().info(f"  Position: x={x}, y={y}, z={z}")
            self.get_logger().info(f"  Orientation (Quaternion): x={qx}, y={qy}, z={qz}, w={qw}")
            self.get_logger().info(f"  Orientation (RPY): roll={r}, pitch={p}, yaw={y}")
            # self.get_logger().info(f"  Linear Velocity: x={vx}, y={vy}, z={vz}")
            # self.get_logger().info(f"  Angular Velocity: x={wx}, y={wy}, z={wz}")
            self.get_logger().info("\n") 
            self.publisher.publish(msg)


        # Check if the target robot is in the list of model names
        if self.target_robot_name in msg.name:
            # Get the index of the robot
            index = msg.name.index(self.target_robot_name)
            # Extract position and orientation
            position = msg.pose[index].position
            orientation = msg.pose[index].orientation
            self.get_logger().info(
                f"Robot Position: x={position.x}, y={position.y}, z={position.z}"
            )
            self.get_logger().info(
                f"Robot Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}"
            )
        else:
            self.get_logger().warn(f"Robot '{self.target_robot_name}' not found in Gazebo.")

def main(args=None):
    rclpy.init(args=args)
    node = RobotLocationSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

