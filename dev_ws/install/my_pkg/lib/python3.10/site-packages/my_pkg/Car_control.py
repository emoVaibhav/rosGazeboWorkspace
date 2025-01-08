import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
import math
import time


class SendVel(Node):
    def __init__(self):
        super().__init__('subscribe_pose')
        self.subscribe_pose = self.create_subscription(
            TransformStamped,
            '/model/simple_car/pose',
            self.send_vel_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        
        # Target position
        self.target_x = 1.0
        self.target_y = 1.0
        
        self.get_logger().info('Position incoming')

    def send_vel_callback(self, msg: TransformStamped):
        # Extract position and orientation
        if msg.child_frame_id == 'simple_car/chassis' :
            x = msg.transform.translation.x
            y = msg.transform.translation.y
            qw = msg.transform.rotation.w
            qx = msg.transform.rotation.x
            qy = msg.transform.rotation.y
            qz = msg.transform.rotation.z

        # Compute target yaw and distance to target
            target_yaw = math.atan2(self.target_y , self.target_x )
            distance = math.sqrt((self.target_x - x) ** 2 + (self.target_y - y) ** 2)

            # Convert quaternion to yaw
            yaw = 0.79

            # Log current status
            self.get_logger().info(f"Position: ({x:.2f}, {y:.2f}), Distance: {distance:.2f}, Yaw: {yaw:.2f}, Target Yaw: {target_yaw:.2f}")

            # Control logic
            velocity = 0.0
            angular_velocity = 0.0

            if distance >= 0.15:  # Move towards the target
                yaw_diff = target_yaw - yaw

                # Normalize yaw difference to [-pi, pi]
                yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))

                if abs(yaw_diff) > 0.5:  # Rotate to align with the target
                    angular_velocity = 0.5 if yaw_diff > 0 else -0.5
                else:  # Move forward if aligned
                    velocity = 0.8
            else:
                self.get_logger().info("Target reached.")
            
            # Publish velocities
            self.twist.linear.x = velocity
            self.twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.3)


def main(args=None):
    rclpy.init(args=args)
    controller = SendVel()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
