import os
from os import environ
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')])}

    # Run the node
    return LaunchDescription([
        DeclareLaunchArgument(
            'ign_args',
            default_value = '',
            description = 'Arguments to pass to Ignition Gazebo'
        ),
        ExecuteProcess(
            cmd = [
                'ign gazebo -v 3 -r ',
                LaunchConfiguration('ign_args'),
                PathJoinSubstitution([
                    FindPackageShare('my_pkg'),
                    'description', 'worlds', 'car_world.sdf',
                ]),
            ],
            output = 'screen',
            additional_env = env,
            shell = True,
        ),
        Node(
            package = 'ros_ign_bridge',
            executable = 'parameter_bridge',
            arguments = [
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/model/simple_car/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/model/simple_car/pose@geometry_msgs/msg/TransformStamped[ignition.msgs.Pose',
                '/model/capsule/pose@geometry_msgs/msg/TransformStamped[ignition.msgs.Pose',
            ],
        ),
        Node(
            package = 'my_pkg',
            executable = 'pose_subscriber',
            parameters = [
                {
                    'model': 'simple_car',
                }
            ],
        ),
        Node(
            package = 'my_pkg',
            executable = 'pose_subscriber',
            parameters = [
                {
                    'model': 'capsule',
                }
            ],
        ),
        Node(
            package = 'my_pkg',
            executable = 'tf2_dist_calculator',
            parameters = [
                {
                    'from_frame': 'simple_car/camera_link/camera',
                    'to_frame': 'capsule/capsule_link',
                    'x_offset': 0.1,
                    'y_offset': 0.0,
                }
            ],
        ),
        Node(
            package = 'my_pkg',
            executable = 'img_saver',
            parameters = [
                {
                    'img_topic': '/camera',
                }
            ],
        ),
    ])
