from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            arguments = ['0', '0', '1', '0', '0', '0', 'world', 'link1'],
        ),
        Node(
            package = 'my_pkg',
            executable = 'frame_tf2_broadcaster',
            parameters = [
                {
                    'parent_frame_name': 'link1',
                    'child_frame_name': 'link2',
                }
            ]
        ),
        Node(
            package = 'my_pkg',
            executable = 'tf2_listener',
            parameters = [
                {
                    'from_frame': 'world',
                    'to_frame': 'link2',
                }
            ],
        ),
    ])
