from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'my_pkg',
            executable = 'param_node',
            name = 'custom_parameter_node',
            output = 'screen',
            emulate_tty = True,
            parameters = [
                {'my_parameter' : 'Aisha'}
            ]
        )
    ])
