from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('my_pkg'),
                    'talker_listener_subs.launch.py',
                ])
            ]),
            launch_arguments={
                'node_ns': 'talker_listener',
                'name_config': '"Faizudeen Olanrewaju Kajogbola"',
                'is_dev_ops_config': 'True',
            }.items()
        ),
        Node(
            package = 'my_pkg',
            namespace = 'talker_listener',
            executable = 'listener',
            name = 'sim',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('my_pkg'),
                    'talker_listener_event_handler.launch.py',
                ])
            ]),
            launch_arguments={
                'node_ns': 'talker_listener_evnt_hdlr',
                'timer_period': '10',
                'name_config': '"Faaizz"',
            }.items()
        ),
    ])
