from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # LaunchConfiguration substitutions allow us to acquire values of launch argument 
    #   in any part of the launch description
    nodeNsConfig = LaunchConfiguration('node_ns')
    nameConfig = LaunchConfiguration('name_config')
    isDevOpsConfig = LaunchConfiguration('is_dev_ops_config')


    # DeclareLaunchArgument is used to define launch arguments that can be passed from the console
    #   or from above launch files
    nodeNsArg = DeclareLaunchArgument(
        'node_ns',
        default_value='talker_listener',
    )
    nameArg = DeclareLaunchArgument(
        'name_config',
        default_value='Faizudeen',
    )
    isDevOpsArg = DeclareLaunchArgument(
        'is_dev_ops_config',
        default_value='False',
    )

    # ExecuteProcess can be used to run a command
    changeName = ExecuteProcess(
        cmd = [[
            'ros2 param set ',
            nodeNsConfig,
            '/sim name ',
            nameConfig,
        ]],
        shell = True
    )
    # Conditioned ExecuteProcess commands can be used to run a command 
    #   if some condition is fulfilled
    changeProfessionConditioned = ExecuteProcess(
        condition = IfCondition(
            PythonExpression([
                isDevOpsConfig,
                ' == ',
                'True'
            ]),
        ),
        cmd = [[
            'ros2 param set ',
            nodeNsConfig,
            '/sim profession ',
            '"DevOps Engineer"'
        ]],
        shell = True,
    )

    talkerNode = Node(
        package = 'my_pkg',
        namespace = 'talker_listener',
        executable = 'talker',
        name = 'sim',
    )

    return LaunchDescription([
        nodeNsArg,
        nameArg,
        isDevOpsArg,
        talkerNode,
        changeName,
        # Set timer to periodically execute action
        TimerAction(
            period=3.0,
            actions=[changeProfessionConditioned],
        )
    ])
