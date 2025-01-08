from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            LogInfo, RegisterEventHandler)
from launch.event_handlers import (OnExecutionComplete,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import (LaunchConfiguration, LocalSubstitution)

def generate_launch_description():
    nodeNsConfig = LaunchConfiguration('node_ns')
    timerPeriodConfig = LaunchConfiguration('timer_period')
    nameConfig = LaunchConfiguration('name_config')

    nodeNsArg = DeclareLaunchArgument(
        'node_ns',
        default_value='talker_listener_evnt_hdlr',
    )
    timerPeriodArg = DeclareLaunchArgument(
        'timer_period',
        default_value='5',
    )
    nameArg = DeclareLaunchArgument(
        'name_config',
        default_value='Faizudeen',
    )

    talkerNode = Node(
        package = 'my_pkg',
        namespace = nodeNsConfig,
        executable = 'talker',
        name = 'sim',
        parameters = [
            { 'timer_period': timerPeriodConfig }
        ]
    )

    listenerNode = Node(
        package = 'my_pkg',
        namespace = nodeNsConfig,
        executable = 'listener',
        name = 'sim',
    )

    changeName = ExecuteProcess(
        cmd = [[
            'ros2 param set ',
            nodeNsConfig,
            '/sim name ',
            nameConfig,
        ]],
        shell = True
    )

    return LaunchDescription([
        nodeNsArg,
        timerPeriodArg,
        nameArg,
        talkerNode,
        listenerNode,
        RegisterEventHandler(
            # OnProcessStart is used to register a callback function that is executed when a process is started
            OnProcessStart(
                target_action = talkerNode,
                on_start = [
                    LogInfo(msg='Talker Started, changing name...'),
                    changeName
                ]
            )
        ),
        RegisterEventHandler(
            # OnProcessIO is used to register a callback that is executed when the action writes to stdout
            OnProcessIO(
                target_action = changeName,
                on_stdout = lambda event: LogInfo(
                    msg = 'changeName says: "{}"'.format(
                        event.text.decode().strip()
                    )
                )
            )
        ),
        RegisterEventHandler(
            # Used to register a callback that is executed when an action completes
            OnExecutionComplete(
                target_action = changeName,
                on_completion = [
                    LogInfo(msg='name changed successfully'),
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown = [
                    LogInfo(
                        msg = [
                            'Launch was asked to shutdown: ',
                            LocalSubstitution('event.reason')
                        ]
                    )
                ]
            )
        )
    ])
