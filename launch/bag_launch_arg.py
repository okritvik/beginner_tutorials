from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    record_flag = LaunchConfiguration('record_flag')

    return LaunchDescription([

        DeclareLaunchArgument(
            'frequency',
            default_value='1.0'
        ),
        
        DeclareLaunchArgument(
            'record_flag',
            default_value='False'
        ),

        Node(
            package='ros2_cpp_pubsub',
            executable='param_talker',
            parameters=[{
                "frequency": LaunchConfiguration('frequency'),
            }]
        ),

        Node(
            package='ros2_cpp_pubsub',
            executable='param_listener',
            arguments=['--ros-args', '--log-level', 'debug']
        ),

        ExecuteProcess(
        condition=IfCondition(record_flag),
        cmd=[
            'ros2', 'bag', 'record', '-o tutorial_bag', '-a'
        ],
        shell=True
        )

    ])
