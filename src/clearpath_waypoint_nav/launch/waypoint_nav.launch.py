from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_namespace',
            default_value='a300_00000',
            description='Clearpath robot namespace',
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.3',
            description='Distance (m) to consider a waypoint reached',
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.5',
            description='Maximum forward speed in m/s',
        ),
        DeclareLaunchArgument(
            'action_duration',
            default_value='3.0',
            description='Seconds to pause and run action at each waypoint',
        ),

        Node(
            package='clearpath_waypoint_nav',
            executable='waypoint_commander',
            name='waypoint_commander',
            output='screen',
            parameters=[{
                'robot_namespace':  LaunchConfiguration('robot_namespace'),
                'goal_tolerance':   LaunchConfiguration('goal_tolerance'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'action_duration':  LaunchConfiguration('action_duration'),
            }],
        ),
    ])
