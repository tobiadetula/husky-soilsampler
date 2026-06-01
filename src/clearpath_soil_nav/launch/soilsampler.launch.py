from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription(
        [
            # Node from SDI12 Sensor over USB
            Node(
                package="sdi12_sensor",
                executable="sdi12_node",
                name="sdi12_node",
                output="screen",
                parameters=[
                    {"sensor_addresses": ["0"]}  # You can pass your parameters here!
                ],
            ),
            # Node from Motor Joystick Control
            Node(
                package="ros_joystick_motor_controller",
                executable="joystick_motor_node",
                name="joystick_motor_node",
                output="screen",
            ),
            # micro-ROS agent node with its own RMW and variables
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent_serial",
                output="screen",
                arguments=[
                    "serial",
                    "--dev",
                    "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6612483CB5D9E2B-if00",
                    "--verbosity",
                    "6",
                ],
                additional_env={
                    "RMW_IMPLEMENTATION": "rmw_microxrcedds",
                    "ROS_DOMAIN_ID": "0",
                },
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("clearpath_soil_nav"),
                        "launch",
                        "navigation.launch.py",
                    )
                )
            ),
            # Foxglove bridge
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {
                        "port": 8765,
                        "topic_whitelist": [".*"],
                        "topic_blacklist": [
                            # clearpath_platform_msgs — not installed
                            ".*/platform/mcu/status$",
                            ".*/platform/mcu/status/stop",
                            ".*/platform/mcu/status/power",
                            ".*/platform/mcu/status/temperature",
                            ".*/platform/mcu/status/pinout",
                            ".*/platform/cmd_lights",
                            ".*/platform/cmd_fans",
                            ".*/platform/display/status",
                            ".*/platform/bms/status",
                            # clearpath_motor_msgs — not installed
                            ".*/platform/motors/status",
                            ".*/platform/motors/feedback",
                            ".*/platform/motors/system_protection",
                            # can_msgs — not installed
                            ".*/vcan0/rx",
                            ".*/vcan0/tx",
                            # pal_statistics_msgs — not installed
                            ".*/controller_manager/statistics/.*",
                            ".*/controller_manager/introspection_data/.*",
                            # controller_manager_msgs — not installed
                            ".*/controller_manager/activity",
                            # canopen_inventus_interfaces — not installed
                            ".*/platform/bms/battery_0/status",
                            ".*/platform/bms/battery_1/status",
                            # wireless_msgs — not installed
                            ".*/platform/wifi_status",
                            # control_msgs — not installed
                            ".*/platform/dynamic_joint_states",
                            # High-bandwidth / not needed on Pi side
                            ".*/sensor_0/info",
                            ".*/lidar3d_0/points",
                ],
            }],
        ),
    ])