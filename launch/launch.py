#! /usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')
    
    node = Node(
            package='prizm_ros2',
            executable='prizm_node',
            namespace="/",
            name='prizm_node',
            parameters=[
                {'serial_port': LaunchConfiguration("serial_port")}
            ]
        )
    
    
    return LaunchDescription([
        serial_port_arg,
        node,
        LogInfo(msg=LaunchConfiguration('serial_port'))
    ])