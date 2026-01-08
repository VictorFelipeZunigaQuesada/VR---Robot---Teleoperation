#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 1) Incluir el launch oficial del xarm_controller
    xarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('xarm_controller'),
            '/launch/lite6_control_rviz_display.launch.py'
        ]),
        launch_arguments={
            'robot_ip': '200.126.19.206',
            'add_gripper': 'true'
        }.items()
    )

    # 2) Nodo del WebSocket del gripper
    gripper_ws_node = Node(
        package='gripper_control',
        executable='lite6_gripper_ws',
        output='screen'
    )

    return LaunchDescription([
        xarm_launch,
        gripper_ws_node
    ])
