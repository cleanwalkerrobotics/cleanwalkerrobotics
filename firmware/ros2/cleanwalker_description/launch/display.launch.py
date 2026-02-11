#!/usr/bin/env python3
"""
CleanWalker CW-1 — RViz Visualization Launch File

Loads the URDF and opens RViz for interactive robot visualization.
Useful for verifying the URDF, testing joint states, and debugging TF.

Usage:
  ros2 launch cleanwalker_description display.launch.py
  ros2 launch cleanwalker_description display.launch.py gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('cleanwalker_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'cleanwalker_cw1.urdf')

    # Launch arguments
    gui = LaunchConfiguration('gui')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch joint_state_publisher_gui for interactive control')

    # Read URDF
    robot_description = ''
    if os.path.exists(urdf_file):
        with open(urdf_file, 'r') as f:
            robot_description = f.read()

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Joint state publisher GUI (for interactive joint control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui),
    )

    # Joint state publisher (non-GUI fallback)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'display.rviz')]
        if os.path.exists(os.path.join(pkg_dir, 'config', 'display.rviz'))
        else [],
    )

    return LaunchDescription([
        declare_gui,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz_node,
    ])
