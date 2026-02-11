#!/usr/bin/env python3
"""
CleanWalker CW-1 — Main Robot Launch File

Brings up the full autonomy stack:
  1. Robot description (URDF -> robot_state_publisher)
  2. Perception pipeline (detection, depth, segmentation)
  3. SLAM + localization (cuVSLAM + RTAB-Map + EKF)
  4. Navigation (Nav2 + coverage planner)
  5. Manipulation (grasp planner + arm controller)

Usage:
  ros2 launch cleanwalker_bringup robot.launch.py
  ros2 launch cleanwalker_bringup robot.launch.py use_sim:=true
  ros2 launch cleanwalker_bringup robot.launch.py enable_segmentation:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    bringup_dir = get_package_share_directory('cleanwalker_bringup')
    description_dir = get_package_share_directory('cleanwalker_description')

    # Launch arguments
    use_sim = LaunchConfiguration('use_sim')
    enable_segmentation = LaunchConfiguration('enable_segmentation')
    robot_params_file = LaunchConfiguration('robot_params_file')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Use simulation (Gazebo) instead of real hardware')

    declare_enable_seg = DeclareLaunchArgument(
        'enable_segmentation', default_value='true',
        description='Enable terrain segmentation node (disable to save GPU)')

    declare_params_file = DeclareLaunchArgument(
        'robot_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'robot_params.yaml'),
        description='Path to robot parameters YAML file')

    # ── Robot Description ──
    urdf_file = os.path.join(description_dir, 'urdf', 'cleanwalker_cw1.urdf')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read() if os.path.exists(urdf_file) else ''}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # ── Perception Nodes ──
    detection_node = Node(
        package='cleanwalker_perception',
        executable='detection_node',
        name='detection_node',
        output='screen',
        parameters=[robot_params_file],
        remappings=[
            ('/oak/rgb/image', '/oak/rgb/image'),
            ('/oak/stereo/depth', '/oak/stereo/depth'),
        ],
    )

    depth_node = Node(
        package='cleanwalker_perception',
        executable='depth_node',
        name='depth_node',
        output='screen',
        parameters=[robot_params_file],
    )

    segmentation_node = Node(
        package='cleanwalker_perception',
        executable='segmentation_node',
        name='segmentation_node',
        output='screen',
        parameters=[robot_params_file],
        condition=IfCondition(enable_segmentation),
    )

    # ── SLAM + Localization ──
    slam_node = Node(
        package='cleanwalker_navigation',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[robot_params_file],
    )

    # TODO: Launch RTAB-Map as a separate node
    # rtabmap_node = Node(
    #     package='rtabmap_ros',
    #     executable='rtabmap',
    #     name='rtabmap',
    #     parameters=[{
    #         'subscribe_depth': True,
    #         'subscribe_scan_cloud': True,
    #         'frame_id': 'base_link',
    #         'odom_frame_id': 'odom',
    #     }],
    # )

    # TODO: Launch robot_localization EKF
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     parameters=[robot_params_file],
    # )

    # ── Navigation ──
    path_planner_node = Node(
        package='cleanwalker_navigation',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[robot_params_file],
    )

    # TODO: Launch Nav2 stack
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('nav2_bringup'),
    #                      'launch', 'navigation_launch.py')),
    #     launch_arguments={'params_file': robot_params_file}.items(),
    # )

    # ── Manipulation ──
    grasp_planner_node = Node(
        package='cleanwalker_manipulation',
        executable='grasp_planner_node',
        name='grasp_planner_node',
        output='screen',
        parameters=[robot_params_file],
    )

    arm_controller_node = Node(
        package='cleanwalker_manipulation',
        executable='arm_controller_node',
        name='arm_controller_node',
        output='screen',
        parameters=[robot_params_file],
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim,
        declare_enable_seg,
        declare_params_file,

        # Robot description
        robot_state_publisher,
        joint_state_publisher,

        # Perception
        detection_node,
        depth_node,
        segmentation_node,

        # SLAM + Localization
        slam_node,

        # Navigation
        path_planner_node,

        # Manipulation
        grasp_planner_node,
        arm_controller_node,
    ])
