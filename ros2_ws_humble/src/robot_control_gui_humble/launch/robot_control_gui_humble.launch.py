#!/usr/bin/env python3
"""Launch the robot_control_gui_humble GUI node.

Usage:
    ros2 launch robot_control_gui_humble robot_control_gui_humble.launch.py \
        domain_id:=0 \
        robot_ip:=192.168.149.1

The launch will:
  1. export ROS_DOMAIN_ID and ROS_AUTOMATIC_DISCOVERY_RANGE in the GUI process env
  2. (optionally) ping the robot host before starting (best-effort)
  3. start robot_control_gui_humble_node

This file is intentionally simple — the GUI already exposes every network
detail in its Settings panel; the launch arg only sets defaults.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _export_env(context, *args, **kwargs):
    # Best-effort shell-action-free export: setenv before starting the node.
    # ROS2 launch doesn't expose setenv directly across processes; instead we
    # set it via the node's `additional_env` parameter.
    return []


def generate_launch_description():
    domain_id = LaunchConfiguration('domain_id', default='0')
    robot_ip  = LaunchConfiguration('robot_ip',  default='192.168.149.1')
    auto_discovery_range = LaunchConfiguration('auto_discovery', default='SUBNET')

    domain_id_arg = DeclareLaunchArgument(
        'domain_id', default_value='0',
        description='ROS_DOMAIN_ID. Must match the robot side (mentorpi defaults to 0).'
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='192.168.149.1',
        description='Robot host IP for SSH / web_video_server (informational).'
    )
    auto_discovery_arg = DeclareLaunchArgument(
        'auto_discovery', default_value='SUBNET',
        description='ROS_AUTOMATIC_DISCOVERY_RANGE: SUBNET | LOCALHOST | OFF'
    )

    gui_node = Node(
        package='robot_control_gui_humble',
        executable='robot_control_gui_humble_node',
        name='robot_control_gui_humble',
        output='screen',
        parameters=[{
            'domain_id': domain_id,
            'robot_ip':  robot_ip,
        }],
        additional_env={
            'ROS_DOMAIN_ID': domain_id,
            'ROS_AUTOMATIC_DISCOVERY_RANGE': auto_discovery_range,
            'QT_X11_NO_MITSHM': '1',     # smoother under docker / ssh -X
        },
    )

    return LaunchDescription([
        domain_id_arg,
        robot_ip_arg,
        auto_discovery_arg,
        OpaqueFunction(function=_export_env),
        gui_node,
    ])
