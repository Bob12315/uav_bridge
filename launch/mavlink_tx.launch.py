#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavlink_url = LaunchConfiguration("mavlink_url")
    waypoint_relative_alt = LaunchConfiguration("waypoint_relative_alt")
    default_takeoff_alt = LaunchConfiguration("default_takeoff_alt")
    default_thrust = LaunchConfiguration("default_thrust")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mavlink_url",
                default_value="udp:127.0.0.1:14551",
                description="MAVLink connection URL.",
            ),
            DeclareLaunchArgument(
                "waypoint_relative_alt",
                default_value="true",
                description="Use relative altitude for waypoint commands.",
            ),
            DeclareLaunchArgument(
                "default_takeoff_alt",
                default_value="5.0",
                description="Default takeoff altitude (meters).",
            ),
            DeclareLaunchArgument(
                "default_thrust",
                default_value="0.5",
                description="Default thrust for attitude control (0.0-1.0).",
            ),
            Node(
                package="uav_bridge",
                executable="mavlink_tx",
                name="mavlink_tx",
                parameters=[
                    {"mavlink_url": mavlink_url},
                    {"waypoint_relative_alt": waypoint_relative_alt},
                    {"default_takeoff_alt": default_takeoff_alt},
                    {"default_thrust": default_thrust},
                ],
                output="screen",
            ),
        ]
    )
