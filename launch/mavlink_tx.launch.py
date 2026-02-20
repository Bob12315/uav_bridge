#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def is_foxy():
    return (os.getenv("ROS_DISTRO") or "").lower() == "foxy"


def defaults_map():
    return {
        "mavlink_url": "udp:127.0.0.1:14551",
        "waypoint_relative_alt": "true",
        "default_takeoff_alt": "5.0",
        "default_thrust": "0.5",
    }


def generate_launch_description():
    defaults = defaults_map()

    mavlink_url = LaunchConfiguration("mavlink_url")
    waypoint_relative_alt = LaunchConfiguration("waypoint_relative_alt")
    default_takeoff_alt = LaunchConfiguration("default_takeoff_alt")
    default_thrust = LaunchConfiguration("default_thrust")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mavlink_url",
                default_value=defaults["mavlink_url"],
                description="MAVLink connection URL.",
            ),
            DeclareLaunchArgument(
                "waypoint_relative_alt",
                default_value=defaults["waypoint_relative_alt"],
                description="Use relative altitude for waypoint commands.",
            ),
            DeclareLaunchArgument(
                "default_takeoff_alt",
                default_value=defaults["default_takeoff_alt"],
                description="Default takeoff altitude (meters).",
            ),
            DeclareLaunchArgument(
                "default_thrust",
                default_value=defaults["default_thrust"],
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
