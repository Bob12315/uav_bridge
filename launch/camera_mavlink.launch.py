#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        "digicam_command": "203",
        "digicam_param5_trigger": "1.0",
        "screenshot_enable_save": "true",
        "screenshot_image_topic": "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image",
        "screenshot_output_dir": "~/uav_captures",
        "screenshot_filename_format": "shot_%04d.jpg",
        "screenshot_log_level": "info",
        "reconnect_timeout_s": "5.0",
        "enable_image": "true",
        "enable_rx": "true",
        "enable_tx": "true",
        "enable_rqt": "true",
        "gz_image_topic": "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image",
        "ros_image_topic": "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image",
        "rqt_image_topic": "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image",
        "mavlink_url_rx": "udpin:0.0.0.0:14540",
        "mavlink_url_tx": "udp:127.0.0.1:14551",
    }


def generate_launch_description():
    defaults = defaults_map()

    enable_image = LaunchConfiguration("enable_image")
    gz_image_topic = LaunchConfiguration("gz_image_topic")
    ros_image_topic = LaunchConfiguration("ros_image_topic")
    mavlink_url_rx = LaunchConfiguration("mavlink_url_rx")
    mavlink_url_tx = LaunchConfiguration("mavlink_url_tx")
    enable_rqt = LaunchConfiguration("enable_rqt")
    rqt_image_topic = LaunchConfiguration("rqt_image_topic")
    enable_rx = LaunchConfiguration("enable_rx")
    enable_tx = LaunchConfiguration("enable_tx")
    waypoint_relative_alt = LaunchConfiguration("waypoint_relative_alt")
    default_takeoff_alt = LaunchConfiguration("default_takeoff_alt")
    default_thrust = LaunchConfiguration("default_thrust")
    reconnect_timeout_s = LaunchConfiguration("reconnect_timeout_s")
    digicam_command = LaunchConfiguration("digicam_command")
    digicam_param5_trigger = LaunchConfiguration("digicam_param5_trigger")
    screenshot_enable_save = LaunchConfiguration("screenshot_enable_save")
    screenshot_image_topic = LaunchConfiguration("screenshot_image_topic")
    screenshot_output_dir = LaunchConfiguration("screenshot_output_dir")
    screenshot_filename_format = LaunchConfiguration("screenshot_filename_format")
    screenshot_log_level = LaunchConfiguration("screenshot_log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_image",
                default_value=defaults["enable_image"],
                description="Launch Gazebo image bridge.",
            ),
            DeclareLaunchArgument(
                "gz_image_topic",
                default_value=defaults["gz_image_topic"],
                description="Gazebo image topic (gz.msgs.Image).",
            ),
            DeclareLaunchArgument(
                "ros_image_topic",
                default_value=defaults["ros_image_topic"],
                description="ROS2 image topic (sensor_msgs/Image).",
            ),
            DeclareLaunchArgument(
                "mavlink_url_rx",
                default_value=defaults["mavlink_url_rx"],
                description="MAVLink RX URL (listener).",
            ),
            DeclareLaunchArgument(
                "mavlink_url_tx",
                default_value=defaults["mavlink_url_tx"],
                description="MAVLink TX URL (target).",
            ),
            DeclareLaunchArgument(
                "enable_rqt",
                default_value=defaults["enable_rqt"],
                description="Launch rqt_image_view.",
            ),
            DeclareLaunchArgument(
                "rqt_image_topic",
                default_value=defaults["rqt_image_topic"],
                description="Image topic for rqt_image_view.",
            ),
            DeclareLaunchArgument(
                "enable_rx",
                default_value=defaults["enable_rx"],
                description="Launch MAVLink RX telemetry node.",
            ),
            DeclareLaunchArgument(
                "enable_tx",
                default_value=defaults["enable_tx"],
                description="Launch MAVLink TX command node.",
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
            DeclareLaunchArgument(
                "digicam_command",
                default_value=defaults["digicam_command"],
                description="MAV_CMD_DO_DIGICAM_CONTROL command id (default 203).",
            ),
            DeclareLaunchArgument(
                "digicam_param5_trigger",
                default_value=defaults["digicam_param5_trigger"],
                description="DIGICAM_CONTROL param5 shutter trigger (non-zero triggers).",
            ),
            DeclareLaunchArgument(
                "screenshot_enable_save",
                default_value=defaults["screenshot_enable_save"],
                description="Save latest image frame to disk on screenshot trigger.",
            ),
            DeclareLaunchArgument(
                "screenshot_image_topic",
                default_value=defaults["screenshot_image_topic"],
                description="Image topic to sample when saving screenshots.",
            ),
            DeclareLaunchArgument(
                "screenshot_output_dir",
                default_value=defaults["screenshot_output_dir"],
                description="Directory to store captured screenshots.",
            ),
            DeclareLaunchArgument(
                "screenshot_filename_format",
                default_value=defaults["screenshot_filename_format"],
                description="Filename format for saved screenshots (supports %d counter).",
            ),
            DeclareLaunchArgument(
                "screenshot_log_level",
                default_value=defaults["screenshot_log_level"],
                description="Log level for mavlink_tx (e.g., debug/info/warn).",
            ),
            DeclareLaunchArgument(
                "reconnect_timeout_s",
                default_value=defaults["reconnect_timeout_s"],
                description="Reconnect timeout in seconds for MAVLink RX.",
            ),
            Node(
                condition=IfCondition(enable_image),
                package="ros_gz_image",
                executable="image_bridge",
                name="ros_gz_image",
                arguments=[gz_image_topic, ros_image_topic],
                parameters=[{"qos": "sensor_data", "lazy": False}],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_rx),
                package="uav_bridge",
                executable="mavlink_bridge",
                name="mavlink_bridge",
                parameters=[
                    {"mavlink_url": mavlink_url_rx},
                    {"reconnect_timeout_s": reconnect_timeout_s},
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_tx),
                package="uav_bridge",
                executable="mavlink_tx",
                name="mavlink_tx",
                parameters=[
                    {"mavlink_url": mavlink_url_tx},
                    {"waypoint_relative_alt": waypoint_relative_alt},
                    {"default_takeoff_alt": default_takeoff_alt},
                    {"default_thrust": default_thrust},
                    {"digicam_command": digicam_command},
                    {"digicam_param5_trigger": digicam_param5_trigger},
                    {"screenshot_enable_save": screenshot_enable_save},
                    {"screenshot_image_topic": screenshot_image_topic},
                    {"screenshot_output_dir": screenshot_output_dir},
                    {"screenshot_filename_format": screenshot_filename_format},
                ],
                output="screen",
                arguments=([] if is_foxy() else ["--ros-args", "--log-level", screenshot_log_level]),
            ),
            Node(
                condition=IfCondition(enable_rqt),
                package="rqt_image_view",
                executable="rqt_image_view",
                name="rqt_image_view",
                arguments=[rqt_image_topic],
                output="screen",
            ),
        ]
    )
