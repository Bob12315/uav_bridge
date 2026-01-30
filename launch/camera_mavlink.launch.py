#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_image",
                default_value="true",
                description="Launch Gazebo image bridge.",
            ),
            DeclareLaunchArgument(
                "gz_image_topic",
                default_value="/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image",
                description="Gazebo image topic (gz.msgs.Image).",
            ),
            DeclareLaunchArgument(
                "ros_image_topic",
                default_value="/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image",
                description="ROS2 image topic (sensor_msgs/Image).",
            ),
            DeclareLaunchArgument(
                "mavlink_url_rx",
                default_value="udpin:0.0.0.0:14540",
                description="MAVLink RX URL (listener).",
            ),
            DeclareLaunchArgument(
                "mavlink_url_tx",
                default_value="udp:127.0.0.1:14551",
                description="MAVLink TX URL (target).",
            ),
            DeclareLaunchArgument(
                "enable_rqt",
                default_value="true",
                description="Launch rqt_image_view.",
            ),
            DeclareLaunchArgument(
                "rqt_image_topic",
                default_value="/uav/camera/image",
                description="Image topic for rqt_image_view.",
            ),
            DeclareLaunchArgument(
                "enable_rx",
                default_value="true",
                description="Launch MAVLink RX telemetry node.",
            ),
            DeclareLaunchArgument(
                "enable_tx",
                default_value="true",
                description="Launch MAVLink TX command node.",
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
            DeclareLaunchArgument(
                "reconnect_timeout_s",
                default_value="5.0",
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
                ],
                output="screen",
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
