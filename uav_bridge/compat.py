#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compatibility helpers for multi-distro (Foxy/Humble/Jazzy).

Centralizes per-ROS distribution differences so node/launch code stays clean.
"""

import os
from dataclasses import dataclass
from typing import Dict

from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSProfile


@dataclass(frozen=True)
class Compat:
    name: str
    qos_profile_sensor: QoSProfile
    qos_profile_cmd: QoSProfile
    image_sub_depth: int
    node_parameters_defaults: Dict[str, object]
    launch_defaults: Dict[str, str]
    enable_log_level_args: bool
    pil_supported: bool


def _make_qos(depth: int, *, reliable: bool) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=(
            QoSReliabilityPolicy.RELIABLE
            if reliable
            else QoSReliabilityPolicy.BEST_EFFORT
        ),
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _detect_distro() -> str:
    return (os.environ.get("ROS_DISTRO") or "").lower()


def _check_pil() -> bool:
    try:
        import PIL  # noqa: F401
    except Exception:
        return False
    return True


def get_compat() -> Compat:
    distro = _detect_distro()
    pil_ok = _check_pil()

    # Common defaults for node parameters
    node_defaults = {
        "mavlink_url": "udp:127.0.0.1:14540",
        "waypoint_relative_alt": True,
        "default_takeoff_alt": 5.0,
        "default_thrust": 0.5,
        "gimbal_mount_mode": 2,
        "digicam_command": 203,
        "digicam_param5_trigger": 1.0,
        "screenshot_enable_save": True,
        "screenshot_image_topic": "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image",
        "screenshot_output_dir": "~/uav_captures",
        "screenshot_filename_format": "shot_%04d.jpg",
    }

    launch_defaults = {
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
        "rqt_image_topic": "/uav/camera/image",
        "mavlink_url_rx": "udpin:0.0.0.0:14540",
        "mavlink_url_tx": "udp:127.0.0.1:14551",
    }

    if distro == "foxy":
        return Compat(
            name="foxy",
            qos_profile_sensor=_make_qos(5, reliable=False),
            qos_profile_cmd=_make_qos(10, reliable=True),
            image_sub_depth=5,
            node_parameters_defaults=node_defaults,
            launch_defaults=launch_defaults,
            enable_log_level_args=False,
            pil_supported=pil_ok,
        )
    if distro == "jazzy":
        return Compat(
            name="jazzy",
            qos_profile_sensor=_make_qos(10, reliable=False),
            qos_profile_cmd=_make_qos(10, reliable=True),
            image_sub_depth=10,
            node_parameters_defaults=node_defaults,
            launch_defaults=launch_defaults,
            enable_log_level_args=True,
            pil_supported=pil_ok,
        )
    # Default to Humble settings for unknown/empty distro
    return Compat(
        name="humble" if distro == "humble" else distro or "unknown",
        qos_profile_sensor=_make_qos(10, reliable=False),
        qos_profile_cmd=_make_qos(10, reliable=True),
        image_sub_depth=10,
        node_parameters_defaults=node_defaults,
        launch_defaults=launch_defaults,
        enable_log_level_args=True,
        pil_supported=pil_ok,
    )


def is_foxy() -> bool:
    return get_compat().name == "foxy"
