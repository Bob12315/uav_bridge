#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 -> MAVLink command sender for ArduPilot (TX only).

Topics (recommended defaults):
  /uav/cmd/arm         std_msgs/Bool                 True=arm, False=disarm
  /uav/cmd/mode        std_msgs/String               ArduPilot mode (e.g., GUIDED)
  /uav/cmd/takeoff     std_msgs/Float32              takeoff altitude (meters)
  /uav/cmd/land        std_msgs/Empty                land now
  /uav/cmd/velocity    geometry_msgs/TwistStamped    ENU m/s + yaw_rate in angular.z
  /uav/cmd/waypoint    sensor_msgs/NavSatFix         lat/lon/alt (alt in meters)
  /uav/cmd/attitude    geometry_msgs/QuaternionStamped  attitude setpoint
  /uav/cmd/thrust      std_msgs/Float32              0.0-1.0
  /uav/cmd/gimbal_target geometry_msgs/Vector3       pitch/roll/yaw in degrees (x=pitch,y=roll,z=yaw)

Notes:
  - This node only sends MAVLink commands; it does not read or publish state.
  - Velocity input assumes ENU; converted to NED for MAVLink.
  - Attitude quaternion is sent as-is; provide NED-frame quaternion if required.
  - Error flag is logged and also published to /uav/tx_error.
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32, Empty, UInt16MultiArray
from geometry_msgs.msg import TwistStamped, QuaternionStamped, PoseStamped, Vector3
from sensor_msgs.msg import NavSatFix

from pymavlink import mavutil


def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def rotate_vector_by_quat(qx: float, qy: float, qz: float, qw: float,
                          vx: float, vy: float, vz: float):
    """Rotate vector by quaternion (x,y,z,w)."""
    # Normalize quaternion to avoid drift.
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return vx, vy, vz
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    # Quaternion-vector multiplication: v' = q * v * q_conj
    # Convert v to pure quaternion (vx, vy, vz, 0).
    ix = qw * vx - qy * vz + qz * vy
    iy = qw * vy - qz * vx + qx * vz
    iz = qw * vz - qx * vy + qy * vx
    iw = qx * vx + qy * vy + qz * vz

    rx = ix * qw + iw * qx + iy * qz - iz * qy
    ry = iy * qw + iw * qy + iz * qx - ix * qz
    rz = iz * qw + iw * qz + ix * qy - iy * qx
    return rx, ry, rz


class MavlinkTxNode(Node):
    def __init__(self):
        super().__init__("mavlink_tx")

        self.declare_parameter("mavlink_url", "udp:127.0.0.1:14540")
        self.declare_parameter("waypoint_relative_alt", True)
        self.declare_parameter("default_takeoff_alt", 5.0)
        self.declare_parameter("default_thrust", 0.5)
        self.declare_parameter("gimbal_mount_mode", 2)

        mavlink_url = (
            self.get_parameter("mavlink_url")
            .get_parameter_value()
            .string_value
        )
        self._waypoint_relative_alt = (
            self.get_parameter("waypoint_relative_alt")
            .get_parameter_value()
            .bool_value
        )
        self._default_takeoff_alt = (
            self.get_parameter("default_takeoff_alt")
            .get_parameter_value()
            .double_value
        )
        self._thrust = (
            self.get_parameter("default_thrust")
            .get_parameter_value()
            .double_value
        )
        self._gimbal_mount_mode = int(
            self.get_parameter("gimbal_mount_mode")
            .get_parameter_value()
            .integer_value
        )

        self._error_flag = False
        self._error_pub = self.create_publisher(Bool, "uav/tx_error", 10)

        self._pose_ready = False
        self._pose_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._pose_quat = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}

        self.get_logger().info(f"Connecting to MAVLink at {mavlink_url}")
        try:
            self.master = mavutil.mavlink_connection(mavlink_url)
            self.master.wait_heartbeat(timeout=10)
            self.get_logger().info(
                f"Heartbeat from system (system {self.master.target_system}, "
                f"component {self.master.target_component})"
            )
            self._mode_mapping = self.master.mode_mapping() or {}
        except Exception as exc:
            self.master = None
            self._mode_mapping = {}
            self._set_error(True, f"MAVLink connect failed: {exc}")

        # command subscriptions
        self.create_subscription(Bool, "uav/cmd/arm", self.on_arm, 10)
        self.create_subscription(String, "uav/cmd/mode", self.on_mode, 10)
        self.create_subscription(Float32, "uav/cmd/takeoff", self.on_takeoff, 10)
        self.create_subscription(Empty, "uav/cmd/land", self.on_land, 10)
        self.create_subscription(TwistStamped, "uav/cmd/velocity", self.on_velocity, 10)
        self.create_subscription(NavSatFix, "uav/cmd/waypoint", self.on_waypoint, 10)
        self.create_subscription(QuaternionStamped, "uav/cmd/attitude", self.on_attitude, 10)
        self.create_subscription(Float32, "uav/cmd/thrust", self.on_thrust, 10)
        self.create_subscription(UInt16MultiArray, "uav/cmd/rc_override", self.on_rc_override, 10)
        self.create_subscription(Vector3, "uav/cmd/move_relative", self.on_move_relative, 10)
        self.create_subscription(Vector3, "uav/cmd/gimbal_target", self.on_gimbal_target, 10)
        self.create_subscription(PoseStamped, "uav/pose", self.on_pose, 10)

        # TODO: placeholders for future command topics (keep for later extension).
        # self.create_subscription(..., "uav/cmd/mission", self.on_mission, 10)
        # self.create_subscription(..., "uav/cmd/rc_override", self.on_rc_override, 10)

    def _now_ms(self) -> int:
        return int((self.get_clock().now().nanoseconds / 1e6) % 4294967295)

    def _set_error(self, flag: bool, msg: str):
        if flag and not self._error_flag:
            self.get_logger().error(f"TX_ERROR_FLAG=1 {msg}")
        elif not flag and self._error_flag:
            self.get_logger().info("TX_ERROR_FLAG=0")
        self._error_flag = flag
        out = Bool()
        out.data = flag
        self._error_pub.publish(out)

    def _ensure_master(self) -> bool:
        if self.master is None:
            self._set_error(True, "MAVLink master not connected")
            return False
        return True

    def _send_command_long(self, command: int, params):
        if not self._ensure_master():
            return
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                command,
                0,
                params[0], params[1], params[2], params[3],
                params[4], params[5], params[6],
            )
        except Exception as exc:
            self._set_error(True, f"COMMAND_LONG failed: {exc}")

    def on_arm(self, msg: Bool):
        arm = 1.0 if msg.data else 0.0
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [arm, 0, 0, 0, 0, 0, 0],
        )

    def on_mode(self, msg: String):
        if not self._ensure_master():
            return
        mode = msg.data.strip().upper()
        if not mode:
            return
        if mode not in self._mode_mapping:
            self._set_error(True, f"Unknown mode: {mode}")
            return
        try:
            self.master.set_mode(self._mode_mapping[mode])
        except Exception as exc:
            self._set_error(True, f"SET_MODE failed: {exc}")

    def on_takeoff(self, msg: Float32):
        alt = msg.data if msg.data > 0.1 else self._default_takeoff_alt
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [0, 0, 0, 0, 0, 0, float(alt)],
        )

    def on_land(self, _msg: Empty):
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            [0, 0, 0, 0, 0, 0, 0],
        )

    def on_velocity(self, msg: TwistStamped):
        if not self._ensure_master():
            return
        # ENU -> NED
        vx = msg.twist.linear.y
        vy = msg.twist.linear.x
        vz = -msg.twist.linear.z
        yaw_rate = msg.twist.angular.z

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        try:
            self.master.mav.set_position_target_local_ned_send(
                self._now_ms(),
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, 0.0,
                float(vx), float(vy), float(vz),
                0.0, 0.0, 0.0,
                0.0, float(yaw_rate),
            )
        except Exception as exc:
            self._set_error(True, f"SET_POSITION_TARGET_LOCAL_NED failed: {exc}")

    def on_waypoint(self, msg: NavSatFix):
        if not self._ensure_master():
            return
        lat = int(msg.latitude * 1e7)
        lon = int(msg.longitude * 1e7)
        alt = float(msg.altitude)

        frame = (
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            if self._waypoint_relative_alt
            else mavutil.mavlink.MAV_FRAME_GLOBAL_INT
        )
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        try:
            self.master.mav.set_position_target_global_int_send(
                self._now_ms(),
                self.master.target_system,
                self.master.target_component,
                frame,
                type_mask,
                lat,
                lon,
                alt,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0,
            )
        except Exception as exc:
            self._set_error(True, f"SET_POSITION_TARGET_GLOBAL_INT failed: {exc}")

    def on_attitude(self, msg: QuaternionStamped):
        if not self._ensure_master():
            return
        q = msg.quaternion
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        if math.isclose(qw, 0.0) and math.isclose(qx, 0.0) and math.isclose(qy, 0.0) and math.isclose(qz, 0.0):
            return
        thrust = float(clamp(self._thrust, 0.0, 1.0))

        type_mask = (
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
        )
        try:
            self.master.mav.set_attitude_target_send(
                self._now_ms(),
                self.master.target_system,
                self.master.target_component,
                type_mask,
                [qw, qx, qy, qz],
                0.0, 0.0, 0.0,
                thrust,
            )
        except Exception as exc:
            self._set_error(True, f"SET_ATTITUDE_TARGET failed: {exc}")

    def on_thrust(self, msg: Float32):
        self._thrust = clamp(float(msg.data), 0.0, 1.0)

    def on_rc_override(self, msg: UInt16MultiArray):
        if not self._ensure_master():
            return
        if not msg.data:
            return

        channels = [0xFFFF] * 8
        for i in range(min(8, len(msg.data))):
            val = int(msg.data[i])
            if val in (0, 0xFFFF):
                channels[i] = 0xFFFF
                continue
            if val < 1000 or val > 2000:
                self.get_logger().warn(
                    f"RC channel {i+1} out of range ({val}), clamped to 1000-2000"
                )
                val = max(1000, min(2000, val))
            channels[i] = val

        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                channels[0],
                channels[1],
                channels[2],
                channels[3],
                channels[4],
                channels[5],
                channels[6],
                channels[7],
            )
        except Exception as exc:
            self._set_error(True, f"RC_CHANNELS_OVERRIDE failed: {exc}")

    def on_gimbal_target(self, msg: Vector3):
        if not self._ensure_master():
            return
        pitch = float(msg.x)
        roll = float(msg.y)
        yaw = float(msg.z)
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                0,
                pitch,
                roll,
                yaw,
                0.0,
                0.0,
                0.0,
                float(self._gimbal_mount_mode),
            )
        except Exception as exc:
            self._set_error(True, f"DO_MOUNT_CONTROL failed: {exc}")

    def on_pose(self, msg: PoseStamped):
        self._pose_pos["x"] = msg.pose.position.x
        self._pose_pos["y"] = msg.pose.position.y
        self._pose_pos["z"] = msg.pose.position.z
        self._pose_quat["x"] = msg.pose.orientation.x
        self._pose_quat["y"] = msg.pose.orientation.y
        self._pose_quat["z"] = msg.pose.orientation.z
        self._pose_quat["w"] = msg.pose.orientation.w
        self._pose_ready = True

    def on_move_relative(self, msg: Vector3):
        if not self._ensure_master():
            return
        if not self._pose_ready:
            self._set_error(True, "No pose received; cannot move relative")
            return

        # Body-frame offset (meters): x forward, y left, z up.
        dx_body = float(msg.x)
        dy_body = float(msg.y)
        dz_body = float(msg.z)

        # Rotate body offset into ENU world frame using current orientation.
        qx = self._pose_quat["x"]
        qy = self._pose_quat["y"]
        qz = self._pose_quat["z"]
        qw = self._pose_quat["w"]
        dx_enu, dy_enu, dz_enu = rotate_vector_by_quat(
            qx, qy, qz, qw, dx_body, dy_body, dz_body
        )

        # Target position in ENU.
        target_e = self._pose_pos["x"] + dx_enu
        target_n = self._pose_pos["y"] + dy_enu
        target_u = self._pose_pos["z"] + dz_enu

        # ENU -> NED for MAVLink LOCAL_NED frame.
        target_ned_x = target_n
        target_ned_y = target_e
        target_ned_z = -target_u

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        try:
            self.master.mav.set_position_target_local_ned_send(
                self._now_ms(),
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                float(target_ned_x), float(target_ned_y), float(target_ned_z),
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0,
            )
        except Exception as exc:
            self._set_error(True, f"SET_POSITION_TARGET_LOCAL_NED (pos) failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkTxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
