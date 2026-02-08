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
  /uav/cmd/move_relative geometry_msgs/Vector3       body move (x=fwd,y=left,z=up, meters)
  /uav/cmd/move_relative_yaw geometry_msgs/Twist     body move + relative yaw (angular.z, rad)
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
from geometry_msgs.msg import TwistStamped, QuaternionStamped, Vector3, Twist
from sensor_msgs.msg import NavSatFix

from pymavlink import mavutil


def clamp(val: float, lo: float, hi: float) -> float:
    """限幅工具：把 val 约束到 [lo, hi]。"""
    return max(lo, min(hi, val))


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

        # 对外暴露的发送异常标志，便于上层节点做降级处理。
        self._error_flag = False
        self._error_pub = self.create_publisher(Bool, "uav/tx_error", 10)

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
        self.create_subscription(Twist, "uav/cmd/move_relative_yaw", self.on_move_relative_yaw, 10)
        self.create_subscription(Vector3, "uav/cmd/gimbal_target", self.on_gimbal_target, 10)

        # TODO: placeholders for future command topics (keep for later extension).
        # self.create_subscription(..., "uav/cmd/mission", self.on_mission, 10)
        # self.create_subscription(..., "uav/cmd/rc_override", self.on_rc_override, 10)

    def _now_ms(self) -> int:
        # MAVLink time_boot_ms 是 uint32，这里取毫秒并做 32bit 环绕。
        return int((self.get_clock().now().nanoseconds / 1e6) % 4294967295)

    def _set_error(self, flag: bool, msg: str):
        # 仅在状态翻转时打印日志，避免重复刷屏。
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
        # COMMAND_LONG 固定 7 个浮点参数。
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
        # ROS 常见 ENU 速度输入 -> MAVLink LOCAL_NED。
        vx = msg.twist.linear.y
        vy = msg.twist.linear.x
        vz = -msg.twist.linear.z
        yaw_rate = msg.twist.angular.z

        # 仅启用速度 + yaw_rate 控制，屏蔽位置/加速度/yaw 角。
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
        # GLOBAL_INT 系列消息使用 1e7 缩放的整型经纬度。
        lat = int(msg.latitude * 1e7)
        lon = int(msg.longitude * 1e7)
        alt = float(msg.altitude)

        frame = (
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            if self._waypoint_relative_alt
            else mavutil.mavlink.MAV_FRAME_GLOBAL_INT
        )
        # 仅给位置目标，忽略速度/加速度/yaw/yaw_rate。
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
            # 全零四元数无效，直接忽略。
            return
        thrust = float(clamp(self._thrust, 0.0, 1.0))

        # 仅按姿态四元数控制，忽略角速度通道。
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
        # 推力作为姿态控制的附加通道，范围固定在 [0, 1]。
        self._thrust = clamp(float(msg.data), 0.0, 1.0)

    def on_rc_override(self, msg: UInt16MultiArray):
        if not self._ensure_master():
            return
        if not msg.data:
            return

        # ArduPilot/PX4 经典 RC_OVERRIDE 为 8 通道；0xFFFF 表示“忽略该通道”。
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
        # 约定输入单位为度：x=pitch, y=roll, z=yaw。
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

    def _send_move_relative_body(self, dx_body: float, dy_body: float, dz_body: float):
        if not self._ensure_master():
            return

        # 直接使用 BODY_OFFSET_NED：
        # 在“发送命令那一刻”的机体朝向下解释位移（机头为前）。
        # BODY_NED 轴: x前, y右, z下；因此 y/z 需要取反。
        target_body_x = dx_body
        target_body_y = -dy_body
        target_body_z = -dz_body

        # 仅发送位置目标。
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
        frame = getattr(
            mavutil.mavlink,
            "MAV_FRAME_BODY_OFFSET_NED",
            mavutil.mavlink.MAV_FRAME_BODY_NED,
        )
        try:
            self.master.mav.set_position_target_local_ned_send(
                self._now_ms(),
                self.master.target_system,
                self.master.target_component,
                frame,
                type_mask,
                float(target_body_x), float(target_body_y), float(target_body_z),
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0,
            )
        except Exception as exc:
            self._set_error(True, f"SET_POSITION_TARGET_LOCAL_NED (pos) failed: {exc}")

    def _send_relative_yaw_deg(self, yaw_deg: float):
        # 相对偏航：正值顺时针，负值逆时针（ArduPilot 约定）。
        if not self._ensure_master():
            return
        if abs(yaw_deg) < 1e-3:
            return
        direction = 1.0 if yaw_deg >= 0.0 else -1.0
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            [abs(yaw_deg), 0.0, direction, 1.0, 0.0, 0.0, 0.0],
        )

    def on_move_relative(self, msg: Vector3):
        # 兼容旧接口：仅相对位移，不带偏航。
        self._send_move_relative_body(float(msg.x), float(msg.y), float(msg.z))

    def on_move_relative_yaw(self, msg: Twist):
        # 新接口：linear 为机体系位移（m），angular.z 为相对偏航（rad）。
        self._send_move_relative_body(
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.linear.z),
        )
        self._send_relative_yaw_deg(math.degrees(float(msg.angular.z)))


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
