#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 <-> MAVLink bridge (只读：HEARTBEAT / GLOBAL_POSITION_INT / ATTITUDE / BATTERY)
+ ENU 坐标转换 & Odometry 输出

Topics:
  /uav/mode            std_msgs/String
  /uav/armed           std_msgs/Bool
  /uav/system_status   std_msgs/UInt8
  /uav/navsatfix       sensor_msgs/NavSatFix
  /uav/pose            geometry_msgs/PoseStamped          (position=ENU meters, orientation=ATTITUDE)
  /uav/odom            nav_msgs/Odometry                  (position=ENU meters, orientation=ATTITUDE)
  /uav/battery         sensor_msgs/BatteryState

Notes:
  - ENU 原点在第一次收到有效 GLOBAL_POSITION_INT 时锁定（lat0,lon0,alt0）。
  - 速度来自飞控 EKF 输出：GLOBAL_POSITION_INT.vx/vy/vz（NED, cm/s）转换为 ENU（m/s）。
  - 角速度来自 ATTITUDE.rollspeed/pitchspeed/yawspeed（rad/s），按机体系写入 odom.twist.angular。
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, UInt8
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from pymavlink import mavutil


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """欧拉角 (rad) -> 四元数 (x, y, z, w)"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # ZYX (yaw-pitch-roll) to quaternion
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def decode_copter_mode(custom_mode: int) -> str:
    """针对 ArduCopter 的简单模式映射"""
    mode_map = {
        0: "STABILIZE",
        1: "ACRO",
        2: "ALT_HOLD",
        3: "AUTO",
        4: "GUIDED",
        5: "LOITER",
        6: "RTL",
        7: "CIRCLE",
        9: "LAND",
        11: "DRIFT",
        13: "SPORT",
        14: "FLIP",
        15: "AUTOTUNE",
        16: "POSHOLD",
        17: "BRAKE",
        18: "THROW",
        19: "AVOID_ADSB",
        20: "GUIDED_NOGPS",
        21: "SMART_RTL",
    }
    return mode_map.get(custom_mode, f"MODE_{custom_mode}")


# ---------------------------- ENU conversion ---------------------------- #
# WGS84 constants
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


def llh_to_ecef(lat_deg: float, lon_deg: float, h_m: float):
    """LLH (deg, deg, m) -> ECEF (m, m, m)"""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1.0 - _WGS84_E2) + h_m) * sin_lat
    return x, y, z


def ecef_to_enu(x: float, y: float, z: float,
                lat0_deg: float, lon0_deg: float,
                x0: float, y0: float, z0: float):
    """ECEF -> ENU (meters) relative to ref (lat0,lon0,x0,y0,z0)"""
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)

    dx = x - x0
    dy = y - y0
    dz = z - z0

    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    e = -sin_lon0 * dx + cos_lon0 * dy
    n = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    u =  cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return e, n, u
# ---------------------------------------------------------------------- #

def is_valid_gps_fix(lat_deg: float, lon_deg: float) -> bool:
    # Reject zero/NaN and out-of-range values to avoid locking ENU at invalid fix
    if not math.isfinite(lat_deg) or not math.isfinite(lon_deg):
        return False
    if abs(lat_deg) < 1e-9 and abs(lon_deg) < 1e-9:
        return False
    if abs(lat_deg) > 90.0 or abs(lon_deg) > 180.0:
        return False
    return True


class MavlinkBridgeNode(Node):
    def __init__(self):
        super().__init__("mavlink_bridge")

        self.declare_parameter("mavlink_url", "udp:127.0.0.1:14540")
        mavlink_url = (
            self.get_parameter("mavlink_url")
            .get_parameter_value()
            .string_value
        )

        self.get_logger().info(f"Connecting to MAVLink at {mavlink_url}")
        self.master = mavutil.mavlink_connection(mavlink_url)
        self.master.wait_heartbeat(timeout=10)
        self.get_logger().info(
            f"Heartbeat from system (system {self.master.target_system}, "
            f"component {self.master.target_component})"
        )

        # publishers
        self.mode_pub = self.create_publisher(String, "uav/mode", 10)
        self.armed_pub = self.create_publisher(Bool, "uav/armed", 10)
        self.sys_status_pub = self.create_publisher(UInt8, "uav/system_status", 10)

        self.navsat_pub = self.create_publisher(NavSatFix, "uav/navsatfix", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "uav/pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "uav/odom", 10)

        self.batt_pub = self.create_publisher(BatteryState, "uav/battery", 10)

        # last pose (ENU meters)
        self._last_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._last_quat = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}

        # last twist
        # linear: ENU (m/s)
        self._last_lin_vel = {"x": 0.0, "y": 0.0, "z": 0.0}
        # angular: body rates (rad/s) about base_link axes
        self._last_ang_vel = {"x": 0.0, "y": 0.0, "z": 0.0}

        # ENU reference (set once on first GPS)
        self._enu_ref_set = False
        self._ref_lat = 0.0
        self._ref_lon = 0.0
        self._ref_alt = 0.0
        self._ref_x0 = 0.0
        self._ref_y0 = 0.0
        self._ref_z0 = 0.0

        self.timer = self.create_timer(0.02, self.read_mavlink)  # 50 Hz

    def read_mavlink(self):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                self.handle_heartbeat(msg)
            elif msg_type == "GLOBAL_POSITION_INT":
                self.handle_global_position_int(msg)
            elif msg_type == "ATTITUDE":
                self.handle_attitude(msg)
            elif msg_type == "BATTERY_STATUS":
                self.handle_battery_status(msg)
            elif msg_type == "SYS_STATUS":
                self.handle_sys_status(msg)

    def handle_heartbeat(self, msg):
        base_mode = msg.base_mode
        custom_mode = msg.custom_mode
        system_status = msg.system_status

        armed_flag = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode_str = decode_copter_mode(custom_mode)

        mode_msg = String()
        mode_msg.data = mode_str
        self.mode_pub.publish(mode_msg)

        armed_msg = Bool()
        armed_msg.data = armed_flag
        self.armed_pub.publish(armed_msg)

        status_msg = UInt8()
        status_msg.data = system_status
        self.sys_status_pub.publish(status_msg)

    def handle_global_position_int(self, msg):
        # MAVLink units:
        #   lat/lon: 1e-7 deg
        #   alt: mm (AMSL)
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt_amsl = msg.alt / 1000.0  # m

        # 1) NavSatFix
        navsat = NavSatFix()
        navsat.header.stamp = self.get_clock().now().to_msg()
        navsat.header.frame_id = "map"
        navsat.latitude = lat
        navsat.longitude = lon
        navsat.altitude = alt_amsl
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.navsat_pub.publish(navsat)

        # 2) Set ENU reference once (skip invalid GPS fix)
        if not self._enu_ref_set and is_valid_gps_fix(lat, lon):
            self._ref_lat = lat
            self._ref_lon = lon
            self._ref_alt = alt_amsl
            self._ref_x0, self._ref_y0, self._ref_z0 = llh_to_ecef(lat, lon, alt_amsl)
            self._enu_ref_set = True
            self.get_logger().info(
                f"ENU reference set: lat0={lat:.7f}, lon0={lon:.7f}, alt0={alt_amsl:.2f}m"
            )
        elif not self._enu_ref_set:
            self.get_logger().warn(
                f"Invalid GPS fix, skip ENU ref set: lat={lat:.7f}, lon={lon:.7f}"
            )
            return

        # 3) Compute ENU meters and update pose position
        x, y, z = llh_to_ecef(lat, lon, alt_amsl)
        e, n, u = ecef_to_enu(
            x, y, z,
            self._ref_lat, self._ref_lon,
            self._ref_x0, self._ref_y0, self._ref_z0
        )

        self._last_pos["x"] = e
        self._last_pos["y"] = n
        self._last_pos["z"] = u

        # EKF velocity (NED, cm/s) -> ENU (m/s)
        # MAVLink: vx=N (cm/s), vy=E (cm/s), vz=D (cm/s)
        # ROS ENU: x=E, y=N, z=U
        vn = msg.vx / 100.0
        ve = msg.vy / 100.0
        vd = msg.vz / 100.0
        self._last_lin_vel["x"] = ve
        self._last_lin_vel["y"] = vn
        self._last_lin_vel["z"] = -vd

        self.publish_pose()
        self.publish_odom()

    def handle_attitude(self, msg):
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        self._last_quat["x"] = qx
        self._last_quat["y"] = qy
        self._last_quat["z"] = qz
        self._last_quat["w"] = qw

        # Body angular rates (rad/s)
        self._last_ang_vel["x"] = msg.rollspeed
        self._last_ang_vel["y"] = msg.pitchspeed
        self._last_ang_vel["z"] = msg.yawspeed

        self.publish_pose()
        self.publish_odom()

    def handle_battery_status(self, msg):
        batt = BatteryState()
        batt.header.stamp = self.get_clock().now().to_msg()

        voltage_v = 0.0
        if msg.voltages and msg.voltages[0] != 0xFFFF:
            voltage_v = msg.voltages[0] / 1000.0

        current_a = float("nan")
        if msg.current_battery != -1:
            current_a = msg.current_battery / 100.0

        if msg.battery_remaining != -1:
            percentage = msg.battery_remaining / 100.0
        else:
            percentage = float("nan")

        batt.voltage = voltage_v
        batt.current = current_a
        batt.percentage = percentage

        self.batt_pub.publish(batt)

    def handle_sys_status(self, msg):
        batt = BatteryState()
        batt.header.stamp = self.get_clock().now().to_msg()

        if msg.voltage_battery != 0xFFFF:
            batt.voltage = msg.voltage_battery / 1000.0
        else:
            batt.voltage = 0.0

        if msg.current_battery != -1:
            batt.current = msg.current_battery / 100.0
        else:
            batt.current = float("nan")

        if msg.battery_remaining != -1:
            batt.percentage = msg.battery_remaining / 100.0
        else:
            batt.percentage = float("nan")

        self.batt_pub.publish(batt)

    def publish_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = self._last_pos["x"]
        pose.pose.position.y = self._last_pos["y"]
        pose.pose.position.z = self._last_pos["z"]

        pose.pose.orientation.x = self._last_quat["x"]
        pose.pose.orientation.y = self._last_quat["y"]
        pose.pose.orientation.z = self._last_quat["z"]
        pose.pose.orientation.w = self._last_quat["w"]

        self.pose_pub.publish(pose)

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self._last_pos["x"]
        odom.pose.pose.position.y = self._last_pos["y"]
        odom.pose.pose.position.z = self._last_pos["z"]

        odom.pose.pose.orientation.x = self._last_quat["x"]
        odom.pose.pose.orientation.y = self._last_quat["y"]
        odom.pose.pose.orientation.z = self._last_quat["z"]
        odom.pose.pose.orientation.w = self._last_quat["w"]

        # Twist (velocity)
        odom.twist.twist.linear.x = self._last_lin_vel["x"]
        odom.twist.twist.linear.y = self._last_lin_vel["y"]
        odom.twist.twist.linear.z = self._last_lin_vel["z"]

        odom.twist.twist.angular.x = self._last_ang_vel["x"]
        odom.twist.twist.angular.y = self._last_ang_vel["y"]
        odom.twist.twist.angular.z = self._last_ang_vel["z"]

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
