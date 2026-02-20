# uav_bridge

ROS 2 ↔︎ MAVLink 工具集，包含遥测接入、指令发送、相机桥接与调试打印，面向 ArduPilot/PX4/SITL。

## 兼容性矩阵
- ROS 2：Foxy (20.04) / Humble (22.04) / Jazzy (24.04)
- Python：3.8–3.12
- 可选依赖：Pillow（截图保存 JPEG/PNG），缺失时自动回退 PPM/PGM；`ros_gz_image`、`rqt_image_view` 缺失不影响核心 RX/TX。
- 启动文件自动根据 `ROS_DISTRO` 选择默认值；Foxy 不附加 `--ros-args --log-level`。

## 组件概览
- `mavlink_bridge`：MAVLink 遥测 → ROS 2 话题（姿态/位置/电池/云台）。
- `mavlink_tx`：ROS 2 指令 → MAVLink 控制（仅发送）。
- `mavlink_dump`：原始 MAVLink 消息打印，便于抓包/调试。
- `camera_mavlink.launch.py`：相机桥 + MAVLink RX/TX 一键组合启动。

## 安装与构建
```bash
sudo apt-get update
# 按需安装 ROS 发行版 (Foxy/Humble/Jazzy)
sudo apt-get install -y ros-${ROS_DISTRO}-topic-tools ros-${ROS_DISTRO}-rqt ros-${ROS_DISTRO}-rqt-image-view ros-${ROS_DISTRO}-ros-gzharmonic || true

pip install --upgrade pip
pip install pymavlink Pillow  # Pillow 可选

cd ~/ros2_ws
colcon build --packages-select uav_bridge
source install/setup.bash
```

## 快速开始
- 遥测 RX：`ros2 run uav_bridge mavlink_bridge --ros-args -p mavlink_url:=udp:127.0.0.1:14540`
- 指令 TX：`ros2 run uav_bridge mavlink_tx --ros-args -p mavlink_url:=udp:127.0.0.1:14551`
- 组合启动：`ros2 launch uav_bridge camera_mavlink.launch.py`

## 参数（精选）
`mavlink_tx`：
- `mavlink_url`：默认 `udp:127.0.0.1:14551`
- `waypoint_relative_alt`：`true` 使用相对高度
- `default_takeoff_alt`：默认 5.0 m
- `default_thrust`：默认 0.5（0–1）
- 截图：`screenshot_enable_save` / `screenshot_image_topic` / `screenshot_output_dir` / `screenshot_filename_format`

`mavlink_bridge`：
- `mavlink_url`：默认 `udp:127.0.0.1:14540`
- `reconnect_timeout_s`：自动重连超时（如需在 Launch 中设置）

## 话题
指令输入（TX）：
- `/uav/cmd/arm` (Bool) 解锁/加锁
- `/uav/cmd/mode` (String) 飞控模式
- `/uav/cmd/takeoff` (Float32) 起飞高度
- `/uav/cmd/land` (Empty) 立即降落
- `/uav/cmd/velocity` (TwistStamped) ENU 速度 + yaw_rate
- `/uav/cmd/move_relative` (Vector3) 机体系位移 (x右,y前,z上)
- `/uav/cmd/move_relative_yaw` (Twist) 机体系位移 + 相对偏航(rad)
- `/uav/cmd/waypoint` (NavSatFix) 经纬高
- `/uav/cmd/attitude` (QuaternionStamped)
- `/uav/cmd/thrust` (Float32) 0–1
- `/uav/cmd/rc_override` (UInt16MultiArray) 8 通道 PWM
- `/uav/cmd/gimbal_target` (Vector3) pitch/roll/yaw (deg)
- `/uav/cmd/screenshot` (Empty) 触发快门
- `/uav/tx_error` (Bool) 发送异常标志

遥测输出（RX）：
- `/uav/pose` (PoseStamped) / `/uav/odom` (Odometry)
- `/uav/navsatfix` (NavSatFix)
- `/uav/battery` (BatteryState)
- `/uav/mode` (String) / `/uav/armed` (Bool) / `/uav/system_status` (UInt8)
- `/uav/gimbal/angle` (Vector3)


指令示例（TX，按需修改话题名/数值）：
```bash
# 解锁
ros2 topic pub --once /uav/cmd/arm std_msgs/Bool "{data: true}"

# 切换模式（示例：GUIDED）
ros2 topic pub --once /uav/cmd/mode std_msgs/String "{data: GUIDED}"

# 起飞到 5m
ros2 topic pub --once /uav/cmd/takeoff std_msgs/Float32 "{data: 5.0}"

# 速度控制（ENU：x=E, y=N, z=U；角速度 yaw_rate=angular.z）
ros2 topic pub --rate 10 /uav/cmd/velocity geometry_msgs/TwistStamped \
  "{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.2}}}"

# 机体系相对位移 + 相对偏航（x右, y前, z上；angular.z 单位 rad，示例 +30deg）
ros2 topic pub --once /uav/cmd/move_relative_yaw geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 2.0, z: 0.0}, angular: {z: 0.5236}}"

# 航点（lat, lon, alt）
ros2 topic pub --once /uav/cmd/waypoint sensor_msgs/NavSatFix \
  "{latitude: 47.397742, longitude: 8.545594, altitude: 10.0}"

# RC override（8通道 PWM，1000-2000；0/65535=不覆盖）
ros2 topic pub --once /uav/cmd/rc_override std_msgs/UInt16MultiArray \
  "{data: [1500,1500,1000,1500,1500,1500,1500,1500]}"

# 云台目标（pitch/roll/yaw，单位度；对应 MAV_CMD_DO_MOUNT_CONTROL）
ros2 topic pub --once /uav/cmd/gimbal_target geometry_msgs/Vector3 \
  "{x: -10.0, y: 0.0, z: 0.0}"

# 截图（一次快门，MAV_CMD_DO_DIGICAM_CONTROL）
ros2 topic pub --once /uav/cmd/screenshot std_msgs/Empty "{}"

# 降落
ros2 topic pub --once /uav/cmd/land std_msgs/Empty "{}"
```

## 截图保存
- 命令 `MAV_CMD_DO_DIGICAM_CONTROL`，`digicam_param5_trigger=1.0` 触发。
- 有 Pillow：按文件扩展名保存（JPEG/PNG）。无 Pillow：自动 PPM/PGM，日志提示一次。
- 文件名支持 `%d` 计数；未提供则自动在扩展名前加 `_N`。

## 坐标与控制约定
- 输入速度为 ENU，发送前转换为 MAVLink LOCAL_NED。
- `move_relative*` 使用机体系 (x右,y前,z上)，下发 `MAV_FRAME_BODY_OFFSET_NED`；默认保持当前偏航（yaw_rate=0）。相对偏航通过 `MAV_CMD_CONDITION_YAW`，正值顺时针。
- ENU 原点在首次有效 `GLOBAL_POSITION_INT` 时锁定。

## 调试与示例
- 原始 MAVLink 打印：`ros2 run uav_bridge mavlink_dump -- --duration 5 --types HEARTBEAT,ATTITUDE`
- 典型 SITL 流程：
  ```bash
  gz sim -v4 -r iris_runway.sdf
  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
    --add-param-file=$HOME/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm \
    --out=udp:127.0.0.1:14540 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
  ```

常见提示：
- Foxy 缺少 GUI/桥接包可忽略；核心 RX/TX 不受影响。
- RC override 需持续发送保持；单次发送会在几秒后失效。

## Launch 可配置参数（常用）
`mavlink_tx.launch.py`：
- `mavlink_url` (默认 `udp:127.0.0.1:14551`)
- `waypoint_relative_alt` (`true`)
- `default_takeoff_alt` (`5.0`)
- `default_thrust` (`0.5`)

`camera_mavlink.launch.py`：
- `enable_image` (`true`)
- `gz_image_topic` (默认 Gazebo 相机话题)
- `ros_image_topic` (默认同上，可改为 `/uav/camera/image`)
- `mavlink_url_rx` (`udpin:0.0.0.0:14540`)
- `mavlink_url_tx` (`udp:127.0.0.1:14551`)
- `enable_rqt` (`true`) / `rqt_image_topic`
- `enable_rx` (`true`) / `enable_tx` (`true`)
- `waypoint_relative_alt` / `default_takeoff_alt` / `default_thrust`
- `digicam_command` (`203`) / `digicam_param5_trigger` (`1.0`)
- `screenshot_enable_save` (`true`)
- `screenshot_image_topic` (默认桥接后的 Gazebo 相机话题)
- `screenshot_output_dir` (`~/uav_captures`)
- `screenshot_filename_format` (`shot_%04d.jpg`)
- `screenshot_log_level` (`info`)
