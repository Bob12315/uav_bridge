# uav_bridge

ROS 2 + MAVLink 工具集（通用），提供 MAVLink 遥测接入与控制发送节点。

## 组件
- `mavlink_bridge`：MAVLink 遥测 → ROS 2 话题（pose/odom/battery 等）
- `mavlink_tx`：ROS 2 指令 → MAVLink 控制（只发送）
- `mavlink_dump`：打印原始 MAVLink 消息，便于调试
- `camera_mavlink.launch.py`：可选的相机桥 + MAVLink RX/TX 组合启动

## 依赖
- ROS 2（Humble 或兼容版本）
- pymavlink
- （可选）ros_gz_image、rqt_image_view

### 推荐安装（Ubuntu 22.04）
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-gzharmonic \
  ros-humble-rqt \
  ros-humble-rqt-image-view \
  ros-humble-topic-tools

pip install --upgrade pip
pip install pymavlink

python3 -c "import pymavlink; print(pymavlink.__version__)"

```

## 编译
```bash
cd $HOME/ros2_ws
colcon build --packages-select uav_bridge
source install/setup.bash
```

```
--out=udp:127.0.0.1:14540   # 给 ROS RX
--out=udp:127.0.0.1:14550   # 给 QGC
--out=udp:127.0.0.1:14551   # 给 ROS TX
```

## 运行示例
```bash
gz sim -v4 -r iris_runway.sdf
```
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --add-param-file=$HOME/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm --out=udp:127.0.0.1:14540 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 
```

## 相机桥（Gazebo → ROS 2，单独启动）
```bash
ros2 run ros_gz_image image_bridge \
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image \
  /uav/camera/image \
  --ros-args -p qos:=sensor_data -p lazy:=false
```

## MAVLink 遥测（RX）
```bash
ros2 run uav_bridge mavlink_bridge --ros-args -p mavlink_url:=udp:127.0.0.1:14540
```

### mavlink_bridge 详细说明

用途：订阅 MAVLink 遥测消息并发布为 ROS 2 话题（只读），目前解析 HEARTBEAT / GLOBAL_POSITION_INT / ATTITUDE / BATTERY_STATUS / SYS_STATUS。

参数：
- `mavlink_url`：MAVLink 连接地址（默认 `udp:127.0.0.1:14540`）。

发布话题：
- `/uav/mode` (std_msgs/String)
- `/uav/armed` (std_msgs/Bool)
- `/uav/system_status` (std_msgs/UInt8)
- `/uav/navsatfix` (sensor_msgs/NavSatFix)
- `/uav/pose` (geometry_msgs/PoseStamped)
- `/uav/odom` (nav_msgs/Odometry)
- `/uav/battery` (sensor_msgs/BatteryState)

坐标/数据约定：
- ENU 原点：第一次收到有效 `GLOBAL_POSITION_INT` 时锁定 (lat0, lon0, alt0)。
- 位置：`/uav/pose` 与 `/uav/odom` 的 position 为 ENU (meters)。
- 速度：来自 `GLOBAL_POSITION_INT.vx/vy/vz`（NED, cm/s），转换为 ENU (m/s) 写入 `odom.twist.twist.linear`。
- 姿态：使用 `ATTITUDE` 的 roll/pitch/yaw 转为四元数发布到 `/uav/pose` 和 `/uav/odom`。
- 角速度：使用 `ATTITUDE.rollspeed/pitchspeed/yawspeed` 写入 `odom.twist.twist.angular`。

电池：
- 优先解析 `BATTERY_STATUS`，若无则解析 `SYS_STATUS`，并统一发布到 `/uav/battery`。

注意：
- 若 MAVLink 无心跳或连接失败，节点会在启动阶段报错并退出。
- 若 GPS 未锁定，ENU 原点可能在非预期位置，必要时请改为使用 `GPS_RAW_INT` 或加入 fix_type 判断。

## MAVLink 控制（TX，仅发送）
```bash
ros2 run uav_bridge mavlink_tx --ros-args -p mavlink_url:=udp:127.0.0.1:14551
```

指令话题（可按需改名）：
- `/uav/cmd/arm` (std_msgs/Bool)
- `/uav/cmd/mode` (std_msgs/String)
- `/uav/cmd/takeoff` (std_msgs/Float32)
- `/uav/cmd/land` (std_msgs/Empty)
- `/uav/cmd/velocity` (geometry_msgs/TwistStamped)  # ENU m/s, yaw_rate=angular.z
- `/uav/cmd/move_relative` (geometry_msgs/Vector3)   # 机体坐标相对位移，单位 m（x前, y左, z上）
- `/uav/cmd/waypoint` (sensor_msgs/NavSatFix)        # lat/lon/alt
- `/uav/cmd/attitude` (geometry_msgs/QuaternionStamped)
- `/uav/cmd/thrust` (std_msgs/Float32)               # 0.0-1.0
- `/uav/cmd/rc_override` (std_msgs/UInt16MultiArray) # 8ch PWM, 1000-2000
- `/uav/cmd/gimbal_target` (geometry_msgs/Vector3)   # pitch/roll/yaw (degrees)
- `/uav/tx_error` (std_msgs/Bool)                    # 错误标志

简单测试示例：
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

# 航点（lat, lon, alt）
ros2 topic pub --once /uav/cmd/waypoint sensor_msgs/NavSatFix \
  "{latitude: 47.397742, longitude: 8.545594, altitude: 10.0}"

# RC override（8通道 PWM，1000-2000；0/65535=不覆盖）
ros2 topic pub --once /uav/cmd/rc_override std_msgs/UInt16MultiArray \
  "{data: [1500,1500,1000,1500,1500,1500,1500,1500]}"

# 云台目标（pitch/roll/yaw，单位度；对应 MAV_CMD_DO_MOUNT_CONTROL）
ros2 topic pub --once /uav/cmd/gimbal_target geometry_msgs/Vector3 \
  "{x: -10.0, y: 0.0, z: 0.0}"

# 降落
ros2 topic pub --once /uav/cmd/land std_msgs/Empty "{}"
```

## 组合启动（相机 + RX/TX）
```bash
ros2 launch uav_bridge camera_mavlink.launch.py
```

可选参数：
```bash
ros2 launch uav_bridge camera_mavlink.launch.py \
  mavlink_url_rx:=udpin:0.0.0.0:14540 \
  mavlink_url_tx:=udp:127.0.0.1:14551 \
  enable_image:=true \
  enable_rx:=true \
  enable_tx:=true \
  enable_rqt:=false
```

## MAVLink 原始数据打印
```bash
ros2 run uav_bridge mavlink_dump
ros2 run uav_bridge mavlink_dump -- --duration 5 --types HEARTBEAT,ATTITUDE
```

## 备注
- RC override 需要持续发送才能保持（单次发送会在几秒后失效）。
- 坐标系约定：速度指令使用 ENU；MAVLink 下发时转换为 NED。
