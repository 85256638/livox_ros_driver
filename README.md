# Livox ROS Driver（钛兴科技定制版）

本分支基于官方 [livox_ros_driver v2.6.0](https://github.com/Livox-SDK/livox_ros_driver) 修改，新增以下功能：

1. **在线工作模式切换** — 运行时通过 ROS Service 切换 LiDAR 工作模式（Normal / PowerSaving / Standby）
2. **可配置点云距离过滤** — 通过 launch 参数设置最大发布距离，无需重新编译

---

## 快速开始

### 前置条件

- Ubuntu 20.04 + ROS Noetic
- **必须使用修改版 Livox SDK**（见下方"Livox SDK 修改"章节）

### 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 启动

```bash
# 单雷达
roslaunch livox_ros_driver livox_lidar.launch

# 多雷达
roslaunch livox_ros_driver livox_lidar_multi.launch
```

---

## 新增功能一：在线工作模式切换

### 使用方法

启动驱动后，在另一个终端执行：

```bash
# 切换到节电模式（电机停转，低功耗）
rosservice call /livox_lidar_mode "{handle: 0, mode: 2}"

# 切回正常模式（电机启动，正常出点）
rosservice call /livox_lidar_mode "{handle: 0, mode: 1}"

# 切换到待机模式
rosservice call /livox_lidar_mode "{handle: 0, mode: 3}"

# 所有雷达同时切换（handle 设为 255）
rosservice call /livox_lidar_mode "{handle: 255, mode: 2}"
```

### 参数说明

| 参数 | 取值 | 说明 |
|------|------|------|
| `handle` | 0~31 | 单个雷达的设备句柄（启动日志中 `Lidar[X]` 的 X 即为 handle）|
| `handle` | 255 | 广播模式，对所有已连接的雷达同时生效 |
| `mode` | 1 | Normal — 正常工作，电机旋转，输出点云 |
| `mode` | 2 | PowerSaving — 节电模式，电机停转 |
| `mode` | 3 | Standby — 待机模式，电机停转 |

### 返回值

| `ret_code` | 含义 |
|------------|------|
| 0 | 请求已接受 |
| 非 0 | 错误（详见终端日志）|

### 断线行为

| 场景 | 行为 |
|------|------|
| Normal 模式下断线 | 3 秒检测到，重连后自动恢复采样 |
| PowerSaving / Standby 下断线 | 15 秒检测到，重连后恢复 Normal 模式 |
| 切换 Normal 时通信失败 | 自动等待重连后重试 |

---

## 新增功能二：可配置点云距离过滤

发布前过滤超出指定距离的点，减少下游处理数据量。

### 使用方法

```bash
# 只发布 5 米以内的点
roslaunch livox_ros_driver livox_lidar.launch max_distance:=5.0

# 只发布 2 米以内的点
roslaunch livox_ros_driver livox_lidar.launch max_distance:=2.0

# 禁用过滤，发布所有点（默认）
roslaunch livox_ros_driver livox_lidar.launch max_distance:=0
```

也可在 launch 文件中修改默认值：

```xml
<arg name="max_distance" default="5.0"/>
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_distance` | double | 0.0 | 最大发布距离（米），0 表示禁用过滤 |

启动时终端会输出确认信息：
```
[ INFO] Distance filter enabled: max_distance = 5.00 m
```

### 三种点云格式均支持

距离过滤对以下三种输出格式都生效（`xfer_format` 参数）：
- `0` — PointCloud2 (PointXYZRTL)
- `1` — Livox CustomMsg
- `2` — PCL PointXYZI

---

## Livox SDK 修改（重要）

**本驱动需要配合修改版的 Livox SDK 使用。** 如果使用未修改的 SDK，模式切换将无法正常工作（切到节电后会立即自动恢复 Normal）。

### 修改内容

修改了 `Livox-SDK/sdk_core/src/command_handler/` 下的两个文件：

#### 1. `command_channel.h`

新增成员变量：
```cpp
uint8_t last_work_state_ = 0;  /**< Last known work state from heartbeat */
```

#### 2. `command_channel.cpp`

**修改 `OnHeartbeatAck()`** — 记录心跳 ACK 中的设备状态：
```cpp
void CommandChannel::OnHeartbeatAck(const CommPacket &packet) {
  last_heartbeat_ = steady_clock::now();
  if (packet.data != NULL && packet.data_len >= sizeof(HeartbeatResponse)) {
    last_work_state_ = reinterpret_cast<HeartbeatResponse *>(packet.data)->state;
  }
}
```

**修改 `OnTimer()`** — 心跳超时从固定 3 秒改为状态感知（Normal=3s，PowerSaving/Standby=15s）：
```cpp
auto heartbeat_timeout = std::chrono::seconds(3);
if (last_work_state_ == 2 || last_work_state_ == 3) {
  heartbeat_timeout = std::chrono::seconds(15);
}
if (now - last_heartbeat_ > heartbeat_timeout) {
  DeviceDisconnect(handle_);
} else {
  HeartBeat(now);
}
```

### 为什么需要修改 SDK

官方 SDK 的心跳超时固定为 3 秒。LiDAR 切换工作模式时（例如电机减速停转），固件响应会短暂延迟，超过 3 秒后 SDK 误判设备断线，触发重连，固件在重连时自动恢复 Normal 模式——导致模式切换失败。

延长超时至 15 秒可确保模式过渡期间会话保持存活。

### SDK 编译安装

```bash
cd ~/Livox-SDK/build
cmake ..
make -j$(nproc)
sudo make install
```

> 安装后静态库位于 `/usr/local/lib/liblivox_sdk_static.a`

---

## 修改文件清单

### Livox SDK（2 个文件）

| 文件 | 改动 |
|------|------|
| `sdk_core/src/command_handler/command_channel.h` | 新增 `last_work_state_` 字段 |
| `sdk_core/src/command_handler/command_channel.cpp` | 状态感知心跳超时 + 记录 work_state |

### ROS Driver（7 个文件）

| 文件 | 改动 |
|------|------|
| `srv/LidarMode.srv` | **新增** — ROS Service 定义 |
| `CMakeLists.txt` | 新增 `add_service_files` |
| `livox_ros_driver/lds_lidar.h` | 新增 `ModeChangeRequest` 结构体和模式切换方法声明 |
| `livox_ros_driver/lds_lidar.cpp` | 完整模式切换实现（含断连重试逻辑） |
| `livox_ros_driver/livox_ros_driver.cpp` | 新增 ROS Service、AsyncSpinner、max_distance 参数 |
| `livox_ros_driver/lddc.h` | 新增 `max_distance_` 成员和 `SetMaxDistance()` |
| `livox_ros_driver/lddc.cpp` | 三种点云格式的距离过滤逻辑 |

---

## 常见问题

### Q: 切到节电模式后立即自动恢复 Normal？
确认使用的是修改版 Livox SDK。重新编译 SDK 后需要 `sudo make install` 安装，然后重新 `catkin_make` ROS Driver。

### Q: handle 值怎么确定？
启动驱动时观察终端日志 `Lidar[X] status_code[...] working state[...] feature[...]`，其中 X 就是 handle。单雷达通常为 0。

### Q: 距离过滤设置了但 RViz 还显示远处的点？
确认 launch 文件中包含 `max_distance` 参数定义和传递，并确认修改的是被编译的源文件（不是副本）。

### Q: 多雷达场景下能否只让部分雷达进入节电？
可以。分别对不同 handle 调用 service 即可：
```bash
rosservice call /livox_lidar_mode "{handle: 0, mode: 2}"  # 0 号进入节电
rosservice call /livox_lidar_mode "{handle: 1, mode: 1}"  # 1 号保持正常
```

---

## 以下为官方原始文档

---

# Livox ROS Driver([览沃ROS驱动程序中文说明](https://github.com/Livox-SDK/livox_ros_driver/blob/master/README_CN.md))

livox_ros_driver is a new ROS package, specially used to connect LiDAR products produced by Livox. The driver can be run under ubuntu 14.04/16.04/18.04 operating system with ROS environment (indigo, kinetic, melodic) installed. Tested hardware platforms that can run livox_ros_driver include: Intel x86 cpu platforms, and some ARM64 hardware platforms (such as nvida TX2 / Xavier, etc.).

## 0. Version and Release History

### 0.1 Current Version

[v2.6.0](https://github.com/Livox-SDK/livox_ros_driver/releases)

### 0.2 Release History

[Release History](https://github.com/Livox-SDK/livox_ros_driver/releases)

## 1. Install dependencies

Before running livox_ros_driver, ROS and Livox-SDK must be installed.

### 1.1 ROS installation

For ROS installation, please refer to the ROS installation guide :

[ROS installation guide](https://www.ros.org/install/)

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;(1) Be sure to install the full version of ROS (ros-distro-desktop-full);

&ensp;&ensp;&ensp;&ensp;(2) There are 7 to 8 steps in ROS installation, please read the installation guide in detail;

### 1.2 Livox-SDK Installation

1. Download or clone [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) from Github to local;

2. Refer to the corresponding [README.md](https://github.com/Livox-SDK/Livox-SDK/blob/master/README.md) document to install and run Livox-SDK;

## 2. Get and build livox_ros_driver

1. Get livox_ros_driver from GitHub :

　　`git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src`

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;Be sure to use the above command to clone the code to the local, otherwise it will compile error due to the file path problem.

2. Use the following command to build livox_ros_driver :

   ```bash
   cd ws_livox
   catkin_make
   ```

3. Use the following command to update the current ROS package environment :

&ensp;&ensp;&ensp;&ensp;`source ./devel/setup.sh`

## 3. Run livox_ros_driver

### 3.1 Use the ROS launch file to load livox_ros_driver

&ensp;&ensp;&ensp;&ensp;The command format is as follows :

&ensp;&ensp;&ensp;&ensp;`roslaunch livox_ros_driver [launch file] [param]`

1. If the [param] parameter is empty, livox_ros_driver will connect to the corresponding device according to the configuration in the configuration file. The connection rules are as follows :

&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;When the connection status of the device specified in the configuration file is configured to enable connection (true), the livox_ros_driver will only connect to the device specified in the configuration file;

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;(1) the json configuration file is in the "ws_livox/src/livox_ros_driver/config" directory;

&ensp;&ensp;&ensp;&ensp;(2) When the connection status of the devices specified in the configuration file is all configured to prohibit connection (false), livox_ros_driver will automatically connect all the devices that are scanned;

2. If the [param] parameter is the broadcast code of LiDAR, take LiDAR (the broadcast code is 0TFDG3B006H2Z11) and LiDAR (the broadcast code is 1HDDG8M00100191) as an example, Use the  command as follows :

```bash
   roslaunch livox_ros_driver livox_lidar_rviz.launch bd_list:="0TFDG3B006H2Z11&1HDDG8M00100191"
```

&ensp;&ensp;&ensp;&ensp;***Broadcast code introduction***

&ensp;&ensp;&ensp;&ensp;Each Livox LiDAR device has a unique broadcast code. The broadcast code consists of a 14-character serial number and an additional character (1, 2, or 3), for a total of 15 characters. The above serial number is located under the QR code of the LiDAR body shell (see the figure below). The broadcast code is used to specify the LiDAR device to be connected. The detailed format is as follows :

&ensp;&ensp;&ensp;&ensp;![Broadcast Code](images/broadcast_code.png)

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;X in the figure above corresponds to 1 in MID-100_Left/MID-40/Horizon/Tele products, 2 in MID-100_Middle, and 3 in MID-100_Right.

## 4. Launch file and livox_ros_driver internal parameter configuration instructions

### 4.1 Launch file configuration instructions

All launch files of livox_ros_driver are in the "ws_livox/src/livox_ros_driver/launch" directory. Different launch files have different configuration parameter values and are used in different scenarios :

| launch file name          | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| livox_lidar_rviz.launch   | Connect to Livox LiDAR device<br>Publish pointcloud2 format data<br>Autoload rviz |
| livox_hub_rviz.launch     | Connect to Livox Hub device<br>Publish pointcloud2 format data<br>Autoload rviz |
| livox_lidar.launch        | Connect to Livox LiDAR device<br>Publish pointcloud2 format data |
| livox_hub.launch          | Connect to Livox LiDAR device<br>Publish pointcloud2 format data |
| livox_lidar_msg.launch    | Connect to Livox LiDAR device<br>Publish livox customized pointcloud data |
| livox_hub_msg.launch      | Connect to Livox Hub device<br>Publish livox customized pointcloud data |
| lvx_to_rosbag.launch      | Convert lvx file to rosbag file<br>Convert lvx files to rosbag files directly |
| lvx_to_rosbag_rviz.launch | Convert lvx file to rosbag file<br>Read raw pointcloud data from lvx file and convert to pointcloud2 format for publishing |

#### 4.2 Livox_ros_driver internal main parameter configuration instructions

All internal parameters of Livox_ros_driver are in the launch file. Below are detailed descriptions of the three commonly used parameters :

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | Set the frequency of point cloud publish <br>Floating-point data type, recommended values 5.0, 10.0, 20.0, 50.0, etc. | 10.0    |
| multi_topic  | If the LiDAR device has an independent topic to publish pointcloud data<br>0 -- All LiDAR devices use the same topic to publish pointcloud data<br>1 -- Each LiDAR device has its own topic to publish point cloud data | 0       |
| xfer_format  | Set pointcloud format<br>0 -- Livox pointcloud2(PointXYZRTL) pointcloud format<br>1 -- Livox customized pointcloud format<br>2 -- Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library | 0       |

&ensp;&ensp;&ensp;&ensp;***Livox_ros_driver pointcloud data detailed description :***

1. Livox pointcloud2 (PointXYZRTL) point cloud format, as follows :

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity         # the value is reflectivity, 0.0~255.0
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```

2. Livox customized data package format, as follows :

```c
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;Customized Point Cloud (CustomPoint) format in the above customized data package :

```c
uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```

1. The standard pointcloud2 (pcl :: PointXYZI)  format in the PCL library :

&ensp;&ensp;&ensp;&ensp;Please refer to the pcl :: PointXYZI data structure in the point_types.hpp file of the PCL library.

## 5. Configure LiDAR parameters

In the "ws_livox/src/livox_ros_driver/launch" path, there are two json files, livox_hub_config.json and livox_lidar_config.json.

1. When connecting directly to LiDAR, use the livox_lidar_config.json file to configure LiDAR parameters. Examples of file contents are as follows :

```json
{
   "lidar_config": [
      {
         "broadcast_code": "0TFDG3B006H2Z11",
         "enable_connect": true,
         "enable_fan": true,
         "return_mode": 0,
         "coordinate": 0,
         "imu_rate": 1,
         "extrinsic_parameter_source": 0
      }
   ]
}
```

&ensp;&ensp;&ensp;&ensp;The parameter attributes in the above json file are described in the following table :

LiDAR configuration parameter
| Parameter                  | Type    | Description                                                  | Default         |
| :------------------------- | ------- | ------------------------------------------------------------ | --------------- |
| broadcast_code             | String  | LiDAR broadcast code, 15 characters, consisting of a 14-character length serial number plus a character-length additional code | 0TFDG3B006H2Z11 |
| enable_connect             | Boolean | Whether to connect to this LiDAR<br>true -- Connect this LiDAR<br>false --Do not connect this LiDAR | false           |
| return_mode                | Int     | return mode<br>0 -- First single return mode<br>1 -- Strongest single return mode<br>2 -- Dual return mode | 0               |
| coordinate                 | Int     | Coordinate<br>0 -- Cartesian<br>1 -- Spherical               | 0               |
| imu_rate                   | Int     | Push frequency of IMU sensor data<br>0 -- stop push<br>1 -- 200 Hz<br>Others -- undefined, it will cause unpredictable behavior<br>Currently only Horizon supports this, MID serials do not support it | 0               |
| extrinsic_parameter_source | Int     | Whether to enable extrinsic parameter automatic compensation<br>0 -- Disable automatic compensation of LiDAR external reference<br>1 -- Automatic compensation of LiDAR external reference | 0               |

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;When connecting multiple LiDAR, if you want to use the external parameter automatic compensation function, you must first use the livox viewer to calibrate the external parameters and save them to LiDAR.

2. When connecting to the Hub, use livox_hub_config.json to configure the parameters of the Hub and LiDAR. Examples of file contents are as follows :

```json
{
   "hub_config": {
      "broadcast_code": "13UUG1R00400170",
      "enable_connect": true,
      "coordinate": 0
   },
   "lidar_config": [
      {
         "broadcast_code": "0TFDG3B006H2Z11",
         "return_mode": 0,
         "imu_rate": 1
      }
   ]
}
```

&ensp;&ensp;&ensp;&ensp;The main difference between the content of Hub json configuration file and the content of the LiDAR json configuration file is that the Hub configuration item "hub_config" is added, and the related configuration content of the Hub is shown in the following table :

HUB configuration parameter
| Parameter      | Type    | Description                                                  | Default         |
| -------------- | ------- | ------------------------------------------------------------ | --------------- |
| broadcast_code | String  | HUB broadcast code, 15 characters, consisting of a 14-character length serial number plus a character-length additional code | 13UUG1R00400170 |
| enable_connect | Boolean | Whether to connect to this Hub<br>true -- Connecting to this Hub means that all LiDAR data connected to this Hub will be received<br>false -- Prohibition of connection to this Hub means that all LiDAR data connected to this Hub will not be received | false           |
| coordinate     | Int     | Coordinate<br>0 -- Cartesian<br>1 -- Spherical             | 0               |

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;(1) The configuration parameters enable_connect and coordinate in the Hub configuration item "hub_config" are global and control the behavior of all LiDARs. Therefore, the LiDAR related configuration in the Hub json configuration file does not include these two contents.

&ensp;&ensp;&ensp;&ensp;(2) The Hub itself supports compensation of LiDAR external parameters, and does not require livox_ros_driver to compensate.

## 6. livox_ros_driver timestamp synchronization function

### 6.1 Hardware requirements

Prepare a GPS device to ensure that the GPS can output UTC time information in GPRMC/GNRMC format through the serial port or USB virtual serial port, and support PPS signal output; then connect the GPS serial port to the host running livox_ros_driver, and connect the GPS PPS signal line to LiDAR. For detailed connection instructions and more introduction to time stamp synchronization, please refer to the following links:

[Timestamp synchronization](https://github.com/Livox-SDK/Livox-SDK/wiki/Timestamp-Synchronization)

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;(1) The time stamp synchronization function of livox_ros_driver is based on the LidarSetUtcSyncTime interface of Livox-SDK, and only supports GPS synchronization, which is one of many synchronization methods of livox devices.

&ensp;&ensp;&ensp;&ensp;(2) Be sure to set the output frequency of GPRMC/GNRMC time information of GPS to 1Hz, other frequencies are not recommended.

&ensp;&ensp;&ensp;&ensp;(3) Examples of GPRMC/GNRMC format strings are as follows :

```bash
$GNRMC,143909.00,A,5107.0020216,N,11402.3294835,W,0.036,348.3,210307,0.0,E,A*31
$GNRMC,021225.00,A,3016.60101,N,12007.84214,E,0.011,,260420,,,A*67
$GPRMC,010101.130,A,3606.6834,N,12021.7778,E,0.0,238.3,010807,,,A*6C
$GPRMC,092927.000,A,2235.9058,N,11400.0518,E,0.000,74.11,151216,,D*49
$GPRMC,190430,A,4812.3038,S,07330.7690,W,3.7,3.8,090210,13.7,E,D*26
```

### 6.2 Enable timestamp synchronization

livox_ros_driver only supports the timestamp synchronization function when connected to LiDAR. The timestamp related configuration item timesync_config is in the livox_lidar_config.json file. The detailed configuration content is shown in the table below :

Timestamp synchronization function configuration instructions
| Parameter        | Type     | Description                                                  | Default        |
| ---------------- | -------- | ------------------------------------------------------------ | -------------- |
| enable_timesync  | Boolean  | Whether to enable the timestamp synchronization <br>true -- Enable timestamp synchronization<br>false -- Disable timestamp synchronization | false          |
| device_name      | String | Name of the serial device to be connected, take "/dev/ttyUSB0" as an example, indicating that the device sending timestamp information to livox_ros_driver is ttyUSB0 | "/dev/ttyUSB0" |
| comm_device_type | Int      | Type of device sending timestamp information<br>0 -- Serial port or USB virtual serial port device<br>other -- not support | 0              |
| baudrate_index   | Int      | Baud rate of serial device<br>0 -- 2400 <br>1 -- 4800 <br>2 -- 9600 <br>3 -- 19200 <br>4 -- 38400 <br>5 -- 57600 <br>6 -- 115200 <br>7 -- 230400 <br>8 -- 460800 <br>9 -- 500000 <br>10 -- 576000 <br>11 -- 921600 | 2              |
| parity_index     | Int      | parity type<br>0 -- 8bits data without parity<br>1 -- 7bits data 1bit even parity<br>2 -- 7bits data 1bit odd parity<br>3 -- 7bits data 1bit 0, without parity | 0              |

## 7. Convert lvx point cloud data file (v1.0/v1.1) to rosbag file

livox_ros_driver supports the conversion of lvx pointcloud data files to rosbag files. Use the command as follows :

`roslaunch livox_ros_driver lvx_to_rosbag.launch lvx_file_path:="/home/livox/test.lvx"`

After replacing "/home/livox/test.lvx" in the above command with the local lvx data file path, you can simply run it; if the conversion is successful, a rosbag format file with the same name will be generated under the above path.

## 8. Application Documents

* [How to use lvx file in ros](https://github.com/Livox-SDK/Livox-SDK/wiki/How-to-use-lvx-file-under-ros)
* [Set publish frequency](https://github.com/Livox-SDK/Livox-SDK/wiki/Set-publish-frequency)
* [外参标定与点云显示](https://github.com/Livox-SDK/Livox-SDK/wiki/Calibrate-extrinsic-and-display-under-ros-cn)

## 9. Support

You can get support from Livox with the following methods :

* Send email to cs@livoxtech.com with a clear description of your problem and your setup
* Report issue on github
