# Livox ROS Driver（钛兴科技定制版）

本分支基于官方 [livox_ros_driver v2.6.0](https://github.com/Livox-SDK/livox_ros_driver) 修改，面向**多雷达 + 工业环境长时间运行**场景，新增以下功能与可靠性修复：

1. **在线工作模式切换** — 运行时通过 ROS Service 切换 LiDAR 工作模式（Normal / PowerSaving / Standby）
2. **远程重启** — 通过 ROS Service 软重启雷达，无需现场断电
3. **可配置点云距离过滤** — 通过 launch 参数设置最大发布距离，无需重新编译
4. **掉线崩溃修复（UAF）** — 修复官方驱动在雷达掉线时的 use-after-free 竞态崩溃
5. **状态抖动断流修复** — 避免温度/电机告警等瞬时状态抖动导致话题断流
6. **丢包可视化** — 异常时日志告警 + `livox/lidar_stats` 实时看板（独立终端原地刷新）
7. **畸形包硬化** — 拒绝非法 `data_type`，堵住缓冲区溢出

> 功能 1/2 需配套修改版 SDK；功能 3~7 为纯 ROS 驱动层改动，配任意 SDK 均可用。详见各章节。

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

### 远程重启

当雷达进入异常状态（如长时间运行后丢包/无响应），可以远程软重启，无需现场断电：

```bash
# 重启单台雷达
rosservice call /livox_lidar_reboot "{handle: 0}"

# 重启所有已连接雷达
rosservice call /livox_lidar_reboot "{handle: 255}"
```

| 参数 | 取值 | 说明 |
|------|------|------|
| `handle` | 0~31 | 单个雷达句柄 |
| `handle` | 255 | 所有已连接雷达 |

> 调用 SDK 的 `RebootDevice()`，雷达会断开并在数秒后重新上线，驱动的重连逻辑会自动恢复采样。Horizon 支持；Mid40/100 需固件 ≥ 03.07。

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

## 新增功能三：可靠性修复（多雷达长时间运行）

> 以下为纯 ROS 驱动层修复，不需要改 SDK。

### 1. 掉线崩溃（use-after-free）修复

**官方 bug**：雷达掉线时，`ResetLidar` 在 SDK 设备状态线程上释放数据队列，而 SDK 数据接收线程仍可能往同一队列写入——两者无任何锁同步，导致 **use-after-free / 堆损坏**，在多雷达偶发掉线时崩溃或话题假死。

**修复**：为每台雷达引入一把 `std::mutex`，把**写入（StorageRawPacket）/ 读取（DistributeLidarData）/ 释放（ResetLidar）** 三条路径互斥；并在 `DeInitQueue` 释放后置空指针、各队列操作加空指针兜底。从根上消除竞态（区别于裸 null 检查的临时补丁）。

### 2. 状态抖动导致话题断流修复

**问题**：早期版本在雷达状态从「任意非 Normal → Normal」时都会重置 `connect_state` 重跑配置，于是**温度/电机告警等瞬时 Error→Normal 抖动**也会触发完整重配置 → 话题断流几百 ms。工业现场高温、震动环境下频繁发生。

**修复**：仅在「确实从节电/待机恢复」或「我们主动请求的 Normal 切换正在完成」时才重配置，瞬时告警抖动不再打断已在采样的雷达。

---

## 新增功能四：丢包可视化

提供两种查看方式，按需选用。

### 方式 A：日志告警（仅异常时输出）

驱动每 5 秒检查一次，**只有在该窗口内发生丢包时才打印一行**，健康运行时日志保持干净：

```
[LivoxStats][WARN] Lidar[0][1PQDH5B00100041] 5s: recv=12480 net_loss=8(0.06%) queue_drop=3(0.02%) | total recv=998400 net_loss=152 drop=10
```

| 字段 | 含义 | 指向 |
|------|------|------|
| `recv` | 最近 5 秒收到的点云包数 | 速率是否稳定 |
| `net_loss` | **网络丢包**（包未到达驱动，按时间戳间隔估算）| 网线 / 交换机 / 雷达硬件 / 散热 |
| `queue_drop` | **队列丢包**（驱动消费不过来）| 下游订阅者慢 / CPU 瓶颈 |
| `total ...` | 自启动以来累计 | 长期趋势 |

> 出现 `[LivoxStats][WARN]` 就代表有丢包；持续没有，说明一切正常。

### 方式 B：实时看板（独立终端，原地刷新，互不干扰）⭐推荐

驱动每秒发布 `livox/lidar_stats` topic。在**另一个终端**运行看板脚本，它会原地刷新（像 `htop`），永远显示当前值，且与驱动日志完全隔离。

#### 使用步骤

**① 确保已重新编译**（看板是新功能，旧版本没有）：
```bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

**② 终端 1 — 启动驱动**（日志在这里滚动）：
```bash
roslaunch livox_ros_driver livox_lidar_multi.launch
```

**③ 终端 2 — 打开看板**（原地刷新，不受驱动日志干扰）：
```bash
rosrun livox_ros_driver livox_stats_monitor.py
```
> 若提示找不到（旧编译缓存），重新 `catkin_make && source devel/setup.bash` 即可；
> 或直接用绝对路径运行：`python3 $(rospack find livox_ros_driver)/livox_ros_driver/scripts/livox_stats_monitor.py`
> （注意本仓库源码目录多嵌套一层 `livox_ros_driver`）。

看板效果（掉线的雷达会明确标 `DISCONNECTED`，不会从看板上消失）：
```
===== Livox LiDAR Stats (1Hz) =====
handle  broadcast_code   state         temp  fan   recv/s  loss/s  drop/s   disc  last_drop   uptime
0       1PQDH5B00100041  Normal        OK    OK      2496       0       0      0         --    2h13m
1       0TFDG3U99101431  Normal        WARN  OK      2498       2       0      7      3m12s    3m12s
2       3WEDH5900103621  DISCONNECTED  -     -          -       -       -      2        45s       --
temp_status changes:  L0:0@--  L1:1@09:12:44  L2:0@--
(updated: 1718000000.0)
```

底部 `temp_status changes` 行记录每台雷达**温度状态变化的次数和最近一次时间**：
- `L0:0@--` → 0 号自启动以来温度状态从未变过（一直 OK）
- `L1:1@09:12:44` → 1 号变过 1 次，最近一次在 09:12:44（此刻 temp 列显示的就是变化后的值）

> 正常情况下全是 `:0@--`——这是对的，说明温度一直在正常区。一旦某台开始 `:1@时间`，就是它真的进过告警区，配合驱动终端的 `[LivoxHealth]` 行能看到具体变成了 WARN 还是 HOT!。

#### 怎么读看板

| 列 | 含义 |
|----|------|
| `state` | `Normal` 正常 / `DISCONNECTED` 掉线 / `PowerSaving` 节电 / `Error` 故障 |
| `temp` | 温度状态 `OK` / `WARN`(偏高偏低) / `HOT!`(极端)。⚠️ 是状态码，**不是具体℃**（见下方说明）|
| `fan` | 风扇状态 `OK` 正常 / `WARN` **故障**（注意：WARN 是风扇坏了，不是"在转"）|
| `recv/s` | 每秒收到的点云包数（应稳定，多台 Horizon 约 2500/s）|
| `loss/s` | 每秒网络丢包数（>0 说明网络/接头/散热在劣化，掉线前兆）|
| `drop/s` | 每秒队列丢包数（>0 说明主机/下游消费不过来）|
| `disc` | **累计掉线次数**（长期跑下来哪台最不稳，一眼看出）|
| `last_drop` | 上次掉线距今多久（`--` = 从未掉过）|
| `uptime` | 本次连接已稳定多久 |

> **判断哪台最该换**：`disc` 高 + `uptime` 短（反复掉、刚回来）的雷达，比偶尔丢几个包的更需要优先处理。

#### 关于温度与风扇（重要说明）

Livox SDK **不暴露具体温度数值**（如 62℃），那个 60℃ 风扇启动阈值是固件内部的。能拿到的只有粗粒度状态码：
- `temp`：`OK`=正常 / `WARN`=偏高或偏低 / `HOT!`=极高或极低
- `fan`：`OK`=正常 / `WARN`=**风扇故障告警**（拿不到转速，也拿不到"现在转没转"）

所以你能监控的是"**温度是否进入告警区 / 风扇是否报故障**"，而不是精确温度曲线。`temp=WARN` 大致对应雷达发热升高，可作为散热吃紧的间接信号。

#### 事件日志（掉线/重连 + 健康变化）

驱动终端在**状态发生变化时**打印带时间戳的事件（不刷屏）：
```
[LivoxEvent]  14:32:07 Lidar[1][0TFDG3U99100671] DISCONNECTED
[LivoxEvent]  14:32:19 Lidar[1][0TFDG3U99100671] RECONNECTED (down 12s)
[LivoxHealth] 14:35:02 Lidar[0] temp=WARN fan=OK motor=OK volt=OK dirty=0 firmware=0 self_heating=0 system=WARN
```
过滤查看：`roslaunch ... 2>&1 | grep -E "LivoxEvent|LivoxHealth"`

> `[LivoxHealth]` 只在 temp/fan/motor 等健康字段**变化时**才打印一行，所以正常时安静，一旦温度进告警区或风扇报故障会立刻看到。

> **某台 `loss/s` 持续 >0 → 重点排查那台的网线/接头/散热；某台 `DISCONNECTED` → 已掉线，可远程重启 `rosservice call /livox_lidar_reboot "{handle: N}"`。**

#### 不想用脚本？直接看原始 topic

```bash
rostopic echo /livox/lidar_stats
```
（会滚动刷屏，不如脚本清爽，但不需要任何额外文件。）

> **为什么不是"置顶在同一个终端"**：终端是线性滚动流，roscpp 日志和驱动 printf 都往同一个 stdout 写，无法稳定地把某几行钉在顶部（ANSI 滚动区域会被其它日志冲掉，重定向到文件还会变乱码）。独立终端的原地刷新看板是更可靠、更清晰的方案。

> **统计与连接状态挂钩**：点云数据（UDP）和心跳是两条独立通道，一台雷达可能"心跳掉线"但数据还在流。驱动判定某台雷达 `connect_state==Off` 后即**不再统计其数据**，因此看板的 `DISCONNECTED` 与驱动的掉线判定始终一致，不会出现"已断开却仍显示正常"的矛盾。

> 网络丢包按时间戳间隔估算（丢一个包，下一个包时间戳跳约 N 个间隔），并对重连 / PPS 同步的大跳变做了上限保护，避免误报。

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

### ROS Driver

| 文件 | 改动 |
|------|------|
| `srv/LidarMode.srv` | **新增** — 模式切换 Service 定义 |
| `srv/LidarReboot.srv` | **新增** — 重启 Service 定义 |
| `CMakeLists.txt` | 注册两个 srv |
| `livox_ros_driver/lds_lidar.h/.cpp` | 模式切换 + 重启 + 状态机抖动修复 |
| `livox_ros_driver/livox_ros_driver.cpp` | 模式/重启 Service、AsyncSpinner、max_distance 参数、`livox/lidar_stats` 看板发布 |
| `livox_ros_driver/lddc.h/.cpp` | 距离过滤 + 读取端 UAF 加锁 |
| `livox_ros_driver/lds.h/.cpp` | 每雷达锁、丢包统计（仅异常打印）、`data_type` 硬化、写入端 UAF 加锁 |
| `livox_ros_driver/ldq.cpp` | 队列释放置空 + 操作空指针兜底 |
| `scripts/livox_stats_monitor.py` | **新增** — 独立终端的实时丢包看板 |

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

### Q: 怎么判断丢包是网络问题还是驱动问题？
看 `[LivoxStats]` 日志：`net_loss` 高 → 网络/雷达硬件（查网线、交换机、散热）；`queue_drop` 高 → 下游消费太慢（订阅者慢 / CPU 瓶颈）。

### Q: 雷达长时间运行后无响应 / 丢包严重，怎么远程恢复？
不用现场断电，调用重启 service：`rosservice call /livox_lidar_reboot "{handle: 255}"`（255 = 全部）。

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
