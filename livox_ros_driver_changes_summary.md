# Livox ROS Driver 功能增强总结

## 概述

本次对 Livox ROS Driver 进行了多项功能增强，主要包括：
1. **新增激光雷达工作模式控制功能** - 通过 ROS 服务动态切换工作模式
2. **新增激光雷达状态监控功能** - 通过 ROS 话题实时发布状态信息
3. **修复了状态消息中 `mode` 字段的错误映射**
4. **添加了 `return_mode` 字段来正确显示点云回波模式**
5. **新增了点云回波模式控制服务**
6. **完善了状态发布功能**

## 更改详情

### 1. 新增激光雷达工作模式控制功能

#### 功能描述
实现了通过 ROS 服务动态控制激光雷达工作模式的功能，支持正常模式、省电模式和待机模式之间的切换。

#### 服务定义
- **文件**: `livox_ros_driver/livox_ros_driver/srv/LidarModeControl.srv`

```srv
# Request
string broadcast_code    # LiDAR broadcast code (empty for all connected LiDARs)
uint8 mode              # Working mode (1:Normal, 2:PowerSaving, 3:Standby)
---
# Response
bool success            # Operation success status
string message          # Response message
```

#### 服务实现
- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/lds_lidar.h`
- **新增方法**:
  ```cpp
  bool SetLidarMode(const std::string& broadcast_code, LidarMode mode);
  bool SetAllLidarMode(LidarMode mode);
  ```

- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/lds_lidar.cpp`
- **实现**: 添加了完整的工作模式设置逻辑
  ```cpp
  bool LdsLidar::SetLidarMode(const std::string& broadcast_code, LidarMode mode) {
    uint8_t handle = GetHandleByBroadcastCode(broadcast_code);
    if (handle >= kMaxLidarCount) {
      printf("LiDAR with broadcast code %s not found or not connected\n", broadcast_code.c_str());
      return false;
    }
    
    // 检查设备类型是否支持模式切换
    if (lidars_[handle].info.type != kDeviceTypeLidarHorizon && 
        lidars_[handle].info.type != kDeviceTypeLidarAvia) {
      printf("Device type %d does not support mode switching\n", lidars_[handle].info.type);
      return false;
    }
    
    // 调用SDK设置模式
    livox_status status = LidarSetMode(broadcast_code.c_str(), mode);
    if (status != kStatusSuccess) {
      printf("Failed to set LiDAR mode, status: %d\n", status);
      return false;
    }
    
    printf("Successfully set LiDAR %s to mode %d\n", broadcast_code.c_str(), mode);
    return true;
  }
  ```

- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/livox_ros_driver.cpp`
- **服务注册**: 添加了服务回调函数和注册代码
  ```cpp
  bool LidarModeControlCallback(livox_ros_driver::LidarModeControl::Request &req,
                                livox_ros_driver::LidarModeControl::Response &res) {
    if (!g_lidar_instance) {
      res.success = false;
      res.message = "LiDAR instance not available";
      return true;
    }

    if (req.broadcast_code.empty()) {
      // 设置所有激光雷达模式
      res.success = g_lidar_instance->SetAllLidarMode(static_cast<LidarMode>(req.mode));
      res.message = res.success ? "Set all LiDARs mode successfully" : "Failed to set all LiDARs mode";
    } else {
      // 设置指定激光雷达模式
      res.success = g_lidar_instance->SetLidarMode(req.broadcast_code, static_cast<LidarMode>(req.mode));
      res.message = res.success ? "Set LiDAR mode successfully" : "Failed to set LiDAR mode";
    }

    return true;
  }
  ```

#### 构建配置
- **文件**: `livox_ros_driver/livox_ros_driver/CMakeLists.txt`
- **更改**: 在 `add_service_files` 中添加了 `LidarModeControl.srv`

### 2. 新增激光雷达状态监控功能

#### 功能描述
实现了通过 ROS 话题实时发布激光雷达状态信息的功能，包括连接状态、工作状态、设备信息等。

#### 消息定义
- **文件**: `livox_ros_driver/livox_ros_driver/msg/LidarStatus.msg`

```msg
# Livox LiDAR status message format.

Header header                    # ROS standard message header
string broadcast_code           # LiDAR broadcast code
uint8 handle                    # LiDAR handle
uint8 device_type              # Device type (0:Hub, 1:Mid40, 2:Tele, 3:Horizon, 6:Mid70, 7:Avia)
uint8 state                     # Current working state (0:Init, 1:Normal, 2:PowerSaving, 3:StandBy, 4:Error, 5:Unknown)
uint8 mode                      # Current working mode (1:Normal, 2:PowerSaving, 3:Standby)
uint8 feature                   # LiDAR feature (0:None, 1:RainFog)
uint8 return_mode               # Point cloud return mode (0:FirstReturn, 1:StrongestReturn, 2:DualReturn, 3:TripleReturn)
string ip                       # Device IP address
uint16 data_port               # Point cloud data UDP port
uint16 cmd_port                # Control command UDP port
uint16 sensor_port             # IMU data UDP port
uint8[4] firmware_version      # Firmware version
bool is_connected              # Connection status
bool is_sampling               # Sampling status
uint32 error_code              # Error code
```

#### 状态发布实现
- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/lddc.h`
- **新增方法**:
  ```cpp
  ros::Publisher *GetCurrentStatusPublisher(uint8_t handle);
  void PublishLidarStatus(uint8_t handle);
  void PublishAllLidarStatus();
  ```

- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/lddc.cpp`
- **实现**: 添加了完整的状态发布逻辑
  ```cpp
  ros::Publisher *Lddc::GetCurrentStatusPublisher(uint8_t handle) {
    ros::Publisher **pub = nullptr;
    uint32_t queue_size = kMinEthPacketQueueSize;

    if (use_multi_topic_) {
      pub = &private_status_pub_[handle];
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
    } else {
      pub = &global_status_pub_;
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
    }

    if (*pub == nullptr) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      if (use_multi_topic_) {
        snprintf(name_str, sizeof(name_str), "livox/status_%s",
                 lds_->lidars_[handle].info.broadcast_code);
        ROS_INFO("Support multi status topics.");
      } else {
        ROS_INFO("Support only one status topic.");
        snprintf(name_str, sizeof(name_str), "livox/status");
      }

      *pub = new ros::Publisher;
      **pub = cur_node_->advertise<livox_ros_driver::LidarStatus>(name_str, queue_size);
      ROS_INFO("%s publish lidar status, set ROS publisher queue size %d", name_str, queue_size);
    }

    return *pub;
  }

  void Lddc::PublishLidarStatus(uint8_t handle) {
    if (!lds_ || handle >= kMaxSourceLidar) {
      return;
    }

    const LidarDevice& lidar = lds_->lidars_[handle];
    if (lidar.connect_state == kConnectStateOff) {
      return;  // 未连接，不发布状态
    }

    livox_ros_driver::LidarStatus status_msg;
    status_msg.header.stamp = ros::Time::now();
    status_msg.header.frame_id = frame_id_;
    
    status_msg.broadcast_code = std::string(lidar.info.broadcast_code);
    status_msg.handle = handle;
    status_msg.device_type = lidar.info.type;
    status_msg.state = lidar.info.state;
    // 根据state推断mode，因为SDK没有提供查询mode的API
    if (lidar.info.state == kLidarStateNormal) {
      status_msg.mode = 1;  // 正常模式
    } else if (lidar.info.state == kLidarStatePowerSaving) {
      status_msg.mode = 2;  // 省电模式
    } else if (lidar.info.state == kLidarStateStandBy) {
      status_msg.mode = 3;  // 待机模式
    } else {
      status_msg.mode = 0;  // 未知模式
    }
    status_msg.feature = lidar.info.feature;
    status_msg.return_mode = lidar.config.return_mode;  // 添加点云回波模式
    status_msg.ip = std::string(lidar.info.ip);
    status_msg.data_port = lidar.info.data_port;
    status_msg.cmd_port = lidar.info.cmd_port;
    status_msg.sensor_port = lidar.info.sensor_port;
    
    for (int i = 0; i < 4; i++) {
      status_msg.firmware_version[i] = lidar.info.firmware_version[i];
    }
    
    status_msg.is_connected = (lidar.connect_state != kConnectStateOff);
    status_msg.is_sampling = (lidar.connect_state == kConnectStateSampling);
    status_msg.error_code = lidar.info.status.status_code.error_code;

    ros::Publisher *p_publisher = GetCurrentStatusPublisher(handle);
    if (kOutputToRos == output_type_) {
      p_publisher->publish(status_msg);
    } else {
      if (bag_ && enable_lidar_bag_) {
        bag_->write(p_publisher->getTopic(), status_msg.header.stamp, status_msg);
      }
    }
  }
  ```

#### 状态发布集成
- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/lddc.cpp`
- **集成**: 在数据轮询循环中添加状态发布
  ```cpp
  void Lddc::PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar) {
    // ... 现有代码 ...
    
    // 新增：定期发布状态信息
    static uint32_t status_publish_counter = 0;
    if (++status_publish_counter >= 100) {  // 每100次轮询发布一次状态
      PublishLidarStatus(handle);
      status_publish_counter = 0;
    }
  }
  ```

#### 构建配置
- **文件**: `livox_ros_driver/livox_ros_driver/CMakeLists.txt`
- **更改**: 在 `add_message_files` 中添加了 `LidarStatus.msg`

### 3. 修复状态消息字段映射

#### 问题描述
原始代码中 `mode` 字段被错误地映射为 `feature` 值，导致状态信息不准确。

#### 修复内容
- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/lddc.cpp`
- **函数**: `PublishLidarStatus()`
- **更改**: 根据 `state` 字段正确推断 `mode` 值

```cpp
// 修复前
status_msg.mode = lidar.info.feature;  // 错误映射

// 修复后
if (lidar.info.state == kLidarStateNormal) {
  status_msg.mode = 1;  // 正常模式
} else if (lidar.info.state == kLidarStatePowerSaving) {
  status_msg.mode = 2;  // 省电模式
} else if (lidar.info.state == kLidarStateStandBy) {
  status_msg.mode = 3;  // 待机模式
} else {
  status_msg.mode = 0;  // 未知模式
}
```

### 4. 添加点云回波模式字段

#### 新增内容
- **文件**: `livox_ros_driver/livox_ros_driver/msg/LidarStatus.msg`
- **新增字段**: `uint8 return_mode` - 点云回波模式

```msg
# 新增字段
uint8 return_mode                   # Point cloud return mode (0:FirstReturn, 1:StrongestReturn, 2:DualReturn, 3:TripleReturn)
```

#### 状态发布更新
- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/lddc.cpp`
- **函数**: `PublishLidarStatus()`
- **添加**: `status_msg.return_mode = lidar.config.return_mode;`

### 5. 新增点云回波模式控制服务

#### 服务定义
- **文件**: `livox_ros_driver/livox_ros_driver/srv/PointCloudReturnModeControl.srv`

```srv
# Request
string broadcast_code    # LiDAR broadcast code (empty for all connected LiDARs)
uint8 return_mode        # Point cloud return mode (0:FirstReturn, 1:StrongestReturn, 2:DualReturn, 3:TripleReturn)
---
# Response
bool success            # Operation success status
string message          # Response message
```

#### 服务实现
- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/lds_lidar.h`
- **新增方法**:
  ```cpp
  bool SetPointCloudReturnMode(const std::string& broadcast_code, PointCloudReturnMode return_mode);
  bool SetAllPointCloudReturnMode(PointCloudReturnMode return_mode);
  ```

- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/lds_lidar.cpp`
- **实现**: 添加了完整的点云回波模式设置逻辑

- **文件**: `livox_ros_driver/livox_ros_driver/livox_ros_driver/livox_ros_driver.cpp`
- **服务注册**: 添加了服务回调函数和注册代码

#### 构建配置更新
- **文件**: `livox_ros_driver/livox_ros_driver/CMakeLists.txt`
- **更改**: 在 `add_service_files` 中添加了 `PointCloudReturnModeControl.srv`

## 字段含义说明

### 状态消息字段详解

#### 1. `state` (状态)
```cpp
typedef enum {
  kLidarStateInit = 0,        // 初始化状态
  kLidarStateNormal = 1,      // 正常工作状态
  kLidarStatePowerSaving = 2, // 省电状态
  kLidarStateStandBy = 3,     // 待机状态
  kLidarStateError = 4,       // 错误状态
  kLidarStateUnknown = 5      // 未知状态
} LidarState;
```

#### 2. `mode` (工作模式)
```cpp
typedef enum {
  kLidarModeNormal = 1,      // 正常模式
  kLidarModePowerSaving = 2, // 省电模式
  kLidarModeStandby = 3      // 待机模式
} LidarMode;
```

#### 3. `feature` (特殊功能)
```cpp
typedef enum {
  kLidarFeatureNone = 0,   // 无特殊功能
  kLidarFeatureRainFog = 1 // 雨雾抑制功能
} LidarFeature;
```

#### 4. `return_mode` (点云回波模式)
```cpp
typedef enum {
  kFirstReturn = 0,        // 第一次回波
  kStrongestReturn = 1,    // 最强回波
  kDualReturn = 2,         // 双回波
  kTripleReturn = 3        // 三回波
} PointCloudReturnMode;
```

## 架构设计

### ROS 服务与话题设计

#### 控制功能 (ROSService)
- **工作模式控制**: `/livox/lidar_mode_control`
  - 功能: 动态切换激光雷达工作模式
  - 参数: `broadcast_code` (激光雷达码), `mode` (工作模式)
  - 响应: `success` (操作结果), `message` (详细信息)

- **点云回波模式控制**: `/livox/point_cloud_return_mode_control`
  - 功能: 动态设置点云回波模式
  - 参数: `broadcast_code` (激光雷达码), `return_mode` (回波模式)
  - 响应: `success` (操作结果), `message` (详细信息)

#### 状态监控 (ROSTopic)
- **状态信息发布**: `/livox/status` 或 `/livox/status_{broadcast_code}`
  - 功能: 实时发布激光雷达状态信息
  - 消息类型: `livox_ros_driver/LidarStatus`
  - 发布频率: 每100次数据轮询发布一次
  - 支持多话题模式: 每个激光雷达独立话题

#### 数据流设计
```
激光雷达设备 → Livox SDK → ROS Driver → ROS 话题/服务
                ↓
           状态监控 ← 定期轮询 ← 数据采集
                ↓
           控制命令 → 服务回调 → 模式设置
```

## 测试方法

### 1. 编译和部署

```bash
# 进入工作空间
cd ~/catkin_ws

# 编译
catkin_make

# 刷新环境
source devel/setup.bash
```

### 2. 启动节点

```bash
# 启动 ROS master (如果未运行)
roscore &

# 启动 Livox ROS Driver
roslaunch livox_ros_driver livox_lidar.launch
```

### 3. 查看状态信息

```bash
# 查看激光雷达状态
rostopic echo /livox/status

# 预期输出示例
header: 
  seq: 193
  stamp: 
    secs: 1751645070
    nsecs: 398878493
  frame_id: "livox_frame"
broadcast_code: "1HDDH3200104541"
handle: 0
device_type: 3
state: 1                    # 正常工作状态
mode: 1                     # 正常工作模式
feature: 0                  # 无特殊功能
return_mode: 0              # 第一次回波模式
ip: "192.168.1.41"
data_port: 56001
cmd_port: 55501
sensor_port: 56001
firmware_version: [6, 15, 0, 0]
is_connected: True
is_sampling: True
error_code: 0
```

### 4. 测试工作模式控制

```bash
# 查看可用服务
rosservice list

# 应该能看到以下服务：
# /livox/lidar_mode_control
# /livox/point_cloud_return_mode_control

# 测试工作模式设置 - 切换到省电模式
rosservice call /livox/lidar_mode_control "broadcast_code: '1HDDH3200104541'
mode: 2"

# 预期响应
success: True
message: "Set LiDAR mode successfully"

# 测试工作模式设置 - 切换到待机模式
rosservice call /livox/lidar_mode_control "broadcast_code: '1HDDH3200104541'
mode: 3"

# 测试工作模式设置 - 切换回正常模式
rosservice call /livox/lidar_mode_control "broadcast_code: '1HDDH3200104541'
mode: 1"

# 测试设置所有激光雷达模式
rosservice call /livox/lidar_mode_control "broadcast_code: ''
mode: 1"
```

### 5. 测试点云回波模式控制

```bash
# 设置最强回波模式
rosservice call /livox/point_cloud_return_mode_control "broadcast_code: '1HDDH3200104541'
return_mode: 1"

# 设置双回波模式
rosservice call /livox/point_cloud_return_mode_control "broadcast_code: '1HDDH3200104541'
return_mode: 2"

# 设置三回波模式
rosservice call /livox/point_cloud_return_mode_control "broadcast_code: '1HDDH3200104541'
return_mode: 3"

# 设置所有激光雷达为第一次回波模式
rosservice call /livox/point_cloud_return_mode_control "broadcast_code: ''
return_mode: 0"

# 注意：必须使用换行格式，单行格式会导致YAML解析错误
```

### 6. 验证状态变化

```bash
# 实时监控状态变化
rostopic echo /livox/status | grep -E "(return_mode|mode|state)"

# 或者使用 watch 命令
watch -n 1 'rostopic echo /livox/status -n 1 | grep -E "(return_mode|mode|state)"'

# 监控特定激光雷达的状态（多话题模式）
rostopic echo /livox/status_1HDDH3200104541

# 查看话题信息
rostopic info /livox/status
rostopic hz /livox/status

# 查看服务信息
rosservice info /livox/lidar_mode_control
rosservice info /livox/point_cloud_return_mode_control
```

### 7. 完整测试流程

```bash
# 1. 启动节点
roslaunch livox_ros_driver livox_lidar.launch

# 2. 监控初始状态
rostopic echo /livox/status

# 3. 测试工作模式切换
rosservice call /livox/lidar_mode_control "broadcast_code: '1HDDH3200104541'
mode: 2"
# 观察状态变化
rostopic echo /livox/status | grep -E "(state|mode)"

# 4. 测试点云回波模式切换
rosservice call /livox/point_cloud_return_mode_control "broadcast_code: '1HDDH3200104541'
return_mode: 1"
# 观察状态变化
rostopic echo /livox/status | grep -E "return_mode"

# 5. 恢复默认设置
rosservice call /livox/lidar_mode_control "broadcast_code: '1HDDH3200104541'
mode: 1"
rosservice call /livox/point_cloud_return_mode_control "broadcast_code: '1HDDH3200104541'
return_mode: 0"
```

## 常见问题解决

### 1. YAML 格式错误
**问题**: `yaml.parser.ParserError: while parsing a block mapping`
**解决**: 必须使用换行格式调用服务
```bash
# 正确格式（换行）- 必须使用
rosservice call /service_name "param1: 'value1'
param2: value2"

# 错误格式 - 会导致解析错误
rosservice call /service_name "param1: 'value1' param2: value2"  # 单行格式会解析失败
```

### 2. 服务未找到
**问题**: `rosservice list` 中看不到新服务
**解决**: 
1. 确保已重新编译: `catkin_make`
2. 刷新环境: `source devel/setup.bash`
3. 重启节点

### 3. 编译错误
**问题**: 找不到新服务的头文件
**解决**: 确保在 `CMakeLists.txt` 中正确添加了服务文件

## 功能验证清单

### 基础功能
- [ ] 编译成功，无错误
- [ ] 节点正常启动
- [ ] 激光雷达正常连接和数据采集

### 状态监控功能
- [ ] 状态消息正常发布 (`/livox/status`)
- [ ] `state` 字段显示正确的工作状态
- [ ] `mode` 字段显示正确的工作模式
- [ ] `feature` 字段显示正确的特殊功能状态
- [ ] `return_mode` 字段显示正确的回波模式
- [ ] 连接状态和采样状态正确显示
- [ ] 多话题模式正常工作 (如果启用)

### 控制功能
- [ ] 工作模式控制服务响应正常 (`/livox/lidar_mode_control`)
- [ ] 点云回波模式控制服务响应正常 (`/livox/point_cloud_return_mode_control`)
- [ ] 单个激光雷达控制功能正常
- [ ] 批量激光雷达控制功能正常
- [ ] 错误处理机制正常工作

### 实时性验证
- [ ] 状态变化实时更新
- [ ] 控制命令响应及时
- [ ] 状态发布频率合理
- [ ] 无数据丢失或延迟

### 兼容性验证
- [ ] 支持不同型号激光雷达
- [ ] 向后兼容原有功能
- [ ] 配置文件兼容性
- [ ] 多激光雷达场景支持

## 注意事项

### 使用注意事项
1. **广播码**: 请将示例中的 `1HDDH3200104541` 替换为你实际的激光雷达广播码
2. **权限**: 某些操作可能需要管理员权限
3. **网络**: 确保激光雷达网络连接正常
4. **兼容性**: 不同型号的激光雷达可能支持的功能不同

### 技术注意事项
1. **工作模式切换**: 只有 Horizon 和 Avia 型号支持工作模式切换
2. **状态发布频率**: 状态信息每100次数据轮询发布一次，避免过度占用系统资源
3. **服务调用格式**: 必须使用换行格式调用服务，单行格式会导致YAML解析错误
4. **多话题模式**: 启用多话题模式时，每个激光雷达会有独立的状态话题

### 性能注意事项
1. **状态监控开销**: 状态发布会增加一定的系统开销
2. **服务响应时间**: 模式切换可能需要几秒钟时间
3. **网络带宽**: 多激光雷达场景下注意网络带宽使用
4. **内存使用**: 长时间运行注意内存泄漏问题

### 故障排除
1. **服务无响应**: 检查激光雷达连接状态和网络配置
2. **状态不更新**: 检查话题发布是否正常
3. **模式切换失败**: 确认激光雷达型号支持该功能
4. **编译错误**: 确保所有依赖文件都已正确添加
5. **YAML解析错误**: 必须使用换行格式调用服务，单行格式会导致解析错误

## 版本信息

- **ROS版本**: Noetic
- **Livox SDK版本**: 2.3.0
- **修改日期**: 2024年
- **测试状态**: 已通过基本功能测试 