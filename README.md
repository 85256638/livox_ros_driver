# Livox ROS Driver（钛兴科技定制版）

本分支基于官方 [livox_ros_driver v2.6.0](https://github.com/Livox-SDK/livox_ros_driver) 修改，面向**多雷达 + 工业环境长时间运行**场景，新增以下功能与可靠性修复：

1. **在线工作模式切换** — 运行时通过 ROS Service 切换 LiDAR 工作模式（Normal / PowerSaving / Standby）；四台批量唤醒按 **0/2/4/6 秒错峰**，Normal ACK 后保留 20 秒启动观察期，每台雷达的配置命令链串行，避免模式/配置命令集中冲击固件
2. **远程重启** — 通过 ROS Service 软重启雷达，无需现场断电
3. **可配置点云距离过滤** — 通过 launch 参数设置最大发布距离，无需重新编译
4. **掉线崩溃修复（UAF）** — 修复官方驱动在雷达掉线时的 use-after-free 竞态崩溃（收包/统计/队列已并入同一把锁的事务）
5. **状态抖动断流修复** — 避免温度/电机告警等瞬时状态抖动导致话题断流
6. **健康与丢包监控** — 异常日志告警 + 分层实时看板（数据源、软件版本、当前告警、逐台状态/滚动趋势、Driver进程历史、继电器当前状态与持久历史），独立终端原地刷新；数据面计数64位，长期连续连接不回绕
7. **畸形包硬化** — 拒绝非法 `data_type`；发布时**按每包自身类型**解析，堵住类型混用越界（ASan 实证过的内存破坏）
8. **零点洪泛防护** — 大丢包/掉线时限制零点回填，避免整片假点污染融合点云
9. **自动恢复看门狗（可选）** — 检测到假活（`Normal` 但**没有点云发布**）、配置长期不完成、`Error`（如电机故障）或显式唤醒后持续无广播时按各自路径恢复，带严格归因、重试上限和防死循环门禁
10. **持久化健康日志（可选）** — 把健康事件与网络趋势落盘成 CSV（边沿事件 + 周期快照），供长期无人值守的趋势分析与故障取证
11. **广播存活但握手卡死的识别与恢复** — 看板区分 `BROADCAST_ONLY / HANDSHAKE_STUCK / POWER_CYCLE_REQUIRED`；按单台雷达清理本地 session 并有限重试，仍失败时明确要求物理断电
12. **共享电源组硬恢复闭环（可选、默认关闭）** — 独立 ROS manager 严格处理 `HANDSHAKE_STUCK / WAKE_DROPOUT / NORMAL_DROPOUT / STARTUP_MISSING`；任一成员需要硬恢复时，共用通道的 4 台只断/上电一次，SQLite 持久化补上电义务、冷却与次数上限，并以 4 台全部持续恢复点云作为最终成功判据

> **整个分支必须配套固定版 SDK。** Driver 的异步 callback context 生命周期依赖 SDK 的 exactly-once completion/cancellation 契约；不能只为模式切换换 SDK、再让其他功能链接任意同名库。

---

## 快速开始

### 前置条件

- Ubuntu 20.04 + ROS Noetic
- Git（首次构建会在 build 目录获取固定版 Livox SDK）
- 不要预装或手工选择官方 SDK；CMake 会固定 fork、分支和精确 commit

### 中国现场：Geph 一键更新 SDK + Driver

仓库根目录提供 `update_livox_geph.sh`。脚本把“最新版本”定义为 GitHub 定制分支的最新 commit SHA，而不是一直不变的 SDK `2.3.0` 字符串。所有远程 Git 操作都显式使用 `socks5h://127.0.0.1:9909`，不会修改全局 Git 配置；开始前必须先启动 Geph。

#### 旧工位首次迁移（当前 SDK 和 Driver 都是旧版本）

这是当前维护的完整迁移流程（2026-07-29），适用于“机器仍运行旧 SDK、旧 Driver，但必须保留现有工位广播码、话题 remap、距离参数和继电器参数”的工位。不要提前删除 SDK/Driver，也不要先 checkout 新分支。脚本默认同时跟踪 SDK 与 Driver 的 `network-relay-added` 分支；迁移前先分别检查已有的 SDK 和 Driver 工作树，不要跳过。如果 `$HOME/Livox-SDK/.git` 不存在，则跳过第一条 SDK 检查，更新脚本会通过 Geph 自动 clone：

```bash
git -C "$HOME/Livox-SDK" status --short
```

```bash
git -C "$HOME/catkin_ws/src/livox_ros_driver" status --short
```

如果 SDK 显示已经确认来源、可以暂时移出的 tracked 本地修改，先用下面这一条命令同时保存二进制 patch 和 Git stash，再确认最后的状态输出为空。该命令不处理 untracked 文件、本地 commit 或分支分叉。迁移完成后不要直接执行 `git stash pop`，因为旧 SDK 修改可能重新引入已修复问题或破坏 Driver/SDK 的配套契约；需要恢复时应对照备份 patch 逐项审阅：

```bash
mkdir -p "$HOME/livox-migration-backup" && ts=$(date +%Y%m%d-%H%M%S) && git -C "$HOME/Livox-SDK" diff --binary HEAD > "$HOME/livox-migration-backup/Livox-SDK-$ts.patch" && git -C "$HOME/Livox-SDK" stash push -m "pre-network-relay-added-$ts" && git -C "$HOME/Livox-SDK" status --short
```

Driver 只允许以下两个现场文件保留**未暂存**修改，不要手工 stash；迁移命令中的 `--preserve-site-config` 会负责持久备份、更新和恢复：

- `livox_ros_driver/config/livox_lidar_config_multi.json`
- `livox_ros_driver/launch/livox_lidar_multi.launch`

如果这两个文件已经 staged，只取消暂存，不撤销文件内容：

```bash
git -C "$HOME/catkin_ws/src/livox_ros_driver" restore --staged -- livox_ros_driver/config/livox_lidar_config_multi.json livox_ros_driver/launch/livox_lidar_multi.launch
```

如果 SDK 或 Driver 还有其他 tracked 修改、本地未推送 commit、分支分叉，或者来源不明的 untracked 源码/CMake 文件，必须停止迁移并先审阅；不要用 `git reset --hard`、不要删除仓库，也不要为了绕过检查而 stash Driver 现场配置。

旧工位不要把下载、配置和重启压缩成一条无法停顿的命令。先完整读完阶段1～7和下方现场参数表，再从阶段1开始；每一步成功后才执行下一步。所有可能失败的复合命令都运行在子 shell 中，不会因为内部失败关闭当前 Terminal。

##### 路径规则（先读，避免创建错误文件）

| 用途 | 正确路径 | 是否人工修改 |
|---|---|---|
| Driver 仓库根目录 | `~/catkin_ws/src/livox_ros_driver` | 否 |
| 雷达白名单 | `~/catkin_ws/src/livox_ros_driver/livox_ros_driver/config/livox_lidar_config_multi.json` | 是 |
| 工位 multi launch | `~/catkin_ws/src/livox_ros_driver/livox_ros_driver/launch/livox_lidar_multi.launch` | 是 |
| 固定继电器 child launch | `~/catkin_ws/src/livox_ros_driver/livox_ros_driver/launch/livox_power_cycle.launch` | 否 |
| 生产继电器配置 | `~/.config/livox/power_cycle.json` | 是，且只允许在仓库外 |
| 继电器示例模板 | `~/catkin_ws/src/livox_ros_driver/livox_ros_driver/config/livox_power_cycle.example.json` | 否；文件名中是 `.example.json`，不是 `_example.json` |
| 内部 manager 源码 | `~/catkin_ws/src/livox_ros_driver/livox_ros_driver/livox_ros_driver/scripts/livox_power_cycle_manager.py` | 否；只有排障时才直接使用 |
| 统一校验入口 | `~/catkin_ws/src/livox_ros_driver/validate_livox_site.sh` | 否；现场校验只运行它 |

绝对不要创建 `livox_ros_driver/config/livox_power_cycle.json`，不要把示例模板重命名为生产文件，也不要在阶段1完成之前运行内部 manager；旧版本没有这些新文件属于正常现象。

**阶段1：开启 Geph，只准备新版，不重启旧 Driver。** 先完成本节前面的 SDK/Driver 工作树检查；SDK必须干净，Driver只允许白名单JSON和multi launch存在未暂存修改。然后以普通用户执行下面这一条。它通过代理取得最新版脚本，配套更新/安装SDK，更新并编译Driver，安全合入继电器launch入口，生成缺失的仓库外生产JSON并安装systemd安全钩子；没有 `--restart-service`，因此内存中的旧Driver继续运行：

```bash
( set -Eeuo pipefail; repo="$HOME/catkin_ws/src/livox_ros_driver"; if [ -d "$HOME/Livox-SDK/.git" ]; then git -C "$HOME/Livox-SDK" remote set-url origin https://github.com/85256638/Livox-SDK.git; fi; git -C "$repo" remote set-url origin https://github.com/85256638/livox_ros_driver.git; git -c http.proxy=socks5h://127.0.0.1:9909 -c https.proxy=socks5h://127.0.0.1:9909 -C "$repo" fetch origin "refs/heads/network-relay-added:refs/remotes/origin/network-relay-added"; git -C "$repo" show "origin/network-relay-added:update_livox_geph.sh" > /tmp/update_livox_geph.sh; LIVOX_JOBS=2 bash /tmp/update_livox_geph.sh --preserve-site-config; bash "$repo/install_livox_power_cycle_service.sh" )
```

**阶段2：确认新文件确实存在。** 下面这一条只检查文件和Git状态，不修改任何内容；必须看到6项 `OK`。最后的Git状态通常只允许显示白名单JSON和multi launch两项 `M`：

```bash
( set -Eeuo pipefail; repo="$HOME/catkin_ws/src/livox_ros_driver"; for file in "$repo/validate_livox_site.sh" "$repo/livox_ros_driver/livox_ros_driver/scripts/livox_power_cycle_manager.py" "$repo/livox_ros_driver/livox_ros_driver/scripts/validate_livox_power_cycle_site.py" "$repo/livox_ros_driver/config/livox_power_cycle.example.json" "$repo/livox_ros_driver/launch/livox_power_cycle.launch" "$HOME/.config/livox/power_cycle.json"; do if [ ! -f "$file" ] || [ -L "$file" ]; then echo "ERROR: 新版必要文件缺失或是符号链接：$file" >&2; false; fi; echo "OK $file"; done; if [ -e "$repo/livox_ros_driver/config/livox_power_cycle.json" ]; then echo "ERROR: 生产配置误放在Git仓库内；正确位置是 $HOME/.config/livox/power_cycle.json" >&2; false; fi; git -C "$repo" status --short )
```

**阶段3：人工修改三份现场配置。** 使用下方“迁移后必须人工核对的现场配置”表格逐项修改。此时新二进制尚未重启，正在运行的旧进程不会读取这些新值。先让外部JSON的目标电源组 `enabled=true`，但让multi launch的 `relay_power_cycle_enable=false`；这样可以校验身份和只读查询继电器，而不会启动自动断电manager。

```bash
nano "$HOME/catkin_ws/src/livox_ros_driver/livox_ros_driver/config/livox_lidar_config_multi.json"
```

```bash
nano "$HOME/catkin_ws/src/livox_ros_driver/livox_ros_driver/launch/livox_lidar_multi.launch"
```

```bash
nano "$HOME/.config/livox/power_cycle.json"
```

**阶段4：离线校验。** 该入口不访问继电器、不改变任何输出，也不会重启服务。目标输出应包含 `Configuration valid`，并且 `Site identity valid` 中应为 `launch_armed=false driver=4 relay_enabled=4 remaps=4`：

```bash
bash "$HOME/catkin_ws/src/livox_ros_driver/validate_livox_site.sh"
```

**阶段5：只读查询真实继电器。** 仍保持 `relay_power_cycle_enable=false`。下面的命令会先重复离线校验，再仅对每个enabled电源组执行B0状态查询；它不会发送OFF/ON。必须确认IP/端口可达、返回4路状态。1号工位CX-5104E-L已实测每个新TCP连接的第一次命令只返回ASCII `v1.0`，同一连接重发一次后才返回正式帧；B0既可能拆成8+1字节，也可能在正确第一校验字节后完全省略第9字节。manager会先短暂等待TCP尾分片，仍没有时才严格验证8字节地址、结束位、四路掩码和第一校验字节。输出中的 `v1.0 handshake`、`fixed AA tail` 或 `single-checksum ... omitted second checksum byte` 兼容警告都是已确认的正常行为：

```bash
bash "$HOME/catkin_ws/src/livox_ros_driver/validate_livox_site.sh" --check-relays
```

**阶段6：武装并再次离线校验。** 本现场已确认继电器1～4路全部只控制本组4台雷达，因此生产JSON必须使用 `channels: [1, 2, 3, 4]`。只有人工确认所配通道集合不含其他负载后，才把multi launch中的 `relay_power_cycle_enable` 改为 `true`；再次运行校验，目标输出必须变为 `launch_armed=true driver=4 relay_enabled=4 remaps=4`：

```bash
nano "$HOME/catkin_ws/src/livox_ros_driver/livox_ros_driver/launch/livox_lidar_multi.launch" && bash "$HOME/catkin_ws/src/livox_ros_driver/validate_livox_site.sh"
```

**阶段7：保持 Geph 开启，第一次切换到配置好的新版。** 版本未变化时会跳过重复编译，但仍会安全备份/恢复两份现场文件，并再次校验继电器身份、child launch和systemd安全钩子；全部成功后才重启。任何更新、编译或校验失败都不会执行这次重启：

```bash
LIVOX_JOBS=2 bash "$HOME/catkin_ws/src/livox_ros_driver/update_livox_geph.sh" --preserve-site-config --restart-service
```

##### 迁移后必须人工核对的现场配置

代码更新不能推断本工位雷达身份、ROS 话题命名和继电器物理接线。旧工位原本正确的前两份文件会由 `--preserve-site-config` 自动恢复或安全合并，**不要用仓库示例覆盖它们**；迁移成功后仍必须逐项核对：

| 文件 | 必须人工核对/修改的内容 | 更新脚本的处理 |
|---|---|---|
| `~/catkin_ws/src/livox_ros_driver/livox_ros_driver/config/livox_lidar_config_multi.json` | `lidar_config` 只保留本工位实际使用的 4 个完整 `broadcast_code`，均设置正确的 `enable_connect`；同时核对 `return_mode`、坐标系、IMU 频率和外参来源 | 检测到本地修改时字节级原样恢复，不会替换广播码 |
| `~/catkin_ws/src/livox_ros_driver/livox_ros_driver/launch/livox_lidar_multi.launch` | 核对4台雷达的 lidar/IMU/status `remap`（广播码必须与 JSON 一致）、`config_file` 和 `max_distance`；生产恢复策略应使 `auto_recover=true`、`health_log=true`，无桌面/systemd 环境应使 `monitor=false`（也可由 `ExecStart` 参数覆盖）。不要手工复制或重复添加继电器 marker/include | 保留现场参数，并安全合入新版唯一继电器入口；无法明确合并时停止而不重启 |
| `~/.config/livox/power_cycle.json` | `members` 必须与Driver白名单完全相同且恰好4个；`protocol=legacy_tcp`；`host` 是本工位继电器IP而不是雷达IP；实机默认 `port=50000`、协议 `address=1`；本现场明确填写 `channels: [1, 2, 3, 4]`，四路作为一个不可拆分电源组通过同一A1掩码动作。真正只用一路的旧现场仍可兼容 `channel: 1..4`，但同一组不能同时出现 `channel` 和 `channels`；已确认的“正确CH+固定AA尾字节”保持 `allow_omitted_status_checksum=false`；`policy.off_seconds=5`；只在准备进入只读实机查询/最终武装时设该组 `enabled=true` | 安装脚本仅在缺失时从示例生成；以后更新永不覆盖。不要修改 `state_db` 和 ROS topics，除非同步审阅全部 service 参数 |
| `/etc/systemd/system/livox-ros-driver.service` | 仅当现有 unit 的用户名、`HOME`、catkin workspace 或 launch 命令本来就不正确时才人工修改；正在正常启动该工位旧 Driver 的 unit 通常无需改 | 安装脚本只安装安全 drop-in 并验证主 unit，不会猜测或重写现场 `ExecStart` |

本现场外部JSON的 `relay` 对象必须是下面的结构；如果旧文件已有 `"channel": 1`，删除该行并改成 `"channels": [1, 2, 3, 4]`，不能保留两个字段：

```json
"relay": {
  "protocol": "legacy_tcp",
  "host": "本工位继电器IP",
  "port": 50000,
  "channels": [1, 2, 3, 4],
  "address": 1,
  "allow_omitted_status_checksum": false
}
```

如果暂不启用继电器自动硬恢复，只需核对前两份 Driver 配置，并让外部JSON电源组和launch继电器开关都保持 `false`；systemd 主 unit 正常时也不用修改。准备执行只读B0实机查询时，可以先把外部JSON目标组设为 `enabled: true`，但必须继续保持 launch 中 `<arg name="relay_power_cycle_enable" default="false"/>`；组enabled本身不会启动manager，launch才是自动硬件控制总开关。完成广播码、继电器地址/通道集合及“所选通道全部只给本组4台雷达供电”的人工验收后，才把launch开关也改为 `true`。SDK源码、CMake文件、`livox_power_cycle.example.json` 和SQLite状态库都不是现场配置，不要手改或用示例文件覆盖生产文件。

修改完成后统一运行根目录入口；不要再手工输入内部Python脚本的三层目录。只有输出 `Configuration valid` 和 `Site identity valid` 才允许进入后续步骤：

```bash
bash "$HOME/catkin_ws/src/livox_ros_driver/validate_livox_site.sh"
```

迁移命令全部成功后，用下面这一条核对 SDK 源码、Driver 源码和服务状态；最后一项应输出 `active`：

```bash
git -C "$HOME/Livox-SDK" rev-parse --short=12 HEAD && git -C "$HOME/catkin_ws/src/livox_ros_driver" rev-parse --short=12 HEAD && systemctl is-active livox-ros-driver
```

随后打开看板，以顶部 `==================== SOFTWARE ====================` 板块为最终生效依据：`Driver commit` 必须等于上一步 Driver 源码 commit，`paired SDK commit` 是该 Driver 编译时固定配套的 SDK。这里读取的是运行二进制内嵌版本，不会把“磁盘源码已更新但服务仍运行旧二进制”误报为新版：

```bash
source "$HOME/catkin_ws/devel/setup.bash" && rosrun livox_ros_driver livox_stats_monitor.py
```

#### 已完成继电器版部署后的日常升级（以后使用这里）

只要工位已经完成前述七阶段迁移、`validate_livox_site.sh --check-relays` 已通过，以后升级 **不需要重新执行七阶段迁移，也不需要重新填写三份现场配置**。先开启 Geph，然后在维护窗口运行下面这一条推荐命令：它保留两份仓库内现场配置，更新并配套编译SDK/Driver，刷新幂等的systemd安全helper，全部成功后才重启服务；`~/.config/livox/power_cycle.json` 位于仓库外，始终不会被更新覆盖：

```bash
LIVOX_JOBS=2 bash "$HOME/catkin_ws/src/livox_ros_driver/update_livox_geph.sh" --preserve-site-config && bash "$HOME/catkin_ws/src/livox_ros_driver/install_livox_power_cycle_service.sh" && LIVOX_JOBS=2 bash "$HOME/catkin_ws/src/livox_ros_driver/update_livox_geph.sh" --preserve-site-config --restart-service
```

如果只想先下载、更新和编译，暂时让内存中的旧Driver继续运行，使用这一条：

```bash
LIVOX_JOBS=2 bash "$HOME/catkin_ws/src/livox_ros_driver/update_livox_geph.sh" --preserve-site-config
```

上述准备命令成功后，决定应用磁盘上的新版时再运行这一条；它先刷新安全helper，再复核版本、配置和systemd门禁，最后重启：

```bash
bash "$HOME/catkin_ws/src/livox_ros_driver/install_livox_power_cycle_service.sh" && LIVOX_JOBS=2 bash "$HOME/catkin_ws/src/livox_ros_driver/update_livox_geph.sh" --preserve-site-config --restart-service
```

重复运行日常升级命令是安全的：远端版本没有变化时跳过重复构建；安装脚本是幂等操作；只有最后带 `--restart-service` 的步骤会造成一次短暂断流。若任一步失败，后续 `&&` 步骤不会执行，当前旧进程不会被更新脚本主动重启。升级完成后以看板 `SOFTWARE` 中运行二进制内嵌的Driver/SDK版本为最终依据。

#### 全新工位（尚无Driver仓库）

全新工位尚无 Driver 仓库时，使用这一条完成代理 clone 和首次配套构建；首次部署服务前不自动重启。之后仍需执行前面的七阶段现场配置与验收，不能直接武装继电器：

```bash
mkdir -p "$HOME/catkin_ws/src" && git -c http.proxy=socks5h://127.0.0.1:9909 -c https.proxy=socks5h://127.0.0.1:9909 clone --branch network-relay-added --single-branch https://github.com/85256638/livox_ros_driver.git "$HOME/catkin_ws/src/livox_ros_driver" && LIVOX_JOBS=2 bash "$HOME/catkin_ws/src/livox_ros_driver/update_livox_geph.sh" --preserve-site-config
```

`--preserve-site-config` 只允许自动处理以下两个现场文件：

- `livox_ros_driver/config/livox_lidar_config_multi.json`
- `livox_ros_driver/launch/livox_lidar_multi.launch`

脚本会先把工位原文件、更新前仓库版本和差异持久备份到 `~/.local/state/livox-stack-updater/site-config-backups/`，短暂暂存工位修改，再 fast-forward Driver。多雷达 JSON 会字节级原样恢复；multi launch 优先用“现场原文件 / 更新前 HEAD / 更新后上游”做三方合并，使现场参数和新版继电器 include 同时保留。如果文本合并仅因现场参数与新版插入区域重叠而冲突，脚本会以现场 launch 为主体，使用严格结构化后备合并，只注入安全默认 `false` 的唯一开关和位于 `livox_driver` 之前的固定 include；不会重排或重写现场其余内容。只有XML合法、没有旧/重复 manager、`LIVOX_RELAY_LAUNCH_INTEGRATION` 唯一、include 精确透传开关且 child launch 仍是固定路径的 armed-only 单节点结构时才继续编译；任何身份不明确的结构仍恢复现场旧 launch、保留候选文件并禁止编译和重启。其他任何 tracked 本地修改仍会使更新停止。若进程中断，下次运行会先恢复未完成的配置事务。该选项只接管未暂存修改；若文件已 staged，脚本会停止并要求先取消暂存。继电器现场配置位于仓库外的 `~/.config/livox/power_cycle.json`，更新天然不会覆盖，不需要加入保留列表。

如果旧 SDK 或 Driver 最初使用 `--single-branch` 克隆，脚本会只为当前目标分支补充缺失的 `origin` fetch refspec；如果上一次迁移恰好停在“本地目标分支已创建、但 upstream 尚未设置”，再次运行也会自动修复跟踪关系后继续 fast-forward，不需要删除仓库、分支或现场配置。

已经用“不重启”命令准备成功后，立即应用新二进制：

```bash
bash "$HOME/catkin_ws/src/livox_ros_driver/install_livox_power_cycle_service.sh" && LIVOX_JOBS=2 bash "$HOME/catkin_ws/src/livox_ros_driver/update_livox_geph.sh" --preserve-site-config --restart-service
```

`--restart-service` 不是另一种启动方式；它只是在全部更新和编译成功后重启 `livox-ros-driver`。集成版 manager 由同一个 launch 管理，不再单独启动或重启。不带该参数时，只有在集成版安全钩子已经安装并通过检查后，才可手动重启：

```bash
sudo systemctl restart livox-ros-driver
```

如果系统仍加载、启用或运行旧的独立 `livox-power-cycle-manager.service`，新版更新器会拒绝 `--restart-service`，绝不会让旧 manager 与 launch manager 并发。此时先运行后文的新版安装脚本；它会安全停止旧 unit、按 SQLite 补 ON、禁用并删除旧 unit，再给 `livox-ros-driver.service` 安装配置无关的启动前/停止后补 ON 钩子。安全 drop-in 未加载时也一律拒绝重启，即使 launch 默认是 `false`；这样既不会遗漏历史 SQLite 补 ON 义务，也不会被 systemd 的额外 roslaunch 参数绕过默认开关。

版本未变化时脚本会跳过重复构建；需要强制重编译时执行：

```bash
LIVOX_JOBS=2 bash "$HOME/catkin_ws/src/livox_ros_driver/update_livox_geph.sh" --preserve-site-config --force
```

生产运行期间建议降低并行数，减少编译对点云接收的影响：

```bash
LIVOX_JOBS=2 bash "$HOME/catkin_ws/src/livox_ros_driver/update_livox_geph.sh" --preserve-site-config
```

#### Driver 正在运行时会发生什么

- **不带 `--restart-service`**：当前 Driver 不会停止，仍运行内存中的旧版代码；源码和磁盘上的二进制完成更新后，需要手动重启才会生效。若旧独立 manager unit 尚未迁移，必须先运行新版安装脚本，不能直接重启到集成版。
- **带 `--restart-service`**：更新和编译期间旧进程继续运行；只有全部成功且旧 unit/安全钩子检查通过后才重启 Driver，此时会短暂断流并重新握手连接雷达；launch 开关为 `true` 时 manager 随 Driver 一起启动。
- **资源影响**：编译会占用 CPU、内存和磁盘 I/O，负载较高时可能增加点云丢包；生产机器建议使用 `LIVOX_JOBS=2`，并在维护窗口重启。
- **失败处理**：更新或编译失败时脚本不会主动重启，当前旧进程通常仍可继续运行；在重新编译成功前不要主动重启服务或主机，因为磁盘上的新二进制可能尚未完整生成。

> 安全策略：SDK 和 Driver 只允许 fast-forward；默认遇到任何 tracked 本地修改都会停止。只有显式添加 `--preserve-site-config` 时，上述两份工位文件才允许自动备份和恢复；其他修改、本地未推送 commit、分支分叉或 SDK/Driver 尚未形成配套版本仍会停止。脚本不会执行 `reset --hard` 或删除用户文件，Driver 使用本地配套 SDK 编译，CMake 不会自行无代理访问 GitHub。

### 编译

```bash
cd ~/catkin_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

> - 加 `-DPYTHON_EXECUTABLE=/usr/bin/python3` 是**强制 catkin 用系统 python3**，避免 conda 等环境让它选错 python（否则编译或运行报 python 相关错）。比 `conda deactivate` 更稳，不受当前环境影响。
> - 首次构建会克隆 `85256638/Livox-SDK` 的配套分支并检出固定 SHA；后续使用 build 目录缓存。不会链接 `/usr/local/lib` 中来源不明的同名库。
> - 离线构建可额外传 `-DLIVOX_SDK_SOURCE_DIR=/绝对路径/Livox-SDK`；该 checkout 必须是 README 下方列出的精确 SHA，且 tracked 文件无修改，否则 CMake 会 fail closed。
> - 新版 CMake（≥3.27）若报 policy 版本错，再补 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`。
> - ⚠️ **编译用的 `catkin_ws` 必须和下面 systemd 服务里 `source` 的是同一个目录**，否则你编译了、服务却跑的是另一份旧的，改动不生效还极难排查。

危险恢复判据的5个C++ QC和Python manager测试已接入CMake/CTest；需要验收时执行这一行：

```bash
cd "$HOME/catkin_ws" && catkin_make run_tests && catkin_test_results
```

### 启动

```bash
# 单雷达
roslaunch livox_ros_driver livox_lidar.launch

# 多雷达
roslaunch livox_ros_driver livox_lidar_multi.launch
```

> 上面是**手动启动**（在桌面上调试用，会自动弹看板）。**生产 24/7 无人值守请用下面的 systemd 服务**，不要手动 roslaunch。

### 生产部署（systemd：开机自启 + 崩溃自重启 + 无人值守自愈）

把驱动跑成系统服务，这样**断电通电后自动启动、进程崩溃后自动重启**，无需人工敲命令。

**① 服务文件** `/etc/systemd/system/livox-ros-driver.service`（把 `<USER>` 换成实际用户名）：

```ini
[Unit]
Description=Livox ROS Driver
After=network-online.target roscore.service
Wants=network-online.target
Wants=roscore.service

[Service]
Type=simple
User=<USER>
Group=<USER>
WorkingDirectory=/home/<USER>
Environment=HOME=/home/<USER>
Environment=ROS_MASTER_URI=http://localhost:11311
Environment=ROS_HOSTNAME=localhost
# ⚠️ 这里 source 的 catkin_ws 必须和你编译用的是同一个目录
ExecStart=/bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /home/<USER>/catkin_ws/devel/setup.bash && exec roslaunch livox_ros_driver livox_lidar_multi.launch monitor:=false auto_recover:=true health_log:=true'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**② 为什么 ExecStart 末尾要带这三个参数**（命令行 `arg:=value` 会覆盖 launch 文件的默认值，且贯穿到驱动）：

| 参数 | 生产值 | 原因 |
|------|--------|------|
| `monitor` | **`false`** | 后台服务**无图形界面**，自动弹 `gnome-terminal` 看板会弹不出来报错。想看看板时单独 `rosrun livox_ros_driver livox_stats_monitor.py` |
| `auto_recover` | **`true`** | 无人值守时雷达故障（假活 / `Error` / Config 卡死 / 握手卡死 / 显式唤醒掉广播）按各自路径恢复；只有原因特定证据完整时才升级物理断电 |
| `health_log` | **`true`** | 健康事件 + 网络趋势**落盘取证**，供事后排查 |

> 这三个值只对服务生效；你**手动 `roslaunch`** 时不带参数，仍是 `monitor:=true`（看板弹出）等默认值，两个场景各取所需、互不影响。

**③ 启用并启动**：

```bash
sudo systemctl daemon-reload
sudo systemctl enable livox-ros-driver     # 开机自启（光有 [Install] 还不够，必须 enable）
sudo systemctl restart livox-ros-driver
```

**④ 验证全部生效**：

```bash
# 自愈 + 日志开了没（应看到两条 ENABLED）
journalctl -u livox-ros-driver -b | grep -iE "Auto-recover|Health logging"
# 开机自启开了没（应显示 enabled）
systemctl is-enabled livox-ros-driver
```

> ⚠️ **`health_log` 的目录必须先存在**：若用了 `health_log_dir:=/some/path`，先 `mkdir -p /some/path`；否则首次写盘失败会**自动禁用日志**（驱动不受影响，但日志不写）。确认日志在写：`ls -la <日志目录>/`，应出现 `livox_events_YYYY-MM-DD.csv`（快照文件 `livox_snapshot_*` 要等第一个周期，默认 10 分钟）。

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

# 所有雷达批量切换（Normal 会错峰，不会同时下发）
rosservice call /livox_lidar_mode "{handle: 255, mode: 1}"
```

### 参数说明

| 参数 | 取值 | 说明 |
|------|------|------|
| `handle` | 0~31 | 单个雷达的设备句柄（启动日志中 `Lidar[X]` 的 X 即为 handle）|
| `handle` | 255 | 逻辑批量模式，对调用时真正已连接的雷达生效；切到 Normal 时按 0/2/4/6 秒错峰下发 |
| `mode` | 1 | Normal — 正常工作，电机旋转，输出点云 |
| `mode` | 2 | PowerSaving — 节电模式，电机停转 |
| `mode` | 3 | Standby — 待机模式，电机停转 |

### 返回值

| `ret_code` | 含义 |
|------------|------|
| 0 | 请求已接受 |
| 非 0 | 错误（详见终端日志）|

> ⚠️ `ret_code = 0` 只表示请求已被驱动/SDK 同步接受（也可能表示设备已在目标态，或断线中的 Normal 请求已排队等重连），不是雷达的异步 ACK，更不代表模式一定切成。后续由 callback 和真实 heartbeat state 完成校验。

### 模式校验、错峰与有界重试

驱动以雷达的**真实 heartbeat `state`**判断切换是否完成，不把 SDK 同步返回或异步 ACK 当成最终成功。两类命令使用不同节奏：

| 目标 | 首次下发 | ACK/观察 | 仍未到目标态 |
|------|----------|----------|----------------|
| PowerSaving / Standby | 单台立即下发 | 每秒核对真实状态 | 2 秒后定向重发，最多 3 次 |
| Normal（单台）| 立即下发 | 收到 accepted / spinning-up ACK 后 **20 秒内不重发** | 20 秒后仍未到 Normal，最多再发 2 次，间隔 5 秒 |
| Normal（`handle:255`）| 该批次前 4 台按 **0/2/4/6 秒**下发 | 每台独立使用上述 20 秒观察期 | 每台独立使用上述 2 次、5 秒间隔的上限 |

同一批次唤醒后，**每台雷达内部**的坐标系、回波模式、IMU、外参和启采样等配置命令串行下发：前一项收到终态 callback（成功、超时或失败）后才发下一项，不再对同一台并发整组配置。这与四台首次模式命令错峰共同降低命令通道的瞬时峰值。

> 仍切不成的极端情况：日志打印 `did not enter mode[..] after N retries -- manual check needed`。休眠/待机最多重发 3 次；Normal 若已收到 accepted/spinning-up ACK，20 秒 grace 后最多重发 2 次。若始终没有收到正向 Normal ACK，则按 2 秒节奏最多重发 7 次（连同首次发送最多 8 次）；每一笔仍必须先等配套 SDK 给出终态 callback，绝不会并发叠加命令。主表 `ASSESS` 会按最近 10 分钟内的发生次数显示 `WATCH/UNSTABLE`，底部 `PROCESS HISTORY` 保留失败总数、最后目标模式和时间。

### 模式命令的其他保障

- **广播只发给"当前真正连着"的雷达**：`handle:255` 不再给 4~31 号不存在的 handle 排队请求（旧行为会留下"陈旧的 Normal 请求"，等以后哪台雷达占了那个 handle 就被误命令）。一台雷达都没连时 `ret_code` 返回未连接而不是假成功。
- **迟到的 Normal 状态事件不会取消新的休眠请求**：雷达的状态事件可能因健康位变化而重复上报；旧逻辑一收到 Normal 就把当前模式请求清掉——若你刚发完唤醒又紧接着发休眠（如调度器两个条件先后触发），迟到的 Normal 事件会把休眠请求删掉、校验重试也随之失效。现在只有"目标就是 Normal"的请求才会被 Normal 事件完成。
- **旧 ACK 不会改写新请求**：每个 logical request、每次 send attempt 和每次连接都有独立 token；迟到 ACK 只有三者都匹配才可更新状态。每个 handle 的“发布请求→SDK enqueue”也串行，避免软件状态虽能识别旧 ACK、硬件却先收到新命令再收到旧命令。
- **低功耗命令有安全准入条件**：新的 PowerSaving / Standby 只在设备处于 `Sampling + Normal` 时接受；已在目标低功耗状态则幂等返回成功、不重发。启动配置期、错误态或存在相反请求时会同步返回失败，调度器应稍后重试。
- **调度器仍不应反复打断当前批次**：`handle:255` 已负责错峰与配置串行，上层不必手工逐台唤醒；但仍应避免在上一条模式命令完成前发送**相反**命令。若 service 返回非 0，等待当前配置/转换结束后重试，不要高频来回命令固件。

### 断线行为

| 场景 | 行为 |
|------|------|
| 已连续 `Normal + Sampling + publishing` 30 秒后断线 | 先确认广播连续消失 5 秒；期间重连立即取消，广播返回则需至少 3 帧并跨 3 秒才移交握手恢复；仍无广播且 `auto_recover=true` 时进入 `POWER_CYCLE_REQUIRED reason=NORMAL_DROPOUT` |
| PowerSaving / Standby 下断线 | 15 秒检测到，重连后恢复 Normal 模式 |
| 切换 Normal 时通信失败 | 自动等待重连后重试 |
| 显式从 PowerSaving / Standby 唤醒后掉线且广播持续消失 | 仅在同一 broadcast code + connection generation 的 60 秒唤醒观察窗内归因；持续无广播 10 秒后进入 `WAKE_DROPOUT` |
| Driver 启动后白名单成员始终无连接、无新鲜广播且从未健康发布 | 30 秒启动宽限后显示 `STARTUP_MISSING`；用合成 `handle=255` 持续发布恢复状态，`auto_recover=true` 时进入 `POWER_CYCLE_REQUIRED reason=STARTUP_MISSING` |
| 未满足上述严格证据的瞬时/身份不明断线 | 保持 `DISCONNECTED` 并等待状态归属；不会仅凭一次 disconnect callback 触发继电器 |

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

> ⚠️ 该 launch 参数目前只在 **`livox_lidar_multi.launch`** 里接了线（单雷达 `livox_lidar.launch` 没有这个 arg，传了会报 unused argument）。

```bash
# 只发布 5 米以内的点
roslaunch livox_ros_driver livox_lidar_multi.launch max_distance:=5.0

# 禁用过滤，发布所有点
roslaunch livox_ros_driver livox_lidar_multi.launch max_distance:=0
```

也可在 launch 文件中修改默认值：

```xml
<arg name="max_distance" default="25.0"/>
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_distance` | double | **25.0**（multi launch 的 default）| 最大发布距离（米），0 表示禁用过滤 |

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

> 以下问题的主要修复位于 ROS Driver；但整个定制分支仍必须链接上文固定 SDK，才能满足异步 context 的完成/取消生命周期契约。

### 1. 掉线崩溃（use-after-free）修复

**官方 bug**：雷达掉线时，`ResetLidar` 在 SDK 设备状态线程上释放数据队列，而 SDK 数据接收线程仍可能往同一队列写入——两者无任何锁同步，导致 **use-after-free / 堆损坏**，在多雷达偶发掉线时崩溃或话题假死。

**修复**：为每台雷达引入一把 `std::mutex`，把**写入（StorageRawPacket）/ 读取（DistributeLidarData）/ 释放（ResetLidar）** 三条路径互斥；并在 `DeInitQueue` 释放后置空指针、各队列操作加空指针兜底。从根上消除竞态（区别于裸 null 检查的临时补丁）。

### 2. 状态抖动导致话题断流修复

**问题**：早期版本在雷达状态从「任意非 Normal → Normal」时都会重置 `connect_state` 重跑配置，于是**温度/电机告警等瞬时 Error→Normal 抖动**也会触发完整重配置 → 话题断流几百 ms。工业现场高温、震动环境下频繁发生。

**修复**：仅在「确实从节电/待机恢复」或「我们主动请求的 Normal 切换正在完成」时才重配置，瞬时告警抖动不再打断已在采样的雷达。

### 3. 零点洪泛防护（大丢包/掉线时）

**官方行为**：检测到时间戳缺口（丢包或短暂掉线）时，驱动会用**零点包**（点全在原点 0,0,0）回填以保持时间戳连续。但回填**无上限**——长掉线或重丢包时会把整个发布预算耗在零点包上，导致下游连续多帧收到整片原点假点，污染多雷达融合点云、浪费 CPU/带宽，还会掩盖真正在退化的雷达。

**修复**：每帧点云的零点回填**最多 10 个包**（`kMaxZeroFillPacketPerMsg`），到上限即停止补零、转去处理真实包并重同步时间戳。三处发布路径（`PublishPointcloud2` / `PublishPointcloudData` / `PublishCustomPointcloud`）一致生效。正常无缺口时计数恒为 0、**行为完全不变**；只在病态丢包下从"无底洞灌假点"变成"补几个就回到真数据"。保留了小丢包（1~2 包）补零以维持时间戳连续的合理用途。

### 4. 数据类型混用越界修复（内存安全）

**官方行为**：发布器用"设备**最新**的 data_type"去解析队列里的**所有**包。但一次回波模式/坐标系重配置（首次连接、休眠唤醒、重连都会触发）之后，队列里可能还残留**旧类型**的包——用新类型的解析器去读旧包，步长就是错的，会越界读写（AddressSanitizer 实测可复现：单回波包被按三回波解析时，向 2KB 栈缓冲写入超过 5KB，足以崩溃或静默破坏内存）。

**修复**：三处发布路径全部改为**按每个包自带的 `data_type`** 选解析器和回波数（包的点数本来就是入队时按包自身类型记录的）。同构数据流（正常情况）行为完全一致；混流时每个包都按自己的真实格式解析，越界在构造上不可能发生。

### 5. 累计计数 64 位化（防 ~20 天回绕）

收包/丢包/队列丢弃累计计数原为 32 位——Horizon 单回波速率下约 **19.9 天**就会回绕，导致日志与 CSV 统计突跳失真。全部改为 64 位，并新增 `published`（真正发布出去的包数——"收到了多少"和"发出去了多少"从此可分开审计）。

> **计数边界必须分清：**这些点云计数是**当前 SDK 连接生命周期内累计**，雷达断线执行 `ResetLidar` 后会清零，并不是 Driver 进程从启动到现在的永久累计。新版看板因此不再把旧 `loss%` 当长期指标，而是在 `RECENT 60 SECONDS` 中显示能识别计数器清零的 `packet_loss`。真正的 Driver 进程历史只放在底部 `PROCESS HISTORY`，并在 Driver 重启时归零。

---

## 新增功能四：丢包可视化

提供两种查看方式，按需选用。

### 方式 A：日志告警（仅明显异常时输出）

驱动每 5 秒检查一次，**只有在该窗口内丢包达到一定程度时才打印一行**，健康运行时日志保持干净：

```
[LivoxStats][WARN] Lidar[0][1PQDH5B00100041] 5s: recv=12480 net_loss=80(0.64%) queue_drop=3(0.02%) | total recv=998400 net_loss=152 drop=10 published=998390
```

触发条件：**窗口网络丢包率 ≥ 0.5%**，或**出现任何队列丢包**（消费跟不上，总是值得知道）。

> ⚠️ 早期版本只要丢 1 个包（0.01%）就报 WARN，导致 UDP 正常抖动也刷屏、看着像出问题。现在提高了门槛：偶发的 1~2 个包丢失（~0.01%）属于正常抖动，**不再打印 WARN**；这些包仍会进入看板 `RECENT 60 SECONDS / packet_loss`，离开窗口后自动消失。周/月趋势应使用后文的持久化健康日志，不能把实时看板当永久累计。

| 字段 | 含义 | 指向 |
|------|------|------|
| `recv` | 最近 5 秒收到的点云包数 | 速率是否稳定 |
| `net_loss` | **网络丢包**（包未到达驱动，按时间戳间隔估算）| 网线 / 交换机 / 雷达硬件 / 散热 |
| `queue_drop` | **队列丢包**（驱动消费不过来）| 下游订阅者慢 / CPU 瓶颈 |
| `total ...` | 当前连接生命周期内累计；断线重建对象后清零 | 当前连接审计，不能跨重连直接相减 |

### 方式 B：实时看板（独立终端，原地刷新，互不干扰）⭐推荐

驱动每秒发布 `livox/lidar_stats` topic。在**另一个终端**运行看板脚本，它会原地刷新（像 `htop`），永远显示当前值，且与驱动日志完全隔离。

#### 用法一：直接启动，看板自动弹出（默认行为）⭐最省事

```bash
roslaunch livox_ros_driver livox_lidar_multi.launch
```
看板默认开启（`monitor` 参数默认 `true`），驱动日志留在当前终端，看板会**自动弹出一个独立窗口**原地刷新，两者互不干扰。

> 需要桌面环境（gnome-terminal + X11）。**无显示器/纯 SSH 的机器**请关掉它，否则会因弹不出窗口报错：
> ```bash
> roslaunch livox_ros_driver livox_lidar_multi.launch monitor:=false
> ```
> 然后用下面的用法二手动开看板。

#### 用法二：手动两个终端（无桌面环境用这个）

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

看板按固定层次显示：数据源存活、运行二进制版本、当前告警、当前逐台状态及掉线次数、最近60秒滚动指标、判定说明、Driver进程历史、继电器当前状态和最近5次持久操作历史。普通瞬时掉线标 `DISCONNECTED`，已归因的运行期掉线标 `NORMAL_NO_BROADCAST / NORMAL_DROPOUT`，启动缺失标 `STARTUP_MISSING`，显式唤醒归因则标 `WAKE_NO_BROADCAST / WAKE_DROPOUT`。白名单成员即使从未取得 SDK handle，也会以合成 `L255` 行出现：
```
==================== DATA SOURCE ====================
  DRIVER   NOW=LIVE  severity=INFO  driver_age=0s  expected=1Hz stale>5s
  LIVE=realtime; DRIVER_STALE=the sections below are the last snapshot
  POWER-MGR NOW=MANAGER_HEARTBEAT  severity=INFO  manager_age=2s  heartbeat=10s stale>30s

===== Livox LiDAR Status (1 Hz) =====
==================== SOFTWARE ====================
  (versions embedded in this running binary)
  Driver commit=<12-char SHA> ROS=2.6.0 | paired SDK commit=e45774c5d4f2 SDK=2.3.0 | compatibility=PINNED
==================== CURRENT ALERTS ==============
  [CRIT] L1 3WEDH5900100671 POWER_CYCLE_REQUIRED reason=HANDSHAKE_STUCK age=12s
    handshake: broadcast=alive; reset=completed; power-cycle request published; see POWER RECOVERY manager
    last SDK event: TIMEOUT; detail=500
==================== CURRENT DEVICES =============
ID  broadcast_code   CURRENT               ASSESS     points/s  HW            connected    disc
0   3WEDH7600111191  NORMAL                STABLE         2496  OK                2h13m       0
1   3WEDH5900100671  POWER_CYCLE_REQUIRED  ACTIVE            -  -                    --       3
2   3WEDJA700100021  NORMAL                UNSTABLE       2498  OK                8m05s       1
3   3WEDH7600103661  POWER_SAVING          IDLE               0  OK                2h13m       0
==================== RECENT 60 SECONDS ===========
  (rolling window; samples expire after 60s)
ID  broadcast_code     packet_loss  queue_drops  handshake_timeouts
0   3WEDH7600111191           0.00%            0                   0
1   3WEDH5900100671           0.00%            0                   7
2   3WEDJA700100021           2.24%            0                   0
3   3WEDH7600103661              --            0                   0
  packet_loss=network point-packet loss; queue_drops=packets received but dropped by Driver queue
  handshake_timeouts=SDK handshake attempts, not independent fault episodes
==================== ASSESSMENT GUIDE ============
  ACTIVE=current fault; RECOVERING=automatic recovery in progress; IDLE=intentional low-power
  STABLE/OBSERVE/WATCH/UNSTABLE combine the rolling 60s metrics with repeated events/actions in the last 10m
==================== PROCESS HISTORY =============
  (Driver process; resets on restart; not current alarms)
  L1 3WEDH5900100671:
    link: disconnect episodes=3; outage duration=12s; current link up=--
    handshake attempts (SDK): ACK=42 timeout=498 rejected=0
      network=0 protocol=0
    handshake failure episodes: stuck=23; escalated-to-power=6 (subset of stuck)
    POWER_CYCLE_REQUIRED: episodes=6; entries=8 (all causes; entries may repeat within one episode)
    session reset actions: accepted=20; rejected=3
    last SDK event: TIMEOUT detail=500 ip=192.168.31.72 at=2026-07-22 11:04:32

==================== POWER RECOVERY ================
  (shared relay; separate manager process)
  MANAGER   NOW=MANAGER_HEARTBEAT  severity=INFO  manager_age=2s
    detail: mode=auto worker=alive

==================== RELAY HISTORY ==================
  (latest 5 persisted cycles; survives Driver/monitor restart)
  2026-07-30 13:42:18  trigger=3WEDH5900100671  reason=HANDSHAKE_STUCK  group=pit1
    OFF=YES  ON=YES  outcome=RECOVERY_VERIFIED
    detail: all 4 members healthy for 10s

(local refresh; liveness ages use monotonic time)
```

上例从上往下回答：数据源是否仍在更新、实际运行的是哪一对 Driver/SDK、4台里当前有几台故障、每台当前状态、最近60秒发生了什么，以及这个 Driver 进程里以前发生过什么。例如 2 号雷达当前仍在出点，但最近60秒 `packet_loss=2.24%`，因此 `ASSESS=UNSTABLE`；1 号雷达当前是握手故障，所以 `reason=HANDSHAKE_STUCK`。另外三种共享硬恢复原因分别显示 `WAKE_DROPOUT`、`NORMAL_DROPOUT` 和 `STARTUP_MISSING`，告警区会给出各自的请求、generation、健康/静默或启动宽限证据，历史按原因分开计数。

#### 怎么读看板

##### 第一层：`DATA SOURCE`

- `DRIVER NOW=LIVE` 才表示下方 Driver 看板仍在实时更新；超过 5 秒没有收到新数据会变为 `DRIVER_STALE/CRITICAL`，此时下方内容只能当最后一次快照，不能当当前状态。
- `POWER-MGR NOW=NOT_SEEN` 表示看板启动以来没有收到manager首帧，可能是继电器功能未启用、正在启动或manager启动失败；它不证明继电器硬件本身故障。收到首帧后显示实际状态，后续心跳超过30秒未收到则显示 `MANAGER_STALE/CRITICAL`。底部 `POWER RECOVERY` 保留更完整的manager/电源组细节。两者都按本机单调时钟计算，不受系统时间跳变或消息内时间戳影响。

##### 第二层：`SOFTWARE` 与 `CURRENT ALERTS`

- `SOFTWARE` 显示**正在运行的二进制**编译时内嵌的 Driver commit 和固定配套 SDK commit。源码更新后若尚未重启，这里仍会如实显示旧二进制版本；`compatibility=PINNED` 表示构建时已通过精确SDK SHA校验，不表示GitHub以后不会再发布更新。
- `CURRENT ALERTS` **只显示当前仍存在的故障**，恢复后立即消失。`POWER_CYCLE_REQUIRED` 以及已武装自动恢复的 `STARTUP_MISSING` 标为 `[CRIT]`，其余当前故障标为 `[ALERT]`；握手告警带 session reset 和 SDK 事件，唤醒告警带 request/generation，正常运行掉线带“健康至少 30 秒 + 当前静默至少 5 秒”证据，启动缺失带 30 秒宽限和合成 `handle=255`，`NO_DATA/ERROR/Config` 仍显示各自的有界软恢复阶段。
- 顶部没有告警不代表进程内从未发生过故障；已经恢复的事件在底部 `PROCESS HISTORY` 查。

##### 第三层：`CURRENT DEVICES`、`RECENT 60 SECONDS` 与 `ASSESS`

| 列 | 含义 |
|----|------|
| `CURRENT` | 这一秒的真实状态：`NORMAL` / `NO_DATA` / `DISCONNECTED` / `PLANNED_POWER_CYCLE` / `NORMAL_NO_BROADCAST` / `BROADCAST_RETURNING` / `NORMAL_DROPOUT` / `STARTUP_MISSING` / `WAKE_NO_BROADCAST` / `WAKE_DROPOUT` / `BROADCAST_ONLY` / `HANDSHAKE_STUCK` / `POWER_CYCLE_REQUIRED` / `POWER_SAVING` / `STANDBY` / `CONFIG` / `INIT` / `ERROR`；计划内共享断电不计单机故障趋势，`POWER_CYCLE_REQUIRED` 的告警详情明确标注四种原因之一，未知 SDK 状态显示 `?` 并进入 `ACTIVE` |
| `ASSESS` | 当前状态优先，再结合最近60秒数据面/握手尝试和最近10分钟故障 episode 得出的可操作分级；它不是又一个连接状态，具体规则见下表 |
| `points/s` | 1 Hz 看板相邻两次刷新间收到的点云包数（近似每秒速率）；Horizon 正常采样时通常约2500，未连接显示 `-` |
| `packet_loss` | `RECENT 60 SECONDS` 中最近60秒点云网络丢包率，按 `lost / (received + lost)` 计算；窗口内没有点云样本显示 `--`。它不是 Driver 启动以来累计；连接 generation 会显式标记断线清零，即使一秒内重连后的新计数已经超过旧值也不会错误差分 |
| `queue_drops` | 最近60秒队列丢包包数：包已到 Driver、但本地队列处理不过来。它和网络丢包 `packet_loss` 是两回事；非0通常指向 CPU、下游订阅者或发布消费瓶颈 |
| `HW` | 当前硬件健康位；`OK` 正常，异常时显示 `temp/motor/fan/dirty/volt/fw/sys`，多个短标签以 `+` 连接，过长显示 `MULTI`（完整标签仍在顶部告警）。这是状态码，不是具体温度℃或风扇转速 |
| `connected` | 当前心跳连接已维持多久；不是点云连续发布时长，`POWER_SAVING` 时也会继续增长，未连接显示 `--` |
| `disc` | **本次Driver进程内**该雷达的非计划掉线episode累计数，0也会常驻显示；Driver重启后清零。共享继电器已通过intent/ACK识别的计划维护断线单独计数，不增加这里的 `disc` |
| `handshake_timeouts` | 最近60秒 SDK 握手 `TIMEOUT` **尝试数**。一次持续卡死期间 SDK 会进行多笔握手，所以它不是独立故障次数，也不是点云UDP丢包率；它单独出现只会把健康雷达提升为 `WATCH`，不会直接判为 `UNSTABLE` |

这里的“60”不是错误码，也不是阈值：它只是滚动观察窗口长度。选择60秒是为了过滤1～2秒瞬时抖动，同时让已经消失的问题在一分钟后退出当前视图；长期判断另用最近10分钟episode和底部进程历史。

`ASSESS` 严格按下面的顺序从左到右判定，命中第一个条件后立即停止：

```text
ACTIVE → RECOVERING → IDLE → UNSTABLE → WATCH → OBSERVE → STABLE
```

因此当前真实状态永远优先于历史趋势：正在故障时显示 `ACTIVE`，自动恢复过程中显示 `RECOVERING`，人为PowerSaving/StandBy显示 `IDLE`；只有三者都不成立，才使用最近60秒指标和最近10分钟事件判定后四种稳定性。

| `ASSESS` | 判定（从上到下优先）|
|---------|----------------------|
| `ACTIVE` | 当前正在 `DISCONNECTED/NORMAL_NO_BROADCAST/BROADCAST_RETURNING/NORMAL_DROPOUT/STARTUP_MISSING/NO_DATA/ERROR/HANDSHAKE_STUCK/WAKE_NO_BROADCAST/WAKE_DROPOUT/POWER_CYCLE_REQUIRED`，Config 自动重启预算已耗尽、状态未知，或当前硬件健康位异常 |
| `RECOVERING` | 当前处于 `BROADCAST_ONLY/CONFIG/INIT`（Config 预算尚未耗尽），或已是 `NORMAL` 但这一秒尚未发布点云，尚未达到对应告警条件 |
| `IDLE` | 人为进入 `POWER_SAVING` 或 `STANDBY`；不会把正常休眠误报为不稳定 |
| `UNSTABLE` | `packet_loss >= 1.00%`；或最近10分钟内同类 `handshake-stuck/wake-dropout/normal-dropout/escalated-to-power/硬件故障/mode-fail` 唯一 episode 至少发生2次；或实际自动重启动作至少执行2次 |
| `WATCH` | `packet_loss >= 0.10%`、`queue_drops > 0`、最近60秒存在任一种握手错误尝试，或最近10分钟出现过任一故障 episode/恢复动作；尚未满足 `UNSTABLE` |
| `OBSERVE` | 当前与窗口内均无异常，但针对这个 broadcast code 的连续观察尚不足 10 分钟 |
| `STABLE` | 当前正常，且已连续观察至少 10 分钟，滚动窗口内没有上述异常 |

典型变化示例：Driver刚开始观察一台正常雷达时为 `OBSERVE`，连续无异常满10分钟后变为 `STABLE`；最近60秒丢包达到0.10%或出现队列丢包时变为 `WATCH`，丢包达到1.00%时直接变为 `UNSTABLE`；最近10分钟只有一次掉线/故障/恢复动作通常为 `WATCH`，同类 `handshake-stuck/wake-dropout/normal-dropout/escalated-to-power/硬件故障/mode-fail` episode或自动重启动作达到2次则为 `UNSTABLE`。普通 `disconnect` 和同一硬恢复episode内重复发布的power-request edge不会单独把雷达升级为 `UNSTABLE`。

> **共享继电器伴生断线不再污染单机历史：**manager 在OFF前先发布带token、Driver instance和4个members的计划断电意图；Driver只有在members与实际白名单完全一致时才为4台建立短时标记并ACK。manager收到ACK后还会再复核一次触发故障，随后才允许OFF。健康伴随雷达的断线记录为 `PLANNED_GROUP_POWER_CYCLE`，单独计入maintenance历史，不增加 `disconnect / NORMAL_DROPOUT / POWER_CYCLE_REQUIRED`，也不影响稳定性趋势。ACK超时、拒绝、实例变化或复核恢复都会先确认所选通道集合仍全部为ON，再取消循环。

> `ASSESS=WATCH` 但 `handshake_timeouts=0` 并不矛盾：该列只展示超时尝试；`WATCH` 还会考虑最近60秒的 `REJECTED/NETWORK_ERROR/PROTOCOL_ERROR`、队列丢包，以及最近10分钟的 episode/恢复动作。

> 两个滚动窗口和 `LinkStat` 进程历史都按 **broadcast code** 隔离；同一个 handle 若被另一台物理雷达复用，会立即清空旧设备的窗口、历史和本地恢复预算，旧雷达证据不会串到新雷达名下。

##### 第四层：`PROCESS HISTORY`

底部只在出现过历史事件时显示，按 broadcast code 汇总**本次 Driver 进程**内的证据；Driver 重启即归零，它不是当前告警：

| 历史行 | 口径 |
|--------|------|
| `link` | 掉线 episode 数、最近一次掉线持续时间、当前心跳连接时长 |
| `handshake attempts (SDK)` | SDK 尝试结果累计：`ACK/timeout/rejected/network/protocol`。`ACK` 只表示握手 ACK 被接受，仍可能停在 DeviceInfo pending，不等于已经公开 `Connect`；`timeout=498` 表示 498 笔尝试超时，不是 498 次独立故障 |
| `handshake failure episodes` | `stuck` 与 `escalated-to-power` 是按广播故障周期去重的 **episode** 计数；`subset of stuck` 表示后者只是满足全部硬断电升级条件的前者子集，无需相等 |
| `wake dropout episodes` | 仅统计具有显式 PowerSaving/StandBy→Normal、同 broadcast code + generation 证据且持续无广播 10 秒的唯一 episode；普通 `DISCONNECTED` 不进入该计数，也不计入 `handshake failure episodes` |
| `normal-dropout episodes` | 仅统计同一 generation 已连续健康发布至少 30 秒、随后控制和广播持续消失至少 5 秒的唯一 episode；广播短暂返回会重置连续静默，稳定 3 秒/3 帧才移交握手路径 |
| `POWER_CYCLE_REQUIRED: episodes / entries` | `episodes` 是所有原因合计并按故障周期去重的硬恢复 episode；`entries` 是 Driver 已提交进入/重新进入该状态的次数。日志/告警标注 `HANDSHAKE_STUCK / WAKE_DROPOUT / NORMAL_DROPOUT / STARTUP_MISSING`。同一 episode 可能在 OFF 前恢复并取消，所以 entries 不能当作独立故障数 |
| `session reset actions` | Driver 请求 SDK 清理 session 的**恢复动作**计数；accepted 只表示 API 接受，不保证 `RESET` 完成或连接恢复，也不能当作新的故障 episode |
| `last SDK event` | 最近握手事件、detail、目标 IP 与时间；当前 episode 结束/成功重连后仍保留。`NETWORK_ERROR` 指向本机 socket/路由/端口证据，不能仅凭它要求雷达断电 |
| `hardware fault episodes` / `temperature state changes` | 硬件故障 episode、故障标签，以及温度状态变化次数和最近时间；恢复后仍保留 |
| `automatic reboot actions` | 自动恢复看门狗实际接受的雷达软重启动作数和最近时间 |
| `mode failures` | 所有目标模式合计的失败总数、最后一次失败的目标模式和最近时间；`last-mode` 不是按模式拆分计数 |

`CURRENT DEVICES / disc` 现在始终显示每台本次Driver进程的掉线次数，包括0次；`PROCESS HISTORY / link: disconnect episodes=...` 只在该台确实出现过历史事件时显示，并补充最近掉线持续时间和当前连接时长。两者的 `disconnect` 口径相同，均不包含已正确标记的共享继电器计划维护断线。

`stuck=23, escalated-to-power=6` **无需相等，这种情况正常**：前者表示 23 个 episode 到达“广播存在但握手持续失败”的门槛，后者只统计其中进一步满足 session reset 已完成、额外观察时间已满、广播仍新鲜且最近没有本机 `NETWORK_ERROR` 等全部条件的 6 个唯一 episode。其余 episode 可能已握手恢复、广播消失、reset 被拒绝/未完成、检测模式未启用恢复，或被网络错误门禁拦住。旧看板的 `power-alert` 更接近现在单独列出的 `POWER_CYCLE_REQUIRED entries`；它是进入硬断电升级状态的次数，同一 episode 被网络门禁取消后可以再次出现，而且极窄竞态下可能在请求真正发布前取消。`reset accepted/rejected` 又是 session 恢复动作，三者都不能一一对应。

`wake-dropout` 与上述握手口径独立：它不需要、也不执行 session reset，不会增加 `stuck`、`session reset actions` 或握手 `escalated-to-power`。它只增加 `wake dropout episodes`，若 `auto_recover=true` 再以 `reason=WAKE_DROPOUT` 进入通用 `POWER_CYCLE_REQUIRED entries`。

> **判断哪台最该排查/换：**先看 `ACTIVE`；没有当前故障时，看同一 Driver 下谁长期反复进入 `UNSTABLE/WATCH`。重点比较 `packet_loss`、最近 10 分钟重复 episode，并用 `PROCESS HISTORY` 的 `timeout/rejected/network/protocol`、故障标签和自动重启次数定位方向。不要只凭一个很大的 `timeout` 历史累计就判定 498 次独立故障。

##### 第五层：`POWER RECOVERY` 与 `RELAY HISTORY`

- `POWER RECOVERY` 是manager对每个共享电源组的**当前/最近状态**，新状态会覆盖旧状态，不是时间线。
- `RELAY HISTORY` 每次以只读方式查询 `~/.local/state/livox-power-cycle-manager/state.sqlite3`，按时间倒序显示最近5个持久化cycle；Driver、manager或看板重启后仍保留。每条显示触发雷达、原因、电源组、OFF/ON是否得到B0确认、最终 `outcome` 和detail。
- `OFF=--` 表示该cycle没有取得OFF确认，可能在OFF前安全取消或命令失败；`ON=--` 表示尚未取得ON确认，必须结合 `outcome/detail` 判断，不能理解为当前一定处于断电。当前必须补ON的义务仍由manager和systemd安全钩子负责。
- 数据库不存在时显示 `no relay cycle has been recorded`；数据库被占用、损坏或schema不兼容时显示 `unavailable`，但不会影响Driver、manager或其他看板板块。

#### 关于温度与风扇（重要说明）

Livox SDK **不暴露具体温度数值**（如 62℃），那个 60℃ 风扇启动阈值是固件内部的。能拿到的只有粗粒度状态码；主表 `HW` 将任一非正常状态压缩为 `temp/fan/...` 标签，具体等级看事件日志：
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

#### "假活"故障：state=Normal 但 points/s=0

Livox 的**心跳通道和点云数据通道是独立的**。偶尔会出现一台雷达**心跳正常上报 Normal，但点云输出卡死**——驱动以为它好好的，实际一个点都不出。看板会把这种情况标成 **`NO DATA`**（而不是 `Normal`），让它一眼扎眼。

> 原因可能是固件卡住，或采样没真正启动。手动重启那台雷达即可恢复。

#### 可选：自动恢复看门狗（`auto_recover`）

默认关闭。开启后，驱动对**五类故障**使用相互隔离的恢复路径：

```bash
roslaunch livox_ros_driver livox_lidar_multi.launch auto_recover:=true
```

**情况 A：假活（连着 + `Normal` + 持续没有点云发布）** —— 两段式：

| 阶段 | 触发 | 动作 |
|------|------|------|
| 1（轻）| 无**发布**数据满 5 秒 | 重发 `StartSampling`（几乎无中断；也能把"上次启采样超时后卡在半路"的雷达重新拉回采样态）|
| 2（重）| 仍无发布数据满 15 秒 | `RebootDevice` 重启该雷达（~10 秒恢复）|
| 循环 | 重启后仍无发布数据满 45 秒 | 回到阶段 1 重来一整轮（**计时归零**，保持 5s/15s 节奏，不会退化成秒级重启风暴）|

- 判据是**"发布出去的点云"**而不是"收到的 UDP 包"，所以能发现“UDP 仍在收、ROS 却没发布”的假活；但 `Config` 明确排除在情况 A 之外，避免配置只完成一半时强行 `StartSampling`
- 只对 `Normal` 状态生效；**节电/待机**模式本就不出数据，不会被误恢复
- **正在执行计划中的模式切换**（唤醒/休眠命令进行中）的雷达不受此路径打扰——切换由自己的校验/重试机制负责，不会被看门狗中途踹一脚

**情况 B：`Error` 状态（如电机故障 `motor=ERR!`）** —— 这类故障雷达自报 `Error`、不算"在出数据"，情况 A 抓不到，单独处理：

| 触发 | 动作 |
|------|------|
| 进入 `Error` 满 **3 秒** | 重启该雷达（第 1 次）|
| 重连后仍 `Error`，每再过 **~40 秒** | 再重启，**最多 3 次** |
| 3 次后仍 `Error` | **停止重启**，每 30 秒打一条 `[LivoxRecover]` `ERROR` 告警"需人工处理（多半是风扇/电机硬件坏了）" |

- 设了上限是为了**避免死循环刷重启**：风扇/电机真物理损坏时，重启救不回来，试 3 次就放弃并明确报警，而不是无限重启掩盖故障
- 重启次数**脱离 `Error` 持续 60 秒才清零**（重启过程会短暂经过 Init/Normal，若见一眼 Normal 就清零，3 次上限会被绕过、变成无限重启）；冷却按"重连后仍 `Error` 的 40 秒"算，偏保守（给它时间稳定）
- `Error` 路径在模式切换期间**照常生效**：唤醒过程不该报 `Error`，报了就是真故障、就该快速重启（运维决策）

**情况 C：长期停在 `Config`** —— 配置命令没有全部完成时绝不绕过配置直接采样：

| 触发 | 动作 |
|------|------|
| 首次持续停在 `Config` 约 **30 秒** | 重启该雷达 |
| 重连后再次卡住，每次约 **40 秒** | 再重启，整个故障 episode 最多 3 次 |
| 3 次后仍卡住 | 停止自动重启，每 30 秒告警需人工处理 |

- 只有恢复到 `Sampling` 且连续有点云发布约 60 秒，才清空本次 Config 重启预算，防止短暂重连绕过上限

**情况 D：广播仍在，但握手/DeviceInfo 卡死** —— 这正是 Driver 和 Viewer 都连不上、断电后立即恢复的故障形态：

**情况 E：显式唤醒后掉线且广播消失（`WAKE_NO_BROADCAST / WAKE_DROPOUT`）** —— 这条路径不从一般 `DISCONNECTED` 推断，只在下列证据全部成立时武装：

- Driver 只在每台雷达的首笔 PowerSaving / StandBy→Normal 命令实际准备入 SDK 队列时，记录请求 ID、broadcast code、connection generation 和单调时间；同步入队失败立即撤销。已在 Normal 时重复发 Normal 不武装该路径
- 每台的 **60 秒观察窗从它自己的实际首次下发时刻起算**；`handle:255` 的第 2/3/4 台因此分别晚约 2/4/6 秒开始。若在错峰 deadline 前先断线重连，Normal 意图可以在新连接继续，但旧连接的低功耗事实不会转移、也不会武装硬恢复。掉线时仍必须是同一 broadcast code + generation；身份变化、相反模式请求、主动软重启或尚未发生归因断线时窗口到期会取消旧证据
- 掉线后先显示 `WAKE_NO_BROADCAST` 并持续观察；若连续 **10 秒**没有任何广播才提交一次唯一 `WAKE_DROPOUT` episode。单个残余广播帧只暂停并重置连续静默计时，不会永久删除已经在 60 秒窗内取得的断线归因；即使新的 10 秒静默确认跨过窗口终点也仍可完成。只有至少 3 帧广播且持续满 3 秒才稳定移交情况 D 握手路径，真实 `Connect` 则立即结束本 episode
- `auto_recover=false` 时只显示 `WAKE_DROPOUT` 告警，不发布硬断电请求；`auto_recover=true` 时升级为 `POWER_CYCLE_REQUIRED reason=WAKE_DROPOUT`
- 这条分支仍只处理显式唤醒证据；无该证据的故障分别交给下述 `NORMAL_DROPOUT` 或 `STARTUP_MISSING` 门禁，不能借用 wake request/generation

**情况 F：已稳定工作的 Normal 雷达同时失去控制连接和广播（`NORMAL_NO_BROADCAST / NORMAL_DROPOUT`）**：

- 必须先在同一 broadcast code + connection generation 下连续满足 `Normal + Sampling + publishing` 至少 **30 秒**；任何无点云、Config、低功耗、模式切换或计划软重启都会撤销预武装
- 随后 SDK disconnect 且广播连续消失 **5 秒**才形成唯一 `NORMAL_DROPOUT` episode；重连立即取消。广播返回会暂停断电路径，只有至少 3 帧且跨 3 秒才稳定移交情况 D；残余帧后再次静默则重新计算 5 秒
- `auto_recover=false` 只报警；`true` 才升级为 `POWER_CYCLE_REQUIRED reason=NORMAL_DROPOUT`

**情况 G：Driver 启动时白名单成员缺失（`STARTUP_MISSING`）**：

- 只监督 JSON/命令行白名单中的完整 broadcast code；Driver 启动后给每个成员 **30 秒**宽限。宽限内出现连接或新鲜广播立即停止计时
- 到期仍无连接、无新鲜广播且从未健康发布时，用合成 `handle=255` 以 1 Hz 发布该 broadcast code 的实时状态；真实广播/连接一出现便立即撤销
- `auto_recover=true` 时升级为 `POWER_CYCLE_REQUIRED reason=STARTUP_MISSING`；manager 仍执行更新帧复核、共享端点冷却和 24 小时熔断，因此不会因 Driver 重启形成无限断电循环

##### 4 号坑 2026-07-27 已确认时间线

下表只记录 Driver 日志和现场断电已确认的事实；它证明本次不是“有广播但握手卡死”，也不是 Driver 进程崩溃。电源瞬时压降仍不能仅凭该日志绝对排除，但“四台同时唤醒、模式重发与配置命令集中”是直接可观测的软件压力：

| 时间 | 已确认事件 |
|------|------------|
| 20:06:47 | 对 4 台雷达发起 `ALL→Normal` |
| 20:06:50～20:06:58 | 4 台都出现模式重发，同一时段进入 spin-up/配置 |
| 20:06:59～20:07:05 | 多笔配置命令在 500 ms 边界进入 timeout 终态；旧日志随后仍打印拼写错误的 `Recieve Ack`，该行是统一 callback 文案，**不能据此认定收到了迟到的真实 ACK** |
| 20:07:03 | handle 0 / `192.168.31.70` 掉线，后续无广播 |
| 20:07:19 | handle 1 / `192.168.31.72` 掉线，后续无广播 |
| 20:07:28 / 20:07:33 | handle 2 / 3 分别因 Config 持续 30 秒执行单机软重启，随后立即重新广播、握手并恢复 |
| 约 20:27:24 | 现场对共享电源的 4 台雷达整组断电 |
| 20:27:32 | 4 台在约 8 ms 内全部恢复广播，随后约 30 ms 内全部握手成功；上电后配置命令约 20 ms 完成 |

handle 0 / 1 的离线时长分别约 1228 秒和 1212 秒，期间始终没有广播；Driver 一直是同一 PID，handle 2 / 3 在 0 / 1 离线期间仍能收发命令与点云。因此新策略一方面通过 0/2/4/6 秒错峰、20 秒 ACK 观察期、2 次有界重发和每台配置命令串行降低重现概率，另一方面把同样的唤醒掉广播在约 10 秒而不是 20 分钟后升级恢复。

##### 错误检测、升级与共享继电器恢复流程

下图把四种硬恢复原因分开判定，只在各自证据完整时才汇入同一共享电源组恢复。时间均使用单调时钟；wall-clock 只作为跨进程证据随消息传递。

```mermaid
flowchart TD
  subgraph SOFT["1. 广播存活的握手软恢复（单台）"]
    A["持续收到广播但尚未公开 Connect<br/>t = 0"] --> B["BROADCAST_ONLY<br/>0～5 秒"]
    B --> C["SDK 继续正常握手<br/>每笔最多等待 500 ms<br/>上一笔终止后由后续广播触发下一笔<br/>同一时刻最多 1 笔 pending"]
    C --> D{"已公开 Connect？"}
    D -->|是| OK["IDLE / 正常连接<br/>结束本次 episode"]
    D -->|否：t 小于 5s| C
    D -->|否：t 约 5s| E["HANDSHAKE_STUCK"]

    E --> AR{"auto_recover 已启用？"}
    AR -->|否| OBS["只显示 / 告警 HANDSHAKE_STUCK<br/>SDK 仍继续正常握手<br/>不请求 reset、不升级硬断电"]
    OBS -->|后续公开 Connect| OK
    AR -->|是| RESET["Driver 只请求 1 次 session reset"]
    RESET --> ACCEPT{"reset API 已接受？"}
    ACCEPT -->|否| FAIL["保持 HANDSHAKE_STUCK<br/>继续允许正常握手<br/>不循环 reset、不共享断电<br/>报警检查 SDK / 本机网络"]
    FAIL -->|后续公开 Connect| OK
    ACCEPT -->|是| WAIT["reset-phase = queued<br/>继续正常握手并等待 SDK RESET 完成<br/>没有“等不到就越级断电”的超时"]
    WAIT --> COMPLETE{"收到 SDK RESET 完成事件？"}
    COMPLETE -->|否| WAIT
    COMPLETE -->|是| G["reset-phase = completed<br/>继续正常握手<br/>从 RESET 完成起再观察 5 秒"]
    WAIT -.->|期间公开 Connect| OK
    G --> H{"已公开 Connect？"}
    H -->|是| OK
    H -->|否| I{"升级条件全部满足？<br/>episode ≥ 10 秒<br/>RESET 完成 ≥ 5 秒<br/>广播在最近 3 秒内出现<br/>最近 5 秒无本机 NETWORK_ERROR"}
    I -->|观察时间未到| G
    I -->|最近有 NETWORK_ERROR| NET["保持 / 退回 HANDSHAKE_STUCK<br/>先排除网卡、路由、端口与本机 socket"]
    NET -->|连续安静 5 秒后重新评估| I
    NET -.->|期间公开 Connect| OK
    I -->|是：通常 t 约 10～12s| PCR["POWER_CYCLE_REQUIRED<br/>reason = HANDSHAKE_STUCK"]
    PCR -.->|后续公开 Connect| OK
    PCR -.->|出现新的 NETWORK_ERROR| NET

    NOTE["全程独立守卫（适用于本区所有未连接状态）<br/>广播超过 3 秒未再出现<br/>→ DISCONNECTED，结束该 live episode"]
    A -.->|并行监测整个 episode| NOTE
  end

  subgraph WAKE["2. 显式唤醒掉广播（单台归因）"]
    W0["真实状态为 PowerSaving / StandBy<br/>收到显式 Normal 意图"] --> WS["ALL→Normal 首次下发错峰<br/>0 / 2 / 4 / 6 秒"]
    WS --> WARM["每台实际首次 SDK enqueue 前<br/>记录 request + bcode + generation<br/>各自启动 60 秒窗；同步失败立即撤销"]
    WARM --> WG["accepted / spinning-up ACK 后<br/>20 秒 grace 内不重发<br/>之后最多 2 次，间隔 5 秒"]
    WG --> WC["每台内部配置命令串行<br/>前一项终态 callback 后再发下一项"]
    WC --> WQ{"请求后 60 秒内掉线？<br/>且 bcode + generation 仍精确相同？"}
    WQ -->|否：正常恢复或窗口到期| WOK["清除唤醒证据<br/>普通运行"]
    WQ -->|是| WN["WAKE_NO_BROADCAST<br/>观察持续无广播时间"]
    WN --> WR{"在 10 秒内恢复？"}
    WR -->|已重连 / 已恢复点云| WOK
    WR -->|广播恢复但未 Connect| WBR["暂停 wake 硬恢复并重置静默计时<br/>≥ 3 帧且持续 ≥ 3 秒：稳定移交握手路径<br/>否则再次静默：回到 WAKE_NO_BROADCAST"]
    WBR -->|广播达到稳定门槛| A
    WBR -.->|原始断线已在60秒窗内；10秒确认可跨窗终点| WN
    WR -->|否：持续无广播 10 秒| WD["WAKE_DROPOUT<br/>唯一 wake episode"]
    WD --> WAR{"auto_recover 已启用？"}
    WAR -->|否| WOBS["只报警 WAKE_DROPOUT<br/>不请求断电"]
    WAR -->|是| WPCR["POWER_CYCLE_REQUIRED<br/>reason = WAKE_DROPOUT"]
  end

  subgraph NORMALDROP["3. 已稳定运行后的普通掉线（单台归因）"]
    N0["同一 bcode + generation<br/>Normal + Sampling + publishing"] --> NARM{"连续健康满 30 秒？"}
    NARM -->|否| N0
    NARM -->|是| ND["武装 NORMAL_DROPOUT<br/>只绑定当前 generation"]
    ND --> NDISC{"SDK disconnect 且身份仍匹配？"}
    NDISC -->|否：继续健康或计划模式/软重启| NOK["清除预武装<br/>普通运行"]
    NDISC -->|是| NN["NORMAL_NO_BROADCAST<br/>开始连续静默计时"]
    NN --> NR{"5 秒内恢复？"}
    NR -->|真实 Connect| NOK
    NR -->|广播返回但未 Connect| NBR["暂停硬恢复<br/>≥3 帧且跨 ≥3 秒：移交握手路径<br/>残余帧后再静默：重新计 5 秒"]
    NBR -->|广播稳定| A
    NBR -.->|再次静默| NN
    NR -->|否：连续无广播 5 秒| NCONF["NORMAL_DROPOUT<br/>唯一 normal episode"]
    NCONF --> NAR{"auto_recover 已启用？"}
    NAR -->|否| NOBS["只报警 NORMAL_DROPOUT<br/>不请求断电"]
    NAR -->|是| NPCR["POWER_CYCLE_REQUIRED<br/>reason = NORMAL_DROPOUT"]
  end

  subgraph STARTUP["4. 白名单成员启动缺失"]
    S0["Driver 启动<br/>读取配置白名单"] --> SG["每个成员独立 30 秒宽限"]
    SG --> SP{"已有连接 / 新鲜广播<br/>或曾健康发布？"}
    SP -->|是| SOK["停止 STARTUP_MISSING<br/>交给真实 handle 状态"]
    SP -->|否：满 30 秒| SM["STARTUP_MISSING<br/>1 Hz 合成 handle = 255 状态"]
    SM --> SAR{"auto_recover 已启用？"}
    SAR -->|否| SOBS["只报警 STARTUP_MISSING"]
    SAR -->|是| SPCR["POWER_CYCLE_REQUIRED<br/>reason = STARTUP_MISSING"]
    SM -.->|任何广播 / Connect 出现| SOK
  end

  subgraph HARD["5. Relay Manager 共享硬恢复（同组 4 台）"]
    PCR --> M0["收到带唯一 episode 身份和 reason 的请求<br/>同组并发事件合并为同一物理端点的一次循环"]
    WPCR --> M0
    NPCR --> M0
    SPCR --> M0
    M0 --> M1{"原因特定安全复核通过？<br/>HANDSHAKE：required + 广播新鲜 + reset 完成<br/>WAKE：两个 generation 相等；窗内归因；静默 ≥10 秒<br/>NORMAL：两个 generation 相等；此前健康 ≥30 秒；静默 ≥5 秒<br/>STARTUP：handle=255；启动缺失 ≥30 秒；无连接/广播/发布<br/>其余原因证据必须为 0<br/>armed + 恰好 4 members + 所选通道全部 ON<br/>B0 预检后再收到同 episode/reason 新状态"}
    M1 -->|否| SUP["不发送 OFF并保留明确告警<br/>瞬态预检：间隔 60 秒，总计最多 5 次<br/>已恢复 / 禁用：取消"]
    M1 -->|是| INTENT["发布计划断电 token<br/>携带 Driver instance + 精确 4 members<br/>此时尚未占用断电预算"]
    INTENT --> ACK{"当前 Driver 已先标记 4 台<br/>且白名单与 members 完全一致并 ACK？<br/>每次等待 3 秒，最多 3 次，间隔 0.5 秒"}
    ACK -->|超时且未满 3 次| INTENT
    ACK -->|拒绝 / 实例或映射不一致<br/>或 3 次均超时| CANCEL0["发布 CANCEL；不发送 OFF<br/>没有 cycle / obligation，也不占预算"]
    ACK -->|是| M2{"ACK 后复核<br/>同一 episode + reason 仍成立？"}
    M2 -->|否| CANCEL0
    M2 -->|是| BUDGET{"预留持久化安全预算<br/>未触发 30 分钟冷却或 24h 3 次上限？"}
    BUDGET -->|否| CANCEL0
    BUDGET -->|是| OBL["持久化 must-be-ON obligation<br/>确保进程中断后仍会补上电<br/>同时每秒刷新计划断电 intent"]
    OBL -->|写入失败| CANCEL["发布 CANCEL；不发送 OFF<br/>确认所选通道集合仍全部为 ON并取消本次循环<br/>释放首条 OFF 前未使用的安全预算"]
    OBL -->|成功| M3{"OFF 前最新 1 Hz 触发状态<br/>仍精确匹配本次 episode + reason + 证据？"}
    M3 -->|否| CANCEL
    M3 -->|是| POFF["单条 A1 掩码命令关闭所选通道集合<br/>本现场 channels = 1,2,3,4<br/>同组 4 台一起断电；伴生断线记为计划维护"]
    POFF --> OFFQ{"B0 已确认所有所选通道 OFF<br/>且未选择的输出（如有）均未变化？"}
    OFFQ -->|是| HOLD["按有效 off_seconds 保持 OFF<br/>新模板 5 秒；未迁移旧配置可能 10 秒<br/>然后必须恢复 ON"]
    OFFQ -->|否：立即补 ON，不等待| PON
    HOLD --> PON["finally 使用新 TCP 连接恢复 ON<br/>并用 B0 查询确认"]
    PON --> ONQ{"所有所选通道已确认 ON？"}
    ONQ -->|否| REPAIR["POWER_ON_UNCONFIRMED<br/>原循环立即结束且不做健康验收<br/>保留持久化补上电义务"]
    REPAIR --> RETRY{"后台每 30 秒或 systemd 启停钩子补 ON<br/>B0 已确认所选通道全部 ON？"}
    RETRY -->|否：继续补 ON| RETRY
    RETRY -->|是| RESTORED["所选通道全部 ON并清除 obligation<br/>OFF 未记录：POWER_CYCLE_FAILED<br/>OFF 已记录：RECOVERY_UNVERIFIED_AFTER_RESTART<br/>均不进入健康验收"]
    ONQ -->|是| PHASE{"OFF 阶段已完整执行<br/>且未选择的继电器通道（如有）未变化？"}
    PHASE -->|否| ABORT["POWER_CYCLE_FAILED 或<br/>NON_TARGET_STATE_CHANGED<br/>所选通道全部保持 ON，不进入健康验收"]
    PHASE -->|是| VERIFY{"上电后 180 秒内<br/>只接受同一 Driver instance 的新状态<br/>4 台全部 connected + Normal + Sampling<br/>握手 IDLE + publishing，并连续健康 10 秒？"}
    VERIFY -->|是| DONE["RECOVERY_VERIFIED"]
    VERIFY -->|否：180 秒超时| TIMEOUT["RECOVERY_TIMEOUT<br/>告警且不立即再次断电"]
    DONE --> LIMIT["端点安全预算继续生效<br/>下一次至少间隔 30 分钟<br/>24 小时最多 3 次"]
    TIMEOUT --> LIMIT
    ABORT --> LIMIT
    RESTORED --> LIMIT
  end

  classDef normal fill:#e8f1ff,stroke:#2f6fbb,color:#17385f;
  classDef warning fill:#fff4d6,stroke:#b7791f,color:#6b4300;
  classDef danger fill:#ffe4e6,stroke:#be123c,color:#7f1d1d;
  classDef success fill:#dcfce7,stroke:#15803d,color:#14532d;
  classDef guard fill:#f3f4f6,stroke:#6b7280,color:#374151;
  class A,B,C,G,WS,WG,WC,WN,N0,ND,NN,SG,HOLD,PON normal;
  class E,OBS,FAIL,WAIT,NET,WD,WOBS,NCONF,NOBS,SM,SOBS,SUP,CANCEL0,CANCEL,TIMEOUT,REPAIR,RESTORED,ABORT warning;
  class PCR,WPCR,NPCR,SPCR,POFF danger;
  class OK,WOK,NOK,SOK,DONE success;
  class D,H,AR,RESET,ACCEPT,COMPLETE,I,NOTE,WQ,WR,WAR,WARM,WBR,NARM,NDISC,NR,NBR,NAR,S0,SP,SAR,M0,M1,INTENT,ACK,M2,BUDGET,OBL,M3,OFFQ,ONQ,RETRY,PHASE,VERIFY,LIMIT guard;
```

| 握手时间（从首次连续广播起） | 状态/动作 |
|------|------|
| 0～5 秒 | `BROADCAST_ONLY`；SDK 仍随新的广播做多次正常握手，每个时刻同一雷达最多只有一个 pending 握手 |
| 约 5 秒 | `HANDSHAKE_STUCK`；只请求一次本地 session reset，清理该 broadcast code 的 pending/provisional session |
| reset 完成后 0～5 秒 | 继续接收广播；上一笔握手终止后，SDK 可由后续可用广播触发下一笔握手，Driver 不重复请求 reset。若 reset 约在第 5 秒完成，这一段通常对应总计第 5～10 秒 |
| 通常约 10～12 秒，reset 完成晚则相应更晚 | reset 请求已被 SDK 接受、SDK `RESET` 完成事件已到达，且从该完成事件起又观察满 5 秒后仍未公开 Connect、广播仍新鲜、最近 5 秒没有本机 `NETWORK_ERROR`：`POWER_CYCLE_REQUIRED`。绝不会只因从首次广播起满 10 秒就越过未完成的 reset 直接升级 |
| 广播超过 3 秒未再出现 | 结束当前握手 live episode；若已有 wake/normal 的同身份断线归因则由对应分支继续确认，否则显示 `DISCONNECTED` |

| 唤醒时间 | 状态/动作 |
|------|------|
| 显式 PowerSaving / StandBy→Normal | 批量请求按 0/2/4/6 秒错峰；**每台在自己的首笔 SDK enqueue 前**记录 request ID、broadcast code、connection generation，并从该时刻启动 60 秒归因窗；同步入队失败撤销 |
| accepted / spinning-up ACK 后 0～20 秒 | 电机启动观察期，不重发 Normal；每台内部配置命令串行 |
| 20 秒后仍未到 Normal | 最多再发 2 次，每次间隔 5 秒，然后明确 mode-fail，不无限重发 |
| 各自 60 秒窗内掉线 | 同 bcode + generation 才进入 `WAKE_NO_BROADCAST`观察；身份为空/不匹配、主动软重启只是普通恢复流程，不授予共享断电权限 |
| 掉线后持续无广播 10 秒 | 提交唯一 `WAKE_DROPOUT` episode；若出现不足稳定门槛的残余广播帧，连续静默从最后一次暂时恢复后重新计时，但保留原始窗内断线归因；`auto_recover=false` 只告警，`true` 则升级 `POWER_CYCLE_REQUIRED reason=WAKE_DROPOUT` |

| 正常运行掉线时间 | 状态/动作 |
|------|------|
| `Normal + Sampling + publishing` 连续 0～30 秒 | 只建立连续健康计时；尚无共享断电权限 |
| 满 30 秒 | 仅为当前 broadcast code + connection generation 武装正常掉线归因 |
| 后续 disconnect，连续无广播 0～5 秒 | `NORMAL_NO_BROADCAST`；Connect 立即取消，残余广播暂停并重置静默 |
| 连续无广播满 5 秒 | `NORMAL_DROPOUT`；`auto_recover=true` 时升级 `POWER_CYCLE_REQUIRED reason=NORMAL_DROPOUT` |
| 广播恢复至少 3 帧且跨 3 秒 | 稳定移交广播存活的握手分支；未达门槛又消失则重新确认 5 秒 |

| 启动缺失时间 | 状态/动作 |
|------|------|
| Driver 启动后 0～30 秒 | 按白名单逐成员等待；出现连接或新鲜广播立即停止缺失计时 |
| 满 30 秒仍无连接/新鲜广播且从未健康发布 | `STARTUP_MISSING`，持续发布合成 `handle=255` 状态；`auto_recover=true` 时升级同名 reason |
| 后续任何广播或 Connect | 立即撤销合成状态和待 OFF 身份；由真实 handle 接管 |

- 整个软恢复窗口内不是只尝试一次握手：上一笔 pending 握手进入终态/超时，或被 session reset 清理后，后续新广播仍可触发下一笔握手；限制的是同一时刻最多一个 pending 握手
- SDK 对同一 broadcast code 最多只保留一个 pending 握手；不会因每次广播都新建 socket
- 握手 ACK 被设备接受但 DeviceInfo/命令服务卡住属于 **provisional 半连接**，也可以定向清理，且不会向 Driver 制造一次假的 Disconnect
- reset API 返回成功只表示请求已排队；Driver 必须再收到 SDK 的 `RESET` 完成事件并观察 5 秒，才允许升级硬断电。API 拒绝或完成事件缺失时保持 `HANDSHAKE_STUCK`，不会循环 reset，也不会误断电
- `NETWORK_ERROR` 会显示真实 socket errno/detail，并使用独立单调时间门禁抑制“雷达必须断电”的误报；即使后续出现 `RESET/TIMEOUT` 也不会覆盖该保护。若 OFF 前发布的新一帧 1 Hz 状态已反映网络错误或成功连接，manager 会取消本次断电；状态帧发布到 OFF 命令之间仍存在一个不足约 1 秒、无法跨进程原子消除的竞态窗口
- `auto_recover=false` 时仍识别并显示 `HANDSHAKE_STUCK`，但不声称已经执行 session reset，也不会升级为 `POWER_CYCLE_REQUIRED`

前面三类恢复只处理**出问题的那一台**。情况 D～G 只在各自原因证据完整时才升级共享硬恢复：D 统计 handshake/session，E 统计 wake dropout，F 统计 normal dropout，G 通过白名单合成实时状态。四种 reason 互斥，非本 reason 的证据字段必须全部为 0。启动日志仍分别显示自动恢复和握手 session 恢复是否启用。

> ⚠️ 这是驱动**自主重启硬件**的行为，所以默认关闭、需显式开启。无显示器的机器也能用（它和看板无关）。

> **某台 `packet_loss` 偏高 → 排查该台网线/接头/散热；`NO_DATA` → 已连接但无点云；`HANDSHAKE_STUCK` → 广播仍在但控制服务卡住；`WAKE_DROPOUT` → 显式唤醒归因；`NORMAL_DROPOUT` → 此前稳定运行后控制和广播一起消失；`STARTUP_MISSING` → 白名单成员启动宽限内从未出现。`POWER_CYCLE_REQUIRED` 必须继续看 reason。本现场配置 `channels: [1,2,3,4]`，任一有效原因都会用一个掩码让四路及4台雷达一起断电5秒；连接 generation、最新状态复核、冷却和熔断会阻止伴生掉线再次循环。**

#### 可选：原因特定的 `POWER_CYCLE_REQUIRED` 自动继电器硬恢复

这一层只处理四种已确认原因：`HANDSHAKE_STUCK`、`WAKE_DROPOUT`、`NORMAL_DROPOUT` 和 `STARTUP_MISSING`。没有连续健康/启动宽限等证据的单次 `DISCONNECTED` 仍不是触发原因。当前继电器1～4路全部只控制这4台雷达，所以该组配置 `channels: [1,2,3,4]`；任一成员通过原因特定复核后，manager用一条A1掩码命令同时操作四路，该组只执行一次OFF/ON。OFF保持采用现场有效 `off_seconds`（新模板5秒，未迁移旧配置可能仍为10秒）。继电器TCP/SQLite运行在独立ROS Python进程中，不进入C++点云收包线程：

1. Driver 在状态首次进入 `POWER_CYCLE_REQUIRED` 时发布带唯一 `event_id`、`recovery_reason` 和原因证据的 `/livox/power_cycle_request`，并以 1 Hz 发布 `/livox/lidar_recovery_state`。normal 请求携带两代相等的 generation、健康起点、归因断线和当前静默起点；startup 请求携带合成 handle 和缺失起点。
2. `livox_power_cycle_manager.py` 只接受配置中 `power_groups.<组名>.members` 明确列出的 broadcast code，并按四种 reason 严格复核：handshake 要求广播新鲜与 reset 终态；wake 要求同 generation、60 秒归因窗和 10 秒静默；normal 要求同 generation、此前健康至少 30 秒和当前静默至少 5 秒；startup 要求 `handle=255`、启动缺失至少 30 秒且无连接/广播/发布。非本原因证据必须为 0。继电器 B0 预检查后还必须收到触发成员的一帧更新状态；触发者恢复或身份变化时 fail closed，不发 OFF。
3. 现场上位机/PLC已在任一雷达异常时中断测量流程，而且已确认继电器1～4路全部只给这4台雷达供电，因此硬恢复不再等待额外的 `SAFE_TO_CYCLE` 许可。守护进程只允许控制该电源组明确白名单化的 `channels` 集合；本现场用掩码 `0x000F` 同时控制1～4路。代码不提供绕过配置范围的无条件“全部关闭”接口；真正只使用一路的旧现场仍可使用兼容字段 `channel`。
4. manager 在占用断电预算和写 obligation **之前**，先通过 `/livox/group_power_cycle_intent` 发送 token、当前 Driver instance 和 4 个 members。Driver 要求 members 与实际连接白名单完全相等，建立默认 15 秒计划停电标记后才通过 `/livox/group_power_cycle_ack` ACK。每次默认等待 3 秒，超时后发布 CANCEL 并间隔 0.5 秒换新 token 快速重试，最多 3 次（总计约 10 秒）；明确拒绝不重试。3 次均超时、Driver拒绝、身份/成员不一致或ACK后故障已恢复，均不发送 OFF，也不会创建 cycle/obligation 或占用断电预算。ACK 成功后每秒刷新 intent，直到继电器操作结束。
5. 发送OFF前已按物理供电端点把“所选通道集合必须全部恢复ON”、通道掩码及4个成员快照提交到SQLite；OFF、ON都通过独立B0查询确认所有选中位。Driver的systemd安全钩子使用 `~/.local/libexec/livox-power-cycle-manager/` 中的稳定原子更新副本，在每次启动前和停止后执行与源码checkout、现场JSON、ROS无关的紧急补上电；仍有任何补上电义务时，全局禁止新的OFF。计划维护标签最多保持180秒；仍未重连时当前状态回到 `DISCONNECTED`，不会永久用维护标签掩盖恢复失败。
6. 上电后必须等待该组 **4 个 members 全部**重新连接、完成配置、握手为 `IDLE`，并在 `Normal + Sampling + publishing` 状态连续健康 10 秒，才记为 `RECOVERY_VERIFIED`。`PowerSaving/StandBy/Init/Config/Error/Off` 均不算本次硬恢复完成；只恢复触发故障的那台或只收到继电器 `OK!` 也不算整组恢复成功。

每次B0/A1交换都允许一次已由实机确认的连接级兼容流程：收到精确ASCII `v1.0` 后在**同一个TCP连接**重发原命令一次。B0是只读命令；A1携带明确的目标状态和enable mask，重复同一帧不会反转输出。客户端随后持续累计TCP字节直到取得完整9字节B0、`OK!`或超时；现场捕获的“前8字节后补最后1字节”属于正常TCP分片。若短暂尾分片等待后第9字节仍不存在，只接受地址、`0D`、四路掩码和第一校验字节全部正确的8字节单校验帧，并打印兼容警告。第二次 `v1.0`、未知版本文本、错误第一校验或非法掩码仍严格失败，并在错误中附带partial十六进制，禁止凭未验证响应继续断电。

自动控制需要这些条件同时成立：Driver安全钩子已安装、launch的 `relay_power_cycle_enable=true`、JSON中目标电源组 `enabled=true`、本次事件的reason-specific状态复核通过，并且没有触发同一物理通道集合 **30分钟冷却**、**24小时3次上限**或继电器安全检查。它不订阅PLC/上位机许可topic；通过全部门禁后按有效 `off_seconds` 对整个选中集合执行OFF/恢复ON，再验收4台点云。

launch 开关是日常唯一总开关：

- `false`（默认）：完全不启动 manager，不读取继电器 JSON，不连接继电器，更不会发送 OFF。
- `true`：launch 强制把 Driver 的 `/auto_recover` 设为 `true`，并以 `required=false + respawn=true + armed` 启动 manager。manager配置、SQLite或继电器网络故障只会让硬恢复看板进入 `MANAGER_STALE/CRITICAL`，不会关闭Driver和点云话题；JSON顶层旧`mode`字段仅为兼容项，命令行覆盖时会明确打印NOTICE，仍不决定launch是否武装。

无论 launch 开关是什么，只要 SQLite 留有历史“必须恢复 ON”义务，已安装的 systemd 启动前/停止后钩子都会优先尝试并确认 ON。这是中断恢复，不是一次新的断电循环。

##### 首次部署与配置

先完成新版 Driver 编译，再运行一次安装脚本。它会创建缺失的外部 JSON、把补 ON helper 原子安装到 `~/.local/libexec/livox-power-cycle-manager/`、给现有 `livox-ros-driver.service` 安装不依赖源码树的补 ON 安全钩子，并安全迁移/删除旧的独立 manager unit；不会覆盖已有 JSON/SQLite，不会启用 launch 开关，也不会自动重启 Driver。旧V1 drop-in必须通过这一步升级到稳定helper V2，否则更新器拒绝重启。生产 Driver unit 必须使用同一普通用户、`Type=simple`、`Restart=always`、`KillMode=control-group`、`RemainAfterExit=no` 和 `SendSIGKILL=yes`，否则安装器 fail closed：

```bash
bash "$HOME/catkin_ws/src/livox_ros_driver/install_livox_power_cycle_service.sh"
```

生成的现场配置是 `~/.config/livox/power_cycle.json`，与雷达白名单JSON分开并位于Git仓库外，所以一键更新不会覆盖。配置格式为 `schema_version=2`：在 `power_groups` 下建立一个共享电源组，`members` **必须恰好填写共用供电的4个完整15位broadcast code**，并只为该组填写一次实际继电器IP、端口和通道集合。本现场必须填写 `"channels": [1, 2, 3, 4]`；manager会把集合排序、生成掩码 `0x000F`，通过单条A1命令同时OFF/ON，并用B0确认四位全部到达目标状态。旧的真实单通道配置 `"channel": 1..4` 保持兼容，但同一relay对象必须且只能填写 `channel` 或 `channels` 之一。空列表、重复值、布尔值、越界值以及两个字段同时出现均fail closed。完成验收后才把该组的 `enabled` 改为 `true`。模板中的组名、广播码和 `192.0.2.55` 都是不可直接使用的占位示例；同一个broadcast code不允许加入多个组，两个enabled组也不能占用同一继电器地址上的任何重叠通道。如果机器上已有旧版 `schema_version=1` / `lidars` 配置，安装脚本会保留而不会覆盖，必须先人工备份并迁移。

新版模板显式使用 `off_seconds: 5`，manager 也强制断电保持时间不得短于 5 秒。为兼容已经部署的 `schema_version=2` 现场配置，省略 `off_seconds` 与显式写 `off_seconds: 10` 都继续按旧值 10 秒生效；更新器会保护仓库外的现场 JSON，不会自动把它们改成 5 秒。因此旧配置无论省略该字段还是显式保存 10 秒，只要希望切换为 5 秒，都必须执行下面的显式迁移。确认现场电气允许后，用这一条命令先在原目录生成候选文件、校验候选文件，通过后才备份生产配置并用 `os.replace` 原子替换；校验输出必须包含 `off_seconds=5`：

```bash
python3 -c 'import json,os,pathlib; p=pathlib.Path.home()/".config/livox/power_cycle.json"; c=p.with_name(p.name+".candidate"); d=json.loads(p.read_text(encoding="utf-8")); d.setdefault("policy",{})["off_seconds"]=5; c.write_text(json.dumps(d,ensure_ascii=False,indent=2)+"\n",encoding="utf-8"); os.chmod(str(c),p.stat().st_mode & 0o777); print("candidate="+str(c))' && python3 "$HOME/catkin_ws/src/livox_ros_driver/livox_ros_driver/livox_ros_driver/scripts/livox_power_cycle_manager.py" --config "$HOME/.config/livox/power_cycle.json.candidate" --validate-config && python3 -c 'import datetime,os,pathlib,shutil; p=pathlib.Path.home()/".config/livox/power_cycle.json"; c=p.with_name(p.name+".candidate"); b=p.with_name(p.name+".bak."+datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")); shutil.copy2(str(p),str(b)); os.replace(str(c),str(p)); print("installed="+str(p)+" backup="+str(b))'
```

修改后使用统一入口做本地格式、安全约束和现场身份校验：

```bash
bash "$HOME/catkin_ws/src/livox_ros_driver/validate_livox_site.sh"
```

只读查询所有已启用映射的四路状态；该入口会先重复离线校验，不会改变任何继电器输出：

```bash
bash "$HOME/catkin_ws/src/livox_ros_driver/validate_livox_site.sh" --check-relays
```

观察集成 manager 与真实故障事件：

```bash
sudo journalctl -u livox-ros-driver -f
```

在有人值守的维护窗口完成人工接线验收，确认所配通道集合平时全部为ON、关闭集合时只让这4台雷达掉电且没有其他负载、恢复ON后4台点云全部恢复。本现场已经确认1～4路全部只控制这4台雷达。全部确认后，把 `livox_lidar_multi.launch` 中这一项改为 `true`：

```xml
<arg name="relay_power_cycle_enable" default="true"/>
```

再次执行上面的 `--validate-config` 和 `--check-relays`，最后只需重启原 Driver 服务：

```bash
sudo systemctl restart livox-ros-driver && systemctl is-active livox-ros-driver
```

需要停用时，把同一 launch 参数改回 `false` 并重启 `livox-ros-driver`；manager 不再启动，systemd 停止后钩子仍会先处理任何遗留补 ON 义务。不要再手工启动 `livox-power-cycle-manager.service`，集成版没有这个独立服务。

##### 默认工业安全策略

| 保护 | 默认行为 |
|------|----------|
| 白名单 | 未加入 `members`、电源组禁用、广播码不合法或一个成员跨组重复，一律 fail closed；launch武装时部署校验器和运行时Driver ACK都要求enabled members与实际Driver白名单完全相等 |
| 计划断电ACK | OFF前manager先等待Driver精确ACK，再占用预算；Driver先标记4台再ACK。每次等待3秒、间隔0.5秒换新token，最多3次；超时token逐一CANCEL，明确拒绝不重试。3次均超时、拒绝、实例/成员不一致或ACK后原因恢复均不创建断电循环。伴生断线单记maintenance，不进入单机故障趋势 |
| 当前状态复核 | Driver 先完成原因归因；manager 再复核时间戳、离线/未发布真值和互斥证据。handshake 要广播新鲜，wake 要同 generation + 60 秒窗 + 10 秒静默，normal 要同 generation + 30 秒健康 + 5 秒静默，startup 要合成 handle 255 + 30 秒缺失。流程有四个 OFF 决策点：初始缓存、B0 预检后强制收到的新状态、Driver ACK 后、obligation 持久化后；任一点恢复/身份变化都取消 OFF。不以其余 3 台健康作为 OFF 前置条件，多台同时异常也只执行一次共享恢复 |
| 测量联锁边界 | 上位机/PLC在任一雷达异常时已负责中断测量；已确认继电器1～4路全部只给这4台雷达供电，因此manager不再要求或等待额外的 `SAFE_TO_CYCLE` 许可 |
| 协议确认 | 私有TCP `B0`状态查询接受完整 `CH/CL`；同时兼容CX-5104E-L实机确认的“正确 `CH` + 固定 `AA` 尾字节”（例如全开状态 `... 0D CD AA`）和“正确 `CH` 后完全省略第二校验字节”（例如 `... 0D CD`），两者均保留WARN。1号工位还确认新TCP连接首次命令返回精确ASCII `v1.0`；manager只对白名单化的这个版本帧在同一连接幂等重发一次，重复版本帧或未知前缀仍失败关闭。接收端先累计TCP分片并短暂等待第9字节；确认没有尾字节时，只在第一校验、地址、`0D`和四路掩码全部正确时接受8字节帧。错误第一校验、未知非 `AA`尾字节及越界状态一律拒绝。只有固件精确返回 `00 00` 时，才需对单个电源组显式设置 `allow_omitted_status_checksum=true` |
| 通道集合与旁路保护 | A1只写配置 `channels` 对应的位掩码；OFF前记录4路状态，所选位OFF和恢复ON后都再次查询。未选择通道如发生变化立即中止并报 `NON_TARGET_STATE_CHANGED`；本现场选择全部1～4路，因此没有旁路输出 |
| 影响范围 | 任一成员触发后，`channels: [1,2,3,4]` 对应的4台雷达都会短暂断流；不会尝试伪装成“只重启一台” |
| 断电时间 | 所选通道全部确认OFF后按有效 `off_seconds` 保持，再将全部选中位恢复ON；新模板和完成上述迁移的配置为5秒，旧配置省略该字段或显式写10时仍为10秒；配置下限是5秒，不能设得更短 |
| 恢复确认 | ON 确认后最多等 180 秒，只接受该次 ON 之后、来自同一 Driver 的新状态；要求组内 4 台全部已连接、握手为 `IDLE`，并在 `Normal + Sampling + publishing` 状态连续健康 10 秒；`PowerSaving/StandBy` 不作为本次硬恢复完成的验收状态 |
| 冷却 | 同一个物理继电器端点及通道集合两次真实或已发送但无法确认的断电至少间隔30分钟，不按触发成员分别计时；若在第一条OFF命令前明确取消，则不占预算 |
| 熔断 | 同一个物理继电器端点及通道集合24小时最多3次真实或无法排除已发生的断电；达到上限只告警，不继续循环断电；明确未发送OFF的取消不计次数 |
| 安全计时 | 冷却/24 小时预算使用 SQLite 持久化的单调逻辑时钟；重启不计作“时间已经过去”，修改系统时间或重启服务不能提前清空预算 |
| 并发 | 同组多台同时异常会合并为该物理端点的一次 OFF/ON；首个循环开始后到达的重复事件由事件去重和端点冷却共同抑制，跨组也由单工作线程串行执行 |
| 稳定身份 | SQLite持久绑定 `power_group` 与继电器IP/端口/地址/规范化通道集合；旧单通道identity原样兼容，新增多通道identity包含排序后的集合。改组名或改接端点/集合会fail closed，不能借改配置清空安全预算 |
| 单写锁 | 除状态库单实例锁外，再按物理继电器端点持有固定的OS文件锁；同一主机、同一运行用户下，不同配置/数据库的第二个manager也不能同时写同一继电器；其他本机用户、GUI或另一主机不遵守该锁，须靠账户权限与防火墙只允许正式守护进程访问继电器端口 |
| 断电后异常 | ON 无法确认时保留持久化 obligation，每 30 秒继续尝试并发出 CRITICAL；systemd 启动前先独立补 ON，配置损坏也不会跳过；补 ON 未完成前禁止任何新 OFF |
| 告警存续 | 每个物理端点的活动 CRITICAL 独立存入 SQLite，重启后在 `MANAGER_READY` 之后重新发布；只有该端点后续完成 `RECOVERY_VERIFIED` 才自动清除 |

配置和状态均在仓库外：更新 Driver 不会覆盖 `~/.config/livox/power_cycle.json`。生产安装把审计/去重数据库固定为 `~/.local/state/livox-power-cycle-manager/state.sqlite3`，配置中的 `state_db` 必须解析到同一路径。不要删除、替换或手工修改 SQLite，否则会丢失冷却预算和补上电义务；v2/v3 会保守补入旧原因字段，v4 会在一个 `BEGIN IMMEDIATE` 事务内原子重建两张带 reason 约束的表并升级到 v5，从而允许四种原因。任一步失败会整体回滚；未知、损坏或 legacy 结构仍严格拒绝，不会静默重建。

卸载同样不是直接删文件：先把 launch 开关改回 `false` 并安全停止 Driver，再执行 `bash "$HOME/catkin_ws/src/livox_ros_driver/install_livox_power_cycle_service.sh" --uninstall`。脚本只接受 Driver 已处于 `inactive/failed`，独立补 ON 成功后才删除 Driver drop-in；任何一步失败都会保留安全钩子，现场 JSON 和 SQLite 始终保留。

查看自动硬恢复的最近状态可继续使用同一看板。看板最上方 `DATA SOURCE` 用本机单调时钟显示 Driver topic 的接收年龄：超过 5 秒没有新 `/livox/lidar_stats` 会明确显示 `NOW=DRIVER_STALE severity=CRITICAL`，不会用旧表和新的渲染时间伪装成实时数据。脚本自身每秒刷新，因此 Driver 和 manager 同时停发时 stale 年龄仍会继续增长。

`livox_stats_monitor.py` 会在顶部 `DATA SOURCE` 常驻 `POWER-MGR` 摘要：首帧到达前显示 `NOT_SEEN/WARN`，收到至少一条通过校验的manager消息后显示实际状态，并在Driver看板之后追加独立的 `POWER RECOVERY` 板块。详情中的 `MANAGER` 行显示manager的 `NOW/severity/manager_age`，每个 `GROUP <power_group>` 再分行显示该共享组的 `NOW/severity/rx_age/trigger/members/relay` 和完整 `detail`；`relay=1,2,3,4` 可直接确认本次状态对应四路集合，结构化status同时保留 `relay_channels` 和四种 `recovery_reason`。白名单外事件显示为 `UNMAPPED trigger=...`，绝不会伪装成manager行。这部分来自独立manager进程，不计入Driver的逐台 `ASSESS` 和 `PROCESS HISTORY`；收到首帧后若manager心跳超过30秒未接收，顶部摘要和底部详情都会明确改显 `MANAGER_STALE/CRITICAL`。所有stale判定都用本机接收时刻，不信任消息内wall-clock。

```bash
rostopic echo /livox/power_cycle_status
```

```bash
rostopic echo /livox/power_cycle_heartbeat
```

维护时若要用图形化科星调试软件手工改变同一台继电器，必须在维护窗口先停止整个 Driver 服务（会中断点云；停止后安全钩子会补 ON），关闭 GUI 后再恢复 Driver，保证现场始终只有一个控制写入者：

```bash
sudo systemctl stop livox-ros-driver
```

```bash
sudo systemctl start livox-ros-driver
```

> 继电器返回的ON/OFF是控制器逻辑状态，不是负载端电压/电流反馈。正式武装前必须在有人值守的维护窗口验证“关闭配置的整个通道集合时，恰好是配置中的4个broadcast code全部消失、没有其他设备掉电；恢复集合ON后4台点云全部恢复”，并确认所有选中通道正常状态均为ON。本现场已确认集合为1～4路。若需要证明接触器没有粘连，应增加独立电压/电流反馈，软件不能凭TCP状态替代该硬件证据。

> 对真正长期无人值守的现场，电气层最好再做成硬件看门狗/时间继电器控制的**单稳态断电脉冲**：OFF 最长 10 秒后由硬件自动回 ON，并实测控制器掉电、主机死机和网络中断时的默认状态也是 ON。SQLite 补上电只能覆盖软件进程重启，不能替代这层硬件失效保护。

> **armed 的电气硬前置**：按 4 台雷达同时冷启动的实测峰值核算浪涌和稳态总电流；继电器触点/外接接触器必须满足实际直流电压、直流分断能力和负载类型，不能只看交流额定值；电源余量、线缆截面积、端子、保险/断路器及压降均须覆盖 4 台合计负载。任何一项未由电气工程师验收，都必须保持 launch 开关为 `false`。

> ROS 1 topic 本身没有认证。服务固定使用本机 `127.0.0.1:11311`，现场仍应把 ROS master 和继电器控制网放在受控 VLAN/防火墙内，只允许 manager 主机访问继电器端口，不要把 11311/50000 暴露到办公网或公网；白名单和状态复核都不能替代网络访问控制。

#### 可选：持久化健康日志（`health_log`，长期无人值守用）

看板和日志都是“当下/滚动”的，重启即失。开了它会把健康状况**落盘成 CSV**，供事后做周/月级趋势分析与故障取证。**默认关闭。**

```bash
roslaunch livox_ros_driver livox_lidar_multi.launch health_log:=true
```

可选：自定义目录与快照周期，命令保持单行：

```bash
roslaunch livox_ros_driver livox_lidar_multi.launch health_log:=true health_log_dir:=/data/livox_logs health_log_snapshot_s:=600
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `health_log` | `false` | 总开关 |
| `health_log_dir` | 空（= 节点工作目录 `~/.ros`）| 落盘目录，**需已存在** |
| `health_log_snapshot_s` | `600` | 快照周期（秒）|

写**两条流**，文件名带日期、**按天自动分文件**：

- **`livox_events_YYYY-MM-DD.csv`（事件，边沿触发）**：一旦发生就记一行 —— 健康位变化（`HEALTH`，附完整解码）、掉线/重连（`DISCONNECT`/`RECONNECT`，附 down 时长）、自动重启（`REBOOT`）、**断流/恢复（`NODATA`/`DATABACK`）**：一台 `Normal` 雷达持续无数据满 3 秒就记一条 `NODATA`（**带精确时刻，方便和上位机/调度器日志对时间，看清"何时开始哑的"**），恢复出数据时记 `DATABACK`、`detail` 写 `silent Ns`（恢复前哑了多久）；若期间掉线，则由 `DISCONNECT` 那行接手。**秒级、不漏任何短瞬故障**（哪怕几秒就自愈的 motor 故障）。列：`wall_time,handle,bcode,event,detail`。
- **`livox_snapshot_YYYY-MM-DD.csv`（快照，每 `N` 秒）**：每台一行，带当前状态、`disc`（本次 Driver 进程累计）以及**当前 SDK 连接生命周期内累计**的 `recv_total/loss_total/drop_total/loss_pct`。同一 broadcast code、同一连续连接内的相邻两行可以相减；遇到 `DISCONNECT/RECONNECT/STARTUP` 边界或后值小于前值时必须开始新分段，不能跨重连把清零后的计数直接相减。按这些边界分段后可汇总周/月网络趋势、定位 EMI 规律。列：`wall_time,handle,bcode,state,temp,fan,motor,dirty,system,recv_total,loss_total,drop_total,loss_pct,disc`。

> 占用极小（4 台、600s 快照 ≈ 0.5 MB/天，事件仅在变化时才写）。打不开文件会**告警一次并自动禁用**，绝不拖垮驱动。事件流秒级捕捉离散故障，快照流按连续连接分段记录网络趋势，两者互补。

> **同一天多次启停 → 自动合并进同一个文件**：文件名只按日期、以**追加**模式打开，所以当天反复结束/重启都接在同一个 `..._YYYY-MM-DD.csv` 里（不覆盖、不重复表头、不多生成文件），跨天才建新文件。每次驱动启动会写一行 `STARTUP` 事件，便于在合并文件里区分各次运行的边界。

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

**必须使用下面这个精确版本，不能用官方 SDK 或仅凭同名静态库判断：**

- fork：`https://github.com/85256638/Livox-SDK.git`
- branch：`network-relay-added`
- commit：[`e45774c5d4f2edab96dd6d61479167784d7df8c9`](https://github.com/85256638/Livox-SDK/commit/e45774c5d4f2edab96dd6d61479167784d7df8c9)

### 配套 SDK 提供的保证

1. mode 2/3 命令**实际发送成功时**立即开启独立 15 秒 transition deadline；已处于 PowerSaving / Standby 时也使用 15 秒阈值，Normal 稳态仍是 3 秒。
2. heartbeat ACK 必须带完整 `HeartbeatResponse` 才能刷新连接或上报状态；短载荷安全拒绝。
3. 异步 API 返回 `kStatusSuccess` 后，ACK、timeout、发送失败、断线、queued-but-unsent 取消和全局 `Uninit()` 路径中必有且仅有一次终态 callback。
4. command payload 使用 RAII；断线清队列不会泄漏 Driver 的 callback context。ACK 同时核对 seq、command set 和 command id。
5. LiDAR channel 查找/移除有同步；从 I/O callback 内断线时，channel 会保留到 raw delegate 真正移除后再析构，避免当前 callback 尚未返回就释放对象。
6. 修复零长度协议 payload 的空指针 `memcpy` UB 和 `<memory>` 直接依赖缺失。
7. 同一雷达最多一个 pending handshake；握手失败不再无限递增 `port_count`，端口固定在按 handle 分配的有限区间，长期故障不会 16 位回绕。
8. 提供 `ResetLidarHandshakeSession(broadcast_code)`，在 SDK I/O 线程定向清理 pending 或 DeviceInfo 未完成的 provisional session，真正已 Connect 的设备拒绝清理。
9. 提供握手诊断 callback，区分 timeout、设备拒绝、协议错误、本机 socket/network 错误和显式 reset，并携带 ret_code/errno/detail。
10. 只有 DeviceInfo 成功才公开 `kEventConnect`；半连接清理不发假 Disconnect，`GetConnectedDevices` 也不暴露 provisional 设备。
11. GNU/GCC 构建不再携带 Clang 专用告警参数，固定长度诊断字段也避免触发 GCC 9 的 `-Werror=stringop-truncation`。

Driver 端的 context registry 只释放 SDK 已明确 callback/cancel 完成的 context，并保留 60 秒 tombstone 防御重复/迟到 callback 的地址复用；它不会凭“过了 N 秒”释放仍可能被 SDK 持有的裸指针。

### CMake 如何保证没有链错 SDK

- 默认只在 catkin build 目录的 `_deps` 下克隆上述 fork/branch，并 detach 到固定 commit。
- SDK 作为 `livox_sdk_static` CMake target 构建和链接，不再使用裸 `livox_sdk_static.a` 名称，也不探测 `/usr/local/lib`。
- 不再执行源码树内 `rm -rf Livox-SDK`，也不会 fallback 到官方默认分支。
- 显式传 `LIVOX_SDK_SOURCE_DIR` 时会校验 Git HEAD 和 tracked 工作树；不匹配即 configure 失败。

首次构建需要访问 GitHub。离线环境先准备正确 checkout：

```bash
git clone --branch 'network-relay-added' --single-branch https://github.com/85256638/Livox-SDK.git ~/Livox-SDK-pinned && git -C ~/Livox-SDK-pinned checkout --detach e45774c5d4f2edab96dd6d61479167784d7df8c9 && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DLIVOX_SDK_SOURCE_DIR=$HOME/Livox-SDK-pinned
```

### SDK 关闭约束

- 调用 SDK 全局 `Uninit()` 前先停止新的 SDK API 调用。
- 不要从 SDK I/O callback 内直接调用 `Uninit()`，应调度到外部线程，避免线程自 `Join()`。

---

## 修改文件清单

### Livox SDK（配套仓库）

| 文件组 | 改动 |
|------|------|
| `sdk_core/src/command_handler/command_channel.*` | heartbeat 校验、transition deadline、exactly-once completion/cancel、payload RAII、ACK identity |
| `sdk_core/src/command_handler/*_command_handler.*` | channel 容器同步、安全 detach 与 delegate 移除后的延迟回收 |
| `sdk_core/src/comm/sdk_protocol.cpp` | 零长度 payload UB 与非法 payload 校验 |
| `sdk_core/src/base/thread_base.h` | `<memory>` 直接依赖 |
| `sdk_core/src/device_discovery.*` | 握手去重、有界端口、timeout/errno/ret_code 诊断，以及 I/O 线程内定向 session reset |
| `sdk_core/src/device_manager.*` | provisional/ready 分层；DeviceInfo 成功后才公开 Connect；半连接静默清理 |
| `sdk_core/src/base/network/*/network_util.cpp` | socket 创建失败时保留真实 errno/WSA error，供诊断上报 |

### ROS Driver

| 文件 | 改动 |
|------|------|
| `srv/LidarMode.srv` | **新增** — 模式切换 Service 定义 |
| `srv/LidarReboot.srv` | **新增** — 重启 Service 定义 |
| `CMakeLists.txt` | 注册两个 srv，并链接固定 SDK CMake target |
| `cmake/pinned_livox_sdk.cmake` | 固定 SDK fork/branch/SHA，校验 clean checkout，fail closed |
| `livox_ros_driver/lds_lidar.h/.cpp` | 模式切换 + 重启 + 批量 Normal 错峰/ACK grace/有界重发 + 每台配置链串行 + 状态机抖动修复 + 广播/握手状态机、session reset 与严格 `WAKE_DROPOUT` 归因 |
| `livox_ros_driver/livox_ros_driver.cpp` | 模式/重启 Service、AsyncSpinner、max_distance 参数、五类自动恢复调度、分层 `livox/lidar_stats` 看板，以及显式停止 timer/spinner 后的正常关闭 |
| `livox_ros_driver/dashboard_metrics.h` | **新增** — 按 broadcast code 隔离的 60 秒/10 分钟滚动窗口与 `ASSESS` 纯判定逻辑；握手与 wake-dropout episode 分开计数 |
| `livox_ros_driver/recovery_event_json.h` | Driver→manager 的结构化恢复状态/请求；显式携带 `recovery_reason` 和原因特定证据 |
| `livox_ros_driver/lddc.h/.cpp` | 距离过滤 + 读取端 UAF 加锁 |
| `livox_ros_driver/lds.h/.cpp` | 每雷达锁、丢包统计（仅异常打印）、`data_type` 硬化、写入端 UAF 加锁 |
| `livox_ros_driver/ldq.cpp` | 队列释放置空 + 操作空指针兜底 |
| `timesync/timesync.h/.cpp` | TimeSync 初始化/停止幂等化；退出标志原子化；先 stop/join 再 SDK `Uninit()` |
| `timesync/user_uart/user_uart.h/.cpp` | UART Open/Close/Read 串行；空闲读取有界返回；完整检查 termios/fcntl/read 错误 |
| `scripts/livox_power_cycle_manager.py` | 原因特定的实时复核、共享组 OFF/ON、SQLite 去重/冷却/补上电义务和四台持续健康验收 |
| `scripts/livox_stats_monitor.py` | 独立终端实时看板，并分区显示共享继电器 manager/group 的最新状态与恢复原因 |

---

## 常见问题

### Q: 切到节电模式后立即自动恢复 Normal？
查看 catkin configure 日志是否明确打印固定 SHA `e45774c...`。本分支不需要 `sudo make install` SDK；若仍链接到系统库，说明运行的不是这份 CMake/工作区。清理对应 catkin build 缓存后重新 `catkin_make`，不要只重编译旧 build 目录里的另一份源码。

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
若 Driver 仍已连接该雷达且命令通道可用，可调用软重启 service：`rosservice call /livox_lidar_reboot "{handle: 255}"`（255 = 全部）。对 `HANDSHAKE_STUCK / WAKE_DROPOUT / NORMAL_DROPOUT / STARTUP_MISSING` 应使用本章的自动闭环；未满足原因证据的短暂 `DISCONNECTED` 不应手工高频断电。

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

Before running this customized branch, ROS must be installed. The build fetches
and links the pinned companion Livox-SDK itself; do not substitute an official
or system-installed library.

### 1.1 ROS installation

For ROS installation, please refer to the ROS installation guide :

[ROS installation guide](https://www.ros.org/install/)

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;(1) Be sure to install the full version of ROS (ros-distro-desktop-full);

&ensp;&ensp;&ensp;&ensp;(2) There are 7 to 8 steps in ROS installation, please read the installation guide in detail;

### 1.2 Pinned Livox-SDK

CMake uses `85256638/Livox-SDK`, branch `network-relay-added`, commit
`e45774c5d4f2edab96dd6d61479167784d7df8c9`. It clones into the build directory
and links the CMake target directly. A local checkout may be supplied with
`-DLIVOX_SDK_SOURCE_DIR=/absolute/path`, but configure fails unless its HEAD and
tracked worktree match the pin.

## 2. Get and build livox_ros_driver

1. Get livox_ros_driver from GitHub :

　　`git clone --branch 'network-relay-added' --single-branch https://github.com/85256638/livox_ros_driver.git ws_livox/src/livox_ros_driver`

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
