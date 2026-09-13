# 小车任务重构：下一轮操作手册与安全交接

更新日期：2026-09-10

## 1. 目标、工程边界和安全原则

本工程重构室内小车的激光定位、固定路线跟踪、局部重规划和安全停车。不使用机械臂、腕部相机、抓取、放置或红绿灯流程。

| 内容 | 位置 | 规则 |
|---|---|---|
| 原比赛工程 | `/home/agilex/competition_ws` | 保持不修改 |
| 重构工程 | `/home/agilex/competition_rebuild_ws` | 所有新代码、配置和日志放这里 |
| Windows 交接副本 | `D:\\CodexProjects\\小车任务重构` | 本 README 与脚本的保存位置 |

禁止直接向 `/cmd_vel` 发布速度。真实底盘唯一允许的链路为：

```text
MPPI -> Safety -> ranger_twist_adapter -> /cmd_vel -> Ranger
```

当前尚未进行自主实车运动。本手册中的“干跑”绝不会启动 `ranger_twist_adapter`，输出只能进入 `/dry_run/cmd_vel_safe`。

## 2. 当前冻结的事实

### 2.1 地图、路线和轨迹

当前有效文件：

```text
/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml
/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.pgm
/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/semantic_map_six_anchor_v3.yaml
/home/agilex/competition_rebuild_ws/routes/indoor_manual_six_anchor_v3_loop.yaml
/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_six_anchor_v3_continuous_trajectory.yaml
```

已离线验证：

- 五段全局规划均通过。
- 连续轨迹：3127 点、约 156.020 m。
- 第一轮速度上限：`0.10 m/s`。
- 当前地图是人工清理后的静态 PGM，必须依靠实时激光与 AMCL 复核，不能把人工填白区域当作已实车证明的可通行区域。

### 2.2 Ranger CAN

当前无机械臂小车的实测配置：

```text
Ranger interface: can2
bitrate: 500000 bit/s
```

旧比赛文档的 `can3` 和 `can2=1 Mbit/s` 属于带 Piper 机械臂的旧配置，不能照搬。实测仅 `can2` 存在持续 Ranger CAN 回包。

### 2.3 定位基准

已验证 FAST-LIO、`/localization/scan`、静态地图、AMCL 与 `map -> body` TF。

车放在标记起点、经 RViz 对齐后，静止 20 秒的参考结果为：

```text
x: -16.083 m
y:  2.309 m
yaw: -179.4 deg
```

该数值只适用于同一物理起点和相同车头朝向。每次运行仍必须发布初始位姿，并在 RViz 中确认白色实时激光点贴合黑色地图墙线。

### 2.4 已验证的干跑与急停

`rebuild_navigation_dry_run.launch.py` 已验证：

- 默认 `route_enable=false` 时：`ROUTE_DISABLED`，输出全零。
- 发布 `/mission/route_enable=true` 后：MPPI 为 `TRACKING`，`/dry_run/cmd_vel_safe` 约为 `0.089 m/s`，Safety 为 `SAFE_ACTIVE`。
- 发布 `route_enable=false` 后：输出归零。
- 干跑没有真实 `/cmd_vel` 发布者。

`rebuild_proximity_dry_test.launch.py` 已验证：

- 由 `/cloud_registered_body` 生成 `/scan`。
- 固定纸箱在车头正前方约 `0.5-0.7 m` 时，`/avoidance/stop_request=true`。
- 移开纸箱后，`/avoidance/stop_request=false`。

当前控制器对避障请求采用临时保守策略：触发后自动解除连续路线，避免没有局部绕行能力时自行恢复。

## 3. 终端分工

以下“终端”均指工控机 Ubuntu 图形桌面或 Windows SSH 登录后的 Linux shell。每个 ROS 终端都要先执行环境加载命令：

```bash
conda deactivate 2>/dev/null || true
source /home/agilex/competition_ws/scripts/car_source_env.sh
source /home/agilex/competition_rebuild_ws/install/setup.bash
```

| 终端 | 只做什么 | 何时关闭 |
|---|---|---|
| A：传感器与底盘 | CAN、Livox、FAST-LIO、Ranger 底盘驱动、`/odom` | 全部测试结束时最后关闭 |
| B：地图与 RViz | 静态地图、RViz、人工观察激光贴图 | 定位检查结束后可保留到测试结束 |
| C：定位扫描 | 点云转 `/localization/scan` | 全部测试结束时关闭 |
| D：AMCL | 配置、激活 AMCL、发布初始位姿 | 全部测试结束时关闭 |
| E：一键干跑 | 运行脚本，只产生隔离的干跑速度 | 用停止脚本结束 |
| F：诊断 | 只读检查话题、TF、状态和日志 | 随时可开关 |

不要在 A-D 的运行终端继续输入新命令；这些终端应保持节点运行。所有检查命令在 F 或另开的普通终端执行。

## 4. 下一次从开机到定位就绪

### 4.1 终端 A：配置 CAN 并启动传感器/底盘

先检查没有旧的重构节点。确认无误后才继续：

```bash
pgrep -af \
'rebuild_mapping|rebuild_final_map|nav2_amcl|ranger_base_node|mppi_control_node|safety_node|pointcloud_to_laserscan|proximity_stop_node|fast_lio|livox_ros_driver2' \
|| true
```

配置当前 Ranger CAN：

```bash
sudo ip link set can2 down 2>/dev/null || true
sudo ip link set can2 type can bitrate 500000 restart-ms 100
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up

ip -details -statistics link show can2
timeout 3s candump -L can2
```

必须看到 Ranger CAN 回包；完全无回包时停止，检查急停、遥控器、底盘供电、USB-CAN 和接口连接。

然后启动 Livox、FAST-LIO 与 Ranger 底盘驱动：

```bash
conda deactivate 2>/dev/null || true
source /home/agilex/competition_ws/scripts/car_source_env.sh
source /home/agilex/competition_rebuild_ws/install/setup.bash

ros2 launch rebuild_bringup rebuild_mapping.launch.py \
  start_livox:=true \
  force_livox_host_timestamps:=true \
  start_fast_lio:=true \
  start_base:=true \
  port_name:=can2 \
  publish_odom_tf:=false \
  start_scan:=false \
  start_slam:=false \
  start_anchor:=false \
  rviz:=false
```

目的：提供 `/cloud_registered_body`、IMU/FAST-LIO 里程计、Ranger `/odom` 和 `/system_state`。它不会启动底盘速度适配器。

### 4.2 终端 B：打开地图和 RViz

```bash
conda deactivate 2>/dev/null || true
source /home/agilex/competition_ws/scripts/car_source_env.sh
source /home/agilex/competition_rebuild_ws/install/setup.bash

ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:=/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml \
  rviz:=true
```

目的：显示静态地图、实时激光、TF 和后续轨迹。RViz 的固定坐标系必须为 `map`。

### 4.3 终端 C：建立 AMCL 使用的二维激光

```bash
conda deactivate 2>/dev/null || true
source /home/agilex/competition_ws/scripts/car_source_env.sh
source /home/agilex/competition_rebuild_ws/install/setup.bash

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r __node:=rebuild_localization_scan \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/pointcloud_to_laserscan_rebuild_localization.yaml \
  -r cloud_in:=/cloud_registered_body \
  -r scan:=/localization/scan
```

目的：把三维点云投影为 AMCL 订阅的 `/localization/scan`。正常频率约 10 Hz。

### 4.4 终端 D：启动并初始化 AMCL

先启动 AMCL：

```bash
conda deactivate 2>/dev/null || true
source /home/agilex/competition_ws/scripts/car_source_env.sh
source /home/agilex/competition_rebuild_ws/install/setup.bash

ros2 run nav2_amcl amcl \
  --ros-args \
  -r __node:=amcl \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/amcl_rebuild_indoor.yaml
```

再在终端 F 配置并激活它：

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
ros2 lifecycle get /amcl
```

最后，车已经放在固定起点时，在 F 发布初始位姿：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: -16.083, y: 2.309, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.99999, w: 0.00524}}, covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.10]}}"
```

目的：建立 `map -> body`。随后必须在 RViz 观察白色实时激光点是否贴住黑色墙线；不贴合时用 RViz 的 **2D Pose Estimate** 微调，等待稳定后再检查。

## 5. 终端 F：每次定位完成后的只读检查

```bash
timeout 5s ros2 topic hz /cloud_registered_body
timeout 5s ros2 topic hz /localization/scan
timeout 5s ros2 topic echo /amcl_pose --once
timeout 5s ros2 topic echo /odom --once
timeout 5s ros2 run tf2_ros tf2_echo map body
ros2 topic info /cmd_vel -v
```

可以进入下一步的最低条件：

- `/cloud_registered_body` 与 `/localization/scan` 都在更新。
- `/amcl_pose` 有输出，`map -> body` 可连续查询。
- RViz 激光与地图贴合，不跳变。
- 在尚未启动真实适配器时，`/cmd_vel` 的发布者数量必须为 0。

## 6. 终端 E：一键干跑预检

脚本文件：

```text
/home/agilex/competition_rebuild_ws/scripts/start_dry_safety_test.sh
/home/agilex/competition_rebuild_ws/scripts/stop_dry_safety_test.sh
```

先复制 Windows 中已写好的两个脚本到工控机，并给予执行权限。脚本前提为 A-D 已完成：

```bash
chmod +x /home/agilex/competition_rebuild_ws/scripts/start_dry_safety_test.sh
chmod +x /home/agilex/competition_rebuild_ws/scripts/stop_dry_safety_test.sh

/home/agilex/competition_rebuild_ws/scripts/start_dry_safety_test.sh
```

脚本会检查：

1. 没有旧的干跑 launch。
2. 真实 `/cmd_vel` 没有发布者。
3. `/cloud_registered_body`、`/odom`、`/amcl_pose` 均有数据。
4. 在后台启动近距停障和 MPPI/Safety 干跑。
5. 输出只允许进入 `/dry_run/cmd_vel_safe`。

需要验证控制链时，在 F 执行：

```bash
ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: true}"
sleep 1
timeout 3s ros2 topic echo /control/status --once --full-length
timeout 3s ros2 topic echo /dry_run/cmd_vel_safe --once
timeout 3s ros2 topic echo /safety/event --once

ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: false}"
```

目的：验证 MPPI 与 Safety 的计算链，不移动小车。完成后停止干跑：

```bash
/home/agilex/competition_rebuild_ws/scripts/stop_dry_safety_test.sh
```

## 7. 自动避障目标与当前缺口

最终运行逻辑应为：

```text
等待人工确认发车
-> 自主路线跟踪
-> 静态障碍：局部重规划、自动绕行
-> 动态障碍：减速或停车
-> 障碍离开：重新规划并在安全条件满足后自动恢复
-> 到达终点停车
```

人工只负责首次发车确认。行驶期间不应依赖人工发送“继续”指令。

但当前 `proximity_stop_node` 只会提出停车请求，不会绕障。因此在完成局部重规划器并验证前，当前的避障锁存策略不能移除。局部重规划和自动恢复至少要满足：

- 连续若干帧确认障碍不在近距安全区；
- 定位、点云和 Safety 输入未超时；
- 新路径不碰撞、位于语义走廊内且满足转弯半径；
- 无法生成安全路径时继续停车并报告原因。

## 8. 什么时候可以实车启动

现在还不能直接发车。必须完成以下项目：

1. 创建统一 `rebuild_navigation_guarded.launch.py`：统一启动点云投影、近距停障、MPPI、Safety；底盘适配器默认关闭。
2. 启用并干跑验证局部重规划：静态绕障、动态障碍离开后的安全恢复。
3. 验证故障分支：定位丢失、雷达超时、CAN/遥控器未就绪、近障急停。
4. 统一入口保持 `start_chassis_adapter:=false` 默认值；只有经人工复核才允许显式接入真实 `/cmd_vel`。
5. 在空旷场地完成首车：`0.10 m/s`、仅前进 `0.5-1.0 m`、急停与遥控器可用、人员全程监护。
6. 首车稳定后分段扩大距离，最后才尝试整圈和狭窄通道。

## 9. 结束测试

在每一个运行 `ros2 launch` 或 `ros2 run` 的终端按 `Ctrl+C`，按以下顺序结束：

1. 先停止一键干跑脚本创建的进程。
2. 停止 AMCL、定位扫描、地图/RViz。
3. 最后停止传感器与底盘终端 A。

不要使用宽泛 `pkill`，不要直接控制 `/cmd_vel`。结束后可检查：

```bash
pgrep -af \
'rebuild_navigation_dry_run|rebuild_proximity_dry_test|rebuild_mapping|rebuild_final_map|nav2_amcl|ranger_base_node|mppi_control_node|safety_node|pointcloud_to_laserscan|proximity_stop_node|fast_lio|livox_ros_driver2' \
|| true
```

## 10. 2026-09-11 实车验证更新

本节以现场结果为准，覆盖前文中“尚未实车运动”的旧表述。

### 已经通过

- Ranger 实车接口确认使用 `can2`、`500000 bit/s`。重新构建
  `/home/agilex/agilex_ws` 中的 `ugv_sdk` 与 `ranger_base` 后，底盘可稳定报告
  `control_mode: 1`、`error_code: 0`。
- FAST-LIO、`/localization/scan`、静态地图、AMCL 与 `map -> body` 已联调通过；
  发车前仍必须在 RViz 确认实时激光贴墙稳定。
- 0.8 m 低速直行实车通过。
- 0.8 m 动态障碍实车通过：障碍物出现时停车，移开后无需再次发送路线使能即可恢复前进。
- 3 m 静态障碍实车通过：局部 Hybrid A* 生成绕行路径，车辆选择更开阔一侧绕过障碍物。
- 10 m 无障碍低速实车完成，现场观察无碰撞且路线运动正常。

### 当前软件入口与测试资产

```text
/home/agilex/competition_rebuild_ws/src/rebuild_bringup/launch/rebuild_navigation_replan_guarded.launch.py
/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_first_motion_0p8m_trajectory.yaml
/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_first_detour_3m_trajectory.yaml
/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_first_baseline_10m_trajectory.yaml
/home/agilex/competition_rebuild_ws/scripts/start_replan_dry_test.sh
/home/agilex/competition_rebuild_ws/scripts/stop_replan_dry_test.sh
/home/agilex/competition_rebuild_ws/scripts/run_guarded_progress_test.sh
```

`rebuild_navigation_replan_guarded.launch.py` 默认不接真实底盘；只有显式传入
`start_chassis_adapter:=true` 才会启动唯一允许的适配器：

```text
/cmd_vel_safe -> ranger_twist_adapter -> /cmd_vel -> ranger_base_node
```

### 仍需处理的现象

- 10 m 测试结束后的一个状态样本出现 `INVALID_STATE` 和 `position_jump`。
  这通常是定位位姿跳变，不能直接当作障碍物检测。下一次长距离测试前，必须重新在
  RViz 检查激光贴图，并记录测试过程是否再次出现该状态。
- `/planning/local_stop_request` 曾出现瞬时 `true`，而稍后的
  `/planning/local_replan_status` 为 `REPLANNED` 或 `REFERENCE_CLEAR`。它可能来自
  实际障碍、局部规划超时、无可行路径或点云/定位短暂不一致；在抓到同一时刻的
  `detail` 前，不削弱 Safety 停车逻辑。
- 156 m 整圈尚未实车测试。

## 11. 下一次恢复与快速推进

### 恢复顺序

1. 检查网络并 SSH 登录工控机；若工控机重启，重新配置 `can2` 为 500 kbit/s。
2. 启动终端 A-D：传感器/底盘、地图/RViz、二维激光、AMCL。
3. 车放在起点后发布初始位姿，在 RViz 确认激光贴图稳定。
4. 只读确认：`control_mode: 1`、`error_code: 0`、`/cmd_vel` 没有非预期发布者、
   `/planning/local_stop_request=false`。
5. 启动 guarded 路径时先使用 `start_chassis_adapter:=false` 干跑；确认后才显式接入底盘。

### 加速而不跳过证据的路线

不再反复停留在 3 m。下一轮采用以下代表性门槛：

1. 复测 10 m 无障碍路线，记录过程中是否再出现 `position_jump`；现场到达终点后人工发送
   `route_enable=false`，不再使用固定等待时间判断完成。
2. 若定位稳定，直接扩大到约 30 m 分段，包含一次静态障碍绕行或动态障碍移开恢复。
   新分段采用约 `0.10 m` 点距，速度仍维持 `0.10 m/s`；不要同时提高速度和改变点距。
   这能减少目标点切换频率，又保留与既有低速实车测试一致的速度边界。
3. 30 m 分段连续通过后，评估整圈前的最终预检：定位稳定、局部停车原因可解释、
   Safety/底盘状态正常、人工接管人员在位。

若再次触发 `position_jump`，记录触发时的实际位姿差、预测时间和检测阈值，再决定是调整
控制器的跳变判定还是修复 AMCL/TF 时间一致性；不凭单次状态样本直接降低安全阈值。

整圈不是由单一时间阈值决定。若 `position_jump` 不再复现且 30 m 分段通过，最快可在同一
次后续现场测试中进入整圈监督评估；若再次出现定位跳变，应优先修复定位而非扩大路线。

## 12. 暂停与恢复

暂停前必须先发送：

```bash
ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: false}"
```

随后确认 `/cmd_vel_safe` 为零，并在 guarded 导航终端按 `Ctrl+C`。SSH 断开不保证所有
ROS 进程退出；车辆无人看管或准备断电时，应按第 9 节顺序停止 A-D 终端。

## 13. 2026-09-11 收尾：自动 CAN 发送待复核

本次 30 m 分段已由现场人员完成，但下一次不得直接扩大到 156 m 整圈。收尾时发现一个
需要先闭环的底盘通信问题：在 `/cmd_vel` 约 20 Hz 且非零的短时自动测试中，`/odom` 和
全部电机 RPM 保持为零；手动遥控时八个电机均能报告非零 RPM，底盘硬件、CAN 回传和电机
使能正常。

已确认的状态：

- `control_mode: 1` 是 CAN 控制，`motion_mode: 0` 是双 Ackermann，`driver_state: 64`
  是驱动使能位，且无低压、过热、过载或驱动器故障位。
- 自动测试期间 `/cmd_vel_safe` 与 `/cmd_vel` 均曾为非零，但 `can2` 的 TX 包计数在 3 秒
  内没有增长；RX 包计数持续增长。
- SDK 源码中的 `AsyncCAN::SendFrame()` 将调用者栈上的 `can_frame` 传给
  `async_write_some()`，缓冲区生命周期不安全。`ranger_messenger.cpp` 对
  `ranger_mini_v3` 的分支也有悬挂 `else`，会覆盖 Mini V3 的车辆参数。
- 现场已停止 guarded 导航；最后确认 `/cmd_vel` 发布者数为 0。

### 下次启动前的必做项

1. 使用干净终端，避免旧工作区覆盖重构包。验证 `competition_control`、
   `competition_safety`、`competition_planning` 与 `rebuild_bringup` 的包前缀均为
   `/home/agilex/competition_rebuild_ws/install`；`ranger_base` 应来自
   `/home/agilex/agilex_ws/install`。
2. 确认 `ugv_sdk` 与 `ranger_base` 的 CAN 发送修复已编译，并重启终端 A 的 Ranger 驱动。
3. guarded 导航接入底盘但保持 `route_enable=false`，先确认 `can2` TX 包计数增长。该检查
   只发送零速度，不会发车。
4. 只有 TX 增长、定位贴合、`/cmd_vel` 仅有唯一 guarded 适配器发布者时，才恢复 30 m
   低速分段验证。
