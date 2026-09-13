# 小车重构运行手册（当前有效）

更新：2026-09-13。本文不覆盖 `README.md`。它记录已经实际验证过的运行环境、转弯故障根因，以及 30 m 和约 155 m 整圈测试的最终运行流程。

## 1. 边界与紧急停止

重构工作区是：`/home/agilex/competition_rebuild_ws`。运行时只加载：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
```

该脚本确保 `competition_control`、`competition_safety`、`competition_planning`、`rebuild_bringup` 都来自重构工作区；`ranger_base` 来自 `/home/agilex/agilex_ws`。不要再在同一终端额外 source `/home/agilex/competition_ws/install/setup.bash`。

禁止直接发布 `/cmd_vel`，禁止发送 CAN 或电机层命令。自主运动唯一允许的链路为：

```text
连续参考轨迹 -> MPPI 控制器 -> Safety -> Ranger twist adapter -> /cmd_vel -> Ranger 底盘
```

任何异常、人员或障碍物进入测试区域、车没有按预期通过转弯时，先在普通诊断终端执行：

```bash
ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: false}"
```

这会撤销路线使能，Safety 输出归零。随后由现场人员使用遥控器处置车辆；不要用终端发布速度指令“修正”车辆位置。

## 2. 当前实车状态与已验证项

- Ranger：`can2`，500 kbit/s；底盘正常状态为 `control_mode: 1`、`error_code: 0`、约 50 V 电池电压。
- `ranger_twist_adapter` 已验证为唯一真实 `/cmd_vel` 发布者；CAN 发送计数随控制输出增长。
- FAST-LIO 约 10 Hz、IMU 约 200 Hz；AMCL、`map -> body` 已可用。
- 安全链、局部重规划、0.8 m、3 m、10 m 和旧几何的 30 m 分段均已经验证到不同程度。
- 旧 30 m 路线在第一处急弯长期停顿。定位与障碍物状态当时正常，车辆接近参考 xy 位置但航向落后约 33 度，Safety 最终报告 `heading_error_persistent`。这不是“必须逐个精确经过点”的证据。
- 2026-09-13 的 409.7 s rosbag 曾暴露 `HYBRID_ASTAR_TIMEOUT` 和 `COSTMAP_STALE`；缩短规划超时并调整数据时效后，这两项已不再是最新测试的首发故障。
- 最新 306.5 s `shortrejoin` rosbag 显示：`146.92 s` 首先发生 `lateral_error_persistent`，局部规划器直到 `194.99 s` 才首次报告无可行路径。局部规划失败是车辆已经转向不足、姿态偏离后的后果。
- 轨迹和实车 MPPI 的 `max_curvature_rate_1pmps` 已统一为 `2.50`，原先运行时为 `0.90` 的约束不一致已经消除。
- 同步 MPPI 曲率变化率后的 `curvrate250_20260913_155838` bag 进一步排除了底盘执行问题：入弯时 `body_cmd`、`cmd_vel_safe`、`cmd_vel` 和 `odom` 的曲率一致，约为 `-0.9 1/m`。Safety 在横向误差约 `0.12-0.17 m` 时进入恢复并把速度从约 `0.195 m/s` 压到 `0.05 m/s`；误差已回落到约 `0.05 m` 时仍触发 `lateral_error_persistent`，随后命令归零、车轮回正。
- 比赛原安全配置明确说明，过窄的恢复带会让正常 U 型转弯退化成低速爬行。其已验证阈值为 `0.20 m/20 deg` 进入恢复、`0.10 m/8 deg` 退出、`0.40 m/45 deg` 硬限制；重构版使用的 `0.12 m/15 deg`、`0.06 m/5 deg`、`0.20 m/25 deg` 过于保守，是当前已确认的主要故障源。
- 恢复比赛误差带、将低速恢复速度设为 `0.15 m/s` 并保持障碍停车及状态检查后，R=1.2 m 的 30 m 实车测试已在无人为动态干预下连续通过第一处转弯。
- 使用同一套 R=1.2 m 路径、`0.20 m/s` 速度、`2.50 1/(m*s)` 曲率变化率和比赛 Safety 恢复带后，约 155 m、3109 点的整圈轨迹已于 2026-09-13 完整实车跑通。

## 3. 当前有效路线

原始整圈轨迹四处转弯的最小半径约 0.62 m，曲率约 `1.613 1/m`。它对实际底盘过于紧，曾导致前轮回正、控制输出归零并等待。

R=0.9 m 路线保留为历史对照，不再作为当前实车候选：

```text
路线：     routes/indoor_manual_six_anchor_v3_radius090_loop.yaml
语义地图： maps/indoor_manual_final/semantic_map_six_anchor_v3_radius090.yaml
规划参数： config/planning/semantic_lidar_guarded_six_anchor_radius090.yaml
速度参数： config/planning/continuous_trajectory_radius090_020mps.yaml
30 m 轨迹：artifacts/indoor_manual_first_segment_30m_radius090_020mps_trajectory.yaml
```

R=0.9 m 的旧 30 m 基线轨迹曾在圆弧曲率切换点被默认曲率变化率限制到 `0.034 m/s`，会造成明显的进、出弯等待。它保留为对照，不再用于实车。

R=0.9 m 候选使用独立的速度参数和来源清单：

```text
速度参数：config/planning/continuous_trajectory_radius090_turnrate250_020mps.yaml
30 m 轨迹：artifacts/indoor_manual_first_segment_30m_radius090_turnrate250_020mps_trajectory.yaml
```

该轨迹有 604 点、长度 30 m、时长约 225.387 s、最小离散半径约 0.91 m、直线最高速度 0.20 m/s、转弯段速度约 `0.107-0.149 m/s`、终点速度为零。它将 `max_curvature_rate_1pmps` 从默认 `0.8` 提升到 `2.5`，消除了进/出弯的近零速度凹口，但未消除实车卡顿。其来源清单与上述文件匹配，不能使用普通 guarded launch 去加载该轨迹，否则 MPPI 会因 provenance 哈希不匹配退出。

当前通过 30 m 测试的是 R=1.2 m 候选：

```text
语义地图： maps/indoor_manual_final/semantic_map_six_anchor_v3_radius120.yaml
规划参数： config/planning/semantic_lidar_guarded_six_anchor_radius120.yaml
30 m 轨迹：artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_020mps_trajectory.yaml
Safety 参数：config/safety/safety_params_rebuild_020mps_turn_recovery.yaml
启动文件： rebuild_navigation_replan_radius120_competition_safety_guarded.launch.py
```

30 m 第一处转弯和约 155 m 整圈均已通过。当前整圈轨迹为：

```text
整圈轨迹：artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml
轨迹点数：3109
最高速度：0.20 m/s
实车结果：完整跑通一圈
```

## 4. 启动定位环境

在四个保持运行的终端中分别执行以下命令。不要向正在运行 launch 的终端输入诊断命令。

### A：底盘、Livox、FAST-LIO

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup rebuild_mapping.launch.py \
  start_livox:=true force_livox_host_timestamps:=true start_fast_lio:=true \
  start_base:=true port_name:=can2 publish_odom_tf:=false \
  start_scan:=false start_slam:=false start_anchor:=false rviz:=false
```

### B：地图与 RViz

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:=/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml \
  rviz:=true
```

RViz 固定坐标系为 `map`。确认激光点贴合静态地图。

### C：二维定位激光

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args -r __node:=rebuild_localization_scan \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/pointcloud_to_laserscan_rebuild_localization.yaml \
  -r cloud_in:=/cloud_registered_body -r scan:=/localization/scan
```

### D：AMCL

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 run nav2_amcl amcl --ros-args -r __node:=amcl \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/amcl_rebuild_indoor.yaml \
  -r scan:=/localization/scan
```

在单独的诊断终端配置并激活：

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

车辆回到起点且车头方向正确后，重新发布初始位姿：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: -16.083, y: 2.309, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.998045, w: 0.062506}}, covariance: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]}}"
```

初始的 `tf2_echo` 短暂报 frame 不存在可能是节点启动竞态；只要随后持续出现稳定的 `map -> body` 数据即可。

## 5. R=1.2 m 实车测试

现场必须有一名人员全程在车旁，能立即按遥控器急停或接管。确保测试区域没有人和非测试障碍物。

### E：启动受控导航

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup rebuild_navigation_replan_radius120_competition_safety_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_020mps_trajectory.yaml \
  start_chassis_adapter:=true
```

该终端必须保持运行。日志需出现 `MPPI ready`、`Local Hybrid A* ready`、`Safety exit ready`、`Ranger twist adapter ready`。

### F：启动前检查

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 topic info /cmd_vel -v
timeout 4s ros2 topic echo /system_state --once
timeout 4s ros2 topic echo /planning/local_stop_request --once
timeout 4s ros2 topic echo /control/status --once --full-length
timeout 4s ros2 topic echo /cmd_vel_safe --once
```

只有同时满足以下条件才能使能：

- `/cmd_vel` 恰有一个发布者 `rebuild_replan_ranger_twist_adapter`，且 Ranger 为订阅者。
- `control_mode: 1`、`error_code: 0`。
- `/planning/local_stop_request` 为 `false`。
- `/control/status` 为 `ROUTE_DISABLED`，`route_enabled: false`。
- `/cmd_vel_safe` 全零。

### 开始与结束

开始：

```bash
ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: true}"
```

正常起步后的预期是 `TRACKING`、`SAFE_ACTIVE`、`/cmd_vel_safe` 约 0.2 m/s。全程观察车辆，尤其是 s 约 23 至 25 m 的第一个转弯。

到达 30 m 终点或任一异常时结束：

```bash
ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: false}"
```

不要用固定 `sleep` 自动结束测试；现场观察和显式禁用更可靠。

### 已验证的整圈运行

完成 30 m 验证并重新回到固定起点、发布初始位姿后，将终端 E 的轨迹替换为整圈文件：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_competition_safety_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml \
  start_chassis_adapter:=true
```

日志必须显示 `continuous_points=3109`。发车前仍执行本节 F 的全部检查，并在独立终端记录：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 bag record \
  -o "/home/agilex/competition_rebuild_ws/log/full_loop_$(date +%Y%m%d_%H%M%S)" \
  /control/status /control/tracking_error /control/body_cmd \
  /planning/local_replan_status /planning/local_stop_request \
  /avoidance/stop_request /safety/event \
  /cmd_vel_safe /cmd_vel /odom /motion_state
```

整圈结束后必须再次发布 `route_enable=false`，确认 `/cmd_vel_safe` 全零，再停止 rosbag 和导航 launch。导航 launch 保持连接在前台；SSH 中断时进程退出并由控制链失联保护停车，重新连接后必须从 A 至 E 完整重启，不能从中断进度续跑。

## 6. 当前导航逻辑

1. **参考轨迹**：离线全局规划器从语义走廊与锚点生成连续 `x/y/yaw/curvature/v` 序列。轨迹点是离散采样的参考，不是要求车辆逐点“打卡”。
2. **MPPI 控制器**：利用实时 `map -> body`、`/odom` 和前方局部参考路径选择进度点并求出连续速度/转向命令。它允许一定横向和航向误差，不会因为未精确经过一个采样点而自行等待。
3. **局部 Hybrid A***：从实时点云构建 `local_costmap`。参考走廊清晰时发布 `REFERENCE_CLEAR`；障碍占据参考路段时生成局部绕行 `REPLANNED`；超时或不可行时发出 `/planning/local_stop_request=true`，要求控制器停车。
4. **Safety**：检查定位、里程计、底盘状态、速度、局部停车请求、近距障碍与持续横向/航向误差。任何硬条件不满足，输出 `/cmd_vel_safe=0` 并发布原因。它不负责设计转弯路径。
5. **Ranger adapter**：把通过 Safety 的 Twist 转发到真实 `/cmd_vel`，不自行决定何时停车或回正。

曲率从 `0` 到约 `1.099 1/m` 的单个 `0.047 m` 样本切换。轨迹参数化器和运行中的 MPPI 均已使用 `2.50 1/(m*s)`，消除了近零速度凹口和运行约束不一致。R=1.2 m 几何配合比赛 Safety 恢复带已经完成整圈实车验证。

最新 rosbag 的故障顺序为：控制器和底盘正常进入圆弧 -> 重构版 Safety 过早进入低速恢复 -> 恢复状态未及时退出 -> `lateral_error_persistent` 将命令归零 -> 车辆停住并回正 -> 后续 Hybrid A* 因姿态偏离而间歇无解。动态障碍物改变 costmap 后偶然出现 `REPLANNED`，不是恢复导航的方法。当前应恢复比赛验证过的跟踪误差带，同时保留近距障碍停车、数据时效、底盘状态和控制模式检查；若仍失败，再启用“参考轨迹优先、持续障碍才触发 Hybrid A*”的备用模式。

## 7. 转弯问题的下一步证据采集

下一次在第一个转弯出现减速、回正或停顿时，若车辆已经安全静止、人员与车保持安全距离且现场没有碰撞风险，先在诊断终端抓取一次实时状态：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

timeout 4s ros2 topic echo /control/status --once --full-length
timeout 4s ros2 topic echo /control/tracking_error --once --full-length
timeout 4s ros2 topic echo /planning/local_replan_status --once --full-length
timeout 4s ros2 topic echo /planning/local_stop_request --once
timeout 4s ros2 topic echo /avoidance/stop_request --once
timeout 4s ros2 topic echo /safety/event --once --full-length
timeout 4s ros2 topic echo /cmd_vel_safe --once
timeout 6s ros2 run tf2_ros tf2_echo map body
```

完成采样后立即禁用路线：

```bash
ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: false}"
```

若车辆并未安全静止，或现场任何人判断存在风险，则跳过采样，先禁用路线并使用遥控器处置。

判读：

- `local_stop_request=true`：局部规划认为当前前方路径不可行，应检查局部 costmap、障碍物和局部重规划参数。
- `local_stop_request=false` 且 `SAFE_HOLD: heading_error_persistent`：问题是控制跟踪/参考曲率或 Safety 阈值，不是障碍物硬停。
- `SAFE_ACTIVE` 但 `/cmd_vel_safe` 近零：检查 MPPI 的目标索引、参考速度和曲率连续性。
- `/cmd_vel_safe` 有非零命令但车辆不动：检查 `/odom`、`/actuator_state` 与 CAN 发送，不修改导航阈值。

当前基线已经是通过整圈验证的 R=1.2 m、0.20 m/s 方案。后续若提高速度，应保留本基线不动，另建 0.30 m/s 候选并先做 30 m 转弯测试；不要直接把已验证基线改到 0.50 m/s。
