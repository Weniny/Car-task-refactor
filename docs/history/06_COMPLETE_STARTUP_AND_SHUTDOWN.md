> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 完整启动与关闭流程（历史已验证基线）

2026-10-27 当前运行入口已更新为 [空场提速完整操作单](10_SPEED_TRIAL_050STRAIGHT_030TURN_2026-10-27.md)，B 导航显示、G 录包、F 发车/停车以该文档及 [显示与发车交接](11_NAV_VIEW_AND_STARTUP_HANDOFF_2026-10-27.md) 为准。下文保留历史 0.20 m/s 操作和当时限制，不能与当前候选入口混用。

本文恢复 2026-09-13 实车跑通时使用的多终端启动流程，并补充统一的关闭步骤。对应配置是 R=1.2 m、最高速度 `0.20 m/s` 的已验证基线。

> **当前禁止直接执行。** 2026-09-18 曾在启动 `ranger_base_node` 后出现底盘异常空转和焦味。只有在硬件检修、纯遥控测试以及底盘节点启动审计全部通过后，才允许由现场负责人解除本限制。详细门槛见 [当前状态与安全交接](01_CURRENT_STATUS_AND_SAFETY.md)。

## 1. 系统边界

运行工作区：

```text
/home/agilex/competition_rebuild_ws
```

每个 ROS 终端都只加载统一环境：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
```

不要在同一终端再次加载 `/home/agilex/competition_ws/install/setup.bash`。禁止绕过 Safety 直接向 `/cmd_vel` 发布速度。

完整控制链为：

```text
连续参考轨迹
-> 局部 Hybrid A* 重规划
-> MPPI 控制器
-> Safety
-> Ranger twist adapter
-> /cmd_vel
-> Ranger 底盘
```

## 2. 终端分工

| 终端 | 任务 | 是否保持运行 |
|---|---|---|
| A | CAN、Livox、FAST-LIO、Ranger 底盘节点 | 是，最后关闭 |
| B | 静态地图和 RViz | 是 |
| C | 三维点云转二维定位激光 | 是 |
| D | AMCL 定位 | 是 |
| E | 局部重规划、MPPI、Safety、底盘适配器 | 是，最先关闭 |
| F | 生命周期、初始位姿、预检、发车与诊断 | 命令执行终端 |
| G（可选） | rosbag 记录 | 测试期间保持运行 |
| H（可选） | 导航调试 RViz | 观察参考、局部规划与执行轨迹 |

不要在 A-E 正在运行 launch 或 node 的终端继续输入其他命令。

## 3. 启动前硬件门槛

以下项目必须由现场人员确认：

- 底盘已检修，不存在焦味、异常温升、异响或轮向异常。
- 物理急停有效，遥控器能够在纯手动模式下正常控制和停车。
- 车辆固定在标记起点，车头方向正确，测试区域清空。
- 有一名人员全程持遥控器并能立即按下物理急停。
- `can2` 接线和 USB-CAN 正常。

任一项不满足时停止，不执行后续 ROS 底盘启动命令。

## 4. 清理旧进程

在 F 执行只读检查：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

pgrep -af \
'rebuild_mapping|rebuild_final_map|nav2_amcl|ranger_base_node|mppi_control_node|safety_node|local_replanner|pointcloud_to_laserscan|proximity_stop_node|fast_lio|livox_ros_driver2' \
|| true
```

如果存在上一轮进程，应回到对应终端使用 `Ctrl+C` 正常结束。不要在无法确认目标时使用宽泛的 `pkill`。

## 5. 配置 CAN

仅在硬件恢复并获准复车后，在 F 执行：

```bash
sudo ip link set can2 down 2>/dev/null || true
sudo ip link set can2 type can bitrate 500000 restart-ms 100
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up

ip -statistics -details link show can2
```

预期状态：

```text
UP, LOWER_UP
can state ERROR-ACTIVE
bitrate 500000
bus-off 0
```

在不启动底盘节点的情况下先确认 RX 计数持续增加：

```bash
ip -statistics link show can2
sleep 2
ip -statistics link show can2
```

无 CAN 回包、错误计数增加或出现 `BUS-OFF` 时立即停止。

## 6. 终端 A：启动传感器、FAST-LIO 和底盘节点

> 此步骤会启动 `ranger_base_node` 并打开底盘 CAN 通信。当前硬件冻结解除前禁止执行。

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup rebuild_mapping.launch.py \
  start_livox:=true \
  force_livox_host_timestamps:=true \
  start_fast_lio:=true \
  fast_lio_config:=fast_lio_mid360_navigation.yaml \
  start_base:=true \
  port_name:=can2 \
  publish_odom_tf:=false \
  start_scan:=false \
  start_slam:=false \
  start_anchor:=false \
  rviz:=false
```

保持 A 运行。FAST-LIO 健康输出应接近：激光 `10 Hz`、IMU `200 Hz`，且没有持续增长的同步丢包。`fast_lio_mid360_navigation.yaml` 与历史 `day1` 配置的定位参数一致，只关闭运行导航时不需要的 PCD 持续保存。

## 7. 终端 B：启动静态地图和 RViz

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:=/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml \
  rviz:=true
```

保持 B 运行。RViz 的 Fixed Frame 使用 `map`。

在 F 确认地图服务器已经激活：

```bash
ros2 lifecycle get /map_server
ros2 topic info /map
```

必须得到 `active [3]`，且 `/map` 有一个发布者。若为 `unconfigured` 或找不到节点，先检查 B 的日志和重复进程，不继续启动导航。

如果不需要 RViz：

```bash
ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:=/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml \
  rviz:=false
```

## 8. 终端 C：启动定位激光转换

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r __node:=rebuild_localization_scan \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/pointcloud_to_laserscan_rebuild_localization.yaml \
  -r cloud_in:=/cloud_registered_body \
  -r scan:=/localization/scan
```

出现 `Got a subscriber to laserscan` 后没有连续控制台输出是正常现象，终端 C 仍需保持运行。

## 9. 终端 D：启动并激活 AMCL

先启动 AMCL：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 run nav2_amcl amcl --ros-args \
  -r __node:=amcl \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/amcl_rebuild_indoor.yaml \
  -r scan:=/localization/scan
```

保持 D 运行。在 F 查询状态：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 lifecycle get /amcl
```

若为 `unconfigured [1]`：

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

若已经是 `active [3]`，不要再次执行 configure 或 activate。

## 10. 终端 F：发布初始位姿

车辆必须位于固定起点且车头方向正确。AMCL 激活后先等待至少 2 秒，再以 `2 Hz` 连续发布 8 次，避免地图或生命周期刚切换时丢失单次初始位姿：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

sleep 2

ros2 topic pub --times 8 -r 2 /initialpose \
geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: -16.083, y: 2.309, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.998045, w: 0.062506}}, covariance: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]}}"
```

该四元数对应名义航向 `3.0165 rad`，约 `172.8 deg`。旧文档中的 `-179.4 deg` 属于更早的起点估计，不用于当前已验证轨迹。

验证定位：

```bash
timeout 5s ros2 topic echo /amcl_pose --once
timeout 8s ros2 run tf2_ros tf2_echo map body
```

初始几秒短暂出现 frame 不存在属于启动竞态；最终必须持续得到稳定的 `map -> body`。同时在 RViz 中确认实时激光与地图墙线贴合。

## 11. 终端 E：启动已验证的 0.20 m/s 导航

### 11.1 30 m 验证轨迹

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_competition_safety_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_020mps_trajectory.yaml \
  start_chassis_adapter:=true
```

### 11.2 已验证整圈轨迹

只有 30 m 复验通过后，才将上面的轨迹替换为：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_competition_safety_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml \
  start_chassis_adapter:=true
```

整圈日志应显示：

```text
continuous_points=3109
MPPI ready
Local Hybrid A* ready
Safety exit ready
Ranger twist adapter ready
```

启动 E 不等于立即发车；`route_enable` 初始应为 `false`。

## 12. 终端 F：发车前完整检查

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 topic info /cmd_vel -v
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 node list | sort | uniq -d
timeout 4s ros2 topic echo /system_state --once
timeout 4s ros2 topic echo /planning/local_replan_status --once --full-length
timeout 4s ros2 topic echo /planning/local_stop_request --once
timeout 4s ros2 topic echo /avoidance/proximity_status --once --full-length
timeout 4s ros2 topic echo /control/status --once --full-length
timeout 4s ros2 topic echo /safety/event --once --full-length
timeout 4s ros2 topic echo /cmd_vel_safe --once
timeout 6s ros2 run tf2_ros tf2_echo map body
```

只有同时满足以下条件才能发车：

- `/cmd_vel` 只有一个发布者 `rebuild_replan_ranger_twist_adapter`。
- `/cmd_vel` 的订阅者为 `ranger_base_node`。
- `/map_server` 和 `/amcl` 均为 `active [3]`；`ros2 node list | sort | uniq -d` 没有输出，表示不存在同名重复节点。
- `vehicle_state: 0`、`control_mode: 1`、`error_code: 0`，电池电压合理。
- `map -> body` 连续稳定，RViz 激光贴图正确。
- 局部规划状态为 `REFERENCE_CLEAR` 或安全的 `REPLANNED`。
- `/planning/local_stop_request` 为 `false`。
- `/control/status` 为 `ROUTE_DISABLED`，`route_enabled: false`。
- Safety 没有故障原因，`/cmd_vel_safe` 为全零。
- 遥控器、物理急停和现场监护人员均已就位。

任一条件不满足时不发车。

## 13. 可选终端 G：记录 rosbag

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 bag record \
  -o "/home/agilex/competition_rebuild_ws/log/full_loop_$(date +%Y%m%d_%H%M%S)" \
  /control/status \
  /control/tracking_error \
  /control/body_cmd \
  /planning/local_replan_status \
  /planning/local_stop_request \
  /avoidance/stop_request \
  /avoidance/proximity_status \
  /avoidance/speed_limit \
  /safety/event \
  /cmd_vel_safe \
  /cmd_vel \
  /odom \
  /motion_state
```

## 14. 发车

确认测试区域安全后，在 F 执行：

```bash
ros2 topic pub --once /mission/route_enable \
std_msgs/msg/Bool "{data: true}"
```

正常状态应为：

```text
control/status: TRACKING
safety/event: SAFE_ACTIVE
planning/local_stop_request: false
```

不得使用固定 `sleep` 自动发车或自动结束。测试过程中现场人员应持续观察车辆和 RViz。

## 15. 正常停止和紧急停止

### 软件停止

在 F 执行：

```bash
ros2 topic pub --once /mission/route_enable \
std_msgs/msg/Bool "{data: false}"

timeout 4s ros2 topic echo /control/status --once --full-length
timeout 4s ros2 topic echo /cmd_vel_safe --once
timeout 4s ros2 topic echo /odom --once --field twist.twist
```

必须确认：

- `route_enabled: false`；
- `/cmd_vel_safe` 线速度和角速度均为零；
- `/odom` 速度归零，车辆物理静止。

### 紧急情况

车辆异常加速、轮向异常、异响、焦味、冒烟、失控或人员进入危险区时：

1. 现场人员立即按物理急停。
2. 在安全条件允许时切断底盘电源。
3. 不等待 SSH 命令，不依赖软件停车。
4. 不再次上电试车，先完成硬件检查。

## 16. 完整关闭顺序

确认车辆静止后按以下顺序关闭：

1. 在 F 发布 `route_enable=false` 并确认最终速度为零。
2. 在 E 按 `Ctrl+C`，先停止 MPPI、局部重规划、Safety 和底盘适配器。
3. 在 G 按 `Ctrl+C`，等待 rosbag 正常写盘完成。
4. 在 D 按 `Ctrl+C`，停止 AMCL。
5. 在 C 按 `Ctrl+C`，停止定位激光转换。
6. 在 B 按 `Ctrl+C`，停止地图和 RViz。
7. 最后在 A 按 `Ctrl+C`，停止 FAST-LIO、Livox 和 `ranger_base_node`。

不要先关闭 A；底盘节点和状态输入应在导航输出断开后再退出。

关闭后在 F 检查残留：

```bash
pgrep -af \
'rebuild_mapping|rebuild_final_map|nav2_amcl|ranger_base_node|mppi_control_node|safety_node|local_replanner|pointcloud_to_laserscan|proximity_stop_node|fast_lio|livox_ros_driver2' \
|| true

ros2 topic info /cmd_vel -v || true
```

确认没有相关进程后，可选关闭 CAN：

```bash
sudo ip link set can2 down
ip -details link show can2
```

## 17. 一键启动脚本说明

统一编排脚本为：

```text
scripts/run_navigation.sh
```

它负责预检、启动整合 launch、等待定位、记录 rosbag、固定起点确认和路线使能。其基本调用形式为：

```bash
bash /home/agilex/competition_rebuild_ws/scripts/run_navigation.sh --mission 30m
bash /home/agilex/competition_rebuild_ws/scripts/run_navigation.sh --mission full_loop
```

脚本有两次不同的人工确认：第一次必须输入 `AT_START` 才会发布固定初始位姿，全部预检通过后第二次输入 `START` 才会使能路线。输入错误或直接关闭 SSH 都会触发清理流程，不能把两次确认混用。

该脚本会间接启动底盘相关节点，不是纯仿真工具。在 2026-09-18 硬件事件关闭前同样禁止运行。恢复测试后，也应先按照本文的多终端流程完成一次受控复验，再考虑重新启用一键入口。

脚本不会替操作员配置或拉起 `can2`；运行前仍须完成第 3 至 5 节的硬件门槛、旧进程清理和 CAN 检查。脚本会拒绝 `can2` 未处于 `UP` 状态、存在旧导航进程或固定起点确认不正确的情况。

## 18. 后续验证 0.50 m/s 候选

0.50 m/s 验证沿用本文的硬件检查、终端 A-D、初始位姿、发车预检、rosbag 和关闭流程，但终端 E 使用独立的 0.50 m/s 配置与轨迹。不能只把 0.20 m/s 启动命令中的轨迹文件名替换后直接发车。

### 18.1 状态定义

- `0.20 m/s` 是历史已验证基线，文件保持不动。
- `0.50 m/s` 当前是未验证候选，不得在报告中称为基线。
- 只有短距离、30 m 转弯、单障碍、复杂障碍和整圈全部通过后，才能将其升级为新基线。

0.50 m/s 候选文件：

```text
控制参数：config/control/control_params_rebuild_050mps.yaml
Safety 参数：config/safety/safety_params_rebuild_050mps_inflation035.yaml
速度参数：config/planning/continuous_trajectory_radius090_turnrate250_050mps.yaml
局部限速参数：config/planning/continuous_trajectory_radius090_turnrate250_050mps_localcap.yaml
30 m 轨迹：artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_050mps_trajectory.yaml
30 m 局部限速轨迹：artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_050mps_localcap_trajectory.yaml
整圈轨迹：artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_050mps_trajectory.yaml
整圈局部限速轨迹：artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_050mps_localcap_trajectory.yaml
多终端导航入口：rebuild_navigation_replan_radius120_050mps_guarded.launch.py
整合试验入口：rebuild_full_navigation_050_trial.launch.py
比赛式停车候选：rebuild_full_navigation_050_competition_stop_candidate.launch.py
```

### 18.2 第一阶段：先回归 0.20 m/s

硬件恢复后，先严格按照本文第 6 至 16 节完成一次 0.20 m/s、30 m 无障碍回归。若 0.20 m/s 无法重新通过，则停止，不进入 0.50 m/s 调参。

### 18.3 第二阶段：0.50 m/s 禁用速度适配器检查

保持终端 A-D 和定位环境正常，在 E 启动 0.50 m/s 导航，但不启动底盘适配器：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_050mps_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_050mps_localcap_trajectory.yaml \
  optimizer_params_file:=/home/agilex/competition_rebuild_ws/config/planning/continuous_trajectory_radius090_turnrate250_050mps_localcap.yaml \
  start_chassis_adapter:=false
```

这里仅断开 `/cmd_vel_safe -> /cmd_vel` 的 Ranger twist adapter，**并不等于底盘完全断开**：终端 A 中的 `ranger_base_node` 仍会打开 CAN、发布 `/odom` 和 `/system_state`。因此本阶段也只能在硬件检修和底盘节点审计通过后执行。若需要真正与底盘物理隔离的纯软件试验，应使用仿真或回放数据，不能把本命令当成物理隔离措施。

`*_050mps_localcap_trajectory.yaml` 与 `continuous_trajectory_radius090_turnrate250_050mps_localcap.yaml` 是经过来源哈希绑定的一对文件，必须同时传入。若轨迹使用 `localcap` 而优化器仍为默认非 `localcap` 配置，MPPI 会以 `trajectory source mismatch: optimizer_params:sha256_mismatch` 退出。

此阶段不得存在真实 `/cmd_vel` 发布者：

```bash
ros2 topic info /cmd_vel -v
```

确认发布者数量为零后，可以临时使能路线，仅运行控制计算链：

```bash
ros2 topic pub --once /mission/route_enable \
std_msgs/msg/Bool "{data: true}"

timeout 4s ros2 topic echo /control/status --once --full-length
timeout 4s ros2 topic echo /control/body_cmd --once
timeout 4s ros2 topic echo /cmd_vel_safe --once
```

静态检查结束后立即撤销路线使能：

```bash
ros2 topic pub --once /mission/route_enable \
std_msgs/msg/Bool "{data: false}"
```

检查项目：

- MPPI、局部规划器和 Safety 节点均存在且没有退出。
- `/planning/local_replan_status` 能持续更新。
- `/planning/local_stop_request` 在无障碍时为 `false`。
- RViz 中蓝色参考线和绿色局部路线正确。
- 路线临时使能时，`/control/body_cmd` 和 `/cmd_vel_safe` 的计算结果合理，但真实 `/cmd_vel` 发布者数量为零。
- 移入单个静态障碍后能够观察到 `REPLANNED` 或安全停车状态，移开后状态能够恢复。

完成后在 E 按 `Ctrl+C`，不得在同一进程上动态接通底盘。

如需观察局部重规划，在图形桌面的 H 打开导航调试配置；普通 `final_map_view.rviz` 不包含全部导航路径显示：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

rviz2 -d \
/home/agilex/competition_rebuild_ws/install/rebuild_bringup/share/rebuild_bringup/rviz/navigation_debug_view.rviz
```

调试视图中蓝线为 `/planning/replan_reference_path`，绿线为 `/planning/local_trajectory`，黄线为 `/control/replan_executed_path`。断底盘静态检查时黄线不增长是正常现象。

### 18.4 第三阶段：专用短距离轨迹

在接通 0.50 m/s 底盘前，应先生成并校验独立的 3 至 5 m 轨迹。当前仓库中的 30 m 文件不能代替短距离安全轨迹，也不使用固定 `sleep` 自动中止车辆。

短距离验证至少检查：

- 起步和停车没有冲击或车轮异常摆动。
- `/cmd_vel_safe`、`/cmd_vel` 与 `/odom` 响应方向一致。
- 制动距离满足现场安全空间。
- 遥控器接管和物理急停有效。

没有专用短距离轨迹或制动距离数据时，不进入 30 m 实车测试。

### 18.5 第四阶段：30 m 转弯测试

短距离通过后，在 E 启动 30 m 局部限速候选：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_050mps_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_050mps_localcap_trajectory.yaml \
  optimizer_params_file:=/home/agilex/competition_rebuild_ws/config/planning/continuous_trajectory_radius090_turnrate250_050mps_localcap.yaml \
  start_chassis_adapter:=true
```

必须重新执行本文第 12 节的全部发车前检查，然后才允许发布 `route_enable=true`。重点观察：

- 第一处转弯是否连续通过；
- 前轮是否反复左右摆动或突然回正；
- 局部规划更新是否造成频繁路线替换；
- Safety 是否在正常转弯中反复进入恢复或停车；
- 实际速度、曲率和制动距离是否符合预期。

出现轮向异常、失去前进能力、异响、异味或状态不一致时立即物理急停，并停止本轮验证。

### 18.6 第五阶段：避障验证

30 m 无障碍通过后，按以下顺序增加难度：

1. 单个静态障碍，保持充足绕行空间。
2. 同一位置重复测试，验证结果是否稳定。
3. 多个静态障碍，但不得封死目标回接点。
4. 障碍移入和移出，观察重新规划与恢复速度。
5. 最后才讨论动态障碍物测试。

每一级都要记录 `/planning/local_replan_status`、`/avoidance/proximity_status`、`/safety/event`、`/cmd_vel_safe`、`/cmd_vel` 和 `/odom`。规划出绿色路线不等于允许前进；最终仍由 Safety 和底盘状态共同决定输出。

### 18.7 第六阶段：0.50 m/s 整圈

只有前五阶段全部通过，车辆重新回到固定起点并重新发布初始位姿后，才允许使用整圈局部限速轨迹：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_050mps_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_050mps_localcap_trajectory.yaml \
  optimizer_params_file:=/home/agilex/competition_rebuild_ws/config/planning/continuous_trajectory_radius090_turnrate250_050mps_localcap.yaml \
  start_chassis_adapter:=true
```

整圈仍执行第 12 至 16 节的预检、记录、停止和关闭流程。整圈无故障通过并完成日志复盘后，才讨论将 0.50 m/s 候选固化为基线。

### 18.8 整合入口不能与多终端入口并用

`rebuild_full_navigation_050_trial.launch.py` 和 `rebuild_full_navigation_050_competition_stop_candidate.launch.py` 会整合启动传感器、定位或底盘相关节点。使用整合入口时不得同时保留本文 A-E 的同类节点，否则可能出现重复的 `/map_server`、`/amcl`、定位扫描或 `ranger_base_node`。

比赛式停车候选仍处于未验证状态，并且曾在其整合启动期间发生底盘异常事件。完成底盘节点审计和上述分阶段验证之前，不运行该整合入口。
