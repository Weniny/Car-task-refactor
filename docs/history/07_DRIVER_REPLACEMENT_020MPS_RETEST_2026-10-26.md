> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-10-26 新驱动器：0.20 m/s 整圈基线复验操作单

2026-10-27 通用操作更新：后续 B 导航显示、G 录包、F 发车/停车采用 [显示与发车交接](11_NAV_VIEW_AND_STARTUP_HANDOFF_2026-10-27.md) 的新入口。本页保留 0.20 基线复验记录；下文旧 topic pub 命令不再作为当前推荐发车方式，提速候选完整步骤见 [当前操作单](10_SPEED_TRIAL_050STRAIGHT_030TURN_2026-10-27.md)。

本轮结果已记录：现场反馈整圈完成，全局进度达到 3108；终点收敛表现与四段跟踪恢复待复盘，人工结束前路线保持使能符合现有流程。详见 [复验结果与后续安排](08_020MPS_FULL_LOOP_RESULT_2026-10-26.md)。下文保留测试时使用的操作步骤。

今晚目标：确认更换驱动器后，原 R=1.2 m、最高 0.20 m/s 基线仍能完整运行并正常停车。按本次安排直接运行整圈，顺序为：硬件与纯遥控 → CAN 被动检查 → 最小底盘节点静止检查 → 定位 → 整圈预检与录包 → 整圈 → 关机与日志复盘。

已知新信息是“驱动器已更换”；纯遥控、底盘节点初始化和实车回归结果尚待现场记录。2026-09-18 的异常空转与焦味事件记录保留，见 [安全交接](01_CURRENT_STATUS_AND_SAFETY.md)。更换硬件后的每一步按下述实测结果推进，不把更换本身写成验收通过。

本文只准备指令，没有连接小车或执行运动命令。以下命令全部在**小车 Ubuntu 的 Bash 终端**执行，不在 Windows PowerShell 执行。

## 1. 今晚固定使用的基线

| 项目 | 文件或数值 |
|---|---|
| 工作区 | `/home/agilex/competition_rebuild_ws` |
| SDK/底盘工作区 | `/home/agilex/agilex_ws` |
| ROS | Humble；每个终端只 source 统一环境脚本 |
| CAN | 历史为 `can2`、500000 bit/s，现场先确认新驱动器配置一致 |
| 底盘协议 | 仍须兼容 Ranger Mini V3 与原 twist adapter；若厂商改了协议或转向语义，先核对适配 |
| 导航入口 | `rebuild_navigation_replan_radius120_competition_safety_guarded.launch.py` |
| 控制参数 | `config/control/control_params_rebuild_020mps.yaml` |
| Safety 参数 | `config/safety/safety_params_rebuild_020mps_turn_recovery.yaml` |
| 整圈轨迹 | `artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml` |
| 整圈规模 | 155.046 m，3109 点，最后一个全局点索引 3108 |
| 参考耗时 | 整圈轨迹 781.717 s，约 13 分钟；实际可能更长 |
| 名义起点 | x=-16.083，y=2.309，yaw=3.0165 rad |
| 整圈终点 | x=-13.906，y=2.066，yaw=3.0349 rad；**与起点不是同一个坐标** |

路线、地图、optimizer、控制和 Safety 参数均沿用历史基线。文件名中的 `radius090` 是沿用名称：本入口实际绑定 radius120 语义地图，不能凭文件名自行替换配置。

本地已核对整圈轨迹的 6 项来源 SHA-256 均匹配，点数为 3109，轨迹速度上限为 0.20 m/s；车端仍须重做第 3 节的校验。

## 2. 终端和现场分工

| 终端 | 用途 |
|---|---|
| A0 | 临时最小底盘节点检查；检查后关闭，再开 A |
| A | Livox、FAST-LIO、Ranger 底盘节点；最后关闭 |
| B | 静态地图与地图 RViz |
| C | 定位激光转换 |
| D | AMCL |
| E | 重规划、MPPI、Safety、底盘适配器 |
| F | 初始化、预检、发车、停车、参数快照 |
| G | rosbag；每次试跑单独录制 |
| H | 导航调试 RViz，可选 |
| I | 运行状态监视，保持 F 可以随时停车 |

A–E、G、H、I 的前台命令运行时，不在同一终端继续输入其他命令。每个新终端先执行：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
cd /home/agilex/competition_rebuild_ws
```

现场一人操作终端，另一人持遥控器并能立即物理急停。不要把 SSH 断线、Ctrl+C 或软件停车当成物理急停。

## 3. F：开测前准备与车端文件校验

### 3.1 先确认硬件

先在不运行 ROS 底盘节点时完成：

1. 记录新驱动器型号、固件/参数版本、更换时间，以及电机方向、转向标定和 CAN 波特率；未改变的项目也记录。
2. 确认线束、电源、保险、驱动器和轮组正常，没有焦味、异常温升或卡滞。
3. 按厂商流程完成纯遥控低速前进、转向、停车、接管及急停验证。
4. 确认 Ranger SDK 的启动控制模式、运动模式和默认发送行为已复核，适用于新驱动器。SDK 源码在小车另一工作区，本次本地准备未审计其实际版本。

以上未完成时，先完成硬件/SDK 核对，不进入 A0。异常再次出现时按第 12 节处理。

### 3.2 检查旧进程、环境与空间

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
cd /home/agilex/competition_rebuild_ws

pgrep -af \
'rebuild_full_navigation|rebuild_navigation_replan|rebuild_mapping|rebuild_final_map|nav2_amcl|ranger_base_node|mppi_control_node|safety_node|local_replanner|ranger_twist_adapter|pointcloud_to_laserscan|proximity_stop_node|fastlio_mapping|livox_ros_driver2' \
|| true

ros2 pkg prefix rebuild_bringup
ros2 pkg prefix competition_control
ros2 pkg prefix competition_safety
ros2 pkg prefix ranger_bringup
df -h /home/agilex/competition_rebuild_ws
mkdir -p /home/agilex/competition_rebuild_ws/log
```

有上一轮进程时回对应终端 Ctrl+C 正常退出。重构包应解析到 `competition_rebuild_ws`，底盘包应来自预期的 SDK 工作区；若指向旧 `competition_ws`，先修正环境。留足整圈 rosbag 的空间。

若车端源码刚同步且尚未构建，只在没有运行节点时构建：

```bash
cd /home/agilex/competition_rebuild_ws
colcon build --symlink-install --packages-up-to rebuild_bringup \
  competition_control competition_planning competition_safety competition_localization
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
```

构建失败先处理，不继续。没有代码变化且安装已确认一致时，无需重复构建。本文没有修改运行代码或基线参数。

### 3.3 校验整圈轨迹及来源，不生成新轨迹

在 F 粘贴完整块：

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
from competition_planning.artifact_provenance import (
    resolve_trajectory_source_paths,
    validate_source_manifest,
)

ws = Path('/home/agilex/competition_rebuild_ws')
sources = resolve_trajectory_source_paths(
    route_file=ws / 'routes/indoor_manual_six_anchor_v3_radius090_loop.yaml',
    semantic_map_file=ws / 'maps/indoor_manual_final/semantic_map_six_anchor_v3_radius120.yaml',
    planning_params_file=ws / 'config/planning/semantic_lidar_guarded_six_anchor_radius120.yaml',
    optimizer_params_file=ws / 'config/planning/continuous_trajectory_radius090_turnrate250_020mps.yaml',
)
name = 'indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml'
data = yaml.safe_load((ws / 'artifacts' / name).read_text(encoding='utf-8'))
validate_source_manifest(data, sources)
assert data['ok'] is True
assert data['point_count'] == len(data['points']) == 3109
assert max(abs(float(p['v'])) for p in data['points']) <= 0.200001
print('PASS', name, 'points=', 3109, 'last_index=', 3108,
      'length_m=', data['path_length_m'], 'finish=', data['points'][-1])
PY
```

必须输出 PASS。若缺包、文件缺失或 `sha256_mismatch`，检查同步和安装；不删除来源校验，不临时重生成轨迹绕过错误。

## 4. F → A0：CAN 与最小底盘节点检查

### 4.1 F 配置并被动检查 CAN

确认实际接口仍为 can2、厂商要求仍为 500 kbit/s 后执行：

```bash
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000 restart-ms 100
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up
ip -statistics -details link show can2
```

应为 UP、ERROR-ACTIVE、bitrate 500000。首次设置前接口不存在或任何命令报错时，先检查接口与接线，不跳过。

不启动底盘节点，比较两次 RX 和错误计数：

```bash
ip -statistics link show can2
sleep 2
ip -statistics link show can2
```

RX 应增长，错误计数不应持续增长；BUS-OFF、无回包或驱动器报警时停止。此处的 sleep 仅用于采样，不用于发车。

### 4.2 A0 只启动底盘，验证初始化不引起运动

硬件、纯遥控和启动行为复核完成后，由现场人员按厂商允许的安全支撑或隔离方式实施。先在 F 检查 `/cmd_vel`：

```bash
ros2 topic info /cmd_vel -v
```

话题尚不存在是正常的；若存在，发布者必须为 0。任何 teleop 或旧适配器发布者都要先退出。

在 A0 执行：**这一步会打开 CAN 并启动 ranger_base_node，即使没有速度发布者也须现场观察。**

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_mapping.launch.py \
  start_livox:=false \
  start_fast_lio:=false \
  start_base:=true \
  port_name:=can2 \
  publish_odom_tf:=false \
  start_scan:=false \
  start_slam:=false \
  start_anchor:=false \
  rviz:=false
```

F 检查：

```bash
ros2 topic info /cmd_vel -v
timeout 5s ros2 topic echo /system_state --once
timeout 5s ros2 topic echo /motion_state --once
timeout 5s ros2 topic echo /odom --once --field twist.twist
ip -statistics -details link show can2
```

本轮建议连续观察至少 60 秒：车辆没有未经请求的运动、轮组没有异常动作，状态持续更新，无报警或异味。CAN 控制模式的切换仅按厂商流程执行，不发送原始模式帧或强制清错。

A0 通过后 Ctrl+C 关闭，并确认 `ranger_base_node` 已退出，再继续 A；不能同时启动两个底盘节点。此步通过只证明受控静止启动正常，运动响应须在整圈起步时现场观察。

## 5. A–D：建立传感器与定位

### 5.1 A：传感器、FAST-LIO、底盘

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

A 保持运行。若无速度使能却出现异常动作，立即物理急停，不继续定位。

### 5.2 B：地图和 RViz

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:=/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml \
  rviz:=true
```

若 B 为无图形环境的 SSH 终端，使用 `rviz:=false`，在小车图形桌面另开 RViz。RViz Fixed Frame 使用 map。

### 5.3 C：定位用二维激光

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r __node:=rebuild_localization_scan \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/pointcloud_to_laserscan_rebuild_localization.yaml \
  -r cloud_in:=/cloud_registered_body \
  -r scan:=/localization/scan
```

### 5.4 D：AMCL

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 run nav2_amcl amcl --ros-args \
  -r __node:=amcl \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/amcl_rebuild_indoor.yaml \
  -r scan:=/localization/scan
```

在 F 查询：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic info /map
```

`/map_server` 应由 B 自动激活。AMCL 若为 unconfigured，依次执行；若为 inactive，只执行 activate；若已 active，不重复切换：

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

两者都必须为 `active [3]`，地图有且仅有一个发布者。

### 5.5 F：固定起点初始化

仅在车辆**确实摆回标记起点、方向正确**时执行。停在别处时不能通过发布这个坐标伪装回到起点。

```bash
sleep 2
ros2 topic pub --times 8 -r 2 /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -16.083, y: 2.309, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.998045, w: 0.062506}}, covariance: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]}}"

timeout 5s ros2 topic echo /amcl_pose --once
timeout 8s ros2 run tf2_ros tf2_echo map body
timeout 8s ros2 topic hz /livox/lidar
timeout 8s ros2 topic hz /livox/imu
timeout 8s ros2 topic hz /cloud_registered_body
```

雷达约 10 Hz、IMU 约 200 Hz，注册点云持续更新；`map → body` 连续稳定，实时激光贴合地图墙线。hz/tf_echo 因 timeout 结束的 124 退出码本身不是故障；完全无输出或连续报 TF/时间同步错误则不能继续。

## 6. E：直接加载整圈轨迹

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_competition_safety_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml \
  start_chassis_adapter:=true
```

E 保持运行，确认加载 3109 点，MPPI、局部规划、Safety、适配器均正常。**必须显式传 trajectory_file**：这个入口默认指向另一条旧 0.8 m 轨迹，其来源与本入口的 R=1.2 m 配置不配套，不能省略。启动适配器不等于发车，控制状态仍应为 ROUTE_DISABLED。

可选 H，在图形桌面打开导航视图：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
rviz2 -d /home/agilex/competition_rebuild_ws/install/rebuild_bringup/share/rebuild_bringup/rviz/navigation_debug_view.rviz
```

蓝线为全局参考 `/planning/replan_reference_path`，绿线为局部轨迹 `/planning/local_trajectory`，黄线为执行轨迹 `/control/replan_executed_path`。

## 7. F：每次发车前完整预检

本节所有检查通过，完成第 8 节录包准备后，才执行第 9 节的整圈使能命令。重新试跑时须重做预检。

### 7.1 核对实际生效参数

```bash
ros2 param get /rebuild_replan_mppi trajectory_file
ros2 param get /rebuild_replan_mppi max_speed_mps
ros2 param get /rebuild_replan_mppi max_curvature_rate_1pmps
ros2 param get /rebuild_replan_mppi require_route_enable
ros2 param get /rebuild_replan_mppi replanning_enabled
ros2 param get /rebuild_replan_safety max_speed_mps
ros2 param get /rebuild_replan_safety recovery_speed_mps
ros2 param get /rebuild_replan_safety max_lateral_error_m
ros2 param get /rebuild_replan_safety max_heading_error_deg
ros2 param get /rebuild_replan_proximity_stop stop_distance_m
ros2 param get /rebuild_replan_proximity_stop grid_inflation_radius_m
ros2 param get /rebuild_local_replanner inflation_radius_m
```

依次应为：本轮轨迹路径、0.20、2.50、true、true、0.20、0.15、0.40、45.0、0.40、0.44、0.04。

注意：控制 YAML 中曲率变化率是 0.9、Safety YAML 中 stop_distance 是 0.0；本 launch 分别覆盖为 **2.50 和 0.40**。以运行时查询为准，禁止临场靠放宽 Safety 让车前进。

### 7.2 节点、状态、停车与定位

```bash
ros2 node list | sort | uniq -d
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_safe -v
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
timeout 5s ros2 topic echo /system_state --once
timeout 5s ros2 topic echo /motion_state --once
timeout 5s ros2 topic echo /control/status --once --full-length
timeout 5s ros2 topic echo /planning/local_replan_status --once --full-length
timeout 5s ros2 topic echo /planning/local_stop_request --once
timeout 5s ros2 topic echo /avoidance/stop_request --once
timeout 5s ros2 topic echo /avoidance/proximity_status --once --full-length
timeout 5s ros2 topic echo /safety/event --once --full-length
timeout 5s ros2 topic echo /cmd_vel_safe --once
timeout 5s ros2 topic echo /cmd_vel --once
timeout 5s ros2 topic echo /odom --once --field twist.twist
timeout 6s ros2 run tf2_ros tf2_echo map body
timeout 35s python3 /home/agilex/competition_rebuild_ws/scripts/navigation_preflight.py --timeout 20
```

人工检查与脚本检查同时通过：

- 无同名重复节点；没有另一套统一入口/teleop 同时运行。
- `/cmd_vel` 唯一发布者为 `/rebuild_replan_ranger_twist_adapter`，订阅者包含 `/ranger_base_node`；`/cmd_vel_safe` 唯一发布者为 `/rebuild_replan_safety`。录包进程作为额外订阅者是正常的。
- `vehicle_state: 0`、`control_mode: 1`、`error_code: 0`；`/motion_state` 为 `motion_mode: 0`（dual Ackermann），电压符合厂商要求。
- map_server、AMCL 均 active；TF 稳定且 RViz 激光贴图正确。
- 无障碍基线预期 `REFERENCE_CLEAR`；若出现 REPLANNED，先解释现场障碍/地图偏差，不能无条件视为无障碍基线正常。
- 局部和近障 stop_request 均 false，连续多次采样稳定；Safety 无故障原因。
- `/control/status` 为 `ROUTE_DISABLED`，`route_enabled: false`；`/cmd_vel_safe`、最终 `/cmd_vel` 全零，里程计与实际车辆静止。
- 预检脚本输出 `PRECHECK PASSED`。该脚本检查固定起点误差 ≤0.30 m、航向误差 ≤12°等软件条件，不能替代上述硬件、运动模式、生命周期和贴图检查。

任何命令超时、没有数据、状态不符或脚本非零退出都视为未通过，不执行发车命令。

## 8. F/G：每轮建立记录目录、参数快照和 rosbag

### 8.1 F 选本轮标签

本轮直接使用整圈标签：

```bash
TEST_TAG=full_loop_020
RUN_DIR="/home/agilex/competition_rebuild_ws/log/retest_${TEST_TAG}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" > /home/agilex/competition_rebuild_ws/log/retest_current_run.txt
date -Is > "$RUN_DIR/start_time.txt"
ros2 param dump /rebuild_replan_mppi > "$RUN_DIR/mppi.yaml"
ros2 param dump /rebuild_replan_safety > "$RUN_DIR/safety.yaml"
ros2 param dump /rebuild_local_replanner > "$RUN_DIR/replanner.yaml"
ros2 param dump /rebuild_replan_proximity_stop > "$RUN_DIR/proximity.yaml"
ip -statistics -details link show can2 > "$RUN_DIR/can_before.txt"
printf '本轮目录：%s\n' "$RUN_DIR"
```

### 8.2 G 录包

确认上一轮 G 已经 Ctrl+C 并完成写盘。新一轮再执行：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
RUN_DIR=$(cat /home/agilex/competition_rebuild_ws/log/retest_current_run.txt)
ros2 bag record -s sqlite3 -o "$RUN_DIR/rosbag" \
  /mission/route_enable \
  /control/status /control/tracking_error /control/body_cmd /control/state_valid \
  /planning/local_replan_status /planning/local_stop_request \
  /planning/replan_reference_path /planning/local_trajectory \
  /control/replan_executed_path \
  /avoidance/stop_request /avoidance/proximity_status /avoidance/speed_limit \
  /safety/event /cmd_vel_safe /cmd_vel \
  /odom /system_state /motion_state \
  /amcl_pose /initialpose /tf /tf_static /rosout
```

确认 G 开始录制、核心话题已订阅，没有存储错误再发车。基线未发布 `/avoidance/speed_limit` 时，单独该话题等待发布者是允许的。本轮不录原始点云以控制磁盘占用；有定位异常时另安排传感器专项诊断。

### 8.3 F 保存发车前快照

```bash
timeout 5s ros2 topic echo /control/status --once --full-length > "$RUN_DIR/control_before.txt"
timeout 5s ros2 topic echo /system_state --once > "$RUN_DIR/system_before.txt"
timeout 5s ros2 topic echo /motion_state --once > "$RUN_DIR/motion_before.txt"
timeout 5s ros2 topic echo /safety/event --once --full-length > "$RUN_DIR/safety_before.txt"
```

## 9. F/I：整圈发车、观察与停车

持遥控器人员就位，路线清空。F 手动执行：

```bash
ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: true}"
```

I 开启状态监视，F 留空备用：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 topic echo /control/status --field data --full-length
```

F 可按需分别采样，禁止整段高频诊断挤占停车操作：

```bash
timeout 4s ros2 topic echo /safety/event --once --full-length
timeout 4s ros2 topic echo /control/tracking_error --once
timeout 4s ros2 topic echo /cmd_vel_safe --once
timeout 4s ros2 topic echo /odom --once --field twist.twist
```

整圈从固定起点一次使能后持续运行，起步前几米重点观察实际运动方向、速度与轮向。新驱动器响应明显与指令不符时立即终止。若中途停止、接管或重新定位，记录本次为中止；重新试跑应回到固定起点、重启 E、重新初始化并预检，另建录包目录。

正常运行预期 TRACKING、SAFE_ACTIVE，进度持续前进；第一处转弯应连续通过，没有轮向反复摆动、异常回正或长期原地等待。`tracking_error.vector.x` 单位为 m，`vector.y` 单位为 rad，比较角度时须转换。

**不要仅凭 `target_index` 或一次 `GOAL_REACHED` 判定完成**：MPPI 当前跟踪的是不断更新的局部轨迹，这两项可能对应局部末端。结合全局 `route_progress_index`（整圈末点 3108）、实际到达整圈终点（x=-13.906、y=2.066）、RViz 路线和停车状态判断。

正常到站且车辆已经静止时，可以先保存终点状态，再撤销使能：

```bash
timeout 5s ros2 topic echo /control/status --once --full-length > "$RUN_DIR/control_at_stop.txt"
```

需要停车或中止时，F 直接执行下面的停车块，不等待任何快照：

```bash
ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: false}"
timeout 5s ros2 topic echo /control/status --once --full-length
timeout 5s ros2 topic echo /cmd_vel_safe --once
timeout 5s ros2 topic echo /cmd_vel --once
timeout 5s ros2 topic echo /odom --once --field twist.twist
ip -statistics -details link show can2 > "$RUN_DIR/can_after.txt"
```

**危险情况直接物理急停。** 确认 route_enabled=false、两级速度命令归零、里程计接近零且实车完全静止。

随后按第 12 节完整关闭，并按第 13 节检查日志。

## 10. 整圈耗时与完成判定

整圈参考耗时约 13 分钟，现场预留至少 20 分钟观察；耗时仅供排程，不用固定 sleep 自动发车或判定结束。本轮先跑清空路线，不主动放置障碍，把驱动器变化与额外避障变量分开验证。

整圈末点全局索引为 3108。进度应进入末端，并且实车沿完整路线到达本文件 finish（x=-13.906、y=2.066），正常减速停车；保存到站状态后立即撤销路线使能。若全局进度未到末端、卡在局部终点或反复出现 GOAL_REACHED，停止并保留日志诊断，不把“车停了”当成整圈完成。

## 11. 今晚建议验收口径

| 项目 | 通过条件 |
|---|---|
| 底盘恢复 | 纯遥控、急停/接管、最小底盘节点静止检查均通过，无非预期动作、故障或异味 |
| 整圈完成 | 从固定起点沿完整约 155 m 路线到 finish，无推车、重置定位、重启导航或人工接管完成剩余路程 |
| 速度 | 运行上限为 0.20 m/s；实际速度与指令方向、量级一致，没有持续超速或异常冲击 |
| 定位与跟踪 | 全程贴图可信，无持续 TF/状态失效；无反复摆轮、长时间卡住或不可解释的进度跳变 |
| Safety | 保持原保护；若触发恢复/停车，记录位置、原因、持续时间；反复触发或需放宽阈值才能继续则不通过 |
| 结束 | route_enable=false，两级命令归零，车辆静止；完整关闭无残留运动发布者 |
| 证据 | 本次整圈 rosbag、运行参数、CAN 前后状态、现场记录齐全且可读 |

Safety 恢复带进入为 0.20 m/20°，退出为 0.10 m/8°，恢复速度 0.15 m/s，硬限制为 0.40 m/45°。硬限制是保护边界，不能作为“误差在此以内就算优秀”的验收口径。无障碍基线若需频繁恢复，先复盘。

建议：无障碍时若约 10 秒无明显进度，先正常停止诊断；这是本轮操作建议，不是自动计时控制。任何危险迹象不等待计时。

## 12. 异常处理与完整关闭

### 正常停止

F 随时可用：

```bash
ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: false}"
```

确认实际停车后，关闭顺序：**E → G → D → C → H/B → A**，均 Ctrl+C，G 等待正常写盘。I 的 echo 也 Ctrl+C。不要先关闭 A 的底盘与状态输入。

### 危险情况

异常空转、轮向异常、非预期加速、异响、焦味/冒烟、失控或人员进入危险区：现场立即物理急停，安全条件允许时断底盘电源。不等 SSH，不通过重新使能路线/反复上电试错，不按 KEY1 强制清错。保留时间、日志和硬件状态，先检查新驱动器及相关线束/电机。

### 关闭后 F 检查

```bash
pgrep -af \
'rebuild_full_navigation|rebuild_navigation_replan|rebuild_mapping|rebuild_final_map|nav2_amcl|ranger_base_node|mppi_control_node|safety_node|local_replanner|ranger_twist_adapter|pointcloud_to_laserscan|proximity_stop_node|fastlio_mapping|livox_ros_driver2' \
|| true
ros2 topic info /cmd_vel -v
```

话题不存在或发布者为 0；确认无相关残留进程后，按现场需要关闭 CAN：

```bash
sudo ip link set can2 down
ip -details link show can2
```

## 13. F：检查记录并留测试结论

每轮 G 正常退出后可执行；若已关 A–D，也可只加载环境后离线读取：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
RUN_DIR=$(cat /home/agilex/competition_rebuild_ws/log/retest_current_run.txt)
ros2 bag info "$RUN_DIR/rosbag"
python3 /home/agilex/competition_rebuild_ws/scripts/diagnose_navigation_bag.py \
  "$RUN_DIR/rosbag" | tee "$RUN_DIR/diagnosis.txt"
```

`retest_current_run.txt` 始终只指最近一轮；有多次整圈试跑时，分析较早记录须显式把 RUN_DIR 设为对应目录。分析脚本适用于单个 sqlite3 `.db3` 文件；如录包被分片，逐个传入 `.db3` 分析，并结合各片判断全程。脚本是诊断摘要，不自动给出整圈通过证明。

重点复盘：全局进度、起停段 safe/odom 速度、Safety 原因、持续静止片段、跟踪误差和规划失败原因。CAN 错误数不得持续增长。

现场记录建议保存到各轮目录的 `notes.md`：

```text
日期/人员：
新驱动器型号、固件/参数、标定结果：
SDK 版本及启动行为复核记录：
电池起止电压：
纯遥控/急停/接管：通过 / 未通过
最小底盘节点静止检查：通过 / 未通过
任务：full_loop_020
发车、到站、停车时间：
是否完整到达轨迹 finish、最后全局进度：
异常位置/时间/现象、Safety 原因、是否接管：
rosbag 与参数目录：
结论：通过 / 未通过 / 中止（原因）
```

只有整圈完整完成、关停正常且日志复盘无未解释异常，才记录“新驱动器下 0.20 m/s 基线复验通过”。本轮结束后再依据结果安排下一轮工作；不在本轮中途切到 0.50 m/s 或修改基线参数。

## 14. 本轮入口选择说明

本轮按上述多终端流程执行，暂不使用 `run_navigation.sh`、`run_guarded_progress_test.sh`、旧 `start_first_motion_smoke_test.sh` 或 0.50 m/s 整合候选。

原因：统一入口会一次拉起包含底盘的整套栈；旧短距/进度脚本仍加载旧 `competition_ws` 环境；此次需要分别确认新驱动器初始化、定位和整圈行为。以后重新启用这些脚本前，再核对它们的环境和保护流程。
