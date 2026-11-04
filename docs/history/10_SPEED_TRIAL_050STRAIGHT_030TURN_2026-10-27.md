> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-10-27 空场整圈提速：直线 0.50 / 弯道 0.30 m/s

> **新一轮修复：** V3 已完成平滑接入、完整地图保护和车端模型回归，尚待实车验收。下一轮避障请使用 [V3 操作单](18_OBSTACLE_V3_RECONNECT_2026-11-02.md)的 E 入口及录包标签；本文 A–D 和常规预检仍适用。下方 V1/V2 说明为此前阶段记录。

> **最新：避障 V1 现场未通过。** 请先阅读 [停顿与膨胀层重叠分析](17_V1_FIELD_FAILURE_AND_COSTMAP_CONSISTENCY_2026-11-02.md)，不再按下文“V1 已准备”通知直接安排下一轮。完整地图一致性修复已部署，尚未证明连续绕行问题解决。

> **避障 V1 已准备：** 下一轮避障使用 [独立 V1 入口与操作单](16_OBSTACLE_V1_CANDIDATE_2026-11-02.md)，增加安全绕行路径沿用、4 m 前视与 3 m 临障 0.20 m/s 限速。本文件原 E 仍为旧入口，不会自动启用 V1；V1 尚未实车验证。

> **避障测试追加结果：** 后续三个障碍物测试中途停止，已复现局部规划反复无路和前进动作受阻，不能将空场整圈结果视为避障通过。详见 [避障失败分析](15_OBSTACLE_FAILURE_ANALYSIS_2026-11-02.md)。

> **2026-11-02 本轮结果：** 已到路线末点，已禁用路线并确认命令与里程计零速、录包结束。首次使能至末点进度约 384.1 秒；弯道持续停车和运动模式切换原因未复现，但仍有约 70 秒恢复限速及第四弯约 32° 航向误差。详见 [本轮结果](13_050_030_FULL_LOOP_RESULT_2026-11-02.md)。这是单次整圈结果，尚非全部性能验收。

> **2026-11-02 更新：** 当前 SSH 地址为 `172.20.10.12`，电脑可用 `ssh car-ipc` 或 `ssh agilex@172.20.10.12`。10 月 27 日原包进度已到 3108，直线实测约 0.48 m/s，但存在弯道恢复与停车，不能视为性能验收通过。已修复 Safety 降速未同步缩放角速度的问题并构建车端；本轮按本文 A–G 流程直接整圈验证，参数仍为 0.50/0.30。详见 [11 月 2 日调试记录](12_TURN_RECOVERY_DEBUG_2026-11-02.md)。本次修复也作用于使用同一 Safety 包的其他启动入口；回退说明见该记录。

本轮按用户要求直接进入空场整圈提速试验，不把坐标偏差标定作为前置门槛，不安排 30 m 前置测试。使用独立候选，保留昨日 0.20 m/s 基线。以下均在小车 Bash 终端执行；电脑 PowerShell 先用 `ssh car-ipc` 登录新地址 `10.117.119.108`。

## 候选已经准备并同步到小车

| 项目 | 结果 |
|---|---|
| 几何 | 与昨日基线的 3109 点 x/y/yaw/s/curvature/ref_id 逐点一致，整圈约 155 m |
| 直线 | 参考速度最高 0.50 m/s |
| 四个转弯 | 按 entry→exit 区间限速最高 0.30 m/s，运行时包含弯前制动预览 |
| 弯内参考速度 | 受原加减速/jerk 和曲率变化率约束，生成峰值约 0.259–0.266 m/s；0.30 是上限 |
| 在线绕行曲线 | 曲率绝对值 ≥0.50 1/m 的在线局部路径额外限速 0.30 m/s |
| 参考耗时 | 359.773 s，约 6 分钟；实车还受跟踪、规划、Safety 等影响，以本轮录包为准 |
| 加速度/减速度/jerk | 0.12 / 0.18 m/s²，0.20 m/s³ |
| 正常速度上限 | MPPI 与 Safety 均为 0.50 m/s |
| Safety | 恢复速度仍为 0.15 m/s；其余误差、超时、故障、近障配置与 0.20 基线一致 |
| 当前状态 | 离线与断适配器静态加载通过；尚未实车验证本组速度 |

四个限速区间的全局索引为 451–489、1142–1180、2021–2061、2722–2760。曲线速度上限并不绕过 Safety，出现恢复条件时实际速度仍会进一步降低。

这是空场速度试验，尚未完成该速度下的专项障碍响应或制动性能验收。场地开阔是本轮条件，不据此保证所有情况下不会碰撞或停滞。

独立文件：

```text
控制：config/control/control_params_rebuild_050straight_030turn.yaml
Safety：config/safety/safety_params_rebuild_050straight_030turn.yaml
优化器：config/planning/continuous_trajectory_radius120_050straight_030turn.yaml
路线：routes/indoor_manual_six_anchor_v3_radius120_050straight_030turn_loop.yaml
轨迹：artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml
启动：rebuild_navigation_replan_radius120_050straight_030turn_guarded.launch.py
校验：scripts/verify_speed_trial_050_030.py
```

## 1. F：环境、残留进程与候选校验

每个新 ROS 终端都先加载统一环境：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
cd /home/agilex/competition_rebuild_ws
```

先在 F 检查：

```bash
pgrep -af \
'rebuild_navigation_replan|rebuild_full_navigation|rebuild_mapping|rebuild_final_map|nav2_amcl|ranger_base_node|mppi_control_node|safety_node|local_replanner|ranger_twist_adapter|pointcloud_to_laserscan|proximity_stop_node|fastlio_mapping|livox_ros_driver2' \
|| true

python3 /home/agilex/competition_rebuild_ws/scripts/verify_speed_trial_050_030.py
```

校验应输出 `status: PASS`。若旧导航仍运行，先回原终端正常退出，不能再叠加第二套。若已经启动本轮 A–D，识别它们即可，不重复启动。候选已在车端完成构建，正常情况下无需重新同步或构建。

当天确认遥控器接管、停车和物理急停可用，测试路线清空，监护人员就位。沿用已经完成恢复回归的新驱动器，不要求重新做昨日整套硬件恢复流程。

## 2. F：配置 CAN

```bash
sudo modprobe can_raw
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000 restart-ms 100
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up
ip -statistics -details link show can2
timeout 5s candump -n 20 can2
```

应为 UP、ERROR-ACTIVE、500000 bit/s，有有效回包且总线错误不持续增加。当前未知话题、旧 dropped 累计值等判断沿用昨日已经确认的规则，重点看本轮接收与错误增量。

## 3. A–D：沿用昨日传感器、地图和定位

A：

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

B，图形桌面可使用 rviz=true；纯 SSH 环境改为 false，RViz 另在图形桌面打开：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:=/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml \
  rviz:=true
```

已修正 B 默认 RViz 配置为 navigation_debug_view.rviz，包含蓝/绿/黄路径、定位激光和局部代价地图，并提供 2D Pose Estimate。正常使用此 B 不再需要 H。若 B 在纯 SSH 中以 rviz=false 运行，再在图形桌面用 H 打开同一导航视图；H 只提供显示，不能替代 B 的地图服务器。

C：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r __node:=rebuild_localization_scan \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/pointcloud_to_laserscan_rebuild_localization.yaml \
  -r cloud_in:=/cloud_registered_body \
  -r scan:=/localization/scan
```

D：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 run nav2_amcl amcl --ros-args \
  -r __node:=amcl \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/amcl_rebuild_indoor.yaml \
  -r scan:=/localization/scan
```

F 查询生命周期：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

map_server 应 active；AMCL 若 unconfigured，执行下面两条；若 inactive 只执行 activate；已 active 不重复切换：

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

## 4. F：初始位姿与贴图

车辆放在昨日标记起点、车头方向一致后，按 [昨日操作单第 5.5 节](07_DRIVER_REPLACEMENT_020MPS_RETEST_2026-10-26.md#55-f固定起点初始化) 发布名义起点。也可使用 RViz 的 2D Pose Estimate，在真实车辆所在位置初始化。

只做与当前场地、墙线一致的正常定位初始化；已经手动修正后，不再发布固定坐标覆盖它。本轮不需要测量并修正约 6° 的 body 偏差后才能继续。

```bash
timeout 5s ros2 topic echo /amcl_pose --once
timeout 8s ros2 run tf2_ros tf2_echo map body
```

实时定位激光与地图墙线贴合，TF 持续更新；起点与路线起点相容后继续。仍需保留原软件预检，不能以空场为由在定位无效时发车。

## 5. E：启动本次提速候选

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh

ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_050straight_030turn_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml \
  start_chassis_adapter:=true
```

只启动 E，不等于发车。启动入口默认适配器为 false；上面的 true 是明确接入最终控制链，路线仍保持禁用，等待后续人工使能。不得与昨日的 E、旧 0.50 整合候选或一键启动脚本并用。

仅在 B 使用 rviz=false 或只想重开显示窗口时，在图形桌面 H 打开（不重复启动地图服务器）：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
rviz2 -d /home/agilex/competition_rebuild_ws/install/rebuild_bringup/share/rebuild_bringup/rviz/navigation_debug_view.rviz
```

## 6. F：发车预检

```bash
ros2 param get /rebuild_replan_mppi trajectory_file
ros2 param get /rebuild_replan_mppi route_file
ros2 param get /rebuild_replan_mppi optimizer_params_file
ros2 param get /rebuild_replan_mppi max_speed_mps
ros2 param get /rebuild_replan_mppi max_curvature_rate_1pmps
ros2 param get /rebuild_replan_safety max_speed_mps
ros2 param get /rebuild_replan_safety recovery_speed_mps
ros2 node list | sort | uniq -d
ros2 topic info /cmd_vel -v
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
timeout 5s ros2 topic echo /system_state --once
timeout 5s ros2 topic echo /motion_state --once
timeout 5s ros2 topic echo /safety/event --once --full-length
timeout 5s ros2 topic echo /cmd_vel_safe --once
timeout 5s ros2 topic echo /cmd_vel --once
timeout 35s python3 /home/agilex/competition_rebuild_ws/scripts/navigation_preflight.py --timeout 20
```

三条路径都应属于 `050straight_030turn` 候选，后面四项数值依次为 0.50、2.50、0.50、0.15。须无同名重复节点；最终 `/cmd_vel` 唯一发布者为 rebuild_replan_ranger_twist_adapter，底盘订阅正常。

两生命周期 active，底盘正常/CAN 控制/无故障/dual Ackermann，车辆静止且两级速度命令为零，Safety 无故障，预检输出 PRECHECK PASSED 后才继续。本预检沿用起点位置 ≤0.30 m、航向 ≤12°等原条件，没有删除跟踪保护。

本轮现场曾因新 TF 监听器发现较慢，出现 body frame 不存在的误报。已修正 `navigation_preflight.py`，将 TF 就绪加入原输入等待期限，不再只在其他话题收到后固定等 2.2 秒就检查。真实预检、TF 延迟到达、TF 缺失拒绝三项检查均通过，见 [检查输出](../evidence/2026-10-27_preflight_tf_wait_checks.txt)。当时实测起点误差约 0.062 m / 4.17°，控制器为 ROUTE_DISABLED、规划器为 REFERENCE_CLEAR。修复脚本无需重启 E。

## 7. G：单独记录本轮

E 正常运行后，在 G 执行并保持前台运行：

```bash
bash /home/agilex/competition_rebuild_ws/scripts/record_navigation_trial.sh
```

脚本自动新建唯一目录、保存参数与 CAN 快照、更新 retest_current_run.txt，再开始录制。不会使能或禁用路线；Ctrl+C 只结束录包，不改变路线状态。/mission/route_enable 和 /initialpose 显式使用 Reliable/Volatile 录制，避免无发布者时自动选择的 QoS 与稍后命令不匹配。

确认录制已开始。本候选没有发布 avoidance/speed_limit 时，仅该话题等待发布者是允许的。其余核心话题应正常录入。不要同时打开两个本轮录包进程。

## 8. F：直接发车跑整圈

场地清空、监护就位、全部预检通过后，手动执行：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py start
```

新指令先执行原导航预检，再确认控制器本身的订阅与实际 DDS 匹配。它不把“有一个录包订阅者”当作控制器就绪，也不固定要求必须有两个订阅者。开始命令只发送一次 true，并等待新的控制器状态同时确认 route_enabled=true、status=TRACKING，成功输出 ACKNOWLEDGED。未确认、超时或发送后被 Ctrl+C 中断时，发送 false 撤销未确认的使能并报错；不会持续重复 true。

这替代本轮最初使用的裸 topic pub 和固定等待两个订阅者命令。录包退出会改变订阅数量，新工具不依赖录包数量判断控制器是否接收到指令。没有录包时技术上仍能得到控制器确认，但正式试跑仍按第 7 节记录。

I 可监视控制状态，F 保留供停车：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 topic echo /control/status --field data --full-length
```

直线关注实际 odom 是否逐步接近目标；弯前关注 route_speed_zone_phase 的 BRAKING，弯内为 ACTIVE、上限约 0.30。Safety 进入恢复时仍可能降至 0.15；这时记录位置和状态，不通过删保护强行维持速度。

观察轮向、实际轨迹和全局进度。目标是完整到达 3108 末点并实际到达终点；不以参考耗时作为自动结束计时，也不要求所有时刻达到速度上限。

## 9. 正常停车、关闭和分析

正常到站、车辆静止时，可先保存状态；需要中止时直接跳到下面的 false 命令，不等待快照：

```bash
RUN_DIR=$(cat /home/agilex/competition_rebuild_ws/log/retest_current_run.txt)
timeout 5s ros2 topic echo /control/status --once --full-length > "$RUN_DIR/control_at_stop.txt"
```

人工结束：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py stop
RUN_DIR=$(cat /home/agilex/competition_rebuild_ws/log/retest_current_run.txt)
timeout 5s ros2 topic echo /control/status --once --full-length
timeout 5s ros2 topic echo /cmd_vel_safe --once
timeout 5s ros2 topic echo /cmd_vel --once
timeout 5s ros2 topic echo /odom --once --field twist.twist
ip -statistics -details link show can2 > "$RUN_DIR/can_after.txt"
```

stop 指令发送 false，并等待控制器禁用、两级速度命令归零和 odom 接近静止的反馈；现场仍确认实车静止。F 若尚未设置 RUN_DIR，先用 `RUN_DIR=$(cat /home/agilex/competition_rebuild_ws/log/retest_current_run.txt)` 读取路径。危险情况直接物理急停，不等待终端命令。

关闭顺序沿用 **E → G → D → C → H/B → A**；G 正常写盘后再分析，I 的监视也 Ctrl+C。确认无残留底盘/导航进程后，可按现场需要关闭 can2。

```bash
RUN_DIR=$(cat /home/agilex/competition_rebuild_ws/log/retest_current_run.txt)
ros2 bag info "$RUN_DIR/rosbag"
python3 /home/agilex/competition_rebuild_ws/scripts/diagnose_navigation_bag.py \
  "$RUN_DIR/rosbag" | tee "$RUN_DIR/diagnosis.txt"
```

比较本轮与昨日的有效行驶时间、直线实际速度、四个弯道误差与恢复时长、局部停车次数和硬件表现。到站后等人工结束的静止时间单列，不计入纯行驶耗时；若接管、重启或重新定位，记录为中止或分段完成。

## 10. 回到昨日基线

正常停车并关闭 E 后，用原入口和原轨迹重新启动 E；A–D 已退出时先恢复定位环境：

```bash
ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_competition_safety_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml \
  start_chassis_adapter:=true
```

每次重新运行都从实际起点初始化、预检并单独录包。原 0.20 控制/Safety/optimizer/路线/轨迹/入口文件均未覆盖。

## 实现与验证记录

- 离线 retime API 和 CLI 新增读取已有 route_speed_profile，保持原冻结几何及无 profile 时的行为；避免只改正常上限、弯道离线轨迹却没有限速。
- 离线与运行时的四段路线限速均已核对，在线局部圆弧参数化另有 0.30 上限。
- 相关测试 12 passed、6 subtests passed；独立 verifier 在本机和车端均 PASS。
- 车端 competition_planning / rebuild_bringup 已构建通过，launch 参数加载通过。
- 车端静态加载时 CAN DOWN、适配器 false、route_enabled=false、cmd_vel 发布者为 0、cmd_vel_safe 全零；结束后测试节点全部退出。
- 车端源码备份目录：`/home/agilex/competition_rebuild_ws/log/speed_trial_050_030_deploy_20261027_201304`，含两份变更前源码及本次文件哈希清单。
- 静态日志：`/home/agilex/competition_rebuild_ws/log/speed_trial_050_030_static_load_20261027.log`。
- 以上是准备与静态验证，不表示本次提速实车通过；实车结果待本轮录包后填写。
