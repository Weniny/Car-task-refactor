# 当前运行指令：起步接入修复，障碍膨胀 0.25 m

更新：2026-11-05。日常运行只使用本文；历史文档编号表示记录顺序，不是软件版本号。冻结标签与依赖见[项目状态](PROJECT_STATE.md)。

## 1. 当前到底是哪一套

当前算法对应原 **29 号文档**的起步接入修复；31 号只是后来固定回 0.25 的选择记录，未增加算法修改。30 号的 0.30 膨胀是临时试验，目前不作为默认入口。

| 项目 | 当前固定值 |
|---|---|
| 小车工作区 | `/home/agilex/competition_startup_capture_20261104_ws` |
| 环境脚本 | `scripts/source_candidate_runtime.sh`，位于上述工作区 |
| 导航入口 | `startup_capture_050_030_inflation025.launch.py` |
| 障碍栅格膨胀 | **0.25 m** |
| 速度上限 | 直线 0.50、转弯区 0.30 m/s |
| 规划附加膨胀 / 跟踪余量 | 0.04 / 0.10 m |
| 最小转弯半径 | 0.60 m |
| 起步位置 / 航向门限 | 距起点不超过 0.30 m / 12° |
| 测试长度 | 直接整圈，不增加 30 m 测试 |

它保留远端受阻时的前段许可机制，并增加起点后方小偏差的平顺接入；不是 V1/V3，也不是早先只有单次整圈通过记录的 forward_rejoin 版本。

## 2. 当前状态

- 下一轮 1.0 m/s、连续转弯及静态/动态避障工作统一见[升级计划](UPGRADE_PLAN.md)。当前仅完成本地审计与设计，未部署高速候选。用户最新澄清：小马扎为细腿折叠款，近距离通过但没有接触；其物理净距与感知覆盖尚待核验，不据此改写更早事件的历史记录。
- **最近复测：重启全部终端后，用户现场反馈最后一弯未再停车。** 本次未改代码或参数；新录包尚未核验，不能据此确认根因已消除。用户此前报告的点云/TF 漂移仍保留为待排查事项。
- 这套 025 算法入口此前已在车端部署、构建，47 项检查、录包模型和隔离 ROS 检查通过。
- 后续实车报告过碰撞，**本候选实车验收未通过**。不能把旧版本的单次整圈通过或软件检查通过当作本候选已验收。
- 本地与车端默认 RUNBOOK、版本清单已同步固定回 025；车端 127 文件校验通过，025 入口配置核对为 0.25 m。核对范围为文件与部署入口，实际 ROS 参数仍在 E 启动后由 F 检查。

11 月 5 日 SSH 已恢复至 `192.168.0.193`，已完成文件、导入来源与 A–D 依赖快照核对；最新复测包的分析尚未完成。本次冻结未修改车端运行文件或发车，具体范围见[项目状态](PROJECT_STATE.md)。

### 最近两轮：最后一个弯道

| 轮次 | 操作与结果 | 证据范围 |
|---|---|---|
| 前一轮 | 最后一弯反复停车；录包中首次请求停止为 `no forward rejoin with clear run beyond blockage`，控制状态为 `LOCAL_PLANNER_STOP` | 已核验录包；该包没有 `position_jump`。遥控回起点后实时读到的跳变保护属于另一个时段 |
| 全终端重启后复测 | 用户反馈测试结束，最后一弯未再出现同一问题 | 现场反馈；新录包、参数和末点进度待核验 |

保持当前 0.25 入口与原速度参数，不以这一次结果调整规划权重或关闭定位保护。[本次对照记录](evidence/2026-11-04_last_corner_restart_retest.json)区分现场反馈与已核验数据。下一轮运行前继续确认静止点云与 TF 稳定，并新建录包。

## 3. 恢复后的使用顺序

冷启动先完成[附录 A–D 和 RViz](#附录冷启动-ad-和-rviz)。A–D 已健康运行时，直接从 E 开始，避免重复启动节点。先确认静止状态下点云、定位激光与地图、TF 稳定；仍漂移时先处理定位问题。

### E：当前导航入口

退出旧 E 后，在新 E 执行：

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup startup_capture_050_030_inflation025.launch.py start_chassis_adapter:=true
```

这一步加载导航和底盘适配器，路线仍默认禁用。不要同时运行旧工作区的导航 E。

### F：确认版本和门检

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/check_import_origin.py
ros2 param get /rebuild_replan_proximity_stop grid_inflation_radius_m
timeout 35s python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/navigation_preflight.py --timeout 20
```

应看到 import 检查 PASS、`Double value is: 0.25`、`PRECHECK PASSED`。门检还会确认起步评估完成、路径许可有效及唯一底盘速度发布者。不能仅凭终端没有报错就发车。

| 规划状态 | 含义 |
|---|---|
| `REFERENCE_CLEAR` | 前方参考段可通行 |
| `STARTUP_CAPTURE` / `STARTUP_CAPTURE_HOLD` | 当前位姿到参考线的完整起步接入路径，允许正常门检 |
| `REPLANNED` / `FORWARD_REJOIN_HOLD` | 完整绕行或保留的完整回接路径，仍需门检通过 |
| `VALIDATED_PREFIX` | 失败后仅用于停车余量的前段，不能作为发车依据 |

`STARTUP_CAPTURE` 适用于起点后方、横向在既有 0.10 m 跟踪余量内的小偏差；新曲线仍检查碰撞和曲率，其他情况沿用原规划与门检。

### G：先开启新的整圈录制

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
bash /home/agilex/competition_startup_capture_20261104_ws/scripts/record_navigation_trial.sh obstacle_startup_capture_inflation025_full_loop
```

使用 `obstacle_` 标签会同时记录扫描和代价地图。录制器显示新的 Recording 目录后，再在 F 发车。

### F：人工发车

```bash
python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/route_command.py start
```

该工具重新门检并等待控制器确认，不使用裸 `/mission/route_enable` 发布，也不固定等待两个订阅者。

### 结束与保存

F 停车：

```bash
python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/route_command.py stop
```

确认停稳后，在 G 按 Ctrl+C 保存录包，再结束 E。全系统关闭时，再结束 D、C、B、RViz，最后结束 A。到达末点后仍按上述步骤人工结束；中途异常时保留同一轮记录。

停车后移动障碍物时，F 可作事件标记：

```bash
python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/mark_trial_event.py obstacle_moved_after_stop
```

该脚本另外支持 `obstacle_placed`、`obstacle_removed`。录包路径保存在工作区 `log/retest_current_run.txt`。

## 附录：冷启动 A–D 和 RViz

A–D 沿用原重构工作区，仅负责底盘、传感器、地图和定位；E/F/G 使用上面的 startup_capture 工作区。

### CAN：仅在尚未配置时

```bash
sudo modprobe can_raw
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000 restart-ms 100
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up
ip -statistics -details link show can2
timeout 5s candump -n 20 can2
```

核对接口 UP、500000、正常回包及错误计数。已配置且正常时沿用。

### A：雷达、FAST-LIO、底盘反馈

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_mapping.launch.py \
  start_livox:=true force_livox_host_timestamps:=true \
  start_fast_lio:=true fast_lio_config:=fast_lio_mid360_navigation.yaml \
  start_base:=true port_name:=can2 publish_odom_tf:=false \
  start_scan:=false start_slam:=false start_anchor:=false rviz:=false
```

### B：同时加载静态地图和 RViz

在小车图形桌面的终端 B 执行。若旧 B 或独立 RViz 已打开，先关闭旧 B 和原独立窗口，再运行一次：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:=/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml \
  rviz:=true \
  rviz_config:=/home/agilex/competition_startup_capture_20261104_ws/src/forward_rejoin_bringup/rviz/navigation_debug_view.rviz
```

B 会同时启动地图服务器和当前导航调试视图，无需另开 H/RViz 终端。

### C：定位激光

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r __node:=rebuild_localization_scan \
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

F 确认生命周期：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

两者应为 active。AMCL 若为 unconfigured，执行一次：

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

### 在 B 打开的 RViz 中设置初始位姿

B 已同时加载地图和 RViz。AMCL active、地图和定位扫描就绪后，用该窗口的 **2D Pose Estimate** 按实车真实位置、朝向初始化；激光与地图贴合后检查 `map -> body`。不再发布固定坐标覆盖手动修正结果。

| 显示 | 话题 | 何时出现 |
|---|---|---|
| 地图 | `/map` | B 启动后 |
| 定位激光 | `/localization/scan` | C 启动后；地图坐标显示还需要定位 TF |
| 蓝线：全局参考 | `/planning/replan_reference_path` | E 启动后 |
| 绿线：局部轨迹 | `/planning/local_trajectory` | E 正常规划后 |
| 黄线：实际轨迹 | `/control/replan_executed_path` | E 获得定位后累积，静止时可能很短 |
| 代价地图 | `/avoidance/local_costmap` | E 收到有效扫描后 |

定位激光采用 Best Effort；蓝线采用 Reliable / Transient Local；其余路径与代价地图按配置中的 Reliable / Volatile 显示。初始化前 TF 尚未连通不等于导航算法版本错误；初始化后仍漂移则先处理定位。

## 来源和版本记录

- [版本对照与验证状态](VERSION_HISTORY.md)。
- [29 号完整技术记录](history/29_STARTUP_CAPTURE_REPAIR_2026-11-04.md)、[31 号固定选择记录](history/31_PINNED_INFLATION025_AND_REST_2026-11-04.md)。
- [固定入口与车端同步证据](evidence/2026-11-04_pinned_inflation025.json)。
