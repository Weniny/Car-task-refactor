# 2026-11-03：独立回接修复候选与次日操作单

今晚不进行实车测试。本候选从已归档的原 0.50/0.30 基线建立，独立目录为 `candidates/forward_rejoin_20261103`，没有覆盖旧运行工程，也没有启动底盘或发车。

**状态：离线检查通过，车端 ROS 构建和运行接口门检待次日执行，实车未验收。** 下文包含这两项前置检查，不应跳过直接发车。模型结果不是现场避障效果保证。

## 修改范围

1. 遇到参考线阻塞后，在收到的栅格边界内选择障碍后方、具有连续回接空间的目标，不再退到障碍前的短直线作为绕行结果。
2. 本次目标选定后保持，只有目标失效或完成回接才改变；每周期用新地图检查已有绕行。安全路径可继续使用，失效路径重新搜索。车辆尚未回到参考线附近、航向尚未符合条件时，不直接切回全局参考线。
3. 保留基础膨胀 0.44 m、原附加膨胀 0.04 m，另设 **0.10 m 跟踪余量**。它会让候选的规划碰撞边界额外向外扩 0.10 m；不是放松保护，也不是一个已实测标定的最终最优值，窄通道可能更容易判无路。
4. 搜索起点采用里程计角速度/线速度推算的当前曲率，不再一律从零曲率起步。低速时保留最近估计，异常大值限幅。这是估计，不是轮转角实测。
5. 普通避障输入使用收到栅格的完整范围和占用格，取消这一分支的 2000 点抽样；路径碰撞检查覆盖点间线段。规划失败时清空绿线，控制器同步清理已失效局部路径。

MPPI 核心、Safety 规则和已验证的曲率保持修复沿用原基线。速度仍为直线最高 0.50、转弯区最高 0.30 m/s；规划半径默认仍为 0.60 m。没有启用 V1 的临障 0.20 限速、V3 的平滑接入或障碍启发式。

0.81 m 单独对照在所测片段仍停滞，所以不作为明天默认参数。完整版本差异和来源清单随候选打包。

## 离线检查结果及边界

| 检查 | 结果 |
|---|---|
| 回接选择、观察边界和路径误差规则 | 10 项通过 |
| 起始曲率、点间碰撞、回接交接、失败清线等接口逻辑 | 6 项通过；ROS 回调使用独立桩测试，尚非在线 DDS 验证 |
| 冻结整圈 3109 点参考线与新增余量的静态密集检查 | 通过，无阻塞点；不代替空场实车 |
| 五个记录片段的连续规划＋MPPI＋Safety 模型 | 5/5 通过，规划停车和静止时间均为零 |
| 含 0.15 s 规划交付延迟、0.25 s 速度响应时间常数、-4° 行驶偏差的用例 | 通过，最大局部跟踪误差约 0.069 m |
| 停稳后移开人工封路障碍 | 通过；模型移开后约 1.6 s 恢复到 >0.05 m/s |
| 绕行中新增封路障碍，停稳后移开 | 通过；原路径失效后停车，移开后恢复，无模型碰撞 |
| 0.81 m 单独对照 | 未通过，保留 0.60 m |

正式模型包含原 0.40 m 近障停止框，原包原始占用点取最近的停车前快照，五个静态片段中未触发。模型只覆盖所选局部通行片段，不是全圈仿真；使用冻结障碍和理想车体曲率，缺少原始三维感知、独立轮角和完整底盘动力学。电脑端耗时不可当成 Jetson 实时性能验收。

开发中失败的目标前移、过早切回参考线、零曲率重新起步等试验记录保留，最终结果只指当前候选。用户补充：障碍主要在停住后被伙伴移动；不把停后移开后恢复视为一次完整静态避障成功。

## 0. 次日传输和独立构建

电脑 PowerShell，在项目目录执行；SSH 地址变化时先更新 `car-ipc`：

```powershell
scp .\output\candidates\forward_rejoin_20261103.zip .\output\candidates\install_forward_rejoin_20261103.py car-ipc:/home/agilex/
ssh car-ipc
```

小车 Bash：

```bash
python3 /home/agilex/install_forward_rejoin_20261103.py
cd /home/agilex/competition_forward_rejoin_20261103_ws
conda deactivate 2>/dev/null || true
bash scripts/build_candidate.sh
```

安装器只解包到新目录；同版本可重复核对，遇到不同的已有文件拒绝覆盖。构建脚本检查完整性、构建四个比赛包及独立 bringup、核对 Python 实际导入路径、运行 10 项规则测试和 6 项接口逻辑测试，并用 `--show-args` 检查 launch 加载。它不启动导航或底盘。任一步失败，保留输出先处理，不继续发车。

新环境用于 E/F/G。A–D 继续使用原重构环境，只负责底盘反馈、传感器、地图、定位。旧 E、V1/V3 和新 E 不能同时运行。

## 1. CAN 与 A–D

CAN 已由当天操作确认正常则不重复配置；首次开机按原设置：

```bash
sudo modprobe can_raw
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000 restart-ms 100
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up
ip -statistics -details link show can2
timeout 5s candump -n 20 can2
```

A：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_mapping.launch.py \
  start_livox:=true force_livox_host_timestamps:=true \
  start_fast_lio:=true fast_lio_config:=fast_lio_mid360_navigation.yaml \
  start_base:=true port_name:=can2 publish_odom_tf:=false \
  start_scan:=false start_slam:=false start_anchor:=false rviz:=false
```

B：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:=/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml rviz:=false
```

图形桌面另开 RViz（只显示，不替代 B）：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
rviz2 -d /home/agilex/competition_forward_rejoin_20261103_ws/src/forward_rejoin_bringup/rviz/navigation_debug_view.rviz
```

C：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r __node:=rebuild_localization_scan \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/pointcloud_to_laserscan_rebuild_localization.yaml \
  -r cloud_in:=/cloud_registered_body -r scan:=/localization/scan
```

D：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 run nav2_amcl amcl --ros-args -r __node:=amcl \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/amcl_rebuild_indoor.yaml \
  -r scan:=/localization/scan
```

确认 map_server、amcl 为 active；AMCL 若 unconfigured，configure 后 activate。使用 RViz 的 2D Pose Estimate 初始化并对齐实时定位激光，已手动修正后不要再发固定坐标覆盖。车辆放回路线起点，朝向与路线相符。此阶段保护与遥控接管沿用已验证流程。

## 2. E：先检查断开适配器的候选

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
cd /home/agilex/competition_forward_rejoin_20261103_ws
python3 scripts/check_import_origin.py
ros2 launch forward_rejoin_bringup forward_rejoin_050_030.launch.py start_chassis_adapter:=false
```

F 用同一新环境检查：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
ros2 param get /rebuild_local_replanner forward_rejoin_enabled
ros2 param get /rebuild_local_replanner tracking_clearance_m
ros2 param get /rebuild_local_replanner min_turning_radius_m
ros2 param get /rebuild_replan_mppi max_speed_mps
ros2 node list
timeout 8s ros2 run tf2_ros tf2_echo map body
timeout 5s ros2 topic echo /control/status --once --full-length
timeout 5s ros2 topic echo /planning/local_replan_status --once --full-length
ros2 topic info /cmd_vel -v
```

参数应为 true、0.10、0.60、0.50。控制器应为 ROUTE_DISABLED，路径、扫描、栅格、TF 正常；断开适配器时最终 /cmd_vel 不应出现本候选发布者，也不应有旧导航发布者。这个阶段不运行完整发车门检，因为它要求适配器已接通。

## 3. E 接通适配器、F 门检、G 录制

E 按 Ctrl+C 停止上一步，然后在同一 E 重启一次：

```bash
ros2 launch forward_rejoin_bringup forward_rejoin_050_030.launch.py start_chassis_adapter:=true
```

默认路线仍禁用。F：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
cd /home/agilex/competition_forward_rejoin_20261103_ws
python3 scripts/check_import_origin.py
timeout 35s python3 scripts/navigation_preflight.py --timeout 20
ros2 topic info /cmd_vel -v
```

门检必须通过，最终发布者唯一且是 rebuild_replan_ranger_twist_adapter。G：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
bash /home/agilex/competition_forward_rejoin_20261103_ws/scripts/record_navigation_trial.sh obstacle_forward_rejoin_empty_full_loop
```

确认录制开始后，F 人工发车：

```bash
python3 /home/agilex/competition_forward_rejoin_20261103_ws/scripts/route_command.py start
```

工具先执行门检、只发送一次使能并等待控制器 ACK；失败时撤销使能。不要再用固定匹配两个订阅者的裸 topic pub。

## 4. 整圈测试安排

1. **第一圈空场整圈**：确认原 3109 点路线、四弯限速、显示与末点行为。没有 30 m 前置测试。
2. 第一圈通过后，车辆回起点，重新初始化/确认定位并重启 E，进行 **三个静止障碍物整圈**。G 标签改为 `obstacle_forward_rejoin_static_full_loop`，本圈不要移动障碍，便于判断完整绕行是否成立。
3. 静态通过后，再单独安排 `obstacle_forward_rejoin_move_after_stop` 记录受控停车后移开障碍的恢复；不要混入静态验收。移动当时在 F 标记：

```bash
python3 /home/agilex/competition_forward_rejoin_20261103_ws/scripts/mark_trial_event.py obstacle_moved_after_stop
```

观察重点：有没有持续的正常绕行、是否反复换边/急着回线、绿线清除与新路线发布是否对应规划状态、静态障碍旁是否长停。静态测试若再次持续停住，先停止路线并保留录包，避免移动障碍后把恢复记成静态通过。实车效果、Jetson 耗时和新余量在实际通道中的适用性以录包为准。

## 5. 停止与回退

正常结束或中止，F：

```bash
python3 /home/agilex/competition_forward_rejoin_20261103_ws/scripts/route_command.py stop
```

确认命令与里程计零速、ACK 后，在 G Ctrl+C 保存录包，再在 E Ctrl+C。紧急情况沿用物理急停。原路线到末点后仍需人工停止，本次没有改变自动结束策略。

回退只需停止新 E，在新终端加载原重构环境；本候选没有覆盖原安装目录。注意原重构目录当前仍含后续 V1/V3 共享源码，**不能把重新启动旧 launch 误称为恢复已验证原 0.50 源码**。真正的基线回退应使用已归档重建副本另行构建，不在现场直接覆盖或混用。

