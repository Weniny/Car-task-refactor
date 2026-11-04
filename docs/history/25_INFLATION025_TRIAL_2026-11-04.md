> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-04：障碍物膨胀 0.25 m 独立试验

后续实车已完成一次整圈，用户确认障碍绕行及动态行人响应通过。当前结果和起步旧绕行线、远处停车请求的复盘见 [第 26 份记录](26_INFLATION025_FULL_LOOP_BEHAVIOR_2026-11-04.md)；下文保留本次入口部署和操作步骤。

按用户要求，新增 `forward_rejoin_050_030_inflation025.launch.py`，将 `/rebuild_replan_proximity_stop` 的 `grid_inflation_radius_m` 从 0.44 改为 0.25 m。旧入口保留 0.44；规划附加膨胀 0.04、跟踪余量 0.10、最小转弯半径 0.60、速度 0.50/0.30、Safety 与 0.40 m 近障停车框均保留。本次没有实现或部署此前讨论的 0.40 m 膨胀试验。

新入口已同步到小车独立工作区并重新构建 bringup。隔离 ROS 检查通过：参数 0.25，合成扫描实际生成的栅格膨胀半径约 0.250000 m；控制器保持 ROUTE_DISABLED，136 条安全速度均为零，最终 /cmd_vel 发布者为零。检查节点已结束，没有自动发车。

这只证明配置和栅格生成生效，不证明窄门可安全通行。现有圆形点模型在 0.25 膨胀下不能保证覆盖完整车角；没有同步改成矩形足迹碰撞模型。下一轮属于用户要求的收窄试验，需记录门口与转弯表现。

## 生效方式

旧 E 若仍在运行，先 Ctrl+C。车辆回到起点并校正定位，A–D 若仍正常则沿用。E：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup \
  forward_rejoin_050_030_inflation025.launch.py \
  start_chassis_adapter:=true
```

F 验证本次实际参数并完成门检：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
ros2 param get /rebuild_replan_proximity_stop grid_inflation_radius_m
timeout 35s python3 /home/agilex/competition_forward_rejoin_20261103_ws/scripts/navigation_preflight.py --timeout 20
```

应输出 0.25 和 PRECHECK PASSED。G 新开本轮录制：

```bash
bash /home/agilex/competition_forward_rejoin_20261103_ws/scripts/record_navigation_trial.sh obstacle_forward_rejoin_inflation025_full_loop
```

确认录制后，F 人工发车：

```bash
python3 /home/agilex/competition_forward_rejoin_20261103_ws/scripts/route_command.py start
```

停止仍用同一目录的 `route_command.py stop`，停稳后结束 G。首次对照保持门和障碍位置不变；若停车后人为移门，记录：

```bash
python3 /home/agilex/competition_forward_rejoin_20261103_ws/scripts/mark_trial_event.py obstacle_moved_after_stop
```

回到旧 0.44 只需停止新 E，再使用 `forward_rejoin_050_030.launch.py`；两套入口不能同时运行。

## 本轮前一包的已知情况

前一包最终全局进度到 3108，结束时路线禁用。全包记录到 137 条规划超时、124 条无前进可行路径；近障状态均为 clear。约 179–232 秒存在一段较长的反复无路，与用户描述的窄门停顿相符；后期还有超时，未将所有失败都归因于门。尚未完成门口几何重建和净宽测量。

## 证据

- [独立配置清单](../evidence/2026-11-04_inflation025_profile.json)。
- [实际 ROS 参数与栅格检查](../evidence/2026-11-04_inflation025_ros_interface.json)。
- 小车对应入口：`/home/agilex/competition_forward_rejoin_20261103_ws/src/forward_rejoin_bringup/launch/forward_rejoin_050_030_inflation025.launch.py`。
