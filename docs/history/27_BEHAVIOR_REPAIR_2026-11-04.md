> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-04：起步旧绕行线与远端短暂停修复

实车后续：新入口已运行一轮，但起步旧绕行仍未解决。车停在起点后方约 0.29 m 的场景未被本版退出条件覆盖，复现与原因见 [第 28 份复盘](28_START_OFFSET_DIAGNOSIS_2026-11-04.md)。下文保留部署及软件检查记录，不代表此起步问题已通过实车验收。

本候选已上传小车并完成独立构建、规则/接口检查和隔离 ROS 控制链检查，尚未实车复测。原来完成整圈的工作区保留，本次没有启用真实底盘导航或真实路线。

新工作区 `/home/agilex/competition_forward_rejoin_behavior_20261104_ws`，新入口 `forward_rejoin_behavior_050_030_inflation025.launch.py`。A–D 和 RViz 沿用当天流程；E/F/G 切换到新工作区。旧导航 E 与新 E 不能同时运行。

## 修复后的行为

1. **起步前退出多余绕行。** 仅在控制器确认路线禁用、里程计静止、数据新鲜的条件下，连续两个不同栅格周期确认参考线畅通且车辆在接入位置/航向范围内，才撤销旧的回接承诺。行驶中的绕行保留原来的回接条件，不直接跳回参考线。
2. **远处路径失效不必立刻清空全部前路。** 新搜索失败时，重新用当前完整栅格验证已捕获旧路径的前段，遇到第一个碰撞或观察边界即截断，保留原曲线几何。只有这段长度覆盖当前速度的停车距离、延迟和余量时，才发布 `VALIDATED_PREFIX` 并继续尝试完整绕行；不会将障碍前的短终点当作已经完成的绕行。
3. **速度许可与路径绑定且会到期。** `/planning/local_plan_guard` 为每条路径绑定精确时间戳、有效期和限速。控制器等待路径/许可成对到达，兼容不同 DDS 到达顺序。前段许可会在每个控制周期复核剩余距离；缺失、过期、偏离或距离不足均停车。受限前段不能用近似几何替换后仍沿用原许可。
4. **停车后不凭前段反复起步。** 这种前段只维持已经正常跟踪的车辆；进入停车/定位保护后，必须等完整路径成功才恢复。

扫描范围、栅格膨胀 0.25 m、规划附加膨胀 0.04 m、跟踪余量 0.10 m、最小半径 0.60 m、直线/转弯 0.50/0.30 m/s 和近障硬停保留。MPPI 核心、Safety supervisor、定位连续性估计器与通过整圈的父版本逐字节相同。新逻辑位于独立候选的规划模块、ROS 规划适配节点及控制节点的路径许可接口中。

## 停车距离如何计算

使用当前已有的最大减速度 0.18 m/s²，基础约束为：

```text
所需距离 = v × 延迟预算 + v² / (2 × 0.18) + 0.10 m 余量
```

延迟预算包含 0.5 s 规划周期、0.4 s 搜索预算、0.15 s 命令延迟以及本次传感器/位姿数据年龄。前段限速通过该式反解；控制器持续扣除已经走过的距离，并增加许可已经经过的时间。它没有新增行人分类或运动预测，也没有把观察范围缩短。

路径许可最长约 1.05 s，并同时受障碍源时间加原 0.9 s 输入门限及一次 0.4 s 搜索预算约束。搜索入口还检查障碍、里程计和 TF 时间。超过这些时限仍会停车，因此本次修复不保证所有远处行人场景都不停顿。

## 已完成验证

| 检查 | 结果 |
|---|---|
| 父版本本地源码/配置与车端核对 | 57 个运行文件无差异；独立保存父版本归档。 |
| 规则及回调逻辑 | 37 项通过，电脑和 Jetson 均通过；包含消息乱序、许可失效、移动时不清绕行、停车后不能以前段释放等。 |
| 本轮起步快照 | 原先偏离约 0.487 m 的旧线，经连续畅通确认后变为 REFERENCE_CLEAR。 |
| 本轮远处快照 | 可验证前段约 3.594 m，允许上限 0.50 m/s，保留原远端回接目标 605。 |
| 本轮近处快照 | 仍要求停车。 |
| 连续控制模型 | 连续两次强制搜索超时，另出现一次搜索超时；合成设定在 1.5 s 移开前方冻结障碍后，通过局部路段，停车请求、许可停车和静止时间均为零。 |
| 原困难路段回归 | 66.33 s、171.33 s 两个记录路段均通过，模型停车和静止时间为零。 |
| 车端构建 | 5 个包通过，约 16.2 s；实际 import 均来自新工作区。 |
| 真实 ROS 规划节点的搜索故障注入 | 只将搜索替换为耗时 0.4 s 后超时，碰撞复核及消息发布均执行真实代码；远处快照发布 37 点 VALIDATED_PREFIX、停止请求 false；近处快照清空路径并保持停止。未创建控制器或底盘进程。 |
| 隔离 ROS | ROS_DOMAIN_ID=94、仅 localhost、无底盘适配器；模拟跟踪成功，暂停规划器后约 0.80 s 触发许可到期停车，完整路径恢复后重新跟踪，最后禁用模拟路线且安全速度为零。`/cmd_vel` 发布者始终为零。 |
| 清理 | 隔离测试节点全部退出。 |

快照验证在电脑和 Jetson 上均完成。连续模型使用冻结占用格、合成障碍移开事件及 0.25 s 速度响应假设，不能等同动态行人实测或完整整圈验收。当前 0.25 圆形膨胀的完整车角覆盖限制仍在；没有实现新的矩形足迹模型。弯后跟踪恢复降速和定位 `position_jump` 本次没有修改。

## 下一轮运行步骤

旧 E 先 Ctrl+C。A–D 和定位正常时沿用；定位未初始化则先在 RViz 发布当前实车初始位姿。

E：

```bash
source /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup forward_rejoin_behavior_050_030_inflation025.launch.py start_chassis_adapter:=true
```

F：

```bash
source /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/source_candidate_runtime.sh
python3 /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/check_import_origin.py
ros2 param get /rebuild_replan_mppi require_local_plan_guard
ros2 param get /rebuild_local_replanner behavior_guard_enabled
ros2 param get /rebuild_replan_proximity_stop grid_inflation_radius_m
timeout 35s python3 /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/navigation_preflight.py --timeout 20
```

应为 true、true、0.25，且 PRECHECK PASSED。新门检还要求当前成对路径许可有效，禁止凭 VALIDATED_PREFIX 发车。空场起步等待 REFERENCE_CLEAR；确有障碍时允许完整的绕行路径。

G 录制：

```bash
source /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/source_candidate_runtime.sh
bash /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/record_navigation_trial.sh obstacle_behavior_repair_full_loop
```

录制已启动且门检通过后，F 人工发车：

```bash
python3 /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/route_command.py start
```

结束时 F 执行 `route_command.py stop`，确认停稳后结束 G。直接整圈，重点观察空场起步、远处横穿行人和近处真实阻塞。移动障碍时用同目录 `mark_trial_event.py obstacle_moved` 标记。

回退：先停止新 E，在新终端使用 `/home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh` 和原 `forward_rejoin_050_030_inflation025.launch.py`，F/G 同时切回旧工作区。

## 文件与证据
  
- [完整离线结果](../evidence/2026-11-04_behavior_offline_checks.json)、[37 项检查输出](../evidence/2026-11-04_behavior_unit_checks.txt)。
- [电脑快照核对](../evidence/2026-11-04_behavior_recorded_cases.json)、[车端快照核对](../evidence/2026-11-04_behavior_robot_recorded_cases.json)、[真实 ROS 规划节点故障注入](../evidence/2026-11-04_behavior_ros_failures.json)、[隔离 ROS 控制链](../evidence/2026-11-04_behavior_ros_interface.json)。
- [修改差异](../evidence/2026-11-04_behavior_source_diff.txt)、[归档指纹](../evidence/2026-11-04_behavior_archive.json)。
- 本地新候选：`candidates/forward_rejoin_behavior_20261104`；车端操作单：新工作区 `RUNBOOK.md`。
- 原通过版本归档：`output/baselines/forward_rejoin_inflation025_field_pass_20261104.zip`。
