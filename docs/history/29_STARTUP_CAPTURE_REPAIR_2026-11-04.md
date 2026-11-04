> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-04：起点后方的平顺起步接入修复

后续实车用户报告碰撞，本轮未通过。已按要求将下一轮障碍栅格膨胀改为 0.30 m，当前入口和命令以 [第 30 份操作单](30_INFLATION030_AFTER_COLLISION_2026-11-04.md) 为准，下文 0.25 入口保留为历史。

新候选已上传小车，完成独立构建、47 项规则与接口检查、实录起步模型和隔离 ROS 检查。实车整圈尚未复测，本次没有启用真实底盘路线。旧工作区和归档均保留。

工作区：`/home/agilex/competition_startup_capture_20261104_ws`。入口：`startup_capture_050_030_inflation025.launch.py`。车端完整操作单为该工作区 `RUNBOOK.md`。

## 本次修改

针对“距起点后方约 0.29 m、横向约 0.036 m、航向差约 6.76°”的实录场景，起步评估分别计算纵向、横向和航向差，不再要求后方的端点距离也必须小于到点容差 0.15 m。

在路线禁用、车辆静止、数据新鲜时，连续两次不同栅格的评估确认原参考段与新接入段可通行后，用从实车定位位姿出发、与前方参考方向相接的三次曲线替换旧线。曲线检查密集采样曲率、输出点间曲率和完整栅格碰撞；不修改定位、不平移全局轨迹、不调用 Hybrid A* 重新绕一大圈。

新分支仅处理起点后方、距起点不超过原门限 0.30 m、朝向差不超过原门限 12°、横向仍处于既有 0.10 m 跟踪余量范围内的情况。其他情况沿用原规划与门检，不能将此候选解释为所有起步位置均适用的快捷接入。

车辆开始运动后沿用已验证的接入曲线，按原回接条件交还参考线。状态为 `STARTUP_CAPTURE`、`STARTUP_CAPTURE_HOLD`；这两者表示完整的起步接入路径，区别于失败后仅用于停车余量的 `VALIDATED_PREFIX`。

门检新增 `startup_capture_supported=true`、`startup_assessed=true` 检查，既防止新脚本误用旧运行节点，也等待静止起步评估完成。发车门限、0.25 m 栅格膨胀、0.04 m 附加膨胀、0.10 m 跟踪余量、0.60 m 最小转弯半径、0.50/0.30 m/s 速度均保持。MPPI 控制节点/核心、Safety supervisor、状态估计器及动态前段许可模块与上一版本逐字节相同。

## 验证结果

| 检查 | 结果 |
|---|---|
| 规则及接口 | 47 项通过，电脑与 Jetson 均通过，新增 10 项覆盖后方偏移、坐标旋转、门限、碰撞和行驶后保持。 |
| 本轮实际起步快照 | 第二次有效评估得到 STARTUP_CAPTURE，起点为记录中的实车定位位姿，终点为原参考第 61 点。 |
| 接入几何 | 相对起点参考方向最大横向约 0.0400 m，密集输出点最大曲率约 0.1108 1/m，低于现有 1/0.60 上限。 |
| 连续规划＋MPPI＋Safety 模型 | 从零速度起步，含 0.25 s 速度响应假设与 0.15 s 路径交付延迟；14.9 s 通过约 4 m 的起步路段，最大横向约 0.0388 m，保护停车时间 0。 |
| 父版本记录回归 | 原起步清线、远处受阻前段、近处受阻停车三个快照检查均通过。 |
| 车端构建 | 5 包通过，约 16.3 s；新模块与各包实际 import 均来自新工作区。 |
| 隔离 ROS | 使用同一后方起步位姿、合成空场扫描，实际节点生成 STARTUP_CAPTURE，模拟跟踪成功；许可到期停车及完整路径恢复检查通过。 |
| 硬件输出与清理 | ROS_DOMAIN_ID=94、仅 localhost、无底盘适配器，/cmd_vel 发布者为零；最终模拟路线禁用、安全速度为零，测试节点已退出。 |

模型仅覆盖冻结障碍下的局部起步和交接，不能替代真实整圈。原 0.25 m 点碰撞模型的完整车角覆盖限制仍在，本次未调整足迹、弯后跟踪恢复和定位跳变保护。

## 下一轮 E/F/G

A–D 和 RViz 正常时沿用。先退出旧 E；定位未初始化时，先按真实车位在 RViz 发布初始位姿。

E：

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup startup_capture_050_030_inflation025.launch.py start_chassis_adapter:=true
```

F：

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/check_import_origin.py
timeout 35s python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/navigation_preflight.py --timeout 20
```

应出现 PRECHECK PASSED。起点后方小偏差可见 STARTUP_CAPTURE，已贴近参考线可见 REFERENCE_CLEAR；前方确有障碍时允许完整绕行。若提示等待起步评估，继续等第二次新鲜评估，不能使用原始 Bool 指令跳过门检。

G：

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
bash /home/agilex/competition_startup_capture_20261104_ws/scripts/record_navigation_trial.sh obstacle_startup_capture_full_loop
```

录制开始、门检通过后，F 人工发车：

```bash
python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/route_command.py start
```

结束时 F：

```bash
python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/route_command.py stop
```

确认停稳后结束 G。本轮直接整圈，重点看起步绿线与实际转向，再记录后续避障。移动障碍物时可用同目录 `mark_trial_event.py obstacle_moved` 标记。

回退时先退出新 E，使用 `/home/agilex/competition_forward_rejoin_behavior_20261104_ws` 的环境与 `forward_rejoin_behavior_050_030_inflation025.launch.py`，F/G 同步切回。该父版本仍有第 28 份记录的后方起步问题。

## 证据

- [47 项检查输出](../evidence/2026-11-04_startup_capture_checks.txt)。
- [本轮实录连续起步模型](../evidence/2026-11-04_startup_capture_model.json)、[父版本快照回归](../evidence/2026-11-04_startup_capture_parent_cases.json)。
- [隔离 ROS 实际节点验证](../evidence/2026-11-04_startup_capture_ros.json)。
- [归档与源码不变项](../evidence/2026-11-04_startup_capture_archive.json)、[差异](../evidence/2026-11-04_startup_capture_diff.txt)。
- 新包 `output/candidates/startup_capture_20261104.zip`，SHA256 `dd0e709bc8b316559c8e7fd1085519302edef0b6b0bd84e2fd1f74efb87f24ae`，122 个清单文件。车端完整性和文件哈希复核通过。
