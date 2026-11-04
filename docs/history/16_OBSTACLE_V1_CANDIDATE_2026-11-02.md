> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-02 避障 V1 候选与运行指令

> **现场结果更新：V1 未通过。** 实车仍反复停顿，已发现规划输入抽样及绿线/障碍层一致性问题。下文保留为本轮历史操作，不再作为下一轮推荐。见 [现场失败与一致性修复](17_V1_FIELD_FAILURE_AND_COSTMAP_CONSISTENCY_2026-11-02.md)。新修复尚未完成连续绕行验证，不凭软件测试直接安排实车验收。

已修改源代码并部署，使用独立启动文件。旧 0.50/0.30 启动入口保留；新增行为默认关闭，旧入口不启用 V1 策略。仍保留此前共享 Safety 的降速曲率修复。

## 改动

| 项目 | V1 行为 |
|---|---|
| 绕行连续性 | 生成绕行后，剩余有效路径至少 1 m 时优先沿用，状态 DETOUR_CONTINUE；每轮用最新障碍层重查碰撞、接入段航向和转弯半径 |
| 接入段检查 | 每 2.5 cm 插值检查，防止当前位置到旧路径之间漏检障碍；出现新障碍或姿态不相容则放弃沿用 |
| 前视 | 3→4 m；实时障碍读取范围前向 4→5 m（代价地图仍覆盖到 5.5 m） |
| 短回接 | 最小实际路径长度 2 m，不再退到 1.2 m 的短回接 |
| 临障速度 | 固定前方区域延长到 3 m、横向半宽 0.8 m，足够激光点时上限 0.20 m/s；清空至少 0.6 秒及连续清晰扫描后解除 |
| 速度链 | /avoidance/speed_limit 同时供控制器和 Safety 使用，Safety 必须收到新鲜有效心跳 |
| 数据异常 | V1 激光允许年龄 0.30 秒；过期、未来/缺失时间戳或错误 frame 均停车 |
| 保留保护 | 原 0.40 m 近障停车、0.44 m 障碍膨胀、额外 0.04 m 规划膨胀、最小转弯半径、恢复/持续误差停车等保持 |

空场正常速度上限仍是 0.50/0.30。软限速区域可能包含侧方物体或墙，因此局部直线实际速度也可能降到 0.20。未切换成路径感知停车，未启用倒车/原地旋转脱困。

## 验证边界

- 本地相关 pytest：41 项、13 子测试通过。车端规划/Safety unittest、三包构建通过；ROS 隔离测试验证 2 m 障碍限速、近障停车、解除延迟和时间/frame 异常停车。
- 独立 ROS_DOMAIN_ID=93 中启动 V1，适配器关闭：核对新参数、确认安全输出零速、最终 /cmd_vel 无发布者。未向真实路线发送 true，未启动底盘。
- 原包激光开环回放，新软限速首次触发约在录包第 35.10 秒，比第一次局部规划停车早约 5.81 秒。录制期间约 76.7% 的使能扫描满足限速/解除延迟条件；其中包括长时间停车和人工移动障碍物，不能当作新车速闭环性能预测。
- 三个原包无路快照应用 V1 新前视/短回接参数后仍无路。V1 不声称能够从原来的死局安全脱困；软限速与路径连续性旨在提前改变行驶过程，需要新一轮实车验证能否避免死局。不能将上述开环结果称为避障成功。
- 本轮未根据未经量化的假设修改初始转向曲率、车体包络或 TF 时间对齐；这些保留为后续有证据的改进项。

证据：[构建及 ROS 测试](../evidence/2026-11-02_obstacle_v1_build_tests.txt)、[最终规划回归](../evidence/2026-11-02_obstacle_v1_final_planning_tests.txt)、[静态加载](../evidence/2026-11-02_obstacle_v1_static_check.json)、[激光回放](../evidence/2026-11-02_obstacle_v1_scan_replay.json)、[死局快照](../evidence/2026-11-02_obstacle_v1_snapshot_check.json)。

部署前备份：`/home/agilex/competition_rebuild_ws/log/obstacle_v1_backup_20261102_203130`。回退运行策略可退出 V1，重新启用原来的 050straight_030turn 启动入口；彻底回退代码需恢复备份源文件并重建对应包。

## 下一轮操作

继续使用原 A–D，确认地图/激光贴合。人工返回实际起点，再启动新 E。首次验证建议先摆一个障碍物在长直线中段，两侧有绕行空间；车辆接近时不移动障碍物。不是额外的 30 m 测试，可直接整圈。

E（退出所有旧 E 后）：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_050_030_obstacle_v1.launch.py \
  start_chassis_adapter:=true
```

F：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/navigation_preflight.py --timeout 20
ros2 param get /rebuild_local_replanner retain_safe_detour
ros2 param get /rebuild_replan_proximity_stop obstacle_slowdown_enabled
ros2 param get /rebuild_replan_safety require_avoidance_speed_limit
```

应预检通过，三个参数均 true。G：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
bash /home/agilex/competition_rebuild_ws/scripts/record_navigation_trial.sh obstacle_v1_050_030
```

F，现场确认后手动发车：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py start
```

观察是否提前降速、绿色路径是否稳定绕行并重接、有没有再次停在障碍物旁。停车或结束后先执行 stop 并确认停稳，再接近或移动障碍物，最后 G Ctrl+C 保存：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py stop
```

若重新遇到无路，不反复使能或强行通过，保留原包继续分析。
