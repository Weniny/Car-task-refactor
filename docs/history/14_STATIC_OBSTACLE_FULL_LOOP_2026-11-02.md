> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-02 单静态障碍物整圈测试

> 实际现场使用三个障碍物，停车期间尝试移动障碍物，最终中途停止。本轮未完成绕行验收；分析见 [避障失败记录](15_OBSTACLE_FAILURE_ANALYSIS_2026-11-02.md)。下文为原计划，不能替代实际测试结果。

沿用已完成整圈的 0.50/0.30 配置及修复后 Safety，不改变参数。已查询运行状态：局部重规划开启，代价地图正常更新，近障 stop_distance_m=0.40，path_aware_stop_enabled=false。0.40 是配置阈值，不是保险杠实测净距或已验收制动距离。

## 现场顺序

1. 上轮已停止并禁用路线、结束录包。先在 E 按 Ctrl+C 退出该组规划/控制节点，A–D 与 B 保持运行。人工遥控返回真实起点并对齐车头，必要时按激光贴图情况修正初始位姿。
2. 在长直线中段、远离弯道/起点/终点放一个稳定、轻质且激光可见的箱体，出发前摆好，两侧留出足够的车辆绕行空间。不用人充当障碍物，不在车接近时临时放置。
3. 在 E 重启同一启动命令，清除上轮末点进度：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_050straight_030turn_guarded.launch.py \
  trajectory_file:=/home/agilex/competition_rebuild_ws/artifacts/indoor_manual_six_anchor_v3_radius120_050straight_030turn_trajectory.yaml \
  start_chassis_adapter:=true
```

4. F 预检：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/navigation_preflight.py --timeout 20
```

5. G 使用独立标签录包，自动额外保存 /scan 和 /avoidance/local_costmap；参数快照与原包均保留：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
bash /home/agilex/competition_rebuild_ws/scripts/record_navigation_trial.sh obstacle_full_loop_050straight_030turn
```

6. 确认预检通过、监护与接管准备完成，在 F 发车并等待 ACKNOWLEDGED：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py start
```

## 判断与停止

- 接近障碍物时观察局部代价地图是否出现对应占用区域，绿色局部路径是否绕开障碍物，车辆是否沿可行路径通过并回到参考路线。
- 不能只依据绿色线判断通过，需确认车体净距与实际轨迹。当前近障保护为固定前方区域，绕行时仍可能停车，不关闭保护以追求通过。
- 若停车或规划失败，先在 F 执行 stop，确认车体停稳，再处理障碍物并读取 /planning/local_replan_status、/avoidance/proximity_status、/safety/event。不要持续重复 true 或在仍使能时移开障碍物，以免自动恢复运动。
- 到达终点或提前结束，执行下列 stop 后，在 G Ctrl+C 保存；危险时优先现场接管或物理急停。

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py stop
```

本轮区分“绕行并重接成功”“保护停车”“识别或执行异常”，不把所有停车统称避障通过。本轮起步时尚未进行，结果以新目录原包和现场反馈为准。
