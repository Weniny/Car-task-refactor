# 2026-11-04 行为修复候选操作单

本候选由当天已完成一次整圈的 0.25 膨胀版本复制建立，原工作区保留。A–D 底盘、雷达、地图和定位沿用当天流程；定位启动并进入 active 后，在 RViz 按实车位置发布初始位姿。E/F/G 使用下述新环境。E 不能与旧导航 E 同时运行。

本候选尚需实车复测，离线模型和隔离 ROS 检查不代替现场验收。扫描范围、0.25 膨胀、0.10 跟踪余量、0.60 最小转弯半径、0.50/0.30 速度及近障硬停保留。当前点模型的整车足迹覆盖限制仍存在。

## 独立安装和构建

```bash
python3 /home/agilex/install_behavior_repair_20261104.py
cd /home/agilex/competition_forward_rejoin_behavior_20261104_ws
bash scripts/build_candidate.sh
```

这些步骤不会启动硬件或发车。安装器遇到已存在的不同文件会拒绝覆盖。

## E：导航

旧 E 先 Ctrl+C；A–D 正常时沿用。正式 E 不使用测试隔离域 ROS_DOMAIN_ID=94。

```bash
source /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup forward_rejoin_behavior_050_030_inflation025.launch.py start_chassis_adapter:=true
```

## F：门检

```bash
source /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/source_candidate_runtime.sh
python3 /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/check_import_origin.py
ros2 param get /rebuild_replan_mppi require_local_plan_guard
ros2 param get /rebuild_local_replanner behavior_guard_enabled
ros2 param get /rebuild_replan_proximity_stop grid_inflation_radius_m
timeout 35s python3 /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/navigation_preflight.py --timeout 20
```

参数应为 true、true、0.25，门检通过。起步区域畅通时，等待规划状态 REFERENCE_CLEAR；确有障碍时可为完整 REPLANNED/FORWARD_REJOIN_HOLD，不能凭 VALIDATED_PREFIX 发车。定位误差门检沿用原标准。

## G：新开整圈录制

```bash
source /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/source_candidate_runtime.sh
bash /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/record_navigation_trial.sh obstacle_behavior_repair_full_loop
```

确认录制后，F 人工发车：

```bash
python3 /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/route_command.py start
```

结束时 F 停车：

```bash
python3 /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/route_command.py stop
```

确认停稳后 Ctrl+C 结束 G。录制增加 /planning/local_plan_guard。移动障碍物时标记：

```bash
python3 /home/agilex/competition_forward_rejoin_behavior_20261104_ws/scripts/mark_trial_event.py obstacle_moved
```

直接整圈，重点看空场起步的多余绕行与行人前方横穿时的连续性。前段必须满足停车距离，已经停车后仍需完整路径成功才能恢复。

## 回退

停止新 E，在新终端 source 原工作区并运行原入口：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup forward_rejoin_050_030_inflation025.launch.py start_chassis_adapter:=true
```

F/G 也切回原工作区。不能混用新门检与旧控制器。
