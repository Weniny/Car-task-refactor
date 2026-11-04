> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-04：碰撞后将障碍栅格膨胀改为 0.30 m

后续用户明确选择固定回 `inflation025`，当前选择以 [第 31 份记录](31_PINNED_INFLATION025_AND_REST_2026-11-04.md) 为准。本页保留 0.30 调整、验证与碰撞记录；当前暂停实测，车端运行单回切尚待恢复 SSH 后同步。

用户观察到本轮碰撞，按未通过记录。修改前确认真实导航进程已退出、/cmd_vel 无发布者，里程计线速度及角速度为零。保留原 bag，并在本轮目录增加 `operator_collision_report_20261104.json`；没有改写录包，也没有推测碰撞在 bag 中的准确时刻。

```text
/home/agilex/competition_startup_capture_20261104_ws/log/retest_obstacle_startup_capture_full_loop_20261104_142703_666820751
```

## 已修改并验证

- 新入口 `startup_capture_050_030_inflation030.launch.py`，`grid_inflation_radius_m` 从 0.25 调为 **0.30 m**。
- 执行代码 AST 对照确认，除入口说明和日志标签外，仅该运行参数变化。0.04 m 规划附加膨胀、0.10 m 跟踪余量、0.40 m 近障停车框、速度 0.50/0.30 m/s 及起步接入/前段许可逻辑保持原值。
- 车端 bringup 已重新构建，约 4.32 s；新 launch 已安装。
- 隔离域 94、仅 localhost、无底盘适配器检查通过：实际参数 0.30，真实 proximity 节点由单点扫描生成的膨胀半径约 0.300000 m。138 条安全速度均为零，最终 /cmd_vel 发布者为零，路线未使能。
- 合成测试第一轮因 TF 输入时间过旧未通过；测试脚本改为固定 20 Hz 输入，并取消重复发布空代价地图后通过。没有放松运行节点的时间保护。
- 车端 RUNBOOK.md 与构建脚本的入口显示已切换 inflation030。0.25 原入口、原运行单和原清单均保留，当前清单版本为 `startup_capture_20261104_inflation030`，126 文件完整性检查通过。

0.30 是本次栅格膨胀参数，不是行人检测距离。调整后仍需实车复测，不能据此确认碰撞原因已经解决。本次没有自动启动真实导航或发车。

## 重新启动 E

旧 E 若还在运行先 Ctrl+C。A–D 和定位正常时沿用。当前小车已更新，无需重新运行原 0.25 基础安装器。

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup startup_capture_050_030_inflation030.launch.py start_chassis_adapter:=true
```

## F 核对与门检

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
ros2 param get /rebuild_replan_proximity_stop grid_inflation_radius_m
timeout 35s python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/navigation_preflight.py --timeout 20
```

必须看到 `Double value is: 0.3` 和 `PRECHECK PASSED`。旧的 inflation025 入口仍会使用 0.25，不能只重启旧命令便认为新参数已生效。

## G 新录制及人工发车

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
bash /home/agilex/competition_startup_capture_20261104_ws/scripts/record_navigation_trial.sh obstacle_startup_capture_inflation030_full_loop
```

录制开始、门检通过后 F：

```bash
python3 /home/agilex/competition_startup_capture_20261104_ws/scripts/route_command.py start
```

结束 F 执行同目录 `route_command.py stop`，确认停稳后结束 G。本次调整未触发这一步。

## 证据

- [0.30 配置与单因素检查](../evidence/2026-11-04_inflation030_profile.json)。
- [实际参数、栅格半径与零输出检查](../evidence/2026-11-04_inflation030_ros.json)。
- [更新包与完整包指纹](../evidence/2026-11-04_inflation030_archive.json)。
- 完整版本包：`output/candidates/startup_capture_inflation030_20261104.zip`；基础版本的更新包：`output/candidates/inflation030_update_20261104.zip`。原 0.25 版本归档保留。
