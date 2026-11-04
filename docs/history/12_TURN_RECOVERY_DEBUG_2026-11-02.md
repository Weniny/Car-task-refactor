> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-02 弯道恢复调试

当前连接：`ssh agilex@172.20.10.12`，电脑 `ssh car-ipc` 别名已同步。检查时无底盘、控制器或录包进程运行。未远程启动底盘、配置 CAN 或使能真实路线。

## 上轮记录

原包：`log/retest_full_loop_050straight_030turn_20261027_212854/rosbag`。

- 路线使能期间进度 0→3108，492.6 秒包含末点约 50.3 秒静止等待；不能将使能总时间等同整圈行驶时间。
- 长直线实际速度约 0.48 m/s。
- 第二弯后索引 1180 静止约 15.4 秒，第三弯后索引 2062 静止约 6.1 秒。主要触发 `lateral_error_persistent`，第二弯另有 `HYBRID_ASTAR_NO_FEASIBLE_PATH`；近障停车样本为零。
- 各弯恢复时出现运动模式反馈 0/2 切换及 `motion_mode_transition_timeout`，控制器请求仍为 dual_ackermann。不能据此认定驱动器在执行零位校准。

## 已修复的确定问题

`SafetySupervisor.filter_command` 原本仅在存在避障速度上限时才按输出速度比例缩放角速度。跟踪恢复、正常速度上限或起步加速限制同样会降低前进速度，却保留原角速度，造成转弯半径缩小。

例如原指令 v=0.30 m/s、w=-0.24 rad/s，曲率 -0.8 1/m；恢复后 v=0.15 若仍 w=-0.24，曲率变为 -1.6 1/m。现在同步变为 w=-0.12，保留原转弯半径。

修复对所有正向降速生效，保留原有速度、加减速、曲率、超时、故障、恢复和持续误差停车阈值。没有修改 MPPI 增益、路线几何或速度配置。此代码由同一 Safety 包的所有启动入口共享，历史 0.20 参数文件保留，但运行它时也会使用修复后的逻辑。

该问题与原包中降速后的急转/模式切换相符，是应先修复的放大因素；它不能单独证明入弯初始跟踪误差的全部原因，也不能保证局部规划失败或持续误差停车都已消失。

## 验证与部署

- 新增 5 项回归：左右弯恢复全过程、无避障限速的起步、正常速度上限、持续误差停车和最小转弯半径约束。
- 本地 Safety 测试 23 项、7 个子测试通过；车端 23 项 unittest 通过。
- 车端 competition_safety 构建完成，并从运行环境导入检查已安装函数包含修复。
- 备份：`/home/agilex/competition_rebuild_ws/log/turn_curvature_fix_20261102_191532`。如需回退，将该目录下 supervisor.py 恢复到源文件并重建 competition_safety；退出 E 后再操作。
- [原包诊断](../evidence/2026-11-02_050_030_bag_diagnosis.txt)、[弯道指令与模式明细](../evidence/2026-11-02_turn_details.txt)、[部署测试](../evidence/2026-11-02_turn_fix_deploy_tests.txt)。

## 本轮执行

按 [整圈运行指令](10_SPEED_TRIAL_050STRAIGHT_030TURN_2026-10-27.md) 执行 F 环境/CAN、A–D、初始化、E、G 录包、F 发车；不安排 30 米前置测试。B 已默认导航视图，无需另开 H。速度保持直线最高 0.50、弯道最高 0.30 m/s。

每个 SSH 终端先 source 统一环境。先启动并检查定位，确认激光贴图和 map→body 连通，再启 E。真实发车仍由现场人员确认后执行：

```bash
# G：持续录包
bash /home/agilex/competition_rebuild_ws/scripts/record_navigation_trial.sh

# F：预检并请求单次发车，等待 TRACKING 确认
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py start

# F：正常结束，等待路线禁用及静止反馈
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py stop
```

观察四弯有无停顿/反复回正、Safety 是否仍恢复或 HOLD、直线实际速度及路线末点进度。结束后复盘原包，对比弯道恢复时间、模式切换、最大跟踪误差与局部规划状态。若仍异常，依记录继续定位初始误差与局部轨迹衔接，不以删除 Safety 或扣除固定偏差代替修复。
