> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-04：候选已上传构建，等待现场门检

SSH 已连接 `192.168.0.193`。候选已安装到 `/home/agilex/competition_forward_rejoin_20261103_ws`，没有覆盖原重构或比赛工作区。

## 已完成

- 安装包与 104 个文件的哈希校验通过。
- 五个 ROS 包构建成功；10 项回接规则、6 项接口逻辑测试通过。
- 规划、控制、Safety、定位估计模块均从新候选目录导入。
- launch 参数加载通过；默认底盘适配器关闭，路线需要显式使能。
- 在隔离 ROS_DOMAIN_ID=94 启动真实候选节点，用零运动的合成 TF、里程计、扫描、栅格和底盘状态检查接口。控制器 ROUTE_DISABLED、规划器 REFERENCE_CLEAR，145 条安全命令全零，最终 /cmd_vel 发布者为零；新参数 true/0.10/0.60 正确。
- 隔离检查结束后相关节点已退出。检查结束时未发现底盘、导航控制、FAST-LIO/Livox 进程。

这是软件与接口检查。**真实传感器、定位和底盘门检尚待场地启动后完成，路线没有使能，也未发车。** 新参数和功能的现场效果仍未验收。

## 到场后的操作

1. 安装和构建已完成，不需再上传。按[完整操作单第 1 节](23_FORWARD_REJOIN_CANDIDATE_2026-11-03.md)配置 CAN、启动 A–D，打开导航 RViz，校正初始位姿。确认激光贴图、地图和 TF 正常。
2. E 使用独立候选环境，启动本次控制链：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup forward_rejoin_050_030.launch.py start_chassis_adapter:=true
```

3. F 做真实现场门检：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
cd /home/agilex/competition_forward_rejoin_20261103_ws
python3 scripts/check_import_origin.py
timeout 35s python3 scripts/navigation_preflight.py --timeout 20
ros2 topic info /cmd_vel -v
```

门检需输出 PRECHECK PASSED；最终 /cmd_vel 只有一个候选适配器发布者。默认仍应 ROUTE_DISABLED、车辆静止；没有同时运行旧 E 或 V1/V3。

4. G 开始第一圈空场整圈录制：

```bash
source /home/agilex/competition_forward_rejoin_20261103_ws/scripts/source_candidate_runtime.sh
bash /home/agilex/competition_forward_rejoin_20261103_ws/scripts/record_navigation_trial.sh obstacle_forward_rejoin_empty_full_loop
```

5. 确认现场门检及录制后，F 人工执行：

```bash
python3 /home/agilex/competition_forward_rejoin_20261103_ws/scripts/route_command.py start
```

本轮直线 0.50、弯道区 0.30，规划半径 0.60；先空场整圈，再进行三个固定障碍物整圈，不安排 30 m 测试。首次静态验收不移动障碍。停后移开障碍的恢复另外录制和标记，见完整操作单。

正常停止：

```bash
python3 /home/agilex/competition_forward_rejoin_20261103_ws/scripts/route_command.py stop
```

确认零速后结束 G 录制，再停止 E。控制指令使用新目录里的工具，A–D 保持原传感器/定位环境。

## 证据

- [部署状态](../evidence/2026-11-04_forward_rejoin_deployment.json)。
- [车端真实 ROS 节点隔离接口检查](../evidence/2026-11-04_forward_rejoin_ros_interface.json)。
- 车端日志：`/home/agilex/competition_forward_rejoin_20261103_ws/log/interface_smoke_20261104/`。
- 完整操作、测试范围和回退说明：[候选操作单](23_FORWARD_REJOIN_CANDIDATE_2026-11-03.md)。
