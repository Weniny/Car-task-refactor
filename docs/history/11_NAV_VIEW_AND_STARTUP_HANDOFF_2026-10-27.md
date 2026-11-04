> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-10-27 收尾：导航显示与发车指令

本轮测试已结束。今晚优先修复终端 B 的路径显示和发车指令，弯道“自校准/停顿”按现场反馈记入明日待办，今晚不改跟踪、转向、速度和 Safety 参数，不再做真实运动测试。

## 1. 终端 B 直接显示蓝、绿、黄路径

问题原因是 `rebuild_final_map.launch.py` 默认加载只含地图的 final_map_view.rviz。导航路径本身此前已经发布，但该窗口没有订阅相关 Path 话题。

已改为默认加载 navigation_debug_view.rviz，并将图形窗口的左侧显示栏恢复可见、视角调整为整圈概览。保留并合并车端已保存的显示设置，修正关键话题/QoS，提供 2D Pose Estimate。

明日在图形桌面终端 B 执行：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_final_map.launch.py rviz:=true
```

| 显示 | 话题 | QoS |
|---|---|---|
| 蓝：全局参考 | /planning/replan_reference_path | Reliable / Transient Local |
| 绿：局部轨迹 | /planning/local_trajectory | Reliable / Volatile |
| 黄：实际执行轨迹 | /control/replan_executed_path | Reliable / Volatile |
| 定位激光 | /localization/scan | Best Effort / Volatile |
| 局部代价地图 | /avoidance/local_costmap | Reliable / Volatile |

蓝线在 E 加载参考轨迹后出现；绿线在局部规划正常输出后出现；黄色轨迹随实际定位移动增长，静止时可能只有一点或短线。地图服务器启动并不等于规划路径已经发布。

**H 只打开 RViz，不能单独替代 B 的地图服务器。** 若想使用 H，B 以 rviz=false 运行，再在图形桌面执行：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
rviz2 -d /home/agilex/competition_rebuild_ws/install/rebuild_bringup/share/rebuild_bringup/rviz/navigation_debug_view.rviz
```

默认 B 已显示导航视图时无需再开 H。已经运行的旧窗口不会自动重载文件，明日重新启动 B 后生效；只重开图形窗口时用 H，避免启动第二个 map_server。

## 3. 发车、停止与录包改为固定入口

原故障包含两个现象：裸 once 命令默认仅等待一个匹配订阅者、发布后只保留 0.1 秒，录包收到 true 不代表控制器确认收到；固定等待两个订阅者又会在录包退出、只剩控制器时一直等待。不能把 CLI 打印 publishing #1 当作实际使能成功。

新增 `scripts/route_command.py`：Reliable/Volatile 单次命令，检查控制器订阅，动态确认匹配，最终以新的控制器状态作为确认。不会固定要求两个订阅者，也不会持续重复 true。

明日 A–E、定位与监护准备好后，G 只执行：

```bash
bash /home/agilex/competition_rebuild_ws/scripts/record_navigation_trial.sh
```

它自动创建新目录、保存四份参数和 CAN 状态，配置命令录包 QoS 并在前台录制。按 Ctrl+C 正常写盘，退出录包不会发送路线命令。

现场确认可发车后，F 执行：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py start
```

start 先执行原导航预检，再只发送一次 true；必须收到新的 `route_enabled=true` 且 `status=TRACKING` 才报告 ACKNOWLEDGED。未确认或中断时撤销未确认的使能并报错，不自动无限重试。超时保持有界，原 Safety 条件不变。

正常停止：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py stop
```

stop 等待禁用状态、两级速度命令归零和 odom 静止反馈；现场仍须确认车体停稳。危险情况物理急停。

只查看状态：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py status
```

详细整圈流程已经同步更新到 [提速操作单](10_SPEED_TRIAL_050STRAIGHT_030TURN_2026-10-27.md)。

## 完成与验证范围

- 修复已同步车端，rebuild_bringup 构建通过，B 的默认 rviz_config 指向导航视图。
- 实际打开导航 RViz 做只读加载检查，确认三条 Path 和定位 LaserScan 的订阅话题、QoS 均正确，2D Pose Estimate 工具也已加载；随后关闭测试窗口。
- 发车工具的 5 项 ROS 隔离集成测试通过：无录包时控制器确认、无控制器时拒绝、缺失确认时撤销且不重复 true、停车等待新零速反馈、只读状态无命令。
- 测试使用独立的 `/route_command_test_*` 话题和模拟控制器，没有向真实 `/mission/route_enable` 发送 true，没有启动底盘。
- [构建与隔离测试输出](../evidence/2026-10-27_navigation_handoff_tests.txt)、[RViz 实际订阅检查](../evidence/2026-10-27_navigation_view_check.json)。
- 车端改动前配置备份：`/home/agilex/competition_rebuild_ws/log/navigation_handoff_fixes_20261027_215708`。车端现场保存的 RViz 配置已合并并保留备份。
- 新发车工具的真实整圈确认留到明日现场验证；今晚完成软件、图形加载和隔离测试，不将其写成实车发车验收通过。

## 明日保留事项

- 本轮待复盘目录：`/home/agilex/competition_rebuild_ws/log/retest_full_loop_050straight_030turn_20261027_212854`。
- 根据本轮原包定位弯道停顿、自校准的具体状态和位置，区分 Safety 恢复、局部规划停车、转向模式/零位和跟踪收敛；目前仅保留现场现象，不预判根因。
- 核对实际直线与弯道速度、恢复段持续时间和整圈有效行驶时间，排除发车前等待与到站后人工结束等待。
- 本次只调整显示与命令送达流程，没有为弯道问题删除保护或修改速度/误差阈值。
