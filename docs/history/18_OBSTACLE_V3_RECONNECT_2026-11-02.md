> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-02：避障 V3 平滑接入修复

本轮完成代码、车端构建、回归模型和独立 ROS 启动验证，未启动真实路线。V3 尚未实车验收，不将 V1 失败记录改写为成功。

## 本轮修复

原路径沿用把当前位置直接连到最近的 10 cm 离散点。跟踪偏差约 9 cm 时，这个短接入段可能违反 0.60 m 转弯半径；旧路径即使剩余部分安全，也会被放弃，继而反复进行完整搜索。

V3 优先保留原先安全的直接接入；不满足时，在同一条绕行路径前方约 0.6、0.9、1.2 m 处尝试平滑曲线连接。连接段和后续路径都必须通过完整地图碰撞检查、接入航向检查和转弯半径检查。发现新障碍则拒绝沿用，不绕过停车保护，也不倒车脱困。

同时开启已有的障碍感知搜索启发式，减少搜索向被遮挡区域扩展。试验过较大的净距代价，模型中导致超时，最终未采用；没有缩小膨胀层或修改 Safety 阈值，也没有应用固定角度补偿。

保留 V2 的完整占用格、带时间戳 TF、占用格面积覆盖、输出逐段碰撞检查和无路清除旧绿线。新功能默认关闭，只在独立 V3 启动入口开启；旧入口保持原策略。

## 模型回归及边界

固定地图来自 V1 原包的障碍物前快照，接入实际规划器、MPPI 和 Safety，以 2 Hz 规划、20 Hz 控制和临障速度上限 0.20 m/s 运行。它是软件简化模型，不发布 ROS 指令，不代表实车或动态障碍测试。

通过指标为穿过约 6 m 局部路段的出口截面，距原参考横向不超过 0.35 m，并在全程检查障碍碰撞。它不是末点停车验收；为避免截断参考轨迹制造虚假的终点减速，使用原路线后续 40 点延伸控制参考。模型假设理想速度执行，不含实际轮胎/驱动器延迟、点云变化和完整定位误差。方向偏差只是扰动输入，不是已标定的硬件参数。

| 固定地图模型 | 结果 | 起步后的停车 | 规划失败 |
|---|---|---:|---:|
| 车端对照：完整地图+启发式，禁用平滑接入，-8° 扰动、种子 17 | 停死，约 12.45 s 判定 | 10.0 s | 20 |
| V3，无方向扰动 | 通过，约 30.25 s | 0 | 0 |
| V3，-6° 扰动、0.25 s 路径交付延迟、种子 17 | 通过，约 32.80 s | 0 | 0 |
| V3，小车电脑运行模型，-8° 扰动、0.25 s 延迟、种子 17 | 通过，约 32.80 s | 0 | 0 |

含延迟的模型初始建立控制约有 0.75 秒低速/静止，单独计为起步过程。车端对照与 V3 在同一小车电脑上分开运行；V3 模型规划 P95 约 90.7 ms、最大约 249.6 ms，对照含失败回退的规划 P95 约 532 ms。这不是完整 ROS 系统实时性能验收。

原包低转向窗口中 body 朝向与位移方向仍有数度差异，但不能区分安装朝向和实际底盘偏行，本轮未据此修改 TF 或控制坐标系。

本地相关测试 47 项、13 子测试通过；车端规划 21 项 unittest 和构建通过。独立 ROS_DOMAIN_ID=93 启动时适配器关闭，检查新参数、安全零输出和最终控制话题无发布者。备份：`/home/agilex/competition_rebuild_ws/log/obstacle_v3_backup_20261102_213144`。

## 下一轮运行指令

先按 [完整操作单](10_SPEED_TRIAL_050STRAIGHT_030TURN_2026-10-27.md) 准备 CAN 和 A–D，车辆返回真实起点，初始化定位并确认激光贴图。不重复启动已有 A–D。退出旧 E 后使用下面的独立入口，其他版本 E 不并用。首次实车验证建议一个提前摆好的静态障碍物，可直接整圈，不增加 30 m 前置测试。

E：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup \
  rebuild_navigation_replan_radius120_050_030_obstacle_v3.launch.py \
  start_chassis_adapter:=true
```

F，确认预检通过，三个参数均 true：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/navigation_preflight.py --timeout 20
ros2 param get /rebuild_local_replanner smooth_detour_reconnect
ros2 param get /rebuild_local_replanner obstacle_aware_heuristic
ros2 param get /rebuild_local_replanner complete_costmap_validation
```

G：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
bash /home/agilex/competition_rebuild_ws/scripts/record_navigation_trial.sh obstacle_v3_050_030
```

现场监护与接管准备完成后，F 手动请求发车并等待 ACKNOWLEDGED：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py start
```

观察绿线与障碍层、连续通过与回接情况；若仍反复停顿或出现重叠，先 stop 并确认停稳，再接近/移动障碍物。正常结束同样 stop 后再到 G Ctrl+C 保存，保留失败原包继续判断，不反复发送 true：

```bash
python3 /home/agilex/competition_rebuild_ws/scripts/route_command.py stop
```

## 复现与证据

- [固定地图输入](../evidence/2026-11-02_avoidance_frozen_fixture.json)
- [模型脚本](../../scripts/offline_avoidance_closed_loop.py)
- [车端禁用接入的对照](../evidence/2026-11-02_v3_car_baseline.json)
- [无扰动模型](../evidence/2026-11-02_v3_model_ideal.json)
- [6° 与延迟模型](../evidence/2026-11-02_v3_model_bias6_delay.json)
- [车端模型](../evidence/2026-11-02_v3_car_model.json)
- [构建与车端回归](../evidence/2026-11-02_v3_build_and_car_model.txt)
- [隔离 ROS 检查](../evidence/2026-11-02_obstacle_v3_static_check.json)
