# 已验证 0.20 m/s 基线

本页记录历史上已经完成实车验证的导航基线。受 2026-09-18 硬件事件影响，它不是重新上车的许可；复车前仍须满足 [当前状态与安全交接](01_CURRENT_STATUS_AND_SAFETY.md) 的所有门槛。

## 已验证结果

- Ranger CAN：`can2`、500 kbit/s。
- FAST-LIO 约 10 Hz、IMU 约 200 Hz；AMCL 和 `map -> body` 可用。
- 固定起点：`x=-16.083`、`y=2.309`，车头约 `-179.4 deg`；每次仍需以实时激光贴图为准重新发布初始位姿。
- R=1.2 m 转弯几何、`0.20 m/s`、曲率变化率 `2.50 1/(m*s)`。
- 30 m 第一处转弯连续通过；约 155 m、3109 点整圈完整跑通。
- 静态障碍局部重规划曾通过；规划器能输出 `REPLANNED` 和 `REPLANNED_SHORT_REJOIN`。

## 有效基线文件

```text
语义地图：maps/indoor_manual_final/semantic_map_six_anchor_v3_radius120.yaml
规划参数：config/planning/semantic_lidar_guarded_six_anchor_radius120.yaml
Safety：config/safety/safety_params_rebuild_020mps_turn_recovery.yaml
30 m：artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_020mps_trajectory.yaml
整圈：artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml
启动：rebuild_navigation_replan_radius120_competition_safety_guarded.launch.py
```

## 已解决的转弯问题

早期路线最小转弯半径约 `0.62 m`，会引起回正、低速等待和持续航向误差。修正包括：

- 将几何半径增至约 `1.2 m`。
- 离线轨迹与运行 MPPI 均使用 `max_curvature_rate_1pmps=2.50`。
- 使用比赛验证过的 Safety 恢复带：进入 `0.20 m / 20 deg`，退出 `0.10 m / 8 deg`，硬限制 `0.40 m / 45 deg`，恢复速度 `0.15 m/s`。

## 已知未解决项

- FAST-LIO 长距离建图漂移未根治；当前静态地图由 PCD 转 PGM 后人工清理墙线得到。
- 地图清理结果必须由实时激光和 AMCL 复核，不能将人工填白区域视为已证明可通行。
- Fast-LIO/Livox 的正常关停稳定性仍需复核。
