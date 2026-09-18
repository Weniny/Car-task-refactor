# 参数调试指南

本指南只说明配置关系，不构成实车发车流程。当前实车已冻结，任何修改只能在硬件恢复和断底盘验证后使用。

## 参数流

```text
轨迹 optimizer YAML -> 轨迹最大速度、曲率和速度曲线
control YAML -> MPPI 最大速度、加减速、曲率变化率
safety YAML -> Safety 速度/跟踪误差限制、代价地图膨胀
launch 文件 -> 对 YAML 的运行时覆盖，优先级最高
```

运行时参数必须用 `ros2 param get <node> <name>` 核对；仅修改 YAML 并不代表正在运行的节点已使用该值。

## 速度

主要文件：

```text
config/control/control_params_rebuild_050mps.yaml
config/planning/continuous_trajectory_radius090_turnrate250_050mps.yaml
```

需一致核对：`max_speed_mps`、`max_acceleration_mps2`、`max_deceleration_mps2`、`max_jerk_mps3` 和 `max_curvature_rate_1pmps`。轨迹来源清单会绑定 optimizer 参数；改完必须重新生成轨迹，否则 MPPI 会因 provenance 不匹配拒绝加载。

## 膨胀与局部规划

代价地图膨胀位于：

```text
config/safety/safety_params_rebuild_050mps_inflation035.yaml
  proximity_stop.grid_inflation_radius_m
```

局部 Hybrid A* 的额外膨胀是 launch 参数 `inflation_radius_m`。它与代价地图膨胀不是同一个量；两者叠加会缩小可行空间。特别是 0.50 m/s path-aware launch 曾将其覆盖为 `0.75 m`，必须在运行时核对。

| 参数 | 含义 |
|---|---|
| `lookahead_distance_m` | 局部重接参考路线的前视长度。 |
| `obstacle_x_max_m` | 从车体前方纳入搜索的障碍范围。 |
| `planning_timeout_s` | 单次搜索预算。 |
| `frequency_hz` | 重规划循环频率。 |
| `min_turning_radius_m` | Hybrid A* 的运动学最小半径。 |
| `goal_heading_tolerance_deg` | 回接目标允许的航向误差。 |

## Safety 与停车策略

Safety 负责输入健康、跟踪误差和速度限制；局部规划器通过 `/planning/local_stop_request` 请求停车。`proximity_stop_node` 可提供固定前向停车框或 path-aware 近障策略。

不要仅凭参数名称判断策略已生效。运行时至少检查：

```bash
ros2 param get /rebuild_replan_proximity_stop stop_distance_m
ros2 param get /rebuild_replan_proximity_stop path_aware_stop_enabled
ros2 param get /rebuild_local_replanner inflation_radius_m
ros2 param get /rebuild_replan_mppi replanning_enabled
```

## 统一入口

`scripts/run_navigation.sh` 是编排器：检查预条件、启动整合 launch、等待定位、记录 rosbag、要求人工输入固定起点确认，再发布路线使能。它不是隔离器；当前版本会涉及底盘节点，因此在硬件问题关闭前禁止执行。
