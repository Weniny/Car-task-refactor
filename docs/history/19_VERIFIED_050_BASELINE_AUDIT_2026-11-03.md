> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-03：已验证 0.50/0.30 基线七项审计

## 范围与证据边界

仅核查原入口 `rebuild_navigation_replan_radius120_050straight_030turn_guarded.launch.py`、它加载的 control/safety/planning 文件及对应实现。证据为 2026-11-02 19:45 单次整圈记录，以及 V1 之前 20:03 的第一次避障原包摘要/快照。不用 V1/V3 的参数、模型通过或现场失败来证明原基线的结论。

当前没有 SSH 连接，旧地址尝试超时。以下不是今天车端实时参数审计；今日完整源码/安装包与历史基线的一致性仍待连接后确认。本次只新增审计文档和本地清单，未改生产代码、参数、启动文件或部署。

## 实际架构

固定 YAML 全局参考 → competition_planning 自定义局部 Hybrid A* → competition_control 自定义 MPPI 跟踪 → competition_safety → Ranger Twist 适配器 → 底盘。

基线 E 未启动 nav2_planner/planner_server、nav2_controller/controller_server 或 Nav2 global/local Costmap2DROS；Nav2 AMCL 用于定位，不能由此推断规划控制也属于 Nav2 官方插件。全局参考为预先生成的 3109 点轨迹，运行时并非由 Smac Hybrid 全局重规划。

## 1. 配置一致性

| 环节 | 基线值/语义 |
|---|---|
| 全局参考 | 固定轨迹约 155 m，不随新障碍重算蓝线 |
| 语义规划配置 | footprint_radius_m=0.40、clearance_m=0.05、corridor_margin_m=0.45；这是生成/语义配置，不是 Nav2 global_costmap 当前运行值 |
| 实时局部图 | /avoidance/local_costmap；分辨率 0.05 m，grid_inflation_radius_m=0.44 |
| 膨胀成本 | 原始点为 100，膨胀圈均为 50，未知为 -1；没有指数梯度、cost_scaling_factor |
| 局部 Hybrid A* | occupancy_threshold=50，把膨胀圈当成硬障碍；另加 inflation_radius_m=0.04 |
| 普通行驶碰撞模型 | 点机器人 + 障碍膨胀；普通模式 footprint=None，0.72×0.50 m 尺寸并非直接传给普通搜索的矩形足迹 |
| MPPI | 跟踪局部路径，不订阅该代价地图，不评分障碍格 |

已确认的矛盾：

- 新障碍可以盖住固定蓝线，而蓝线本身保持不变；全局参考可见不代表它仍被局部碰撞模型允许通过。
- 静态地图在普通 Hybrid A* 中只按额外 0.04 m 膨胀，实时墙点已先膨胀 0.44 m 再经额外处理。同一墙体的几何包络可能不一致。
- 名义全局半径/净距约 0.45 m，与局部实时障碍的约 0.49 m 排除距离不完全一致；固定轨迹的历史生成过程还需核验，不能将配置文本当作完整实时碰撞保证。
- MPPI 不知道障碍成本，只能追踪规划器交来的线；路径可行性与跟踪后的实际运动可行性没有 Nav2 Obstacle Critic 那样的直接耦合。

0.04 m 在 0.05 m 栅格上的 inflated_blocked 会向上取整为一格，因此局部实时层的有效排除距离名义约 0.44+0.05=0.49 m，实际还受栅格化、抽样、坐标变换影响，不是精确物理净距。

## 2. 膨胀覆盖与建议范围

配置尺寸 0.72×0.50 m，对应内切圆半径 0.25 m，外接圆半径 sqrt(0.36²+0.25²)=0.4383 m。当前点机器人模型需要用能覆盖角点的外接半径思考，不能用半车宽 0.25 m 代替所有姿态下的机器人半径。

结论：不能由现有基线证据认定 0.44 m 本身过大。它接近车体外接半径；局部额外一格使其约为 0.49 m。圆形包络在狭窄空间可能比实际矩形更保守，但这需要实际障碍尺寸、通道净宽和同时间戳路径/地图验证。更直接的问题是整个膨胀圈被当作硬禁区，以及各环节的包络定义不统一。

按用户提出的“机器人半径+0.1~0.2 m”规则，以外接圆为基准约为 0.54~0.64 m。这是点模型下保守总排除距离的设计参考，不是本轮建议直接写入的值；它比现有值更大，不会帮助解除已被封住的通路。若用矩形足迹，应在足迹外另定义 0.1~0.2 m 净距，不能再简单按外接半径叠加一次车体。

当前不建议直接降至 0.30/0.35 m；在没有矩形碰撞检测补偿的点模型中会漏掉部分车体角点。也不建议仅凭经验公式增大。先量化原基线蓝线/绿线在完整实时硬障碍中的长度占比、连续重叠距离、首次重叠到停车的时间。

本地目前缺该基线原始 bag 的完整逐帧副本，因此不报告覆盖百分比。V1 的 10/162、776/785 等数字不用于本审计。

## 3. MPPI 障碍代价耦合

基线没有 ObstaclesCritic、repulsion_weight 或 Critic 插件列表。MPPI rollout 目标函数只有位置、航向、速度、曲率、控制平滑和终端误差，没有 costmap/障碍距离项。

明确建议：不调整 repulsion_weight，不新增一个同名无效参数。不能用“MPPI 看见膨胀成本升高就强烈排斥”解释这个基线，因为它没有读取该成本。

Nav2 官方 MPPI 确实要求结合膨胀范围调节障碍排斥权重，大范围时常需降低相对影响；这不是所有实现通用的固定比例法则，更不适用于当前缺少该 Critic 的代码。参考 [Nav2 Humble MPPI 文档](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md)、[Humble ObstaclesCritic 实现](https://api.nav2.org/nav2-humble/html/obstacles__critic_8cpp_source.html)。

## 4. Hybrid A* 的 cost_scaling_factor

当前没有 cost_scaling_factor，膨胀圈也没有指数衰减，因此不存在把 4~5 调到 3.0/10.0 的实际操作对象。

用户提到的敏感现象有官方仓库报告依据：[Nav2 #4314](https://github.com/ros-navigation/navigation2/issues/4314)，但报告针对 2024 年特定 Rolling 提交、非圆形足迹和 Smac 搜索启发式，不能当作全部 Hybrid A* 的普遍规律。当前基线也未使用该插件。结论：本轮不调整该参数。

## 5. 路径跟随权重

当前没有 PathAlignCritic/PathFollowCritic。实际 MPPI 默认目标权重为 position=45、heading=12、speed=4、curvature=0.8、control_smoothness=2、terminal=80。不同项量纲不同，不能凭数字大小判断谁过低。

结论：没有证据支持微调 PathFollowCritic；不能用加大一个不存在的 Critic 修复问题。实际位置/航向权重也暂不修改，先检查规划路径的可执行性、控制器接受的轨迹、实际运动与碰撞包络是否一致。强行增大跟踪权重不能使已进入硬禁区的参考线变安全。

## 6. 执行死区

当前基线配置、MPPI 和 Twist 适配器没有 min_speed_theta/min_speed_xy，也没有已标定的底盘最小可执行速度参数。Ranger 使用速度/曲率语义，适配器按几何换算 angular.z，而非通过设置最小角速度确保转向。

硬件死区仍可能存在，现有本地证据不足以确认或排除。连接后优先从原包对齐 /control/body_cmd → /cmd_vel_safe → /cmd_vel → /odom 和 /motion_state，判断非零指令是否持续无响应；不要把规划器已经主动发零的区段算作底盘死区。也不能通过强抬最低速度覆盖应当停车的指令；最小角速度还会改变曲率和底盘模式选择。

## 7. 停车原因

**空场成功基线 19:45：** 进度到 3108，局部规划停车/近障停车采样均为零；约 70.2 秒恢复限速与跟踪误差相关。它不能证明有障碍时也会通过。

**相同原入口、V1 之前的 20:03 避障记录：** 三个障碍物，人工移动过障碍物，进度约 30→869。三段长静止约 23.5、20.4、29.4 秒；local_stop 为 1787/4850 个控制采样，avoidance_stop=0。主要为 HYBRID_ASTAR_NO_FEASIBLE_PATH → LOCAL_PLANNER_STOP → controller_not_tracking → 零速，不能解释为 ObstaclesCritic 得分过高或已证明的执行死区。

原基线快照中，约 185.4、213.4 秒起点本身可通行，但三种初始前进动作全部被当前碰撞检查拒绝。只说明“当前障碍表示与姿态/运动约束下无前进解”，不证明物理空间完全没有路。

当前没有 critics_stats 或 ObstacleCritic 分项日志，不能虚构统计。主要证据支持规划侧拒绝路径；同时存在规划/跟踪之间缺少障碍耦合的结构风险，需要分开验证。

## 后续工作边界

先连接小车，复制 19:45 整圈和 20:03 首次避障的参数快照、原包与当时源码/构建备份，做完整基线冻结。仅切回旧 launch 不等于恢复历史软件，因为多个版本修改过共享 Python 包。本地原入口及配置哈希见 [审计清单](../evidence/2026-11-03_baseline_audit_manifest.json)，该哈希不冒充历史全软件冻结。

下一次修改前先完成完整地图覆盖量化与指令—反馈对齐。若确认几何排除区过度保守，单独测试一个包络参数，并保持其他参数固定；若是路径/运动学可行性冲突，先修接口，不能用 Critic 权重掩盖。V1/V3 不作为后续修改基线。

本次未回滚或更改小车文件，未宣布恢复历史基线完成。

原始证据：[空场原包诊断](../evidence/2026-11-02_postfix_full_loop_diagnosis.txt)、[V1 之前避障诊断](../evidence/2026-11-02_obstacle_bag_diagnosis.txt)、[原基线快照](../evidence/2026-11-02_obstacle_snapshot_replay.json)。源码核对入口为原 050straight_030turn launch、proximity_stop._build_local_costmap、MPPIController._rollout_costs 和 Ranger Twist 适配器；本次已确认后两项主要纯函数较仓库 HEAD 未被后续候选修改。
