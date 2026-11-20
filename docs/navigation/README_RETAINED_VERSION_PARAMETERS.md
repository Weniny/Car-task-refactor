# 所有保留版本参数总表

更新：2026-11-20。[返回导航总览](README_NAVIGATION_OVERVIEW.md) · [第一阶段成果](stage1/README_STAGE1_RESULT.md) · [第二阶段目标](stage2/README_STAGE2_LOCALIZATION.md)

**当前第一阶段成果的 Safety 恢复速度为 0.35 m/s。** 下表中的配置限值、生成轨迹的参考速度和实车反馈分别记录，不能互相替代。

本表核对本地保留归档、YAML、实际启动入口覆盖及已有 ROS 参数快照，未连接车端读取新的在线状态，也未修改参数。本文使用的控制、安全、规划、路线和 A–D 配置原值，以及展开后的启动参数，保存在 [retained_version_parameters.json](reference/retained_version_parameters.json)。该数据区分 YAML 模板、显式启动覆盖和已有实录快照，未将代码内所有隐式默认值宣称为在线读数。

## 1. 保留版本与用途

| 版本 | 保留状态 | D 定位配置 |
| --- | --- | --- |
| 原025：0.50/0.30 | 原始冻结来源，供恢复与回归 | 原 `likelihood_field` |
| 直线提速：1.00/0.30 | 旧对照，完成过空场整圈 | 原 AMCL `likelihood_field`，配置文件同原025 |
| 弯速对照：1.00/0.35 | 旧对照，保留实测和归档 | 原 AMCL `likelihood_field`，配置文件同原025 |
| **第一阶段成果：1.00/0.55 平顺接缝** | **当前保留成果，第二阶段基线** | **原 `likelihood_field`** |
| 第二阶段 AMCL 概率模型试验 | 配置、工具和失败记录保留，实车未通过 | `likelihood_field_prob` |

上表“配置文件同原025”具体指 D 使用 `/home/agilex/competition_rebuild_ws/config/localization/amcl_rebuild_indoor.yaml`，模型为 `likelihood_field`。这两个旧对照的 A–C 配置也沿用原025；E–G 则使用第7节列出的各自导航目录，速度和制动参数按第2节区分。

第二阶段失败试验沿用第一阶段导航，主要配置变化在 D 的定位模型。它不能因 E 目录相同就被当成第一阶段原组合。更早的前向回接源码目录及已废弃的 060/080 方案属于历史材料，不作为本表的当前保留运行版。

## 2. 各版本变化的关键参数

| 参数（单位） | 原025 | 1.00/0.30 | 1.00/0.35 | **第一阶段成果** | 第二阶段失败试验 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 直线目标：MPPI / Safety / 路线默认（m/s） | 0.50 | 1.00 | 1.00 | 1.00 | 1.00 |
| 路线四弯限速（m/s） | 0.30 | 0.30 | 0.35 | 0.55 | 0.55 |
| 在线局部高曲率限速（m/s） | 0.30 | 0.30 | 0.35 | 0.55 | 0.55 |
| **Safety 恢复速度** `recovery_speed_mps`（m/s） | 0.15 | 0.15 | 0.15 | **0.35** | **0.35** |
| 正向加速度：MPPI / Safety / 规划（m/s²） | 0.12 | 0.12 | 0.12 | 0.12 | 0.12 |
| 常规减速度幅值：MPPI / Safety / 规划（m/s²） | 0.18 | 0.80 | 0.80 | 0.80 | 0.80 |
| jerk：MPPI / 离线全局轨迹（m/s³） | 0.20 | 2.00 | 2.00 | 2.00 | 2.00 |
| 侧向加速度：离线 / 在线规划（m/s²） | 0.12 | 0.12 | 0.12 | 0.28 | 0.28 |
| 曲率变化率：MPPI / 离线全局（m⁻¹/s） | 2.50 | 2.50 | 2.50 | 2.50 | 2.50 |
| 控制与局部规划最小转弯半径（m） | 0.60 | 0.60 | 0.60 | 0.60 | 0.60 |
| 局部前视长度（m） | 3.00 | 3.00 | 3.00 | 3.00 | 3.00 |
| 新增五次接缝过渡半长（m） | 未启用 | 未启用 | 未启用 | 0.60，共8处 | 沿用第一阶段 |
| D 的 AMCL 模型 | likelihood_field | likelihood_field | likelihood_field | likelihood_field | likelihood_field_prob |

表中的纵向加减速和 jerk 是软件限幅及轨迹/预算配置，不是本轮实测制动能力。普通转弯还需联合核对 `ω=vκ`、`dω/dt=κ·dv/dt+v·dκ/dt` 和 `a_lat=v²κ`。曲率变化率 2.50 m⁻¹/s 不能直接当成 2.50 rad/s² 的角加速度。

**1.00/0.35 旧弯速对照的“0.35”指弯道限速，其 Safety 恢复速度仍为 0.15 m/s。** 第一阶段成果才同时采用弯道参考 0.55 和恢复限速 0.35 m/s。

## 3. 参考轨迹与实车反馈分开看

下列参考值来自归档的 3109 点全局轨迹。各弯以对应 entry–exit 索引统计，并检查前后各约 0.60 m 接缝区域。实际控制还会使用在线局部重定时和 Safety 输出。

| 版本 | 四弯内参考速度范围 m/s | 参考全程时长 s | 代表性实测四弯最低速度 m/s |
| --- | --- | ---: | --- |
| 原025 | 0.1432–0.2658 | 359.773 | 未纳入同口径对照，不填参考值代替实测 |
| 1.00/0.30 | 0.1432–0.3000 | 201.393 | 0.094 / 0.102 / 0.096 / 0.098 |
| 1.00/0.35 | 0.1432–0.3500 | 198.567 | 0.096 / 0.088 / 0.090 / 0.090 |
| **第一阶段成果** | **0.5500–0.5500** | 188.842 | **0.294 / 0.292 / 0.296 / 0.296** |
| 第二阶段失败试验 | 全局参考沿用第一阶段 | 沿用第一阶段 | 该轮未完成整圈，不沿用第一阶段的四弯实测值 |

1.00/0.30 和 1.00/0.35 的实测来自 11 月 13 日两轮对照，见 [整圈复盘](../../.codex-build/localization-20261119/docs/history/2026-11-13_1MPS_FULL_LOOP_REVIEW.md)。第一阶段成果实测来自 11 月 19 日 17:45，见 [逐弯数据](../../.codex-build/localization-20261119/docs/evidence/2026-11-19_smooth_corner_trial.json)。这些是代表性记录，并非该版本在所有运行中的固定最低速度。

第一阶段在线局部重定时的弯内参考最低约 0.540–0.548 m/s，亦与全局 0.55 参考、Safety 0.35 限速及反馈约 0.29–0.30 分属不同数据层。参考时长也不是实车整圈耗时。

## 4. 各版本共同保留的控制与规划设置

以下用于表中五个配置组合，第二阶段失败试验的 E 沿用第一阶段。

### MPPI 和车辆运动

| 参数 | 值 | 含义或来源 |
| --- | --- | --- |
| `frequency_hz` | 20 Hz | 控制 / Safety 启动配置 |
| `horizon_steps` | 30 | MPPI 预测步数 |
| `rollout_count` / `iterations` | 384 / 2 | MPPI 采样数 / 迭代次数 |
| `temperature` | 0.35 | 优化参数，无速度单位 |
| `speed_noise_std_mps` | 0.01 m/s | 速度采样噪声 |
| `curvature_noise_std_1pm` | 0.08 m⁻¹ | 曲率采样噪声 |
| `lateral_feedback_gain_1pm_per_m` | 1.5 | 横向反馈增益 |
| `heading_feedback_gain_1pm_per_rad` | 1.0 | 航向反馈增益 |
| `feedback_blend` | 0.35 | 反馈混合比，独立于 Safety 的 0.35 m/s |
| `progress_search_window_points` / `max_progress_advance_points` | 40 / 3 | 进度搜索与单次推进 |
| 局部路径近似复用位置 / 航向容差 | 0.05 m / 5° | MPPI 启动参数 |
| 正常运动模式 | `dual_ackermann` | 四轮转向模型 |
| 轴距 / 轮距 | 0.494 / 0.364 m | 控制与底盘适配几何 |
| 驱动器最小转弯半径 | 0.47644 m | 驱动器参数，区别于控制 / 规划的 0.60 m |
| 内轮转角配置上限 | 0.698 rad | 车辆参数模板 |
| 协议速度 / 协议转向速度配置 | 2.00 / 0.70 m/s | 协议能力字段，普通导航目标按第2节 |
| `spin_recovery_enabled` | false | 普通跟踪不启用旋转恢复 |

### 局部规划与路径许可

| 参数 | 值 |
| --- | --- |
| 局部规划频率 | 2 Hz |
| `lookahead_distance_m` | 3.00 m |
| `planning_timeout_s` / `empty_history_planning_timeout_s` | 0.40 / 0.40 s |
| `relaxed_extension_timeout_s` | 0.35 s |
| `guard_command_delay_s` | 0.15 s，命令延迟配置项 |
| `guard_max_speed_mps` / `guard_deceleration_mps2` | 随第2节本版速度与减速度变化 |
| `tracking_clearance_m` | 0.10 m |
| 规划附加 `inflation_radius_m` | 0.04 m |
| 路径采样 / 搜索步长 | 0.10 / 0.20 m |
| 曲率离散数 / 航向离散数 | 9 / 72 |
| `max_expansions` | 250000 |
| `search_heuristic_weight` / `reference_deviation_weight` | 1.6 / 2.0 |
| 目标位置 / 航向容差 | 0.15 m / 20° |
| `reference_search_window_points` | 160 |
| `max_obstacle_age_s` / `max_odom_age_s` | 0.90 / 0.50 s |
| `costmap_occupancy_threshold` | 50 |
| `forward_rejoin_enabled` / `behavior_guard_enabled` | true / true |
| `require_local_plan_guard` | true |
| `local_trajectory_timeout_s` | 2.00 s |

0.15 s 只是延迟配置的一项，不等于从观测到停稳的总延迟。3 m 是名义前视，也不是已经实测通过的停车距离。原离线全局规划模板另有 `min_turning_radius_m=1.10`、`footprint_radius_m=0.40`，不能与本节运行中的局部半径 0.60 m 混成一个参数。

与18:35运行快照对照时，启动字典中的局部规划 `max_curvature_rate_1pmps` 未列在该节点的参数列表中。本表的 2.50 m⁻¹/s 明确指 MPPI 输出与离线全局轨迹设置，不据这个字典项推断局部节点另有同样的独立限幅。

## 5. Safety、定位状态和近障检查

### Safety 跟踪保护

| 参数 | 所有保留组合的共同值 |
| --- | --- |
| `recovery_lateral_error_m` / `recovery_heading_error_deg` | 0.20 m / 20° |
| `recovery_clear_lateral_error_m` / `recovery_clear_heading_error_deg` | 0.10 m / 8° |
| `max_lateral_error_m` / `max_heading_error_deg` | 0.40 m / 45° |
| `tracking_error_timeout_s` | 1.00 s，持续误差判定时间 |
| `command_timeout_s` / `state_timeout_s` / `system_state_timeout_s` | 0.15 / 0.15 / 0.20 s |
| `avoidance_timeout_s` | 2.00 s |
| `require_avoidance_source` / `require_traffic_rules` | true / false |
| 纯旋转 / 平行移动配置上限 | 0.15 rad/s / 0.06 m/s，属于对应运动模式 |

进入恢复档的速度按第2节版本列分别为 0.15 或 0.35 m/s。恢复触发阈值与退出阈值有滞回，持续误差停车也与恢复限速分开处理。

### 定位状态连续性设置

| 参数 | 共同值 |
| --- | --- |
| `map_frame` / `base_frame` | `map` / `body` |
| `pose_timeout_s` / `velocity_timeout_s` | 0.20 / 0.50 s |
| `max_pose_prediction_s` | 3.00 s |
| `max_position_jump_m` / `max_heading_jump_deg` | 0.25 m / 20° |
| `initial_pose_settle_s` | 0.50 s |

0.25 m 是绝对跳变上限，状态检查还结合时间间隔与速率连续性校核，不能解释为每个控制周期都允许 0.25 m 的变化。

### 导航近障扫描与代价地图

| 参数 | 启动后的配置 |
| --- | --- |
| `stop_distance_m` | **0.40 m**，body 坐标近障检查配置 |
| `grid_inflation_radius_m` | **0.25 m** |
| `grid_resolution_m` | 0.05 m |
| 网格范围 x / y | [−0.5, 5.5] / [−2.5, 2.5] m |
| `min_points` / `fusion_frame_count` | 3 / 1 |
| `x_min_m` / `lateral_half_width_m` | 0.25 / 0.45 m |
| `front_half_angle_rad` | 0.4363 rad，约 ±25° |
| 名义车身长 / 宽 | 0.72 / 0.50 m |
| `max_scan_age_s` | 2.00 s |
| E 的扫描范围 / 角分辨率 | 前方180° / 0.5° |
| E 的距离范围 / 高度裁剪 | 0.10–6.00 m / [−0.25, 0.80] m |
| `path_aware_stop_enabled` | false，11月19日运行快照已核对，属于共同导航实现的默认配置 |

固定近障检查包含扇区与矩形走廊。3 点门槛针对该独立停车条件，不等于至少 3 点才能标记栅格。18:35 试验右侧近点为 2、2、1 时没有触发近障停车，相关失败记录仍保留。

**YAML 模板和启动覆盖的区别：** 归档 YAML 中 `stop_distance_m=0.0`、`grid_inflation_radius_m=0.44`，实际保留入口统一覆盖为 **0.40 / 0.25 m**。YAML 中 `trajectory_tracker.max_lateral_error_m=0.15` 也不能当作本节 Safety 的 0.20 m 恢复或 0.40 m 持续误差阈值。

## 6. A–D 共用平台参数与第二阶段差异

原025归档与11月19日原平台快照中的 AMCL、定位扫描、FAST-LIO 配置文件散列一致。除第二阶段试验改动 D 的模型外，本表按该共用平台版本列出。

| 模块 / 参数 | 原共用平台 | 第二阶段概率模型试验 |
| --- | --- | --- |
| A 启动 `force_livox_host_timestamps` | true，来自保留平台启动单 | 相同 |
| A 启动 `publish_odom_tf` / `start_slam` / `start_anchor` | false / false / false | 相同 |
| D：`laser_model_type` | **likelihood_field** | **likelihood_field_prob** |
| `do_beamskip` | true，原所选模型不使用该功能 | true，功能还受收敛与失配条件控制 |
| `beam_skip_distance` / `beam_skip_threshold` / `beam_skip_error_threshold` | 0.50 m / 0.30 / 0.90 | 相同 |
| `base_frame_id` / `odom_frame_id` / `global_frame_id` | body / camera_init / map | 相同 |
| `transform_tolerance` | 0.20 s | 相同 |
| `robot_model_type` | nav2_amcl::DifferentialMotionModel | 相同 |
| `alpha1–4` / `alpha5` | 各0.05 / 0.02 | 相同 |
| `min_particles` / `max_particles` | 500 / 2000 | 相同 |
| `max_beams` / `sigma_hit` | 180 / 0.20 m | 相同 |
| `z_hit` / `z_rand` | 0.75 / 0.20 | 相同 |
| AMCL距离范围 / `laser_likelihood_max_dist` | 0.20–20.00 m / 2.00 m | 相同 |
| `update_min_d` / `update_min_a` | 0.05 m / 0.05 rad | 相同 |
| `resample_interval` | 1 | 相同 |
| C：定位扫描范围 / 角分辨率 | 360° / 0.5° | 相同 |
| C：距离范围 / 高度裁剪 | 0.20–20.00 m / [−0.25, 0.80] m | 相同 |
| A：FAST-LIO `scan_rate` / `point_filter_num` | 10 Hz / 3 | 相同 |
| `blind` | 0.50 m | 相同 |
| `filter_size_surf` / `filter_size_map` | 0.50 / 0.50 m | 相同 |
| `time_sync_en` / `time_offset_lidar_to_imu` | false / 0.0 s | 相同 |
| `extrinsic_est_en` | false | 相同 |
| `extrinsic_T` / `extrinsic_R` | [−0.011,−0.02329,0.04412] m / 单位矩阵 | 相同 |
| `acc_cov` / `gyr_cov` | 0.10 / 0.10 | 相同 |
| `b_acc_cov` / `b_gyr_cov` | 0.0001 / 0.0001 | 相同 |

两个定位模型的版本组合已经分开，未将概率模型作为第二阶段已接受结果。AMCL 的激光似然组合方式也随模型选择改变，不能只将其理解为无其它影响的单一过滤开关。

## 7. 对应目录、文件和核对来源

以下目录名均以车端 `/home/agilex/` 为前缀。导航入口都沿用 `startup_capture_050_030_inflation025.launch.py`，文件名中的旧数字不代表当前加载速度。

| 版本 | E–G 导航目录 | D 配置 |
| --- | --- | --- |
| 原025 | `competition_startup_capture_20261104_ws` | 共用平台原 AMCL |
| 1.00/0.30 | `competition_stepwise_100_20261113_ws` | 共用平台原 AMCL |
| 1.00/0.35 | `competition_stepwise_100_turn035_20261113_ws` | 共用平台原 AMCL |
| 第一阶段成果 | `competition_stepwise_100_turn055_smooth_20261119_ws` | 共用平台原 AMCL |
| 第二阶段失败试验 | 沿用第一阶段导航目录 | `competition_localization_beamskip_20261119/amcl_beamskip.yaml` |

共用平台原 AMCL 为 `competition_rebuild_ws/config/localization/amcl_rebuild_indoor.yaml`。当前日常启动单为 [A–D 共用平台](common/README_A_D_PLATFORM.md) 和 [第一阶段 E–G](stage1/README_E_G_STAGE1.md)。本表是版本对照，不表示车端当前已经切换到某列。

| 数值来源 | 归档内相对路径 |
| --- | --- |
| 控制、运动和状态配置 | `config/control/control_params_rebuild_050straight_030turn.yaml` |
| Safety和近障模板 | `config/safety/safety_params_rebuild_050straight_030turn.yaml` |
| 离线 / 在线速度生成 | `config/planning/continuous_trajectory_radius120_050straight_030turn.yaml` |
| 全局规划模板 | `config/planning/semantic_lidar_guarded_six_anchor_radius120.yaml` |
| 路线速度区 | `routes/indoor_manual_six_anchor_v3_radius120_050straight_030turn_loop.yaml` |
| 启动覆盖 | `src/forward_rejoin_bringup/launch/startup_capture_050_030_inflation025.launch.py` |
| 完整配置、来源散列、参考速度及已录在线快照 | [retained_version_parameters.json](reference/retained_version_parameters.json) |

### 保留归档

- [原025：导航与平台依赖归档](../../baselines/baseline-20261104-050-030-i025/runtime/car_dependencies.tar.gz)，SHA256 `fb84c910f62003f5f2279c93cb5b6cf90884d325630d0726d2d69855b6efb92b`。
- [1.00/0.30：导航归档](../../.codex-build/stepwise-025/.codex-build/competition_stepwise_100_20261113_ws.tar.gz)，SHA256 `476d05352afea3874563056ee3f372267e8dc7c6380f4627617450bdcd66ffb0`。
- [1.00/0.35：导航归档](../../.codex-build/stepwise-025/.codex-build/competition_stepwise_100_turn035_20261113_ws.tar.gz)，SHA256 `5eeaa69d4d5b451b5ae2ecf8674dd206a1c60496ff96a4f52e06af150d4cec69`。
- [第一阶段成果：导航归档](../../baselines/baseline-20261119-100-055-smooth-r035/navigation_runtime_100_055_smooth_r035.tar.gz)，SHA256 `102a0ddd18f2538fba84d44abcaa2b59b1aba3d267a1f6517fe8504470aabafd`。
- 第二阶段失败试验的原部署包 SHA256 为 `22b738872d381ed3ceebf93bd4ce70b95b6a9d856c000ca7209de9d50b04f17a`，试验配置及失败记录见 [对应说明](../../.codex-build/localization-20261119/candidates/localization_20261119/README_LOCALIZATION_BEAMSKIP_FAILED.md)。

以后新增可保留版本时，先核对归档、入口覆盖和实际组合，再增加一列或一条记录。配置变更与新实测结果分别更新，保持来源可追溯。
