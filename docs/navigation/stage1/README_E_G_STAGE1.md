# 第一阶段 E–G 启动单

[返回导航总览](../README_NAVIGATION_OVERVIEW.md) · [A–D 共用平台](../common/README_A_D_PLATFORM.md) · [第一阶段成果](README_STAGE1_RESULT.md)

**本页只对应第一阶段 1.00／0.55 平顺弯道版本。** A–D 先按平台单就绪，D 模型为原 `likelihood_field`。切换旧导航前先在旧 F 停车、确认停稳，在旧 G 保存录包，再退出旧 E。

| 终端 | 用途 | 使用顺序 |
| --- | --- | --- |
| E | 启动本版本导航和底盘适配 | 1 |
| G | 开始本版本录包 | 2 |
| F | 核对版本、门检和人工发车 | 3 |

E、F、G 各用新的车端 SSH 会话，三者都进入 `/home/agilex/competition_stepwise_100_turn055_smooth_20261119_ws`。

## E：启动第一阶段导航

```bash
cd /home/agilex/competition_stepwise_100_turn055_smooth_20261119_ws &&
source scripts/source_candidate_runtime.sh &&
ros2 launch forward_rejoin_bringup startup_capture_050_030_inflation025.launch.py start_chassis_adapter:=true
```

启动文件名中的 `050_030` 沿用冻结来源，当前速度以本目录参数和 F 的在线版本检查为准。启动后路线初始禁用。

## G：先开始录包

```bash
cd /home/agilex/competition_stepwise_100_turn055_smooth_20261119_ws &&
bash scripts/record_navigation_trial.sh obstacle_stage1_turn055_smooth
```

看到新的 `Recording:` 目录并开始录制后，再去 F 核对。带 `obstacle_` 的标签会记录扫描与代价地图。

## F：核对版本和门检

```bash
cd /home/agilex/competition_stepwise_100_turn055_smooth_20261119_ws &&
source scripts/source_candidate_runtime.sh &&
python3 scripts/check_import_origin.py &&
python3 scripts/check_active_candidate.py &&
ros2 param get /amcl laser_model_type &&
python3 scripts/navigation_preflight.py --timeout 20
```

| 核对项 | 第一阶段应有结果 |
| --- | --- |
| Python 导入来源 | `PASS`，来自本工作区 |
| 在线 E 版本 | `ACTIVE CANDIDATE PASSED` |
| D 的模型 | `String value is: likelihood_field` |
| 门检 | `PRECHECK PASSED` |
| 本版配置 | 直线 1.00、弯道参考 0.55、恢复速度 0.35 m/s |

模型若为 `likelihood_field_prob`，表示仍使用后续试验的 D，须按 [平台单](../common/README_A_D_PLATFORM.md)核对所选配置后再继续。若缺少 TF 或起步评估尚未就绪，先完成对应初始化与门检条件。

## F：现场确认后人工发车

G 正在录制、上述版本与门检结果符合第一阶段配置，且现场安排好后，在 F 执行：

```bash
python3 scripts/route_command.py start
```

该工具会再次门检并等待控制器确认。

## F 停车，G 保存

终点或中途异常时在 F 执行：

```bash
python3 scripts/route_command.py stop
```

确认实车停稳与回执后，在 G 按 Ctrl+C 保存，再按需要退出 E。整套关闭时，在 E 结束后依次关闭 D、C、B、A。A–D 需要继续沿用时，保留已经健康运行的平台。

本页复现第一阶段空场提速结果。已报告的障碍物接触和定位停顿继续按 [第二阶段待办](../stage2/README_STAGE2_LOCALIZATION.md)核查，尚未完成相应场景验收。
