# 室内小车导航重构

## 当前状态：实车测试冻结

2026-09-18 的一次整合候选启动中，`ranger_base_node` 打开 CAN 底盘通信后，车辆出现异常空转和焦味。现场已急停、断电，未再次上电验证。

在完成硬件检修、遥控器恢复和底盘节点启动行为复核前：

- 禁止启动任何包含 `start_base:=true`、`ranger_base_node` 或真实底盘适配器的 launch。
- 禁止解除急停后进行 ROS 运动测试。
- 禁止将 SSH 断线、进程退出或软件停车视为物理急停的替代品。

这不是 `0.5 m/s` 参数验收通过的证据。当前所有 `0.5 m/s` 配置和候选均为**未验证**状态。

## 文档入口

| 文档 | 用途 |
|---|---|
| [当前状态与安全交接](docs/01_CURRENT_STATUS_AND_SAFETY.md) | 事件记录、冻结范围、恢复前的硬件检查门槛。 |
| [已验证 0.20 m/s 基线](docs/02_VERIFIED_020MPS_BASELINE.md) | 已跑通的 R=1.2 m 路线、定位与避障能力边界。不是当前发车许可。 |
| [未验证 0.50 m/s 恢复计划](docs/03_UNVERIFIED_050MPS_RECOVERY.md) | 失败根因、比赛式停车候选、恢复后的分阶段验证。 |
| [参数调试指南](docs/04_PARAMETER_TUNING_GUIDE.md) | 速度、膨胀、局部规划与统一入口的参数流和修改方法。 |
| [0.50 m/s 历史证据](docs/05_HISTORICAL_050MPS_EVIDENCE.md) | rosbag 结论、已淘汰/未验证方案及其原因。 |
| [阶段性技术汇报](项目阶段性技术汇报_2026-09-13.md) | 2026-09-13 的项目历史汇报。 |

## 工程边界

- 运行工作区：`/home/agilex/competition_rebuild_ws`
- Ranger SDK/底盘节点：`/home/agilex/agilex_ws`
- 运行环境脚本：`/home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh`
- 小车当前 SSH 地址曾为 `172.20.10.12`；每次以当天实际网络地址为准。

## 历史与包级文档

`src/*/README.md` 为对应 ROS 包的说明；`artifacts/*.md` 描述轨迹产物。它们不是实车操作手册。
