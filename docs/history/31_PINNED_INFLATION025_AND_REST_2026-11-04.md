> 历史记录，仅用于追溯。当前版本、状态和操作统一见 [当前运行指令](../CURRENT_RUNBOOK.md)。文中“当前/明日/下一轮”均指记录当时。

# 2026-11-04：固定 0.25 入口，暂停实测休息

用户明确选择保留以下版本作为默认运行入口：

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup startup_capture_050_030_inflation025.launch.py start_chassis_adapter:=true
```

电脑端已核对该入口的 `grid_inflation_radius_m=0.25`，同步恢复默认 RUNBOOK.md 与构建脚本的入口显示，新增 CURRENT_RUNTIME_PROFILE.json。未修改节点实现、轨迹、速度或其他保护；127 个清单文件完整性检查通过。

当前 SSH 地址 172.20.10.12 连接超时。用户随后要求让小车休息，并报告长时间工作后激光点云一直漂移；停止继续连接和车端测试。电脑端固定已完成，车端默认运行单和清单待下次连接同步，不能将它表述为已在车端切换。原 inflation025 入口此前一直保留，未被 0.30 入口覆盖。

本轮碰撞记录仍为未通过。选择固定 0.25 不改变这个结果；点云漂移是否由连续运行时长导致尚未确认。

下次继续顺序：

1. 小车休息结束、恢复连接后，先同步默认版本记录。
2. 重新启动传感器和定位，静止观察点云、地图和 TF 是否稳定；若仍漂移，优先定位原因。
3. 稳定后再发布真实初始位姿、完成门检并新开录包，人工发车。

本次没有启动导航、底盘或路线。

## 待同步文件

- `output/candidates/pin_inflation025_update_20261104.zip`：只更新默认运行单、构建入口显示和版本清单。
- `output/candidates/startup_capture_pinned_inflation025_20261104.zip`：完整固定版本。
- [本地固定及未同步状态](../evidence/2026-11-04_pinned_inflation025.json)。
