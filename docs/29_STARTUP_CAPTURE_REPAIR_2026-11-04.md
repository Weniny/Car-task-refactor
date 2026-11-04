# 29 号入口说明：当前使用的 0.25 m 版本

**是，这就是当前选择的起步接入修复算法。** 固定入口为：

```bash
source /home/agilex/competition_startup_capture_20261104_ws/scripts/source_candidate_runtime.sh
ros2 launch forward_rejoin_bringup startup_capture_050_030_inflation025.launch.py start_chassis_adapter:=true
```

完整步骤、当前验收状态和同步记录已统一到 [当前运行指令](CURRENT_RUNBOOK.md)，以后只维护这一份操作单。

- [29 号原始技术记录](history/29_STARTUP_CAPTURE_REPAIR_2026-11-04.md)：实现和软件验证来源。
- [版本对照](VERSION_HISTORY.md)：30 是临时 0.30 参数试验，31 是固定回 0.25 的选择记录，没有新增算法版本。

本地与车端默认选择已同步，点云/TF 稳定性仍待复查。碰撞记录保留，实车验收未通过。本文件保留原地址，便于已有收藏与对话链接继续使用。
