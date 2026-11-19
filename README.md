# 室内小车导航

2026-11-19 收车后的版本与文档入口见 [README_CURRENT_NAVIGATION_20261119.md](docs/README_CURRENT_NAVIGATION_20261119.md)。

用户指定保留 17:45 效果明显改善的平顺弯道提速版本，标识为 `baseline-20261119-100-055-smooth-r035`。后续定位与障碍物试验发生接触和停车，实车验证未通过。剩余实测安排到 11 月 20 日。

详细记录位于独立开发目录 `.codex-build/localization-20261119`，分支为 `codex/localization-continuity-20261119`。新对话先读[项目状态](docs/PROJECT_STATE.md)和[明日交接](.codex-build/localization-20261119/docs/handoffs/README_NEXT_TEST_20261120.md)。

| 需要查看什么 | 入口 |
|---|---|
| 最新版本、状态与明日入口 | [README_CURRENT_NAVIGATION_20261119.md](docs/README_CURRENT_NAVIGATION_20261119.md) |
| 保留的平顺弯道版本 | [README_1MPS_TURN055_SMOOTH.md](.codex-build/localization-20261119/candidates/stepwise_025/README_1MPS_TURN055_SMOOTH.md) |
| 冻结 025 的运行与恢复 | [CURRENT_RUNBOOK.md](docs/CURRENT_RUNBOOK.md) |
| 当前版与旧方案的区别 | [版本对照](docs/VERSION_HISTORY.md) |
| 下一轮提速、弯道和避障升级 | [升级计划与本地审计](docs/UPGRADE_PLAN.md) |
| 原 01–31 技术与测试记录 | [历史记录目录](docs/history/README.md) |

原冻结 `baseline-20261104-050-030-i025`、运行源码及依赖保持原样。当前保留版只支持空场弯速与衔接对照，最低弯速 0.50 m/s 和障碍物安全验收仍未通过。收车期间助手仅整理本地文档及已有证据。
