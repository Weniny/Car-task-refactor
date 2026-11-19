# 当前导航交接入口：2026-11-19 收车

今日现场测试已结束，剩余实测安排到 2026-11-20。用户指定保留 17:45 效果明显改善的平顺弯道提速对照，标识为 `baseline-20261119-100-055-smooth-r035`，包含原导航归档和当时的原 AMCL `likelihood_field` 配置。

详细记录在独立分支 `codex/localization-continuity-20261119`、目录 `.codex-build/localization-20261119`，本根目录只维护接续入口，不合并试验源码。

| 用途 | 唯一明确命名的说明 |
| --- | --- |
| 明日先读 | [README_NEXT_TEST_20261120.md](../.codex-build/localization-20261119/docs/handoffs/README_NEXT_TEST_20261120.md) |
| 今日保留的 1.00／0.55 平顺弯道版本 | [README_1MPS_TURN055_SMOOTH.md](../.codex-build/localization-20261119/candidates/stepwise_025/README_1MPS_TURN055_SMOOTH.md) |
| 原始归档、清单与定位配置 | [README_FROZEN_TURN055_20261119.md](../.codex-build/localization-20261119/baselines/baseline-20261119-100-055-smooth-r035/README_FROZEN_TURN055_20261119.md) |
| 原 1.00／0.30 对照 | [README_1MPS_TURN030_BASELINE.md](../.codex-build/localization-20261119/candidates/stepwise_025/README_1MPS_TURN030_BASELINE.md) |
| 后续失败的 AMCL 概率模型试验 | [README_LOCALIZATION_BEAMSKIP_FAILED.md](../.codex-build/localization-20261119/candidates/localization_20261119/README_LOCALIZATION_BEAMSKIP_FAILED.md) |
| 提速经验与故障复盘 | [README_SPEED_UPGRADE_EXPERIENCE.md](../.codex-build/localization-20261119/docs/speed_upgrade/README_SPEED_UPGRADE_EXPERIENCE.md) |
| 全部文档入口 | [README_NAVIGATION_INDEX.md](../.codex-build/localization-20261119/docs/README_NAVIGATION_INDEX.md) |

18:35 概率模型试验发生右侧接触、两次短暂定位跳变和动态障碍弯道规划超时，实车未通过，没有合入成功对照。保留版的最低反馈弯速仍约 0.29–0.30 m/s，空场改善不等于最低 0.50 m/s 或避障安全已验收。

收车期间助手只整理本地文件和已有分析，没有切换车端 D/E 或发布运动指令。复现成功对照时需同时核对导航工作区和原 AMCL 配置，不能据此推断车端当前已自动回退。
