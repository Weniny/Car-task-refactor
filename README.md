# 小车导航

**现场操作先看两张单：** [A–D 共用平台](docs/navigation/common/README_A_D_PLATFORM.md) → [第一阶段 E–G 启动](docs/navigation/stage1/README_E_G_STAGE1.md)。

| 阶段 | 当前状态 | 入口 |
| --- | --- | --- |
| 第一阶段：直线提速与弯道平顺化 | **成果已归档，作为后续基线** | [第一阶段成果](docs/navigation/stage1/README_STAGE1_RESULT.md) |
| 第二阶段：定位跳变诊断与优化 | **当前工作主线** | [第二阶段目标与待办](docs/navigation/stage2/README_STAGE2_LOCALIZATION.md) |

第一阶段固定为 `baseline-20261119-100-055-smooth-r035`，直线目标 1.00、弯道参考 0.55、恢复速度 0.35 m/s，使用原 AMCL `likelihood_field`。实测最低弯速约 0.29–0.30 m/s，后续定位与障碍问题单独保留。

## 文件从哪里找

- **日常启动、阶段结果和待办：** [docs/navigation/README_NAVIGATION_OVERVIEW.md](docs/navigation/README_NAVIGATION_OVERVIEW.md)。
- **固定归档与恢复材料：** `baselines/`。
- **开发工作树与历史分析：** `.codex-build/`，通过成果页或待办页的追溯链接查看即可。

电脑上的项目目录与 SSH 中的 `/home/agilex/...` 是两处位置，启动单会明确每个终端使用哪个车端目录。完整目录图、A–G 职责及历史资料统一见 [导航总览](docs/navigation/README_NAVIGATION_OVERVIEW.md)。

新对话先读 [项目状态](docs/PROJECT_STATE.md)，再读 [第二阶段说明](docs/navigation/stage2/README_STAGE2_LOCALIZATION.md)。
