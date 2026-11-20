# 导航升级交接：第一阶段已归档，进入第二阶段

更新：2026-11-20。先读 [项目状态](../PROJECT_STATE.md) 和 [导航总览](../navigation/README_NAVIGATION_OVERVIEW.md)。

## 第一阶段结果

用户确认将昨天保留的弯道提速版本作为第一阶段升级结果，固定标识为 `baseline-20261119-100-055-smooth-r035`。直线目标 1.00、弯道参考 0.55、恢复速度 0.35 m/s，D 使用原 `likelihood_field`。[结果与实测边界](../navigation/stage1/README_STAGE1_RESULT.md)

第一阶段原运行归档及 128 文件清单集中在主目录 `baselines/baseline-20261119-100-055-smooth-r035/`，原运行包和配置散列保持不变。原始来源仍为 `baseline-20261104-050-030-i025`。

## 第二阶段目标

定位跳变诊断与连续性优化是当前主线，后续源码在独立候选中从第一阶段结果继承。AMCL 概率模型已有一轮失败试验，尚无已接受的第二阶段运行版本。具体顺序、证据和关联问题见 [第二阶段说明](../navigation/stage2/README_STAGE2_LOCALIZATION.md)。

右侧障碍物接触和动态弯道规划超时继续保留，不能写作已解决。最低弯速 0.50 m/s 仍为后续提升项。

## 启动文档分工

- [A–D 共用平台](../navigation/common/README_A_D_PLATFORM.md)：启动底盘、传感器、地图和定位。
- [第一阶段 E–G 启动](../navigation/stage1/README_E_G_STAGE1.md)：按具体版本启动导航、核对和录包。

第二阶段若改 D 或 E–G，会随具体候选给出差异，不混入第一阶段操作单。用户负责现场启动和发车，未经现场安排不启动导航或运动。

## 开发与历史

当前第二阶段工作树为 `.codex-build/localization-20261119`，分支 `codex/localization-continuity-20261119`。第一阶段工作树 `.codex-build/stepwise-025` 及原冻结保留。遵守知识图谱优先规则，先查固定来源和实际运行参数。

11 月 5 日旧交接及原冻结背景可从原 Git 标签追溯。`165344` 包已经用户指出有定位可靠性疑点，不再作为当前离线验收依据。
