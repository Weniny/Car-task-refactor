# 冻结保留：2026-11-19 平顺弯道提速对照

2026-11-20 用户确认将本版本作为第一阶段升级结果。固定文件已集中到主目录 `baselines/`，日常说明见 [第一阶段成果](../../docs/navigation/stage1/README_STAGE1_RESULT.md)。本次仅更新归档位置与文档入口，原导航包和定位配置保持原字节。

用户在当日收车前指定保留 17:45 那轮效果明显改善的版本。标识为 `baseline-20261119-100-055-smooth-r035`，范围为空场整圈的弯速与衔接对照。障碍物安全验收和最低弯速 0.50 m/s 目标尚未通过。

| 项目 | 保留内容 |
| --- | --- |
| 现场导航目录 | `/home/agilex/competition_stepwise_100_turn055_smooth_20261119_ws` |
| 导航来源提交／实测记录提交 | `bebbbd6`／`219dba1` |
| 原始冻结来源 | `baseline-20261104-050-030-i025` |
| 直线／弯道参考／跟踪恢复速度 | 1.00／0.55／0.35 m/s |
| 加速度／减速度／jerk | 0.12 m/s²／0.80 m/s²／2.00 m/s³ |
| 侧向加速度／曲率变化率 | 0.28 m/s²／2.50 m⁻¹/s |
| 平顺接缝 | 五次曲线，半长 0.60 m，共八处，最大位移约 12.5 mm |
| 当时 AMCL 模型 | **原 `likelihood_field`** |
| 实测最低速度，弯 1–4 | 0.294／0.292／0.296／0.296 m/s |
| 单弯耗时 | 3.743／3.785／3.801／3.650 s |
| 录包 | `retest_obstacle_stepwise_100_turn055_smooth_20261119_174500_906483360` |
| 原始数据库 SHA256 | `e5af12a91143887bd9fe696700cf12ed091c5436c55314cc3c4fe08d7fe7836b` |

使能期间 4251 帧控制状态均为 `TRACKING`，这轮未记录定位跳变或规划控制停车。后续带障碍物的试验出现接触和停车，不能将本冻结说明作为避障已通过的结论。

## 保留的文件

- `navigation_runtime_100_055_smooth_r035.tar.gz` 为现场待测前生成的原导航归档，SHA256 为 `102a0ddd18f2538fba84d44abcaa2b59b1aba3d267a1f6517fe8504470aabafd`，保留原字节，归档内 128 个导航文件均按清单复核。
- `NAVIGATION_MANIFEST.json` 为原导航文件清单，记录生成参数与来源。
- `localization/` 保存成功对照采用的原 AMCL、定位扫描和 FAST-LIO 配置。其中 AMCL SHA256 为 `ef2a040ee0921cde4927a54d8f37a19aff6a656faea85091be63210fdc4536e3`。
- `LOCALIZATION_SOURCE_SNAPSHOT.json` 保存上述配置、实际 AMCL 包版本与库指纹。
- `CHECKSUMS.json` 为本目录保留文件的 SHA256 和实测范围。README 的后续文字整理不改变原运行归档。

归档不包含原始录包、完整系统镜像或已构建的 ROS 包。原始录包保存在上述车端目录的 `log/` 中。A–D 更广的依赖快照仍参见原冻结 025。

## 恢复时选择整套配置

日常查看 [A–D 共用平台](../../docs/navigation/common/README_A_D_PLATFORM.md) 和 [第一阶段 E–G 启动单](../../docs/navigation/stage1/README_E_G_STAGE1.md)。恢复该次成功对照时，D 应使用保留的原 `likelihood_field` 配置；之后的 `likelihood_field_prob` 对照未合入本冻结版本，追溯见 [第二阶段说明](../../docs/navigation/stage2/README_STAGE2_LOCALIZATION.md)。

本地归档应先核对 SHA256。需要重建时，将归档传到车端新的恢复父目录，解出归档自带的 `competition_stepwise_100_turn055_smooth_20261119_ws` 子目录，执行其 `scripts/build_candidate.sh`。从新终端加载该目录的运行环境，核对导入来源、在线轨迹路径和参数后，再按现场安排初始化与门检。不要解压覆盖原成功工作区。

当前接续入口为 [第二阶段定位跳变优化](../../docs/navigation/stage2/README_STAGE2_LOCALIZATION.md)。文档重排没有切换车端 D、重启 E 或发车。
