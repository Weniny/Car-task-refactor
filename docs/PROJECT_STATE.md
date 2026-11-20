# 项目状态

更新：2026-11-20。当前文档入口为 [导航总览](navigation/README_NAVIGATION_OVERVIEW.md)。

## 阶段划分

| 阶段 | 状态 | 说明 |
| --- | --- | --- |
| 第一阶段：直线提速与弯道平顺化 | **成果已归档** | 用户确认将 11 月 19 日 17:45 保留版作为第一阶段结果 |
| 第二阶段：定位跳变诊断与优化 | **当前工作主线** | 从第一阶段结果继承，定位跳变与连续性修正为主目标 |

第一阶段结果固定为 `baseline-20261119-100-055-smooth-r035`，导航目录为 `/home/agilex/competition_stepwise_100_turn055_smooth_20261119_ws`，原 AMCL 模型为 `likelihood_field`。直线目标 1.00、弯道参考 0.55、恢复速度 0.35 m/s，实测四弯最低速度为 0.294／0.292／0.296／0.296 m/s。完整参数与范围见 [第一阶段成果](navigation/stage1/README_STAGE1_RESULT.md)。

最低弯速 0.50 m/s、静态净距与动态避障仍有未完成的验证。阶段收束不改写这些实测结论。

## 唯一日常启动文档

1. [A–D 共用平台启动单](navigation/common/README_A_D_PLATFORM.md)：底盘、雷达、地图和定位。
2. [第一阶段 E–G 启动单](navigation/stage1/README_E_G_STAGE1.md)：本版本导航、核对与录包。

A–D 使用 `competition_rebuild_ws` 及既有依赖，E–G 使用具体版本的导航目录。D 的定位配置也要与该版本匹配。第二阶段尚无已接受的新运行版，已有 `likelihood_field_prob` 失败试验未合入第一阶段成果。

## 第二阶段接续

定位主线包括 AMCL 全局修正、FAST-LIO 连续局部运动、IMU 与时间戳、控制状态保护及异常恢复。并行保留右侧黑色泡沫箱接触、稀疏点防护和动态弯道规划超时记录。按 [第二阶段目标与待办](navigation/stage2/README_STAGE2_LOCALIZATION.md) 接续，不把所有停顿归为同一原因。

11 月 19 日实测表明：第一次绕行近障有效点为 2、2、1，未达 3 点停车门槛；另有两次短暂 `position_jump`；弯道最后持续停车的直接请求来自规划超时与空路径。原始记录通过第二阶段说明中的追溯链接查看。

## 文件与源码位置

| 位置 | 职责 |
| --- | --- |
| `docs/navigation/` | 当前权威操作单、阶段成果和目标 |
| `docs/navigation/README_RETAINED_VERSION_PARAMETERS.md` | 所有保留版本的配置参数、参考速度和实测值对照 |
| `baselines/baseline-20261119-100-055-smooth-r035/` | 第一阶段原导航归档、128 文件清单与定位配置 |
| `.codex-build/localization-20261119` | 当前第二阶段已有开发工作树，分支 `codex/localization-continuity-20261119` |
| `.codex-build/stepwise-025` | 第一阶段开发与旧分析，保留追溯 |
| `baselines/baseline-20261104-050-030-i025/` | 最初冻结 025 来源及更广的依赖快照 |

主目录 `src/` 和可变的 rebuild 工作区不自动作为车端导航 E 的权威来源。实际版本按阶段操作单、固定归档和在线检查确定。

本次只整理本地目录、文档和阶段定义，没有切换车端节点、修改运行参数或发车。
