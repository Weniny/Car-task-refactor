# 第一阶段提速升级

本分支供组员复现已保留的直线提速与弯道平顺化版本。固定成果标识为 `baseline-20261119-100-055-smooth-r035`，对应 2026-11-19 17:45 那轮效果明显改善的实车测试。

| 项目 | 本版数值 |
| --- | --- |
| 直线速度上限 | 1.00 m/s |
| 弯道参考速度上限 | 0.55 m/s |
| Safety 跟踪恢复速度 | 0.35 m/s |
| 加速度／减速度 | 0.12／0.80 m/s² |
| 实测四弯最低速度 | 0.294／0.292／0.296／0.296 m/s |
| 定位配置 | 原 AMCL `likelihood_field` |

第一阶段已取得空场整圈提速和衔接改善。最低弯速 0.50 m/s 尚未达到；后续测试报告的定位跳变、障碍物接触和动态障碍弯道停车仍待修正。本分支没有合入后续的 `likelihood_field_prob` 试验配置。

## 组员从这里开始

以下命令在车端 Ubuntu 执行。每人使用独立的新目录；示例统一为 `~/car_stage1`，该目录须尚未存在。私有仓库需要组员自己的 GitHub 访问权限。

```bash
cd "$HOME" &&
git clone --single-branch --branch '第一阶段提速升级' \
  https://github.com/Weniny/Car-task-refactor.git car_stage1 &&
cd "$HOME/car_stage1" &&
git rev-parse HEAD &&
python3 scripts/verify_stage1_release.py
```

本仓库顶层负责文档和追溯，**`runtime/` 才是本版导航 ROS 工作区**。它逐字节保留冻结归档中的源码、地图、轨迹、配置和脚本，不使用开发主目录里的旧 `src/`。工作区路径由现有环境脚本自动计算，可与车上原成功目录并存。

1. [先准备依赖和构建](docs/README_SETUP.md)：构建命令不启动导航或发车。
2. [A–D：共用平台](docs/README_A_D_PLATFORM.md)：底盘、传感器、地图、定位。
3. [E–G：本分支测试](docs/README_E_G_TEST.md)：导航、录包、版本检查、门检、人工发车和停车。
4. [参数、实测结果与已知问题](docs/README_PARAMETERS_AND_LIMITS.md)。
5. [组内协作与问题反馈](docs/README_COLLABORATION.md)。

| 目录 | 用途 |
| --- | --- |
| `runtime/src/` | 五个本版导航 ROS 包及原始 launch 参考文件 |
| `runtime/config/`、`routes/`、`artifacts/`、`maps/` | 本版参数、路线、平顺轨迹和地图 |
| `platform_config/` | 成功测试对应的 AMCL、定位扫描和 FAST-LIO 配置 |
| `docs/` | 本分支唯一操作说明 |
| `provenance/` | 原导航归档、原平台源码依赖快照、指纹和发布清单 |

`runtime/README.md`、`runtime/RUNBOOK*.md` 和 `runtime/log/` 内五个旧参数快照属于清单校验覆盖的历史来源，仅用于追溯；日常操作使用上面 `docs/` 的说明。`050_030` 等历史文件名不能代表当前速度。

下载和构建完成后，仍须按现场安排执行启动、初始化和门检；同一台车同一时刻只运行一套导航 E。仓库没有包含 ROS、已编译驱动、完整车端系统镜像或实车录包。
