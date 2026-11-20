# 依赖与构建

[返回入口](../README.md) · [下一步 A–D](README_A_D_PLATFORM.md)

## 已有测试车环境

本分支保持成功版运行环境约定：Ubuntu、ROS 2 Humble，车端账户为 `agilex`，已有 `/home/agilex/agilex_ws/install/setup.bash` 和 `/home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh`。前者提供 Livox、FAST-LIO、Ranger 和 SDK，后者提供 A–D 的 `rebuild_bringup`。

系统还需 `colcon`、Python 3、NumPy、PyYAML，以及 Humble 的 `pointcloud_to_laserscan`、`nav2_amcl`、`nav2_map_server`、`nav2_lifecycle_manager`、`rviz2`、`rosbag2`。五个导航包的 ROS 依赖见各自 `package.xml`。

本分支直接复用上述车端依赖。换另一台车或系统时，先完成驱动构建、CAN、雷达网络、安装外参和地图适配；原扫描坐标、地图路线和硬件参数有具体现场含义。导航脚本保留原 `/home/agilex/agilex_ws` 前缀，不能把 Windows 下载后的文件直接当作可运行的车端环境。

## 下载后只构建

使用新终端，避免同时加载其他候选的导航环境：

```bash
cd "$HOME/car_stage1" &&
test -r /opt/ros/humble/setup.bash &&
test -r /home/agilex/agilex_ws/install/setup.bash &&
test -r /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh &&
python3 scripts/verify_stage1_release.py &&
cd runtime &&
bash scripts/build_candidate.sh
```

构建脚本选择 `competition_control`、`competition_planning`、`competition_safety`、`competition_localization`、`forward_rejoin_bringup`，随后运行离线回归、导入检查和 launch `--show-args`。这一步不启动传感器、导航进程或底盘适配器，也不发布路线使能。

构建后需要看到导入来源为本次 `~/car_stage1/runtime/src/`，并完成脚本自带检查。每个实际使用 E、F、G 的新终端都须重新 `cd` 并加载本目录 `scripts/source_candidate_runtime.sh`。

## 依赖来源和完整性

- `provenance/navigation_runtime_100_055_smooth_r035.tar.gz`：原导航归档，SHA256 `102a0ddd18f2538fba84d44abcaa2b59b1aba3d267a1f6517fe8504470aabafd`；`runtime/` 的 128 个受检文件与其清单一致。
- `provenance/car_dependencies_025.tar.gz`：最初冻结版的 747 文件平台／源码依赖快照，SHA256 `fb84c910f62003f5f2279c93cb5b6cf90884d325630d0726d2d69855b6efb92b`，用于追溯 A–D 和驱动来源。它含原有第三方许可文件，没有完整系统安装与编译产物，不是自动安装程序。
- `provenance/PLATFORM_SOURCE_SNAPSHOT_025.json` 和 `LOCALIZATION_SOURCE_SNAPSHOT.json`：平台、包版本和库指纹；其中绝对路径记录的是原采集现场。
- `platform_config/`：本阶段实际保留的三个配置，根目录校验脚本同时检查这些文件。

需要查看原依赖快照时，先只列目录：

```bash
cd "$HOME/car_stage1"
tar -tzf provenance/car_dependencies_025.tar.gz | less
```

快照成员以 `files/home/agilex/` 或 `files/opt/ros/` 开头。恢复依赖应先在新的临时目录查看并单独构建；本分支启动单直接使用既有平台，不提供覆盖现有工作区的命令。
