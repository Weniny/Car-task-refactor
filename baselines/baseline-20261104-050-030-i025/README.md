# 冻结基线 baseline-20261104-050-030-i025

核对与归档日期：2026-11-05。名称中的 20261104 是候选选择日期。Git 标签同目录名；标签指向本次归档提交，不代表高速或完整实车验收通过。

## 内容

- 原固定导航源码：仓库 `candidates/startup_capture_20261104/`，原 127 文件清单保留不改。
- [runtime/car_dependencies.tar.gz](runtime/car_dependencies.tar.gz)：车端实际导航文件与 A–D 依赖源码、配置、地图、安装后的包 share 文件，共 747 个文件。
- [runtime/SNAPSHOT.json](runtime/SNAPSHOT.json)：各文件 SHA256、实际包路径、30 个程序/库指纹、ROS Debian 包版本、上游 Git 状态和排除项。
- [runtime/import_check.txt](runtime/import_check.txt)：只读导入检查 PASS，六项核心模块均来自 startup_capture 工作区。
- 操作入口：[当前运行单](../../docs/CURRENT_RUNBOOK.md)。

归档 SHA256：`fb84c910f62003f5f2279c93cb5b6cf90884d325630d0726d2d69855b6efb92b`。

## 对照结论与例外

127 文件对照中 126 个逐字相同。唯一例外是 `src/forward_rejoin_bringup/rviz/navigation_debug_view.rviz`。解析 YAML 后，差异限于窗口/视角、缩放、保存时的显示浮点精度；话题、QoS、导航参数未变。车端文件与原固定版分别保留，例外的精确 SHA256 见 SNAPSHOT 的 `reviewed_display_only_exception`。未重写车端原 manifest，因此其中的 RViz 历史散列仍不匹配；实际归档以本目录 SNAPSHOT 的文件散列为准。

A–D 与 E 使用的 map.yaml、map.pgm 散列分别一致。A–D 的 rebuild_bringup 从 rebuild 工作区加载；FAST-LIO、Livox、Ranger 从 agilex_ws 加载；AMCL、map_server、pointcloud_to_laserscan 来自 `/opt/ros/humble`。ugv_sdk 作为独立 SDK 归档，不是此环境注册的 ROS 包。

## 恢复原则

1. 用 Git 标签在新目录检出基线，不覆盖正在开发的工作树。
2. 核验压缩包 SHA256。解压到临时检查目录，归档中的 `files/home/agilex/...` 和 `files/opt/ros/...` 表示原始绝对路径；不要直接向 `/` 解压覆盖。
3. 导航 E 可从 `candidates/startup_capture_20261104` 重建；若需要还原实际车端视图，使用归档中对应 RViz 文件。依赖源码、A–D 配置与地图按 SNAPSHOT 核对后恢复到相应工作区并构建。
4. ROS 系统包按记录版本核对；驱动与 FAST-LIO 重建后的产物指纹可能因编译环境变化不同，应重新检查来源、配置和功能。
5. 按运行单重新完成冷启动、定位和发车前门检，再由操作者开始测试。此次仅核验文件、包来源与归档内容，没有运行实车门检、导航或发车。

这是源码、配置与环境指纹快照，不是整机磁盘镜像；未打包已安装二进制、系统依赖全集、驱动器固件、原始录包、PCD 和视频。排除项有清单，恢复后仍需构建和门检。最新整圈录包的分析不属于本次冻结完成条件，留在升级交接单中。
