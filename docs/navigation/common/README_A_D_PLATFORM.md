# A–D 共用平台启动单

[返回导航总览](../README_NAVIGATION_OVERVIEW.md) · [下一步：第一阶段 E–G](../stage1/README_E_G_STAGE1.md)

本页只负责底盘、传感器、地图和定位。当前平台基准对应第一阶段成果，D 使用原 AMCL `likelihood_field` 配置。各终端都在车端执行，B 在小车图形桌面执行，以便显示 RViz。

| 终端 | 内容 |
| --- | --- |
| A | Livox、FAST-LIO、底盘反馈 |
| B | 静态地图与 RViz |
| C | 定位激光 `/localization/scan` |
| D | AMCL，连接 `map → camera_init → body` |

已经健康运行的终端沿用。冷启动才按 A→B→C→D 顺序启动，每项保持一套进程。

## A：底盘、雷达与 FAST-LIO

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_mapping.launch.py \
  start_livox:=true force_livox_host_timestamps:=true \
  start_fast_lio:=true fast_lio_config:=fast_lio_mid360_navigation.yaml \
  start_base:=true port_name:=can2 publish_odom_tf:=false \
  start_scan:=false start_slam:=false start_anchor:=false rviz:=false
```

<details>
<summary>仅供冷启动时尚未配置 CAN 的情况</summary>

```bash
sudo modprobe can_raw
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000 restart-ms 100
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up
ip -statistics -details link show can2
timeout 5s candump -n 20 can2
```

在启动 A 前完成。CAN 已正常运行时沿用原配置。

</details>

## B：地图与 RViz

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:=/home/agilex/competition_rebuild_ws/maps/indoor_manual_final/map.yaml \
  rviz:=true \
  rviz_config:=/home/agilex/competition_startup_capture_20261104_ws/src/forward_rejoin_bringup/rviz/navigation_debug_view.rviz
```

这里引用的 RViz 配置来自原冻结目录，作用是显示地图、定位扫描和导航话题。导航版本由后续 E 加载的目录确定。

## C：定位激光

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r __node:=rebuild_localization_scan \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/pointcloud_to_laserscan_rebuild_localization.yaml \
  -r cloud_in:=/cloud_registered_body -r scan:=/localization/scan
```

## D：第一阶段基准定位

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 run nav2_amcl amcl --ros-args -r __node:=amcl \
  --params-file /home/agilex/competition_rebuild_ws/config/localization/amcl_rebuild_indoor.yaml \
  -r scan:=/localization/scan
```

该文件对应原 `likelihood_field`。11 月 19 日试过的 `likelihood_field_prob` 没有合入第一阶段成果。第二阶段的新 D 配置需随具体候选说明，本页保留已归档的基准。

## 初始化并确认平台就绪

在 F 先检查：

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 param get /amcl laser_model_type
```

两个生命周期应为 `active`，模型应为 `likelihood_field`。AMCL 若为 `unconfigured`，在 F 执行一次：

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

随后在 RViz 用 **2D Pose Estimate** 按小车真实位置和朝向初始化，确认定位扫描贴合静态墙面。需要核对 TF 时，在 F 执行：

```bash
ros2 run tf2_ros tf2_echo map body
```

看到持续输出的变换后按 Ctrl+C 结束查看。若门检报 `no data from TF map->body`，先核对 D 是否 active、C 是否有数据，以及是否完成实际位姿初始化。此问题就绪后再进入 E–G。

| RViz 显示 | 话题 | 出现时机 |
| --- | --- | --- |
| 地图 | `/map` | B 启动 |
| 定位扫描 | `/localization/scan` | C 启动，定位 TF 连通 |
| 蓝色参考线 | `/planning/replan_reference_path` | E 启动 |
| 绿色局部轨迹 | `/planning/local_trajectory` | E 正常规划 |
| 黄色执行轨迹 | `/control/replan_executed_path` | E 获取有效定位后累计 |

平台就绪后打开 [第一阶段 E–G 启动单](../stage1/README_E_G_STAGE1.md)。
