# A–D：第一阶段共用平台

[返回入口](../README.md) · [依赖与构建](README_SETUP.md) · [下一步 E–G](README_E_G_TEST.md)

各终端在车端执行，B 在小车图形桌面执行。下面使用 `~/car_stage1`；若克隆到了别处，各终端统一替换该目录。A–D 已健康运行时先核对版本与配置，冷启动才按 A→B→C→D 顺序执行，每项只保留一套进程。

## A：底盘、雷达和 FAST-LIO

```bash
cd "$HOME/car_stage1" &&
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh &&
ros2 launch rebuild_bringup rebuild_mapping.launch.py \
  config_path:="$PWD/platform_config" \
  start_livox:=true force_livox_host_timestamps:=true \
  start_fast_lio:=true fast_lio_config:=fast_lio_mid360_navigation.yaml \
  start_base:=true port_name:=can2 publish_odom_tf:=false \
  start_scan:=false start_slam:=false start_anchor:=false rviz:=false
```

`config_path` 显式选本分支保存的原 FAST-LIO 配置。CAN 已正常工作时沿用；冷启动尚未配置 CAN 时，先在车端完成：

```bash
sudo modprobe can_raw
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000 restart-ms 100
sudo ip link set can2 txqueuelen 1000
sudo ip link set can2 up
ip -statistics -details link show can2
timeout 5s candump -n 20 can2
```

## B：地图与 RViz

```bash
cd "$HOME/car_stage1" &&
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh &&
ros2 launch rebuild_bringup rebuild_final_map.launch.py \
  map_yaml:="$PWD/runtime/maps/indoor_manual_final/map.yaml" \
  rviz:=true \
  rviz_config:="$PWD/runtime/src/forward_rejoin_bringup/rviz/navigation_debug_view.rviz"
```

## C：定位扫描

```bash
cd "$HOME/car_stage1" &&
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh &&
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r __node:=rebuild_localization_scan \
  --params-file "$PWD/platform_config/pointcloud_to_laserscan_rebuild_localization.yaml" \
  -r cloud_in:=/cloud_registered_body -r scan:=/localization/scan
```

## D：原 AMCL 模型

```bash
cd "$HOME/car_stage1" &&
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh &&
ros2 run nav2_amcl amcl --ros-args -r __node:=amcl \
  --params-file "$PWD/platform_config/amcl_rebuild_indoor.yaml" \
  -r scan:=/localization/scan
```

显式读取本分支的 `likelihood_field` 配置，避免沿用后续 `likelihood_field_prob` 试验的可变文件。

## F：平台就绪核对与初始化

```bash
source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 param get /amcl laser_model_type
```

两个生命周期应为 `active`，模型应为 `likelihood_field`。AMCL 若为 `unconfigured`，在 F 执行一次：

```bash
ros2 lifecycle set /amcl configure &&
ros2 lifecycle set /amcl activate
```

随后在 RViz 用 **2D Pose Estimate** 按真实起点和朝向初始化，确认定位扫描贴合墙面：

```bash
ros2 run tf2_ros tf2_echo map body
```

持续输出变换后 Ctrl+C 结束查看。若没有 `map→body`，先检查 AMCL、C 的扫描和初始位姿，再进行 E–G。

| RViz 内容 | 话题 |
| --- | --- |
| 地图／定位扫描 | `/map`／`/localization/scan` |
| 蓝色参考线 | `/planning/replan_reference_path` |
| 绿色局部轨迹 | `/planning/local_trajectory` |
| 黄色执行轨迹 | `/control/replan_executed_path` |

导航轨迹在 E 就绪后出现。绿黄线断开要结合定位、规划和控制状态诊断，不应只凭图形判断为 IMU 漂移。
