# competition_planning

负责目标点选择、语义走廊全局路径、occupancy-grid A* 搜索、几何轨迹平滑，以及后续 Hybrid A*/State Lattice 插槽。规划结果必须显式发布，不能直接绕过轨迹跟踪器控制底盘。

Day3 当前交付全局搜索 + 几何平滑路径：

- `semantic_planner.py`：纯 Python 规划与合法性校验接口。
- `occupancy_grid_planner.py`：基于 `map.yaml/map.pgm` 的膨胀栅格 A* 搜索。
- `trajectory_smoother.py`：保留语义锚点的 cubic Bezier 几何路径平滑。
- `offline_global_plan`：离线读取 route、semantic map、planning params，输出每个可规划 step 的路径或失败原因。
- `semantic_global_path_node`：复用同一规划结果，按当前 `step_id` 发布一条 `nav_msgs/Path` 到 `/planning/global_path`。

本模块不输出速度命令，不控制底盘，不触发机械臂动作。
