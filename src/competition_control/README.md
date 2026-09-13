# competition_control

负责 MPPI 轨迹跟踪、RANGER 四轮四转曲率模型和 ROS body command adapter。

- `mppi_controller.py` 是纯 Python seam；输入冻结的连续轨迹和车辆状态，输出 `BodyCommand`。
- `mppi_control_node.py` 使用锚定 FAST-LIO `map -> body` 位姿和 `/odom` 速度，发布 `/control/body_cmd`，不发布最终底盘 topic。
- 正常跟踪曲率限制为 `1 / 0.81m`，避免 Ranger 驱动隐式切入差速自旋。
- RPP 当前未实现，只保留为后续诊断对照意图，不能用于正式验收。
