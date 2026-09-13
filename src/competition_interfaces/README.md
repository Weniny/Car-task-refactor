# competition_interfaces

共享的 ROS 2 接口。当前 `ArmTask.action` 是总状态机与常驻机械臂任务节点之间的
唯一任务接缝：一次 goal 对应一个停车点上的 `PICKUP` 或 `DROP`，feedback 描述
图片识别、实物识别和操作阶段，result 明确区分成功、未识别和操作失败。

存放 `VehicleTrajectory`、`TrajectoryPoint`、`BodyCommand`、`WheelCommand`、任务状态、避障状态以及 `Dock`、`LogisticsTask` 等自定义接口。

当前只建立接口目录；字段仍是方案草案，待规划、控制、机械臂和避障成员联调后冻结。
