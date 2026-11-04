# competition_safety

负责急停、CAN/遥控控制权、状态断流、跟踪误差、底盘错误、意外运动模式、超速和避障停车请求。

`SafetySupervisor` 是纯 Python seam，`safety_node` 是 ROS adapter。节点订阅 `/system_state` 和 `/motion_state`，只有系统正常、CAN 控制模式、双阿克曼模式、控制/定位消息新鲜且误差在界内时才允许非零输出。默认输出 `/cmd_vel_safe`；普通控制节点不得绕过本包直接向 `/cmd_vel` 发布。
