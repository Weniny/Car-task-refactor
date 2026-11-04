# competition_localization

负责定位链路中的项目自有适配逻辑。

当前包含 FAST-LIO 本地坐标系到项目 `map` 坐标系的可选锚定节点。该包不负责启动硬件；启动编排仍由 `competition_bringup` 处理。
