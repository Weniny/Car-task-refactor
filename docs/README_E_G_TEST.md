# E–G：第一阶段独立测试

[返回入口](../README.md) · [先完成 A–D](README_A_D_PLATFORM.md)

E、F、G 各开新的车端终端，均进入同一次克隆的 `~/car_stage1/runtime`。先在旧 F 停车并确认停稳、旧 G 保存录包，再退出旧 E，之后才启动新 E；同一台车的组员测试需安排不同时间。

## E：启动导航

```bash
cd "$HOME/car_stage1/runtime" &&
source scripts/source_candidate_runtime.sh &&
ros2 launch forward_rejoin_bringup startup_capture_050_030_inflation025.launch.py start_chassis_adapter:=true
```

`050_030` 为继承的文件名，本版实际为直线 1.00、弯道参考 0.55、恢复速度 0.35 m/s。路线初始禁用。上面的命令接通底盘适配，执行须有现场安排。

## G：先录包

```bash
cd "$HOME/car_stage1/runtime" &&
bash scripts/record_navigation_trial.sh obstacle_stage1_turn055_smooth
```

看到 `Recording:` 和录制开始后再去 F。标签以 `obstacle_` 开头，会额外记录 `/scan` 和 `/avoidance/local_costmap`；参数、状态、TF 和指令随包保存。

## F：核对、门检

```bash
cd "$HOME/car_stage1/runtime" &&
source scripts/source_candidate_runtime.sh &&
python3 scripts/check_import_origin.py &&
python3 scripts/check_active_candidate.py &&
ros2 param get /amcl laser_model_type &&
python3 scripts/navigation_preflight.py --timeout 20
```

依次应看到：导入来源 `PASS` 且来自当前 `runtime/src/`、`ACTIVE CANDIDATE PASSED`、`likelihood_field`、`PRECHECK PASSED`。在线检查会核对轨迹和地图路径、速度、加减速度和 Safety 恢复速度。

缺少 TF 时回到 A–D 初始化；等待起步评估时保持静止并等新评估，不绕过门检。

## F：人工发车

现场人员确认净空、G 正在录制、版本和门检均通过后：

```bash
python3 scripts/route_command.py start
```

工具会再次门检并等待控制器确认。该阶段先复现空场整圈。障碍物接触和定位跳变仍未修复，新增场景须单独安排测试。

## F 停车，G 保存

```bash
python3 scripts/route_command.py stop
```

确认实车停稳和回执后，G 按 Ctrl+C 保存，随后按需要退出 E。整套收车时，在 E 退出后依次关闭 D、C、B、A。下次测试不覆盖旧成功目录。

每轮反馈记录完整提交号、使用目录、测试时间、录包路径、实际现象和改动；模板见 [组内协作](README_COLLABORATION.md)。
