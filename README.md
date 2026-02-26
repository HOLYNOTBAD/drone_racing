# Drone Racing (ROS/Catkin)

基于 Fast-Planner 与 `uav_simulator` 的无人机竞速仿真工程，包含：

- 轨迹规划与状态机（`plan_manage`）
- SO3 控制与动力学仿真（`uav_simulator`）
- 圈速相关数据记录与可视化（`lap_logs.csv` + `data/plot.py`）

## 1. 项目结构

- `src/plan_manage`: 竞速主流程、状态机、轨迹服务、日志记录
- `src/uav_simulator`: 飞行器动力学仿真、控制器、可视化工具
- `src/bspline`, `src/traj_utils`: 轨迹表示与可视化支持
- `data/lap_logs.csv`: 每圈速度/推力日志
- `data/plot.py`: 日志绘图脚本（pandas + matplotlib）

## 2. 环境依赖

建议环境（与原工程一致）：

- Ubuntu + ROS (catkin 工作空间)
- Python 3

常用系统依赖（按需补齐）：

- `libarmadillo-dev`
- `nlopt`
- Python 依赖（用于画图）：

```bash
python3 -m pip install pandas matplotlib
```

## 3. 编译

在工作空间根目录（本目录）执行：

```bash
catkin_make
source devel/setup.bash
```

## 4. 运行仿真（含 auto_fsm / 手动模式）

先启动 RViz：

```bash
source devel/setup.bash
roslaunch plan_manage rviz.launch
```

再开一个终端启动竞速系统：

```bash
source devel/setup.bash
roslaunch plan_manage racing.launch
```

### 4.1 auto_fsm 模式（自动跑圈）

`racing.launch` 当前默认就是自动模式：

- `fsm/auto_fsm=true`
- `fsm/auto_fsm_iteration=10`

含义：

- `auto_fsm=true`：在 `WAIT_TRIGGER` 和 `HOVER` 状态不需要人工触发，FSM 会自动进入下一阶段
- `auto_fsm_iteration=10`：最多自动跑 10 圈（`<=0` 表示不限制）

自动模式下典型状态流：

- `INIT -> GO_START -> WAIT_TRIGGER -> EXEC_TRAJ -> HOVER -> GO_START ...`
- 每完成一圈，`/planning/lap_count` 自增并发布
- 达到 `auto_fsm_iteration` 后会自动退出自动模式（内部将 `auto_fsm_` 置为 `false`）

如何修改自动圈数：

```bash
source devel/setup.bash
roslaunch plan_manage racing.launch
```

然后在 launch 文件中调整：

- `src/plan_manage/launch/racing.launch` 里的 `fsm/auto_fsm_iteration`

### 4.2 非 auto_fsm 模式（手动触发）

将 `racing.launch` 中参数改为：

- `fsm/auto_fsm=false`

此时状态机会在 `WAIT_TRIGGER` 等待人工触发。触发方式：

- 在 RViz 中使用 `2D Nav Goal` 工具发送目标（话题 `/move_base_simple/goal`）

手动模式下常用流程：

1. 启动 `rviz.launch` 与 `racing.launch`
2. 等待无人机到起点附近并完成朝向对齐（进入 `WAIT_TRIGGER`）
3. 在 RViz 点一次 `2D Nav Goal`，开始一圈（进入 `EXEC_TRAJ`）
4. 飞完后进入 `HOVER`
5. 想继续下一圈，再点一次 `2D Nav Goal`

手动模式下的额外行为：

- 飞行中（`EXEC_TRAJ`）若再次触发 `2D Nav Goal`，会中断当前轨迹并进入 `HOVER`

注意：

- 这里的 `2D Nav Goal` 主要作为“触发信号”；赛道航点来自 `racing.launch` 里 `fsm/waypoint*` 参数配置。

## 5. 圈速日志与绘图

`il_traj_server` 会把每圈数据写入 CSV，字段如下：

- `lap_id`
- `t`
- `cmd_speed`
- `cmd_thrust`
- `real_speed`
- `real_thrust`

绘图命令：

```bash
python3 data/plot.py
```

可选参数：

```bash
python3 data/plot.py --lap 1
python3 data/plot.py --save data/lap_plot.png
```

## 6. 重要注意事项

当前代码里日志路径是硬编码的：

- `src/plan_manage/src/il_traj_server.cpp:622`
- `g_lap_logger.init("/home/holy/drone_racing/data/lap_logs.csv");`

如果你的工程路径不是 `/home/holy/drone_racing`，请修改这一行后重新编译，否则日志可能写不到当前仓库的 `data/lap_logs.csv`。

## 7. 常见问题

- 运行 `plot.py` 提示缺少模块：安装 `pandas`、`matplotlib`
- 图像窗口不弹出：可使用 `--save` 先导出图片
- 启动后无轨迹：确认 `source devel/setup.bash` 已执行，且各 ROS 节点正常启动
