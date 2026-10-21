# 激光感知离线基线

本阶段实现测距分类、真实角度转换、局部候选通道选择和可视化。`process_core(urg)` 仍返回 `(steer, speed)`。尚未实现 S 弯行为、终点判断、方向 PD 或速度 PI；当前的转向只是目标弧度到控制量的比例映射，有候选方向时使用固定动力值。

## 数据如何变成目标方向

1. `perception.py:prepare_scan` 保留原始下标，用本帧角度参数计算每个方向，并叠加可配置的雷达安装偏角。前方 0° 两侧的数组首尾会在角度空间连起来。
2. 有限、正数且处于量程内的测距是有效点；正无穷单独标记为无回波/未知，其他非法值归为无效。负无穷或正数但低于最小量程表示近障碍风险，直接停止，不能作为短缺口补齐。**不会把 inf 替换为最大距离参与评分。**
3. 按固定角宽分箱，每箱检查有效比例、保留最近有效点。只有两端都有真实观测、距离相近且角宽很小的缺口允许保守补齐，补齐值取两侧较小距离并单独标记。长缺测段仍是未知。
4. 在前向范围内寻找连续、有足够观测深度和宽度的通道。根据宽度、前后悬和裕量建立以雷达为中心的车体包络圆盘，沿候选直线做扫掠检查。所有有效障碍点都参加检查，包括低覆盖分箱里的近点。
5. 从合格候选中优先选择已测净空较大、方向较接近前方和上一目标的方向；有限测距均值是次要评分。找不到可信通道就输出停止值。

这是一种保守的局部分析方法：图中的绿色包络是直线检查区域，不是车辆实际可执行的转弯轨迹。车体参数、雷达朝向、控制中位/符号均需现场标定。长段未知区域会导致停止，即使相机看起来有路；此时应核实无回波语义和其他观测，不能仅为让它继续走而把未知区域当作空旷。

## 构建与运行

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select process_node --symlink-install
source install/setup.bash
ros2 launch process_node process_node.launch.py use_sim_time:=true
```

另开终端回放：

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag play /data/data_whole/rosbag2_2026_10_06-18_13_20 --clock
```

launch 会读取 `config/baseline.yaml`，发布 `/process_node/control`。可用 `config:=/绝对路径/参数.yaml` 选择其他配置；更改参数后重新启动节点。也可以直接运行：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run process_node process_node --ros-args -r __ns:=/process_node \
  --params-file ~/ros2_ws/src/process_node/config/baseline.yaml -p use_sim_time:=true
```

纯终端运行时给 launch 加 `show_visualization:=false`，或给 `ros2 run` 加 `-p show_visualization:=false`。没有图形桌面时程序也会自动关闭显示。主入口与其他已有包的接口保持兼容。

## 如何读图和日志

- **Steering Visualization**：背景仍是翻转后的 RGB 输入。盘上指针表示相对中位的 steer 控制量；单独显示的 `target` 才是实际目标方向的角度。
- **Laser Diagnostics**：车头朝上，车体左侧在屏幕左侧；白点是真实有效测距（超过显示半径的点画在边缘），外圈红点只标记未知方向，不表示在那里测到障碍；橙色点是补齐的短缺口。
- 绿色弧线表示候选通道开口；绿色圆盘及平行线是被检查的车体扫掠包络；青色箭头表示最终目标。未知区域不会画成已知可通行区域。
- `valid` 是全帧有效比例，`support` 是选中方向窗口内的原始有效比例，`finite_mean` 是经过有限评分上限裁剪的有效距离均值。`front` 是已观测条件下的直行包络净空，`unknown` 表示证据不足。净空不是制动距离。
- `OK` 表示本基线选到了候选方向；`NO_OBSERVED_GAP` 表示没有观测充分且足够宽的开口；`NO_CLEAR_PATH` 表示开口存在，但支持比例或车体检查不合格；`NO_VALID_RANGES`/`INVALID_METADATA` 表示输入不可用，`TOO_CLOSE` 表示近于最小量程。后几种状态均输出停止值。
- `WAITING` 表示尚未收到数据，`STALE_DATA` 表示超过输入超时。暂停/播完后会显示缓存的末帧并撤去活动目标，独立定时器继续处理窗口事件。
- 按 q、Esc 或关闭任一窗口会发送停止值并退出节点，Ctrl+C 也可退出。输入超时还会周期性发送停止值；这些值的物理效果仍须在实车上确认。

## 主要参数

全部默认值见 `config/baseline.yaml`；直接 `ros2 run` 的内置默认值与其一致。以下默认值为离线调试起点。

| 参数 | 含义 |
|---|---|
| `sensor_yaw_offset_deg` | 将雷达坐标旋转到车体坐标的偏角，正值逆时针 |
| `vehicle_width_m`、`front_overhang_m`、`rear_overhang_m` | 车宽，以及雷达到前后端的距离；计算圆盘时取最远车角 |
| `safety_margin_m` | 车体包络外的额外裕量 |
| `lookahead_m`、`max_distance_m` | 需检验的前行距离、有限评分/显示上限 |
| `front_half_angle_deg`、`bin_angle_deg` | 前向搜索半角、分箱角宽 |
| `min_bin_valid_ratio`、`min_window_valid_ratio` | 分箱、候选方向窗口的最低原始有效比例 |
| `max_interpolate_gap_deg`、`interpolate_range_jump_m` | 允许补齐的最长短缺口及两端最大距离差；前者设 0 可禁用补齐 |
| `max_inferred_ratio` | 选中窗口内补齐分箱的最高比例 |
| `forward_weight`、`continuity_weight` | 前行倾向和目标连续性的评分权重 |
| `steer_center`、`steer_limit`、`steer_gain_per_rad` | 转向中位、相对中位的最大幅值、每弧度控制增益；与真实车轮角无直接等价关系 |
| `cruise_speed`、`stop_speed` | 有候选时的固定动力值、停止动力值，不是 m/s |
| `sensor_timeout_s`、`log_period_s` | 接收超时与普通日志周期，使用单调时钟；状态切换立即记录 |
| `group_label` | 小组/版本文字标记；图中建议使用英文或数字，OpenCV 默认字体不支持中文 |

## 验证

在工作区根目录加载环境后执行：

```bash
python3 -m pytest src/process_node/test/test_perception.py \
  src/process_node/test/test_node_baseline.py src/process_node/test/test_window_monitor.py -q
```

测试覆盖扫描点数和角度排列变化、数组首尾、非有限值、短缺口/长未知段、过窄开口、孤立近障碍、左右镜像、安装偏角、控制限幅、超时和窗口关闭。录制数据回放只验证输入解释与决策计算，不代表已经完成实车闭环绕障。

2026-10-20 使用默认参数按时间索引抽取 16 帧独立雷达数据（约 0–350 秒），5 帧选出候选方向，10 帧为 `NO_OBSERVED_GAP`，1 帧为 `NO_CLEAR_PATH`。例如约 120 秒选中 −40.5°、约 350 秒选中 −44.5°；开头因前向长缺测段输出停止值。它们是决策结果而非人工标注真值，也不是任务通过率。单帧感知耗时约 1–7 毫秒（不含绘图和 ROS 通信）。

另外在隔离的 ROS 命名空间/域中发送了一条实际 `AllSensor` 消息，观察到控制输出 `(212, -100)`；停止输入且 `/clock` 不推进时，独立定时器仍将其变为配置的停止输出 `(0, 0)`。

窗口兼容性：课程镜像使用的 OpenCV 4.5 GTK3 后端，在窗口正常打开时，`WND_PROP_VISIBLE` 也可能返回 −1，不能将它直接理解成“用户关闭了窗口”。`window_monitor.py` 对 GTK3 使用原生窗口列表（`python3-gi`）识别关闭，其他后端先确认窗口已出现，再判断关闭；不支持查询且缺少原生绑定时，不因未知返回值退出，仍可用 q/Esc、Ctrl+C 退出。

已在当前 GTK3 图形环境验证：没有输入且 ROS 时钟不推进时两个窗口持续显示；注入一帧实际 bag 消息后 RGB 与激光图更新；触发 GTK 原生窗口关闭事件后输出停止值并退出。相关自动化测试合计 42 项通过。
