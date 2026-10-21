# 激光方向跟踪与低速 PD/PI 控制

本阶段实现激光局部方向选择、独立里程计反馈、方向 PD 和前进速度 PI。`process_core(urg)` 仍返回 `(steer, speed)`。默认 `control_mode=closed_loop`，请求速度为 0.2 m/s；所有增益、控制符号和前馈参数仍是离线起点，尚未实车整定。90°转弯、S 弯、终点判断、走廊中线跟踪和实际转弯路径检查仍待实现。跨设备迁移见 [MIGRATION.md](MIGRATION.md)。

**新增行为：默认必须收到有效的独立 `/encoder_imu_node/odom`，且它与激光消息时间接近，才允许输出行驶指令。** 当前课程 bag 已包含该话题，完整回放命令不变。只有激光数据时，会输出停止值。

## 数据如何变成目标方向

1. `perception.py:prepare_scan` 保留原始下标，用本帧角度参数计算每个方向，并叠加可配置的雷达安装偏角。前方 0° 两侧的数组首尾会在角度空间连起来。
2. 有限、正数且处于量程内的测距是有效点；正无穷单独标记为无回波/未知，其他非法值归为无效。负无穷或正数但低于最小量程表示近障碍风险，直接停止，不能作为短缺口补齐。**不会把 inf 替换为最大距离参与评分。**
3. 按固定角宽分箱，每箱检查有效比例、保留最近有效点。只有两端都有真实观测、距离相近且角宽很小的缺口允许保守补齐，补齐值取两侧较小距离并单独标记。长缺测段仍是未知。
4. 在前向范围内寻找连续、有足够观测深度和宽度的通道。根据宽度、前后悬和裕量建立以雷达为中心的车体包络圆盘，沿候选直线做扫掠检查。所有有效障碍点都参加检查，包括低覆盖分箱里的近点。
5. 从合格候选中优先选择已测净空较大、方向较接近前方和上一目标的方向；有限测距均值是次要评分。找不到可信通道就输出停止值。

这是一种保守的局部分析方法：图中的绿色包络是直线检查区域，不是车辆实际可执行的转弯轨迹。车体参数、雷达朝向、控制中位/符号均需现场标定。长段未知区域会导致停止，即使相机看起来有路；此时应核实无回波语义和其他观测，不能仅为让它继续走而把未知区域当作空旷。

## 运动反馈与控制调用链

```text
独立 Odometry → motion.py：位置/姿态差分 → 带时间戳的运动状态缓存
                                                   ↓ 按激光消息时间关联
AllSensor.urg → analyze_scan → ControlReference + MotionFeedback
                                      ↓ ControlInput
                               PIDController（默认）
                                      ↓ ControlCommand
                               ROS Control 消息
```

`motion.py` 仅使用独立里程计的 `pose` 和 `header`，不使用 AllSensor 的运动字段，也不使用底层填写不正确的 `odom.twist.linear`。

- **速度**：将相邻位置差投影到这段运动的中间车头方向，再除以消息时间差。前进为正、倒退为负；`raw_speed_mps` 为原始差分，`speed_mps` 为低通滤波结果。估计值继承底层里程计的尺度误差和噪声，实车仍需核验。
- **航向**：从归一化四元数计算 yaw，按最短角差展开，跨 ±π 不产生整圈跳变；提供当前航向、累计航向变化和滤波角速度。
- **位置/行程**：保留 odom 位置；局部位置以本次起点的车头方向为 +x、左侧为 +y；累计路程倒退也增加，有符号行程倒退减少。这些量在时间回跳或异常重建起点后重新计数，尚不代表任务阶段完成。
- **时间**：差分和滤波使用消息时间，接收超时使用单调时钟。缓存最近 2 秒有效状态，关联与本帧激光时间最接近的状态，默认容差 0.15 秒；不外推、不等待未来消息、没有额外引入同步依赖。关联值表示差分区间终点的状态，不是精确插值到激光时刻的状态。
- **有效性**：至少两个具有足够时间间隔的有效位姿才产生速度；重复时间戳不刷新有效估计的接收时间。无效姿态/坐标系、时间回跳、过长间隔和运动跳变会阻止控制，必要时重建起点。坐标系必须符合配置，本模块不做 TF 变换。

`control.py` 中的接口均有字段注释：

| 对象 | 内容 |
|---|---|
| `ControlReference` | 激光时间、相对车头的目标方向误差、物理目标速度、停止请求及原因 |
| `ControlInput` | 参考目标和已检查有效性/时间关联的运动反馈 |
| `ControlCommand` | 底层 steer/speed 控制量、状态及输出原因 |

默认模式将物理目标速度填入 `ControlReference.target_speed_mps`，`pid_controller.py` 实现方向 PD 与速度 PI。`control_mode=baseline` 保留旧比例转向/固定动力输出供离线对照，此时目标速度字段仍为 None，`target_speed_mps` 参数不控制旧基线的动力值。

## 控制器如何工作

**方向 PD** 的误差是当前目标方向相对车头的角度（rad）。当前没有额外计算走廊横向位置误差：

```text
转向偏移 = steer_sign × (steer_kp × 方向误差 + steer_kd × 滤波后的误差变化率)
steer = steer_center + 经过限幅、变化率限制的转向偏移
```

误差微分使用控制消息时间差，一阶低通默认 0.15 秒。相邻误差跳变超过 20°，或 `ControlReference.reference_id` 改变时，D 项清零，随后重新计算；P 项仍受输出变化率限制。这里的误差变化率不是 IMU 角速度。

**速度 PI** 使用 `motion.py` 的有符号滤波速度（m/s）：

```text
内部目标速度：从零按 target_accel_mps2 逐步接近请求速度
速度误差 = 内部目标速度 - 实测前进速度
前进动力幅度 = 前馈偏置 + 前馈系数 × 内部目标速度 + Kp × 误差 + 积分项
speed = stop_speed + speed_sign × 经过限幅、变化率限制的前进动力幅度
```

本版只支持非负目标速度，动力幅度限制在 `[0, speed_effort_limit]`。超速时可降低动力至停止值，**不会自动发送倒车动力充当制动**。机械制动效果仍取决于底层对停止值的定义。积分项有独立限幅，并采用条件积分：若输出幅值或变化率受限，拒绝会继续加重限制的积分增量，允许反方向积分帮助恢复。

控制时间来自相邻 `urg.header.stamp`，不假设固定帧率。首个有效输入保持停止并报告 `CONTROL_WARMUP`，下一次足够间隔的消息才计算输出。过短间隔累计后计算；重复/回跳时间或过长间隔停止并清空历史。传感器失效、超时、无可行目标、零速请求及退出时，停止输出绕过普通平滑限制，清空积分、微分和内部目标速度。恢复后重新预热，不延续旧动力。参考任务切换清除 I/D 历史，正常输出仍受变化率限制。

两个参考模式可用于分阶段测试：

| `reference_mode` | 行为 |
|---|---|
| `perception`（默认） | 跟踪当前感知选出的目标方向，运行方向 PD 和速度 PI |
| `straight` | 仅接受感知检查合格的 0° 候选，没有则 `NO_STRAIGHT_PATH` 停车；方向误差为零，用于中位和速度 PI 的直路测试 |

`straight` 不锁定初始全局航向，也不计算走廊中线误差；若车辆因中位偏差逐渐偏航，它不会根据绝对航向纠偏。测试方向 PD 时使用 `perception`。两种模式均不能证明现有直线包络对应实际车辆转弯轨迹。

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

仅测试中位和低速前进 PI，可在加载环境后运行（离线回放用 true，实车用 false）：

```bash
ros2 run process_node process_node --ros-args -r __ns:=/process_node \
  --params-file ~/ros2_ws/src/process_node/config/baseline.yaml \
  -p use_sim_time:=true -p reference_mode:=straight -p target_speed_mps:=0.15
```

实际参数集中写入专用 YAML 后，可继续使用 `ros2 launch ... config:=/绝对路径/参数.yaml`。当前 launch 仅提供 `config`、`use_sim_time`、`show_visualization` 三个启动参数；控制模式和增益通过 YAML 或 `ros2 run` 的 ROS 参数传递。参数在启动时读取，修改后重新启动节点。

仅需查看激光感知、输入又不含独立里程计时，可以显式关闭反馈门控：

```bash
ros2 run process_node process_node --ros-args -r __ns:=/process_node \
  --params-file ~/ros2_ws/src/process_node/config/baseline.yaml \
  -p use_sim_time:=true -p control_mode:=baseline -p require_motion_feedback:=false
```

此组合只供离线观察；它仍会发布旧基线控制消息，不应连接实车驱动使用。闭环模式不允许关闭反馈检查，启动时会拒绝该配置。

## 有终端输出但没有图形窗口

窗口需要同时满足 `show_visualization=true` 和启动进程具有有效的桌面显示环境。VS Code、SSH 或新开的容器终端可能没有继承桌面的 `DISPLAY`；即使容器桌面正在运行，程序也会关闭可视化，并在启动日志中明确提示缺少 `DISPLAY/WAYLAND_DISPLAY`。这与控制器是否获得反馈、是否输出行驶指令无关；等待输入或停车时也应显示窗口。

在启动节点的终端检查：

```bash
echo "$DISPLAY"
```

通过 VS Code 连接容器、窗口由其显示转发到本机时，应使用 VS Code 自动提供的显示环境。VS Code 更新、重启或重新连接后，若转发异常，优先重新连接容器并新开终端；必要时重启容器，再按原命令运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch process_node process_node.launch.py use_sim_time:=true show_visualization:=true
```

显示编号可能随连接改变，不要将 `DISPLAY` 或 `XAUTHORITY` 固定写入 `~/.bashrc`，也不要用容器 VNC 的 `:1` 覆盖 VS Code 的本机转发地址。程序只读取这些变量，不会修改它们。若出现“窗口显示失败”而非“缺少显示环境”，应按日志核对显示连接、授权和 OpenCV GUI 依赖。

2026-10-21 窗口问题的记录：异常启动进程缺少显示环境变量；在容器 VNC `:1` 中进行的隔离检查确认 GTK 绘图和关闭处理可用，但该桌面不等于用户本机的显示转发目标。用户重启容器后，原启动命令恢复正常，无需手动 export；随后检查到当前会话自动提供了 `DISPLAY=:19`。此编号仅为当次观察，不作为配置值。

## 如何读图和日志

- **Steering Visualization**：背景仍是翻转后的 RGB 输入。盘上指针表示相对中位的 steer 控制量；单独显示的 `target` 才是实际目标方向的角度。
- **Laser Diagnostics**：车头朝上，车体左侧在屏幕左侧；白点是真实有效测距（超过显示半径的点画在边缘），外圈红点只标记未知方向，不表示在那里测到障碍；橙色点是补齐的短缺口。
- 绿色弧线表示候选通道开口；绿色圆盘及平行线是被检查的车体扫掠包络；青色箭头表示最终目标。未知区域不会画成已知可通行区域。
- `valid` 是全帧有效比例，`support` 是选中方向窗口内的原始有效比例，`finite_mean` 是经过有限评分上限裁剪的有效距离均值。`front` 是已观测条件下的直行包络净空，`unknown` 表示证据不足。净空不是制动距离。
- 节点 `OK` 表示感知与控制门控均通过；日志中的 `perception=OK` 仅表示选到了候选方向。`NO_OBSERVED_GAP` 表示没有观测充分且足够宽的开口；`NO_CLEAR_PATH` 表示开口存在，但支持比例或车体检查不合格；`NO_VALID_RANGES`/`INVALID_METADATA` 表示输入不可用，`TOO_CLOSE` 表示近于最小量程。后几种状态均输出停止值。
- `WAITING` 表示尚未收到数据，`STALE_DATA` 表示超过输入超时。暂停/播完后会显示缓存的末帧并撤去活动目标，独立定时器继续处理窗口事件。
- `odom=OK` 表示反馈可用；`WAITING_ODOM`/`INITIALIZING_ODOM` 表示等待输入或差分起点尚未就绪；`STALE_ODOM` 表示有效估计接收超时；`UNALIGNED_ODOM` 表示反馈与激光时间差过大；`INVALID_ODOM`、`DUPLICATE_ODOM`、`ODOM_TIME_REWIND`、`ODOM_TIME_GAP`、`ODOM_POSE_JUMP` 分别标记输入或差分连续性问题。默认门控开启时，不可用反馈会阻止行驶，定时器也会独立检测反馈超时。
- 新日志 `v/raw_v` 为滤波/原始物理速度（m/s），`yaw/turn` 为当前/累计航向（度），`travel/signed` 为累计路程/有符号行程（m），`local` 为局部位置（m），`dt` 为差分间隔，`skew` 为关联反馈时间减激光时间（s）。反馈无效时仍可能显示缓存值，但它们仅供诊断，不能用于控制。
- 闭环日志增加 `v_ref`（斜坡后目标）、`requested`（请求目标）、`v_used`（本次控制使用的关联速度）、`e_v`、动力 `[FF,P,I]`、转向 `[P,D]`、`control_dt`，以及幅值饱和 `sat`、变化率限制 `rate`、冻结积分 `I_frozen` 和撤去微分 `D_reset` 标记。激光图也显示目标/反馈速度及 P/I/D 项。
- `CONTROL_WARMUP` 为有效输入恢复后的首帧等待；`CONTROL_TIME_INVALID`/`CONTROL_TIME_GAP` 为控制时间不连续；`STOP_REQUESTED` 包括零目标速度。这些状态均停止。录制数据中车辆速度不受本次输出影响，实测速度高于斜坡目标时动力输出为停止值是可能的，不能据此判断实车调速效果。
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
| `control_mode` | 默认 closed_loop；baseline 用于旧固定动力基线对照 |
| `reference_mode`、`target_speed_mps` | perception/straight；请求的非负物理目标速度，默认 0.2 m/s，0 直接停车 |
| `steer_center`、`steer_limit`、`stop_speed` | 转向中位、偏移幅度上限、停止动力值，均为底层控制单位 |
| `steer_sign`、`steer_kp`、`steer_kd` | 转向符号 ±1、P/D 增益；误差用 rad，时间用 s |
| `steer_derivative_tau_s`、`steer_jump_reset_deg` | 微分低通时间常数/相邻误差跳变撤去 D 项的角度阈值 |
| `steer_rate_limit`、`speed_rate_limit` | 转向/动力控制量每秒最大正常变化量；内部保留浮点数，发布时四舍五入 |
| `speed_sign`、`speed_kp`、`speed_ki` | 前进动力符号 ±1、速度 P/I 增益；误差用 m/s，时间用 s |
| `speed_feedforward_gain`、`speed_feedforward_offset` | 动力线性前馈系数/偏置；必须实测，不能由当前无控制输出的 bag 拟合 |
| `speed_effort_limit`、`speed_integral_limit` | 前进动力幅度上限/积分项绝对上限，单位均为底层控制单位 |
| `target_accel_mps2`、`target_decel_mps2` | 内部目标速度的正常加/减速上限，m/s²；请求零速仍直接停止 |
| `max_target_speed_mps` | 接口允许的最高物理目标速度，默认 0.5 m/s，超范围拒绝运行 |
| `control_min_dt_s`、`control_max_dt_s` | 控制消息计算间隔，默认 0.01/0.5 秒 |
| `steer_gain_per_rad`、`cruise_speed` | 仅 baseline 模式使用的比例转向增益和固定动力值 |
| `require_motion_feedback` | 默认 true；closed_loop 强制开启，baseline 可关闭供离线观察 |
| `odom_topic` | 独立里程计输入话题，默认 `/encoder_imu_node/odom` |
| `odom_frame_id`、`odom_child_frame_id` | 预期坐标系，默认 `odom`、`base_link`；不执行坐标变换 |
| `odom_min_dt_s`、`odom_max_dt_s` | 差分允许的最短/最长消息间隔，默认 0.01/0.5 秒 |
| `odom_filter_tau_s` | 速度与角速度低通时间常数，默认 0.15 秒；0 禁用滤波 |
| `odom_max_speed_mps`、`odom_max_yaw_rate_rps` | 运动跳变检查阈值，默认 3 m/s、6 rad/s；不是目标速度或指令限幅 |
| `odom_timeout_s` | 有效反馈的单调时钟接收超时，默认 0.5 秒 |
| `odom_scan_tolerance_s`、`odom_history_s` | 时间关联容差/历史长度，默认 0.15/2 秒 |
| `sensor_timeout_s`、`log_period_s` | 接收超时与普通日志周期，使用单调时钟；状态切换立即记录 |
| `group_label` | 小组/版本文字标记；图中建议使用英文或数字，OpenCV 默认字体不支持中文 |

## 验证

在工作区根目录加载环境后执行：

```bash
python3 -m pytest src/process_node/test/test_perception.py \
  src/process_node/test/test_motion.py src/process_node/test/test_control.py \
  src/process_node/test/test_pid_controller.py src/process_node/test/test_node_closed_loop.py \
  src/process_node/test/test_node_baseline.py src/process_node/test/test_window_monitor.py -q
```

测试覆盖扫描点数和角度排列变化、数组首尾、非有限值、短缺口/长未知段、过窄开口、孤立近障碍、左右镜像、安装偏角、运动反馈单位/符号、航向接缝、时间对齐、反馈失效停车、控制限幅、超时和窗口关闭。录制数据回放只验证输入解释与决策计算，不代表已经完成实车闭环绕障。

2026-10-20 使用默认参数按时间索引抽取 16 帧独立雷达数据（约 0–350 秒），5 帧选出候选方向，10 帧为 `NO_OBSERVED_GAP`，1 帧为 `NO_CLEAR_PATH`。例如约 120 秒选中 −40.5°、约 350 秒选中 −44.5°；开头因前向长缺测段输出停止值。它们是决策结果而非人工标注真值，也不是任务通过率。单帧感知耗时约 1–7 毫秒（不含绘图和 ROS 通信）。

此前在隔离的 ROS 命名空间/域中发送了一条实际 `AllSensor` 消息，观察到控制输出 `(212, -100)`；停止输入且 `/clock` 不推进时，独立定时器仍将其变为配置的停止输出 `(0, 0)`。这是引入反馈门控前的单帧验证；当前默认配置还需要同时输入独立里程计。

2026-10-21 按时间索引读取 bag 约 1、60、120 秒处的三帧 AllSensor，并分别读取其前 0.5 秒内的 25 条独立里程计，读取消息体合计约 4.7 MB。每段均在起点初始化后得到 24 次有效差分；关联时间差分别约 −4.02、−0.11、−2.58 ms，滤波速度分别约 −0.0014、0.3535、0.2916 m/s。这是小样本接口检查，不是整段里程计精度结论，也没有用录制数据整定控制器。

在隔离 ROS 域 222、命名空间 `/interface_test` 中，通过真实 ROS 发布/订阅输入约 120 秒片段：只有 AllSensor 时保持 `WAITING_ODOM` 和停止输出；加入独立里程计后得到 `(212, -100)`，关联差约 −2.58 ms；继续发送激光但停止里程计后进入 `STALE_ODOM`，输出 `(0, 0)`。整个检查中模拟时钟保持 0，验证反馈超时不依赖 `/clock` 前进。

反馈接口阶段在课程容器中构建成功，五个测试文件合计 77 项通过；上述 `(212, -100)` 是旧基线输出。当前 PD/PI 的输出由消息序列、斜坡和速度误差共同决定，不再是固定动力。尚未在实车或助教设备上核验反馈精度与控制响应。

PD/PI 阶段在课程容器中构建成功，上述七个测试文件合计 **115 项通过**。新增检查包括 P/D 和 P/I 数值、消息时间、微分滤波、目标跳变、参考切换、积分和输出饱和、变化率限制、停车与恢复。一个明确假设的一阶速度/转向模型用于检查负反馈方向和数值收敛，它不是课程车辆动力学模型，不能用于实车整定。

另外在隔离 ROS 域 224、命名空间 `/pid_test` 中加载安装后的 `baseline.yaml`，按时间索引读取约 120–121 秒的 12 帧 `/scan` 和 50 条独立里程计（共 196,912 字节）；将真实扫描填入 AllSensor.urg，经 ROS 发布/订阅输入节点。观察到等待反馈、首帧预热及 10 次正常计算，末次输出 `(220, -16)`，内部目标约 0.167 m/s、关联测量约 0.325 m/s。随后停止输入，反馈超时输出停止值；恢复后首帧预热，下一帧内部目标重新从 0.02 m/s 开始。模拟时钟全程为 0，独立超时仍有效。这是消息链路及控制计算检查，不是车辆已响应这些指令的证据。

上机调试顺序建议：先在手动模式核验控制符号、中位、停止值、速度单位与反馈；测定前进动力死区及粗略前馈关系；再在空旷直路低速测试 PI，逐步调 Kp/Ki；随后以小方向误差测试方向 P，再加入 D。若不动，先核对前馈和死区，不应仅靠增大积分强行启动。每次保存参数版本并录制传感器、独立里程计和控制输出。

窗口兼容性：课程镜像使用的 OpenCV 4.5 GTK3 后端，在窗口正常打开时，`WND_PROP_VISIBLE` 也可能返回 −1，不能将它直接理解成“用户关闭了窗口”。`window_monitor.py` 对 GTK3 使用原生窗口列表（`python3-gi`）识别关闭，其他后端先确认窗口已出现，再判断关闭；不支持查询且缺少原生绑定时，不因未知返回值退出，仍可用 q/Esc、Ctrl+C 退出。

已在当前 GTK3 图形环境验证：没有输入且 ROS 时钟不推进时两个窗口持续显示；注入一帧实际 bag 消息后 RGB 与激光图更新；触发 GTK 原生窗口关闭事件后输出停止值并退出。相关自动化测试合计 42 项通过。
