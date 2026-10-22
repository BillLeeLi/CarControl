# 迁移到助教设备与实车运行

适用于课程提供的 ROS 2 Humble 环境。目标是将当前源码迁移到另一台设备，在目标环境重新编译并核对运行接口。包的日常运行、参数和图例说明见 [README.md](README.md)。

本文的工作区路径统一写为 `~/ros2_ws`；助教设备路径不同时，替换为其实际工作区。若 ROS 运行在 Docker 中，目标端的解压、构建和运行命令均在对应容器内执行。

## 1. 在自己的设备上打包

在源码所在的容器或 Linux 环境中执行：

```bash
cd ~/ros2_ws/src
tar --exclude='__pycache__' --exclude='*.pyc' \
    --exclude='build' --exclude='install' --exclude='log' \
    -czf ~/process_node_transfer.tar.gz process_node
```

打包整个 `process_node/`，其中包含 Python 模块、`launch/`、`config/`、`package.xml`、`setup.py`、`setup.cfg`、`resource/`、测试和本文。当前仅需迁移这个包；课程原有消息包和驱动由目标环境提供。

- 该方式包含尚未提交 Git 的修改。若改用 Git 迁移，必须先确认需要的文件已经提交；未提交的修改不会随克隆迁移。
- 不迁移 `build/`、`install/`、`log/`，在目标环境重新生成。
- 大型 bag 不属于运行代码必需文件；只有目标端需要离线回放时才另行复制。
- 后续若新增自己的包，应同步更新本文的打包清单及构建命令。

## 2. 复制并放入目标工作区

通过 U 盘、共享目录或网络传输压缩包。以下约定目标 ROS 环境中能通过 `/tmp/process_node_transfer.tar.gz` 访问它。

如果目标使用 Docker，可以在目标宿主机执行以下命令，将压缩包送入容器；替换实际文件路径和容器名：

```bash
docker cp <压缩包路径> <容器名>:/tmp/process_node_transfer.tar.gz
```

先确认助教整车软件使用的工作区。若 `~/ros2_ws/src/process_node` 已存在，在目标 ROS 环境中将旧包备份到工作区外：

```bash
mkdir -p ~/course_backups
mv ~/ros2_ws/src/process_node \
   ~/course_backups/process_node_$(date +%Y%m%d_%H%M%S)
```

旧包不存在时跳过上面的备份命令。然后解压：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
tar -xzf /tmp/process_node_transfer.tar.gz
```

解压后的结构应为：

```text
~/ros2_ws/src/process_node/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
├── config/baseline.yaml
├── launch/process_node.launch.py
└── process_node/
    ├── process_node.py
    ├── perception.py
    ├── motion.py
    ├── control.py
    ├── pid_controller.py
    ├── visualization.py
    └── window_monitor.py
```

不要解压成 `src/process_node/process_node/package.xml`，也不要把旧包备份留在 `src/` 下：构建工具会递归查找包，两个 `process_node` 会产生同名包冲突。保留目标设备原有的其他包和设备配置。

## 3. 核对依赖并构建

以下命令在目标设备的 ROS 环境中执行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
# 工作区已经构建过时，加载原有课程包。
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

ros2 pkg prefix robotsdk_msgs
colcon build --packages-select process_node --symlink-install
source install/setup.bash
ros2 pkg prefix process_node
```

`robotsdk_msgs` 是课程自定义消息包。选择性构建以目标环境已经安装、加载它及其他依赖为前提。如果找不到它，但目标工作区中已有其源码，可用下面的命令构建所需的工作区依赖，再加载环境：

```bash
colcon build --packages-up-to process_node --symlink-install
source install/setup.bash
```

如果消息包来自另一个课程工作区，应先加载那个工作区的 `install/setup.bash`。不要自行改写消息定义来绕过缺包问题。

当前依赖已在 `package.xml` 中声明，包括 `rclpy`、`sensor_msgs`、**`nav_msgs`（独立里程计）**、`geometry_msgs`、`std_msgs`、`robotsdk_msgs`、`cv_bridge`、NumPy、OpenCV、`python3-gi` 及 launch 相关包。相同镜像通常已具备这些依赖；缺失时根据报错补齐。GTK3 窗口关闭识别使用系统包 `python3-gi`。

构建完成后，`ros2 pkg prefix process_node` 应指向当前目标工作区的安装目录。若指向其他工作区，在新终端中按“ROS Humble → 课程依赖工作区（如有）→ 当前工作区”的顺序重新加载环境。

## 4. 不启动控制节点，先验证导入

```bash
python3 -c "import rclpy, numpy, cv2, cv_bridge; from nav_msgs.msg import Odometry; from robotsdk_msgs.msg import AllSensor, Control; from process_node.process_node import LaserWindowAnalyzer; print('导入成功')"
```

该命令不会创建控制节点或发布控制消息，可发现主要 Python 模块、消息包或环境加载问题。它不验证设备连接和车辆运动。

需要运行现有自动化测试时，在 `~/ros2_ws` 中执行：

```bash
python3 -m pytest src/process_node/test/test_perception.py \
  src/process_node/test/test_motion.py src/process_node/test/test_control.py \
  src/process_node/test/test_pid_controller.py src/process_node/test/test_node_closed_loop.py \
  src/process_node/test/test_node_baseline.py \
  src/process_node/test/test_window_monitor.py -q
```

测试需要 `pytest`；现有测试通过也不能证明实车绕障成功。

## 5. 实车启动

先由助教确认整车入口。课程提供的命令为：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch poss_cpp poss_launch.py
```

`poss_cpp` 不在当前仓库内，由实车环境提供。确认整车启动文件是否已经启动你的 `process_node`，避免重复运行控制节点。需要单独启动时，在另一个已加载环境的终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch process_node process_node.launch.py use_sim_time:=false
```

没有图形桌面时使用：

```bash
ros2 launch process_node process_node.launch.py \
    use_sim_time:=false show_visualization:=false
```

如果桌面存在但两个窗口都不出现，检查启动终端的 `echo "$DISPLAY"` 和启动日志。程序要求 `show_visualization=true` 且进程有有效显示连接。使用 VS Code 容器显示转发时，环境由 VS Code 自动提供；更新或重连后异常，优先重新连接并新开终端，必要时重启容器。2026-10-21 用户重启容器后已恢复，无需手动 export。不要将容器 VNC 的 `:1` 当成本机转发目标，也不要把临时显示编号或授权路径固定写入 `~/.bashrc`。助教设备使用其实际桌面/转发环境；详细记录见 README 的“有终端输出但没有图形窗口”。

| 场景 | `use_sim_time` | 输入来源 |
|---|---|---|
| 实车 | `false` | 实时传感器及同步节点，无需 `ros2 bag play` |
| 离线回放 | `true` | `ros2 bag play <数据包路径> --clock` |

launch 默认加载随包安装的 `config/baseline.yaml`，并将控制话题放在 `/process_node/control`。使用现场专用配置时，可指定 `config:=/绝对路径/参数.yaml`。修改配置后重新启动节点；修改依赖、安装规则或启动文件后重新构建并加载环境。

## 6. 上车前核对

按课程要求保持遥控器 ON（手动模式），车辆置于地面空旷处，再核对：

```bash
ros2 topic info /sync_store_node/allsensor --verbose
ros2 topic info /encoder_imu_node/odom --verbose
ros2 topic info /process_node/control --verbose
```

确认输入话题有传感器发布端和本节点订阅端；控制话题只有预期的控制发布端，并有底层驱动订阅。话题存在不代表数据持续到达，还应结合本节点日志确认接收与分析正常。实际传感器话题不同，可通过配置中的 `sensorTopic` 适配。

**当前默认要求独立里程计反馈有效**：话题通过 `odom_topic` 配置，坐标系默认 `odom` → `base_link`（对应 `odom_frame_id`、`odom_child_frame_id`，本节点不做 TF 变换）。节点用位置/姿态和消息时间估计速度与航向，不使用底层有问题的 `twist.linear`。需至少两个有效位姿，默认差分间隔 0.01–0.5 秒、有效反馈接收超时 0.5 秒、与激光关联时间差不超过 0.15 秒。缺失、异常、超时或不能对齐的反馈均会输出停止值；从日志 `odom`、`v/raw_v`、`dt`、`skew` 检查具体原因。

时间阈值、滤波和跳变检查参数见 `config/baseline.yaml` 及 README；完整 bag 回放需保留独立里程计话题。

当前版本标签为 `midterm-corridor-v3.1`，默认 `control_mode=closed_loop`、`reference_mode=corridor`，由走廊走向和横向偏移生成目标，再执行方向 PD 与速度 PI。正常物理请求速度 `target_speed_mps=0.2`，单墙、历史保持或较大偏差时默认减半。先连续 3 帧确认双墙，再由控制器预热一帧。停止、传感器/反馈失效或时间异常会清空相应控制历史，超时/异常也会清除走廊参考。corridor 仅支持 closed_loop 和有效独立反馈；仅离线观察激光时需同时设置 `reference_mode=perception`、`control_mode=baseline`、`require_motion_feedback=false`，仍不能连接实车驱动使用。

首次中位/速度 PI 测试可用 `reference_mode=straight` 和较低的 `target_speed_mps`。straight 只接受合格的 0°候选，不锁定全局航向。完成基础标定后，以 `reference_mode=corridor` 在直走廊低速检查回正与回中；`reference_mode=perception` 保留上一版局部评分供离线对照。可用 `ros2 run` 的 ROS 参数覆盖，具体命令见 README。launch 的启动参数仍是 config、use_sim_time、show_visualization，其他参数通过 YAML 配置。

现场按实际车辆核对以下配置：

- 雷达安装偏角、车宽、雷达到车体前后端的距离及安全裕量。
- `+inf` 继续按受限无回波参与规划，`no_return_distance_m` 默认 5 m，实际受 `LaserScan.range_max` 限制；设 0 可禁用。先核对目标雷达驱动的 `+inf` 语义及有效探测范围，不把蓝色规划标记当成真实回波。`min_scan_finite_ratio` 默认 0.05，全无有限回波仍停止。`min_bin_valid_ratio`/`min_window_valid_ratio` 指有限回波与受限无回波的合计可用比例。
- 新增 `corridor.py` 及整套 `corridor_*` 参数，必须连同新 YAML 迁移。`corridor_sensor_x_m/y_m` 是雷达相对 odom 子坐标系原点的安装平移，默认 0；雷达偏航仍由原 `sensor_yaw_offset_deg` 设置。核验走廊方向 `axis`、中线在左为正的 `center_left`、观测宽度 `width`，确保安装偏移不会被误当成车辆偏离中线。
- 墙线拟合范围、网格、支持点数/跨度/误差、平行与宽度约束、历史跳变门槛及失效期限，见 README 的走廊参数表。墙线只使用真实回波；离群点只在拟合中排除，仍参与碰撞检查。单墙依赖近期双墙宽度，最多 0.6 s；双墙均缺失时默认最多保持 0.3 s，过期后重新确认。
- 里程计坐标系和消息时间；低速前进的 `v` 应为正、倒退为负，静止应接近零，航向及位置尺度需核验。
- 转向中位、正负方向、幅度、增益，以及动力值和停止值；`steer` 和 `speed` 均是底层控制量，不直接等于转角和 m/s。
- `steer_sign`、`speed_sign`、前进动力死区和前馈系数/偏置，随后标定 PD/PI 增益；`speed_effort_limit`、变化率限制和物理目标速度上限需适配具体车辆。默认增益没有实车验证。
- 控制话题与底层驱动一致；切换自动前由助教确认车辆状态，临近碰撞立即切回手动。

当前已实现走廊墙线/中线参考、里程计平移/转动补偿、限时单墙/历史保持和降速，以及方向 PD、前进速度 PI、目标速度斜坡、限幅、变化率限制及条件积分抗饱和。零速和保护停止直接覆盖普通输出。动力仅支持前进幅度，不自动倒车制动。90°转弯行为、S 弯行为、终点判断和实际弯曲轨迹检查尚未实现；到路口失去直走廊参考可能停车，本轮没有自动转弯接管。默认参数供离线调试，直走廊稳定回中与增益均需实车验证。

候选继续覆盖前向和 0°，corridor 还加入精确巡航参考角，并仅在其附近默认 ±5°选取合格方向；没有就 `CRUISE_BLOCKED` 停止。绿色开口仅供诊断，可能出现 `gaps=0` 但有合格目标；蓝点是无回波上限，红点是其他无效/未知。棕色墙线和紫色中线/参考是模型，青色才是获准执行的目标；参考出现不表示控制已获准。日志增加 corridor 状态、confidence、axis、center_left、width、观测/双墙年龄和降速比例。模型保存到 odom 坐标系，先补偿车体平移与转动再融合。

输入调度修复新增 `input_buffer.py`，迁移时必须随包复制。ROS 独立线程只接收消息；主线程先批量更新里程计，再处理最新扫描、控制和 GTK 窗口。默认 `input_odom_queue_size=200`、`visualization_period_s=0.1`，GUI 最多 10 Hz 且不重复传输同一画面；启动日志应显示“输入独立接收”。接收时间不因处理延迟刷新，发布前再次检查新鲜度，缓存溢出重建反馈；`odom_scan_tolerance_s` 保持 0.15 s。没有新增第三方依赖，构建和启动命令不变。

## 7. 常见问题

| 现象 | 排查方向 |
|---|---|
| `Package 'process_node' not found` | 检查构建是否成功；在当前终端加载目标工作区的 `install/setup.bash` |
| 找不到 `robotsdk_msgs` 或导入消息失败 | 加载课程依赖环境；有源码时构建依赖包 |
| 同名包冲突 | 把旧包备份移出工作区 `src/`，检查是否多套源码重复放置 |
| 运行的似乎仍是旧版本 | 检查 `ros2 pkg prefix process_node`、环境加载顺序，并退出旧进程后重新启动 |
| 一直 `WAITING` 或频繁 `STALE_DATA` | 检查整车驱动、输入话题、数据是否到达及跨容器通信环境 |
| `WAITING_ODOM`/`INITIALIZING_ODOM` | 检查独立里程计话题和足够间隔的有效位姿；AllSensor 内的里程计不能替代它 |
| `STALE_ODOM`/`UNALIGNED_ODOM` | 检查反馈接收、消息时间源和与激光的时间差，实车 use_sim_time=false；GUI 开启后才频繁出现时确认已迁移 input_buffer.py 与新版 main，观察 input_skipped_scans，不先放宽时间容差 |
| `ODOM_QUEUE_OVERFLOW` | 接收缓存溢出，节点会停止并重建反馈；检查主线程长阻塞或系统负载，勿无限增大缓存 |
| `INVALID_ODOM`/`ODOM_POSE_JUMP` | 查看日志原因，核对坐标系、姿态、位置尺度和运动上限参数 |
| `CONTROL_WARMUP` | 等待下一次有效控制消息；若一直出现，检查是否反复触发保护或时间异常 |
| `CONTROL_TIME_INVALID`/`CONTROL_TIME_GAP` | 检查激光时间戳是否重复、回跳或间隔过长；控制器已停止并清空历史 |
| `NO_STRAIGHT_PATH` | straight 测试没有合格的 0°候选；不应通过强制前进绕过感知检查 |
| `CORRIDOR_ACQUIRING`/`CORRIDOR_LOST` | 检查真实墙线是否有足够跨度、双墙是否平行及宽度是否合理；初始化需连续确认，不能仅增大保持时间让车盲走 |
| `CORRIDOR_SINGLE_WALL`/`CORRIDOR_HOLD` | 参考正在限时降速；观察缺失侧、观测年龄、双墙年龄，过期后停止是预期行为 |
| `CRUISE_BLOCKED` | 有其他候选但参考附近不可通行；本轮没有自动绕障接管，不应改成直接追逐侧向开口 |
| `CORRIDOR_OUT_OF_RANGE`/`CORRIDOR_TIME_INVALID` | 巡航纠偏角超出范围，或激光时间不连续；核对安装、位姿与扫描时间，不直接取消限幅 |
| `INSUFFICIENT_FINITE_SUPPORT`/`NO_VALID_RANGES` | 检查真实有限回波是否过少；即使前方允许无回波，整帧异常仍停止 |
| `NO_CLEAR_PATH` | 检查真实近点、车体包络、前瞻与无回波上限；不直接删除近点或盲目缩小车体参数 |
| PI 输出较小、车辆不启动 | 核对控制符号、前馈和起动死区；增益、积分上限和斜坡见 README，不盲目加大积分 |
| 有输出但底层收不到 | 检查是否发布到了 `/control`；优先使用本文 launch，核对 `/process_node/control` 的端点 |
| 有输出但窗口都不出现 | 检查 show_visualization 与 DISPLAY；VS Code 更新/重连后可重新连接或重启容器恢复转发，勿写死显示地址 |

本文步骤依据当前仓库配置整理。此前 PD/PI 版本通过 115 项测试，并用约 1 秒真实雷达/里程计数据验证 ROS 链路、控制计算及超时恢复。2026-10-22 的无回波版本已构建成功并通过 **135 项测试**；对 bag 开头 30 秒逐条运行实际节点回调：359 帧中 328 帧正常控制、3 帧预热、28 帧因真实近点进入包络而停止，331 帧包含 0°候选；不是实车通过率。尚未在助教设备或实车验证。本阶段没有新增第三方运行依赖，打包和启动命令不变；新增小型测距测试样本随整个包迁移。首次迁移后应记录实际工作区、依赖差异及可用的整车启动方式，并据此更新本文。

走廊参考版本已构建并通过 **174 项测试**，同一 30 秒得到 356 帧双墙参考、1 帧单墙、2 帧初始确认；最终 326 帧正常控制、3 帧控制预热、2 帧走廊确认、28 帧原近点包络停止。10–30 秒均正常计算，并产生有符号的回中修正。已在隔离 ROS 域用安装后的 launch/YAML 与真实 bag play 检查运行、反馈未对齐时停止并重新确认、播放停止后的超时停车和正常退出。新增 `corridor.py` 随包自动安装，RANSAC/最小二乘只用已有 NumPy，无新增第三方运行依赖；现场仍需标定安装偏移、目标点距离、横向增益、降速门槛和 PD/PI。

v3.1 调度修复通过 **181 项测试**。在用户相同的 VS Code/X11 图形环境中开启真实窗口回放约 30 s，316 帧反馈全部有效、无未对齐，关联差约 ±13 ms；10–30 秒处理的 211 帧均正常控制。主动替换 35 个中间扫描以处理最新观测，播放停止后超时停车并正常退出。此前只关闭 GUI 做实时验证未覆盖图形转发负载，本次补充了该条件；以上仍非实车结果。
