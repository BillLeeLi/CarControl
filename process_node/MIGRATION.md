# U 盘迁移、编译与运行

2026-10-23 按用户向助教确认的方式整理：**把整个 `process_node` 文件夹用 U 盘复制到小车系统，在小车上编译，然后启动本包。** 现场按第 1–4 节操作即可；环境检查、离线回放和自动化测试均放在文档最后，便于之后在自己的干净容器中复制执行。

本文以目标工作区 `~/ros2_ws` 为例。小车登录用户名可能不是 ubuntu，`~` 表示该用户的家目录；实际工作区不同就替换路径。若 ROS 在容器中，以下目录和终端均指运行 ROS 的容器，不能只复制到宿主机的同名目录。

## 1. 用 U 盘复制整个目录

源目录是：

```text
/home/ubuntu/ros2_ws/src/process_node
```

用文件管理器把整个文件夹放到 U 盘，再复制到小车工作区的 `src` 下，得到：

```text
~/ros2_ws/src/process_node/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
├── config/baseline.yaml
├── launch/process_node.launch.py
├── process_node/                  # Python 模块目录，里面有 __init__.py
│   ├── process_node.py
│   ├── perception.py
│   ├── corridor.py
│   ├── exits.py
│   ├── turning.py
│   ├── recovery.py
│   ├── input_buffer.py
│   ├── motion.py
│   ├── control.py
│   ├── pid_controller.py
│   ├── visualization.py
│   └── window_monitor.py
├── tools/
├── test/
├── README.md
├── TUNING.md
└── MIGRATION.md
```

复制时注意：

- `package.xml` 应直接在 `src/process_node/` 下；不要多套一层同名文件夹。
- 小车上已有旧的 `process_node` 时，先停止旧控制节点，把旧文件夹移到工作区外备份，再放入新文件夹。不要仅改名为 `process_node_old` 留在 `src` 内，否则仍可能被识别为同名包。
- 复制全部源码、配置和文档，包含未提交 Git 的修改；不需要压缩、解压或 Git 克隆。
- 不搬开发电脑工作区的 `build/`、`install/`、`log/`，这些在目标设备生成。bag 不属于运行本包所需源码。
- 保留小车上其他课程包、硬件配置和已有依赖；本次只替换 `process_node`。

## 2. 首次复制后编译

在小车的 **Bash 终端**依次输入：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
colcon build --packages-select process_node --symlink-install
source install/setup.bash
```

第三行加载该工作区原有的课程消息包；没有 `install/setup.bash` 时会跳过。若课程依赖安装在另一个工作区，还需先加载那个工作区的环境，见文末补充。

看到 `Finished <<< process_node` 且构建成功后，在**同一个终端**直接执行下一节的一行启动命令。构建报错时先排查，不继续运行旧的安装版本。

## 3. 启动与停止：现场只用这一行

编译完成且环境已加载后：

```bash
ros2 launch process_node process_node.launch.py
```

本包的 launch 默认已经设置好：

| 项目 | 默认行为 |
|---|---|
| 实车时间 | `use_sim_time=false`，现场不必额外输入 |
| 参数文件 | 当前安装的 `config/baseline.yaml` |
| 控制话题 | `/process_node/control` |
| 图形窗口 | 默认开启；没有 DISPLAY/WAYLAND_DISPLAY 时自动转为终端模式 |
| 节点名称 | `/process_node/process_node` |

**每次打开新终端**，先加载环境再启动：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch process_node process_node.launch.py
```

按 `Ctrl+C` 停止；有窗口时也可按 q、Esc 或关闭窗口退出。节点会尝试发送配置的停止值。按课程操作要求，在地面空旷处保持手动模式准备，确认设备和停止方式后再切自动；停止控制节点不代替遥控器接管。

### 启动前需要小车系统提供什么

**不需要沿用课件的组合启动命令。** 上面一行启动的是我们的控制节点，不会自动启动相机、雷达、里程计和串口底层驱动。由助教确认小车系统已经启动这些服务，或按目标设备的实际方式启动它们，并确保没有另一个控制节点同时发指令。

本包默认需要：

| 接口 | 用途 |
|---|---|
| `/sync_store_node/allsensor` | 雷达及相机输入 |
| `/encoder_imu_node/odom` | 独立位置/姿态反馈，用于估速和运动补偿 |
| 底层节点订阅 `/process_node/control` | 将输出传给控制板 |

实际接口不同，在 `baseline.yaml` 中修改 `sensorTopic`、`odom_topic`，并核对控制端点。只启动本包而没有传感器/反馈时，会停在 WAITING、WAITING_ODOM 等状态，不会让车辆自行运动。实车不需要 `ros2 bag play`。

## 4. 修改参数或代码后怎么重新运行

先切回手动、停止当前控制节点，再编辑文件。

### 修改 Python、launch、依赖或重新复制源码

在工作区根目录重新编译并启动：

```bash
cd ~/ros2_ws
colcon build --packages-select process_node --symlink-install
source install/setup.bash
ros2 launch process_node process_node.launch.py
```

以上假定该终端已加载 ROS 环境；新终端先按第 3 节加载环境。

### 只修改 YAML 参数

编辑 `~/ros2_ws/src/process_node/config/baseline.yaml`。要继续使用第 3 节的短启动命令，按上一段重新构建，确保安装位置的配置更新，然后重启节点。不要假定所有环境都会自动同步安装目录中的 YAML。

也可以直接加载源码配置，**无需重新编译**，但启动指令会稍长；以下必须在 `~/ros2_ws` 中执行：

```bash
ros2 launch process_node process_node.launch.py config:=src/process_node/config/baseline.yaml
```

两种方式都在启动时读取参数；保存文件后，正在运行的节点不会自动应用新值。

现场核心参数的含义、调试顺序、调整方向和故障判断见 [TUNING.md](TUNING.md)。其中数值以当前 YAML 为准，特别注意当前 `steer_kp=3000.0` 与 Python 内置默认值 `300.0` 不同；本包 launch 会加载 YAML。

### 当前配置中需要记住的内容

- 当前标签 `midterm-recovery-v6`，默认 `closed_loop + corridor`，巡航目标 0.2 m/s，右转上限 0.1 m/s，局部恢复默认开启、上限 0.06 m/s。首次直路标定可设 `turn_enabled: false`、`recovery_enabled: false` 和较低目标速度；中位/速度测试用 `reference_mode: straight`。
- 实测尺寸为宽 0.22 m、雷达到车头 0.16 m、到车尾 0.26 m，安全裕量保持 0.10 m。旧的现场专用 YAML 可能仍覆盖为旧尺寸，应核对。
- 前后距离统一相对雷达定义，转弯矩形按 `corridor_sensor_x_m/y_m` 转到 base_link；实际安装平移仍未确认。控制中位、符号、增益、动力死区、最小转弯半径和停车能力仍需实车标定，不能由车体尺寸推定。
- 当前能执行走廊巡航和单个右转；左转、短 Z 衔接、S 弯绕障和终点判断未完成。具体参数和图例见 [README.md](README.md)。
- `TURN_BLOCKED` 先停车；参考/旧路径问题可进入局部恢复，经停车、低速反馈和安全短弧连续确认后才动作。图中 `RECOVERY/阶段` 和日志 `recovery/cause/reason` 表示恢复正在仲裁；原 `turn_status=TURN_FAULT` 不一定仍要求永久停车，应结合恢复结果看。
- `RECOVERY_NO_GUIDE` / `RECOVERY_BLOCKED` 表示缺少可信方向或安全短弧，继续停车。`RECOVERY_COMPLETE` 连续确认交回巡航；不可恢复的 `TURN_FAULT` 和 `RECOVERY_FAULT` 仍需记录原因、人工检查后重启。不要通过放宽时间/路径门控解除故障。`recovery_enabled: false` 可关闭新增行为，其他启动命令不变。

## 5. 依赖前提和干净容器验证范围

**只拷贝 process_node 的前提，是目标系统已有课程消息包和硬件运行环境。** 本包不是完整整车软件包。

| 依赖 | 来源/说明 |
|---|---|
| ROS 2 Humble、colcon | 课程系统或同一课程镜像 |
| `robotsdk_msgs` | 课程自定义消息包，必须已安装并加载，或另有源码可构建；不包含在本次拷贝目录中 |
| `rclpy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`std_msgs`、`cv_bridge`、launch 相关包 | 当前 package.xml 已声明 |
| NumPy、OpenCV、`python3-gi` | 图像/数值处理及 GTK 窗口关闭识别，依赖已声明 |
| 设备驱动与同步/底层节点 | 小车系统提供，普通干净容器不会凭空具备实车设备 |

之后建议用**同一课程镜像新建的容器**，只复制 `process_node` 后按第 2–3 节验证，观察是否存在未声明或未加载的依赖。通用 `ros:humble` 镜像不等于课程镜像；缺少 `robotsdk_msgs` 时不能仅靠这个目录完成迁移。

验证可以分两层：

1. 编译、导入、launch、无输入等待及正常退出：不需要小车或 bag；无输入停车是预期结果。
2. 接收传感器、计算控制、窗口更新及超时停车：需要另外提供课程 bag 或实车输入。干净容器的结果不能代替车辆控制响应标定。

已有本地验证：当前 v6 已构建成功，通过 **302 项功能测试**，含局部恢复及软故障交回巡航；有界小消息与真实双窗口回放检查通过，后者未执行恢复动作。此前 v5 两窗口回放观察到完整单右转交接。**尚未按本次 U 盘流程在新建干净容器或助教设备上重新验证**；验证后应把镜像、依赖差异、工作区路径和结果回填本文。具体帧数、能力和限制保留在 README，避免现场操作文档过长。

---

## 补充 A：环境与接口检查（现场遇到问题时再用）

以下不是每次启动都要输入的命令，可在自己的设备上复制执行。

### A.1 找不到包或加载了另一份代码

先加载 ROS 和当前工作区：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix process_node
ros2 pkg prefix robotsdk_msgs
```

process_node 应指向目标工作区的安装目录。新终端需重新加载环境；旧控制进程也需退出后重启。

如果 `robotsdk_msgs` 来自另一课程工作区，按“ROS → 课程依赖工作区 → 当前工作区”的顺序加载各自的 setup.bash。如果目标工作区已有它的源码，可在工作区根目录构建依赖后再启动：

```bash
cd ~/ros2_ws
colcon build --packages-up-to process_node --symlink-install
source install/setup.bash
```

只有 process_node 而没有自定义消息包源码/安装文件时，应补齐课程环境，不自行改写消息定义绕过缺包问题。

### A.2 有节点但没有控制/窗口

```bash
ros2 topic info /sync_store_node/allsensor --verbose
ros2 topic info /encoder_imu_node/odom --verbose
ros2 topic info /process_node/control --verbose
ros2 param get /process_node/process_node steer_kp
```

- `WAITING` / `WAITING_ODOM`：先检查设备节点和输入，不是靠增大增益解决。
- `UNALIGNED_ODOM` / `STALE_ODOM`：看消息时间、接收和 skew，实车 use_sim_time 应为 false；不先放宽对齐容差。
- `CORRIDOR_ACQUIRING` / `CORRIDOR_LOST`：查看真实墙面和模型是否可靠；`CRUISE_BLOCKED` 表示参考附近没有合格候选。
- `TURN_FAULT`：结合 `recovery` 看是否为可恢复路径问题；`RECOVERY_FAULT` 或硬转弯故障保持停车。保留首次 cause 和当前 reason，不能只依据截图认定道路太窄。
- 没有图形桌面时，显式使用 `show_visualization:=false`；有桌面却没窗口时检查 DISPLAY 和启动日志。不要把某次 DISPLAY/XAUTHORITY 固定写入 ~/.bashrc；VS Code 转发异常时重新连接或重启容器。详见 README。
- `speed` 是底层动力控制量，不是 m/s。PI 输出很小或为零时，查看目标/实测速度、动力符号和死区，不直接增大积分。

## 补充 B：测试、回放和录制（文档末尾，可在自己的电脑复制执行）

本节无需在小车上逐条手敲；主要用于之后的干净容器验证。测试前先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
```

### B.1 不创建控制节点的导入检查

```bash
python3 -c "import rclpy, numpy, cv2, cv_bridge; from nav_msgs.msg import Odometry; from robotsdk_msgs.msg import AllSensor, Control; from process_node.process_node import LaserWindowAnalyzer; print('导入成功')"
```

不会发布控制，可发现主要 Python 模块、消息包或环境加载问题。

### B.2 没有小车和 bag 时的启动检查

```bash
ros2 launch process_node process_node.launch.py show_visualization:=false
```

观察节点持续等待输入且无导入异常，随后 Ctrl+C 能正常退出。缺少输入时不能验证行驶控制。

### B.3 自动化功能测试

需要 pytest；不运行镜像旧模板的风格/版权测试，测试不会验证真实车辆运动。

```bash
python3 -m pytest src/process_node/test/test_perception.py \
  src/process_node/test/test_motion.py src/process_node/test/test_control.py \
  src/process_node/test/test_pid_controller.py src/process_node/test/test_node_closed_loop.py \
  src/process_node/test/test_node_baseline.py src/process_node/test/test_window_monitor.py \
  src/process_node/test/test_corridor.py src/process_node/test/test_node_corridor.py \
  src/process_node/test/test_input_buffer.py \
  src/process_node/test/test_exits.py src/process_node/test/test_node_exits.py \
  src/process_node/test/test_turning.py src/process_node/test/test_node_turning.py \
  src/process_node/test/test_recovery.py src/process_node/test/test_node_recovery.py -q
```

### B.4 录制数据离线回放

需要另行复制或挂载 bag，并保持独立 odom 话题。终端一启动本包，使用回放时间：

```bash
ros2 launch process_node process_node.launch.py use_sim_time:=true
```

终端二加载环境后回放，按实际位置替换数据路径：

```bash
ros2 bag play /data/data_whole/rosbag2_2026_10_06-18_13_20 --clock
```

无桌面时在终端一增加 `show_visualization:=false`。回放不会响应当前控制改变下一帧数据，不能证明实车转弯成功。课程 bag 后段 AllSensor 存在长间隔/成批到达，可能触发时间保护；保持已有阈值。重新从头播放时也应重启控制节点。

恢复开始修正前要求实测速度不高于 0.05 m/s；回放中的原车可能持续行驶，导致恢复等待或累计路程超限，这是录制输入不会响应本次停车指令的限制。不能通过放宽保护来追求回放控制连续。无新增依赖/话题/启动参数；U 盘需包含 recovery.py，仍只拷贝整个 process_node 并按原步骤编译。新建干净容器的迁移验证仍待用户执行。

### B.5 实车短段录制

另开终端加载环境后执行；输出目录需可写且不存在同名录制结果：

```bash
ros2 bag record -o /data/trial_01 \
  /encoder_imu_node/cartoimu /encoder_imu_node/odom /scan \
  /sync_store_node/allsensor /process_node/control
```

每段另存参数 YAML、终端日志、车辆编号、电池电压和观察到的现象，便于回到开发环境分析。
