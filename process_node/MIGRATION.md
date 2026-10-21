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

当前依赖已在 `package.xml` 中声明，包括 `rclpy`、`sensor_msgs`、`geometry_msgs`、`std_msgs`、`robotsdk_msgs`、`cv_bridge`、NumPy、OpenCV、`python3-gi` 及 launch 相关包。相同镜像通常已具备这些依赖；缺失时根据报错补齐。GTK3 窗口关闭识别使用系统包 `python3-gi`。

构建完成后，`ros2 pkg prefix process_node` 应指向当前目标工作区的安装目录。若指向其他工作区，在新终端中按“ROS Humble → 课程依赖工作区（如有）→ 当前工作区”的顺序重新加载环境。

## 4. 不启动控制节点，先验证导入

```bash
python3 -c "import rclpy, numpy, cv2, cv_bridge; from robotsdk_msgs.msg import AllSensor, Control; from process_node.process_node import LaserWindowAnalyzer; print('导入成功')"
```

该命令不会创建控制节点或发布控制消息，可发现主要 Python 模块、消息包或环境加载问题。它不验证设备连接和车辆运动。

需要运行现有自动化测试时，在 `~/ros2_ws` 中执行：

```bash
python3 -m pytest src/process_node/test/test_perception.py \
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

| 场景 | `use_sim_time` | 输入来源 |
|---|---|---|
| 实车 | `false` | 实时传感器及同步节点，无需 `ros2 bag play` |
| 离线回放 | `true` | `ros2 bag play <数据包路径> --clock` |

launch 默认加载随包安装的 `config/baseline.yaml`，并将控制话题放在 `/process_node/control`。使用现场专用配置时，可指定 `config:=/绝对路径/参数.yaml`。修改配置后重新启动节点；修改依赖、安装规则或启动文件后重新构建并加载环境。

## 6. 上车前核对

按课程要求保持遥控器 ON（手动模式），车辆置于地面空旷处，再核对：

```bash
ros2 topic info /sync_store_node/allsensor --verbose
ros2 topic info /process_node/control --verbose
```

确认输入话题有传感器发布端和本节点订阅端；控制话题只有预期的控制发布端，并有底层驱动订阅。话题存在不代表数据持续到达，还应结合本节点日志确认接收与分析正常。实际传感器话题不同，可通过配置中的 `sensorTopic` 适配。

现场按实际车辆核对以下配置：

- 雷达安装偏角、车宽、雷达到车体前后端的距离及安全裕量。
- 转向中位、正负方向、幅度、增益，以及动力值和停止值；`steer` 和 `speed` 均是底层控制量，不直接等于转角和 m/s。
- 控制话题与底层驱动一致；切换自动前由助教确认车辆状态，临近碰撞立即切回手动。

当前实现仅为激光感知基线：转向使用比例映射，有候选方向时动力值固定；尚未实现 S 弯行为、终点判断、独立运动反馈及 PD/PI。默认参数供离线调试，不能因编译成功就视为已完成整车控制标定。

## 7. 常见问题

| 现象 | 排查方向 |
|---|---|
| `Package 'process_node' not found` | 检查构建是否成功；在当前终端加载目标工作区的 `install/setup.bash` |
| 找不到 `robotsdk_msgs` 或导入消息失败 | 加载课程依赖环境；有源码时构建依赖包 |
| 同名包冲突 | 把旧包备份移出工作区 `src/`，检查是否多套源码重复放置 |
| 运行的似乎仍是旧版本 | 检查 `ros2 pkg prefix process_node`、环境加载顺序，并退出旧进程后重新启动 |
| 一直 `WAITING` 或频繁 `STALE_DATA` | 检查整车驱动、输入话题、数据是否到达及跨容器通信环境 |
| 有输出但底层收不到 | 检查是否发布到了 `/control`；优先使用本文 launch，核对 `/process_node/control` 的端点 |
| 无桌面或窗口无法打开 | 使用 `show_visualization:=false`；有桌面时检查显示环境和 GUI 依赖 |

本文步骤依据当前仓库配置整理，尚未在助教设备上验证。首次迁移后应记录实际工作区、依赖差异及可用的整车启动方式，并据此更新本文。
