# 仓库说明与协作约定

本仓库用于《智能机器人概论》2026 年期中作业：控制小车沿楼道行驶一圈，并在后半段完成 S 弯绕障。仓库根目录是 `~/ros2_ws/src`，ROS 2 工作区根目录是它的上一级 `~/ros2_ws`。

## 作业资料与约束


- `docs/期中任务.pdf`：9 页，任务、评价、提交要求及控制量标定。
- `docs/实习课3.pptx`：23 页，开发、话题、行为设计、PID 和离线调试说明。
- `docs/期中上机准备-学生.pptx`：8 页，实车启动、录制及设备操作要求。

当前 `.gitignore` 忽略 `docs/`，这些原始材料不随 Git 提交；本文件和 `IMPLEMENTATION_PLAN.md` 保留主要要求。任务细节有新增通知时，应更新文档并注明来源。

必须遵守的课程要求：

1. 完成一圈行驶并到达终点区域；碰撞、不能完成 S 弯绕障、不能到达终点均视为失败。障碍物不可撞、压，位置和间距不预先约定，数量在比赛前公布。路线图仅为示意，不能据此写死路线坐标。
2. 按行为式架构设计，明确行为集、触发机制、行为选择；同时实现纵向速度和横向方向的闭环控制。
3. 主要入口是 `process_node/process_node/process_node.py` 的 `process_core`。允许修改整个 `process_node` 包，也允许新增自己的包和消息类型；**不得修改或删除 `process_node` 以外的已有包**（《实习课3》第 10 页）。其他包用于阅读和接口核对。
4. 《实习课3》第 9 页要求：`AllSensor` 使用激光、深度和彩色图像字段，其他字段不要直接使用。运动反馈从独立的 `/encoder_imu_node/odom` 或 `/encoder_imu_node/cartoimu` 获取。即使当前聚合源码填充了其他字段，也按课程约束使用接口。
5. 设计报告 A4 至少一页，包含行为设计与两类运动控制方法。实车评价关注行驶距离和完成速度。

2026-10-22 用户结合录制数据和路线缩略图补充：外圈按顺时针行驶，三个大直角转弯是丁字路口并选右侧分支，前方可能仍通；右下角有三角形拐角及短 Z 形（先左后右），短连接段不强求回中。路线图只提供拓扑/分支选择先验，不写死坐标、距离、时长或转弯计数。侧向开口必须有道路几何证据，门洞不能直接触发右转。

## 目录结构

| 路径 | 作用 |
|---|---|
| `process_node/` | 本次作业主要修改范围，Python ROS 2 包 |
| `process_node/process_node/process_node.py` | 传感器订阅、`process_core`、控制发布、图像显示 |
| `process_node/process_node/perception.py` | 无 ROS 依赖的测距处理、候选通道和直线车体扫掠检查 |
| `process_node/process_node/corridor.py` | 真实墙线拟合、中线、里程计平移/转动补偿、回正/回中参考及限时降级 |
| `process_node/process_node/exits.py` | 前向/左右入口几何诊断、真实边界支持、odom 编号关联和短时保留；不接管控制 |
| `process_node/tools/analyze_exits.py` | 有界读取独立 scan/odom 小消息，输出出口 CSV/PNG，不发布控制 |
| `process_node/process_node/input_buffer.py` | 独立接收线程与主线程之间的有界里程计缓存、最新扫描和真实接收时间 |
| `process_node/process_node/motion.py` | 独立里程计位姿差分、滤波、局部运动状态和按激光时间关联 |
| `process_node/process_node/control.py` | 统一参考/反馈/控制量及诊断接口，保留旧比例映射/固定动力适配器 |
| `process_node/process_node/pid_controller.py` | 方向 PD、前进速度 PI、斜坡、抗积分饱和、限幅与变化率限制 |
| `process_node/process_node/visualization.py` | RGB 指示盘和激光诊断图绘制 |
| `process_node/process_node/window_monitor.py` | 窗口关闭识别，兼容 GTK3 不支持 OpenCV 可见性属性的情况 |
| `process_node/config/baseline.yaml`、`process_node/README.md` | 感知基线参数、运行和图例说明 |
| `process_node/MIGRATION.md` | 迁移到助教设备、依赖与构建、实车启动及故障排查 |
| `process_node/process_node/process_node_bk.py` | 镜像附带的另一版本，含额外依赖，不作为默认入口 |
| `process_node/launch/process_node.launch.py` | 启动控制节点，并设置 `/process_node` 命名空间 |
| `process_node/setup.py`、`setup.cfg` | 注册可执行入口和安装路径 |
| `process_node/test/` | 原有风格检查，以及新增感知/ROS 集成测试 |
| `robotsdk_msgs/msg/` | `AllSensor`、`Control`、`ImuEncoderAngle` 消息定义，只读 |
| `encoder_imu_node/` | 编码器/IMU处理、里程计发布及控制消息转串口，只读 |
| `sync_store_node/` | 同步传感器并发布 `AllSensor`，只读 |
| `visualization/` | 传感器可视化和 `message_filters` 同步示例，只读 |
| `fishbot_cartographer/` | 建图配置和启动文件，只读；不是当前离线回放的必需步骤 |
| `process_node_cpp/` | 课程提供的 C++ 模板，只读 |
| `hw1*`、`hw2/`、`bag2csv_imu.py` | 其他课程作业和工具，保留原状 |
| `docs/` | 本地课程原始资料，当前被 Git 忽略 |
| `IMPLEMENTATION_PLAN.md` | 初步方案、分阶段验证和待标定事项 |

`~/ros2_ws/build`、`install`、`log` 是工作区构建产物。仓库内也残留同名目录及 `cmake-build-*`、`__pycache__`，均不是源码，不提交。构建统一在 `~/ros2_ws` 执行，避免在 `src` 下继续生成构建目录。录制数据放在 `/data` 等仓库外位置。

## 数据与控制链路

离线：`ros2 bag play` → 传感器话题 → `process_node` → `Control` 消息与图像叠加。

实车：传感器/同步节点 → `process_node` → `encoder_imu_node` → 串口 → 底层控制板。

| 话题 | 类型 | 用途 |
|---|---|---|
| `/sync_store_node/allsensor` | `robotsdk_msgs/msg/AllSensor` | 当前默认输入；`msg.urg` 为雷达，`msg.xtion_color` 为彩色图像，`msg.depth` 为深度图 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 课程提供数据包中的独立雷达输入 |
| `/encoder_imu_node/odom` | `nav_msgs/msg/Odometry` | 位置、姿态及运动反馈；使用前验证各字段 |
| `/encoder_imu_node/cartoimu` | `sensor_msgs/msg/Imu` | 朝向、角速度及加速度接口 |
| `/process_node/control` | `robotsdk_msgs/msg/Control` | 实车底层节点默认订阅的控制话题 |

注意话题和单位：

- 直接执行 `ros2 run process_node process_node` 时，相对话题 `control` 默认解析成 `/control`；附带 launch 或 `-r __ns:=/process_node` 才会发布 `/process_node/control`。实车必须核对发布、订阅是否匹配，避免重复启动控制节点。
- 文档和当前 bag 使用 `/scan`；部分旧源码和建图配置仍使用 `/urg_node/scan`。应检查实际话题，通过参数或重映射适配，不能假设两个名字相同。
- 雷达角度单位为弧度，距离为米：`theta_i = angle_min + i * angle_increment`。课程说明前方为 +x、左侧为 +y，扫描约 360°；仍应读取每帧元数据并核实安装方向。前方扇区可能跨越扫描数组首尾。
- 原始模板的 `Angle_increment = 1`、`Middle_angle = 540` 是硬编码；现有基线已改为读取每帧真实角度、处理数组首尾和无效测距。`target` 日志为真实角度（度），不再把下标称为角度。
- 当前 `midterm-exits-v4.1` 延续受限无回波规划：保留原始 ranges，+inf 默认最多 5 m，受 range_max 限制，0 可禁用；不与 NaN、零值或未扫描方向混同，整帧默认至少 5% 真实有限回波。候选覆盖前向及 0°，绿色开口仅供诊断。corridor 模式另加入精确走廊参考，只选参考附近合格候选。蓝色为无回波上限，红色为其他无效/未知，棕色/紫色为墙线/中线模型，青色为活动目标。
- `Control.steer`、`Control.speed` 均为 `int16`。底层节点将二者打包后原样发送；`speed` 是动力控制量，不是 m/s，`steer` 也不是已知车轮转角。中位、正负、范围和响应需逐车标定。
- 现有 `encoder_imu_node.py` 的 `odom.twist.twist.linear` 使用速度差填写，不应未经核验当作真实线速度。当前 `motion.py` 已用独立里程计位置/姿态与消息时间差估速和角速度；不要因此修改底层包。默认要求坐标系 `odom` → `base_link`、有效反馈接收未超时且与激光时间接近；具体参数见 `baseline.yaml`。

## 构建与加载环境

使用课程提供的 ROS 2 Humble 容器，常用依赖有 `rclpy`、`nav_msgs`、NumPy、OpenCV、`cv_bridge` 和 `robotsdk_msgs`。新增同步等依赖时，在 `process_node` 内同步更新声明。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select process_node --symlink-install
source install/setup.bash
ros2 pkg prefix process_node
```

上述选择性构建以课程镜像中已安装其他依赖包为前提。`ros2 pkg prefix` 应指向当前工作区的安装目录。每个新终端都要重新加载环境；`ros2 run` 可在任意目录执行。

## 离线运行

当前本地数据包目录为 `/data/data_whole/rosbag2_2026_10_06-18_13_20`，应包含 `metadata.yaml` 和 `.db3` 文件。换机器后按实际位置替换路径。

终端一，先启动订阅节点（统一控制话题命名空间，并使用回放时钟）：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run process_node process_node --ros-args -r __ns:=/process_node -p use_sim_time:=true
```

终端二，查看并回放数据：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 bag info /data/data_whole/rosbag2_2026_10_06-18_13_20
ros2 bag play /data/data_whole/rosbag2_2026_10_06-18_13_20 --clock
```

终端三，可核对接口和输出：

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list -t
ros2 topic info /process_node/control --verbose
ros2 topic echo /process_node/control
```

推荐用 `ros2 launch process_node process_node.launch.py use_sim_time:=true` 启动当前基线，命名空间相同，并加载 `config/baseline.yaml`。无桌面环境加 `show_visualization:=false`；参数和测试说明见 `process_node/README.md`。

课程称为“模拟器”的这套流程是录制数据回放，控制输出不会改变下一帧输入。窗口由 `process_node` 创建，背景是水平翻转后的录制 RGB 图像。指针相对 `steer_center` 的零偏移朝上，正负最大偏移分别朝右和朝左；表示显示比例，不表示真实转角。另有激光图显示真实角度、未知区域、候选通道及选中目标。不能用回放证明车辆已完成闭环绕障，或据此标定控制增益。

播放结束后订阅节点继续等待，独立单调时钟定时器处理输入超时和窗口事件，超时输出配置的停止值。按 q、Esc、点击任一窗口关闭按钮或 Ctrl+C 可退出；无图形桌面可关闭可视化。

输入由独立 ROS 接收线程写入 input_buffer：里程计最多 200 条，AllSensor 只保留最新待处理帧。主线程先批量更新反馈，再计算控制和操作 GTK；单调时钟定时器只唤醒主线程，GUI 不可移到 ROS 工作线程。真实接收时间不能用处理时间代替，发布前重新检查新鲜度，缓冲溢出停止并重建反馈。GUI 默认最多 10 Hz，不重复传输同一画面，按键/关闭持续处理。input_skipped_scans 是主动替换的中间扫描数；禁止通过放宽 0.15 s 对齐容差来掩盖 GUI 导致的里程计积压。后续实时验证必须覆盖 GUI 开启场景。

当前默认 `control_mode=closed_loop`、`reference_mode=corridor`、`target_speed_mps=0.2`。`corridor.py` 仅用真实有限点，通过网格采样、固定种子 RANSAC 和正交最小二乘拟合墙线；双墙按平行、宽度和运动补偿后的历史一致性筛选，默认连续确认 3 帧。中线保存于 odom 坐标系，补偿平移和转动后融合，按走廊航向和有符号横向偏移生成参考。单墙依赖近期双墙宽度，默认最多 0.6 s；无墙历史默认最多 0.3 s/0.2 m，过期重新确认。单墙、保持或较大误差时请求速度减半；参考附近默认 ±5°无合格候选则 CRUISE_BLOCKED 停止，不自动转向侧向开口。安装平移、拟合、门控、目标点和降速参数均为 corridor_ 前缀。

当前版本标记 `midterm-exits-v4.1`。出口诊断默认开启，参数 exit_ 前缀；独立于巡航控制，不能用 preferred_id 绕过控制门控。沿可信接近走廊检查前向和两侧入口，记录可见宽度和纵深可见比例（不能把遮挡区域称为已验证自由）；真实边界点数/跨度及 3 帧/0.15 s 才确认，纯 inf 不构成道路结构证据。入口按 odom 位置/方向关联编号，暂失显示 HELD，0.5 s/0.5 m 后过期；接近坐标诊断历史最多 3 s/2 m，不延长原控制保持。当前可见的已确认入口才参与右优先诊断建议；没有“必须先回中”或统一转弯冷却门槛。黄色/暗色/灰色入口、白圈是诊断；青色仍为控制目标。尚非完整路口分类器，宽门/房间可能误认，Z 形遮挡时也不能保证看见第二弯；真实转弯行为和路径检查待实现。

v4.1 巡航过期判断先检查当前一致双墙/可用单墙，无新证据时才应用无墙保持期限；单墙的双墙宽度年龄、大扫描间隔、输入/反馈超时和回跳保护不放宽。日志 scan_dt 是处理扫描的消息间隔，fitted_walls 是当前拟合墙数，ACQUIRING 不等于没有拟合出墙。新增 exit_corner_*：真实墙端/局部线段加旁侧 20–85° 可见开口只生成 CORNER_HINT / 橙色 SUSPECTED，不能仅凭持续出现升级或参与路线建议；转为标准 SIDE 后重新确认，可关联原编号。无接近坐标时仅相对车头报告墙端线索 EXIT_HINTS_ONLY，不假设宽度、不延长巡航历史。tools/analyze_exits.py 支持 --scan-stride，保留全部里程计模拟跳过扫描。

课程 bag 后段 298–313 s 的 AllSensor 录制时间存在 >0.5 s 间隔/成批到达（11 次，最大约 0.94 s），独立 /scan 连续（最大约 0.097 s）。因此独立扫描的几何分析通过，不代表 AllSensor 实时回放的控制输出连续。GUI 验证需记录 scan_dt 并保留现有大间隔保护，不通过放宽阈值掩盖输入时序问题。该发现来自 2026-10-22 对用户 straight.png 附近数据及真实 GUI 回放的检查。

`ControlInput` 将参考与关联反馈交给方向 PD/前进速度 PI，包含速度斜坡、抗积分饱和、限幅及变化率限制。有效输入恢复后 CONTROL_WARMUP 停止一帧，异常/超时/受阻/零请求覆盖正常平滑并重置控制历史，输入或运动异常还清除巡航参考。corridor 强制 closed_loop 和有效反馈；旧评分对照用 reference_mode=perception，只有 baseline+perception 下可关闭反馈门控。straight 只接受合格 0°，用于中位/速度 PI 测试，不保持全局航向。控制符号、安装偏移、前馈和增益待实车标定，参数修改需重启。90°转弯、S 弯、终点及实际弯曲轨迹检查仍待实现，直走廊稳定回中也需实车验证。

## 实车运行与录制

课程规定的整车入口是：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch poss_cpp poss_launch.py
```

`poss_cpp` 属于实车环境；当前仓库未提供该包，当前容器加载现有工作区后也未找到它。本地回放无需启动整车驱动；上车时由助教确认整车入口、环境和节点命名空间。

整车运行期间，可在另一个终端录制关键话题：

```bash
source ~/ros2_ws/install/setup.bash
cd /data
ros2 bag record -o trial_01 /encoder_imu_node/cartoimu /encoder_imu_node/odom /scan /sync_store_node/allsensor /process_node/control
```

使用实车时按《期中上机准备》执行：启动前保持手动模式（遥控器 ON 为手动，OFF 为自动），车辆放在地面空旷处，禁止桌面启动；临近碰撞立即切手动。检查组合电池 18–21 V、大电池 11.0–12.6 V，低电量及时更换。控制量不得过大或突变，避免碰撞损坏及参数变化；注意行人，禁止遥控快速驾驶回机房。结束后正常关闭 Ubuntu，再关闭设备电源，按要求整理设备，不拆装电池单元。

## 后续开发与验证

- 后续修改代码、配置或包结构后，必须同步核对并更新 `process_node/MIGRATION.md`，使迁移步骤与当前实现一致。涉及新增包、依赖、安装规则、构建/启动命令、参数、话题或实车标定要求时，必须同步修改对应说明；实现范围发生变化时也应更新文档中的当前能力和限制。现场迁移发现的环境差异及验证结果应回填该文档。
- 先按 `IMPLEMENTATION_PLAN.md` 实施感知、行为、控制和离线分析；在用户要求仅分析或规划的轮次，只更新文档。
- 保留 `process_core` 接收激光数据、返回 `(steer, speed)` 的集成入口，可将实现拆成 `process_node` 内部模块。将增益、阈值、车体尺寸、标定值等做成参数，并显示小组/版本标记。
- 对 `NaN`、`inf`、无有效测距、数据超时、时间回跳和控制饱和作明确处理。控制器的时间差使用消息/ROS 时间，不能固定假定帧率。
- 验证优先覆盖真实风险：扫描首尾拼接、障碍物左右通过顺序、状态切换、转向抖动、速度反馈与输出约束。现有风格测试不能证明避障正确，也不要为通过旧模板风格检查而改写其他包。
- 离线记录行为、触发原因、目标与反馈、输出及风险距离；实际距离、用时和通过率由实车测试取得。报告区分设计目标、回放观察和实车结果。
- 仅提交源码、配置、文档和小型测试材料；大型 bag、解压资料和构建产物放仓库外。保留用户未提交的修改，不自动提交 Git。
