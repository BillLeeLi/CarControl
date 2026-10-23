# 走廊巡航、单个右转与低速 PD/PI 控制

当前标记为 `midterm-right-turn-v5`，默认 `control_mode=closed_loop`、`reference_mode=corridor`、`turn_enabled=true`。直路按真实墙线/中线巡航；右侧道路出口经确认后，可接管为 **CRUISE → APPROACH → TURN → EXIT_ALIGN → CRUISE**，使用锁定在 odom 的圆角参考、逐帧短程曲线整车检查，以及已有方向 PD / 速度 PI。`process_core(urg)` 仍返回 `(steer, speed)`。巡航正常请求 0.2 m/s，接近/右转/对齐默认上限 0.1 m/s，并受全局目标速度限制。左转、短 Z 连弯、S 弯绕障和终点判断未实现；最小转弯半径、控制响应及停车能力尚需实车标定。跨设备迁移见 [MIGRATION.md](MIGRATION.md)。

**新增行为：默认必须收到有效的独立 `/encoder_imu_node/odom`，且它与激光消息时间接近，才允许输出行驶指令。** 当前课程 bag 已包含该话题，完整回放命令不变。只有激光数据时，会输出停止值。

## 车辆尺寸记录（2026-10-23）

依据用户向老师测量的结果更新默认尺寸，Python 内置默认值与 baseline.yaml 一致：

| 参数 | 当前值 | 测量依据 |
|---|---|---|
| `vehicle_width_m` | 0.22 m | 车宽约 22 cm，雷达在车宽中线 |
| `front_overhang_m` | 0.16 m | 雷达到车头约 15–16 cm，取上限 |
| `rear_overhang_m` | 0.26 m | 雷达到车尾约 25–26 cm，取上限 |
| `safety_margin_m` | 0.10 m | 保留原额外安全裕量，不包含在上述实测尺寸中 |

对应车体矩形长 0.42 m，以雷达为参照为 x∈[-0.26, 0.16]、y∈[-0.11, 0.11]；不是以雷达为车长中点。巡航包络圆盘仍采用最远车角加裕量，半径约 0.382 m（原默认约 0.447 m），因此仍比真实车宽保守。转弯矩形加原裕量后宽 0.42 m、长 0.62 m，采样间隙还会额外膨胀。

尺寸始终相对雷达定义。`PlannerConfig.footprint_bounds` 把此矩形按 `corridor_sensor_x_m/y_m` 转到 base_link，供转弯碰撞、旋转膨胀和绘图使用，避免把同一组尺寸误当成相对两个不同原点的距离。雷达偏角只在扫描整理时旋转一次。**雷达与 base_link 的实际相对位置仍未确认**，安装平移的 0 仍是默认假设；雷达位于车宽中线不代表它与里程计原点重合。

右转完成还要求整车矩形连同安全裕量越过入口，按当前航向投影车角，避免因雷达更靠前而忽略车尾。尺寸与坐标转换修订已构建成功，通过 **263 项功能测试**，包含前后不对称、安装平移后的前/后/侧碰撞及带裕量整车进入出口；未作实车运行验证。

本次未获得有效实车运动标定数据，转向/动力增益、中位、符号、最小转弯半径和停车能力仍为待标定项。车体尺寸更新不会自动修复左转/Z 形能力缺失或 TURN_FAULT 锁存。

## 数据如何变成目标方向

1. `perception.py:prepare_scan` 保留原始下标，用本帧角度参数计算每个方向，并叠加可配置的雷达安装偏角。前方 0° 两侧的数组首尾会在角度空间连起来。
2. 有限、正数且处于量程内的测距是有效回波；`+inf` 单独标记为无回波，按“有限规划范围内未命中”的假设参与候选检查，原始 `ranges` 和有限回波标记不变。范围取 `min(no_return_distance_m, range_max)`，默认最多 5 m；设为 0 可禁用。`NaN`、零、负值及超量程有限值仍无效，不能获得相同语义。负无穷或正数但低于最小量程直接触发停止。
3. 按固定角宽分箱，检查有限回波与受限无回波的合计可用比例，取最近的规划距离。一个真实近点不会被同箱大量 `inf` 平均掉。很短的无效缺口仅在两端都有真实有限回波且距离相近时才补齐；长 `NaN` 段和未扫描方向仍然未知。无回波自身不计作插值。
4. **直接在前向角度空间生成候选并显式加入 0°**，不再要求候选先属于连续远距离回波构成的开口。候选的整个角度窗口须有分箱覆盖，原始可用比例和插值比例须合格；候选窗口可以完全没有有限回波。仍要求整帧有有限回波且其比例至少为 `min_scan_finite_ratio`（默认 5%），全 `inf` 或只有零散回波不会启动车辆。
5. 根据宽度、前后悬和裕量建立车体包络圆盘，沿每个候选直线扫掠。所有真实有效障碍点都参加检查，包括低覆盖分箱里的近点。若候选窗口使用了无回波，圆盘中心可移动的距离还受“无回波规划范围减包络半径”限制，保证末端圆盘留在有限规划范围内。
6. 达到前瞻要求后，额外净空不再增加评分，避免因有限回波与无回波的范围上限不同而拉偏方向；规划距离均值、向前偏好和上一目标连续性参与选优。没有合格候选就停止。

例如，走廊两侧能测到墙、正前方全部为 `+inf` 时，现在允许生成 0°候选；候选的 `finite` 可以是 0%，`support` 仍可以是 100%。这是一项有上限的无回波规划假设，不代表真实测到了 5 m 自由空间；迁移时需核对雷达驱动的无回波语义。全帧有限回波比例只是可用性门控，不能识别所有设备异常。

上述步骤生成的是合格候选；在默认 corridor 模式中，最终目标由下节的走廊参考决定，原评分只作为候选的次级排序。巡航模式的绿色圆盘包络是直线检查区域；右转另画短程圆弧和矩形包络（见下文）。车体参数、雷达朝向、控制中位/符号均需现场标定。

## 走廊方向与横向回中

`corridor.py` 不依赖 ROS，包含带中文字段注释的配置、墙线和走廊估计，以及有历史的 `CorridorTracker`。

1. **只用真实有限回波拟合墙面。** 将点转换到车体坐标，加入 `corridor_sensor_x_m/y_m` 安装平移，只取近处前向区域，并按空间网格采样。固定种子的 RANSAC 找到多条候选墙线，再用正交最小二乘细化。`inf` 和插值点不作为墙线证据；拟合没选中的真实点仍保留在原始碰撞检查中。
2. **选择可信双墙。** 支持点数量、沿墙跨度和拟合误差须合格；两条墙位于车体两侧、大致平行，宽度在参数范围内。跟踪后还检查运动补偿后的走向、中线和宽度跳变，降低门洞、短突出物改变参考的机会。初始需连续 3 帧一致双墙，期间 `CORRIDOR_ACQUIRING` 停车；不能只用单墙和一个假定宽度初始化。
3. **保存世界坐标中的中线。** 中线作为 `odom` 中的直线保存。每帧先用关联到激光的位姿补偿平移和转动，再与新观测融合；滤波时间常数默认 0.25 s，使用消息时间。不是直接平均相邻帧相对车头的目标角，也不会每帧重复执行历史转角。
4. **组合回正与回中。** `axis` 是走廊相对车头的走向；`center_left` 是中线相对车体原点的有符号左向垂直距离，正值表示应向左回中，负值表示应向右。沿走廊默认取前方 1.5 m，再加入横向修正：

   ```text
   横向修正角 = atan2(corridor_lateral_gain × center_left, corridor_goal_distance_m)
   参考角 = axis + 限幅后的横向修正角
   ```

   横向修正默认最多 ±20°；总参考超过 ±45°则 `CORRIDOR_OUT_OF_RANGE` 停止。居中且对齐时目标接近 0°；靠左但已平行时目标向右；居中但车头偏左时同样向右纠偏。它保持的是观测到的走廊方向，并非永久锁定启动时航向。
5. **先验证新观测，再判断历史是否过期。** 当前双墙若通过运动补偿后的几何门控，直接更新参考，不能仅因距上次处理超过 0.3 s 就重建。单墙可结合距最近双墙最多 0.6 s 的宽度，并刷新墙面观测时间，但不刷新双墙时间。本帧既无可信双墙也无可用单墙时，才应用无墙保持的 0.3 s / 0.2 m 期限；已因缺证据清除的模型必须重新连续确认。时间回跳、超过 0.5 s 的扫描间隔、输入/反馈超时和运动重建仍重置；没有放宽原保护阈值。单墙、历史保持、走向误差大于 15°或偏移大于 0.25 m 时，请求速度默认乘 0.5；PI 继续按原斜坡跟踪请求。激光完全停止时仍由原 sensor_timeout_s/odom_timeout_s 看门狗负责停止。
6. **参考也必须通过感知检查。** 精确参考角会加入候选采样；仅从合格候选中选择最接近参考、且偏差不超过默认 5°的方向。参考附近不可通行就 `CRUISE_BLOCKED` 停车，不追逐大角度侧向空旷区域。只有下述右转行为在独立检查通过后可以接管；尚无 S 弯绕障行为。

墙线的 `confidence` 为支持跨度和拟合误差等生成的诊断分数，不是正确概率。走廊宽度由观测估计，宽度上下界仅用于排除不合理配对。门洞很大、墙线歧义、走廊突然变化时仍可能失去参考并停车，需结合诊断与实车数据验证。

## 单个右转：四阶段行为

`turning.py` 只接管右侧道路，分支选择不要求前方封闭，适用于顺时针外圈的丁字路口。没有写死路线坐标、转弯时间或转弯计数。左侧线索仍会显示，但本版不会执行左转。

| 阶段 | 行为与切换依据 |
|---|---|
| `CRUISE` | 使用原走廊参考；接近范围内出现右侧 CONFIRMED 或 SUSPECTED 后进入 APPROACH |
| `APPROACH` | 默认最多 0.1 m/s。SUSPECTED 只允许沿可信接近轴观察，不能直接朝斜开口大转向；线索暂失 0.5 s 取消接近，无新双墙时接近轴最多沿用 1 s。标准 SIDE 确认后才尝试圆角路径；无可接入路径则 TURN_NO_PATH 停车 |
| `TURN` | 到达锁定路径的圆角附近后执行右转。入口暂失、preferred_id 改变或旧 CORRIDOR_LOST 不取消路径；每帧曲线仍必须通过当前真实障碍与可见性检查 |
| `EXIT_ALIGN` | 接近圆角末端且接近出口朝向时，清除旧走廊拟合，重新确认新双墙。新道路确认后用其中线和转弯近前瞻修正横向偏移；位置已越过入口、车头/新墙方向吻合、横向偏差合格，连续 3 帧后交还 CRUISE |

### 路径、检查和控制如何连接

1. **固定参考。** 将接近中线和被选出口轴求交，在交点前后构造相切右圆弧（尝试半径 0.7–1.2 m），前后接直段。半径不是测量结果。长参考不能穿过当前已知真实障碍；未观察到的后半段不被宣布自由。路径一次保存到 odom，出口位置抖动不会每帧重画，也不因另一入口更空旷而改选。
2. **逐帧追踪。** 用关联里程计寻找路径进度，在前方约 0.65 m 取点。纯追踪曲率 `k=2*y/(x*x+y*y)` 限制在 `±1/turn_min_radius_m`；以 `atan(k*turn_lookahead_m/2)` 转回方向参考交给原 PD，速度请求交给原 PI。行为切换使用同一入口的 reference_id，交接巡航时才重置 I/D 历史；没有固定输出满舵或按秒转 90°。
3. **短程曲线整车检查。** 从当前位姿按该曲率向前采样，检查距离至少 0.8 m，实测速度较大时覆盖 `v*响应延迟+v²/(2*减速度)`。检查沿圆弧运动的整车矩形，含前后悬、安全裕量和覆盖相邻采样间隙的额外膨胀；所有原始真实点参与碰撞检查。矩形覆盖方向的角箱必须有足够观测深度，未知、NaN、插值和墙后区域不通过，+inf 仍只使用受限规划范围。该检查独立于原 1.5 m 直线候选，不把曲线包装成直线 Candidate。
4. **停止与恢复。** 本帧曲线不合格则 `TURN_BLOCKED` 停车，保留相同入口/路径，障碍清除后控制器预热再恢复。越过路径误差上限、超时/位移上限、输入/反馈失效或大扫描间隔会进入锁存的 `FAULT` / `TURN_FAULT`，本轮不再自动改选出口，重启节点重新确认任务。bag 回跳重播也应重启节点。重复完成的同位置同方向入口被抑制，不设置统一转弯冷却期。

**模型边界：** 被检查的是期望几何圆弧。PD 输出经过底层转向装置、限幅与变化率限制后，实际曲率未必等于 k，尤其尚未标定时；因此本版不是已验证的车辆动力学预测器。上车先测最小半径、方向误差到 steer 的响应、转向建立时间和零动力滑行停车距离，必要时降低速度/增加裕量、调整前瞻和增益。回放的位姿属于原录制驾驶，不受当前输出影响，不能用回放标定这些量。宽门/房间可能被识别成道路，也需现场核对。

### 参数和观察方式

完整字段及中文注释见 `TurnConfig` 和 `config/baseline.yaml`，均带 `turn_` 前缀，修改后重启。

| 参数（省略 turn_） | 默认值/含义 |
|---|---|
| `enabled` | true；false 恢复仅巡航，straight/perception 模式自动不接管 |
| `approach_speed_mps`、`speed_mps` | 0.10 / 0.10 m/s，均与全局 target_speed_mps 取小 |
| `activate_distance_m` | 入口车体前向距离不超过 3.5 m |
| `min_radius_m`、`max_radius_m`、`radius_samples` | 0.7 / 1.2 m，尝试 4 个半径，优先可接入的较大半径 |
| `lookahead_m`、`check_distance_m`、`sample_step_m` | 追踪前瞻 0.65 m、至少检查 0.8 m、采样 0.04 m |
| `reaction_time_s`、`braking_decel_mps2` | 停车距离估计使用 0.3 s / 0.3 m/s²，须核验滑行响应 |
| `exit_extension_m`、`max_tracking_error_m` | 出口延伸 1.5 m、路径横向跟踪误差上限 0.55 m |
| `min_angle_deg`、`max_angle_deg` | 相对接近道路的右转角 55–125° |
| `hint_timeout_s`、`hint_context_s` | 疑似暂失 0.5 s 取消，接近轴最多 1 s 无更新 |
| `max_update_gap_s`、`max_duration_s`、`max_travel_m` | 扫描间隔 0.5 s、单次行为 60 s / 从起点位移 7 m；进一步降低目标速度时需相应核对时限 |
| `exit_heading_tolerance_deg`、`exit_center_tolerance_m`、`exit_confirm_frames` | 完成容差 15° / 0.3 m，连续 3 帧 |
| `passed_exit_distance_m` | 1 m 范围内同方向已完成入口不重复触发；不屏蔽不同方向的新出口 |

启动命令不变。图中灰蓝长线是锁定参考，绿色一串矩形是当前检查的短弧；受阻时包络变暗红。青色箭头只有节点状态 OK 且方向获准时出现；控制预热、受阻或故障时没有青色箭头。终端新增 `behavior`、`turn_status`、`locked_exit`、`curvature` 和具体原因；入口的黄色/橙色显示本身仍不是命令。

可有界读取独立 scan/odom 并同时运行行为（不启动 GUI/PID，不发布控制）：

```bash
python3 ~/ros2_ws/src/process_node/tools/analyze_exits.py \
  /data/data_whole/rosbag2_2026_10_06-18_13_20/rosbag2_2026_10_06-18_13_20_0.db3 \
  --offset 110 --duration 30 --turns --output /tmp/right_turn_check
```

CSV 增加 behavior / turn_status / locked_exit / curvature / turn_reason。图片标记 DIAGNOSTIC_ONLY，不画获准控制青色箭头。该工具使用 dataclass 默认值，现场 YAML 另行加载到节点；不要混淆配置。默认不加 --turns 仍只分析出口。

### v5 验证记录（2026-10-22）

最终 **258 项功能测试通过**，在工作区选择性构建成功。

新增验证覆盖前方仍开放时的右转接管、疑似/未确认/暂失入口、锁定后不改选、旧走廊丢失、途中障碍停车/恢复、曲线内侧障碍、车尾扫掠、未知/无回波上限、安装平移、角度接缝、时间/反馈异常和行为预算。解析墙段加理想曲率位姿积分可以依次经过全部四阶段；另将可见入口中心人为偏置 0.35 m，验证出口按新双墙回中，而不是永久跟随错误入口中心。这是几何闭环试验，不是课程车辆动力学模型。

独立扫描 110–140 s 共 360 帧，只读取约 5.91 MB 小消息：约 116.92 s APPROACH、120.76 s TURN、126.01 s EXIT_ALIGN、126.68 s 交回 CRUISE，活动阶段没有 TURN_BLOCKED/FAULT。最终对齐修订用 110–130 s 的 240 帧再次得到相同切换时间。工具未运行 PID，不能据此评价动力输出或实车转弯误差。

真实 VS Code/X11 两窗口开启，以 AllSensor + 独立 odom 回放相应片段：219 帧反馈全部有效，216 帧节点 OK、1 帧控制预热、2 帧走廊初始确认；完整经过四阶段，约 126.76 s 交还巡航，接收至回调完成最大约 0.32 s，停止回放后正常超时停车并退出。随后修改了出口对齐近前瞻与扫掠采样裕量，最终几何/节点测试及上述独立扫描用于回归。GUI 记录验证显示和整条回放调用链，仍不是车辆在执行本程序输出。相关本地记录位于仓库外 /tmp/right_turn_v5。

## 运动反馈与控制调用链

ROS 接收在独立线程中运行，订阅回调只将消息交给 `input_buffer.py`：里程计有界缓存默认 200 条，AllSensor 只保留最新待处理帧。主线程先按接收顺序批量更新里程计，再处理最新扫描、计算控制并操作 GTK/OpenCV 窗口。因此慢绘图不会占住 ROS 接收回调，旧图像也不会不断排队形成旧控制。

接收时间在接收回调入口记录，处理缓存时不会刷新它；计算完成、发布前再次检查新鲜度。里程计缓冲溢出时停止并重新建立差分历史。主线程按单调时间检查超时和窗口事件，ROS 单调时钟定时器负责唤醒，回放时钟暂停不影响这些检查。GUI 默认最多 10 Hz 传输图像，同一画面不重复传输，状态变化立即更新，关闭/按键事件仍持续处理；所有 GTK 操作始终在主线程。

日志 `input_skipped_scans` 是接收缓冲中被新扫描替换的中间帧数，不包含 DDS 自身丢帧。负载较高时允许跳过中间帧以处理最新观测，控制器仍使用真实消息 dt。时间关联容差保持 0.15 s；缺少或过期的反馈仍停止，不能通过放宽门控来掩盖队列积压。

```text
独立 Odometry → motion.py：位置/姿态差分 → 带时间戳的运动状态缓存
                                                   ↓ 按激光消息时间关联
AllSensor.urg → prepare_scan → 走廊估计/运动补偿 → 巡航参考
                      ↓                          ↓
              analyze_prepared_scan ← 加入精确参考角
                      ↓ 合格候选 + 参考附近选优
            出口识别 → 右转行为仲裁（或继续巡航）
                            ↓
                ControlReference + MotionFeedback
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

## 道路出口识别与追踪（几何证据）

2026-10-22 用户根据课程路线补充：外圈顺时针，三处大转弯是丁字路口并应右转；右下角还有三角形拐角和短 Z 形连续反向转弯。缩略图只用于理解道路拓扑，程序没有读取图中的坐标、长度或固定转弯时间。

`exits.py` 独立运行于原有巡航目标之外，处理流程为：

1. 从已确认双墙取得接近走廊的方向、中线和宽度，并保存到 odom 坐标。直走廊参考丢失后，出口诊断可继续使用经平移/转动补偿的接近坐标，默认最多 3 s / 2 m；这不会延长巡航控制的历史保持期限。没有可信接近坐标时，不生成可确认的前向/侧向入口；若仅有墙端线索则显示 `EXIT_HINTS_ONLY`，否则为 `EXIT_NO_CONTEXT`，不凭默认宽度猜路。
2. 沿走廊前方 4 m 范围探测左右入口，尝试侧向 90° 及其 ±20° 的方向假设；同时检查前向延续。**右侧出口不要求前方封闭**。若已见侧向入口，前向延续截面放在路口另一侧，避免把路口前的短直路误报成直行出口。
3. 可见入口默认至少 1.0 m 宽，向外至少观察 1.0 m 纵深。墙角会遮挡侧向走廊深处，故入口宽度使用近截面，并单独记录能看到完整纵深的比例（默认至少 15%）。**未被看见的其余纵深仍是未知区域，不据此宣布整幅入口可通行**。`depth_visible` / 日志 `deep` 表示这个比例。
4. 可见性使用真实有限回波及受限 `+inf`，不穿过墙、NaN、未扫描方向或插值箱；另外保留原始真实点对入口内部的障碍检查。确认还需要边界真实点数和跨度，不能仅凭一片 `inf`。触及搜索边界、超宽区域、缺少真实结构支持的开口保留为 `UNVERIFIED`。
5. 入口中心和朝向保存于 odom 坐标，以位置及方向关联编号；短时遮挡显示 `HELD`，默认最多 0.5 s / 0.5 m。重新出现后重新连续确认，不能让旧确认无限续期。前向截面随车移动，按方向和横向位置关联，不冒充固定路口地标。
6. 连续 3 帧且覆盖至少 0.15 s 才为 `CONFIRMED`。扫描时间重复/回跳、长间隔、反馈失效、接收超时或运动历史重置会清除相应历史。不存在转弯后的统一冷却期或“必须先回中”的门槛；新接近走廊参考建立后可以立即探测下一处反向出口。

7. **墙端旁的斜向疑似出口。** `CORNER_HINT` 用有限回波的测距跳变定位墙端，要求附近真实点形成有足够跨度、较小拟合误差的线段，再检查旁侧 20–85° 范围内的连续可见开口：在墙端距离处弦宽至少 1.0 m，超出墙端距离至少可见 0.6 m。未知、墙后和无结构的纯 inf 区域不能提供该线索。它只产生橙色 `SUSPECTED`：圈出墙端、短箭头指向可见开口，不把弦宽画成已确认入口截面。连续出现也不会自动确认或参与建议；后续形成正常 SIDE 几何时，允许按较宽的位置/方向容差关联原编号，重新满足 3 帧 / 0.15 s 才确认。它不是出口中心线或可执行转弯路径。

   没有可信接近坐标时，墙端线索仅相对当前车头划分左右，不假设走廊宽度、不生成路线建议；有可信坐标时，左右相对接近走廊。两种情况都仅使用当前真实墙面和可见自由区域，不能为巡航续期。

图中黄色入口截面/短箭头表示 `CONFIRMED`；橙色圆圈/短箭头和 `S` 表示 `SUSPECTED`；暗色为 `TENTATIVE`（待确认）或 `UNVERIFIED`（缺结构/完整边界证据），灰色为 `HELD`。`E编号` 用于观察同一入口是否持续关联。白圈只是路线建议，**不是控制目标**；青色箭头表示巡航或右转控制获准的目标；右转是否接管另看 behavior。新增图底部说明区域，不改变地图比例。

日志新增 `exits=`、`suggestion=`、`preferred=`，以及每个入口的编号、相对接近走廊的 LEFT/FORWARD/RIGHT、状态、车体位置、方向、宽度、纵深可见比例和年龄。顺时针路线默认优先推荐已确认、当前可见的右侧入口，其次前向、左侧；`LEFT_AVAILABLE` 只说明看到左侧入口，**不证明其他方向被堵或必须左转**。`HELD` 和未确认入口不参与建议。编号在重置后不复用；短时遮挡过长后可能取得新编号。

局限：这是局部入口几何假设，不是完整拓扑地图或路口语义分类器。宽门/房间可能满足这些几何条件，仍须通过录制片段核对；斜向假设也不等于精确拟合出口航向。短 Z 形路段不要求回中，但仍需要可见的入口证据及有效接近坐标，不能保证墙角遮挡时提前看见第二弯。本版右转行为会另行生成路径并检查短程整车扫掠，黄色出口本身不保证可转弯。左侧出口、无法接入的圆角、未知/遮挡及不满足半径等情况仍会停车，不能仅凭前方空旷或 preferred_id 强行通过。短 Z 的左转和连续衔接留待下一阶段。

参数均为 `exit_` 前缀，完整默认值见 YAML，字段中文注释见 `ExitConfig`：

| 参数（省略 exit_） | 含义 |
|---|---|
| `enabled`、`prefer_right` | 开关及顺时针路线的右侧建议优先级；关闭出口识别也就没有新右转触发证据 |
| `search_front_m`、`search_rear_m`、`sample_step_m` | 侧向入口搜索前/后范围和采样间隔，4.0/0.3/0.1 m |
| `min_width_m`、`max_width_m` | 可见入口宽度范围，1.0–3.5 m，不代表车辆转弯所需空间 |
| `probe_depth_m`、`min_depth_visible_ratio` | 纵深 1.0 m、至少 15% 入口宽度具有该纵深观测 |
| `forward_gate_m`、`clearance_m` | 前向截面起点 1.3 m、可见性余量 0.08 m |
| `direction_offset_deg` | 侧向正交方向之外尝试 ±20° |
| `wall_band_m`、`min_wall_points`、`min_wall_span_m` | 真实边界支持带宽/点数/跨度，0.25 m / 5 / 0.45 m |
| `confirm_frames`、`confirm_time_s` | 连续确认门槛，3 帧且至少 0.15 s |
| `match_distance_m`、`match_angle_deg` | 世界坐标关联容差，0.8 m / 30° |
| `hold_time_s`、`hold_travel_m` | 不可见入口编号保留，0.5 s / 0.5 m |
| `context_timeout_s`、`context_travel_m` | 接近走廊参考保持，3.0 s / 2.0 m，仅诊断使用 |
| `max_update_gap_s` | 超过 0.5 s 扫描间隔则重建跟踪 |
| `corner_enabled`、`corner_min_angle_deg`、`corner_max_angle_deg` | 墙端疑似开口开关、侧偏角范围 20–85° |
| `corner_depth_m`、`corner_jump_m` | 超出墙端的可见深度 0.6 m、相邻箱测距跳变 0.7 m |
| `corner_wall_radius_m`、`corner_wall_span_m` | 墙端真实点邻域半径 0.6 m、线段跨度下限 0.35 m；误差上限为 wall_band_m / 3 |
| `corner_match_distance_m`、`corner_match_angle_deg` | 疑似墙端与后续标准入口的关联容差，1.2 m / 60°；升级仍需重新确认 |

### 有界离线分析

正常 GUI 回放命令不变。另有不启动节点、不发布控制的分析工具，只读取 `.db3` 中指定时间段的独立 `/scan` 与 `/encoder_imu_node/odom` 小消息，跳过 AllSensor 图像载荷：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/process_node/tools/analyze_exits.py \
  /data/data_whole/rosbag2_2026_10_06-18_13_20/rosbag2_2026_10_06-18_13_20_0.db3 \
  --offset 0 --duration 30 --output /tmp/exit_analysis
```

输出 `frames.csv`、`summary.json` 和少量诊断 PNG。修改 offset 选择路口片段；每次最多 120 s / 20000 条小消息。重复分析建议换一个 output 目录，避免混入此前保存的 PNG。该工具使用 dataclass 默认配置，不读取现场 YAML；结果不能直接等同于 AllSensor 实时回放或实车通过率。输出放仓库外，不提交大型实验结果。

可加 `--scan-stride 4` 每四帧扫描处理一帧，保留全部里程计，检查主动跳帧后的行为。CSV 新增 `scan_dt_s`、`fitted_walls`、`corridor_reason`、`yaw_deg`，入口记录含 kind/heading_deg。实际节点日志也增加 `scan_dt`（相邻被处理扫描的消息时间差）及 `fitted_walls`。`CORRIDOR_ACQUIRING` 表示当前已找到双墙但尚未完成确认；此时不画棕色模型线，并不等于没有拟合出墙。

2026-10-22 验证：原功能及新增几何/节点测试共 213 项通过；覆盖前方开放时右侧分支、双侧丁字路口、斜向拐角、窄门洞、无回波/未知区域、障碍遮挡、短 Z 形反向出口、位姿补偿、确认/失效，以及开关诊断后的控制输出一致性。当前课程 bag 的独立扫描局部分析：0–30 s 的 361 帧中，初始化后 356 帧建议前向，未产生已确认侧向出口；110–140 s 有 62 帧右侧建议；250–280 s 有 30 帧左侧、33 帧右侧建议（约 254.54 s / 260.37 s 分别开始确认）。这些是几何检测结果，并非人工标注的识别准确率。四段共读取约 22 MB 小消息载荷，没有完整读取 6 GB 图像数据。结果留在本机 `/tmp/exit_diagnosis`。

另外开启实际 VS Code/X11 窗口，以 AllSensor 回放约 20 s：处理 182 帧，反馈全部有效，无 `UNALIGNED_ODOM`，窗口正常显示；观察到的最大接收至回调完成时间约 0.35 s。该测试验证实时接收与图形负载，不能替代转弯控制的实车验证。

### v4.1 修正验证与剩余输入限制

226 项功能测试通过，新增覆盖 0.31/0.34/0.49 s 跳帧的连续双墙、单墙宽度期限、无观测/大间隔仍重置，及墙端斜向线索、左右镜像、扫描接缝、无走廊上下文、疑似升级后重新确认和编号关联。

课程 bag 的 298–310 s 直道，用 `--scan-stride 4` 保留全部里程计、每四帧处理一帧：相同 36 帧，旧代码为 24 帧 ACQUIRING / 12 帧 TRACKING，新代码为 2 帧初始确认 / 34 帧持续跟踪。该处约 303.39 s 帧与 straight.png 的点数/inf 数/候选数吻合，双墙宽约 1.75 m，本身可正常拟合。三角形拐角片段约 281.88 s 已出现右前方 SUSPECTED，早于 corner.png 对应的约 284.13 s；该截图位置也有疑似线索。v4.1 当时尚无转弯控制，疑似线索不参与建议。

另以安装后的节点开启真实 GUI 回放后段，处理 64 帧、反馈全部有效、窗口正常，但因处理扫描间隔超过原 0.5 s 门槛仍有重新确认，最大间隔约 0.92 s。这不是 0.3 s 无墙期限误用。进一步检查录制时间：298–313 s 的 AllSensor 有 11 次间隔超过 0.5 s（最大约 0.94 s），独立 /scan 最大间隔约 0.097 s；AllSensor 成批到达时，最新帧策略还可能跳过中间帧。保留大间隔、反馈和超时保护，不能据此取消保护或宣称 GUI 回放箭头已完全连续。现场应检查实时 AllSensor 的间隔，而非照搬录制中的发布节奏。CSV 和图片留在仓库外 `/tmp/exit_fix_v41`。

## 控制器如何工作

**方向 PD** 的误差是选中目标相对车头的角度（rad）。corridor 模式已在上游将航向和横向回中合成为参考角，PD 本身仍使用同一角度接口：

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

三个参考模式可用于分阶段测试：

| `reference_mode` | 行为 |
|---|---|
| `corridor`（默认） | 按走廊走向和横向偏移生成参考，只选择参考附近合格候选；需要 closed_loop 和有效独立反馈 |
| `perception` | 使用上一版本的局部评分选方向，供对照；不计算中线纠偏 |
| `straight` | 仅接受感知检查合格的 0° 候选，没有则 `NO_STRAIGHT_PATH` 停车；方向误差为零，用于中位和速度 PI 的直路测试 |

`straight` 不锁定初始全局航向，也不计算走廊中线误差；若车辆因中位偏差逐渐偏航，它不会根据绝对航向纠偏。初步中位/速度标定使用 straight，直走廊回正/回中使用 corridor，旧评分对照使用 perception。三种模式均不能证明现有直线包络对应实际车辆转弯轨迹。

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
  -p use_sim_time:=true -p reference_mode:=perception \
  -p control_mode:=baseline -p require_motion_feedback:=false
```

此组合只供离线观察；它仍会发布旧基线控制消息，不应连接实车驱动使用。闭环模式不允许关闭反馈检查，启动时会拒绝该配置。

有完整反馈但要对比上一版本的方向评分时，使用相同命令并保留 closed_loop，只覆盖 `-p reference_mode:=perception`。原 `ros2 launch ... use_sim_time:=true` 命令现在默认进入 corridor，无需新增启动参数。

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
- **Laser Diagnostics**：车头朝上，车体左侧在屏幕左侧；白点是真实有效测距，蓝点在有限规划上限处标记无回波，红色外圈标记其他无效/未知方向，橙色点是补齐的短缺口。蓝点和红点均不表示那里测到了障碍。
- 绿色弧线是粗略深度/宽度开口，仅供诊断，不再决定候选能否生成，所以有目标时也可能 `gaps=0`。绿色圆盘及平行线是被检查的车体扫掠包络；青色箭头仅在有目标且节点 `OK` 时显示。
- `valid` 是全帧真实有限回波比例。选中窗口的 `support` 为有限回波加受限无回波的合计比例，`finite` 为真实有限回波比例，`no_return` 为 `+inf` 比例；`planning_mean` 是可用射线的有限规划距离均值。`no_return_limit` 是本帧无回波范围上限，`front` 是可能包含无回波假设的直行规划净空，`unknown` 表示覆盖不足。净空不是制动距离。
- 节点 `OK` 表示感知与控制门控均通过；`perception=OK` 仅表示选到了候选方向。`NO_OBSERVED_GAP` 表示候选窗口覆盖不足；`NO_CLEAR_PATH` 表示有覆盖但真实障碍或规划上限导致净空不足；`INSUFFICIENT_FINITE_SUPPORT` 表示整帧真实有限回波比例低于门槛；`NO_VALID_RANGES`/`INVALID_METADATA` 表示输入不可用，`TOO_CLOSE` 表示近于最小量程。这些保护状态均输出停止值。
- `WAITING` 表示尚未收到数据，`STALE_DATA` 表示超过输入超时。暂停/播完后会显示缓存的末帧并撤去活动目标，独立定时器继续处理窗口事件。
- `odom=OK` 表示反馈可用；`WAITING_ODOM`/`INITIALIZING_ODOM` 表示等待输入或差分起点尚未就绪；`STALE_ODOM` 表示有效估计接收超时；`UNALIGNED_ODOM` 表示反馈与激光时间差过大；`INVALID_ODOM`、`DUPLICATE_ODOM`、`ODOM_TIME_REWIND`、`ODOM_TIME_GAP`、`ODOM_POSE_JUMP` 分别标记输入或差分连续性问题。默认门控开启时，不可用反馈会阻止行驶，定时器也会独立检测反馈超时。
- 新日志 `v/raw_v` 为滤波/原始物理速度（m/s），`yaw/turn` 为当前/累计航向（度），`travel/signed` 为累计路程/有符号行程（m），`local` 为局部位置（m），`dt` 为差分间隔，`skew` 为关联反馈时间减激光时间（s）。反馈无效时仍可能显示缓存值，但它们仅供诊断，不能用于控制。
- 闭环日志增加 `v_ref`（斜坡后目标）、`requested`（请求目标）、`v_used`（本次控制使用的关联速度）、`e_v`、动力 `[FF,P,I]`、转向 `[P,D]`、`control_dt`，以及幅值饱和 `sat`、变化率限制 `rate`、冻结积分 `I_frozen` 和撤去微分 `D_reset` 标记。激光图也显示目标/反馈速度及 P/I/D 项。
- 巡航图新增棕色墙线模型、紫色中线/参考目标；青色仍是实际获准执行的目标，最后绘制避免被参考覆盖。紫色模型出现不代表控制已获准。`axis` 是走廊方向，`center_left` 正表示中线在左、负表示中线在右，`width` 是走廊宽度；日志还显示 `confidence`、观测年龄 `age`、双墙年龄 `pair_age` 和 `speed_scale`。传入出口诊断时图高为 1060 像素，增加底部入口信息；不传时仍为 900 像素。
- `CORRIDOR_TRACKING` 表示采用双墙，`CORRIDOR_SINGLE_WALL`/`CORRIDOR_HOLD` 表示限时降速；这三个是参考状态，节点还需通过控制器才能 `OK`。`CORRIDOR_ACQUIRING`、`CORRIDOR_LOST`、`CORRIDOR_OUT_OF_RANGE`、`CORRIDOR_TIME_INVALID`、`CRUISE_BLOCKED` 均停止。参考置信分数不等于碰撞检查结果。
- `CONTROL_WARMUP` 为有效输入恢复后的首帧等待；`CONTROL_TIME_INVALID`/`CONTROL_TIME_GAP` 为控制时间不连续；`STOP_REQUESTED` 包括零目标速度。这些状态均停止。录制数据中车辆速度不受本次输出影响，实测速度高于斜坡目标时动力输出为停止值是可能的，不能据此判断实车调速效果。
- 按 q、Esc 或关闭任一窗口会发送停止值并退出节点，Ctrl+C 也可退出。输入超时还会周期性发送停止值；这些值的物理效果仍须在实车上确认。

## 主要参数

全部默认值见 `config/baseline.yaml`；直接 `ros2 run` 的内置默认值与其一致。车体尺寸来自 2026-10-23 用户测量，其余默认阈值和控制参数仍为离线调试起点。

| 参数 | 含义 |
|---|---|
| `sensor_yaw_offset_deg` | 将雷达坐标旋转到车体坐标的偏角，正值逆时针 |
| `vehicle_width_m`、`front_overhang_m`、`rear_overhang_m` | 车宽，以及雷达到前后端的距离；计算圆盘时取最远车角 |
| `safety_margin_m` | 车体包络外的额外裕量 |
| `lookahead_m`、`max_distance_m` | 需检验的前行距离、有限评分/显示上限 |
| `no_return_distance_m` | 无回波的有限规划范围，默认 5 m，受本帧 range_max 限制；0 禁用，不改变原始 ranges |
| `min_scan_finite_ratio` | 整帧真实有限回波最低比例，默认 0.05；即使设 0，全无有限回波仍停止 |
| `front_half_angle_deg`、`bin_angle_deg` | 前向搜索半角、分箱角宽 |
| `min_bin_valid_ratio`、`min_window_valid_ratio` | 分箱、候选窗口内有限回波与受限无回波的最低合计可用比例，沿用旧参数名 |
| `max_interpolate_gap_deg`、`interpolate_range_jump_m` | 允许补齐的最长短缺口及两端最大距离差；前者设 0 可禁用补齐 |
| `max_inferred_ratio` | 选中窗口内补齐分箱的最高比例 |
| `forward_weight`、`continuity_weight` | 前行倾向和目标连续性的评分权重 |
| `control_mode` | 默认 closed_loop；baseline 用于旧固定动力基线对照 |
| `reference_mode`、`target_speed_mps` | 默认 corridor，可选 perception/straight；正常物理请求速度默认 0.2 m/s，0 直接停车 |
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
| `input_odom_queue_size` | 独立接收线程的里程计缓存上限，默认 200 条，至少 2；溢出后重建反馈 |
| `visualization_period_s` | 图像最大刷新频率的倒数，默认 0.1 s（10 Hz）；事件和超时检查独立持续进行 |
| `group_label` | 小组/版本文字标记；图中建议使用英文或数字，OpenCV 默认字体不支持中文 |

走廊参数均带 `corridor_` 前缀，默认值见 YAML，完整字段注释见 `CorridorConfig`：

| 参数（以下省略 corridor_ 前缀） | 含义 |
|---|---|
| `sensor_x_m`、`sensor_y_m` | 雷达相对 odom_child_frame_id 的安装平移，前/左为正；默认 0，上车必须核验 |
| `fit_range_m`、`fit_front_m`、`fit_rear_m`、`voxel_m` | 拟合最大真实测距、前后区域边界和网格采样尺寸，默认 4.5/4.0/0.3/0.05 m |
| `ransac_trials`、`max_lines` | 固定种子采样次数和候选墙线数，默认 80/6 |
| `line_tolerance_m`、`min_inliers`、`min_span_m`、`max_rmse_m` | 内点距离门槛、采样后支持点数、沿墙跨度、拟合均方根误差，默认 0.06 m/12/1.2 m/0.04 m |
| `min_wall_distance_m`、`max_heading_deg` | 仅拟合时的墙面最小距离及走向范围，默认 0.25 m/45°；不移除碰撞检查中的近点 |
| `parallel_tolerance_deg`、`min_width_m`、`max_width_m` | 双墙平行容差和合理宽度范围，默认 8°、0.9–3.5 m |
| `max_heading_change_deg`、`max_center_change_m`、`max_width_change_m` | 运动补偿后观测跳变门槛，默认 12°/0.20 m/0.25 m |
| `confirm_frames`、`filter_tau_s`、`max_update_gap_s` | 初始化连续确认帧数、低通时间常数、扫描长间隔重建门槛，默认 3/0.25 s/0.5 s |
| `single_wall_timeout_s`、`hold_timeout_s`、`max_unobserved_travel_m` | 单墙最久使用时间、无墙历史保持时间和无新观测最大位移，默认 0.6 s/0.3 s/0.2 m |
| `goal_distance_m`、`lateral_gain` | 沿走廊放置目标点的距离及横向回中系数，默认 1.5 m/1.0 |
| `max_lateral_correction_deg`、`max_target_deg`、`candidate_tolerance_deg` | 横向修正角上限、巡航总角度上限、候选偏离参考的容差，默认 20°/45°/5° |
| `recovery_heading_deg`、`recovery_offset_m`、`slow_speed_scale` | 较大误差的降速门槛及请求速度比例，默认 15°/0.25 m/0.5 |

## 验证

在工作区根目录加载环境后执行：

```bash
python3 -m pytest src/process_node/test/test_perception.py \
  src/process_node/test/test_corridor.py src/process_node/test/test_node_corridor.py \
  src/process_node/test/test_motion.py src/process_node/test/test_control.py \
  src/process_node/test/test_pid_controller.py src/process_node/test/test_node_closed_loop.py \
  src/process_node/test/test_node_baseline.py src/process_node/test/test_window_monitor.py \
  src/process_node/test/test_input_buffer.py \
  src/process_node/test/test_exits.py src/process_node/test/test_node_exits.py \
  src/process_node/test/test_turning.py src/process_node/test/test_node_turning.py -q
```

测试覆盖扫描点数和角度排列变化、数组首尾、无回波与其他非有限值的区别、短缺口/长未知段、过窄开口、inf 邻近孤立近点、量程与规划上限、整帧回波不足、左右镜像、安装偏角、运动反馈单位/符号、航向接缝、时间对齐、反馈失效停车、控制限幅、超时和窗口关闭。`test/data/corridor_start_scan.npz` 保留约 4.9 KB 的录制首帧雷达用于回归，无需安装完整 bag。录制数据回放只验证输入解释与决策计算，不代表已经完成实车闭环绕障。

2026-10-22 输入调度修复：此前单线程将订阅、墙线拟合和图形转发放在一起，用户 GUI 回放中反馈积压约 0.2–0.9 s，反复出现 `UNALIGNED_ODOM` 并清除走廊参考。现已分离接收、批量更新反馈并限制 GUI 重传，容差未放宽。包含 `test_input_buffer.py` 的十个测试文件合计 **181 项通过**，新增检查覆盖最新帧替换、队列溢出、真实接收时间、批量关联、旧扫描拒绝、发布前过期检查和空闲窗口事件。

同一容器的 VS Code/X11 显示环境中，开启两个真实窗口、隔离 ROS 域回放开头约 30 s：处理 316 帧，316 帧反馈均 `OK`，未出现 `UNALIGNED_ODOM`；关联差范围约 −11.7～+12.8 ms。313 帧采用双墙、1 帧单墙、2 帧初始确认；286 帧正常控制、25 帧原近点包络停止、其余 5 帧确认/预热。第 10–30 秒处理的 211 帧全部正常控制；缓冲主动跳过 35 个中间扫描，最大接收至回调结束年龄约 0.25 s。播放停止后正常超时停车并退出。帧数受实时负载影响，不与逐帧离线统计直接比较，也不是实车通过率。

2026-10-22 此前巡航参考版本已构建成功，原九个测试文件合计 **174 项通过**。同一开头 30 秒的 359 帧/1501 条反馈，356 帧为双墙参考、1 帧为单墙参考，最初 2 帧用于连续确认。最终 326 帧 `OK`、3 帧控制预热、2 帧走廊确认、28 帧因原近点包络检查停止；10–30 秒均正常控制。约 20–25 秒估计中线位于车体右侧 0.10–0.23 m，生成约 −6.27°至 −1.65°的右回中目标，而上一版局部评分此时多为接近 0°。这是基于默认安装参数的估计与参考计算，不能当作横向位置真值或实车回中结果。扫描回调（不含 GUI）耗时中位约 40 ms、95 分位约 45 ms。

本轮测试还覆盖偏航/横向偏移符号、安装平移、跨 ±π 的转动和平移补偿、门洞与稀疏离群点、单墙期限、历史超时/位移期限、突变拒绝、精确参考候选检查、巡航受阻与反馈看门狗清除参考。

另在隔离 ROS 域 175 中，用安装后的 launch/YAML 和实际 `ros2 bag play --clock` 回放开头约 30 秒（关闭 GUI），确认默认 corridor 启动、墙线/参考进入控制、请求与反馈诊断，以及停止播放后的 `STALE_ODOM`/`STALE_DATA` 停车和正常退出。启动时曾短暂出现消息到达顺序导致的 `UNALIGNED_ODOM`，节点保持停止，反馈恢复后重新确认并预热；没有通过放宽时间容差绕过它。上述精确帧数属于按记录顺序的回调检查，实际 DDS 调度不保证逐帧结果完全相同。

2026-10-22 此前无回波修订已构建成功，原七个测试文件合计 **135 项通过**。保持前瞻 1.5 m 和原车体尺寸，同一开头 30 秒的无回波版本为 **328 帧 `OK`、3 帧预热、28 帧 `NO_CLEAR_PATH`**；0°合格候选从更早版本的 0 帧增加到 **331 帧**。前 10 秒的 28 帧停止均有真实近点落入默认圆盘包络（样例在约 167–177°后方、距离约 0.22 m）；尚不能据此判断是车体回波、人员还是其他物体，因此保留检查。该统计不是实车通过率。

下列 2026-10-20/21 的结果属于此前保守感知和 PD/PI 版本，保留用于记录接口演进，不代表本轮无回波版本仍会在开头因相同原因停止。

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
