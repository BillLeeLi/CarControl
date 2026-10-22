# 测距回归样本

`corridor_start_scan.npz`（约 4.9 KB）来自用户提供的课程录制数据
`rosbag2_2026_10_06-18_13_20`，bag 起点后约 0.064655 秒的第一帧
`/sync_store_node/allsensor` 内的 `urg`。仅保存测距与角度/量程元数据，不含图像、深度或运动字段。

数组 `ranges` 保留 1658 个原始下标及 154 个 `+inf`；其余键为
`angle_min`、`angle_increment`、`range_min`、`range_max`、相对 bag 起点的 `t`。
用于复现“直走廊正前方无回波导致没有候选”的问题，不是人工标注的完整通行真值。

测试检查正前方能进入候选且目标接近前向；车辆真实闭环响应仍须实车验证。
