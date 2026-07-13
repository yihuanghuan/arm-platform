# D435i 阶段 7.5 闭环实验前置修正报告

## 修正摘要

阶段 7 的 AprilTag 位置误差约 `1.6-1.7 cm`，根因是 Gazebo RGB-D sensor 实际挂载在 `camera_link`，但 RGB 图像、CameraInfo 和 AprilTag 检测使用 `camera_color_optical_frame`。官方 nominal extrinsics 中：

```text
camera_link -> camera_color_frame: xyz = 0 0.015 0
camera_color_frame -> camera_color_optical_frame: xyz = 0 0 0
```

本阶段将 RGB-D sensor 的 Gazebo reference 改为 `camera_color_frame`。该 frame 与 `camera_color_optical_frame` 原点重合，同时保持 Gazebo camera `+X` 前向与 ROS optical `+Z` 前向之间的既有旋转约定。IMU 仍挂载在 `camera_link`，不属于本阶段 RGB/AprilTag 修正范围。

## EE 相机回归

启动：

```bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false \
  camera_mount_mode:=ee \
  control_mode:=kinematic_visualization
```

验证命令：

```bash
ros2 run manipulator check_apriltag_ground_truth.py \
  --duration 8 --sample-hz 5 \
  --output-csv /tmp/d435i_phase75_ee_poseN.csv
```

| 场景 | 关节命令 `[j1..j6]` rad | 检测率 | 丢失率 | 位置误差均值 | 姿态误差均值 | CSV |
|---|---:|---:|---:|---:|---:|---|
| pose0 | `[0, 0, 0, 0, 0, 0]` | `15.114 Hz` | `0.000` | `0.00159 m` | `1.376 deg` | `/tmp/d435i_phase75_ee_pose0.csv` |
| pose1 | `[0.20, -0.25, 0.18, 0.0, -0.10, 0.0]` | `16.226 Hz` | `0.000` | `0.00101 m` | `0.455 deg` | `/tmp/d435i_phase75_ee_pose1.csv` |
| pose2 | `[-0.18, -0.35, 0.28, 0.12, -0.18, 0.10]` | `15.101 Hz` | `0.000` | `0.00113 m` | `0.563 deg` | `/tmp/d435i_phase75_ee_pose2.csv` |

结论：RGB sensor 原点修正有效，三姿态位置误差由阶段 7 的 `1.6-1.7 cm` 降至约 `1-2 mm`，满足 `< 0.005 m` 目标。姿态误差 pose0 和 pose2 未满足 `< 0.5 deg` 目标，应继续定位 AprilTag PnP、渲染分辨率、纹理采样或模型 frame 约定造成的残差。

## 动态 Ground Truth

新增：

```text
scripts/dynamic_ground_truth_logger.py
```

方法：

- 节点默认使用 `use_sim_time:=true`，检测样本使用 `/apriltag/detections.header.stamp`。
- GT buffer 保存最近 `10 s` 的 `world -> base_link`、`world -> link6`、`world -> camera_color_optical_frame` 和静态 `world -> tag`。
- 平移线性插值，姿态使用 quaternion SLERP。
- 无法按检测时间戳匹配的样本标记 invalid，不用 latest TF 强行替代。
- 同时记录 `/joint_states`、`/arm_velocity_controller/commands` 和 `/apriltag/detections`。

动态验证启动：

```bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false \
  camera_mount_mode:=ee \
  control_mode:=physical_dynamics
```

验证结果：

```text
messages: 315
detections_with_id_0: 315
detection_rate_hz: 15.749
loss_rate: 0.000
gt_timer_frequency_hz: 100.000
camera_gt_buffer_frequency_hz: 100.000
valid_samples: 299
invalid_samples: 16
new_position_error_mean_m: 0.00134
new_position_error_std_m: 0.00036
new_orientation_error_mean_deg: 0.729
gt_match_error_max_sec: 0.01200
old_position_error_mean_m: 0.00143
position_error_improvement_m: 0.00010
csv: /tmp/d435i_phase75_dynamic.csv
```

说明：`/joint_states` 为 `99.99 Hz`，`/tf` 实测约 `75 Hz`，`/clock` topic 实测约 `10 Hz`。为避免 ROS-time timer 受低频 `/clock` 限制，logger 使用 steady wall timer 触发 100 Hz 采样，但所有样本时间仍按仿真 TF/detection stamp 评价。最大匹配误差 `12 ms` 略高于一个 100 Hz GT 周期，需要在进入高精度动态闭环前继续压低。

## Base 相机回归

新增：

```text
scripts/check_camera_mount_regression.py
```

启动：

```bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false \
  camera_mount_mode:=base \
  control_mode:=kinematic_visualization
```

验证结果：

| 场景 | base-camera 平移变化 | base-camera 姿态变化 | 检测率 | 丢失率 | 位置误差均值 | 姿态误差均值 |
|---|---:|---:|---:|---:|---:|---:|
| pose0 | `0 m` | `0 rad` | `15.125 Hz` | `0.000` | `0.00190 m` | `5.1e-08 deg` |
| pose1 | `0 m` | `0 rad` | `15.000 Hz` | `0.000` | `0.00190 m` | `5.1e-08 deg` |
| pose2 | `0 m` | `0 rad` | `15.125 Hz` | `0.000` | `0.00190 m` | `5.1e-08 deg` |

CSV：

```text
/tmp/d435i_phase75_base_regression.csv
```

结论：Base 模式下 `base_link -> camera_color_optical_frame` 不随机械臂关节变化，满足 `1e-6 m` / `1e-6 rad` 外参恒定要求；检测率、丢失率、位置误差和姿态误差均满足阶段目标。

## 验证与进入闭环建议

已通过：

- `python3 -m py_compile` 检查新增脚本；
- `check_urdf /tmp/d435i_phase75.urdf`；
- `colcon build --packages-select manipulator --symlink-install`；
- EE/Base AprilTag headless Gazebo 实测；
- 动态 GT logger 20 s 低速小幅正弦运动实测。

未完全达标：

- EE pose0/pose2 姿态误差高于 `< 0.5 deg` 目标；
- 动态 GT 最大匹配误差 `0.012 s`，略高于 100 Hz 的一个周期。

建议：可以进入只依赖平移误差的闭环原型验证；不建议直接进入高精度 6D 位姿闭环或姿态闭环，需先继续定位 EE 姿态残差和动态时间匹配边界。
