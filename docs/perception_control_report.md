# Perception-Control 阶段 0：现有仿真基线冻结报告

## 目标与结论

本阶段只冻结并验证现有仿真基线，不加入任何控制算法。验证入口以 moving-base baseline 为准：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase0_sine_x.csv \
  replay_output_csv:=/tmp/windylab_phase0_sine_x_replay.csv \
  start_delay_sec:=4.0
```

结论：

- Gazebo Classic 中机械臂、D435i、RGB-D、IMU、AprilTag 场景和 ROS 2 control baseline 均可启动。
- D435i 通过 URDF 固定挂载在 `link6` 后，运行时 `link6 -> camera_color_optical_frame` TF 稳定。
- RGB、Depth、PointCloud2、IMU 和 AprilTag detection 均正常发布。
- `/joint_states` 数据由 Gazebo ROS 2 control 的 `joint_state_broadcaster` 产生；本次运行没有 `student_arm_node`。
- `/arm_velocity_controller/commands` 可控制六个关节；baseline 实验中由零速度节点保持关节不动。
- Base 扰动 CSV 可按 100 Hz 回放，Gazebo entity state 记录显示无失败、跟踪误差很小。

## 构建与静态检查

构建：

```bash
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
```

结果：`manipulator` 构建成功。

URDF 检查：

```bash
xacro $(ros2 pkg prefix manipulator)/share/manipulator/arm_with_d435i.urdf.xacro \
  camera_enabled:=true rgbd_enabled:=true rgbd_frame_name:=camera_color_optical_frame \
  use_ros2_control:=true fix_base_to_world:=false \
  > /tmp/windylab_phase0.urdf
check_urdf /tmp/windylab_phase0.urdf
```

结果：`check_urdf` 成功解析。`fix_base_to_world:=false` 时 root link 为 `base_link`，TF 链包含：

```text
base_link -> link1 -> link2 -> link3 -> link4 -> link5 -> link6
link6 -> camera_bottom_screw_frame -> camera_link -> camera_color_frame -> camera_color_optical_frame
```

阶段 0 扰动轨迹：

```bash
ros2 run manipulator generate_base_disturbance.py \
  --config src/arm-platform/config/base_disturbance_profiles.yaml \
  --profile sine_x \
  --duration-sec 20 \
  --sample-rate-hz 100 \
  --output-csv /tmp/windylab_phase0_sine_x.csv
```

结果：生成 `2001` 个样本，CSV 行数为 `2002`。

## Baseline 运行结果

Base 扰动回放摘要：

```text
samples: 2001
target_rate_hz: 100.000
actual_rate_hz: 100.014
set_failures: 0
joint_max_step_rad: 0.000000000
joint_final_discontinuity_rad: 0.000000000
pose_tracking_error_mean_m: 0.000005003
pose_tracking_error_max_m: 0.000218801
csv: /tmp/windylab_phase0_sine_x_replay.csv
```

控制器检查：

```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
ros2 topic echo --once /arm_velocity_controller/commands
ros2 topic hz /arm_velocity_controller/commands
```

结果：

```text
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active
arm_velocity_controller velocity_controllers/JointGroupVelocityController active
joint1..joint6 velocity command interfaces: available, claimed
joint1..joint6 position/velocity/effort state interfaces: present
/arm_velocity_controller/commands sample: [0, 0, 0, 0, 0, 0]
/arm_velocity_controller/commands observed rate: about 20 Hz in this sim-time run
```

`/joint_states` 检查：

```bash
ros2 node info /joint_state_broadcaster
ros2 topic echo --once /joint_states
ros2 topic hz /joint_states
```

结果：

- `/joint_state_broadcaster` 发布 `/joint_states` 和 `/dynamic_joint_states`；
- 节点列表中没有 `student_arm_node`；
- `/joint_states` 实测约 `99.98 Hz`；
- sample 中六个关节位置和速度均在 `1e-13` 量级附近，baseline 保持初始构型。

说明：本机本次运行中 `ros2 topic info /joint_states --verbose` 曾报告 `Publisher count: 0`，但 `ros2 node info /joint_state_broadcaster`、`ros2 topic echo` 和 rosbag 均确认 `/joint_states` 数据由 `joint_state_broadcaster` 输出。因此阶段 0 以 node graph 和实际消息流为准。

TF 检查：

```bash
ros2 run tf2_ros tf2_echo link6 camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link link6
```

结果：

```text
link6 -> camera_color_optical_frame:
  translation [0.071, 0.033, 0.053]
  rpy [-90.000, -0.000, -90.000] deg

base_link -> link6:
  translation [0.353, 0.002, 0.374]
  rpy [0.000, -0.000, -0.000] deg
```

Moving-base baseline 使用 `fix_base_to_world:=false`，robot TF 树没有 `world` root；`world -> base/link6/camera proxy` 由 Gazebo `/get_entity_state` 写入 replay CSV，而不是通过 TF 发布。

## 感知与 rosbag

AprilTag 节点使用现有配置单独启动，因为 `moving_base_stabilization.launch.py` 只启动 Gazebo baseline，不包含 `apriltag_ros`：

```bash
ros2 run apriltag_ros apriltag_node --ros-args \
  -r __ns:=/apriltag \
  --params-file $(ros2 pkg prefix manipulator)/share/manipulator/apriltag_36h11_00000.yaml \
  -p use_sim_time:=true \
  -r image_rect:=/d435i/color/image_raw \
  -r camera_info:=/d435i/color/camera_info
```

Topic 和频率：

```text
/d435i/color/image_raw: about 8.7 Hz in the 8 s sample
/d435i/depth/image_raw: about 14.8 Hz
/d435i/depth/points: about 14.8 Hz
/apriltag/detections: about 15.0 Hz
/d435i/imu: recorded in rosbag
```

CameraInfo / PointCloud2 / IMU sample：

```text
/d435i/color/camera_info:
  width=640, height=480, frame_id=camera_color_optical_frame
  K[0]=462.2663372707335, K[4]=462.2663372707335

/d435i/depth/points:
  width=640, height=480, frame_id=camera_color_optical_frame
  point_step=32, row_step=20480, is_dense=true

/d435i/imu:
  frame_id=camera_accel_optical_frame
  linear_acceleration.z ~= 9.8
```

AprilTag 检测：

```bash
ros2 run manipulator check_apriltag_ground_truth.py \
  --duration 8 --sample-hz 5 \
  --output-csv /tmp/windylab_phase0_apriltag_validation.csv
```

结果：

```text
messages: 131
detections_with_id_0: 131
detection_rate_hz: 16.293
loss_rate: 0.000
samples: 0
csv: /tmp/windylab_phase0_apriltag_validation.csv
```

`samples: 0` 的原因是该 helper 默认要求 `world` TF frame，而 moving-base baseline 没有发布 `world` TF。检测链路本身有效，`/apriltag/detections` 持续输出 `tag36h11`、`id=0`、`hamming=0`。

Rosbag：

```bash
ros2 bag record -o /tmp/windylab_phase0_baseline_bag \
  /clock /joint_states /tf /tf_static \
  /d435i/color/image_raw /d435i/color/camera_info \
  /d435i/depth/image_raw /d435i/depth/camera_info /d435i/depth/points \
  /d435i/imu /apriltag/detections \
  /arm_velocity_controller/commands /model_states /link_states
```

结果：

```text
path: /tmp/windylab_phase0_baseline_bag
duration: 21.793 s
size: 3.7 GiB
messages: 11110
/joint_states: 2156
/apriltag/detections: 324
/d435i/color/image_raw: 324
/d435i/depth/image_raw: 324
/d435i/depth/points: 324
/d435i/imu: 4318
/arm_velocity_controller/commands: 436
/model_states: 211
/link_states: 210
```

## 速度控制器 Spot Check

为验证 `/arm_velocity_controller/commands` 能控制六个关节，单独启动无 baseline publisher 的外部速度源实验：

```bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=false use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_apriltag_test.world \
  camera_enabled:=true rgbd_enabled:=true rgbd_frame_name:=camera_color_optical_frame \
  imu_enabled:=true use_ros2_control:=true \
  control_mode:=physical_dynamics velocity_command_source:=external \
  fix_base_to_world:=true
```

命令：

```bash
ros2 topic pub --rate 20 --times 20 \
  /arm_velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.2, -0.2, 0.15, -0.15, 0.1, -0.1]}"

ros2 topic pub --once \
  /arm_velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

结果：

```text
before joint positions:
  [0, 0, 0, 0, 0, 0]

after joint positions:
  joint2=-0.3710188855
  joint3= 0.2557099820
  joint1= 0.3775564008
  joint4=-0.1940107063
  joint5= 0.1615770737
  joint6=-0.2425272984

after velocity:
  all joints near 0 after zero command
```

说明：`/joint_states.name` 的输出顺序为 `joint2, joint3, joint1, joint4, joint5, joint6`，因此评价时按名称读取，不按数组位置硬编码。

## 阶段 0 冻结产物

```text
/tmp/windylab_phase0_sine_x.csv
/tmp/windylab_phase0_sine_x_replay.csv
/tmp/windylab_phase0_apriltag_validation.csv
/tmp/windylab_phase0_baseline_bag/
```

本阶段未修改控制算法、未新增闭环节点、未改变 Gazebo/ROS 2 control 逻辑。后续阶段可以在此 baseline 上继续实现视觉末端位姿标准化。

# Perception-Control 阶段 1：标准化视觉末端位姿输出

## 目标与结论

本阶段新增视觉末端位姿标准化节点，将 AprilTag 检测链路转换为控制器可直接订阅的 `world -> link6` 末端位姿输出：

```text
/visual_ee_pose        geometry_msgs/PoseStamped
/visual_ee_pose_valid  std_msgs/Bool
/visual_ee_pose_debug  std_msgs/String(JSON)
```

结论：

- `/visual_ee_pose.header.frame_id` 固定为 `world`；
- `/visual_ee_pose.header.stamp` 使用对应 `/apriltag/detections.header.stamp`；
- 三个静态关节构型下位置误差均值 `0.000497 m`，最大 `0.000716 m`；
- 检测丢失率 `0.000`，视觉位姿 sample loss `0.000`；
- 当前阶段只标准化感知输出，不发布关节命令，不接入闭环控制；
- 项目状态适合进入阶段 2：迁移并 dry-run 验证 Pinocchio CLIK 控制算法。

## 修改内容

新增：

- `scripts/visual_ee_pose_estimator.py`
- `scripts/check_visual_ee_pose.py`

修改：

- `launch/d435i_apriltag_test.launch.py`
- `CMakeLists.txt`

`visual_ee_pose_estimator.py` 订阅 `/apriltag/detections`，筛选 `tag36h11/id=0`，并等待同一 detection stamp 的 `camera_color_optical_frame -> apriltag_36h11_00000` TF 到达后再处理，避免 TF 顺序导致的 future extrapolation。

节点默认参数：

```text
world_frame: world
base_frame: base_link
ee_frame: link6
camera_frame: camera_color_optical_frame
detected_tag_frame: apriltag_36h11_00000
tag_family: tag36h11
tag_id: 0
position_estimation_mode: kinematic_orientation
world_to_tag_xyz: 1.23042456 0.000976374 0.35065986
world_to_tag_rpy: -3.12204785 -1.56214388 3.12103003
detection_timeout_sec: 0.5
tf_timeout_sec: 0.1
```

说明：`world_to_tag_*` 表示 `apriltag_ros` 检测 TF 中 tag frame 的固定世界位姿，不是仅从 SDF 读取的 tag model nominal pose。阶段 7.5 已确认 AprilTag PnP 姿态仍有小残差；若直接使用完整 PnP 姿态反推出 `link6`，相机外参杠杆臂会把姿态残差放大为末端位置误差。因此阶段 1 默认使用 `kinematic_orientation` 模式：末端位置来自 AprilTag 视觉平移，末端/相机姿态来自当前 TF 运动学链路。`full_pose` 模式保留为参数选项，供后续阶段 6 定位和处理完整 6D 姿态误差。

## 验收命令

构建：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
```

启动：

```bash
source setup_env.bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false run_visual_ee_estimator:=true
```

Topic spot check：

```bash
ros2 topic echo --once /visual_ee_pose
ros2 topic echo --once /visual_ee_pose_valid
ros2 topic echo --once --full-length /visual_ee_pose_debug
ros2 topic hz /visual_ee_pose
```

观测结果：

```text
/visual_ee_pose.header.frame_id: world
/visual_ee_pose_valid: true
/visual_ee_pose_debug.reason: ok
/visual_ee_pose_debug.position_estimation_mode: kinematic_orientation
/visual_ee_pose: about 15 Hz
```

三构型验证：

```bash
source setup_env.bash
ros2 run manipulator check_visual_ee_pose.py \
  --use-sim-time \
  --output-csv /tmp/windylab_phase1_visual_ee_pose.csv
```

结果：

```text
samples: 45
valid_samples: 45
visual_loss_rate: 0.000
detection_messages: 227
target_detections: 227
detection_loss_rate: 0.000
visual_frames: ['world']
stamp_in_detection_history_fraction: 1.000
position_error_mean_m: 0.000497
position_error_max_m: 0.000716
position_error_std_m: 0.000156
csv: /tmp/windylab_phase1_visual_ee_pose.csv
```

按构型拆分：

```text
config 0: samples=15 mean=0.000414205 m max=0.000414205 m
config 1: samples=15 mean=0.000715896 m max=0.000715896 m
config 2: samples=15 mean=0.000361166 m max=0.000361166 m
```

## 已知限制

- 当前阶段优先满足位置闭环前置条件；姿态输出默认来自 TF 运动学姿态，不作为最终视觉 6D 姿态验收结果。
- `full_pose` 模式会暴露 AprilTag PnP 姿态残差，零位姿下末端位置误差可被放大到厘米级；阶段 6 前应继续定位姿态误差来源。
- `check_visual_ee_pose.py` 默认用 TF `world -> link6` 作为末端 frame ground truth；可用 `--ground-truth-source gazebo_entity` 查看 Gazebo link entity pose，但该 entity pose 与 ROS/URDF `link6` frame 存在固定 frame/origin 差异。

# Perception-Control 阶段 2：Pinocchio CLIK Dry-Run 控制节点

## 目标与结论

本阶段从 `~/westlake/WindyLab-RobotArmControl` 的末端稳定控制实现中迁移可复用的 Pinocchio 运动学计算，新增只读输入、只发布调试量的 dry-run 控制节点：

```text
visual_ee_stabilization_controller
```

结论：

- 控制节点订阅 `/joint_states` 和 `/visual_ee_pose`，并可使用 `/visual_ee_pose_valid` 做安全门控；
- 启动后首次收到有效视觉末端位姿时锁定 `world_T_ee_target`；
- 每周期计算 `delta = visual_world_T_ee.inverse() * world_T_ee_target`、`error = pin.log6(delta)`；
- 使用 Pinocchio `LOCAL` frame Jacobian 和阻尼伪逆计算 `dq_raw`，再限幅得到 `dq_limited`；
- 本阶段不发布 `/arm_velocity_controller/commands`，不发布 `/joint_states`，不接管 Gazebo 控制器；
- Base 扰动生成、Pinocchio ABA 内部仿真、已知 Base 位姿输入和原参考项目的内部目标转换逻辑均未迁入。

## 修改内容

新增：

- `scripts/visual_ee_stabilization_controller.py`

修改：

- `launch/d435i_apriltag_test.launch.py`
- `CMakeLists.txt`
- `docs/perception_control_report.md`

节点默认参数：

```text
urdf_path: <manipulator share>/arm.urdf
ee_frame: link6
joint_names: joint1 joint2 joint3 joint4 joint5 joint6
joint_states_topic: /joint_states
visual_pose_topic: /visual_ee_pose
visual_valid_topic: /visual_ee_pose_valid
control_rate: 100.0
damping: 0.05
max_joint_velocity: 1.0
measurement_timeout_sec: 0.35
joint_state_timeout_sec: 0.35
task_gain: [4, 4, 4, 2, 2, 2]
max_task_velocity: [0.5, 0.5, 0.5, 1, 1, 1]
```

调试输出均为 `std_msgs/msg/Float64MultiArray`：

```text
/visual_stabilization/error       [vx, vy, vz, wx, wy, wz]
/visual_stabilization/dq_raw      [joint1..joint6] rad/s
/visual_stabilization/dq_limited  [joint1..joint6] rad/s
/visual_stabilization/target_pose [x, y, z, qx, qy, qz, qw]
```

安全行为：

- `/joint_states` 按 joint name 读取，不依赖消息数组顺序；
- 目标未锁定、视觉 invalid、视觉超时、关节状态超时时，`dq_raw` 和 `dq_limited` 发布六维零速度；
- 计算结果含 NaN/Inf 或 Pinocchio 求解异常时，本周期降级为零速度调试输出；
- 本阶段没有任何实际关节命令发布者。

## 验收命令

静态检查：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
python3 -m py_compile \
  src/arm-platform/scripts/visual_ee_stabilization_controller.py \
  src/arm-platform/launch/d435i_apriltag_test.launch.py
```

构建：

```bash
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
```

启动 dry-run：

```bash
source setup_env.bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false \
  run_visual_ee_estimator:=true \
  run_visual_stabilization_controller:=true
```

Topic spot check：

```bash
ros2 topic echo --once /visual_stabilization/error
ros2 topic echo --once /visual_stabilization/dq_raw
ros2 topic echo --once /visual_stabilization/dq_limited
ros2 topic echo --once /visual_stabilization/target_pose
ros2 topic info /arm_position_controller/commands --verbose
ros2 topic info /joint_states --verbose
```

预期结果：

```text
/visual_stabilization/error: 6 finite values
/visual_stabilization/dq_raw: 6 finite values
/visual_stabilization/dq_limited: 6 finite values, abs(value) <= max_joint_velocity
/visual_stabilization/target_pose: 7 finite values
```

在阶段 2 dry-run 启动方式下，`visual_ee_stabilization_controller` 不应出现在 `/arm_velocity_controller/commands` 的发布者列表中，`/joint_states` 仍应由 Gazebo `joint_state_broadcaster` 发布。

本阶段默认 `control_mode:=kinematic_visualization`，因此 `/arm_velocity_controller/commands` 可不存在；若后续用 `physical_dynamics` 启动，则该 topic 的发布者也不应包含 `visual_ee_stabilization_controller`。

## 本次验收结果

构建：

```text
colcon build --packages-select manipulator --symlink-install
Summary: 1 package finished
```

仍有既有 Pinocchio/eigenpy 触发的 Boost Python header CMake warning，不影响构建产物。

短运行命令：

```bash
source setup_env.bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false \
  run_visual_ee_estimator:=true \
  run_visual_stabilization_controller:=true
```

观测结果：

```text
visual_ee_stabilization_controller loaded arm.urdf and locked target:
  [0.3532, 0.0015, 0.3743]

/visual_stabilization/error:
  [0, 0, 0, 0, 0, 0]

/visual_stabilization/dq_raw:
  [0, 0, 0, 0, 0, 0]

/visual_stabilization/dq_limited:
  [0, 0, 0, 0, 0, 0]

/visual_stabilization/target_pose:
  [0.3532337061, 0.0015405685, 0.3743455966, 0, 0, 0, 1]
```

发布者检查：

```text
/joint_states publisher:
  joint_state_broadcaster

/visual_stabilization/error publisher:
  visual_ee_stabilization_controller

/visual_stabilization/dq_raw publisher:
  visual_ee_stabilization_controller

/visual_stabilization/dq_limited publisher:
  visual_ee_stabilization_controller

/visual_stabilization/target_pose publisher:
  visual_ee_stabilization_controller

/arm_position_controller/commands publisher:
  student_joint_command_bridge
```

`visual_ee_stabilization_controller` 没有发布实际 Gazebo 控制命令；本阶段 dry-run 边界保持成立。

## 项目状态评估

阶段 2 完成后，项目具备视觉末端位姿输入和 Pinocchio CLIK dry-run 计算链路。若验收中确认误差、`dq_raw` 和 `dq_limited` 方向符合预期且无 NaN/Inf，可以进入阶段 3，将同一计算链路扩展为 XYZ 闭环并显式接入 `/arm_velocity_controller/commands`。阶段 3 前仍需保留 AprilTag 丢失、数据超时、速度限幅和关节限位保护。

# Perception-Control 阶段 3：静态环境下的 XYZ 闭环验证

## 目标与结论

本阶段在静态 base、静态 AprilTag 场景下启用视觉 XYZ 闭环。控制器启动后锁定首次有效 `/visual_ee_pose` 为末端世界位置目标，只控制位置误差：

```text
error = [target_x - current_x, target_y - current_y, target_z - current_z, 0, 0, 0]
```

姿态误差继续置零，不进入闭环控制。控制器显式启用后向 Gazebo 速度控制器发布：

```text
/arm_velocity_controller/commands  std_msgs/Float64MultiArray
```

结论：

- XYZ 闭环可以在静态环境中把约 `46.3 mm` 的视觉位置误差收敛到 `3 mm` 死区内；
- 验收过程中 `/visual_ee_pose_valid` 有效率为 `1.000`；
- 最大控制器输出关节速度 `0.318042 rad/s`，低于本阶段限幅 `0.35 rad/s`；
- dry-run 默认行为保持不变，只有 `visual_stabilization_dry_run:=false` 时才发布真实速度命令；
- 阶段 3 后项目适合进入阶段 4：连续 Base 平移扰动下的 XYZ 稳定。

## 修改内容

新增：

- `scripts/static_xyz_closed_loop_check.py`

修改：

- `scripts/visual_ee_stabilization_controller.py`
- `launch/d435i_apriltag_test.launch.py`
- `CMakeLists.txt`
- `docs/perception_control_report.md`

控制节点新增参数：

```text
dry_run: true
control_mode: xyz
command_topic: /arm_velocity_controller/commands
command_start_delay_sec: 0.0
max_task_velocity_xyz: 0.08 0.08 0.08
position_deadband_m: 0.003
joint_limit_margin_rad: 0.05
```

`xyz` 模式使用 Pinocchio `LOCAL_WORLD_ALIGNED` frame Jacobian 的前三行，将世界系 XYZ 速度命令映射为六关节速度。`se3_debug` 保留阶段 2 的 6D dry-run 计算路径，供后续阶段 6 使用。

安全行为：

- 目标未锁定、视觉 invalid、视觉数据超时、关节状态超时、计算异常或 NaN/Inf 时输出零速度；
- 关节速度按 `max_joint_velocity` 限幅；
- XYZ 任务速度按 `max_task_velocity_xyz` 限幅；
- 位置误差进入 `position_deadband_m` 后输出零任务速度；
- 若关节已在限位 margin 内且命令继续推向限位，则本周期输出零关节速度；
- `command_start_delay_sec` 用于阶段 3 实验中先锁定目标，再注入外部速度脉冲。

## 验收命令

静态检查和构建：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
python3 -m py_compile \
  src/arm-platform/scripts/visual_ee_stabilization_controller.py \
  src/arm-platform/scripts/static_xyz_closed_loop_check.py \
  src/arm-platform/launch/d435i_apriltag_test.launch.py

source setup_env.bash
colcon build --packages-select manipulator --symlink-install
```

启动阶段 3 闭环：

```bash
source setup_env.bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false \
  control_mode:=physical_dynamics \
  velocity_command_source:=external \
  fix_base_to_world:=true \
  run_visual_ee_estimator:=true \
  run_visual_stabilization_controller:=true \
  visual_stabilization_dry_run:=false \
  visual_stabilization_control_mode:=xyz \
  visual_stabilization_max_joint_velocity:=0.35 \
  visual_stabilization_max_task_velocity_xyz:='0.08 0.08 0.08'
```

注入关节速度脉冲并记录闭环响应：

```bash
source setup_env.bash
ros2 run manipulator static_xyz_closed_loop_check.py \
  --use-sim-time \
  --output-csv /tmp/windylab_phase3_static_xyz_active_pulse.csv \
  --max-joint-velocity 0.35 \
  --pulse-velocity '0.0 0.4 -0.3 0.0 0.0 0.0' \
  --pulse-duration-sec 0.8 \
  --monitor-duration-sec 8.0
```

## 本次验收结果

构建结果：

```text
colcon build --packages-select manipulator --symlink-install
Summary: 1 package finished
```

阶段 3 active-pulse 验收：

```text
Static XYZ closed-loop summary
  samples: 160
  visual_valid_fraction: 1.000
  initial_error_max_m: 0.046315
  error_max_m: 0.046315
  steady_error_mean_m: 0.000000
  steady_error_max_m: 0.000000
  max_abs_joint_velocity_rad_s: 0.318042
  pass: true
  csv: /tmp/windylab_phase3_static_xyz_active_pulse.csv
```

说明：`steady_error_mean_m` 和 `steady_error_max_m` 为 `0` 表示控制器输出调试误差已进入 `position_deadband_m=0.003 m` 死区，不表示视觉测量没有亚毫米级残差。

## 项目状态评估

阶段 3 已完成静态 XYZ 视觉闭环接入和可复现实验脚本。下一阶段可以在相同控制节点基础上使用 moving-base 扰动入口，将 `fix_base_to_world:=false` 并引入 base 扰动 CSV，对比 baseline 零速度与 visual XYZ 闭环的世界系末端 RMS 误差、最大误差、关节速度峰值和 AprilTag 丢失率。

# Perception-Control 阶段 4：连续 Base 平移扰动下的 XYZ 稳定

## 目标与结论

本阶段实现 moving-base 动态扰动实验入口和指标汇总工具，用同一套 Gazebo replay CSV 对比：

```text
baseline    : /arm_velocity_controller/commands 始终为零
visual_xyz  : /visual_ee_pose + Pinocchio XYZ CLIK 输出关节速度
```

评价主指标使用 Gazebo 真值 `actual_link6` 相对实验首个有效样本的位置偏移，避免控制输入和评价同源。

结论：

- 阶段 4 的实验框架、扰动 profile、CSV 记录和指标汇总已经实现；
- `sine_x`、`sine_y`、`sine_z`、`sine_xyz`、`random_translation_3d` 均已完成 baseline/visual_xyz headless 实测；
- 当前 visual_xyz 实测未达到验收标准：visual 组 `visual_loss_rate=1.0`，关节速度峰值为 `0`，Gazebo 真值 RMS 与 baseline 基本相同；
- 因此当前项目状态不适合进入阶段 5，必须先修复 moving-base replay 下 AprilTag/visual pose 持续失效的问题。

## 修改内容

新增：

- `scripts/phase4_dynamic_xyz_metrics.py`

修改：

- `scripts/generate_base_disturbance.py`
- `scripts/base_disturbance_replay.py`
- `scripts/visual_ee_pose_estimator.py`
- `launch/moving_base_stabilization.launch.py`
- `config/base_disturbance_profiles.yaml`
- `CMakeLists.txt`
- `docs/perception_control_report.md`

扰动生成：

- 新增 `sine_xyz` profile；
- `random_translation_3d` 增加 `max_translation_norm: 0.10`，保证三维平移模长最大为 10 cm；
- `sine_x/y/z/xyz` 和 `random_translation_3d` 均可用同一脚本生成固定 duration、sample rate、seed 的 CSV。

moving-base launch：

- `experiment_mode:=baseline|visual_xyz`；
- `visual_xyz` 自动启动 `apriltag_ros`、`visual_ee_pose_estimator.py` 和 `visual_ee_stabilization_controller.py`；
- 默认 `visual_xyz` 使用 `max_joint_velocity=1.0 rad/s`、`max_task_velocity_xyz=0.25 0.25 0.25 m/s`，覆盖 10 cm、0.35 Hz 平移扰动所需的任务空间速度；
- replay 增加 `replay_state_sample_stride`，允许降低 Gazebo get-entity-state 查询频率，减少对 RGB/AprilTag 的干扰。

replay CSV 新增字段：

```text
visual_valid
latest_visual_pose_age_sec
visual_error
visual_dq_limited
```

指标脚本输出：

```text
xyz_rms_m
xyz_max_m
steady_xyz_mean_m
steady_xyz_max_m
joint_velocity_peak_rad_s
joint_max_step_rad
command_saturation_ratio
visual_loss_rate
visual_pose_age_mean_sec
visual_pose_age_max_sec
visual_error_rms_m
visual_error_max_m
```

视觉估计器修正：

- `camera->tag` TF 和 `ee->camera` TF 分开 lookup；
- `ee->camera` 固定外参允许回退 latest TF，避免动态时间戳微小超前阻塞；
- pending detection 若被后续 detection 超越，会丢弃旧样本继续处理；
- shutdown race 增加保护，避免 timeout 清理时误报。

## 验收命令

静态检查和构建：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
python3 -m py_compile \
  src/arm-platform/scripts/generate_base_disturbance.py \
  src/arm-platform/scripts/base_disturbance_replay.py \
  src/arm-platform/scripts/visual_ee_pose_estimator.py \
  src/arm-platform/scripts/phase4_dynamic_xyz_metrics.py \
  src/arm-platform/launch/moving_base_stabilization.launch.py

colcon build --packages-select manipulator --symlink-install
```

生成扰动并运行全量实验：

```bash
source setup_env.bash
profiles=(sine_x sine_y sine_z sine_xyz random_translation_3d)
outdir=/tmp/windylab_phase4_full
mkdir -p "$outdir"

for profile in "${profiles[@]}"; do
  python3 src/arm-platform/scripts/generate_base_disturbance.py \
    --config src/arm-platform/config/base_disturbance_profiles.yaml \
    --profile "$profile" \
    --duration-sec 30 \
    --sample-rate-hz 20 \
    --output-csv "$outdir/${profile}.csv"

  for mode in baseline visual_xyz; do
    timeout -s INT -k 10s 65s \
      ros2 launch manipulator moving_base_stabilization.launch.py \
      gui:=false use_rviz:=false \
      experiment_mode:=$mode \
      disturbance_csv:="$outdir/${profile}.csv" \
      replay_output_csv:="$outdir/${profile}_${mode}_replay.csv" \
      replay_rate_hz:=20.0 \
      replay_state_sample_stride:=4 \
      start_delay_sec:=6.0
  done
done
```

汇总指标：

```bash
python3 src/arm-platform/scripts/phase4_dynamic_xyz_metrics.py \
  --run sine_x_baseline=/tmp/windylab_phase4_full/sine_x_baseline_replay.csv \
  --run sine_x_visual=/tmp/windylab_phase4_full/sine_x_visual_xyz_replay.csv \
  --run sine_y_baseline=/tmp/windylab_phase4_full/sine_y_baseline_replay.csv \
  --run sine_y_visual=/tmp/windylab_phase4_full/sine_y_visual_xyz_replay.csv \
  --run sine_z_baseline=/tmp/windylab_phase4_full/sine_z_baseline_replay.csv \
  --run sine_z_visual=/tmp/windylab_phase4_full/sine_z_visual_xyz_replay.csv \
  --run sine_xyz_baseline=/tmp/windylab_phase4_full/sine_xyz_baseline_replay.csv \
  --run sine_xyz_visual=/tmp/windylab_phase4_full/sine_xyz_visual_xyz_replay.csv \
  --run random_baseline=/tmp/windylab_phase4_full/random_translation_3d_baseline_replay.csv \
  --run random_visual=/tmp/windylab_phase4_full/random_translation_3d_visual_xyz_replay.csv \
  --output-csv /tmp/windylab_phase4_full/phase4_summary.csv
```

## 本次验收结果

构建结果：

```text
colcon build --packages-select manipulator --symlink-install
Summary: 1 package finished
```

全量实验结果：

| profile | mode | XYZ RMS m | XYZ max m | joint vel peak rad/s | visual loss | set failures |
|---|---|---:|---:|---:|---:|---:|
| sine_x | baseline | 0.067577 | 0.100000 | 0.000000 | n/a | 0 |
| sine_x | visual_xyz | 0.067582 | 0.100000 | 0.000000 | 1.000 | 0 |
| sine_y | baseline | 0.067576 | 0.100000 | 0.000000 | n/a | 0 |
| sine_y | visual_xyz | 0.067577 | 0.100000 | 0.000000 | 1.000 | 0 |
| sine_z | baseline | 0.040609 | 0.060000 | 0.000000 | n/a | 20 |
| sine_z | visual_xyz | 0.040546 | 0.060000 | 0.000000 | 1.000 | 0 |
| sine_xyz | baseline | 0.103815 | 0.153623 | 0.000000 | n/a | 0 |
| sine_xyz | visual_xyz | 0.103815 | 0.153623 | 0.000000 | 1.000 | 0 |
| random_translation_3d | baseline | 0.059898 | 0.099977 | 0.000000 | n/a | 0 |
| random_translation_3d | visual_xyz | 0.059896 | 0.099977 | 0.000000 | 1.000 | 0 |

结果文件：

```text
/tmp/windylab_phase4_full/phase4_summary.csv
/tmp/windylab_phase4_full/*_baseline_replay.csv
/tmp/windylab_phase4_full/*_visual_xyz_replay.csv
```

说明：

- baseline 与 visual_xyz 的 Gazebo 真值 RMS 未出现明显差异；
- visual_xyz 组 `visual_loss_rate=1.000`，`latest_visual_pose_age_sec` 在 18-35 s 量级，说明 replay 期间 `/visual_ee_pose` 没有持续刷新；
- visual_xyz 组 `joint_velocity_peak_rad_s=0`，说明控制器因视觉失效安全输出零速度；
- `sine_z_baseline` 出现 20 个 set failures，但 visual 对应组为 0；该单项不影响“visual 未实际闭环”的主要结论。

## 项目状态评估

阶段 4 的工程入口和指标工具已经完成，但闭环验收未通过。下一步不应进入阶段 5，而应先修复阶段 4 阻塞项：

- 在 `moving_base_stabilization.launch.py` + `base_disturbance_replay.py` 组合下，定位为什么 AprilTag 同步/`/visual_ee_pose` 会在 replay 开始后失效；
- 优先检查 Gazebo `set_entity_state` service 调用与 RGB-D sensor update 的互相影响；
- 若 service replay 会持续干扰传感器，应改为 Gazebo 插件、模型速度控制或更轻量的 base 扰动机制；
- 修复后重新运行本阶段 5 个 profile 的 baseline/visual_xyz 对比，确认 visual XYZ RMS 明显低于 baseline。

## 阶段 4 Visual Chain Debug 修复

根据 `phase4_visual_chain_debug_plan.md`，本次先修复最可能导致 `/visual_ee_pose` 中断的两类问题：

- `base_disturbance_replay.py` 的主 replay 循环不再高频调用 `/get_entity_state`。现在循环内只调用 `/set_entity_state`，Gazebo 真值 pose 通过 `/model_states` 和 `/link_states` 订阅缓存采样；`/get_entity_state` 仅保留为启动阶段实体存在性检查的后备路径。
- `visual_ee_pose_estimator.py` 新增 `tag_tf_mode:=stamped|latest`。阶段 4 launch 默认使用 `latest`，按最新 `camera_color_optical_frame -> apriltag_36h11_00000` TF 估计视觉 pose，同时用 `visual_max_tag_tf_age_sec` 严格限制 TF 年龄，避免使用陈旧 TF。
- `/visual_ee_pose_debug` 增加 TF 诊断计数，包括 camera->tag lookup、future/past extrapolation、stale TF、ee->camera lookup 和 fallback 成功次数。
- 新增 `phase4_visual_chain_diagnostics.py`，可持续记录 RGB、CameraInfo、AprilTag detections、tag TF、`/visual_ee_pose`、valid/debug 和 `/clock` 速率到 CSV。
- `moving_base_stabilization.launch.py` 新增诊断和负载隔离参数：`run_phase4_diagnostics`、`phase4_diagnostics_output_csv`、`visual_tag_tf_mode`、`visual_max_tag_tf_age_sec`、`rgbd_update_rate`、`rgbd_width`、`rgbd_height`、`imu_enabled`、`imu_update_rate`。

推荐先用低 replay 频率和低传感器负载复测视觉链路：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/arm-platform/scripts/generate_base_disturbance.py \
  --profile sine_x \
  --output-csv /tmp/phase4_debug_sine_x.csv

timeout -s INT -k 10s 70s \
  ros2 launch manipulator moving_base_stabilization.launch.py \
  gui:=false use_rviz:=false \
  experiment_mode:=visual_xyz \
  disturbance_csv:=/tmp/phase4_debug_sine_x.csv \
  replay_output_csv:=/tmp/phase4_debug_sine_x_visual_replay.csv \
  replay_rate_hz:=5.0 \
  start_delay_sec:=8.0 \
  run_phase4_diagnostics:=true \
  phase4_diagnostics_output_csv:=/tmp/phase4_debug_visual_chain.csv \
  rgbd_update_rate:=10 \
  rgbd_width:=320 \
  rgbd_height:=240 \
  imu_enabled:=false
```

复测时优先查看：

- `/tmp/phase4_debug_visual_chain.csv` 中 `image_rate_hz`、`apriltag_msg_rate_hz`、`target_detection_rate_hz`、`tag_tf_update_rate_hz`、`visual_pose_rate_hz` 是否持续非零；
- `tag_tf_age_sec` 是否稳定低于 `visual_max_tag_tf_age_sec`；
- `visual_debug_reason` 是否从 `waiting_for_detection`、`tag_tf_stale`、`tag_tf_lookup_failed_latest` 转为 `ok`；
- replay CSV 中 `visual_valid` 是否大部分为 `true`，`velocity_command` 是否出现非零关节速度。

若 5 Hz replay 仍出现 RGB 或 AprilTag rate 归零，说明 `set_entity_state` 方式仍会压制 Gazebo sensor，应按调试计划继续切换到 Gazebo ModelPlugin 或虚拟 Base 关节扰动方案。若视觉链路持续有效，再扫描 `replay_rate_hz:=10/15/20` 并重新跑 5 个阶段 4 profile 的 baseline/visual_xyz 对比。

## 阶段 4 修复后自主短测

测试设置：

- profile：`sine_x`
- duration：20 s
- trajectory sample rate / replay rate：5 Hz
- RGB-D：10 Hz，320x240
- IMU：关闭
- visual mode：`visual_tag_tf_mode:=latest`

输出文件：

```text
/tmp/phase4_short_sine_x.csv
/tmp/phase4_short_sine_x_baseline_replay.csv
/tmp/phase4_short_sine_x_visual_replay.csv
/tmp/phase4_short_visual_chain.csv
/tmp/phase4_short_sine_x_visual_slow_replay.csv
/tmp/phase4_short_visual_slow_chain.csv
```

主要结果：

| run | set failures | visual loss | visual pose age mean/max s | joint velocity peak rad/s | diagnostics target detection mean Hz | diagnostics visual pose mean Hz |
|---|---:|---:|---:|---:|---:|---:|
| baseline | 0 | n/a | n/a | 0.0 | n/a | n/a |
| visual default | 0 | 0.832 | 4.578 / 12.985 | 1.0 | 1.677 | 1.048 |
| visual slow, max joint 0.2 rad/s | 0 | 0.535 | 0.651 / 4.518 | 0.2 | 4.362 | 2.332 |

诊断结论：

- RGB、CameraInfo、AprilTag detection topic 本身已经稳定恢复到约 10 Hz，说明之前的“视觉链路完全停止”问题已被缓解；
- `/clock` 诊断订阅 QoS 已修正，短测中 clock rate 约 9.8-10.0 Hz；
- visual pose 已不再是 0 Hz，但默认控制参数下 `visual_loss_rate` 仍高达 0.832，主要 debug reason 是 `target_not_detected`；
- 默认控制多次打到 1.0 rad/s 限幅，保守限速后 visual loss 降到 0.535，说明控制动作过猛会把相机/末端带出可检测范围；
- 保守限速仍未达到阶段 4 验收标准，visual loss 仍远高于 10%。

指标注意事项：

- visual 两组 replay CSV 的第 0 行出现一次 Gazebo link state 瞬态异常，`actual_link6`/`actual_camera` 为几十米量级；
- 当前 `phase4_dynamic_xyz_metrics.py` 使用第一个 GT 样本作为原点，因此 visual 两组官方 summary 的 78-80 m 级 `xyz_rms_m` 被第 0 行污染；
- 剔除第 0 行并使用首个正常 GT 样本作为原点后，短测 XYZ RMS 约为：
  - baseline：0.0114 m
  - visual default：0.3728 m
  - visual slow：0.2840 m
- 因此结论不变：修复后视觉链路恢复，但阶段 4 闭环仍失败，visual 未优于 baseline。

下一步建议：

- 先修 replay 记录侧的初始 GT 原点问题：等待 `/model_states`/`/link_states` 稳定并丢弃明显非物理首样本，再开始写 CSV 或在 metrics 中跳过异常原点；
- 将阶段 4 visual 控制参数调低并加入目标可见性约束，避免短暂视觉误差直接打满关节速度；
- 继续用 diagnostics 区分 `target_not_detected` 与 `tag_tf_stale`，优先解决 tag 在 FOV 中持续可见的问题，再做 10/15/20 Hz replay rate 扫描。

## 阶段 4 二次修正实施

本次修正落实上述两个阻塞点：

- replay 侧只缓存物理合理的 Gazebo GT pose，默认任一坐标绝对值超过 `5.0 m` 的 `/model_states` 或 `/link_states` 样本会被丢弃；正式 replay 前要求关键实体连续 `3` 个稳定样本；
- metrics 侧新增 `invalid_gt_samples`，并使用首个正常 `actual_link6` 样本作为 GT origin，避免第 0 行 Gazebo 瞬态污染 RMS；
- AprilTag 从 `x=1.23 m` 后移到 `x=1.60 m`，背景墙同步移动到 `x=1.63 m`；`moving_base_stabilization.launch.py` 中 `world_to_tag_xyz` 同步更新为带既有标定偏移的 `1.60042456 0.000976374 0.35065986`；
- visual_xyz 默认限速改为 `visual_stabilization_max_joint_velocity:=0.2` 和 `visual_stabilization_max_task_velocity_xyz:=0.05 0.05 0.05`；
- visual 控制器新增可见性门控：出现 `1` 个 ready 视觉 pose 后锁定目标，视觉连续丢失 `0.5 s` 后重置目标，XYZ 视觉误差超过 `0.20 m` 时重置目标并输出零速度。

新增/更新的关键 launch 参数：

```text
replay_gt_max_abs_position_m:=5.0
replay_entity_stable_samples:=3
visual_required_consecutive_valid_poses:=1
visual_relock_after_visual_loss_sec:=0.5
visual_max_visual_error_norm_m:=0.20
visual_max_tag_tf_age_sec:=1.5
```

后续复测仍按先短测、再三轴、最后复合扰动的顺序执行。短测优先看 `visual_loss_rate`、`invalid_gt_samples`、`target_detection_rate_hz`、`visual_pose_rate_hz` 和 corrected `xyz_rms_m`。

实施后 20 s `sine_x` 短测结果：

| run | invalid GT | set failures | visual loss | joint velocity peak rad/s | target detection mean Hz | visual pose mean Hz | XYZ RMS m |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 0 | 0 | n/a | 0.0 | n/a | n/a | 0.0684 |
| tag moved, no lock due strict gate | 0 | 0 | 0.139 | 0.0 | 9.857 | 4.112 | 0.0684 |
| tag moved, relaxed lock | 0 | 0 | 0.277 | 0.2 | 3.948 | 2.353 | 6.4170 |

结论：

- GT 首样本异常已修复，`invalid_gt_samples=0`；
- Tag 后移对静态可见性有效，未运动时 `target_detection_rate_hz` 可稳定接近 10 Hz，`target_not_detected` 基本消失；
- 放宽门控后控制器可以进入闭环并输出非零速度；
- 但闭环运动会再次显著降低 target detection，且当前视觉控制会造成过大的机械臂运动，visual 仍明显差于 baseline；
- 因此阶段 4 仍未通过，下一轮应重点审查视觉 pose 到关节速度的控制符号、雅可比坐标系、目标锁定策略，而不是继续单纯放宽可见性门控。
