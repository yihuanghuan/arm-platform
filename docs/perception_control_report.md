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
