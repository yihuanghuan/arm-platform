# D435i Gazebo Classic 集成报告

## 阶段 0：仓库与运行环境调查

阶段 0 的目标是在不改动机械臂控制逻辑的前提下，确认当前仓库结构、ROS/Gazebo 环境和原有机械臂基线，并新增一个最小 Gazebo Classic 启动入口，让当前六关节机械臂可以在 Gazebo 中显示。

本阶段不引入 D435i、不添加 Gazebo 传感器、不添加 AprilTag，也不实现 SLAM、视觉伺服或 Base 扰动补偿。

### 环境

- 工作空间：`/home/yihuang/westlake/windylab-arm-for6/windylab_ws`
- Gazebo Classic：`11.10.2`
- ROS：ROS 2 Humble，`ROS_VERSION=2`，`ROS_DISTRO=humble`
- 内核：`Linux JIAOLONG-Series 6.8.0-124-generic #124~22.04.1-Ubuntu SMP PREEMPT_DYNAMIC Tue May 26 21:05:19 UTC x86_64`
- 构建方式：ROS 2 `ament_cmake` 包，通过 `colcon build` 构建
- 工作空间包：
  - `manipulator`：`src/arm-platform`
  - `dummy_interface`：`src/dummy-interface`
  - `dummy_description`：`src/dummy_description`
  - `serial`：`src/serial`

### Gazebo ROS 包与插件

阶段 0 初始检查时已安装：

- `gazebo_dev`
- `gazebo_msgs`
- `gazebo_ros`

`gazebo_ros` 提供阶段 0 模型 spawn 所需的核心库：

- `/opt/ros/humble/lib/libgazebo_ros_factory.so`
- `/opt/ros/humble/lib/libgazebo_ros_init.so`
- `/opt/ros/humble/lib/libgazebo_ros_force_system.so`

后续 D435i RGB-D/IMU 阶段需要的通用 Gazebo ROS sensor 插件通过以下命令补齐：

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

插件检查命令：

```bash
source setup_env.bash
find /opt/ros/"$ROS_DISTRO" \
  \( -name 'libgazebo_ros*camera*.so' \
  -o -name 'libgazebo_ros*openni*.so' \
  -o -name 'libgazebo_ros*depth*.so' \
  -o -name 'libgazebo_ros*imu*.so' \) \
  2>/dev/null | sort
```

### 当前机器人描述

- 当前主机器人描述：`src/arm-platform/config/arm.urdf`
- 安装后 launch 使用的描述：`share/manipulator/arm.urdf`
- `robot_description` 入口：`src/arm-platform/launch/student_arm.launch.py`
- 当前主模型类型：直接读取纯 URDF 文件，不是原生 xacro 生成链
- Base link：`base_link`
- 当前代码使用的第六关节末端 link：`link6`
- 当前活动 URDF 中没有 `tool0`、flange 或额外 end-effector fixed link
- 当前 TF 链：`base_link -> link1 -> link2 -> link3 -> link4 -> link5 -> link6`
- 当前活动 URDF 中没有 `world -> base_link` 固定关节
- `base_link` 作为 root link 带 inertial，会触发 `robot_state_publisher` 的 KDL root inertial warning

`dummy_description` 下存在部分 effector xacro 定义了 `tool0`，但它们没有接入当前 `student_arm.launch.py` 使用的 `robot_description` 路径。

### 控制与模型加载

- 当前学生仿真是自定义虚拟臂节点，不是 Gazebo 物理控制。
- `/joint_states` 由 `student_arm_node` 发布。
- TF 由 `robot_state_publisher` 发布。
- 学生命令入口：`/student/joint_command`，类型为 `sensor_msgs/msg/JointState`
- 可选反馈：`/student/joint_feedback`，类型为 `dummy_interface/msg/MotorState`
- 学生模式控制器：自定义 `SmoothPositionController`
- 当前没有启用 `ros2_control`、`ros_control`、`gazebo_ros_control`、transmission 或 controller yaml 链路。
- IK 模块：`src/arm-platform/demo/pinocchio_ik_6dof.py`
- IK 读取模型：`src/arm-platform/config/arm.urdf`
- IK 末端 frame：`link6`
- 重力控制和碰撞检查也通过 Pinocchio 从配置的 URDF 路径加载模型。

### 原始基线验证

构建命令：

```bash
source setup_env.bash
colcon build
```

结果：4 个包均完成构建。`dummy_description`、`dummy_interface`、`serial` 有既有 CMake cache 路径警告，因为缓存来自旧路径：

```text
/home/yihuang/westlake/windylab-arm/windylab_ws
```

原学生仿真启动命令：

```bash
source setup_env.bash
ros2 launch manipulator student_arm.launch.py use_rviz:=False
```

观察到的话题：

- `/joint_states`
- `/parameter_events`
- `/robot_description`
- `/rosout`
- `/student/joint_command`
- `/student/joint_feedback`
- `/tf`
- `/tf_static`

运动验证命令：

```bash
ros2 topic pub --once /student/joint_command sensor_msgs/msg/JointState \
  "{position: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

结果：`/joint_states` 和 `tf2_echo base_link link6` 均反映 joint1 运动。

IK 自测：

```bash
cd src/arm-platform/demo
python3 pinocchio_ik_6dof.py
```

结果：脚本加载 6 自由度模型，末端 frame 为 `link6`，10 个随机 FK/IK 回代目标中收敛 8 个；未收敛样本为随机困难或不可达目标，符合当前 demo 行为。

### 阶段 0 Gazebo 基线

新增 launch：

```text
src/arm-platform/launch/gazebo_arm.launch.py
```

使用方式：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py
```

无 GUI 验证：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py gui:=false
```

有 GUI 验证：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py gui:=true
```

该 launch 通过 `gazebo_ros` 启动 Gazebo Classic，将当前机械臂描述发布为 `robot_description`，并用 `spawn_entity.py` 生成名为 `windylab_arm` 的 Gazebo 实体。

为避免 Gazebo GUI 卡在在线模型库或 mesh 路径解析上，launch 做了以下处理：

- 将项目 mesh URI 改写为本地 `file://` 路径；
- 将 `GAZEBO_MODEL_PATH` 设置为包含 Gazebo Classic 本地 `sun` 和 `ground_plane` 模型；
- 将 `GAZEBO_MODEL_DATABASE_URI` 设置为空，禁用在线模型库查询；
- 去掉发布给 `spawn_entity.py` 的 XML encoding 声明，规避 ROS 2 Humble 中 `spawn_entity.py`/lxml 对 Unicode XML 声明的解析问题。

阶段 0 的 Gazebo 模型被标记为 static，只用于稳定显示基线；此时尚未接入 Gazebo 关节控制。

验证结果：

```text
Spawn status: SpawnEntity: Successfully spawned entity [windylab_arm]
```

### Git 状态

工作空间根目录不是 Git 仓库，实际 Git 仓库位于：

- `src/arm-platform`
- `src/dummy_description`
- `src/dummy-interface`
- `src/serial`

阶段 0 修改前，`src/arm-platform` 已经存在较多未提交改动，包括学生仿真入口和 6 自由度 demo 文件。阶段提交只纳入与本阶段相关的 Gazebo baseline 和报告文件。

## 阶段 1：引入并验证官方 D435i 描述资源

阶段 1 使用 ROS 包 `realsense2_description` 引入官方 D435i xacro 和 mesh。官方文件不复制到本仓库，也不修改官方文件；项目侧只新增 wrapper xacro 和集成入口。

安装依赖：

```bash
sudo apt-get install -y ros-humble-xacro ros-humble-realsense2-description
```

新增项目描述文件：

- `config/sensors/d435i_mount.xacro`
- `config/arm_with_d435i.urdf.xacro`

默认安装参数：

- `camera_name:=camera`
- `camera_parent_link:=link6`
- `camera_xyz:="0.06 0 0.04"`
- `camera_rpy:="0 0 0"`
- `camera_use_nominal_extrinsics:=true`

默认挂载位置是当前真实末端 link：`link6`。

### 阶段 1 静态检查

展开并检查 URDF：

```bash
source setup_env.bash
xacro $(ros2 pkg prefix manipulator)/share/manipulator/arm_with_d435i.urdf.xacro \
  camera_enabled:=true > /tmp/robot_with_d435i.urdf
check_urdf /tmp/robot_with_d435i.urdf
```

结果：

- `check_urdf` 成功解析；
- 展开后模型包含 21 个 link 和 20 个 joint；
- 未发现重复 link 或 joint 名称；
- 生成的关键 TF 链包括：

```text
link6
  -> camera_bottom_screw_frame
  -> camera_link
  -> camera_depth_frame
  -> camera_depth_optical_frame
```

同时生成官方 nominal color、infrared、accel、gyro frames。

### 阶段 1 运行验证

原始学生启动保持兼容：

```bash
ros2 launch manipulator student_arm.launch.py use_rviz:=False camera_enabled:=false
```

带 D435i 的学生启动：

```bash
ros2 launch manipulator student_arm.launch.py use_rviz:=False camera_enabled:=true
```

验证结果：

- `robot_state_publisher` 成功加载 D435i frame；
- 发布一次 `/student/joint_command` 后 joint1 到达 `0.3 rad`；
- `base_link -> camera_depth_optical_frame` 随机械臂运动变化；
- `link6 -> camera_depth_optical_frame` 保持固定，说明相机随末端刚性运动。

Gazebo 验证：

```bash
ros2 launch manipulator gazebo_arm.launch.py gui:=true camera_enabled:=true
```

结果：

- `windylab_arm` 成功 spawn；
- Gazebo 中能看到机械臂末端的 D435i 外形；
- 模型位姿稳定，无 mesh 缺失、NaN 或模型爆炸。

截图证据：

- RViz：`/tmp/d435i_phase1_rviz.png`
- Gazebo 窗口：`/tmp/d435i_phase1_gazebo_window.png`

### 阶段 1 回归

- `colcon build` 完成 4 个工作空间包构建；
- 既有 CMake cache 旧路径 warning 仍存在于 `dummy_description`、`dummy_interface`、`serial`；
- `python3 src/arm-platform/demo/pinocchio_ik_6dof.py` 仍为 10 个随机目标收敛 8 个，与阶段 0 基线一致。

## 后续阶段注意事项

- 阶段 1.5 已补齐 Gazebo 侧 ROS 2 控制链路；默认 `gazebo_arm.launch.py` 不再使用 static 模型。
- 阶段 2 才会加入 RGB-D 和 IMU sensor plugin，当前阶段不发布图像、深度、点云或 IMU 数据。

## 阶段 1.5：连接 Gazebo 与 ROS 2 控制链路

阶段 1.5 的目标是让 Gazebo 中的六关节机械臂真正响应现有学生 demo 的 `/student/joint_command`，并让 Gazebo 侧的 `joint_state_broadcaster` 成为 `/joint_states` 的唯一发布者。这样 RViz、TF 和 D435i 位姿都跟随 Gazebo 中的关节状态，而不是再由原来的假臂节点单独驱动。

### 安装依赖

本阶段补齐了 Gazebo Classic 与 ROS 2 control 所需包：

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control
```

### 新增与修改文件

- `config/sensors/gazebo_ros2_control.xacro`：为六个关节声明 `position` command interface，以及 `position/velocity/effort` state interface；默认增加 `world -> base_link` 固定关节，避免 Gazebo 动力学中底座下落。
- `config/gazebo_controllers.yaml`：配置 `joint_state_broadcaster` 和 `position_controllers/JointGroupPositionController`。
- `scripts/student_joint_command_bridge.py`：订阅 `/student/joint_command` 的 `sensor_msgs/msg/JointState`，按 `joint1` 到 `joint6` 重排并限幅后，发布到 `/arm_position_controller/commands`。
- `launch/gazebo_arm.launch.py`：默认启用 `gazebo_ros2_control`，启动控制器 spawner 和桥接节点；保留 `use_ros2_control:=false` 的只显示模型路径；默认 `disable_collisions:=true`，从 Gazebo 版 `robot_description` 中移除 collision geometry，避免 SolidWorks STL 碰撞网格与位置接口导致物理抖动。
- `package.xml`、`CMakeLists.txt`：加入运行依赖和桥接脚本安装规则。

### 启动方式

默认控制模式：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py gui:=true
```

无 GUI 控制模式：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py gui:=false use_rviz:=false
```

兼容旧的 static 显示模式：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py use_ros2_control:=false static_model:=true
```

如需调试完整碰撞几何，可显式关闭碰撞移除：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py gui:=true disable_collisions:=false
```

当前不建议在阶段 1.5 默认打开完整 collision，因为当前 URDF 使用详细 STL 碰撞网格，且 Gazebo position interface 会和物理求解器形成硬约束修正，容易表现为机械臂颤抖。

### 验证结果

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select manipulator --symlink-install
```

结果：`manipulator` 构建成功。

URDF 展开与解析：

```bash
source install/setup.bash
xacro $(ros2 pkg prefix manipulator)/share/manipulator/arm_with_d435i.urdf.xacro \
  camera_enabled:=true use_ros2_control:=true fix_base_to_world:=true \
  > /tmp/windylab_arm_control.urdf
check_urdf /tmp/windylab_arm_control.urdf
```

结果：

- `check_urdf` 成功解析；
- root link 为 `world`；
- `world -> base_link -> ... -> link6 -> camera_link` 链路存在；
- D435i 官方子 frame 仍保留在 robot description 中，RViz 配置只显示主 frame。

控制器状态：

```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
```

结果：

```text
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active
arm_position_controller position_controllers/JointGroupPositionController active
```

六个 `joint*/position` command interface 均为 `available` 且 `claimed`，六个关节均发布 `position/velocity/effort` state interface。

`/joint_states` 发布者检查：

```bash
ros2 topic info /joint_states --verbose
```

结果：`/joint_states` 只有 1 个发布者，节点为 `joint_state_broadcaster`；`robot_state_publisher` 是订阅者。未发现 `student_arm_node` 与 Gazebo 同时发布 `/joint_states` 的冲突。

手动命令验证：

```bash
ros2 topic pub --once /student/joint_command sensor_msgs/msg/JointState \
  "{name: ['joint1','joint2','joint3','joint4','joint5','joint6'], position: [0.35, -0.25, 0.2, 0.1, -0.15, 0.3]}"
```

结果：命令通过桥接节点进入 `/arm_position_controller/commands`，Gazebo 发布的 `/joint_states` 发生变化。

demo 验证：

```bash
python3 src/arm-platform/demo/move_arm_demo_6dof.py
```

结果：demo 不需要修改，仍向 `/student/joint_command` 发布命令；Gazebo 中机械臂关节状态随 demo 变化。

有头 Gazebo 验证：

```bash
ros2 launch manipulator gazebo_arm.launch.py gui:=true verbose:=false use_rviz:=false camera_enabled:=true use_ros2_control:=true
```

结果：

- `windylab_arm` 成功 spawn；
- `gazebo_ros2_control` 成功加载；
- `joint_state_broadcaster` 和 `arm_position_controller` 均进入 active；
- 对 `/student/joint_command` 发布一次目标角后，Gazebo 侧 `/joint_states` 更新；
- 测试结束时 `gzclient`、`gzserver`、`robot_state_publisher` 和桥接节点均能正常退出。

### 抖动处理记录

阶段 1.5 初版接入后，`gui:=true` 下机械臂会持续颤抖。定位结果：

- 阶段 0/1 的 Gazebo 模型是 static，不参与动力学，所以不会暴露该问题；
- 阶段 1.5 改为动态 ROS 2 control 模型后，SolidWorks 导出的 `arm.urdf` 仍存在关节 `effort="0"`、`velocity="0"` 且缺少 `dynamics` 阻尼/摩擦的问题；
- 仅补关节 effort、velocity、damping、friction 和初始保持命令后，`/joint_states.velocity` 仍有明显跳动；
- 将 Gazebo 版 robot description 的 collision geometry 移除后，静止状态下各关节速度降到 `1e-14` 量级，命令后关节能精确到达目标角。

因此当前阶段默认使用无碰撞可视化控制模型。后续如果需要做真实碰撞或动力学，应单独为 Gazebo 建立简化 collision geometry，而不是直接复用视觉 STL 作为碰撞网格。

## 阶段 2：加入 RGB-D 传感器与 ROS 数据输出

阶段 2 在阶段 1/1.5 的官方 D435i 描述和 Gazebo ROS 2 control 链路基础上，加入 Gazebo Classic RGB-D depth camera sensor，并建立固定视觉测试场景。本阶段只发布 RGB、Depth、CameraInfo 和 PointCloud2，不加入 IMU、AprilTag、SLAM 或视觉闭环控制。

### 插件依据

本机 ROS 2 Humble 实际可用的相关 Gazebo 插件：

```text
/opt/ros/humble/lib/libgazebo_ros_camera.so
/opt/ros/humble/lib/libgazebo_ros_imu_sensor.so
```

Humble 下没有 ROS1 常见的 `libgazebo_ros_openni_kinect.so` 或单独 depth camera 插件；RGB-D 使用 `gazebo_ros_pkgs` 官方示例：

```text
/opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_depth_camera_demo.world
```

### 新增与修改文件

- `config/sensors/d435i_gazebo.xacro`：新增项目侧 Gazebo RGB-D wrapper，不修改 RealSense 官方 xacro。
- `config/arm_with_d435i.urdf.xacro`：加入 `rgbd_enabled`、分辨率、频率、FOV、clip range、frame/topic 参数。
- `worlds/d435i_rgbd_test.world`：新增固定 RGB-D 测试场景。
- `launch/gazebo_arm.launch.py`：新增 `world` 参数和 RGB-D 参数透传。
- `CMakeLists.txt`：安装 `worlds/`。
- `package.xml`：声明 `gazebo_plugins` 运行依赖。

### Topic 与 frame 约定

默认启动后发布：

```text
/d435i/color/image_raw
/d435i/color/camera_info
/d435i/depth/image_raw
/d435i/depth/camera_info
/d435i/depth/points
```

当前 Humble `libgazebo_ros_camera.so` 对单个 depth sensor 只配置一个 `frame_name`，本阶段设置为：

```text
camera_depth_optical_frame
```

因此 color image、depth image、两个 CameraInfo 和 PointCloud2 的 header frame 均为 `camera_depth_optical_frame`。官方 RealSense nominal TF 仍保留 `camera_color_optical_frame` 和 `camera_depth_optical_frame`，但 Gazebo 当前是一台共位虚拟 RGB-D 相机；后续如果需要严格模拟 D435i color/depth 物理外参，应拆成独立 color/depth sensor 或增加专用数据同步节点。

### 工程风险说明

当前 RGB-D 数据来自 Gazebo Classic 通用插件的模拟输出，不是 RealSense D435i 真机驱动的原生数据。由于插件只能配置一个 `frame_name`，本阶段暂时把彩色图像、深度图、CameraInfo 和点云都标记为 `camera_depth_optical_frame`。这只是当前仿真的临时接口限制，不应被视为真实 D435i 的标准接口。

真实 D435i 通常应区分：

- 彩色图像与彩色 CameraInfo：`camera_color_optical_frame`；
- 原始深度图与原始点云：`camera_depth_optical_frame`；
- 两个 optical frame 之间存在固定外参。

后续 AprilTag 检测基于彩色图像，因此其位姿结果应以 `camera_color_optical_frame` 为参考坐标系，不能长期依赖当前全部使用 depth frame 的临时配置。彩色图像和原始深度图的同一像素位置也不能默认一一对应；如果需要根据彩色像素读取深度，必须使用 depth-to-color alignment，或根据内参、外参完成重新投影。

上层算法不得硬编码 topic、frame、分辨率、FOV 或相机内参，应通过 ROS 参数、launch remap、TF 和 `CameraInfo` 获取。当前阶段的 RGB-D 测试仍然有效，但它只证明传感器数据链路、TF 基础结构和仿真功能正常，不能证明真实 D435i 的精度、噪声、延迟和对齐性能。

默认 RGB-D 参数：

```text
update_rate: 15 Hz
width: 640
height: 480
horizontal_fov: 1.211 rad
near/far: 0.1 / 5.0 m
noise stddev: 0.0
```

### 测试场景

启动测试 world：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=false use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_rgbd_test.world
```

测试物体均为 static visual geometry：

| 名称 | 位姿 xyz rpy | 尺寸 | 颜色 |
|---|---|---|---|
| `rgbd_ground_plane` | `0 0 0 0 0 0` | `6 x 6 m` plane | gray |
| `rgbd_back_wall` | `2.30 0.00 0.75 0 0 0` | `0.04 x 2.40 x 1.50 m` box | off-white |
| `rgbd_box_red_060m` | `1.08 -0.22 0.44 0 0 0` | `0.16 m` cube | red |
| `rgbd_box_green_100m` | `1.48 0.00 0.44 0 0 0` | `0.20 m` cube | green |
| `rgbd_box_blue_150m` | `1.98 0.24 0.44 0 0 0` | `0.24 m` cube | blue |
| `rgbd_cylinder_yellow` | `1.36 0.34 0.18 0 0 0` | radius `0.08 m`, length `0.36 m` | yellow |

这些位置按当前零位姿相机前方布置，三个 box 距默认相机约 0.6 m、1.0 m、1.5 m。

### 验证结果

构建：

```bash
source setup_env.bash
colcon build --packages-select manipulator --symlink-install --cmake-force-configure
```

结果：`manipulator` 构建成功，仅有既有 CMake/编译 warning。

URDF 与 world 静态检查：

```bash
xacro $(ros2 pkg prefix manipulator)/share/manipulator/arm_with_d435i.urdf.xacro \
  camera_enabled:=true rgbd_enabled:=true use_ros2_control:=true fix_base_to_world:=true \
  > /tmp/windylab_arm_rgbd.urdf
check_urdf /tmp/windylab_arm_rgbd.urdf
gz sdf -k src/arm-platform/worlds/d435i_rgbd_test.world
```

结果：

- `check_urdf` 成功解析；
- TF 链仍包含 `world -> base_link -> ... -> link6 -> camera_link -> camera_depth_optical_frame`；
- 展开后的 URDF 包含 `camera_rgbd_sensor`、`libgazebo_ros_camera.so` 和 `/d435i` remap；
- `gz sdf -k` 返回 `Check complete`。

运行后 topic：

```text
/d435i/color/camera_info
/d435i/color/image_raw
/d435i/depth/camera_info
/d435i/depth/image_raw
/d435i/depth/points
```

实测频率：

```text
/d435i/color/image_raw: 约 13.7 Hz
/d435i/depth/image_raw: 约 15.0 Hz
/d435i/depth/points: 约 11.8 Hz
```

CameraInfo 检查：

- color/depth `width=640`，`height=480`；
- `K` 和 `P` 非零；
- `frame_id=camera_depth_optical_frame`；
- color/depth CameraInfo 时间戳相差约 0.067 s，可被普通视觉节点同步订阅。

数据体检：

```text
image: 640x480 encoding=rgb8 frame=camera_depth_optical_frame bytes=921600
cloud: 640x480 frame=camera_depth_optical_frame point_step=32 row_step=20480
finite_points_in_first_10000: 9344
sample_z_range: 1.856..1.856
```

运动联动验证：

```text
before_mean_depth_first_50000=1.856 finite=46698
after_mean_depth_first_50000=1.327 finite=50000
joint2=-0.350 joint3=0.200
```

说明 `/student/joint_command` 经桥接节点驱动 Gazebo 关节后，末端 D435i 的点云数据随相机位姿变化。

控制链路回归：

- `joint_state_broadcaster` 和 `arm_position_controller` 均为 `active`；
- 六个 `joint*/position` command interface 均为 `available` 且 `claimed`；
- `/joint_states` 只有 1 个发布者：`joint_state_broadcaster`；
- `move_arm_demo_6dof.py` 可继续向 `/student/joint_command` 发布，Gazebo 关节状态随 demo 变化。

### 可视化验收方法

启动仿真：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=true use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_rgbd_test.world
```

查看 RGB 或深度图像：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
ros2 run rqt_image_view rqt_image_view
```

在 `rqt_image_view` 中选择：

```text
/d435i/color/image_raw
/d435i/depth/image_raw
```

预期效果：彩色图像能看到 Gazebo 测试场景中的墙、box 和 cylinder；深度图能看到近远物体的深度差异。如果缺少工具，安装：

```bash
sudo apt install ros-humble-rqt-image-view
```

查看点云：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
rviz2
```

RViz2 设置：

- `Global Options / Fixed Frame` 设置为 `world`；如临时 TF 显示异常，可设为 `camera_depth_optical_frame` 只看点云本体；
- 添加 `PointCloud2`，topic 选择 `/d435i/depth/points`；
- 点云显示建议 `Style=Points`，`Size=0.005` 或 `0.01`；
- `Color Transformer` 优先试 `RGB8`，若显示异常则用 `AxisColor` 或 `FlatColor`；
- 可同时添加 `RobotModel` 和 `TF`，检查机械臂、D435i frame 与点云的相对关系。

预期效果：RViz2 中能看到测试墙面、box 和 cylinder 形成的三维点云；发布 `/student/joint_command` 后，机械臂末端相机运动，点云视角随之变化。


## 阶段 3：加入 D435i IMU 仿真

阶段 3 在现有 D435i 官方描述、Gazebo ROS 2 control 和 RGB-D 输出基础上加入 Gazebo Classic IMU sensor。本阶段只发布 `sensor_msgs/msg/Imu`，不使用固定发布器伪造数据，不加入 AprilTag、SLAM 或视觉闭环控制。

### 新增与修改文件

- `config/sensors/d435i_gazebo.xacro`：新增 `windylab_d435i_gazebo_imu` 宏，使用 Gazebo Classic `sensor type="imu"` 和 ROS 2 Humble 插件 `libgazebo_ros_imu_sensor.so`。
- `config/arm_with_d435i.urdf.xacro`：新增 IMU xacro 参数并调用 IMU wrapper。
- `launch/gazebo_arm.launch.py`：新增 IMU launch 参数透传。

默认 IMU 参数：

```text
imu_enabled: true
imu_namespace: d435i
imu_topic: imu
imu_frame_name: camera_accel_optical_frame
imu_update_rate: 200 Hz
imu_visualize: false
imu_noise_mean: 0.0
imu_angular_velocity_noise_stddev: 0.0
imu_linear_acceleration_noise_stddev: 0.0
```

IMU sensor 挂接在 `${camera_name}_link` 上，属于 D435i 子树；消息 `header.frame_id` 使用官方 D435i nominal TF 中已有的 `camera_accel_optical_frame`。当前官方描述提供 `camera_accel_frame`、`camera_accel_optical_frame`、`camera_gyro_frame`、`camera_gyro_optical_frame`，没有统一的 `camera_imu_optical_frame`，因此本阶段不新增虚构 frame。

### 插件语义

本机 Humble 可用 IMU 插件：

```text
/opt/ros/humble/lib/libgazebo_ros_imu_sensor.so
```

插件默认发布 topic 为 `~/out`，本阶段通过：

```xml
<remapping>~/out:=imu</remapping>
```

在 ROS namespace `d435i` 下发布为：

```text
/d435i/imu
```

本阶段显式设置：

```xml
<initial_orientation_as_reference>false</initial_orientation_as_reference>
```

因此 orientation 以 world 为参考，符合插件默认的 REP 145 语义。实测消息包含 orientation、angular velocity 和 linear acceleration 字段；协方差矩阵为全 0，表示未知协方差。

静止样例中线加速度模长为 `9.800000 m/s^2`，说明当前 Gazebo IMU 输出包含重力项。

### 静态检查

URDF 展开与解析：

```bash
source setup_env.bash
xacro src/arm-platform/config/arm_with_d435i.urdf.xacro \
  camera_enabled:=true rgbd_enabled:=true imu_enabled:=true \
  use_ros2_control:=true fix_base_to_world:=true \
  > /tmp/windylab_arm_imu.urdf
check_urdf /tmp/windylab_arm_imu.urdf
```

结果：

- `check_urdf` 成功解析；
- root link 仍为 `world`；
- TF 链仍包含 `world -> base_link -> ... -> link6 -> camera_link -> camera_accel_optical_frame`；
- 展开后的 URDF 包含 `camera_imu_sensor`、`libgazebo_ros_imu_sensor.so`、`~/out:=imu` 和 `frame_name=camera_accel_optical_frame`。

### 构建与启动

构建：

```bash
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
```

结果：`manipulator` 构建成功。

启动验证：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=false use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_rgbd_test.world \
  imu_enabled:=true rgbd_enabled:=true
```

结果：

- `windylab_arm` 成功 spawn；
- RGB-D 插件继续发布 color、depth、CameraInfo 和 point cloud；
- `joint_state_broadcaster` 和 `arm_position_controller` 均为 `active`；
- `/joint_states` 仍只有 1 个发布者。

### Topic 验证

新增 topic：

```text
/d435i/imu
```

topic 类型与发布者：

```text
Type: sensor_msgs/msg/Imu
Publisher count: 1
Node name: camera_imu_controller
Node namespace: /d435i
```

实测频率：

```text
/d435i/imu: 约 199.96 Hz
```

IMU 打开后 RGB-D 仍稳定发布：

```text
/d435i/color/image_raw: 约 15.0 Hz
/d435i/depth/points: 约 15.0 Hz
```

静止 IMU 样例：

```text
frame_id: camera_accel_optical_frame
orientation: x=-7.75e-16 y=1.60e-16 z=1.95e-17 w=1.0
angular_velocity: x=-1.70e-15 y=-1.67e-14 z=-2.81e-16 rad/s
linear_acceleration: x=-6.94e-13 y=5.80e-15 z=9.800000 m/s^2
```

### 运动测试

使用与 `move_arm_demo_6dof.py` 相同的 50 Hz 正弦关节位置命令，对前三个关节施加 `0.3 rad` 幅值、`4 s` 周期的轨迹，并同步统计 `/d435i/imu`。

统计结果：

```text
static_first_1s: samples=202 frame=camera_accel_optical_frame
static_first_1s: angular_norm min=0.000000 max=0.000000 rad/s
static_first_1s: accel_norm min=9.800000 max=9.800000 m/s^2
static_first_1s: stamp_start=110.730 stamp_end=111.735 monotonic=True

moving_last_5s: samples=1000 frame=camera_accel_optical_frame
moving_last_5s: angular_norm min=0.000000 max=0.000000 rad/s
moving_last_5s: accel_norm min=9.800000 max=9.800000 m/s^2
moving_last_5s: accel_z min=8.506501 max=9.800000 m/s^2
moving_last_5s: stamp_start=111.740 stamp_end=116.735 monotonic=True
```

线加速度随相机姿态和运动发生变化，时间戳持续递增且 frame 固定。当前角速度没有随关节运动变化；进一步检查 `/joint_states.velocity` 也为 0，说明阶段 1.5 当前使用的 Gazebo position interface 会更新关节位置和 link pose，但没有向 Gazebo 物理链路提供连续关节速度。IMU 数据仍来自 Gazebo IMU sensor，并未通过固定发布器或自定义节点伪造。后续若需要可用的仿真角速度，应将 Gazebo 控制链路改为速度/力矩/轨迹控制，使 Gazebo 物理引擎产生真实 link velocity。

### 阶段 3 结论

- `/d435i/imu` 已由 Gazebo Classic IMU sensor 和 `gazebo_ros_imu_sensor` 插件稳定发布；
- IMU frame 位于 D435i TF 子树中，使用 `camera_accel_optical_frame`；
- 静止时 orientation 接近单位四元数，角速度接近 0，线加速度稳定且包含重力项；
- 运动时线加速度和时间戳行为正常，但角速度受当前 position interface 控制链路限制仍为 0；
- RGB-D 与 IMU 可同时运行；
- 机械臂 Gazebo 控制链路和 `/joint_states` 单发布者约束保持不变。

## 阶段 3 修正：加入 physical_dynamics 控制模式

本修正只处理阶段 3 中 Gazebo 机械臂运动时 D435i IMU `angular_velocity` 始终为 0 的问题，不加入 Base 扰动、AprilTag、SLAM 或视觉闭环，也不修改现有 IK 和动力学算法。

### 根因分析

阶段 1.5/3 的默认 Gazebo 控制链路使用：

```text
/student/joint_command
  -> student_joint_command_bridge.py
  -> /arm_position_controller/commands
  -> position_controllers/JointGroupPositionController
  -> joint*/position command interface
```

该链路能更新 Gazebo 关节 position 和 link pose，适合 RGB-D、TF、AprilTag 等可视化/几何链路验证；但实测 `/joint_states.velocity` 在运动过程中仍为 0。Gazebo IMU 插件读取 Gazebo sensor/physics 中的 link angular velocity，而不是从 TF 数值微分，因此 D435i IMU 的 `angular_velocity` 也保持为 0。

### 采用方案

保留原稳定模式并新增可切换模式：

```text
control_mode:=kinematic_visualization
  -> joint_state_broadcaster + arm_position_controller + student_joint_command_bridge.py

control_mode:=physical_dynamics
  -> joint_state_broadcaster + arm_velocity_controller + student_joint_velocity_bridge.py
```

物理模式使用 `velocity_controllers/JointGroupVelocityController`，让 Gazebo 通过 `joint*/velocity` command interface 真实积分关节运动。新增的 `student_joint_velocity_bridge.py` 仍订阅原有 `/student/joint_command`，保持“目标关节角”语义不变；节点根据当前 `/joint_states.position` 和目标位置生成速度命令，并使用 `/student/joint_command.velocity` 作为短时前馈。命令流超时后会清零前馈速度，只保持最后目标位置，避免 demo 退出后继续运动。

### 修改文件

- `config/sensors/gazebo_ros2_control.xacro`：六个关节增加 `velocity` command interface，保留 `position` command interface。
- `config/gazebo_controllers.yaml`：新增 `arm_velocity_controller`。
- `scripts/student_joint_velocity_bridge.py`：新增目标位置到速度命令的桥接节点，不发布 `/joint_states` 或 IMU。
- `launch/gazebo_arm.launch.py`：新增 `control_mode` 和 velocity bridge 参数，并按模式加载对应 controller/bridge。
- `CMakeLists.txt`、`package.xml`：安装新脚本并声明 `velocity_controllers` 运行依赖。

### 新增 launch 参数

```text
control_mode:=kinematic_visualization | physical_dynamics
velocity_kp:=4.0
velocity_feedforward_scale:=1.0
velocity_max_velocity:=1.0
velocity_position_tolerance:=0.005
velocity_publish_rate:=100.0
velocity_command_timeout_sec:=0.25
```

### 启动命令

可视化/几何验证模式：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=false use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_rgbd_test.world \
  control_mode:=kinematic_visualization
```

IMU/动力学实验模式：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=false use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_rgbd_test.world \
  control_mode:=physical_dynamics
```

### 验证结果

构建：

```bash
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
```

结果：`manipulator` 构建成功。

静态检查：

```bash
xacro src/arm-platform/config/arm_with_d435i.urdf.xacro \
  camera_enabled:=true rgbd_enabled:=true imu_enabled:=true \
  use_ros2_control:=true fix_base_to_world:=true \
  > /tmp/windylab_arm_velocity.urdf
check_urdf /tmp/windylab_arm_velocity.urdf
```

结果：`check_urdf` 成功解析；六个关节均包含 `position` 和 `velocity` command interface。

`kinematic_visualization` 回归：

```text
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active
arm_position_controller position_controllers/JointGroupPositionController active
/joint_states publisher count: 1
/d435i/imu: 约 199.94 Hz
```

`physical_dynamics` 控制器：

```text
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active
arm_velocity_controller velocity_controllers/JointGroupVelocityController active
```

硬件接口：

```text
joint1/position [available] [unclaimed]
joint1/velocity [available] [claimed]
joint2/position [available] [unclaimed]
joint2/velocity [available] [claimed]
joint3/position [available] [unclaimed]
joint3/velocity [available] [claimed]
joint4/position [available] [unclaimed]
joint4/velocity [available] [claimed]
joint5/position [available] [unclaimed]
joint5/velocity [available] [claimed]
joint6/position [available] [unclaimed]
joint6/velocity [available] [claimed]
state interfaces: joint*/position, joint*/velocity, joint*/effort
```

`/joint_states` 发布者：

```text
Publisher count: 1
Node name: joint_state_broadcaster
Subscribers: student_joint_velocity_bridge, robot_state_publisher
```

运动测试使用与 `move_arm_demo_6dof.py` 相同的 50 Hz 正弦关节目标。统计结果：

```text
static: joint_samples=103
static: joint_vel_first3_max_abs=0.000000 mean_abs=0.000000
static: imu_ang_norm_max=0.000000 mean=0.000000

moving: joint_samples=450
moving: joint1_pos_start=0.2141 joint1_pos_end=0.3021
moving: joint_vel_first3_max_abs=0.485792 mean_abs=0.297128
moving: sample_vel_first3=[-0.18465, 0.295867, 0.478437]
moving: imu_samples=900 frame=camera_accel_optical_frame
moving: imu_ang_norm_max=0.841186 mean=0.660292
moving: sample_angular=(-0.180479, -0.774339, -0.038101)
direction_check_joint1: agree=416/416 ratio=1.000

stopped: joint_samples=150
stopped: joint_vel_first3_max_abs=0.000000 mean_abs=0.000000
stopped: imu_samples=300 frame=camera_accel_optical_frame
stopped: imu_ang_norm_max=0.000000 mean=0.000000
```

实际 demo 回归：

```bash
timeout -s INT 4 python3 src/arm-platform/demo/move_arm_demo_6dof.py
```

结果：demo 正常向 `/student/joint_command` 发布；Gazebo 关节由 velocity controller 平滑运动。demo 停止 1 秒后 `/joint_states.velocity` 回到 `1e-15` 量级，`/d435i/imu.angular_velocity` 回到 `1e-14` 量级。

RGB-D/IMU 回归：

```text
/d435i/color/image_raw: 约 14-15 Hz
/d435i/imu: 约 200.0 Hz
```

结论：

- `physical_dynamics` 模式下 `/joint_states.velocity` 在运动期间连续、非零，方向与目标前馈速度一致；
- D435i IMU `angular_velocity` 随机械臂运动明显变化，停止后回到接近 0；
- `/joint_states` 仍只有 Gazebo `joint_state_broadcaster` 一个权威发布源；
- RViz 和 Gazebo 均由同一 `/joint_states` 驱动，姿态一致；
- 未观察到明显抖动、模型爆炸或控制发散；
- 当前无碰撞可视化模型继续使用，本修正不处理 collision geometry。

## 阶段 4：相机安装位置参数化

阶段 4 的目标是允许同一套官方 D435i 描述、RGB-D 插件和 IMU 插件在末端安装与 Base 安装之间切换，不复制模型，也不修改官方 RealSense xacro。

### 新增与修改文件

- `config/arm_with_d435i.urdf.xacro`：新增 `camera_mount_mode` 参数，并支持 `camera_parent_link`、`camera_xyz`、`camera_rpy` 使用 `auto` 默认值。
- `launch/gazebo_arm.launch.py`：新增 `camera_mount_mode:=ee|base`，并在 launch 层解析默认 parent 和安装位姿。
- `launch/student_arm.launch.py`：同步支持 `camera_mount_mode:=ee|base`，用于纯学生/RViz 启动入口。

默认参数：

```text
camera_mount_mode:=ee
  -> camera_parent_link:=link6
  -> camera_xyz:=0.06 0 0.04
  -> camera_rpy:=0 0 0

camera_mount_mode:=base
  -> camera_parent_link:=base_link
  -> camera_xyz:=0.02 0 0.06
  -> camera_rpy:=0 0 0
```

显式传入 `camera_parent_link`、`camera_xyz` 或 `camera_rpy` 时优先生效。非法模式会在 launch 阶段报错：

```text
camera_mount_mode must be one of: ee, base
```

### 静态检查

URDF 展开与解析：

```bash
source setup_env.bash
xacro src/arm-platform/config/arm_with_d435i.urdf.xacro \
  camera_enabled:=true camera_mount_mode:=ee \
  rgbd_enabled:=true imu_enabled:=true \
  use_ros2_control:=true fix_base_to_world:=true \
  > /tmp/d435i_ee.urdf
check_urdf /tmp/d435i_ee.urdf

xacro src/arm-platform/config/arm_with_d435i.urdf.xacro \
  camera_enabled:=true camera_mount_mode:=base \
  rgbd_enabled:=true imu_enabled:=true \
  use_ros2_control:=true fix_base_to_world:=true \
  > /tmp/d435i_base.urdf
check_urdf /tmp/d435i_base.urdf
```

结果：

- `ee` 模式 TF 链为 `world -> base_link -> ... -> link6 -> camera_bottom_screw_frame -> camera_link -> camera_depth_optical_frame`；
- `base` 模式 TF 链为 `world -> base_link -> camera_bottom_screw_frame -> camera_link -> camera_depth_optical_frame`，相机固定在机械臂基座上，不随内部关节运动；
- 两种模式均包含 `camera_rgbd_sensor`、`camera_imu_sensor`、`camera_depth_optical_frame` 和 `camera_accel_optical_frame`；
- 显式覆盖 `camera_parent_link:=link6 camera_xyz:="0.01 0.02 0.03" camera_rpy:="0.1 0.2 0.3"` 可覆盖 `camera_mount_mode:=base` 的默认 parent/pose；
- `gz sdf -k src/arm-platform/worlds/d435i_rgbd_test.world` 返回 `Check complete`。

### Gazebo 验收

末端模式启动：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=false use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_rgbd_test.world \
  camera_mount_mode:=ee control_mode:=kinematic_visualization
```

Base 模式启动：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=false use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_rgbd_test.world \
  camera_mount_mode:=base control_mode:=kinematic_visualization
```

两种模式均成功 spawn：

```text
SpawnEntity: Successfully spawned entity [windylab_arm]
```

控制器状态：

```text
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active
arm_position_controller position_controllers/JointGroupPositionController active
```

两种模式均发布相同 topic：

```text
/d435i/color/image_raw   sensor_msgs/msg/Image
/d435i/depth/image_raw   sensor_msgs/msg/Image
/d435i/depth/points      sensor_msgs/msg/PointCloud2
/d435i/imu               sensor_msgs/msg/Imu
/joint_states
```

发布验证命令：

```bash
ros2 topic pub --once /student/joint_command sensor_msgs/msg/JointState \
  "{name: ['joint1','joint2','joint3','joint4','joint5','joint6'], position: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

结果：两种模式下 `/joint_states` 中 `joint1` 均到达约 `0.3 rad`。

末端模式 TF：

```text
link6 -> camera_depth_optical_frame
before/after joint1 command:
translation [0.071, 0.018, 0.053], rotation [-90 deg, 0 deg, -90 deg]

base_link -> camera_depth_optical_frame
before: translation [0.424, 0.019, 0.427], RPY [-90 deg, 0 deg, -90 deg]
after:  translation [0.424, -0.108, 0.413], RPY [-90 deg, 17.189 deg, -90 deg]
```

说明：相机相对 `link6` 保持固定，随末端运动。

Base 模式 TF：

```text
base_link -> camera_depth_optical_frame
before/after joint1 command:
translation [0.031, 0.018, 0.072], rotation [-90 deg, 0 deg, -90 deg]

link1 -> camera_depth_optical_frame
before: translation [-0.031, 0.018, 0.072], RPY [-90 deg, 0 deg, -90 deg]
after:  translation [-0.031, 0.038, 0.064], RPY [-90 deg, -17.189 deg, -90 deg]
```

说明：相机相对 `base_link` 保持固定，不随 joint1 或其他机械臂内部关节运动；安装高度保持在约 `0.07 m`。

### 学生入口回归

末端模式：

```bash
ros2 launch manipulator student_arm.launch.py \
  use_rviz:=False camera_enabled:=true camera_mount_mode:=ee
```

TF 验证：

```text
link6 -> camera_depth_optical_frame
translation [0.071, 0.018, 0.053]
```

Base 模式：

```bash
ros2 launch manipulator student_arm.launch.py \
  use_rviz:=False camera_enabled:=true camera_mount_mode:=base
```

TF 验证：

```text
base_link -> camera_depth_optical_frame
translation [0.031, 0.018, 0.072]
```

两种模式中 `robot_state_publisher` 均成功加载 `camera_depth_optical_frame`。

### 阶段 4 结论

- 只通过 launch 参数即可在 `camera_mount_mode:=ee` 与 `camera_mount_mode:=base` 间切换；
- 两种模式复用同一套 D435i xacro、RGB-D sensor 和 IMU sensor；
- RGB-D/IMU topic 命名保持一致；
- 末端模式相机随 `link6` 刚性运动；
- Base 模式相机相对 `base_link` 固定，安装在基座附近且不随机械臂内部关节运动；
- 原有 Gazebo 控制链路和学生启动入口保持可用。

## 阶段 6：在环境中加入静态 AprilTag

阶段 6 的目标是在独立 Gazebo Classic 测试 world 中加入一个尺寸已知、位姿固定、可被 D435i RGB 相机看见的静态 AprilTag。本阶段只加入视觉目标和测试 world，不运行 AprilTag 检测器，不新增 detection topic，也不修改机械臂控制逻辑。

### 新增与修改文件

- `worlds/d435i_apriltag_test.world`：新增独立 AprilTag 测试场景，包含 sun、地面、背景墙、一个静态 AprilTag 和一个绿色深度参考 box。
- `models/apriltag_36h11_00000/`：新增 Gazebo Classic AprilTag 模型，包含 `model.config`、`model.sdf`、纹理、material script、来源说明和许可证副本。
- `docs/third_party_sources.md`：记录 AprilTag 纹理来源、上游 commit、许可证和生成说明。
- `CMakeLists.txt`：安装 `models/` 到 `share/manipulator/models`。
- `launch/gazebo_arm.launch.py`：将包内 `models` 安装目录加入 `GAZEBO_MODEL_PATH`，使 `model://apriltag_36h11_00000` 可在安装后解析。

### AprilTag 定义

第一版 Tag 参数：

```text
family: tag36h11
id: 0
static: true
detection tag size: 0.20 m
texture plane size: 0.30 m x 0.30 m
```

尺寸定义：

- `0.20 m` 指 AprilTag 检测器应配置的黑色有效检测边长，也就是上游 AprilTag 文档中 detection corners 之间的 tag size；
- 纹理总平面为 12 个 cell，黑色有效检测边为 8 个 cell，因此总平面尺寸为 `0.20 * 12 / 8 = 0.30 m`；
- 外层白色 quiet zone/背景不计入 detector `tag_size`。

纹理来源：

```text
repository: https://github.com/AprilRobotics/apriltag
commit: 0e16a12dd380fd607e4afd54712ee9b1ffb9ec8f
license: BSD 2-Clause
source files: tag36h11.c, apriltag.c, README.md, LICENSE.md
generated date: 2026-07-13
```

纹理按上游 `apriltag_to_image()` 布局语义生成：`width_at_border=8`、`total_width=10`、`reversed_border=false`，并额外增加 1 个 cell 的白色 quiet zone。PNG 文件为：

```text
models/apriltag_36h11_00000/materials/textures/tag36_11_00000.png
```

### World 位姿

Tag 在测试 world 中通过 include 放置：

```xml
<include>
  <name>apriltag_36h11_00000_target</name>
  <uri>model://apriltag_36h11_00000</uri>
  <pose>1.23 0.0 0.35 0 -1.57079632679 0</pose>
</include>
```

模型局部 Tag 平面法向为 `+Z`。world pose 中 `pitch=-pi/2` 后，Tag 平面法向为 world `-X`，面向默认朝 world `+X` 方向观察的 D435i。实测 RGB 图中 Tag 未出现镜像或翻转。

### 静态检查

SDF 检查：

```bash
source setup_env.bash
gz sdf -k src/arm-platform/models/apriltag_36h11_00000/model.sdf
GAZEBO_MODEL_PATH=$PWD/src/arm-platform/models:/usr/share/gazebo-11/models \
  gz sdf -k src/arm-platform/worlds/d435i_apriltag_test.world
```

结果均为：

```text
Check complete
```

构建验证：

```bash
source setup_env.bash
colcon build --packages-select manipulator
```

结果：构建通过；仍有既有 Pinocchio/eigenpy 触发的 Boost Python header CMake warning。

### Gazebo 验收

末端模式启动：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=false use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_apriltag_test.world \
  camera_mount_mode:=ee control_mode:=kinematic_visualization
```

结果：

```text
SpawnEntity: Successfully spawned entity [windylab_arm]
/d435i/color/image_raw 发布 640x480 rgb8
```

Base 模式启动：

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py \
  gui:=false use_rviz:=false \
  world:=$(ros2 pkg prefix manipulator)/share/manipulator/worlds/d435i_apriltag_test.world \
  camera_mount_mode:=base control_mode:=kinematic_visualization
```

结果：

```text
SpawnEntity: Successfully spawned entity [windylab_arm]
/d435i/color/image_raw 发布 640x480 rgb8
```

保存的 RGB 验证截图：

```text
/tmp/d435i_phase6_ee_color.png
/tmp/d435i_phase6_base_color.png
```

两种模式的 RGB 图像中均可看到完整 AprilTag，Tag 位于背景墙前，方向一致且未被机械臂遮挡。Base 模式图像中还可看到绿色 box，用于辅助验证深度场景几何。

### 阶段 6 结论

- 独立 AprilTag world 已加入；
- AprilTag 是 Gazebo Classic 静态视觉模型，不是 logical camera；
- Tag family、id、尺寸定义、纹理来源、许可证和 world pose 已记录；
- 包内 `models/` 已安装并由 launch 自动加入 `GAZEBO_MODEL_PATH`；
- `camera_mount_mode:=ee` 和 `camera_mount_mode:=base` 均能看到清晰 Tag；
- 本阶段未运行 AprilTag 检测器，检测链路留给阶段 7。

## 阶段 7：AprilTag 检测链路基础验证

阶段 7 的目标是从 Gazebo D435i 彩色图像和 CameraInfo 运行真实 AprilTag 检测，并将检测结果与 Gazebo ground truth 做基础几何对比。本阶段不使用 Gazebo ground truth 伪造检测结果，不接入机械臂控制闭环，也不实现 SLAM 或扰动补偿。

### 依赖

本阶段安装系统包：

```bash
sudo apt-get install -y ros-humble-apriltag-ros ros-humble-image-proc
```

实际新增 ROS 包包括 `apriltag_ros`、`apriltag_msgs`、`apriltag`、`image_proc` 和图像传输插件。`apriltag_ros` Humble 版提供：

```text
apriltag_ros/apriltag_node
apriltag_msgs/msg/AprilTagDetectionArray
```

### 新增与修改文件

- `config/apriltag_36h11_00000.yaml`：配置 `tag36h11`、ID `0`、检测边长 `0.20 m`、tag frame `apriltag_36h11_00000`。
- `launch/d435i_apriltag_test.launch.py`：启动 AprilTag 测试 world、D435i RGB-D、`apriltag_ros` 检测节点，并可选运行 ground truth 验证脚本。
- `scripts/check_apriltag_ground_truth.py`：订阅检测结果、读取检测 TF、通过 Gazebo state service 获取 Tag ground truth，并输出误差统计和 CSV。
- `worlds/d435i_apriltag_test.world`：加载 `libgazebo_ros_state.so`，为验证脚本提供 `/get_entity_state`。
- `package.xml`、`CMakeLists.txt`：声明依赖并安装验证脚本。

### 检测接口

启动命令：

```bash
source setup_env.bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false camera_mount_mode:=ee
```

该 launch 显式设置：

```text
rgbd_frame_name:=camera_color_optical_frame
```

因此 AprilTag 输入图像和 CameraInfo 使用同一 frame：

```text
/d435i/color/image_raw
/d435i/color/camera_info
frame_id: camera_color_optical_frame
```

检测节点：

```text
node: /apriltag/apriltag
topic: /apriltag/detections
type: apriltag_msgs/msg/AprilTagDetectionArray
TF: camera_color_optical_frame -> apriltag_36h11_00000
```

ID 检查：

```text
id=0 family=tag36h11 hamming=0 margin=95.321 detections=1
```

检测频率：

```text
/apriltag/detections: 约 14.98-15.10 Hz
```

控制链路回归：

```text
joint_state_broadcaster: active
arm_position_controller: active
/joint_states publisher count: 1, node=joint_state_broadcaster
```

### Ground truth 对比工具

验证脚本默认比较：

```text
detected: camera_color_optical_frame -> apriltag_36h11_00000
ground truth: inverse(world -> camera_color_optical_frame) * (world -> apriltag_36h11_00000_target)
```

Gazebo state service：

```text
/get_entity_state
```

验证命令示例：

```bash
ros2 run manipulator check_apriltag_ground_truth.py \
  --duration 8 --sample-hz 5 \
  --output-csv /tmp/d435i_apriltag_phase7_pose0.csv
```

当前 AprilTag Gazebo 模型 frame 与 `apriltag_ros` PnP tag frame 在本模型纹理约定下无需额外旋转：

```text
tag_frame_rpy_in_model: 0 0 0
```

### 误差结果

三组末端相机位姿下均持续检测到 ID 0，丢失率为 0：

| 场景 | 关节命令 `[j1..j6]` rad | 检测率 | 丢失率 | 检测距离均值 | GT 距离均值 | 位置误差均值 | 位置误差标准差 | 姿态误差均值 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pose0 | `[0, 0, 0, 0, 0, 0]` | `15.103 Hz` | `0.000` | `0.8110 m` | `0.8061 m` | `0.0168 m` | `0.0000 m` | `0.00 deg` |
| pose1 | `[0.20, -0.25, 0.18, 0.0, -0.10, 0.0]` | `14.983 Hz` | `0.000` | `0.7282 m` | `0.7281 m` | `0.0159 m` | `0.0000 m` | `0.31 deg` |
| pose2 | `[-0.18, -0.35, 0.28, 0.12, -0.18, 0.10]` | `15.048 Hz` | `0.000` | `0.7110 m` | `0.7112 m` | `0.0157 m` | `0.0000 m` | `0.60 deg` |

CSV 日志：

```text
/tmp/d435i_apriltag_phase7_pose0.csv
/tmp/d435i_apriltag_phase7_pose1.csv
/tmp/d435i_apriltag_phase7_pose2.csv
```

### 阶段 7 结论

- AprilTag 检测从仿真 RGB 图像和 CameraInfo 得到，未读取 ground truth 伪造检测；
- `/apriltag/detections` 持续发布，ID 为 `0`，family 为 `tag36h11`；
- 检测 TF `camera_color_optical_frame -> apriltag_36h11_00000` 存在，方向与 Gazebo 模型约定一致；
- 估计距离与 Gazebo ground truth 一致，三组位姿位置误差约 `1.6-1.7 cm`；
- `libgazebo_ros_state.so` 只用于验证脚本取 ground truth，不参与检测节点；
- 未接入机械臂控制闭环，原 Gazebo 控制器和 `/joint_states` 单发布者约束保持不变。

### 已知现象

- 验证脚本启动早于 `joint_state_broadcaster` 时，前几秒可能出现 TF tree 未连接 warning；控制器 active 后可正常采样。
- 使用 `timeout` 或 Ctrl-C 停止 launch 时，既有 `student_joint_command_bridge.py` 偶尔在 `destroy_node()` 阶段打印 `KeyboardInterrupt` traceback；这不影响 AprilTag 检测结果，Gazebo、`apriltag_node` 和 `robot_state_publisher` 均正常退出。

## 阶段 7.5：闭环实验前置修正

阶段 7.5 只修正闭环实验前的感知/验证基础设施，不实现反扰动控制、视觉伺服、SLAM 或 Jacobian 闭环。

### RGB sensor frame 修正

阶段 7 的三姿态位置误差约 `1.6-1.7 cm`。展开 URDF 后确认 RGB-D sensor 原先实际挂载在：

```text
<gazebo reference="camera_link">
```

但 AprilTag 输入图像、CameraInfo 和检测 TF 使用：

```text
camera_color_optical_frame
```

官方 nominal extrinsics 中 `camera_link -> camera_color_frame` 有 `0 0.015 0` 平移，`camera_color_frame -> camera_color_optical_frame` 平移为 0。因此阶段 7 的固定位置误差与 sensor 原点使用 `camera_link` 而输出 frame 使用 color optical frame 一致。

本阶段将 RGB-D sensor reference 改为 `camera_color_frame`。该 frame 与 `camera_color_optical_frame` 原点完全重合，同时保持 Gazebo camera `+X` 前向和 ROS optical `+Z` 前向之间的既有旋转约定。修正后展开检查：

```text
camera_color_frame [{'name': 'camera_rgbd_sensor', 'type': 'depth'}]
camera_link [{'name': 'camera_imu_sensor', 'type': 'imu'}]
camera_color_joint camera_link -> camera_color_frame xyz=0 0.015 0 rpy=0 0 0
camera_color_optical_joint camera_color_frame -> camera_color_optical_frame xyz=0 0 0 rpy=-1.5707963267948966 0 -1.5707963267948966
```

### 新增脚本与 launch 参数

新增：

- `scripts/dynamic_ground_truth_logger.py`：按 AprilTag detection stamp 插值 GT，记录动态误差和 old/latest 方法误差。
- `scripts/check_camera_mount_regression.py`：验证 Base 相机外参恒定，并记录 Base 模式三姿态 AprilTag 误差。
- `docs/pre_closed_loop_validation_report.md`：阶段 7.5 完整测试报告。

修改：

- `config/sensors/d435i_gazebo.xacro`：RGB-D sensor 挂载到 `camera_color_frame`。
- `launch/gazebo_arm.launch.py`：新增并透传 `use_sim_time`。
- `launch/d435i_apriltag_test.launch.py`：新增 `use_sim_time`、`run_dynamic_logger`、`run_mount_regression` 等验证参数。
- `CMakeLists.txt`：安装新增脚本。

### EE 相机三姿态回归

启动：

```bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false \
  camera_mount_mode:=ee \
  control_mode:=kinematic_visualization
```

结果：

| 场景 | 关节命令 `[j1..j6]` rad | 检测率 | 丢失率 | 位置误差均值 | 姿态误差均值 | CSV |
|---|---:|---:|---:|---:|---:|---|
| pose0 | `[0, 0, 0, 0, 0, 0]` | `15.114 Hz` | `0.000` | `0.00159 m` | `1.376 deg` | `/tmp/d435i_phase75_ee_pose0.csv` |
| pose1 | `[0.20, -0.25, 0.18, 0.0, -0.10, 0.0]` | `16.226 Hz` | `0.000` | `0.00101 m` | `0.455 deg` | `/tmp/d435i_phase75_ee_pose1.csv` |
| pose2 | `[-0.18, -0.35, 0.28, 0.12, -0.18, 0.10]` | `15.101 Hz` | `0.000` | `0.00113 m` | `0.563 deg` | `/tmp/d435i_phase75_ee_pose2.csv` |

结论：位置误差从 `1.6-1.7 cm` 降至 `1-2 mm`，满足 `< 0.005 m` 目标。姿态误差 pose0 和 pose2 未满足 `< 0.5 deg` 目标，需继续定位 PnP、渲染采样或 tag/model frame 残差。

### 动态时间同步 Ground Truth

动态验证使用 `control_mode:=physical_dynamics`，并发布低速小幅关节正弦命令。`dynamic_ground_truth_logger.py` 保存最近 `10 s` GT buffer，按 detection stamp 查询检测 TF，并对 `world -> camera_color_optical_frame` 做平移线性插值和 quaternion SLERP。无法匹配的样本标记 invalid，不使用 latest TF 强行替代。

实测：

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

补充频率检查：

```text
/joint_states: 约 99.99 Hz
/tf: 约 75 Hz
/clock: 约 10 Hz
```

由于 `/clock` topic 频率较低，ROS-time timer 无法稳定触发 100 Hz GT 采样；logger 使用 steady wall timer 触发采样，但检测时间和 TF/GT 匹配仍基于仿真时间戳。动态最大匹配误差 `12 ms`，略高于一个 100 Hz 周期。

### Base 相机阶段 7 回归

启动：

```bash
ros2 launch manipulator d435i_apriltag_test.launch.py \
  gui:=false use_rviz:=false \
  camera_mount_mode:=base \
  control_mode:=kinematic_visualization
```

结果：

| 场景 | base-camera 平移变化 | base-camera 姿态变化 | 检测率 | 丢失率 | 位置误差均值 | 姿态误差均值 |
|---|---:|---:|---:|---:|---:|---:|
| pose0 | `0 m` | `0 rad` | `15.125 Hz` | `0.000` | `0.00190 m` | `5.1e-08 deg` |
| pose1 | `0 m` | `0 rad` | `15.000 Hz` | `0.000` | `0.00190 m` | `5.1e-08 deg` |
| pose2 | `0 m` | `0 rad` | `15.125 Hz` | `0.000` | `0.00190 m` | `5.1e-08 deg` |

CSV：

```text
/tmp/d435i_phase75_base_regression.csv
```

Base 模式满足 `base_link -> camera_color_optical_frame` 三姿态恒定要求，translation difference 和 orientation difference 均为 0。

### 阶段 7.5 结论

已完成 RGB sensor 原点修正、动态时间同步 GT logger、Base 相机阶段 7 回归。可以进入只依赖平移误差的闭环原型验证；不建议直接进入高精度 6D 位姿闭环或姿态闭环，需先继续定位 EE 姿态残差和动态时间匹配边界。

## 阶段 8.0：Base 扰动与实验基础设施

阶段 8.0 只建立可重复移动 Base 实验平台，不实现视觉补偿、SLAM、AprilTag 闭环或 Base 扰动补偿控制。

### 新增与修改文件

- `config/base_disturbance_profiles.yaml`：定义 `static`、`sine_x`、`sine_y`、`sine_z`、`random_translation_3d`，默认 30 s、100 Hz、seed 42。初版验收幅值为 XYZ `0.03/0.03/0.02 m`；为便于 Gazebo 可视化，后续默认调大为 XYZ `0.10/0.10/0.06 m`，正弦频率 `0.35 Hz`，随机频带 `0.10-1.0 Hz`。
- `scripts/generate_base_disturbance.py`：从 YAML 生成 CSV 轨迹。随机轨迹使用固定 seed 的有限正弦分量叠加，连续、带限且可重复；同 seed 输出 CSV 完全一致。
- `scripts/base_disturbance_replay.py`：读取轨迹 CSV，通过 Gazebo `/set_entity_state` 对 `windylab_arm` 整体模型重放 pose 和 twist，并记录 commanded pose、Gazebo actual model/base/link6/camera-proxy pose、实际频率、tracking error、关节连续性和速度命令。
- `scripts/baseline_velocity_command.py`：baseline 模式持续向 `/arm_velocity_controller/commands` 发布 6 维零速度。
- `launch/moving_base_stabilization.launch.py`：阶段 8.0 专用入口，默认 `control_mode:=physical_dynamics`、`fix_base_to_world:=false`、`velocity_command_source:=external`、`experiment_mode:=baseline`。
- `launch/gazebo_arm.launch.py`：新增 `velocity_command_source:=student_bridge|external`。默认仍为 `student_bridge`，保持原 `physical_dynamics` 行为；选择 `external` 时只加载 `arm_velocity_controller`，不启动 `student_joint_velocity_bridge.py`。
- `CMakeLists.txt`、`package.xml`：安装新增脚本/config/launch，并补充 `gazebo_msgs` 运行依赖。

### Ground Truth 记录约束

Gazebo `/link_states` 中实际存在的 arm links 为：

```text
windylab_arm::base_link
windylab_arm::link1
...
windylab_arm::link6
```

D435i 的 `camera_*` fixed links 在 Gazebo 中被合并，不是可通过 `/get_entity_state` 查询的独立 entity。因此阶段 8.0 的 Gazebo 直接 GT 记录：

- `world -> base_link`：`windylab_arm::base_link`
- `world -> link6`：`windylab_arm::link6`
- `world -> camera proxy`：默认 `windylab_arm::link6`

`camera_entity_name` 可在 launch 中覆盖；Base 相机实验可设置为 `windylab_arm::base_link`。这些 GT 字段只写入 replay CSV 和验收统计，不作为控制器输入。

### 静态检查与构建

Python 语法检查：

```bash
python3 -m py_compile \
  scripts/generate_base_disturbance.py \
  scripts/base_disturbance_replay.py \
  scripts/baseline_velocity_command.py \
  launch/gazebo_arm.launch.py \
  launch/moving_base_stabilization.launch.py
```

`fix_base_to_world:=false` URDF 检查：

```bash
xacro src/arm-platform/config/arm_with_d435i.urdf.xacro \
  camera_enabled:=true rgbd_enabled:=true imu_enabled:=true \
  use_ros2_control:=true fix_base_to_world:=false \
  > /tmp/windylab_arm_moving_base.urdf
check_urdf /tmp/windylab_arm_moving_base.urdf
```

结果：`check_urdf` 成功解析，root link 为 `base_link`，没有 `world_to_base_link` 固定关节。

构建：

```bash
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
```

结果：`manipulator` 构建成功，仅有既有 Boost/Python header check warning。

### 轨迹生成验证

同 seed 两次生成 `random_translation_3d` 后用 `cmp` 比较，CSV 完全一致。

修正记录：初版 `generate_static()` 让正弦分支 position/velocity 共用同一个 dict，导致速度覆盖 position。已修为独立 position/velocity dict，并重新完整验收。修正后正弦位置幅值为：

```text
sine_x: x=[-0.030000, 0.030000] m
sine_y: y=[-0.030000, 0.030000] m
sine_z: z=[-0.020000, 0.020000] m
```

### 完整 30 秒重放验收

启动模板：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/phase80_fixed_<profile>.csv \
  replay_output_csv:=/tmp/phase80_fixed_<profile>_replay.csv \
  start_delay_sec:=5.0
```

结果：

| 轨迹 | 样本数 | 实际频率 | set 失败 | 最大 pose error | 最大关节步进 | CSV |
|---|---:|---:|---:|---:|---:|---|
| `sine_x` | `3001` | `100.036 Hz` | `0` | `0.000074804 m` | `0.000000000 rad` | `/tmp/phase80_fixed_sine_x_replay.csv` |
| `sine_y` | `3001` | `100.001 Hz` | `0` | `0.000109182 m` | `0.000000000 rad` | `/tmp/phase80_fixed_sine_y_replay.csv` |
| `sine_z` | `3001` | `100.019 Hz` | `0` | `0.000038730 m` | `0.000000000 rad` | `/tmp/phase80_fixed_sine_z_replay.csv` |
| `random_translation_3d` | `3001` | `100.026 Hz` | `0` | `0.000106749 m` | `0.000000000 rad` | `/tmp/phase80_fixed_random_translation_3d_replay.csv` |

Gazebo actual base/link6 位姿随模型整体移动，不是只修改 TF。示例实际范围：

```text
sine_x actual_base_x=[-0.030000, 0.030000]
sine_y actual_base_y=[-0.030000, 0.030000]
sine_z actual_base_z=[-0.020000, 0.020000]
random actual_base_x=[-0.030000, 0.000049]
random actual_base_y=[-0.002128, 0.030000]
random actual_base_z=[-0.015468, 0.019804]
```

### Baseline 发布者检查

运行中检查：

```bash
ros2 topic info /arm_velocity_controller/commands --verbose
```

结果：

```text
Publisher count: 1
Node name: baseline_velocity_command
Subscription count: 1
Node name: arm_velocity_controller
```

节点列表中没有 `student_joint_velocity_bridge`。Baseline CSV 中 `velocity_command` 始终为：

```text
0;0;0;0;0;0
```

### 阶段 8.0 结论

- Base 轨迹连续、带限并可重复；
- 同 seed 随机轨迹 CSV 完全一致；
- `fix_base_to_world:=false` 下 Gazebo 模型整体移动；
- baseline 中关节保持初始构型，最大关节步进为 `0 rad`；
- `/arm_velocity_controller/commands` 只有 baseline 零速度节点一个发布者；
- GT 只写入 CSV，不进入控制器；
- `sine_x`、`sine_y`、`sine_z` 和 `random_translation_3d` 均完成 30 秒运行。

### 阶段 8.0 参数调整

为便于在 Gazebo GUI 中直接观察 Base 扰动带来的整机移动，`base_disturbance_profiles.yaml` 默认扰动参数调大：

```text
translation_amplitude: x=0.10 m, y=0.10 m, z=0.06 m
sine_frequency_hz: 0.35
frequency_band: 0.10-1.0 Hz
```

该调整只改变轨迹生成默认参数，不改变 replay、baseline、controller 或 Ground Truth 隔离逻辑。
