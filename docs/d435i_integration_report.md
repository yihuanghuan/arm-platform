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

### 后续任务清单

- 增加独立的“仿真—真机接口对齐”阶段，在接入真实 D435i 前统一 topic、namespace、`frame_id`、TF、CameraInfo、depth alignment、点云参考坐标系、时间戳和数据同步策略。
- 为 AprilTag 彩色图像链路明确使用 `camera_color_optical_frame`，避免检测结果长期绑定到当前临时 depth frame 配置。
- 若需要根据彩色图像像素读取深度，加入 depth-to-color alignment 或基于内参/外参的重投影模块。
- 将上层视觉节点的 topic、frame、分辨率、FOV 和同步策略全部参数化，不依赖阶段 2 的临时 Gazebo topic/frame 默认值。
- 在真实 D435i 上单独验收精度、噪声、延迟、曝光、深度空洞和 color/depth 对齐性能；阶段 2 的仿真结果不能替代真机标定与数据质量验证。

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
  -> camera_parent_link:=link1
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
- `base` 模式 TF 链为 `world -> base_link -> link1 -> camera_bottom_screw_frame -> camera_link -> camera_depth_optical_frame`，相机固定在 base 关节转动 link 上；
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
link1 -> camera_depth_optical_frame
before/after joint1 command:
translation [0.031, 0.018, 0.072], rotation [-90 deg, 0 deg, -90 deg]

base_link -> camera_depth_optical_frame
before: translation [0.092, 0.018, 0.072], RPY [-90 deg, 0 deg, -90 deg]
after:  translation [0.092, -0.005, 0.074], RPY [-90 deg, 17.189 deg, -90 deg]
```

说明：相机相对 `link1` 保持固定，会随 base 关节 joint1 运动；安装高度从上一版约 `0.37 m` 降到约 `0.07 m`。

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
link1 -> camera_depth_optical_frame
translation [0.031, 0.018, 0.072]
```

两种模式中 `robot_state_publisher` 均成功加载 `camera_depth_optical_frame`。

### 阶段 4 结论

- 只通过 launch 参数即可在 `camera_mount_mode:=ee` 与 `camera_mount_mode:=base` 间切换；
- 两种模式复用同一套 D435i xacro、RGB-D sensor 和 IMU sensor；
- RGB-D/IMU topic 命名保持一致；
- 末端模式相机随 `link6` 刚性运动；
- Base 模式相机相对 `link1` 固定，安装在 base 关节附近并随 joint1 运动；
- 原有 Gazebo 控制链路和学生启动入口保持可用。
