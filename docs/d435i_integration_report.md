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
