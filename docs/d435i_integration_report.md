# D435i Gazebo Classic Integration Report - Phase 0

## Scope

Phase 0 records the existing repository and runtime baseline, then adds a
minimal Gazebo Classic launch entry that displays the current six-joint arm in
an empty Gazebo world. This phase does not add D435i files, Gazebo sensors,
AprilTags, SLAM, visual servoing, or any changes to IK, dynamics, or low-level
control.

## Environment

- Command workspace:
  `/home/yihuang/westlake/windylab-arm-for6/windylab_ws`
- Gazebo Classic: `11.10.2`
- ROS version: `2`
- ROS distribution: `humble`
- Kernel: `Linux JIAOLONG-Series 6.8.0-124-generic #124~22.04.1-Ubuntu SMP PREEMPT_DYNAMIC Tue May 26 21:05:19 UTC x86_64`
- Build system: ROS 2 `ament_cmake` packages built with `colcon build`
- Workspace packages:
  - `manipulator` from `src/arm-platform`
  - `dummy_interface` from `src/dummy-interface`
  - `dummy_description` from `src/dummy_description`
  - `serial` from `src/serial`

## Gazebo ROS Packages And Plugins

Installed ROS packages found before this phase:

- `gazebo_dev`
- `gazebo_msgs`
- `gazebo_ros`

`gazebo_ros` provides the core Gazebo ROS libraries needed for the Phase 0
model spawn baseline, including:

- `/opt/ros/humble/lib/libgazebo_ros_factory.so`
- `/opt/ros/humble/lib/libgazebo_ros_init.so`
- `/opt/ros/humble/lib/libgazebo_ros_force_system.so`

The generic Gazebo ROS sensor plugins needed by later D435i phases were not
installed in the current non-root session. The required install command is:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

The following check produced no camera/depth/IMU plugin paths before package
installation:

```bash
source setup_env.bash
find /opt/ros/"$ROS_DISTRO" \
  \( -name 'libgazebo_ros*camera*.so' \
  -o -name 'libgazebo_ros*openni*.so' \
  -o -name 'libgazebo_ros*depth*.so' \
  -o -name 'libgazebo_ros*imu*.so' \) \
  2>/dev/null | sort
```

Attempting to install from this Codex session was blocked because `sudo`
requires an interactive password. No system package changes were made by Codex.

## Current Robot Description

- Main active robot description source:
  `src/arm-platform/config/arm.urdf`
- Installed description used by launches:
  `share/manipulator/arm.urdf`
- `robot_description` generation entry:
  `src/arm-platform/launch/student_arm.launch.py`
- Current description type: plain URDF file read directly into the
  `robot_description` parameter
- Base link: `base_link`
- Sixth joint/end-effector link used by current code: `link6`
- No active `tool0`, flange, or explicit end-effector fixed link is present in
  the active `arm.urdf`
- Current TF chain:
  `base_link -> link1 -> link2 -> link3 -> link4 -> link5 -> link6`
- No active `world -> base_link` fixed joint is present in the active URDF
- The root link `base_link` has inertial data, which triggers the standard KDL
  warning in `robot_state_publisher`

There are separate effector xacro files under `dummy_description` that define
`tool0`, but they are not part of the current `student_arm.launch.py`
`robot_description` path.

## Control And Model Loading

- Current student simulation mode is a custom virtual arm node, not Gazebo
  physics control.
- `/joint_states` is published by `student_arm_node`.
- TF is published by `robot_state_publisher`.
- Student command input:
  `/student/joint_command` with `sensor_msgs/msg/JointState`
- Optional feedback:
  `/student/joint_feedback` with `dummy_interface/msg/MotorState`
- Controller type in student mode: custom `SmoothPositionController`
- No `ros2_control`, `ros_control`, `gazebo_ros_control`, transmission, or
  controller YAML chain is currently active.
- IK module:
  `src/arm-platform/demo/pinocchio_ik_6dof.py`
- IK URDF input:
  `src/arm-platform/config/arm.urdf`
- IK end-effector frame:
  `link6`
- Gravity control and collision checking also load URDF models through
  Pinocchio from configured URDF paths.

## Baseline Verification

Build command:

```bash
source setup_env.bash
colcon build
```

Result: all four packages finished. Existing CMake cache warnings were observed
for `dummy_description`, `dummy_interface`, and `serial` because their cache was
created under the old path
`/home/yihuang/westlake/windylab-arm/windylab_ws`.

Existing student simulation launch:

```bash
source setup_env.bash
ros2 launch manipulator student_arm.launch.py use_rviz:=False
```

Observed topics:

- `/joint_states`
- `/parameter_events`
- `/robot_description`
- `/rosout`
- `/student/joint_command`
- `/student/joint_feedback`
- `/tf`
- `/tf_static`

Motion command used for baseline verification:

```bash
ros2 topic pub --once /student/joint_command sensor_msgs/msg/JointState \
  "{position: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

`/joint_states` and `tf2_echo base_link link6` reflected the commanded joint1
motion.

IK self-test:

```bash
cd src/arm-platform/demo
python3 pinocchio_ik_6dof.py
```

Result: the script loaded a six-DoF model with end frame `link6` and converged
on 8 of 10 random FK/IK round-trip targets. The non-converged samples were
random difficult or unreachable targets and match the current demo behavior.

## Gazebo Baseline Added In Phase 0

New launch file:

```text
src/arm-platform/launch/gazebo_arm.launch.py
```

Usage:

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py
```

Headless/server-only verification:

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py gui:=false
```

GUI verification:

```bash
source setup_env.bash
ros2 launch manipulator gazebo_arm.launch.py gui:=true
```

The launch starts Gazebo Classic through `gazebo_ros`, publishes the existing
`arm.urdf` as `robot_description`, and spawns a Gazebo entity named
`windylab_arm` from that topic. This baseline is for visual/model spawn
verification only. It does not add Gazebo joint control.

The launch rewrites `dummy_description` mesh URIs in the spawned Gazebo
description to `file://` paths, so Gazebo does not query the online model
database for project meshes. It sets `GAZEBO_MODEL_PATH` to include Gazebo
Classic's local `sun` and `ground_plane` models, and disables the Gazebo Classic
online model database by setting `GAZEBO_MODEL_DATABASE_URI` to an empty value.
The spawned model is marked static for this visual baseline, avoiding unstable
free-body physics before a Gazebo joint-control stack exists.

The launch strips the leading XML encoding declaration from the published URDF
string before handing it to `spawn_entity.py`. This works around the ROS 2
Humble `spawn_entity.py`/lxml behavior where Unicode strings with an XML
encoding declaration are rejected. The source `arm.urdf` file is not modified.

Headless verification completed successfully after this workaround. Gazebo
reported:

```text
Spawn status: SpawnEntity: Successfully spawned entity [windylab_arm]
```

The default GUI launch also started both `gzserver` and `gzclient`, then spawned
the same `windylab_arm` entity successfully.

When the verification command was stopped by `timeout`, `gzserver` required
SIGKILL after SIGINT/SIGTERM. This was observed during shutdown only, after the
entity had already spawned successfully.

## Git State

The workspace root is not a Git repository. The actual Git repositories are:

- `src/arm-platform`
- `src/dummy_description`
- `src/dummy-interface`
- `src/serial`

Before Phase 0 edits, `src/arm-platform` already contained many uncommitted
changes from earlier work, including the student-arm simulation entry and 6-DoF
demo files. This phase only adds the Gazebo baseline launch, this report, and
the minimal package metadata needed to install them.

## Planned Files For Later Phases

Likely later changes should stay in the active `manipulator` package unless a
dedicated Gazebo package is introduced:

- D435i wrapper xacro under the active robot description path
- Gazebo sensor xacro for RGB-D and IMU plugin tags
- Optional D435i simulation config
- AprilTag test world/model assets
- Sensor verification scripts

Before Phase 1 sensor work, install `ros-humble-gazebo-plugins` and re-run the
plugin path check above. The D435i integration should connect to the active
`robot_description` chain instead of creating a parallel display-only robot.

## Phase 1 D435i Description Baseline

Phase 1 uses the ROS package `realsense2_description` from
`ros-humble-realsense2-description`. Official RealSense files are not copied
into this repository and are not modified. The project-side wrapper includes
the official `_d435i.urdf.xacro` macro and mounts the camera on the active
end-effector link `link6`.

Installed dependencies for this phase:

```bash
sudo apt-get install -y ros-humble-xacro ros-humble-realsense2-description
```

New project description files:

- `config/sensors/d435i_mount.xacro`
- `config/arm_with_d435i.urdf.xacro`

Default mount parameters:

- `camera_name:=camera`
- `camera_parent_link:=link6`
- `camera_xyz:="0.06 0 0.04"`
- `camera_rpy:="0 0 0"`
- `camera_use_nominal_extrinsics:=true`

Static validation command:

```bash
source setup_env.bash
xacro $(ros2 pkg prefix manipulator)/share/manipulator/arm_with_d435i.urdf.xacro \
  camera_enabled:=true > /tmp/robot_with_d435i.urdf
check_urdf /tmp/robot_with_d435i.urdf
```

Result:

- `check_urdf` parsed successfully.
- Expanded model contains 21 links and 20 joints.
- No duplicate link or joint names were found.
- The generated TF chain includes:
  `link6 -> camera_bottom_screw_frame -> camera_link -> camera_depth_frame -> camera_depth_optical_frame`.
- Nominal color, infrared, accel, and gyro frames are also generated.

Runtime validation:

- `ros2 launch manipulator student_arm.launch.py use_rviz:=False camera_enabled:=false`
  still starts the original six-joint arm description.
- `ros2 launch manipulator student_arm.launch.py use_rviz:=False camera_enabled:=true`
  starts with the D435i frames in `robot_state_publisher`.
- Publishing one joint command to `/student/joint_command` moved joint1 to
  `0.3 rad`; `base_link -> camera_depth_optical_frame` changed consistently,
  while `link6 -> camera_depth_optical_frame` stayed fixed.
- `ros2 launch manipulator gazebo_arm.launch.py gui:=true camera_enabled:=true`
  spawned `windylab_arm` successfully and displayed the D435i mesh at the arm
  end in Gazebo Classic.

Evidence screenshots:

- RViz: `/tmp/d435i_phase1_rviz.png`
- Gazebo window: `/tmp/d435i_phase1_gazebo_window.png`

Regression:

- `colcon build` completed for all four workspace packages.
- Existing CMake cache path warnings remain for `dummy_description`,
  `dummy_interface`, and `serial`.
- `python3 src/arm-platform/demo/pinocchio_ik_6dof.py` still reports
  convergence on 8 of 10 random FK/IK targets, matching the previous baseline.
