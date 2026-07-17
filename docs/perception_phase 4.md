# Perception Phase 4 分阶段执行与验收记录

## 执行规则

本文件只记录 `perception_control.md` 中“阶段 4：连续 Base 平移扰动下的 XYZ 稳定”的修复与验收过程。

执行时遵守以下硬门禁：

1. 当前子阶段未达到全部通过条件时，不修改下一子阶段的参数，也不进入下一子阶段。
2. 每个子阶段记录目的、修改内容、验收终端命令和实际效果。
3. 每个子阶段通过后，必须提交并推送到 `origin/develop`，确认 `HEAD` 与 `origin/develop` 一致后才进入下一子阶段。
4. 单个子阶段若经过重复定位仍无法满足门禁，则停止工作并报告，不用后续参数掩盖问题。

这里的“子阶段 0 通过”只表示已经稳定、可重复地复现和量化当前失败，不表示原阶段 4 任务已经通过。

## 总体计划与当前状态

| 子阶段 | 单一目标 | 进入下一阶段的硬门禁 | 状态 |
|---|---|---|---|
| 0 | 冻结当前失败基线和判废规则 | 15 次固定矩阵数据完整；失败链路重复 3 次；异常初态不输出正式成绩 | 已通过 |
| 1 | 修正 Gazebo Base plant | 已知小关节脉冲下 Base 不发生非指令位移；3 次结果通过阈值 | 已通过 |
| 2 | 统一仿真时间和回放时序 | 回放、感知、控制都使用同一仿真时间；暂停/恢复不破坏轨迹 | 已通过 |
| 3 | 验证 Pinocchio 与 Gazebo 运动学一致 | 多构型、多轴小脉冲的方向、尺度和 frame 一致 | 已通过 |
| 4 | 仅用 Gazebo GT 完成动态 XYZ 闭环 | GT 相比 baseline 的 RMS 明显下降，且不发散、可重复 | 已通过 |
| 5 | 修正安全状态机 | safety 可区分等待、可恢复丢帧和故障锁存；命令行为逐项通过 | 未开始，锁定 |
| 6 | 验收视觉开环链路 | 图像、Tag TF、世界系 EE 估计的频率、延迟、方向达到门槛 | 未开始，锁定 |
| 7 | 保证相机视场和姿态可观测性 | 全扰动范围内 Tag 可见率和相机姿态满足门槛 | 未开始，锁定 |
| 8 | visual dry-run 对照 GT | 同时刻视觉/GT 误差及 CLIK 命令方向、尺度一致 | 未开始，锁定 |
| 9 | 静态 Base 视觉闭环 | 多个小偏差均收敛，无 Base 漂移、无 safety 误触发 | 未开始，锁定 |
| 10 | 逐级动态扰动 | 按幅值和频率逐级通过，不跨级调参 | 未开始，锁定 |
| 11 | 完整阶段 4 验收 | `sine_x/y/z/xyz` 和随机平移均优于 baseline，且多次可复现 | 未开始，锁定 |

---

## 子阶段 0：冻结失败基线

### 状态

通过。完成日期：2026-07-17。

阶段 4 本身仍为不通过，不能进入原任务的 `sine_y` 或更高难度扰动。

### 目的

本子阶段不调整 Base、时钟、CLIK、视觉或 safety 参数，只完成四件事：

1. 固定仿真配置和轨迹，使失败可以重复。
2. 建立 baseline、visual 和 ground-truth 隔离对照。
3. 证明大幅摆动发生在外部 Base 扰动开始之前还是之后。
4. 拒绝“采样开始时系统已经失稳”的运行，避免把约 4–6 m 的异常初态计算成正常跟踪成绩。

### 冻结配置

| 项目 | 固定值 |
|---|---|
| World | `d435i_apriltag_board_2x2.world` |
| 初始关节 | `[0, 0, 0, 0, 0, 0] rad` |
| 相机 | EE 默认外参，RGB-D `640x480 @ 15 Hz` |
| IMU | 开启，`200 Hz` |
| 轨迹采样 | `100 Hz`，30 s，3001 样本 |
| 随机种子 | 42 |
| `sine_x` | 幅值 0.10 m，频率 0.35 Hz |
| replay 启动等待 | 12.0 s |
| Base 描述 | 保持现状：`fix_base_to_world:=false` |
| 控制参数 | 全部保持 launch 默认值 |

冻结轨迹 SHA-256：

```text
static.csv  970a0c56504d52e398b206a42635a2bc906a26a064de0865491441ea00d1ee48
sine_x.csv  10add165bf9dbf6b44081b5ec862e41587b0c8e4e79adc55986f60ac8fd22b94
```

### 修改内容

本子阶段只增加观测和判废能力，没有修改控制算法或参数。

- `scripts/phase4_dynamic_xyz_metrics.py`
  - 新增 `run_valid`、`invalid_reason`。
  - 检查首个 replay 样本的 Base 位置跟踪误差、姿态跟踪误差和初始关节偏差。
  - 默认判废阈值分别为 `0.02 m`、`0.10 rad`、`0.05 rad`。
  - 无效运行的正式 `xyz_rms_m`、`xyz_max_m` 和稳态指标输出为空；原始计算只放入 `diagnostic_*` 列。
  - 新增 `--expected-initial-joints` 以明确固定初始构型。
- `scripts/phase4_failure_sequence_metrics.py`
  - 只读分析 static-visual rosbag。
  - 记录 target lock、首次非零关节命令、Base 超过 1 cm/1 m、safety stop 的先后时刻。
  - 记录最大 Base 位移、最大关节偏差、最大命令和 safety 后非零命令比例。
  - `--require-sequence` 可直接作为失败链路的终端门禁。
- `CMakeLists.txt`
  - 安装新增的失败链路分析脚本。

### 实验矩阵

每组独立启动 Gazebo，结束后清理完整 launch 进程组；每组重复 3 次：

| 组别 | 轨迹 | 控制输入 | 重复数 |
|---|---|---|---:|
| static-baseline | static | 六关节零速度 | 3 |
| static-visual | static | AprilTag + visual XYZ CLIK | 3 |
| sine-baseline | sine_x | 六关节零速度 | 3 |
| sine-visual | sine_x | AprilTag + visual XYZ CLIK | 3 |
| sine-ground-truth | sine_x | Gazebo link6 GT + XYZ CLIK | 3 |

static-visual 三次额外录制：

```text
/model_states
/link_states
/joint_states
/arm_velocity_controller/commands
/visual_stabilization/status
/visual_stabilization/error
/visual_ee_pose
/visual_ee_pose_valid
/clock
```

### 验收终端命令

构建：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
source install/setup.bash
```

生成冻结轨迹：

```bash
mkdir -p /tmp/windylab_phase4_stage0

ros2 run manipulator generate_base_disturbance.py \
  --config src/arm-platform/config/base_disturbance_profiles.yaml \
  --profile static --duration-sec 30 --sample-rate-hz 100 --random-seed 42 \
  --output-csv /tmp/windylab_phase4_stage0/static.csv

ros2 run manipulator generate_base_disturbance.py \
  --config src/arm-platform/config/base_disturbance_profiles.yaml \
  --profile sine_x --duration-sec 30 --sample-rate-hz 100 --random-seed 42 \
  --output-csv /tmp/windylab_phase4_stage0/sine_x.csv

sha256sum /tmp/windylab_phase4_stage0/static.csv \
  /tmp/windylab_phase4_stage0/sine_x.csv
wc -l /tmp/windylab_phase4_stage0/static.csv \
  /tmp/windylab_phase4_stage0/sine_x.csv
```

单次 launch 模板；分别替换 `experiment_mode`、轨迹、输出名并运行 `r1/r2/r3`：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  experiment_mode:=baseline gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase4_stage0/sine_x.csv \
  replay_output_csv:=/tmp/windylab_phase4_stage0/sine_baseline_r1_replay.csv \
  start_delay_sec:=12.0 \
  rgbd_width:=640 rgbd_height:=480 rgbd_update_rate:=15 \
  imu_enabled:=true imu_update_rate:=200
```

visual 运行增加以下参数：

```bash
experiment_mode:=visual_xyz \
run_phase4_diagnostics:=true \
phase4_diagnostics_output_csv:=/tmp/windylab_phase4_stage0/sine_visual_r1_visual_chain.csv \
phase4_diagnostics_sample_hz:=5.0
```

汇总指标；其他组使用相同格式：

```bash
ros2 run manipulator phase4_dynamic_xyz_metrics.py \
  --run baseline_r1=/tmp/windylab_phase4_stage0/sine_baseline_r1_replay.csv \
  --run baseline_r2=/tmp/windylab_phase4_stage0/sine_baseline_r2_replay.csv \
  --run baseline_r3=/tmp/windylab_phase4_stage0/sine_baseline_r3_replay.csv \
  --run visual_r1=/tmp/windylab_phase4_stage0/sine_visual_r1_replay.csv \
  --run visual_r2=/tmp/windylab_phase4_stage0/sine_visual_r2_replay.csv \
  --run visual_r3=/tmp/windylab_phase4_stage0/sine_visual_r3_replay.csv \
  --run gt_r1=/tmp/windylab_phase4_stage0/sine_ground_truth_r1_replay.csv \
  --run gt_r2=/tmp/windylab_phase4_stage0/sine_ground_truth_r2_replay.csv \
  --run gt_r3=/tmp/windylab_phase4_stage0/sine_ground_truth_r3_replay.csv \
  --output-csv /tmp/windylab_phase4_stage0/dynamic_summary.csv
```

验收 static-visual 因果顺序：

```bash
ros2 run manipulator phase4_failure_sequence_metrics.py \
  --bag r1=/tmp/windylab_phase4_stage0/static_visual_r1_bag \
  --bag r2=/tmp/windylab_phase4_stage0/static_visual_r2_bag \
  --bag r3=/tmp/windylab_phase4_stage0/static_visual_r3_bag \
  --output-csv /tmp/windylab_phase4_stage0/static_visual_failure_sequence_summary.csv \
  --require-sequence
```

判废规则离线回归：

```bash
python3 src/arm-platform/scripts/phase4_dynamic_xyz_metrics.py \
  --run baseline=/tmp/windylab_phase4_stage0/sine_baseline_r1_replay.csv \
  --run invalid_visual=/tmp/windylab_phase4_stage0/sine_visual_r1_replay.csv \
  --output-csv /tmp/windylab_phase4_stage0/initial_state_rejection.csv
```

预期：baseline 为 `run_valid=true` 且有正式 XYZ 指标；visual 为 `run_valid=false`，正式 XYZ 指标为空，`diagnostic_xyz_*` 保留。

### 硬门禁与结果

| 门禁 | 通过条件 | 实际结果 | 判定 |
|---|---|---|---|
| 数据完整性 | 15 次均为 3001 样本，`set_failures=0` | 15/15 满足 | 通过 |
| 正常对照初态 | static/sine baseline 和 sine GT 初态有效 | 9/9 有效 | 通过 |
| 异常初态判废 | 6 次 visual 失稳初态不得输出正式成绩 | 6/6 `run_valid=false`，正式 XYZ 为空 | 通过 |
| 扰动可重复性 | sine-baseline RMS 三次跨度小于 `1e-4 m` | 跨度 `9.85e-6 m` | 通过 |
| 失败可重复性 | static-visual 三次均出现“非零命令后 Base 超 1 cm、再超 1 m”，关节偏差大于 0.1 rad | 3/3 满足 | 通过 |
| 根因隔离 | GT 组完整且可与 baseline 比较 | 3/3 完整 | 通过 |

### 核心数据

| 组别 | 正式有效性 | XYZ RMS | XYZ 最大值 | 其他关键结果 |
|---|---|---:|---:|---|
| static-baseline | 3/3 有效 | `0 m` | `0 m` | 关节步进 `0 rad` |
| static-visual | 0/3，有效性判废 | 正式值为空；诊断 `4.330–5.608 m` | 不作为成绩 | replay 初始 Base 误差 `4.342–5.218 m`，初始关节偏差 `0.919–1.073 rad` |
| sine-baseline | 3/3 有效 | `0.067627–0.067637 m` | `0.100013–0.100019 m` | 三次高度可重复 |
| sine-visual | 0/3，有效性判废 | 正式值为空；诊断 `4.415–6.346 m` | 不作为成绩 | replay 初始 Base 误差 `4.428–6.190 m`，初始关节偏差 `0.872–1.080 rad` |
| sine-ground-truth | 3/3 有效 | `0.077214–0.077354 m` | `0.125667–0.129546 m` | 关节速度峰值约 `0.1797 rad/s` |

sine-baseline 三次平均 RMS 为 `0.0676339 m`；sine-ground-truth 三次平均为 `0.0772935 m`，GT 控制相对 baseline **恶化 14.282%**。因此问题不只在视觉链路，当前 CLIK 与 Gazebo plant 组合本身尚未形成有效抗扰。

static-visual rosbag 的因果时序：

| 运行 | target lock | 首次非零命令 | Base > 1 cm | Base > 1 m | safety stop | 最大 Base 位移 | 最大关节偏差 |
|---|---:|---:|---:|---:|---:|---:|---:|
| r1 | 1.107 s | 3.405 s | 3.604 s | 4.505 s | 4.706 s | 57.147 m | 0.968 rad |
| r2 | 2.283 s | 4.382 s | 4.582 s | 5.483 s | 12.785 s | 59.351 m | 0.965 rad |
| r3 | 4.492 s | 4.691 s | 4.890 s | 5.791 s | 未触发 | 103.546 m | 1.074 rad |

三次均在 static 轨迹、外部扰动尚未开始时发生相同的前半段顺序：

```text
视觉 target 锁定
→ 小幅非零关节速度命令
→ 约 0.20 s 后 Base 非指令位移超过 1 cm
→ 约 0.90 s 后超过 1 m
→ 整机与关节大幅摆动
```

r1/r2 在误差越过 `0.20 m` 后 safety 锁存，之后非零命令比例为 0；r3 在视觉丢失前的最大 status 误差只有 `0.173 m`，未越过阈值，因此没有触发 safety，最大命令达到 `0.198 rad/s`。这说明 safety 结果依赖视觉何时丢失，保护行为本身不确定；该问题保留到子阶段 5，当前不通过调阈值掩盖。

六次 visual 运行中：

- RGB 图像约 `14.37–15.00 Hz`；
- 世界系视觉 EE pose 仅约 `3.43–3.93 Hz`；
- replay 内视觉丢失率为 `23.3%–58.1%`；
- 一旦整机翻转或远离 AprilTag，检测率和 Tag TF 更新率会降到 0。

### 当前阶段 4 完成情况

依据 `perception_control.md` 的验收要求，当前原阶段 4 **未完成**：

- visual closed loop 六次运行全部在正式扰动采样前失稳，无法形成有效的 baseline 对比；
- 没有“XYZ RMS 明显下降”，visual 正式 RMS 必须判空；
- 存在明确控制发散和关节大幅运动；
- 只有失败现象可重复，稳定控制结果不可用；
- GT 隔离组反而比 baseline 恶化 14.282%。

因此目前不能宣称末端产生了有效抗扰意图。

### 为什么会整机大幅摆动且末端没有可见抗扰

1. **首要问题是 plant 与控制模型不一致。** `moving_base_stabilization.launch.py` 使用 `fix_base_to_world:=false`，Gazebo 中 Base 是自由刚体；当前 Pinocchio 模型和 CLIK Jacobian 则按固定根机械臂计算。关节速度执行后的反作用会移动自由 Base，但控制器把 Base 当作不动的世界参考。Base 一旦转动，Jacobian、相机方向和世界系误差之间继续失配，形成快速放大的错误闭环。
2. **控制在 replay 等待期已经启用。** 视觉 target 锁定后，毫米级估计变化超过 `0.003 m` 死区便产生关节命令。rosbag 已直接证明首次非零命令先于 Base 的非指令运动约 0.20 s；因此 `sine_x` 不是最初的发散触发器。
3. **safety 只能在发散后锁零，不能恢复机械状态。** r1/r2 在 Base 已经移动到米级后才锁存零命令；此时模型和关节已经偏离初态。r3 又因视觉先丢失而未达到大误差阈值，继续输出非零命令。两种结果在 Gazebo 中都不会呈现正常的末端抗扰。
4. **视觉更新和可见性不足以支撑当前闭环。** 15 Hz 图像只形成约 3.4–3.9 Hz 的视觉 EE 更新；大姿态变化后 AprilTag 丢失。控制任务只约束 XYZ，没有相机朝向或视场约束，无法主动保持目标可见。
5. **现有任务速度上限低于扰动所需速度。** 冻结正弦的 Base 峰值速度为 `2*pi*0.35*0.10 = 0.2199 m/s`，而默认 XYZ 任务速度上限为 `0.05 m/s`，相差约 4.4 倍。即使先修复发散，当前限幅也不可能理想抵消该轨迹；但该参数只能在 plant、时序和 GT 闭环依次通过后单独调整。
6. **GT 对照排除了“只修视觉就能完成”的可能。** GT 输入没有视觉丢帧，却仍比零控制 baseline 更差，说明必须先修复 Base plant、时序和运动学一致性，再回到视觉闭环。

### 达到的效果

- 当前失败已从“Gazebo 看起来乱晃”转化为可重复、带时序和量化阈值的故障链路。
- 正常 baseline 与异常 visual 运行不会再混在同一成绩口径中。
- 已确认 Base 回放服务不是主要问题：15 次均 `set_failures=0`，零控制正弦轨迹三次高度可重复。
- 已确认问题不只来自 AprilTag：GT 控制也未改善 RMS。
- 子阶段 1 的工作边界已经收敛为 Gazebo Base plant，禁止先调视觉增益、速度限幅或 safety 阈值。

### 版本控制验收

本子阶段提交说明固定为：

```text
phase4: freeze failure baseline and reject invalid runs
```

提交和推送后执行：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws/src/arm-platform
git status --short --branch
git log -1 --oneline
git rev-parse HEAD
git rev-parse origin/develop
```

只有工作区干净且最后两个提交 ID 一致，才允许开始子阶段 1。

---

## 子阶段 1：修正 Gazebo Base plant

### 状态

通过。完成日期：2026-07-17。

这里的通过只表示 Base 已成为由轨迹明确规定的运动边界，不再被关节执行器反作用自由推动。视觉闭环和原阶段 4 仍未通过。

### 目的

消除子阶段 0 已确认的首要问题：`fix_base_to_world:=false` 时，replay 开始前没有节点约束自由 Base，视觉控制器产生小关节命令后，关节反作用可在约 0.2 s 内推动 Base，随后形成整机发散。

本子阶段只改变 Base plant 的规定方式，不修改 CLIK、视觉估计、速度限制、增益、deadband 或 safety 参数。

### 候选方案探针

先用相同的 `d435i_apriltag_board_2x2.world` 比较两个候选，避免 world 插件差异干扰结论。

| 候选 | 探针结果 | 决策 |
|---|---|---|
| `fix_base_to_world:=true` 机械固定根 | `/set_entity_state(x=0.1)` 返回成功，但 2 s 后模型仍在约 0 m；固定关节立即恢复原点 | 否决：不能执行规定 Base 平移 |
| 自由 Base + 100 Hz 持续规定模型 pose/twist | `joint2` 运动 0.006920 rad 时，模型与 `base_link` 最大位移均为 `1.98e-5 m`，EE 正常移动 0.002824 m | 采用 |

采用方案仍保留 `fix_base_to_world:=false`，但 replay 节点在服务和实体就绪后，不再空等 `start_delay_sec`；它在整个等待期以 replay 频率持续写入轨迹第 0 行的 pose/twist。这样 Base 是可移动的规定输入，而不是会被机械臂执行器反作用推动的自由状态。

### 修改内容

- `scripts/base_disturbance_replay.py`
  - 新增 `--hold-initial-state-during-start-delay`，默认 `true`。
  - 在 replay 等待期按 `rate_hz` 持续调用 `/set_entity_state` 写入第 0 个轨迹状态。
  - replay CSV 和终端摘要新增：
    - `pre_roll_hold_enabled`
    - `pre_roll_hold_attempts`
    - `pre_roll_hold_failures`
    - `pre_roll_hold_rate_hz`
  - 显式拒绝负数 `start_delay_sec`。
- `launch/moving_base_stabilization.launch.py`
  - 新增 `hold_initial_state_during_start_delay` launch 参数，默认开启。
  - 新增 `experiment_mode:=plant_test`；该模式不启动 baseline、visual 或 GT 命令发布者，用于单独注入已知关节脉冲。
- `scripts/phase4_joint_pulse_check.py`
  - CSV 新增 Base 位置和四元数。
  - 自动汇总 Base 最大位移、最大姿态误差和被测关节实际运动量。
  - 新增 `--require-base-stable` 终端硬门禁。
- `scripts/phase4_dynamic_xyz_metrics.py`
  - 汇总 pre-roll hold 状态、次数、失败数和实际频率。
  - hold 存在调用失败时，将该运行标记为无效。

### 验收终端命令

构建和语法检查：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws/src/arm-platform
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  scripts/base_disturbance_replay.py \
  scripts/phase4_dynamic_xyz_metrics.py \
  scripts/phase4_joint_pulse_check.py \
  launch/moving_base_stabilization.launch.py
git diff --check

cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
source install/setup.bash
```

Base plant 脉冲验收。先启动无命令发布者的 plant test：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  experiment_mode:=plant_test gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase4_stage0/static.csv \
  replay_output_csv:=/tmp/windylab_phase4_stage1_pulse_r1_replay.csv \
  start_delay_sec:=20.0 \
  hold_initial_state_during_start_delay:=true \
  rgbd_width:=320 rgbd_height:=240 imu_enabled:=false
```

看到日志 `Holding the initial commanded base state` 后，在另一个终端执行：

```bash
ros2 run manipulator phase4_joint_pulse_check.py \
  --output-csv /tmp/windylab_phase4_stage1_pulse_r1_pulse.csv \
  --joint-name joint2 \
  --velocity 0.02 \
  --pulse-duration-sec 0.5 \
  --pre-sec 0.5 \
  --duration-sec 3.0 \
  --sample-hz 50 \
  --max-base-displacement-m 0.001 \
  --max-base-orientation-error-rad 0.01 \
  --min-joint-motion-rad 0.005 \
  --require-base-stable \
  --use-sim-time
```

独立重启 Gazebo 并重复 `r1/r2/r3`。每次必须输出 `base_stable_gate: true`。

规定 Base 仍可移动的验收轨迹：

```bash
ros2 run manipulator generate_base_disturbance.py \
  --config src/arm-platform/config/base_disturbance_profiles.yaml \
  --profile sine_x \
  --duration-sec 5 \
  --sample-rate-hz 100 \
  --random-seed 42 \
  --output-csv /tmp/windylab_phase4_stage1_sine_x_5s.csv
```

每次独立运行：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  experiment_mode:=baseline gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase4_stage1_sine_x_5s.csv \
  replay_output_csv:=/tmp/windylab_phase4_stage1_sine_r1_replay.csv \
  start_delay_sec:=5.0 \
  hold_initial_state_during_start_delay:=true \
  rgbd_width:=320 rgbd_height:=240 imu_enabled:=false
```

汇总三次结果：

```bash
ros2 run manipulator phase4_dynamic_xyz_metrics.py \
  --run sine_r1=/tmp/windylab_phase4_stage1_sine_r1_replay.csv \
  --run sine_r2=/tmp/windylab_phase4_stage1_sine_r2_replay.csv \
  --run sine_r3=/tmp/windylab_phase4_stage1_sine_r3_replay.csv \
  --output-csv /tmp/windylab_phase4_stage1_sine_summary.csv
```

### 硬门禁与结果

| 门禁 | 通过条件 | 三次实际结果 | 判定 |
|---|---|---|---|
| 已知脉冲确实执行 | `joint2` 实际运动 ≥0.005 rad | 三次均为 `0.006938 rad` | 通过 |
| Base 非指令平移 | 最大位移 ≤0.001 m | `1.59e-5`、`4.42e-7`、`1.32e-6 m` | 通过 |
| Base 非指令旋转 | 最大误差 ≤0.01 rad | 最大 `1.79e-4 rad`，其余两次为 0 | 通过 |
| pre-roll 保持完整 | 5 s 内约 500 次、0 failure、频率 95–105 Hz | 均为 500 次、0 failure、`100.005 Hz` | 通过 |
| Base 仍可规定移动 | 5 s sine 实际 X 跨度 ≥0.19 m | `0.199682`、`0.199627`、`0.199893 m` | 通过 |
| 非扰动轴稳定 | Y/Z 跨度和姿态误差 ≤0.001 | 三次均为 0 | 通过 |
| replay 完整 | 每次 501 样本、0 set failure | 3/3 满足 | 通过 |

### 关于一次未通过的无效验收指标

第一次动态验收曾使用 `pose_tracking_error_max <= 0.001 m`，实际得到 `0.0274–0.0295 m`，所以该断言没有通过，未被静默忽略。

检查采样语义后确认：replay 以 100 Hz 发命令，但 `pose_tracking_error_m` 使用最新的 10 Hz `/link_states` 缓存与当前 100 Hz 命令直接相减。0.35 Hz、0.10 m 正弦的峰值速度为 0.2199 m/s，单个 0.1 s 状态采样间隔就可形成约 0.022 m 的表观滞后。因此该列当前不能作为毫米级 plant 跟踪门禁。

本阶段改用直接可观测的 Base 实际 X/Y/Z/姿态、set failure 和重复性完成验收。命令/状态时间戳对齐属于子阶段 2；在子阶段 2 通过前，不使用这列调 Base 或控制参数。

### static-visual 压力复测

正式门禁通过后，保持全部 visual/CLIK/safety 默认参数，额外运行一次 12 s pre-roll + 30 s static replay：

| 指标 | 子阶段 0 | 子阶段 1 压力复测 |
|---|---:|---:|
| pre-roll hold | 无 | 1200 次，0 failure，100.003 Hz |
| replay 初始 Base 跟踪误差 | 4.34–5.22 m | 0 m |
| static 期间 EE 诊断 RMS | 4.33–5.61 m | 0.002713 m |
| 最大 EE 诊断偏差 | 米级 | 0.003098 m |
| 初始关节偏差 | 0.92–1.07 rad | 0.241 rad |

原来的 57–103 m 整机飞散已经消失，说明 Base plant 修复确实作用于原故障链路。但视觉控制器在 12 s 内仍累积了 0.241 rad 关节漂移，所以该压力运行仍被 `initial_joint_error_exceeded` 正确判废。

关节漂移没有通过调整增益、deadband 或阻尼处理；它作为运动学一致性和静态视觉闭环的待解决输入，保留到子阶段 3、8、9。

### 达到的效果

- Base 从“自由动力学状态”改为“可移动、由轨迹规定的边界输入”。
- 已知关节脉冲不再产生可见 Base 平移或旋转。
- `sine_x` 仍可达到约 0.20 m 的峰峰值，未被机械固定。
- 原始 static-visual 整机大幅飞散消失，世界系 EE 保持在毫米量级；剩余关节漂移被独立暴露。
- 没有修改任何视觉或 CLIK 参数，避免 Base plant 与控制调参耦合。

### 版本控制验收

本子阶段提交说明固定为：

```text
phase4: prescribe the base throughout replay pre-roll
```

提交、推送并执行：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws/src/arm-platform
git status --short --branch
git log -1 --oneline
git rev-parse HEAD
git rev-parse origin/develop
```

只有工作区干净且两个提交 ID 一致，才允许开始子阶段 2。

---

## 子阶段 2：统一仿真时间与回放时序

### 状态

通过。完成日期：2026-07-17。

### 目的

消除两套并行时间基准：修改前 Base replay 使用 `time.monotonic()` 以墙钟 100 Hz 调度，而视觉、GT、baseline 和 ROS 2 control 节点声明 `use_sim_time=true`。Gazebo 实测只以 10 Hz 发布 `/clock` 和 `/model_states`，因此 100 Hz replay、100 Hz 控制 timer 和 10 Hz 状态缓存并未处在同一离散时间轴上。

本子阶段只修改仿真 clock/state 发布和 replay 调度语义，不调整 Base plant、运动学、视觉或控制参数。

### 修改前基线

```text
/gazebo.publish_rate: 10.0 Hz
/clock:               9.994-9.999 Hz
/model_states:        9.995-9.998 Hz
Base replay:           wall monotonic clock, 100 Hz
visual/GT/baseline:    ROS simulation clock timers
```

这也解释了子阶段 1 中原 `pose_tracking_error_m` 的 2–3 cm 表观误差：当前 100 Hz 命令与最多滞后 0.1 s 的状态缓存直接相减。

### 修改内容

- `config/gazebo_ros_params.yaml`
  - 新增 `/gazebo.publish_rate: 100.0`，使 Gazebo `/clock` 以 100 Hz 发布。
- `worlds/d435i_apriltag_board_2x2.world`
  - `gazebo_ros_state.update_rate` 从 10 Hz 提升为 100 Hz，使 `/model_states` 和 `/link_states` 与控制频率一致。
- `launch/gazebo_arm.launch.py`
  - 新增 `gazebo_params_file` 参数，默认把项目的 Gazebo ROS 参数文件传给 `gzserver.launch.py`。
- `launch/moving_base_stabilization.launch.py`
  - 新增 `replay_clock_source`，默认 `sim`。
  - 阶段 4 replay 明确使用 ROS 仿真时钟调度。
- `scripts/base_disturbance_replay.py`
  - 新增 `--schedule-clock {sim,wall}`；独立脚本为兼容旧用法默认 `wall`，阶段 4 launch 显式传 `sim`。
  - sim 模式等待非零 `/clock` 后才开始 pre-roll。
  - pre-roll 和正式 replay 都按 ROS clock 调度；正式样本直接使用 CSV 的 `time_sec`，不再用墙钟 `index/rate` 推算。
  - 仿真暂停时 schedule clock 不前进，因此不发新轨迹样本；恢复后从同一仿真时间继续。
  - 增加时钟回退检测和 clock-ready 超时；服务健康超时继续使用墙钟，避免仿真暂停时故障检测也永久停止。
  - replay CSV 新增：
    - `sim_time_sec`
    - `schedule_clock`
    - `schedule_elapsed_sec`
    - `schedule_error_sec`
  - 终端摘要区分仿真实际频率和墙钟实际频率。
- `scripts/phase4_dynamic_xyz_metrics.py`
  - 新增 schedule error、sim/wall duration、sim/wall 最大样本间隔。
  - 新增 Base pose/orientation 跟踪 RMS 和最大误差。

### 验收终端命令

构建：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
source install/setup.bash
```

时钟和状态发布率检查。先启动：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  experiment_mode:=plant_test gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase4_stage0/static.csv \
  replay_output_csv:=/tmp/windylab_phase4_stage2_rate_probe.csv \
  start_delay_sec:=30.0 \
  replay_clock_source:=sim \
  rgbd_width:=320 rgbd_height:=240 imu_enabled:=false
```

另一个终端执行：

```bash
ros2 param get /gazebo publish_rate
timeout 5 ros2 topic hz /clock --window 100
timeout 5 ros2 topic hz /model_states --window 100
```

控制 timer 发布率检查，把启动模式改为 `experiment_mode:=baseline` 后执行：

```bash
timeout 5 ros2 topic hz /arm_velocity_controller/commands --window 100
timeout 5 ros2 topic hz /joint_states --window 100
```

暂停/恢复试验。每次独立启动 5 s `sine_x`：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  experiment_mode:=plant_test gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase4_stage1_sine_x_5s.csv \
  replay_output_csv:=/tmp/windylab_phase4_stage2_pause_r1_replay.csv \
  start_delay_sec:=2.0 \
  replay_clock_source:=sim \
  hold_initial_state_during_start_delay:=true \
  rgbd_width:=320 rgbd_height:=240 imu_enabled:=false
```

replay 开始约 1.5 s 后暂停 2 s 墙钟再恢复：

```bash
gz world -p 1
sleep 2
gz world -p 0
```

分别生成 `r1/r2/r3` 后汇总：

```bash
ros2 run manipulator phase4_dynamic_xyz_metrics.py \
  --run pause_r1=/tmp/windylab_phase4_stage2_pause_r1_replay.csv \
  --run pause_r2=/tmp/windylab_phase4_stage2_pause_r2_replay.csv \
  --run pause_r3=/tmp/windylab_phase4_stage2_pause_r3_replay.csv \
  --output-csv /tmp/windylab_phase4_stage2_pause_summary.csv
```

### 硬门禁与结果

| 门禁 | 通过条件 | 实际结果 | 判定 |
|---|---|---|---|
| Gazebo clock 参数 | `/gazebo.publish_rate=100` | `100.0` | 通过 |
| clock 实测频率 | 95–105 Hz | `99.967–99.992 Hz` | 通过 |
| model state 实测频率 | 95–105 Hz | `100.004 Hz` | 通过 |
| ROS timer 控制命令 | 95–105 Hz | `100.011 Hz` | 通过 |
| joint state | 95–105 Hz | `99.904 Hz` | 通过 |
| pause 数据完整性 | 3 次均 501 样本、0 set failure | 3/3 满足 | 通过 |
| 时间源 | 全部 replay 为 `schedule_clock=sim` | 3/3 满足 | 通过 |
| 仿真轨迹时长 | `5.00 ± 0.01 s` | 三次均 `5.000 s` | 通过 |
| 暂停不推进轨迹 | 墙钟时长 ≥6.5 s，wall gap ≥1.8 s | `7.276–7.288 s`，gap `2.285–2.295 s` | 通过 |
| 调度迟到 | 最大不超过一个 10 ms 周期 | `0、10、0 ms` | 通过 |
| 仿真样本间隔 | 最大不超过两个 10 ms 周期 | `10、20、10 ms` | 通过 |
| Base 跟踪 RMS | ≤0.5 mm | `0.164、0.331、0.164 mm` | 通过 |
| Base 跟踪最大值 | ≤2.1 mm | `0.320、1.978、0.320 mm` | 通过 |
| Base 姿态误差 | ≤0.001 rad | 三次均为 0 | 通过 |

### 关于一次过严门禁失败

三次 pause 数据首次汇总时使用了 `sim_sample_gap_max <= 0.011 s`。r2 出现一个 `0.020 s` 间隔，该断言失败；同次 `schedule_error_max=0.010 s`，后续样本补齐，最终仍为 501 样本和精确 5.000 s 仿真时长。

原因是一次 `/set_entity_state` 服务调用跨过了一个 10 ms clock tick。它没有在暂停期间推进轨迹，也没有丢 CSV 样本。时间门禁因此明确为：单个样本最多迟到一个周期，相邻样本最多间隔两个周期。该失败和门禁修订均保留，不通过修改控制参数处理。

### 暂停试验核心数据

| 运行 | schedule RMS/最大迟到 | sim 时长 | wall 时长 | sim 最大间隔 | wall 最大间隔 |
|---|---:|---:|---:|---:|---:|
| r1 | `0 / 0 ms` | 5.000 s | 7.276 s | 10 ms | 2.285 s |
| r2 | `1.842 / 10 ms` | 5.000 s | 7.284 s | 20 ms | 2.293 s |
| r3 | `0 / 0 ms` | 5.000 s | 7.288 s | 10 ms | 2.295 s |

### 达到的效果

- Gazebo clock、Gazebo state、ROS timer 控制发布和 Base replay 统一在 100 Hz 仿真时间轴上。
- Gazebo 暂停 2 s 时，轨迹仿真时间不前进；恢复后总样本数和 5 s 轨迹时长不变。
- 子阶段 1 中 2–3 cm 的状态缓存表观误差降为亚毫米 RMS，证明状态观测与命令频率已经对齐。
- replay CSV 同时保留 sim/wall 时间，可以明确区分仿真暂停、实时因子变化和调度迟到。
- 没有修改 CLIK、视觉、限幅或 safety 参数。

### 版本控制验收

本子阶段提交说明固定为：

```text
phase4: schedule replay on the gazebo clock
```

提交、推送并执行：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws/src/arm-platform
git status --short --branch
git log -1 --oneline
git rev-parse HEAD
git rev-parse origin/develop
```

只有工作区干净且两个提交 ID 一致，才允许开始子阶段 3。

---

## 子阶段 3：验证 Pinocchio 与 Gazebo 运动学一致

### 状态

通过。完成日期：2026-07-17。

三次独立 Gazebo 实例共 72 个用例全部通过。完成本节提交、推送并确认 `HEAD == origin/develop` 之前，子阶段 4 仍保持锁定。

### 目的

本子阶段只回答运动学层问题，不修改 GT/视觉闭环、CLIK 参数、关节限幅或 safety：

1. Pinocchio 的关节顺序和正方向是否与 Gazebo 一致。
2. Pinocchio 的 `base_link -> link6` FK 是否与 Gazebo 的 `inv(T_world_base) * T_world_link6` 使用同一 frame、尺度和姿态约定。
3. 一致性是否同时成立于零构型和一个非零构型，而不是只在初始点偶然成立。
4. 六个关节正、反方向均产生足够的实际运动；平移很小的腕部轴仍必须通过完整姿态增量检查。

### 固定验收配置

| 项目 | 固定值 |
|---|---|
| Pinocchio 模型 | `config/arm.urdf` |
| EE frame/link | `link6` / `windylab_arm::link6` |
| Base link | `windylab_arm::base_link` |
| 构型 0 | `[0, 0, 0, 0, 0, 0] rad` |
| 构型 1 | `[0.20, -0.25, 0.30, -0.15, 0.10, -0.10] rad` |
| 构型到位容差 | `0.005 rad`，使用实际 `/joint_states` 做 FK，不用名义目标代替 |
| 单轴激励 | `+/-0.03 rad/s`，各持续 `1.80 s` 仿真时间 |
| Base 输入 | 600 s static，100 Hz，seed 42 |
| 重复方式 | 每次全新启动 Gazebo，共 3 次 |

600 s static 轨迹为 60001 个样本，SHA-256：

```text
94a4d4ab7e82d4e73536189db3a02088c8ab1018b65fe0fb36089d9145897a18
```

### 修改内容

- `scripts/phase4_kinematic_consistency.py`
  - 新增独立运动学验收节点；不复用被测控制器输出的 FK 结果。
  - 从按名称解析后的六关节实际位置计算 Pinocchio `base_link -> link6` FK。
  - 从 `/link_states` 独立计算 Gazebo `inv(T_world_base) * T_world_link6`。
  - 在两个构型下对六个关节逐一施加正、反向速度脉冲，共 24 个用例。
  - 同时检查绝对位置/姿态、脉冲前后位置/姿态增量、平移方向余弦和尺度比。
  - `/joint_states` 与 `/link_states` 的订阅深度固定为 1，避免处理大消息时使用积压状态。
  - 到位过程中若关节状态超过 `0.10 s` 墙钟未刷新，立即持续发布零速度，不允许在陈旧反馈下保留旧命令。
  - 到位超时按仿真时间计算；仿真停止推进另有墙钟超时。
  - 正常结束和异常退出均重复发布 `0.25 s` 零速度，避免诊断节点退出后控制器保留最后一条速度。
  - 每完成一个用例即更新 CSV，使中途失败仍保留已完成证据；`--require-pass` 在任一用例失败时返回非零状态。
- `CMakeLists.txt`
  - 安装新增验收脚本。

没有修改 URDF 几何、Gazebo plant、replay、控制器、CLIK、视觉或 safety 参数。

### 验收终端命令

构建和静态检查：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  src/arm-platform/scripts/phase4_kinematic_consistency.py
git -C src/arm-platform diff --check
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
```

生成足够覆盖整次诊断的 static 轨迹：

```bash
ros2 run manipulator generate_base_disturbance.py \
  --config src/arm-platform/config/base_disturbance_profiles.yaml \
  --profile static --duration-sec 600 --sample-rate-hz 100 --random-seed 42 \
  --output-csv /tmp/windylab_phase4_stage3_static_600s.csv
wc -l /tmp/windylab_phase4_stage3_static_600s.csv
sha256sum /tmp/windylab_phase4_stage3_static_600s.csv
```

每个 `r1/r2/r3` 都独立启动以下 launch，只替换两个输出文件中的运行编号：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  experiment_mode:=plant_test gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase4_stage3_static_600s.csv \
  replay_output_csv:=/tmp/windylab_phase4_stage3_r1_replay.csv \
  start_delay_sec:=5.0 replay_clock_source:=sim \
  hold_initial_state_during_start_delay:=true \
  replay_state_sample_stride:=10 \
  rgbd_width:=320 rgbd_height:=240 rgbd_update_rate:=1.0 \
  imu_enabled:=false
```

确认控制器 active 后，在另一个终端运行；命令返回 0 后停止本次 launch，再全新启动下一次：

```bash
ros2 control list_controllers

ros2 run manipulator phase4_kinematic_consistency.py \
  --output-csv /tmp/windylab_phase4_stage3_kinematic_r1.csv \
  --require-pass --use-sim-time

wc -l /tmp/windylab_phase4_stage3_kinematic_r1.csv
```

三个结果文件均为 25 行，即 1 行表头加 24 个完整用例。

### 硬门禁与结果

| 门禁 | 通过条件 | 三次最坏值 | 判定 |
|---|---:|---:|---|
| 独立重复与完整性 | 3 次均 24/24 | `72/72` | 通过 |
| 实际关节运动 | 每轴每方向 `>=0.004 rad` | 最小 `0.005720577 rad` | 通过 |
| 绝对位置一致 | `<=0.001 m` | `7.516971e-16 m` | 通过 |
| 绝对姿态一致 | `<=0.005 rad` | `2.107342e-08 rad` | 通过 |
| 位置增量一致 | `<=0.0005 m` | `9.539781e-16 m` | 通过 |
| 姿态增量一致 | `<=0.005 rad` | `2.980232e-08 rad` | 通过 |
| 平移方向 | 可观测平移的余弦 `>=0.98` | 最小 `1.0` | 通过 |
| 平移尺度 | 相对误差 `<=10%` | 最大 `3.091971e-12` | 通过 |

逐次汇总：

| 运行 | 用例 | 最小实际关节位移 | 最大绝对位置误差 | 最大位置增量误差 | 最大姿态增量误差 |
|---|---:|---:|---:|---:|---:|
| r1 | 24/24 | `0.005720577 rad` | `7.185693e-16 m` | `6.359601e-16 m` | `2.980232e-08 rad` |
| r2 | 24/24 | `0.007571358 rad` | `7.516971e-16 m` | `9.539781e-16 m` | `2.107342e-08 rad` |
| r3 | 24/24 | `0.007571358 rad` | `5.748822e-16 m` | `8.471580e-16 m` | `2.980232e-08 rad` |

构型 0 下 joint4 的末端平移低于 `1e-4 m`，因此不对该用例伪造平移方向成绩；其关节方向、绝对姿态和姿态增量仍全部通过。构型 1 下 joint4 产生可观测平移，其正反方向余弦和尺度同样通过。

### 验收过程中发现并修正的问题

1. 首次运行在采样前暴露 `configuration/configurations` 参数字段命名错误；修正后重新构建，未把该次计入正式结果。
2. 初始 `0.2 s` 脉冲使不同关节只移动 `0.00088–0.00577 rad`。没有降低 `0.004 rad` 门禁，而是把脉冲延长到 `1.80 s`，正式三次的最小实际位移达到 `0.00572 rad`。
3. 初版到位超时使用墙钟，实时因子变化时会提前退出；改为仿真时间后再验收。
4. 异常退出时单次零速度可能尚未送达 BEST_EFFORT 控制器，旧速度会被保留；改为正常/异常退出都重复发布零速度。
5. 一次探索运行在零构型 12/12 通过后，切换非零构型发散到 `1.862952 rad`。失败发生时仿真时间约 369 s，原 300 s Base 保持窗口已经结束，并且两个 100 Hz 状态订阅各积压 50 条。将 static 输入延长到 600 s、订阅深度改为 1，并增加陈旧关节反馈停机保护后，重新从 r1 计数的三次均为 24/24。

这些失败均保留为诊断方法的修订依据；没有通过放宽 FK 误差、方向或尺度门限来获得通过结果。

### 达到的效果

- 已证明 Pinocchio 与 Gazebo 对六关节顺序、正负方向、米/弧度尺度及 `base_link -> link6` frame 的定义一致。
- 该结论覆盖零构型、非零构型、正反方向和完整位置/姿态，不只是单点 XYZ 符号检查。
- 因此，阶段 0 看到的整机大幅摆动和末端无抗扰意图不能归因于 Pinocchio/Gazebo 几何模型、关节顺序或 frame 符号不一致。
- 子阶段 4 可以在提交推送完成后，单独检验 Gazebo GT 动态 XYZ 闭环；视觉链路仍保持锁定。

### 版本控制验收

本子阶段提交说明固定为：

```text
phase4: verify gazebo and pinocchio kinematics
```

提交、推送并执行：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws/src/arm-platform
git status --short --branch
git log -1 --oneline
git rev-parse HEAD
git rev-parse origin/develop
```

只有工作区干净且两个提交 ID 一致，才允许开始子阶段 4。

---

## 子阶段 4：仅用 Gazebo GT 完成动态 XYZ 闭环

### 状态

通过。完成日期：2026-07-17。

这里的通过只证明 Gazebo plant、GT 位姿反馈、Pinocchio CLIK 和速度控制器可以在连续 `sine_x` Base 平移下形成有效 XYZ 抗扰闭环。视觉估计和 safety 状态机尚未参与，子阶段 5 在本阶段提交推送完成前继续锁定。

### 目的与冻结门禁

本阶段先排除视觉链路，只回答两个问题：

1. Base 是规定边界输入、而不是受关节反作用自由漂移时，机械臂是否能产生方向正确的末端抗扰动作。
2. 在同一 30 s 正弦轨迹下，GT 闭环是否稳定、可重复并显著优于零命令 baseline。

正式验收在调参前冻结为：

| 项目 | 硬门禁 |
|---|---:|
| 轨迹 | `sine_x`，30 s，100 Hz，seed 42；所有运行使用相同 CSV |
| 独立重复 | 3 组 baseline/GT 配对；每个运行均全新启动 Gazebo |
| 运行有效性 | 初始 Base/关节正常、3001 个 GT 样本有效、`set_failures=0`、不发散 |
| Base 位置跟踪 | RMS `<=0.0005 m`，最大值 `<=0.0021 m` |
| Base 姿态保持 | 最大误差 `<=0.01 rad` |
| 单对改善 | 每个 GT 的 XYZ RMS 相对同组 baseline 至少下降 30% |
| 重复性 | 三次 GT XYZ RMS 的样本变异系数 CV `<=5%` |

### 内容修改

- `src/gazebo_base_command_hold_plugin.cpp`
  - 新增可选 Gazebo Classic ModelPlugin，订阅 `/windylab/base_command`。
  - 只接受目标模型、`world` 参考系、有限数值和非零范数四元数。
  - 在 Gazebo 物理线程内每 6 个 1 ms 物理步规定一次模型 pose/twist，抑制关节执行器反作用造成的自由 Base 漂移。
  - 使用目标线速度在两条 100 Hz replay 指令之间外推位置，避免把离散轨迹点反复写回造成约 5 ms 等效滞后。
  - 插件只在收到第一条有效命令后生效；普通 `gazebo_arm.launch.py` 默认不启用。
- `scripts/base_disturbance_replay.py`
  - 在保留 `/set_entity_state` 服务调用、成功率和时序指标的同时，把同一 `EntityState` 发布到 `/windylab/base_command`。
- `config/arm_with_d435i.urdf.xacro`、`config/sensors/gazebo_ros2_control.xacro`
  - 新增默认关闭的 `base_command_hold_enabled`；配置插件话题和 6 个物理步保持间隔。
- `launch/gazebo_arm.launch.py`
  - 传递插件开关，并把本包 `lib` 加入该 launch 进程的 `GAZEBO_PLUGIN_PATH`。
- `launch/moving_base_stabilization.launch.py`
  - 仅移动 Base 实验启用插件。
  - GT 与视觉控制参数彻底分离，避免为 GT 提高速度上限时同时改变尚未验收的视觉控制器。
  - 冻结 GT 参数：100 Hz、任务增益 `[5,5,5]`、任务速度上限 `[0.35,0.35,0.35] m/s`、关节速度上限 `1.5 rad/s`、关节加速度上限 `3 rad/s^2`、阻尼 `0.02`。
- `scripts/ground_truth_xyz_controller.py`
  - JointState 和 LinkStates 队列深度从 20 改为 1，避免使用积压反馈。
  - 增加阻尼、任务增益和任务速度向量的有限值/范围校验。
  - 修正字符串 launch 参数与向量参数默认类型的一致性，并使正常 ROS shutdown 不产生误报异常。
- `CMakeLists.txt`、`package.xml`
  - 增加 Gazebo 插件构建、依赖和安装规则。

视觉估计、视觉 CLIK 参数、deadband、丢帧策略和 safety 阈值均未修改。

### 验收终端命令

构建和静态检查：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws/src/arm-platform
git diff --check
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  launch/gazebo_arm.launch.py \
  launch/moving_base_stabilization.launch.py \
  scripts/base_disturbance_replay.py \
  scripts/ground_truth_xyz_controller.py

cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws
source setup_env.bash
colcon build --packages-select manipulator --symlink-install
source install/setup.bash
```

生成冻结正式轨迹：

```bash
ros2 run manipulator generate_base_disturbance.py \
  --config src/arm-platform/config/base_disturbance_profiles.yaml \
  --profile sine_x --duration-sec 30 --sample-rate-hz 100 --random-seed 42 \
  --output-csv /tmp/windylab_phase4_stage4/formal_sine_x_30s.csv
wc -l /tmp/windylab_phase4_stage4/formal_sine_x_30s.csv
sha256sum /tmp/windylab_phase4_stage4/formal_sine_x_30s.csv
```

强关节脉冲 plant 门禁。每次独立启动以下 launch；正式复现时建议使用子阶段 3 的 600 s static CSV，保证脉冲期间 replay 仍在发布目标：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  experiment_mode:=plant_test gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase4_stage3_static_600s.csv \
  replay_output_csv:=/tmp/windylab_phase4_stage4/plugin_n6_pulse_r1_replay.csv \
  start_delay_sec:=20.0 hold_initial_state_during_start_delay:=true \
  rgbd_width:=320 rgbd_height:=240 rgbd_update_rate:=1.0 imu_enabled:=false
```

确认控制器 active 后运行；返回 0 后停止 launch，独立重启并替换 `r1/r2/r3`：

```bash
ros2 control list_controllers
ros2 run manipulator phase4_joint_pulse_check.py \
  --output-csv /tmp/windylab_phase4_stage4/plugin_n6_pulse_r1.csv \
  --joint-name joint2 --velocity 1.0 --pulse-duration-sec 0.5 \
  --pre-sec 0.5 --duration-sec 3.0 --sample-hz 100 \
  --max-base-displacement-m 0.001 \
  --max-base-orientation-error-rad 0.01 \
  --min-joint-motion-rad 0.1 --require-base-stable --use-sim-time
```

正式 baseline。每次完成 replay 后停止 launch；替换 `r1/r2/r3` 并独立重启：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  experiment_mode:=baseline gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase4_stage4/formal_sine_x_30s.csv \
  replay_output_csv:=/tmp/windylab_phase4_stage4/formal_r1_baseline_replay.csv \
  start_delay_sec:=5.0 hold_initial_state_during_start_delay:=true \
  rgbd_width:=320 rgbd_height:=240 rgbd_update_rate:=1.0 imu_enabled:=false
```

正式 GT，运行方式和编号与 baseline 一一配对：

```bash
ros2 launch manipulator moving_base_stabilization.launch.py \
  experiment_mode:=ground_truth_xyz gui:=false use_rviz:=false \
  disturbance_csv:=/tmp/windylab_phase4_stage4/formal_sine_x_30s.csv \
  replay_output_csv:=/tmp/windylab_phase4_stage4/formal_r1_gt_replay.csv \
  start_delay_sec:=5.0 hold_initial_state_during_start_delay:=true \
  rgbd_width:=320 rgbd_height:=240 rgbd_update_rate:=1.0 imu_enabled:=false
```

统一汇总 6 次运行：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws/src/arm-platform
python3 scripts/phase4_dynamic_xyz_metrics.py \
  --run r1_baseline=/tmp/windylab_phase4_stage4/formal_r1_baseline_replay.csv \
  --run r1_gt=/tmp/windylab_phase4_stage4/formal_r1_gt_replay.csv \
  --run r2_baseline=/tmp/windylab_phase4_stage4/formal_r2_baseline_replay.csv \
  --run r2_gt=/tmp/windylab_phase4_stage4/formal_r2_gt_replay.csv \
  --run r3_baseline=/tmp/windylab_phase4_stage4/formal_r3_baseline_replay.csv \
  --run r3_gt=/tmp/windylab_phase4_stage4/formal_r3_gt_replay.csv \
  --output-csv /tmp/windylab_phase4_stage4/formal_all_summary.csv \
  --max-joint-velocity 1.5
```

最终门禁脚本从汇总 CSV 检查逐对改善、Base 继承门槛和 CV：

```bash
python3 - <<'PY'
import csv
import statistics

with open('/tmp/windylab_phase4_stage4/formal_all_summary.csv') as handle:
    runs = {row['label']: row for row in csv.DictReader(handle)}

gt_rms = []
passed = True
for index in range(1, 4):
    baseline = runs[f'r{index}_baseline']
    gt = runs[f'r{index}_gt']
    baseline_rms = float(baseline['xyz_rms_m'])
    gt_value = float(gt['xyz_rms_m'])
    pair_pass = (
        baseline['run_valid'] == 'True'
        and gt['run_valid'] == 'True'
        and gt_value <= 0.70 * baseline_rms)
    plant_pass = (
        float(gt['base_pose_tracking_error_rms_m']) <= 0.0005
        and float(gt['base_pose_tracking_error_max_m']) <= 0.0021
        and float(gt['base_orientation_tracking_error_max_rad']) <= 0.01)
    passed = passed and pair_pass and plant_pass
    gt_rms.append(gt_value)
    print(index, pair_pass, plant_pass, (baseline_rms - gt_value) / baseline_rms)

cv = statistics.stdev(gt_rms) / statistics.mean(gt_rms)
print('gt_cv', cv)
print('formal_gate', passed and cv <= 0.05)
raise SystemExit(0 if passed and cv <= 0.05 else 1)
PY
```

### 强脉冲门禁结果

冻结 6 步保持间隔后重新从 r1 计数，三次均通过：

| 运行 | joint2 实际运动 | Base 最大平移 | Base 最大姿态误差 | 判定 |
|---|---:|---:|---:|---|
| r1 | `0.296559591 rad` | `0.000220272 m` | `0.002544013 rad` | 通过 |
| r2 | `0.296560479 rad` | `0.000220264 m` | `0.002540866 rad` | 通过 |
| r3 | `0.296673757 rad` | `0.000220388 m` | `0.002539291 rad` | 通过 |

这比原阶段 1 的 `0.02 rad/s` 小脉冲更强：本次命令为 `1.0 rad/s`、持续 `0.5 s`，实际关节运动大于 `0.296 rad`，因此不是通过冻结关节换取 Base 稳定。

### 正式动态 XYZ 结果

冻结轨迹为 3001 样本，SHA-256：

```text
10add165bf9dbf6b44081b5ec862e41587b0c8e4e79adc55986f60ac8fd22b94
```

逐对结果：

| 配对 | baseline RMS | GT RMS | RMS 改善 | GT 最大误差 | 判定 |
|---|---:|---:|---:|---:|---|
| r1 | `0.067636302 m` | `0.047232754 m` | `30.166563%` | `0.0743585 m` | 通过 |
| r2 | `0.067636040 m` | `0.047141540 m` | `30.301154%` | `0.0738316 m` | 通过 |
| r3 | `0.067636361 m` | `0.046674067 m` | `30.992639%` | `0.0728796 m` | 通过 |

Base 与命令完整性：

| 运行 | Base 位置 RMS | Base 位置最大值 | Base 姿态最大值 | 关节速度峰值 | 饱和比例 | set failure |
|---|---:|---:|---:|---:|---:|---:|
| GT r1 | `0.000246945 m` | `0.001703371 m` | `0.005636195 rad` | `1.495474 rad/s` | `0` | `0` |
| GT r2 | `0.000244298 m` | `0.000528550 m` | `0.005612700 rad` | `1.477721 rad/s` | `0` | `0` |
| GT r3 | `0.000243834 m` | `0.000523759 m` | `0.005565222 rad` | `1.439999 rad/s` | `0` | `0` |

三次 GT RMS 均值为 `0.047016120 m`，样本变异系数 CV 为 `0.637477%`，远低于 5% 门槛。最终脚本输出 `formal_gate true`。

正式文件 SHA-256：

```text
5d46f7b4f705cc8b43cf2d4e0345f2918610d28fec77a1dbfc5a31fda0a056f4  formal_r1_baseline_replay.csv
2b8c52eee316556660a0fedaecdb2320c5327f3eeafac120266cafe5d952cfa6  formal_r1_gt_replay.csv
eb772af0b1dc8507debdf1f841dbe198be6fbf73e7c8ab71c77b6dbd4f75da87  formal_r2_baseline_replay.csv
1bdf31cf3a05ec0ddaf23c0faea1e6d3ea509bda0e15cf961652c814bc94f34f  formal_r2_gt_replay.csv
966dc389a3a2b7011cc0234bdc70ccda7058728fa86a28eafe593a4f005f5161  formal_r3_baseline_replay.csv
16f13ace260a5acf4b140859dd7661b3149b609142d689f6a05aeab5dd52d87e  formal_r3_gt_replay.csv
237f284aab925178d79b6b97876d56ab4a20b1d1c900448aa862a6d9718116cb  formal_all_summary.csv
```

### 验收过程中发现并修正的问题

1. 原 GT 复用视觉控制的 `0.05 m/s` 任务速度、`0.2 rad/s` 关节速度和 `0.3 rad/s^2` 加速度上限，而 `sine_x` 的 Base 峰值速度为 `0.2199 m/s`；旧 GT 在当前冻结轨迹上比 baseline 差约 14.1%。因此先分离 GT/视觉参数，没有改尚未验收的视觉参数。
2. 只提高 GT 速度后，12 s 预检可改善 30% 以上，但强命令使自由 Base 姿态误差达到 `0.0124–0.0160 rad`，违反子阶段 1 的 Base plant 门槛。没有用 GT 成绩掩盖 plant 失败。
3. 尝试把 `/set_entity_state` 服务提高到 200 Hz 时，强脉冲 Base 姿态仍为 `0.01147 rad`；300 Hz 时数值发散到约 `45.3 m / 3.13 rad`。该服务频率方案已完整撤销。
4. 插件每个 1 ms 物理步调用模型级 `SetWorldPose` 时 Base 完全稳定，但关节实际运动为 0；只重定位 `base_link` 又造成 `0.420 m / 2.841 rad` Base 发散和 `7.05 rad` 关节异常。两种方案均被强脉冲门禁否决。
5. 模型级保持每 4 步执行时，强脉冲通过，但离散 pose 被反复写回使正弦 Base 跟踪 RMS 增至 `1.052 mm`。加入线速度外推后恢复到 `0.150 mm`。
6. 4 步保持间隔把 `1 rad/s * 0.5 s` 的关节运动削弱到约 `0.266 rad`；放宽到 6 步后实际运动约 `0.297 rad`，同时三次 Base 强门禁仍通过。
7. GT 预检严格逐项修改：gain 4 改善 `22.98%`；gain 5 且任务上限 `0.30 m/s` 改善 `28.46%`；保持 gain 5、仅把任务上限提高到 `0.35 m/s` 后改善 `30.08%`，才冻结正式参数。gain 6 产生限幅并恶化，加速度上限从 3 提高到 6 没有改善，均未保留。

没有降低 30% 改善、Base 位姿或重复性门槛，也没有把失败探索计入正式三次。

### 达到的效果

- 原始故障中的“关节反作用推动自由 Base、整机大幅摆动”已在移动 Base launch 中被消除；强关节脉冲下 Base 仍保持在毫米/百分之一弧度门槛内。
- Base 仍能按 100 Hz 正弦轨迹规定移动，不是改成固定根；正式 baseline 的末端 RMS 与子阶段 0 的 `0.06763 m` 一致。
- GT 闭环会产生最高约 `1.50 rad/s` 的明确抗扰关节动作，三次都把世界系末端 XYZ RMS 降低至少 30%，且没有速度饱和或发散。
- 这证明阶段 0 中“末端看不到抗扰意图”不能归因于运动学或 Gazebo GT 闭环能力；剩余工作应按门禁转向 safety 状态机和视觉测量链路。
- 本结果只覆盖计划中的 `sine_x` GT 子阶段，不能替代后续视觉闭环或最终 `sine_x/y/z/xyz` 完整验收。

### 版本控制验收

本子阶段提交说明固定为：

```text
phase4: stabilize gazebo ground-truth xyz control
```

提交、推送并执行：

```bash
cd /home/yihuang/westlake/windylab-arm-for6/windylab_ws/src/arm-platform
git status --short --branch
git log -1 --oneline
git rev-parse HEAD
git rev-parse origin/develop
```

只有工作区干净且两个提交 ID 一致，才允许开始子阶段 5。
