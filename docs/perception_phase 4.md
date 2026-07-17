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
| 1 | 修正 Gazebo Base plant | 已知小关节脉冲下 Base 不发生非指令位移；3 次结果通过阈值 | 未开始，锁定 |
| 2 | 统一仿真时间和回放时序 | 回放、感知、控制都使用同一仿真时间；暂停/恢复不破坏轨迹 | 未开始，锁定 |
| 3 | 验证 Pinocchio 与 Gazebo 运动学一致 | 多构型、多轴小脉冲的方向、尺度和 frame 一致 | 未开始，锁定 |
| 4 | 仅用 Gazebo GT 完成动态 XYZ 闭环 | GT 相比 baseline 的 RMS 明显下降，且不发散、可重复 | 未开始，锁定 |
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
