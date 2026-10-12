# 遥操作跟手与录制循环回放方案

日期：2026-10-12。本文只读代码后给出设计，不改控制行为。安全边界以 [PR #1](https://github.com/DucksAreRoasted/openarm-vr-teleop/pull/1)（分支 `cursor/repo-health-check-a4e9`）的 `docs/HEALTH_CHECK.md` 为准，下文用 A1–A3、B1 指那份并行计划里的任务编号。

本次没有编译、没有跑仿真、没有接 Quest、CAN 或真机。凡是头显刷新、Wi-Fi、总线占用和手感，都标了「未验证」。文件路径和常数来自当前 `main` 的源码。

默认遥操模式是 `human_mimic`（`VR_TELEOP_MODE`，入口 `control/openarm_quest_cartesian_control.cpp` 里 `teleop_mode` 的初值）。下面的延迟结论按这条路径写。`cartesian_precision` 和 `skeleton_proximal` 只在它们已经做成、而默认模式还没做成的地方被拿来对照。

## 0. 五条优先建议

1. **把关节参考速度送进 MIT 的 `dq`。** 默认模式在 `HumanMimicController::fill_reference()` 里把每个关节的 `velocity` 写成 0，跟随线程再原样打进 MIT。`config/follower.yaml` 近端 `Kd` 为 3.0，电机侧阻尼会对着真实速度往回拉。关节参考发生器里已经有平滑过的 `qdot`。这一项只在真机上能感觉到：仿真在 `kp>0` 时直接改 MuJoCo 的 `qpos`。
2. **默认模式的整臂 IK 按骨骼模式的办法降到约 60–90 Hz，关节参考仍按 1000 Hz 往前走。** 现在参考线程以 `FREQUENCY`（1000 Hz）对同一帧手柄姿态重复做最多 12 次 DIFF-QP。骨骼模式已经在 `skeleton_proximal_controller.cpp` 里写明：同一帧头显数据不该在 1000 Hz 上反复求逆解。
3. **先把姿态到达时刻量准，再做最多一帧的外推。** 网页把追踪限在 60 Hz；控制器再用 90 Hz 线程轮询 UDP，时间戳盖的是轮询醒来的时刻。外推放在 B1 把姿态口收成回环或令牌之后，预测长度不超过约 20 ms，年龄到 50 ms 就停止外推并保持最后一次关节目标，150 ms 仍冻结。超时后的重新握持留给 A1，本方案不改 `last_grip_`。
4. **录制写盘离开 1000 Hz 跟随线程。** 双臂遥操在 `BimanualFollowerControlThread::on_timer()` 里每次 MIT 发送后同步写 CSV。录制一旦打开，磁盘延迟会直接拉长控制周期。指令轨迹降到与回放相同的 200 Hz 再写；控制台以控制器打出的录制日志更新按钮状态。
5. **循环回放复用现有接入和限速，不新写一套伺服。** 圈与圈之间用现在的慢速接入（0.3 rad/s、0.6 rad/s²）从末帧走到首帧；播放中仍是臂 0.6 rad/s、1.2 rad/s²。停止先把指令速度收到 0，再走今天的 `disable_all()`。仿真和真机共用 `TeachingReplayReference` 的状态机。不放宽同向示教那条 0.45 rad 跟踪误差和 150 ms CAN 检查。

## 1. 更跟手、更流畅

### 1.1 当前链路

Grip 按下后的默认数据流：

1. Quest 浏览器跑 `quest/client.html` 的 WebXR 帧回调，读 `gripSpace` 的位置和四元数，经 WSS 8442 发给 `quest/quest_bridge.py`。
2. 桥把每只手打成 CSV，UDP 发到本机 `47911`（右）和 `47912`（左）。
3. `QuestReceiveThread` 排空 socket，用本机 `steady_clock` 写入 `receive_timestamp_ns`，只保留最新一包。
4. `CartesianReferenceThread` 读缓存姿态，跑 `HumanMimicController::step()`：虚拟臂骨骼、整臂 IK、`JointReferenceGenerator`，再 `set_all_references()`。
5. `BimanualFollowerControlThread` 以 1000 Hz 调用 `Control::unilateral_step()`，把参考打成 MIT `{kp, kd, q, dq, tau=0}`，然后 `recv_all(200)`。

S 曲线（`src/controller/s_curve.hpp`，默认 0.6 rad/s、1.2 rad/s²、6 rad/s³）用在真机启动轨迹、双臂 CSV 启动和 MoveIt 段内加减速。Grip 跟随环不经过它。把 `OPENARM_SCURVE_VMAX` 调大，不会让遥操更跟手。

### 1.2 各段实际在做什么

| 环节 | 位置 | 从代码能确定的数字 |
| --- | --- | --- |
| 头显采样上限 | `quest/client.html` 约 297 行 | `time - lastTrackingUpdate < 1000/60` 时直接返回。追踪被限在 60 Hz，帧间隔 16.7 ms。头显若以 72/90/120 Hz 刷新，多出来的帧被丢掉。 |
| Quest 时间戳 | 同文件约 308 行；`quest_bridge.py` 原样转发 | `Math.round(performance.now() * 1000)`，单位是微秒。C++ 字段名叫 `timestamp_ms`（`cartesian_types.hpp`）。控制律不读它。 |
| WebSocket | `quest_bridge.py` `handle_client` | 一帧一个 JSON，左右手一起解析。时间戳不递增则整帧丢弃。桥到控制器的 UDP 目标默认 `127.0.0.1`。 |
| UDP 接收 | `src/io/quest_pose_receiver.cpp` | 非阻塞，绑 `INADDR_ANY`（健康检查 H4）。一次排空里后到的包覆盖先到的包，应用层没有多包队列。同一轮排空里的包共用排空开始时的 `now_ns`。 |
| 接收线程 | `kQuestReceiveHz = 90` | 周期 11.1 ms。包在内核缓冲里等到下一次 `on_timer` 才被看见。独立均匀到达时，这段等待的平均值约 5.6 ms，上限约 11.1 ms。该线程的 `before_start` 没有调用基类，因此没有 `SCHED_FIFO`。 |
| 新鲜度 | `human_mimic_types.hpp`；`human_mimic_controller.cpp` 约 2345–2364、2641 行 | `quest_stale_sec = 0.05`，`quest_timeout_sec = 0.15`。年龄 = 控制律时刻 − `receive_timestamp_ns`。50–150 ms 不再用这包姿态更新 IK，关节参考继续走向最后一次 `q_target`；≥150 ms 调用 `hold_reference()` 冻结，并把 `last_grip_` 写成 false（健康检查 H1）。 |
| 姿态滤波 | 同文件；`filter_orientation()` | 朝向一阶 SLERP，`tau_orientation = 0.006` s，死区 `0.05°`。位置没有低通。`fine_motion_hold_enabled` 默认关。握持后要平移超过 `clutch_start_motion_m = 0.003` m 或转过 `clutch_start_rotation_deg = 2°` 才开始跟。 |
| 参考线程 | `kCartesianDlsHz = FREQUENCY = 1000` | `on_timer` 里的 `dt` 被夹在 0.5–3 倍名义周期，也就是 0.5–3 ms。IK 超时后，关节参考的积分会慢于墙钟。 |
| 整臂 IK | `HumanMimicConfig` | 默认后端 `DIFF_QP`，`ik_max_iters = 12`。之后还有 `q_target_slew_max_radps = 10`，肘关节 `elbow_joint_slew_radps = 4.5`，肘 swivel `elbow_cmd_slew_radps = 2.0`。 |
| 骨骼模式的 IK 节拍 | `skeleton_proximal_types.hpp` `ik_update_hz = 60` | 同文件 `k_joint = 10`、`qdot_max = 3.5`、`qddot_max = 40`，比默认模式更慢。它的 `fill_reference` 同样把速度写成 0。 |
| 关节参考发生器 | `joint_reference_generator.cpp` | `v = clamp(k*(q_target-q_ref), ±qdot_max)`，再对加速度限幅。默认 `k_joint = 14`，`qdot_max = 6` rad/s，`qddot_max = 80` rad/s²。 |
| 关节输出低通 | `joint_output_filter_enabled = false` | 打开后才使用 14 Hz 低通、3.2 rad/s、16 rad/s²。类型注释写明曾经引起近端抖动，只留环境变量 `OPENARM_HM_Q_FILT` 做对照。本方案不建议打开它。 |
| MIT | `config/follower.yaml`；`control.cpp` 跟随分支 | `Kp = [240, 240, 240, 240, 24, 31, 25, 16]`，`Kd = [3, 3, 3, 3, 0.2, 0.2, 0.2, 0.2]`，`tau` 固定 0。`recv_all` 的第一包超时是 200 µs。 |
| 跟随线程调度 | `FollowerControlThread::before_start` | 只打印日志，不调用基类的 `SCHED_FIFO` 50。周期落后时 `periodic_timer_thread.hpp` 会按绝对时间连续补拍（健康检查 M6 / 任务 A4）。 |
| 仿真执行 | `mujoco_teleop_hardware.cpp` `send_arm` | `kp>0` 或 `kd>0` 时直接 `qpos = cmd.q`、`qvel = cmd.dq`。默认 Kp 远大于 0，所以仿真里的手臂贴着参考走，电机阻尼不会出现。 |
| 笛卡尔模式（非默认） | `make_cartesian_config()` | 平滑器上限被设成 `1e6`，等于关掉。DLS 的 `max_joint_velocity_radps = 2.2`，`kp_pos = 9`（1/s），朝向 SLERP `alpha = 0.20`，角速度前馈增益 0。笛卡尔路径会把 `q_dot` 写入 `reference.velocity`，并且有 `max_tracking_error_rad = 0.25`。 |

`PoseKalmanFilter` 只挂在 `CartesianController` 上。它的 `predict()` 会沿内部速度积分，但 `update()` 里速度被写成「这一拍位置修正 / dt」，并不是一个可用的恒速状态。默认模式没有调用它。在修好这个状态之前，不要把现成的 `predict()` 接到 `human_mimic`。

`CUTOFF_FREQUENCY = 90`（`openarm_constants.hpp`，`diff.hpp`）用于软件微分器。跟随 MIT 的 `dq` 来自关节参考，不来自这个微分器。它不是 Grip 跟随滞后的主项。

### 1.3 延迟和抖动从哪里来

下面的时间是由常数推出来的，不是示波器结果。

**关节参考发生器是小动作滞后的主体。** `k_joint = 14` 时，小误差的一阶时间常数是 `1/14 ≈ 71 ms`（到 63%），大约 3 倍时间常数、214 ms 到 95%。速度要顶到 6 rad/s，误差得超过 `6/14 ≈ 0.43 rad`。日常小于这个角度的动作，提高 `qdot_max` 几乎不缩短滞后；提高 `k_joint` 才会。加速度上限让速度不能瞬间建立：从 0 加到 2.8 rad/s（对应 0.2 rad 误差）要 `2.8/80 = 35 ms`。方向从 +2 rad/s 收到 −2 rad/s 要 50 ms。快速回头时的「黏」主要来自这里。

肘还有第二层限速。IK 之后肘关节目标先被限制在 4.5 rad/s，再发生器以最多 6 rad/s 去追。肘 swivel 另有 2 rad/s。指尖滞后和「肘跟不上」是两件事。

**60 Hz 台阶加上 90 Hz 轮询，在滤波器之前就有大约一帧的空等。** 平均采样间隔 8.3 ms，轮询再加约 5.6 ms。控制器盖章之后才开始算那 50 ms / 150 ms 的年龄，所以界面上的 `age_ms` 比头显真实曝光更乐观。Wi-Fi 和 OpenXR 运行时自己的预测不在这份时间里，未验证。Quest 的 `performance.now` 和电脑的 `steady_clock` 也不能直接相减；可以用相邻包的 Quest 时间差估计手柄速度，那不需要钟同步。

**1000 Hz 重复 IK 是抖动来源，也是 CPU 来源。** 姿态 16.7 ms 才变一次，参考线程每 1 ms 都做最多 12 次迭代。这一拍如果超过 1 ms，周期线程补拍，而参考律自己的 `dt` 又被夹到最多 3 ms，关节参考相对墙钟变慢，下一次追上时参考速度会跳。骨骼模式把 IK 和关节参考拆开，就是为了避开这件事。

**真机上，零速度指令会让 Kd 帮忙刹车。** 近端以 2 rad/s 运动、`dq_des = 0`、`Kd = 3` 时，阻尼项量级是 6 N·m，方向与运动相反。远端 `Kd = 0.2`，同样速度下只有约 0.4 N·m。所以「肩膀发沉、手腕还行」和这组增益一致。仿真看不出这段，因为模型被写成了参考位置。未验证：达妙固件内部用的速度估计是否就是协议里的 `dq` 误差；力矩数值是协议公式乘配置，不是实测电流。

**录制会把写盘放进同一条 1000 Hz 线程。** 见第 2 节。不录的时候 `append_motor_sample()` 仍会拿锁看一眼录制器是否打开，开着的时候则格式化并写两行 CSV。

**现有环境变量可以做对照，但不要当成已经验收的默认值。** `OPENARM_HM_K_JOINT`、`OPENARM_HM_QDOT_MAX`、`OPENARM_HM_QDDOT_MAX`、`OPENARM_HM_Q_SLEW_MAX`、`OPENARM_HM_ELBOW_Q_SLEW`、`OPENARM_HM_ORI_TAU`、`OPENARM_HM_ORI_DEADBAND` 都已接到 `make_human_mimic_config()`。在 A3 的跟踪误差上限落地之前，不建议把 `qdot_max` 再往 6 rad/s 以上调。健康检查 Q5 还没决定 0.6 / 1.5 / 2 rad/s 哪一个是可接受的最大关节速度；默认遥操的发生器已经是 6 rad/s，比那三个都大。

### 1.4 按影响和风险排列的改法

| 顺序 | 改什么 | 预期 | 风险 | 规模 |
| --- | --- | --- | --- | --- |
| L1 | 接收侧使用内核到达时间（`SO_TIMESTAMPNS`），接收线程提高到 500–1000 Hz，或者改成带短超时的阻塞接收。仍只保留最新包。 | 去掉大约 0–11 ms 的轮询等待，并让 `age_ms` 表示包到达电脑的时刻。 | 低。不改控制律。和 B1 同一文件。 | S |
| L2 | `human_mimic` 的 IK 按累积时间降到 60–90 Hz；两次 IK 之间只跑 `JointReferenceGenerator`。传给斜坡的 `dt` 必须是累积时间，上限仍用 `quest_stale_sec`（50 ms）。 | 参考线程更容易留在 1 ms 以内，台阶不再每毫秒触发一次分支守卫。 | 中。若误把 `dt` 仍传 1 ms，10 rad/s 和 4.5 rad/s 的斜坡会慢一个数量级。放在 A1 之后。 | M |
| L3 | `JointReferenceGenerator` 暴露当前 `qdot`，`fill_reference()` 把它写入 `velocity`，绝对值不超过 `qdot_max`。冻结、超时、IK 失败保持速度 0。 | 真机近端不再用 Kd 对抗参考速度。仿真的 `qpos` 跟随几乎不变。 | 中。必须先有 A3 的 `|q_des-q_meas|` 上限，否则更快的跟踪会加大电机弹簧的超调。 | M |
| L4 | 在 IK 使用姿态之前，对位置做 One Euro，截止频率随速度升高；外推只用最近两帧的 Quest 时间差，水平不超过 20 ms 或 5 cm（笛卡尔模式已有 `max_position_jump_m = 0.05`）。年龄达到 50 ms 后停止外推，150 ms 仍冻结。 | 静止时压手柄噪声，快速移动时少加滞后；补上 60 Hz 台阶中间的空档。 | 中。外推放大错误包。B1 之前只对外推来自 `127.0.0.1` 的包，或先不做外推、只做 L1。不要调用现有 `PoseKalmanFilter::predict()`。 | M |
| L5 | 网页采样上限做成可配置，默认仍 60。对照实验再改到头显刷新率。桥上已有每秒 UDP 频率日志，用来确认没有在 Wi-Fi 上堆包。 | 台阶从 16.7 ms 降到头显帧周期。 | 低，但带宽未验证。`quest/` 归 B。不要在网页和 C++ 两边同时外推。 | S |
| L6 | 用日志做分段计时，见 1.6。在日志证明小动作被 6 rad/s 顶住之前，不改 `qdot_max` 的默认值。`k_joint` 的对照可以从 14 试到 25（时间常数约 40 ms），每次只改一个量。 | 避免把「调快」做成「抖动和跟踪误差变大」。 | 低，只要不把试验默认值写进仓库。 | S |
| L7 | 若 L6 的周期直方图显示 `recv_all(200)` 经常吃满、或跟随线程 `get_elapsed_time_us()` 的 p95 超过 1000 µs，再评估把 MIT 从 1000 Hz 降到 500 Hz。8 个电机、每拍命令加回包，单总线大约 1.6 万帧/秒，仲裁段仍是 1 Mbit/s。 | 只在总线确实挤占时才降命令率。 | 高，未验证前不动。和 A4 的补拍策略绑在一起看。 | L |

不采用的做法：打开 `OPENARM_HM_Q_FILT`；把笛卡尔平滑器那组 `1e6` 抄进默认模式；在超时之后继续外推；为了跟手放宽 URDF 限位；在这条线里修改 `SafetyFilter`（那是 A3）。

One Euro 的起始参数只作为试验，不是测量结果：采样按实际姿态间隔（约 60 Hz），最小截止约 1 Hz，速度系数约 0.5，微分截止约 1 Hz。朝向保持现有 6 ms 时间常数。死区 `0.05°` 先用 `OPENARM_HM_ORI_DEADBAND=0` 做 A/B，确认它是否造成微小的粘滞后再决定是否改默认值。

### 1.5 任务、文件，以及和 A1–A3、B1 怎么错开

| 任务 | 规模 | 文件 | 和健康检查的关系 |
| --- | --- | --- | --- |
| L1 到达时间戳 + 提高排空频率 | S | `src/io/quest_pose_receiver.cpp`；`control/openarm_quest_cartesian_control.cpp` 里的 `kQuestReceiveHz` 和 `QuestReceiveThread` | B1 拥有整个 `src/io/`，并且要改这个 receiver 的绑定地址。时间戳和回环绑定放在同一次改动里。`kQuestReceiveHz` 所在的 cpp 归 A。只改这个常数和接收线程的 `before_start`，不动 `SeedHoldMeasured`、退出阻尼和 `unilateral_step`。A4 若重写 `PeriodicTimerThread`，接收线程改为「醒来就排空，落后不补控制步」。 |
| L2 IK 降频 | M | `src/controller/human_mimic_controller.cpp` 的 `step()`；必要时在 `human_mimic_types.hpp` 增加 `ik_update_hz`，默认 60 或 90 | A1 拥有这个 cpp，改的是 `last_grip_` / `activate_clutch`。L2 不碰这两处，等 A1 合并后再改 IK 节拍。骨骼控制器不在 A1 范围内，本任务也不改它。 |
| L3 速度前馈 | M | `src/controller/joint_reference_generator.hpp` 增加 `qdot()` 读取；`human_mimic_controller.cpp` 的 `fill_reference()` | 发生器头文件不在 A/B 的文件清单里，可以先加只读接口。调用点等 A1 合并。A3 的跟踪误差钳位若做在 `unilateral_step()` 或参考写入之前，L3 的 `dq` 必须使用钳位后的位置所对应的速度，避免位置被夹住、速度仍指向夹住前的目标。A2 负责「CAN 不新鲜就不发带 Kp 的命令」；L3 只改参考内容，不绕过那次发送。 |
| L4 One Euro 与一帧外推 | M | 新文件 `src/controller/pose_stream_filter.hpp`（纯函数，便于无硬件测试）；调用点仍在 `human_mimic_controller.cpp` | 新文件无人认领。调用点同样等 A1。外推在 `quest_timeout_sec` 处清零，并且不得把 `require_reclutch_` 清掉。B1 未合并时，非回环来源的包不做外推。 |
| L5 网页采样上限 | S | `quest/client.html` | B 拥有 `quest/`。与 B2 的 HTTP 路径检查不在同一函数，可以并行。 |
| L6 计时日志 | S | 参考线程已有的每秒状态行（`udp=`、`age_ms=`）旁边增加 p50/p95；桥继续打印 `udp=Hz` | 日志语句在 A 拥有的 cpp 里。只加局部变量和一行输出，避免和 A2 的启动失败路径缠在一起。 |
| L7 命令率（有直方图之后） | L | `BimanualFollowerControlThread` 与 `Control::unilateral_step()` | 与 A4 同一线程函数。在 A4 确定「超时保持上一参考」之前不改发送节拍。 |

A1 的完成标准是「Grip 按住穿过超时再恢复时离合保持断开」。L2/L3/L4 的测试要沿用这个夹具：外推和降频都不能在 Grip 未松开时调用 `activate_clutch()`。健康检查 H1 的修复不放进本方案。

### 1.6 怎么测量

分三段记单调钟，单位微秒，每臂一行，先在仿真里用注入的 UDP 包跑，再在真机上复测。建议环境变量 `OPENARM_LATENCY_LOG=1`，默认关闭，避免 1000 Hz 刷盘本身制造延迟。打开时降到 100 Hz 采样，另把每秒的 p50/p95/最大值打到现有状态行。

| 时刻 | 含义 |
| --- | --- |
| `t_quest_us` | 包内现有时间戳，注明它是 `performance.now()*1000` |
| `t_kernel_ns` | `SO_TIMESTAMPNS` |
| `t_ik_us` | 本拍 IK 耗时；IK 被跳过时记 0，并记距上次求解的毫秒 |
| `t_mit_us` | `send_arm` 返回时刻；`PeriodicTimerThread::get_elapsed_time_us()` 的 p95 |
| `q_target, q_ref, qdot_ref, q_meas` | 七个臂关节。真机才有有意义的 `q_meas` 滞后 |

本地基准，不需要头显：向 `127.0.0.1:47911` 发送一段已知的正弦（例如单关节等价的 0.2 rad、0.5 Hz 位置序列，Grip=1）。记录 `t_kernel` 到 `q_ref` 越过 63% 的时间。验收看这条基准，而不是看一次主观试握。

建议的通过线（基线尚未测量，所以第一条是「先记下基线」，后面几条是改完后的相对标准）：

- L1 之后，内核时间戳到 `latest()` 可见的 p95 小于 2 ms。
- L2 之后，仿真双臂参考线程 `on_timer` 的 p95 小于 1 ms，并且 IK 调用率在 60–90 Hz，关节参考仍每拍更新。
- L3 之后，真机上 0.2 rad、0.5 Hz 的 `|q_meas - q_ref|` p95 低于改动前的基线，同时不超过 A3 最终写进测试的跟踪误差上限。仿真只检查 `dq` 字段等于发生器的 `qdot`，并且冻结时为 0。
- L4 之后，静止握持（手柄噪声）时 `q_ref` 的高频标准差下降；0.5 Hz 正弦的 63% 时间不比 L3 之后再增加 15 ms。超时 150 ms 的包仍冻结。
- 任何一步都不增加 IK 连续失败冻结的次数。失败阈值仍是 `solve_fail_freeze_count = 30`。

主观跟手只作为附加记录：同一段往返动作，仿真和真机各做一次，保存上述日志。仿真改善而真机不改善，说明剩下的项在 Kd / CAN；两者都不改善，说明剩下的项在 60 Hz 和发生器时间常数。

### 1.7 给负责人的问题

1. 现在觉得慢，是仿真、真机，还是两边都慢？仿真贴参考，真机还有 Kd。两边手感不一样是预期现象。
2. 近端 `Kd = 3` 是否要保持？速度前馈是在这个 Kd 下补上 `dq`。若希望更软，应单独改 YAML，那会影响保持刚度，不属于本方案。
3. 60 Hz 上限是为了压 Wi-Fi，还是历史值？L5 默认不动它。
4. 健康检查 Q10：超时后自动续上是否曾是操作习惯。本方案假设 A1 会改成必须松开再握。外推不依赖「自动续上」。
5. 健康检查 Q5 的最大关节速度。本方案把 6 rad/s 当作现有发生器上限，不把它再提高。

## 2. 录制和循环播放

### 2.1 现在怎么录

操作入口有两个，协议相同，都是发到 `127.0.0.1:47930` 的一行文本：

- 控制台「数据采集」：`script/openarm_app/main_window.py` 的开始 / 停止 / 成功 / 失败 / 重录，经 `script/openarm_app/services.py` `send_record()`。
- `./run.sh` 附带的网页：`script/record_panel.py`，按钮状态也是发出去就更新。

控制器在 `QuestUiBridge::handle_record_msg()` 解析：

`start` 或 `start,<批次名>`、`stop`、`success`/`fail`、`redo`、`rerecord,<NN>`、`export`、`dump`、`shutdown`。

一次成功的 Quest 遥操片段目录是 `recordings/sessions/<批次>/<两位序号>/`，里面同时有三份内容：

| 文件 | 速率和内容 | 谁在写 |
| --- | --- | --- |
| `real_teleop_split/{left,right}/real_teleop_*.csv` | Quest 遥操：每个 `unilateral_step` 一行，名义 1000 Hz。列是 `time,q,dq,tau`。写的是刚发出的 MIT 指令位置。默认模式下 `dq` 和 `tau` 来自被填成 0 的参考。 | `BimanualFollowerControlThread` → `append_motor_sample()` → `CsvRecorder::append()`。`ofstream` 没有周期 `flush`。 |
| 同路径，同向示教 / KER | 名义 200 Hz（循环 `sleep` 5 ms）。同向示教：左臂是限速后的指令，右臂是实测。KER：左右都是限速后的指令，并写 `collection_mode.json`。 | 同向示教和 KER 的 200 Hz 循环调用 `append_frame()`。 |
| `lerobot_episode.csv` | 约 30 Hz（`33 ms`）。左 8 + 右 8 的实测 `obs` 和 MIT 指令 `act`，加相对时间和 `unix`。 | 同一把 `record_mutex_` 里，两臂都到齐才写。 |
| `cameras/<name>/` | JPEG 加 `times.csv`。相机进程没起来时只有关节。 | `lerobot_cameras.py`，或已经开着的预览端口 47932。 |
| `outcome.txt` | `success` 或 `failure`。 | 停录之后的标注。标注后才会导出 OpenArm Dataset / LeRobot。 |
| `real_teleop_dual_merged.csv` | 停录时把左右 CSV 按时间对齐合并。列 `time,q0,dq0,tau0,...,q15,dq15,tau15`。速率用两边时间差的中位数，上限 1000 Hz。 | `script/merge_dual_real_recordings.py`，由停录路径调用。 |

批次名必填，序号在控制器里自增（`%02d`）。`redo` 删掉本条并退回序号。`rerecord` 可以指定以后复用某个序号。录制中或待标注时，数据管理不允许改当前批次。MoveIt 和零重力的界面直接不提供录制。

控制台在 `send_record()` 返回后就改按钮：开始即认为在录，停止即进入待标注。控制器若没在听、批次目录创建失败、或上一笔还没标注，界面和进程会不一致。控制器的失败信息只出现在子进程日志里，界面没有对应的回执解析。`record_panel.py` 同样是发出去就改按钮。

`CsvRecorder::append()` 用默认格式写 `double`，停录时 `close()` 才关闭文件。进程被强杀时，缓冲里最后一小段可能没落盘。写盘发生在跟随线程里，并且持有 `record_mutex_`；界面线程处理 start/stop 时也拿这把锁。这是跟手和录制可靠性叠在一起的点。

60 秒、1000 Hz、双臂、每行大约 25 个数，分臂 CSV 大约几十 MB，合并文件再一份。回放前会整文件读进内存。这个量级可以工作，但和 200 Hz 的回放伺服相比，多出来的样本只是在重复一条已经平滑过的指令。

回放使用的是合并后的 **指令位置**。`TeachingTrajectory::sample()` 用位置差分重算速度，忽略文件里的 `dq`（`tests/test_teaching_replay.py` 用 `dq=999` 锁住了这一点）。因此 L3 即使以后把真实 `dq` 写进 CSV，现有回放也不会把记录的速度下发。30 Hz 的 `lerobot_episode.csv` 是数据集导出的源，回放窗口不读它。Quest 遥操回放出来的是「当时发给电机的平滑指令」，其中已经包含发生器的滞后；同向示教的右臂则是实测拖动。

### 2.2 现在怎么放

回放是另一个进程，和遥操互斥。控制台要求先「停止并释放」，再打开「动作回放…」或数据管理里的「回放此条」。窗口是 `script/openarm_app/replay_dialog.py`，它调用 `script/replay_teaching.py`，后者每次都 `cmake` 配置并编译 `openarm_teaching_replay`，再执行 `control/openarm_teaching_replay.cpp`。

播放状态机在 `openarm_teaching_replay.cpp`，限速在 `src/controller/teaching_replay.hpp` 的 `TeachingReplayReference`：

| 阶段 | 行为 | 数字 |
| --- | --- | --- |
| 检查 | 列、有限值、严格递增时间、URDF 限位。臂关节允许 `1e-6 rad` 的六位小数舍入，超过则整段拒绝。 | 至少两帧。夹爪按所选单位检查：真机 −0.98～0.62 rad，仿真 0～0.044 m。 |
| 接入 | 从当前实测位置走到首帧。指令速度前馈为 0。 | 0.3 rad/s，0.6 rad/s²。参考误差 `< 0.015` rad 且实测误差 `< 0.12` rad 后进入播放。超过 120 s 报错退出。 |
| 播放 | 轨迹时间按 `dt * speed` 前进。误差 `> 0.03` rad 时冻结轨迹时间，等参考追上。 | 七个臂关节 0.6 rad/s、1.2 rad/s²；夹爪 1 rad/s、4 rad/s²。位置误差反馈增益在播放时为 8（1/s），接入时为 2。 |
| 暂停 / 继续 | 标准输入 `pause` / `resume`。时间停住。 | 窗口把这两个词写进子进程标准输入。 |
| 完成 | 末帧误差同样低于 0.015 / 0.12，并稳住 0.3 s。然后一直保持末帧，直到停止。 | 进度条停在末尾。 |
| 停止 | 窗口发的是 `SIGINT`，不是标准输入 `stop`。循环退出后 `MirrorTeachingArm::stop()` 直接 `disable_all()`。5 s 后仍未退出再 `SIGTERM`。 | 仿真里 `disable_all()` 是空函数。真机立即失能，没有遥操那条默认 6 s 阻尼（健康检查 M4）。操作说明要求人托住手臂。 |
| 伺服周期 | `sleep_until` 5 ms，名义 200 Hz。`dt > 0.05` s 直接抛错退出。 | 仿真若在同一循环里渲染过久，会触发这条例外。未在本环境跑过。 |

速度：窗口 0.25 / 0.5 / 1 倍，默认 0.5。命令行 `--speed` 允许 0.1～1.0。倍率同时缩放轨迹时间和差分速度，但仍然被上面的 0.6 rad/s 卡住，所以 1 倍只是上限，文件里更快的动作会被拉长。这是现有文档已经写明的行为，循环播放保持它。

真机两边都走 `MirrorTeachingArm`：启动时 1 s 内等齐 8 个电机，运行中年龄超过 150 ms 或 `error_code > 1` 就抛错并在析构里失能。单关节指令与实测之差超过 0.45 rad（夹爪 0.2）也会抛错。实测速度从 0.8 rad/s 开始衰减驱动力，到 2 rad/s 方向的保护见 `mirror_speed_guard.hpp`。重力按当前 `gravity_comp.yaml` 重算，记录的力矩不下发。

仿真走同一个 `TeachingReplayReference` 和同一套限速，但 `MujocoTeleopHardware::send_arm` 在 `kp>0` 时把 `qpos` 设成指令。仿真能核对状态机、插值和限位，不能核对力矩、CAN 年龄和 0.45 rad 保护。默认场景仍是 `run.sh` 那条机器相关的 `OPENARM_MUJOCO_XML`（健康检查 M1）。回放脚本里的默认 XML 也是这条外部路径。

`CsvReplay`（`src/io/teleop_io.cpp`）是另一条播放器，只给真机启动用：当前姿态用 S 曲线接到 `assets/steady_action_on_table.csv` 的首帧，播完保持，然后才允许 Grip。它没有循环，也没有接到「动作回放」窗口。循环播放不要改这条启动路径。

### 2.3 循环播放要补上的行为

目标是：同一条已录轨迹可以按次数或一直重复，仿真和真机的阶段一致，开始和每一圈的衔接都不跳变，停止时先停稳再失能。

状态在现有接入 / 播放 / 暂停 / 完成之外增加「接缝」和「停稳」：

1. **开始。** 保持今天的接入：从实测位置以 0.3 rad/s、0.6 rad/s² 走到首帧，条件仍是参考误差 0.015 rad、实测误差 0.12 rad。
2. **播放。** 时间、倍率和「误差大于 0.03 rad 就冻结时间」保持不变。
3. **接缝。** 播放到末帧并满足完成条件后，若还剩循环次数，把目标换成首帧，进入与接入相同的限速（0.3 rad/s、0.6 rad/s²，速度前馈为 0）。到达首帧后再把轨迹时间清零，次数减一，回到播放。末帧和首帧的最大关节差小于 0.05 rad 时，可以跳过长时间接缝，直接进入下一圈，但单拍指令增量仍不得超过播放加速度乘 `dt`。
4. **次数。** `--loops N`，`N>=1` 表示总共播放 N 遍。`N=0` 表示无限。窗口用次数框加「无限循环」勾选。默认 1，也就是今天的行为。
5. **无限循环的真机上限。** 真机无限循环必须同时勾选现有的 CAN / 急停确认，并且默认墙钟不超过 10 分钟（`--max-seconds` 可改）。到时走停稳，不突然失能。仿真的无限循环可以不设这 10 分钟，方便看接缝。
6. **暂停。** 在播放和接缝中都停住轨迹时间或接缝进度，指令保持当前位置，速度收到 0（用当前阶段的加速度上限）。继续时从停住的地方走，不回到首帧。
7. **停止。** 窗口先写标准输入 `stop`。控制器进入停稳：目标冻结在当前参考，速度按 1.2 rad/s²（夹爪 4 rad/s²）收到 0，参考速度低于 0.02 rad/s 并保持 0.3 s 后，再调用现有的 `disable_all()`。第二次停止或 5 s 超时仍走今天的 `SIGINT` / `SIGTERM`，避免停稳卡死时手臂一直带力矩。仿真的 `disable_all()` 仍是空操作，停稳阶段仍要跑完，这样两边的日志阶段一致。
8. **进度。** 日志在现有 `PROGRESS` 上增加 `loop=` 和 `loops=`。窗口显示「第 k / N 圈」；无限时显示第 k 圈，进度条在单圈内循环。完成一句改为「已完成 N 圈，保持末帧」。

速度仍只有 0.25 / 0.5 / 1，命令行仍是 0.1～1.0。接缝不乘播放倍率，始终用接入那组更慢的限制，避免 1 倍循环时从末帧跳回首帧。

仿真和真机的一致性指：同一份 CSV、同一倍率、同一循环次数，两边的阶段顺序和指令限速相同。比较对象是回放进程打出的参考位置日志，不是 MuJoCo 画面和真机电流。真机额外保持 CAN 150 ms、`error_code > 1`、0.45 rad 跟踪误差和速度守卫。接缝如果让跟踪误差超过 0.45 rad，应当像今天一样退出并失能，而不是把阈值放大。

回放源继续是 `real_teleop_dual_merged.csv`（或现场合并的分臂 CSV）。第一阶段不改成播放 30 Hz 实测 `obs`。指令回放和「手臂实际走过的路」之间的差别写进窗口说明里。若以后要播实测，作为单独任务 R6，并且仍然经过同一套接入、接缝和限速。

### 2.4 录制操作上要补的行为

这些改动让循环播放的输入更完整，也避免录制伤害跟手。

- **回执。** 控制器已有 `[quest-ui] recording segment`、`recording stopped`、`labeled`、`failed to open`。控制台和 `record_panel.py` 解析这些行再改按钮。UDP 发出后按钮进入「等待控制器」；超时约 1 s 没有回执则恢复原状态并显示日志里的错误。
- **写盘。** 跟随线程只把样本推进一个有界队列（例如 256 帧，满了丢弃最旧并记一次告警）。单独线程写 CSV，每 100 ms `flush`。Quest 遥操的分臂指令文件改为 200 Hz，与回放伺服和同向示教记录一致；`lerobot_episode.csv` 保持约 30 Hz。队列丢样时在片段目录写 `record_warnings.txt`，回放前检查脚本若看到它就在窗口里提示，不因此拒绝播放。
- **名字和片段。** 批次名仍由人填，序号仍是两位。停止后的提示里带上序号、时长和帧数（控制器在 `recording stopped` 那一行带上即可）。成功 / 失败 / 重录的语义不变。
- **可靠性。** 停录时先关队列、再合并。合并失败要在日志里写明，并且不要打印 `replay-ready`。现在窗口会从日志里找这一行来预填路径。
- **构建。** `replay_teaching.py` 在可执行文件比源文件新时跳过 `cmake --build`。现在每次打开回放都配置并编译，和播放逻辑无关，但会让「点开始」等很久。检查模式 `--check` 保持不动。

### 2.5 任务、文件，以及和安全改动的边界

| 任务 | 规模 | 文件 | 和 A1–A3、B1 的关系 |
| --- | --- | --- | --- |
| R1 循环状态机 | M | `src/controller/teaching_replay.hpp` 增加接缝/停稳所需的只读查询（当前参考、是否已到位）；`control/openarm_teaching_replay.cpp` 增加 `--loops`、`--max-seconds` 和阶段 | 这两个文件不在 A/B 清单。不要改 `mirror_teaching_arm.hpp`。A2 若把 CAN 检查从同向示教抽成公共函数，回放继续调用 `MirrorTeachingArm` 的现有检查，避免两处同时改 150 ms / `error_code > 1`。 |
| R2 窗口和命令行 | S | `script/openarm_app/replay_dialog.py`；`script/replay_teaching.py`；`tests/test_replay_dialog.py`、`tests/test_teaching_replay.py` | 纯界面和参数转发。停止改为先写 `stop`，超时再发信号。 |
| R3 录制回执 | S | `script/openarm_app/main_window.py`；`script/record_panel.py` | 不改 C++ 协议，只消费已有日志。可以和 A 线并行。 |
| R4 写盘移出跟随线程，指令 CSV 改为 200 Hz | M | `control/openarm_quest_cartesian_control.cpp` 的 `append_motor_sample()` 和 `BimanualFollowerControlThread::on_timer()`；`src/io/teleop_io.cpp` 的 `CsvRecorder` 增加 `flush` | 跟随线程的函数和 A4 重叠：A4 改周期落后时的行为，R4 删掉这条线程上的文件 IO。约定 A4 不重排 `append_motor_sample` 的调用位置，R4 不改 `clock_nanosleep`。`src/io/` 归 B，但 B1 的具体文件是姿态 receiver。R4 只改 `teleop_io.cpp` 的 recorder，不改 `quest_pose_receiver.cpp`。 |
| R5 跳过重复编译 | S | `script/replay_teaching.py` | 无安全交叉。默认 XML 仍尊重 `OPENARM_MUJOCO_XML`；是否改成仓库内场景属于健康检查 C2 / Q6，本方案不改默认路径。 |
| R6 可选：回放 30 Hz 实测 | L | `openarm_teaching_replay.cpp` 的加载；数据集列定义在 `script/lerobot_collect.py` | 放到 R1 稳定之后。实测轨迹更抖，仍必须经过 0.3 rad/s 接入和 0.6 rad/s 播放上限。不在本阶段实现。 |

R4 会让新录的分臂 CSV 从 1000 Hz 变成 200 Hz。已有片段仍可回放，因为播放器按时间戳插值。合并脚本的 `inferred_hz()` 会跟着新文件变成约 200。需要在 `collection_mode.json` 或停录日志里写上 `command_log_hz`，避免以后把两种文件当成同一采样率的学习数据。30 Hz 的 LeRobot 导出不受这条影响。

### 2.6 测试和验收

已有 `tests/test_teaching_replay.py` 用一段临时 C++ 检查插值、忽略记录速度、限位和 0.6 rad/s。循环播放沿这个做法加断言，不需要 CAN：

- 末帧与首帧相差 0.4 rad 时，接缝阶段每 5 ms 的速度增量不超过 `0.6 * 0.005`，速度绝对值不超过 0.3 rad/s，位置连续。
- `--loops 3` 时，轨迹时间归零三次，然后进入保持末帧。
- `stop` 之后、失能标志之前，指令速度降到 0.02 rad/s 以下，位置相对停止瞬间的变化不超过停稳阶段允许的减速距离。
- 播放中关节误差大于 0.03 rad 时，循环时间不前进。
- 非法 `--loops`、非有限 CSV、超限位帧仍在使能前拒绝。

界面测试沿 `tests/test_replay_dialog.py`：默认次数为 1；无限循环在真机模式下若未勾选安全确认则不能开始；停止按钮的第一次动作是写入 `stop`。

仿真验收（有显示和 `OPENARM_MUJOCO_XML` 时，本次未跑）：

```bash
.venv/bin/python script/replay_teaching.py recordings/sessions/<批次>/<序号> --sim --speed 0.5 --loops 3
```

日志里依次出现接入、播放、接缝、播放，共三圈，然后保持末帧。再用标准输入 `stop` 看到停稳之后的 `STOPPED`。同一条命令加 `--check` 不使能、不打开窗口。

真机验收放在仿真通过之后，人在旁边，硬件急停可用：先 `--real --check`，再 `--real --speed 0.25 --loops 2`。CAN 年龄和 0.45 rad 保护保持有效。无限循环要带 `--max-seconds`，并确认到时会停稳而不是继续。

录制验收用一次性目录：开始后看到控制器的 `recording segment` 界面才显示在录；拔掉或停掉控制器时界面在约 1 s 内回到可开始；停录后分臂 CSV 的中位间隔约 5 ms，`lerobot_episode.csv` 仍约 33 ms；录制过程中跟随线程耗时的 p95 相对未录制时的增加小于 200 µs（这条依赖 L6 的计时，未录制基线也还没有）。

### 2.7 给负责人的问题

1. 循环播放要看的是「当时的电机指令」，还是「手臂实际到达的姿态」？本方案默认继续播指令，因为现有文件和测试都是这条。实测回放是 R6。
2. 真机是否允许无限循环？本方案要求安全确认，并且默认 10 分钟封顶。
3. 新录的指令 CSV 从 1000 Hz 改成 200 Hz，是否会影响已有的离线脚本？合并和回放按时间戳工作，LeRobot 仍是 30 Hz。若有脚本把行数当成 1000 Hz，需要一起改。
4. 停止时，回放是否保持「立即失能、人托住手臂」，只在失能前先把指令速度收到 0？本方案按这个做，不把遥操的 6 s 阻尼搬进回放。

## 3. 建议的落地顺序

可以马上做、且几乎不碰 A/B 文件的是：R1、R2、R3、R5，以及 L6 里那些只加日志的语句（若 A 线已经占用该 cpp，就把日志推迟到 A4 的线程改动合并之后）。

L1 和 B1 合成一次 `quest_pose_receiver.cpp` 的改动。L2、L3、L4 等 A1 合并；L3 还等 A3 的跟踪误差上限有了测试里的具体弧度。L5 可以和 B2 并行。R4 等 A4 对跟随线程的改动范围定下来再动 `on_timer`。L7 等 L6 的直方图。

本文件是设计，不包含上述任务的实现。

## 4. 这次没有验证的事项

- 没有测量 Quest 到电脑的 Wi-Fi 时间，也没有确认头显实际刷新率。60 Hz 是网页里的上限，不是头显的测量值。
- 没有确认达妙 MIT 的 `Kd*(dq_des-dq)` 在固件里的速度定义，6 N·m 是 `3.0 * 2 rad/s` 的协议侧估算。
- 没有测量 1000 Hz 线程的超时比例，也没有数过 CAN FD 上的实际帧率。1.6 万帧/秒是按 8 个电机、每拍一发一收估算的。
- 没有单步运行 H1 的重新握持。本方案把那次修复完全留给 A1。
- 没有运行 `unittest`，没有打开 MuJoCo 或 Qt，因此循环播放的交互只按现有测试的写法设计，还没有新的测试代码。
- `PoseKalmanFilter` 的速度状态是阅读 `update()` 得到的结论，没有数值对照试验。
