# 右臂 J7 回放无力：调查、对齐与诊断采集

用户确认：只测试右臂，遥操正常，实机回放无力；壶重量未知。

## 代码调查与处理

旧回放与遥操并没有使用同一种 MIT 控制策略。已将 `ReplayArm::send` 改为直接复用遥操 `Control::unilateral_step`，并抽取共享 `follower_config.hpp`，不是另写一份相同数值。

| 项目 | 旧实机回放 | 对齐后与遥操共用 |
|---|---|---|
| J7 Kp / Kd | 8 / 0.3 | 默认 25 / 0.2，读取 follower.yaml |
| J1–J7 Kp | 22/22/16/16/8/8/8 | 默认 240/240/240/240/24/31/25 |
| 重力前馈倍率 | ff_gain，默认 0.92 | tracking_ff_gain，默认 1.08 |
| 重力模型 | 夹爪质量有独立 fallback | 同一个安装状态、质量、COM、相机和末端模型配置函数 |
| 摩擦前馈 | 零重力示教摩擦策略 | 与遥操相同，只发送重力前馈 |
| 力矩限制 | PD+FF 估算总和腕部曾限 2，后调至 3 Nm，通过负 FF 抵消 PD | 与遥操相同：限制重力 FF，腕部 2 Nm；PD 正常叠加，不是总力矩 2 Nm |
| 夹爪 | 独立 6/0.45，总力矩 1 Nm | 共用默认 10/0.45、左右覆盖、接触保持/释放与滑移逻辑 |
| 环境变量 | 部分无效 | 共用 MIT_SCALE、GRAVITY_FF/GAIN/RAMP、GRIPPER_KP/KD 及左右覆盖 |
| 安全过滤 | 独立回放控制 | 共用 Control 安全过滤、锁存停止及 CAN 反馈检查 |

同为 6° 位置误差时，J7 位置项由约 0.84 Nm 变为 2.62 Nm。单纯提高总力矩上限不会改变旧 Kp=8 时的支撑力，解释了为什么上限调大后仍可能偏软。这是确定的代码差异；还不能证明某次实机无力完全由它造成。

“对齐”指底层控制策略及参数来源。输入任务仍不同：遥操由 IK 产生目标，回放由录制轨迹产生目标。回放仍有起手轨迹、限速/限加速度、暂停和进度机制；名义更新频率仍为 200 Hz，遥操为 1000 Hz。MIT 电机在内部执行位置反馈，所以共用参数不保证两种模式轨迹/负载响应完全一致。回放实测速度超过对应阈值（J1–J6 为 2 rad/s，J7 为 3 rad/s；左右一致）、跟踪误差超过 0.45 rad、无效目标或过期 CAN 仍停止。原回放 0.8 rad/s 起的扭矩软化已由共用 follower 策略替换；不能沿用旧 3 Nm 制动模型结果作为新版的物理验收。

## 已有数据分析

命令：

```bash
python3 script/diagnose_j7.py legacy recordings/sessions/session_20261012_155617/01
```

该历史遥操记录包含 1221 个观测/动作配对，右 J7 位置误差 p95 约 5.93°，最大约 6.77°。它没有对应实机回放逐关节反馈、确切历史增益或载荷重量，因此只能量化录制中的误差，不能反推出回放当时是否饱和、过温或驱动掉线。此前审核见 `REPLAY_WRIST_LOAD_AUDIT.md`，其中 2/3 Nm 为历史策略。

## 已埋入的自动采集

重新启动新版遥操或实机回放即可自动采集，默认目录 `recordings/control_diagnostics/run_<pid>/`。左右臂各一份 CSV。只在实际 CAN 后端启用，不启动任何额外动作。

每轴记录单调/墙钟时间、实际循环间隔、阶段与回放时间、轨迹源目标、实际 MIT q/dq、反馈 q/dq/扭矩、Kp/Kd、原始重力、标定后的重力请求、实际发送 FF、FF 限幅、协议编码范围、CAN 反馈年龄/状态、MOS/转子温度、PD 与估计总力矩。100 Hz 采样，固定队列，控制线程 try-lock；磁盘写入和格式化在后台，忙时丢诊断帧，正常退出写 `.stats`。配置和 URDF 自动快照，`.settings` 记录关键环境覆盖。异常退出可能没有尾部数据或最终丢帧计数；停止原因同时查进程日志。超时/故障会在发送前停止，因此最后的正常 CSV 样本不一定包含故障帧。

反馈扭矩来自电机估计，不是末端力传感器。`total_estimate` 是利用采样反馈计算的理想 MIT PD+FF，存在 CAN 延迟、固件内部更新与编码量化差异，不能称为实测输出力矩。`protocol_limit` 是报文编码范围，不是电机连续安全额定扭矩。

可在启动应用的同一个终端准备独立对照目录（命令只复制配置、输出环境设置，不启动电机）：

```bash
python3 script/diagnose_j7.py prepare --label '右臂 回放 壶重量待填写' --source recordings/sessions/session_20261012_155617/01/real_teleop_dual_merged.csv
# 执行输出的两行 export，再从此终端正常启动 ./app.sh
```

应用已在运行时，终端 export 不会改变它继承的环境；使用默认自动采集目录即可。关闭采集：`OPENARM_CONTROL_DIAGNOSTICS=0`。

采集后离线分析：

```bash
python3 script/diagnose_j7.py analyze recordings/control_diagnostics/run_<实际PID> --output /tmp/j7-report.json
```

报告分开统计播放阶段与全程，区分轨迹过滤延迟、机械跟踪误差、低速持续误差、FF 限幅、协议范围、反馈异常、循环延迟、温度与丢帧。单次静态误差不能区分模型误差和摩擦；需要同姿态、同轨迹、同增益的空载/带载对照，并记录壶装好后的质量。实际负载重量及 COM 尚未确定，本次没有猜测它们或自动改动重力标定。

## 成熟开源方案的参考

- [Unitree xr_teleoperate 的 arm controller](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm.py) 为腕部设置专用阻抗参数，并对关节参考速度做限制；[IK 源码](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py) 用 Pinocchio RNEA 产生前馈。这里借鉴“位置反馈＋模型前馈”的分工，不复制宇树硬件增益或把它的目标姿态重力求值方式直接替换本项目已工作的测量姿态策略。
- [libfranka 关节阻抗例程](https://frankarobotics.github.io/libfranka/0.15.0/joint_impedance_control_8cpp-example.html) 比较指令与反馈扭矩，用单独线程打印，避免阻塞控制回路；本次采用相同的采集原则。Franka 自身的重力补偿边界与 OpenArm 不同，不能直接照搬其扭矩公式。
- [libfranka setLoad](https://frankarobotics.github.io/libfranka/latest/classfranka_1_1Robot.html) 独立配置载荷质量、质心和惯量。这说明负载建模与刚度设置是两个问题；更硬的闭环不能替代实际载荷参数的核实。

## 验证边界

测试覆盖实际 ReplayArm::send 与遥操配置在相同反馈/参考下输出完全一致 MIT 帧、双臂完整重力渐入、超速不再发送指令，以及 1 Nm 恒定负载的简化 J7 动力学模型。模型 I=0.03 kg·m²、附加粘性阻尼 0.4 Nm·s/rad，200 Hz 指令、内部 4 kHz 积分，期望稳态误差 1/25=0.04 rad；该简化模型不包含真实驱动/CAN 延迟或壶晃动。另有后台写入/丢帧计数及诊断分类测试。

本次未连接 CAN 或运行实臂。重载物理效果、温升和真实时序必须由新采集验证。

验证命令与结果：

```bash
cmake --build build --target openarm_teaching_replay quest_cartesian_control teleop_gravity_test -j 4
python3 -m unittest discover -s tests -p 'test_replay_*.py' -v
QT_QPA_PLATFORM=offscreen .venv/bin/python -m unittest discover -s tests -p test_replay_dialog.py -v
python3 -m unittest discover -s tests -p test_teaching_replay.py -v
python3 -m unittest discover -s tests -p test_teleop_gravity.py -v
python3 -m unittest discover -s tests -p test_control_diagnostics.py -v
python3 -m unittest discover -s tests -p test_gripper_follow.py -v
python3 -m unittest discover -s tests -p test_safety_filter_contract.py -v
git diff --check
```

三个目标编译成功。系统 Python 的回放检查实际执行 16 项、跳过 13 项 Qt 检查；随后使用项目 `.venv` 补跑 13 项 Qt 检查全部通过。轨迹 4、控制/重力 1（含双臂控制对齐、负载模型、停止等 C++ 子场景）、采集诊断 2、夹爪 1、安全过滤 3 项通过，合计 40 项通过。仿真预览覆盖速度、暂停、拖拽、起手路径、等待播放和循环；空白检查通过。`diagnose_j7.py prepare` 已在 `/tmp` 使用真实旧录制验证配置快照，未启动硬件。

## 起手超速的后续修复（2026-10-12）

`/tmp/openarm_teaching_replay_20261012_173138_226734_106219.log` 显示右 J7 在起手阶段实测 −3.02564 rad/s，超过已放宽到 3 rad/s 的停止阈值。只提高阈值没有解决高速段的跟踪超调。共享 `StartupTrajectory` 现对高速段局部重定时，按停止阈值的 65% 留指令速度余量，平滑过渡到原速度，保留全部位置。原起手文件本体 6.433 秒调整为约 6.957 秒。遥操和回放使用同一处理，硬停止阈值不继续提高。`test_startup_speed_retiming.py` 使用实际起手文件、200 Hz 指令及简化惯性模型复现原始超速，并检查修正后峰值、起末位置、速度余量和源文件不变；不构成真机安全保证。
