# 仿真与真机共用起手姿势

当前已实现仿真遥操起手：`run.sh --sim` 默认启用与真机相同的完整 CSV 启动流程，从 MuJoCo 当前姿态平滑接入文件第一帧，按原时间戳播放所有录制轨迹，保持末帧，然后重新播种遥操参考并开放 Grip 与录制。仿真夹爪按真机角度转成米，窗口在过渡和轨迹期间持续渲染；中途停止不会跳到终点。下文其余内容是原方案和历史状态，停止回位等内容不代表已实现。

> 本文是历史诊断/规划记录。同向示教功能已移除；涉及该功能的段落不再描述当前代码。


> 本文保留原分支的诊断与规划快照，部分现状已被整合版本更新；本次已实现和已验证的范围见 [整合记录](INTEGRATION_20261012.md)。规划条目不代表已经实现。

日期：2026-10-12。本文只读 `main`（`73dc004`）后给出设计，不改控制、模型或界面行为。安全边界以 [PR #1](https://github.com/DucksAreRoasted/openarm-vr-teleop/pull/1) 的 `docs/HEALTH_CHECK.md` 为准，跟手与循环回放以 [PR #2](https://github.com/DucksAreRoasted/openarm-vr-teleop/pull/2) 的 `docs/TELEOP_RECORDING_PLAN.md` 为准。下文用 A2、A3、A4 指健康检查里的任务编号，用 R1 指录制方案里的回放状态机。

本次没有编译、没有打开 MuJoCo、没有连接 Quest、CAN 或真机。CSV 里的关节数是对 `assets/steady_action_on_table.csv` 做的本地统计。凡是桌面是否真的托住手臂、达妙掉线后电机怎么反应，都标了「未验证」。

默认遥操模式是 `human_mimic`。`cartesian_precision`、`skeleton_proximal`、`pika_style` 和它走同一条启动与退出（`control/openarm_quest_cartesian_control.cpp` 的 `main`）。同向示教、零重力、MoveIt 在进入这条路径之前就返回，不在本方案的实现范围内，只在第 6 节说明它们今天的仿真初始姿态。

## 0. 优先建议

1. **起手姿势只用一份定义：启动文件的最后一帧。** 真机遥操接受 Grip 之前，握住的是 `assets/steady_action_on_table.csv` 播完后的姿态，不是 `TABLE_READY_POSITION`。仓库里的 MuJoCo XML 没有 keyframe，也没有 `qpos`。仿真今天用 `set_home` 瞬移到另一组数。
2. **平滑来自指令序列，不能靠 MuJoCo 的关节弹簧。** `MujocoTeleopHardware::send_arm` 在 `kp>0` 或 `kd>0` 时直接把 `qpos` 写成 `cmd.q`。跟随增益来自 `config/follower.yaml`，近端 Kp 为 240，所以仿真手臂贴着参考走。参考跳一帧，画面就瞬移一帧。起手和回位都要先把目标拆成 S 曲线再写入参考。
3. **停止遥操和停止回放都先回到这份起手姿势，再释放。** 速率用现有 S 曲线默认值 0.6 rad/s、1.2 rad/s²、6 rad/s³（`src/controller/s_curve.hpp`）。回位超时默认 15 s。日志沿用控制与安全工程师已经定下的字段：`phase=returning`、`return_left=`、`DISABLE reason=return_timeout`。
4. **输出已经被关掉时不要再规划回位。** [PR #4](https://github.com/DucksAreRoasted/openarm-vr-teleop/pull/4) 在 CAN 不新鲜或 `SafetyFilter` 锁存停止后会跳过约 6 秒阻尼。回位必须插在那次判断之前：已经 `OutputDisabled()` 就直接 `Shutdown()` / `disable_all()`。仿真不绑 CAN，仍然把同一条参考走完。
5. **控制台按这些日志改状态，并且把等待时间放到 15 秒之后。** 今天主界面在进程刚拉起时就写「仿真运行中」，停止后 8 秒发 `SIGTERM`；回放窗口 5 秒发 `SIGTERM`，而且 `phase=` 用字典硬查。新阶段若先于界面合入，回放窗口会在 `KeyError` 上停住日志解析。

## 1. 真机现在怎么起步，起手姿势到底是什么

`./run.sh --real` 在非同向 时默认加上启动运动（`run.sh` 约 152–168 行）：

| 环境变量 | 默认 | 作用 |
| --- | --- | --- |
| `OPENARM_QUEST_STARTUP_MOVE` | `1` | `0` 时不传 `--startup-move` |
| `OPENARM_QUEST_STARTUP_FILE` | `assets/steady_action_on_table.csv` | 双臂合并 CSV |
| `OPENARM_QUEST_STARTUP_SEC` | `4` | 只给「没有时间戳的航点文件」用 |
| `OPENARM_QUEST_STARTUP_TRANSITION_SEC` | `5` | 当前姿态到 CSV 第一帧的时长 |

仿真分支不传这些参数。`README.md` 写的「VR 真机遥操默认先沿该文件进入桌面姿态」指的是这条路径。

文件是双臂录制格式：表头 `time,q0,dq0,tau0,…,q15,dq15,tau15`，共 49 列。数据 6434 行，时间 0 到 6.433 s，中位间隔 0.001 s。左臂是 `q0`–`q7`，右臂是 `q8`–`q15`，每臂 7 个关节加夹爪。它不是静止姿态，肘（每臂第 4 个数，关节 4）从大约 0.14 / 0.19 rad 走到大约 1.94 / 2.00 rad。

`run_startup_dual_csv()` 的顺序是：

1. 用 `CsvReplay::prepend_transition_from_with_result()` 从当前测量姿态接到第一帧。测量速度先被清零。`--startup-transition-sec 5` 把这段压成或拉成 5 秒（函数内部再夹到 0.05–10 s）。静止到静止时用 `s_curve.hpp` 的 0.6 rad/s、1.2 rad/s²、6 rad/s³，再按这段时长缩放。
2. 以 500 Hz 播放文件里的位置。播放器忽略记录的 `dq` 做不到限速；参考位置就是文件里的 `q`。
3. 播完后把速度写成 0，按 `OPENARM_QUEST_STARTUP_HOLD_SEC`（默认 0.6 s，上限 5 s）保持最后一帧。
4. `reseed_teleop_after_startup()` 用保持之后的测量值重新播种遥操和 `SafetyFilter`。在此之前 Grip 线程还没启动。

因此真机接受 Grip 时的起手姿势是 **CSV 最后一帧**，不是第一帧，也不是下面的 `TABLE_READY`。最后一帧（弧度，夹爪也是文件里的原值）：

| 臂 | q1…q7 | 夹爪 |
| --- | --- | --- |
| 左 | 0.221447, −0.412184, −0.039101, 1.942660, −0.616719, −0.384718, −0.093271 | 0.036812 |
| 右 | −0.326352, 0.692519, 0.515946, 1.995310, 1.172030, 0.228313, −0.560197 | 0.040246 |

第一帧与最后一帧的最大关节差约 1.81 rad（两臂都是）。对照 `models/OpenArm_V1/openarm_bimanual.xml` 里各 `joint` 的 `range`，最后一帧的 14 个臂关节都在限位内。这次对照是读 XML 和 CSV，没有调用 `Dynamics::GetJointPositionLimits`，也没有做桌面碰撞。

文件在 1.551 s 附近有单拍速度尖峰，最大约 13.2 rad/s（左关节 4，一毫秒大约 0.013 rad）。超过 2 rad/s 的拍有 366 个。末尾 0.3 s 的最大位置差分仍约 0.77 rad/s，并不是数值上的完全静止。真机上这些尖峰交给 Kp 弹簧，电机跟不到 13 rad/s；仿真若把同一参考直接写入 `qpos`，会一拍不差地跟上。起手姿势（最后一帧）两边仍然相同。本方案不改这段已有播放，避免真机启动轨迹被第二次限速拉长。

`TABLE_READY_POSITION`（`src/openarm_constants.hpp`）是另一组数：`{0, 0, 0, 1.570796, 0, 0, 0, 0}`。注释写它对应仓库外的 `openarm_mujoco/v2/cell.xml` keyframe `home`。它和 CSV 第一帧的最大差约 1.43 rad（左）、1.38 rad（右），和最后一帧的最大差约 0.62 rad（左）、1.17 rad（右）。它还被用作：

- 仿真 `prime_sim_arm()` 的瞬移目标
- `make_human_mimic_config()` / 笛卡尔零空间的 `q_nominal`
- 没有启动文件时 `make_startup_waypoints()` 的单点目标
- 同向示教 仿真窗口里的 `set_home`

`HumanMimicController::ramp_arm_reset()` 会以 `reset_speed`（默认 1 rad/s）把关节目标拉向 `q_nominal`。默认 Trigger 是扭转模式，这条斜坡只在 `trigger_twist_mode=false` 时运行。它不是进程停止，也不要把它当成起手姿势。

关节↔电机转换是恒等的（`OpenArmJointConverter`）。夹爪注释写明：仿真指尖单位是米（大约 0 开、0.04 合），真机是电机弧度。启动 CSV 把夹爪原值塞进同一条参考。真机上 0.037 rad 是小角度；仿真若原样写入指尖 `qpos`，0.037 m 会接近 `openarm_*_finger_joint*` 的上限 0.045 m。回放器 `TeachingTrajectory::convert_grippers()` 会在「源为弧度、目标为仿真」时把 `[-0.98, 0.62]` 线性映射到 `[0, 0.044]`。0.037 rad 按这个比例大约是 0.028 m。启动路径没有做这个映射。夹爪外观是否和真机一致，未在画面上验证。

## 2. 仿真现在怎么起步、怎么停

`models/OpenArm_V1/openarm.xml`、`openarm_bimanual.xml`、`scene.xml`、`scene_table.xml` 都没有 `<keyframe>`，也没有 `qpos="..."`。关节上的 `ref="0.0"` 是弹簧参考，不是初始角。`scene_table.xml` 的注释写明：桌面姿态靠控制器 `set_home`，不靠 keyframe。MuJoCo 因此从全零 `qpos` 开始，直到控制器改写。

`run.sh` 默认场景仍是仓库外的 `/home/ducks/motion-trajectory-studio/model_library/OpenArm_V1/scene_table.xml`（健康检查 M1）。仓库内对应文件是 `models/OpenArm_V1/scene_table.xml`。本方案不改默认路径。

`prime_sim_arm()` 在 `sim_table_ready` 为真时（默认真，可用 `--no-sim-table-ready` 关掉）调用 `MujocoTeleopHardware::set_home(TABLE_READY 的前 7 个数, 夹爪 0)`。`set_home` 在一把锁里把 7 个臂 `qpos` 和两个指尖 `qpos` 写成目标，速度和 `ctrl` 清零，然后 `forward_and_render()`。这是一次瞬移，没有 S 曲线。随后 5 拍 `unilateral_step()`，再把参考设成当前 `qpos`。

同向示教和回放在 `MirrorTeachingArm` 构造函数里对仿真执行 `set_home({0, 0, 0, 1.570796, 0, 0, 0}, 0)`。真机同向示教不回零、不播这张 CSV（`README.md` 已写明）。

停止：

- `SIGINT` / `SIGTERM` 都进 `signal_handler()`，只把 `keep_running` 设为 false。第二次信号不会另外失能。
- 主循环退出后先停参考线程和 Quest 线程，再停跟随线程和渲染线程。
- 仅真机接着 `send_kd_only_release()`：抓住第一帧测量姿态，按 Kp/Kd 保持约 6 秒（`OPENARM_QUEST_DAMP_SEC`，夹在 0.2–60 s），再 `Shutdown()`。保持的是停下时的姿态，不会回到 CSV 末帧。
- 仿真不调用这段。`disable_all()` 在 `MujocoTeleopHardware` 里是空函数。跟随线程已停，所以 `qpos` 留在最后一条参考上，窗口随后关掉。
- 渲染线程或跟随线程看到 MuJoCo 窗口关闭，也会把 `keep_running` 设为 false，然后走同一条退出。仿真窗口关闭今天没有回位。
- `run.sh` 的 `trap cleanup EXIT INT TERM` 会对控制进程再发一次 `SIGINT` 并 `wait`。控制台「停止并释放」对整个进程组发 `SIGINT`，8 秒后 `SIGTERM`，再等 3 秒（`script/openarm_app/main_window.py` 的 `_stop`）。`SIGTERM` 同样只被信号函数收下，不会杀死进程。6 秒阻尼通常赶在 8 秒内结束。若前面加上最长 15 秒的回位，8 秒等待会先到期；第二次 `wait(3)` 超时后再抛出的话，界面可能停在「正在释放…」。这一点没有在运行中的控制台上试过。

四种遥操模式共用上述启动和退出。模式差异在参考线程里的控制器，不在 `set_home` 或 `send_kd_only_release`。

## 3. 回放现在怎么起步、怎么停

回放是另一个进程：`script/replay_teaching.py` → `control/openarm_teaching_replay.cpp`。控制台要求先停止遥操。窗口是 `script/openarm_app/replay_dialog.py`。

仿真构造 `MirrorTeachingArm` 时瞬移到 `TABLE_READY` 肘 90°、夹爪 0，再从这条测量值以 0.3 rad/s、0.6 rad/s² 接入录像第一帧。真机从电机当前位置接入，不播 `steady_action_on_table.csv`。接入超时 120 s。播放中臂关节 0.6 rad/s、1.2 rad/s²。循环（`--repeat`，界面里的次数）在遍与遍之间再次进入 `aligning`，目标是 **这条录像的第一帧**，不是起手姿势。

停止有两条入口，今天的窗口只用第一条：

- 控制台 `_stop()` 发进程组 `SIGINT`。5 秒后若仍在跑再发 `SIGTERM`。状态文案直接写成「正在停止并释放双臂…」。
- 标准输入 `stop` 也能把循环标志清掉。窗口没有写这个词。PR #2 的 R2 打算改成先写 `stop`。

循环退出后 `MirrorTeachingArm::stop()` 立刻 `disable_all()`。真机因此马上失能，没有 6 秒阻尼，文档要求人托住手臂。仿真的 `disable_all()` 为空，进程退出前画面停在最后一条指令上。

进度行是：

```text
[replay] PROGRESS time=… duration=… phase=aligning|playing|paused|complete error=…
```

`replay_dialog.py` 用字典把这四个 `phase` 译成中文。字典里没有 `returning`。多出来的阶段会在界面线程里产生 `KeyError`，这条日志之后的进度不再更新。

## 4. `kp>0` 直接改 `qpos` 对「不要瞬移」意味着什么

`MujocoTeleopHardware::send_arm()`（`src/hardware/mujoco_teleop_hardware.cpp`）：

- `kp>0` 或 `kd>0`：`qpos = cmd.q`，`qvel = cmd.dq`，执行器 `ctrl = 0`。
- 否则：`ctrl = tau`，交给物理往前积分。
- 夹爪 `send_gripper()` 不论增益，总是写两个指尖的 `qpos`。

跟随线程把 `config/follower.yaml` 的 Kp/Kd 打进 MIT，`tau` 固定为 0。默认 Kp 远大于 0，所以上面的力矩分支在遥操和回放里不会走到。仿真没有电机侧的 Kp 弹簧，也没有速度滞后。参考里的每一拍就是下一帧画面。

后果：

- `set_home` 或单拍把参考写成末帧，画面从全零或从当前姿态跳到末帧。这就是今天仿真「已经在桌面姿态」的做法，目标还是错误的 `TABLE_READY`。
- 若参考按 S 曲线每拍只前进 `v·dt`，仿真会沿这条曲线运动，看起来是平滑的。限速必须做在参考发生器里。MuJoCo 的阻尼和力限制补不上被跳过的中间姿态。
- 真机同一条参考会再被电机 Kp/Kd 滤一遍，所以仿真比真机更贴指令。验收时比较的是两边的参考日志，不是把 MuJoCo 关节角和真机电流对齐。
- 回放和同向示教的仿真发送也走 `send_arm`。回放限速（0.3 / 0.6 接入，0.6 / 1.2 播放）在仿真里同样是「画面等于指令」。把回位做成同一类限速参考，仿真和真机的阶段才能对齐。

不要为了「仿真也有惯性」去把 Kp 改成 0。那会让仿真开始走力矩支路，和真机 MIT 位置弹簧不再是同一条指令。

## 5. 建议的行为

### 5.1 一份起手姿势

新增只读函数，例如头文件 `src/controller/start_pose.hpp`：

- 输入路径默认 `assets/steady_action_on_table.csv`，并认 `OPENARM_QUEST_STARTUP_FILE`。
- 读最后一帧的 16 个位置。臂关节单位是弧度，仿真和真机相同。
- 夹爪保留文件原值，并带一个单位标记（这份文件是电机弧度）。仿真下发前走与 `TeachingTrajectory::convert_grippers(source_meters=false, simulation=true)` 相同的线性映射；真机保持原弧度。
- 臂关节超出对应 URDF 限位则拒绝启动，不要静默夹紧。
- `TABLE_READY_POSITION` 继续只做 IK 零空间和 `q_nominal`。不要把它改成 CSV 末帧，否则遥操肘的偏好会跟着变。注释里「仿真姿态就是 TABLE_READY」在仿真改完后要改掉。

启动运动仍播放整段 CSV（当前姿态 → 第一帧的 S 曲线 → 文件 → 保持末帧）。回位的目标只取末帧，不倒放 6.4 秒文件。停在启动途中时也一样：放弃剩余的文件，从当前参考用 S 曲线走到末帧。

### 5.2 仿真启动：走真机那条启动，不要先 `set_home`

`run.sh` 的仿真分支传入与真机相同的 `--startup-move`、`--startup-file`、`--startup-transition-sec`。同时传 `--no-sim-table-ready`，避免先瞬移到 `TABLE_READY` 再从那里接第一帧。

`prime_sim_arm()` 在即将播放启动轨迹时不要调用 `set_home`。初始 `qpos` 保持 XML 的全零。第一拍参考必须是当前测量（全零），然后才是 S 曲线。从全零到 CSV 第一帧，最大关节差约 1.32 rad，固定 5 秒过渡低于 0.6 rad/s，不会把 S 曲线加速到超限。

`OPENARM_QUEST_STARTUP_MOVE=0` 时仿真停在全零，不再瞬移到 `TABLE_READY`。真机保持今天的行为：跳过轨迹，握住上电时的测量姿态。

四种遥操模式都经过 `run_startup_for_arms()`，所以仿真侧改这一处即可。不要在 `human_mimic` 里再写一份。

### 5.3 停止时的平滑回位

遥操和回放共用同一组限制：`openarm::scurve::limits_from_env()`，默认 0.6 rad/s、1.2 rad/s²、6 rad/s³。时长用曲线自己的时间（调用过渡生成时时长传 0，让它取 `max(自然时长, 0.5 s)`）。不要复用启动那条固定 5 秒：距离远时，固定 5 秒会把速度放大到 0.6 以上。现有过渡函数还有 10 秒上限；按 0.6 rad/s，本臂关节行程（关节 2 大约 3.5 rad）的自然时长低于 10 秒，正常回位不会被这道上限压短。

回位超时默认 15 秒，环境变量 `OPENARM_RETURN_TIMEOUT_SEC`，夹在 1–60 秒。15 秒盖得住上述行程。超时后停止发跟踪指令并释放，日志：

```text
DISABLE reason=return_timeout
```

到位条件与回放接入同一数量级：臂关节参考误差 < 0.015 rad，仿真里 `qpos` 与参考之差同样 < 0.015 rad（因为 `qpos` 被写成参考，这一条在仿真里会立刻满足）。真机再要求测量误差 < 0.12 rad，与今天接入录像的门槛一致，避免在弹簧还没跟上时失能。速度低于 0.02 rad/s 并保持 0.3 秒后进入释放。这些门槛做成命名常量，测试直接引用。

日志每 0.5 秒一行，字段名按控制与安全工程师的约定，不另起名字：

```text
phase=returning return_left=<剩余秒数>
```

本方案把 `return_left` 定义为剩余秒数，供界面倒计时。若该工程师已经把它定义成左臂误差，以他们的单位说明为准，界面原样显示，不要再做一个平行字段。

遥操的打印前缀保持 `[quest-cartesian]`，回放保持 `[replay] PROGRESS`，这样现有检索还能用。回放在 `phase=` 上增加 `returning`，不要删掉 `aligning` / `playing` / `paused` / `complete`。

释放顺序：

1. 停参考线程和 Quest 接收，避免 Grip 在回位途中改参考。跟随线程和仿真渲染继续跑。
2. 回位函数只写 `set_all_references()`，由已经在跑的跟随线程下发。这样仿真的 `qpos` 会更新，真机仍走 MIT。不要在回位里调用 `set_home`。
3. 到位后停跟随线程和渲染。
4. 真机遥操再执行今天的 `send_kd_only_release()`（约 6 秒，保持的是已经到达的起手姿势），然后 `Shutdown()`。
5. 真机回放在到位后 `disable_all()`，不加这 6 秒。负责人的决定是「先平滑回到默认起手姿势，再释放」。6 秒阻尼仍然只属于遥操退出。
6. 仿真两边都没有 CAN 可失能。回位走完后进程退出，窗口关掉。

回放的遍间衔接不变：那是回到录像第一帧，限速仍是 0.3 rad/s、0.6 rad/s²。只有操作员停止时才走向起手姿势。

第二次 `SIGINT` 或 `SIGTERM` 放弃剩余回位，立即走第 4 或第 5 步。今天的信号函数分不出第一次和第二次，实现时要加一个计数或原子状态，而不是只靠 `keep_running`。

仿真回放进程开头同样不要 `set_home`。从 XML 全零用同一条 S 曲线走到起手姿势，日志 `phase=startup`，完成后再进入现有的 `aligning`（走向录像第一帧）。这样窗口打开时的第一段运动和遥操仿真一致，而不是先闪到肘 90°。

### 5.4 异常退出

| 情况 | 行为 |
| --- | --- |
| 第一次停止（控制台按钮、回放停止、`run.sh` 收到 INT） | 进入 `phase=returning` |
| 回位途中 CAN 年龄超限或 `error_code` 超限 | 停止回位并失能。原因用现有 CAN 日志，不要写成 `return_timeout`。回放继续用 `MirrorTeachingArm::read()` 里已有的 150 ms / `error_code > 1` |
| 回位开始前输出已经关掉（PR #4 的 `OutputDisabled()`） | 跳过回位和 6 秒阻尼，直接 `Shutdown()` / `disable_all()` |
| 到达 15 秒仍未满足到位条件 | `DISABLE reason=return_timeout`，然后释放 |
| 第二次 INT/TERM、回放窗口的第二下停止 | 不再跟踪，直接释放 |
| C++ 异常已经失能电机 | 不要再规划回位。今天遥操的 `catch` 在 `main` 外层，正常阻尼根本不会跑；回位不要放进这个 `catch` 去「补救」一个已经失败的对象 |
| `SIGKILL`、掉电、CAN 适配器消失、进程崩溃 | 软件回位不会发生。电机随后的行为未验证（健康检查 Q1） |
| 关掉 MuJoCo 窗口 | 视为第一次停止，在窗口还开着时走完回位再退出。第二次关闭窗口立即退出。可用 `OPENARM_SIM_RETURN_ON_CLOSE=0` 恢复今天的「关闭即停」 |

回位期间不要重置 `SafetyFilter` 之后立刻把参考写成末帧。PR #4 的 `max_first_step_delta`（默认 0.35 rad）只限制复位后的第一步，下一步 1 ms 仍能跳变。S 曲线从当前参考出发，第一步远小于 0.35 rad，不依赖这道闸做全程限速。

### 5.5 控制台要显示的阶段

主界面今天在子进程刚 `Popen` 成功时就把状态写成「仿真运行中」或「真机运行中」（`_started`）。启动轨迹大约 5 + 6.4 + 0.6 秒，这段时间 Grip 还没有意义。界面应解析控制进程已经打出的行：

| 日志 | 状态栏 |
| --- | --- |
| `startup dual-CSV:` 或 `phase=startup` | 正在前往起手姿势 |
| `startup dual-CSV complete` 或 `startup move complete` | 仿真运行中 / 真机运行中（与今天的模式名一致） |
| `phase=returning` | 正在回到起手姿势 |
| 同一行里的 `return_left=` | 括在状态栏后面，单位秒 |
| `DISABLE reason=return_timeout` | 回到起手姿势超时，正在释放 |
| 进程退出且曾经进入回位 | 已停止并释放 |
| 进程退出且没有这些行（启动失败） | 保持今天的「启动失败」或返回码 |

回放窗口把 `returning` 译成「正在回到起手姿势」，`startup` 译成「正在前往起手姿势」。字典用默认值接未知阶段，避免 `KeyError`。停止按钮在看到 `returning` 之前不要把文案写成已经释放。

等待时间：

- 遥操停止：先等回位最多 15 秒，再等阻尼最多 6 秒。控制台的 `wait` 改为大约 22 秒，然后再发第二次信号。不要在 8 秒时发 `SIGTERM`。
- 回放停止：先写标准输入 `stop`（与 PR #2 的 R2 相同），界面等待 `returning` 和随后的 `STOPPED` 或 `DISABLE`。第二次信号放在 15 秒之后，替换今天的 5 秒。
- 不要用 `SIGKILL` 当常规收尾。超时释放由控制进程自己完成。

录制按钮在 `phase=startup` 和 `phase=returning` 期间保持禁用。今天「运行中」一出现就可以点录制，那会把启动轨迹录进去。

## 6. 和进行中的工作怎么错开

### PR #3（Quest UDP 只听回环）

启动轨迹和回位都不读 Quest 姿态。PR #3 改的是 `src/io/*_receiver.cpp`、`quest/quest_bridge.py` 和 `run.sh` 里传给桥的地址。本方案要改的 `run.sh` 只在仿真参数列表里追加 `--startup-move` 和 `--no-sim-table-ready`，不要改它的 `--udp-host` 或场景路径。两边都碰 `run.sh` 时，启动参数和 UDP 地址是两段互不覆盖的代码。Grip 在回位前已经被停掉，回环绑定不会挡住回位。

### PR #4（CAN 新鲜度、`SafetyFilter`、超时不补拍）

PR #4 已在真机退出处写成：跟随线程停下之后，若任一臂 `OutputDisabled()`，就跳过 `send_kd_only_release()`，否则仍做约 6 秒保持，然后 `Shutdown()`。仿真仍然不进这段。

回位若加在「跟随线程已经 stop」之后，仿真的 `qpos` 不会再变，真机也会绕开 `Control::unilateral_step()`。所以回位要放在 `follower_thread.stop_thread()` 之前，并且一开始就看 `OutputDisabled()`。PR #4 合入后再改这个退出块，避免两个人同时重排同一段。回放文件 `openarm_teaching_replay.cpp` 不在 PR #4 的改动列表里，回放回位可以并行，但不要在回放里再复制一套 CAN 年龄判断。

`SafetyFilter` 的位置夹紧和首步 0.35 rad 不是回位的速度曲线。回位不要依赖它把末帧「慢慢夹过去」。

### 控制与安全工程师的回放回位

约定已经是：`phase=returning`、`return_left=`、`DISABLE reason=return_timeout`、超时默认 15 秒。本方案直接采用这些记号，遥操退出打同一对字段。

PR #2 的 R1 写的是另一件事：停止时把指令速度收到 0，再 `disable_all()`，并且不把 6 秒阻尼搬进回放。负责人后来决定回放停止要先回到默认起手姿势再释放。实现时以该决定为准：遍间衔接、0.45 rad 跟踪误差、150 ms CAN 检查仍然按 PR #2 保留；「停在原地再失能」只保留为第二次信号和超时之后的释放，不作为第一次停止的目标。不要为了落地 R1 把 `returning` 做成速度归零后原地失能。

### 界面工程师的控制台改动

`main` 上 `73dc004` 已经改过 `script/openarm_app/main_window.py` 和 `replay_dialog.py` 的布局，回放进度仍按四个 `phase` 硬查。后续界面改动若再改这两个文件，阶段文案按第 5.5 节接在现有状态栏上，不要新开一套 UDP。先合入「未知 `phase` 用原文显示」可以避免控制端先打出 `returning` 时界面崩溃。等待时间的修改和回位实现放在同一次发布里，否则 8 秒 / 5 秒的第二次信号会把 15 秒回位截断。

同向示教、零重力、MoveIt 不播这张 CSV，也不做这次回位。它们的仿真窗口仍会 `set_home` 到 `TABLE_READY`，和遥操起手姿势不同。第一批任务不要改 `run_mirror_teaching`、`openarm_zero_g.cpp` 和 `real_arm.cpp`。

## 7. 任务

规模：S 是一个局部函数加测试；M 是一条已有路径上的行为变化；L 是遥操退出和信号，并且要等 PR #4 的退出块。

| 任务 | 规模 | 文件 | 测试与验收 | 依赖 |
| --- | --- | --- | --- | --- |
| P1. 起手姿势加载 | S | 新建 `src/controller/start_pose.hpp`；`tests/` 里一个不链接 CAN 的程序或 Python 夹具 | 最后一帧与本文第 1 节的 16 个数一致（允许 1e-6）；空文件、列数不对、非有限值失败；臂关节超出 URDF 限位失败；夹爪弧度按回放同一公式映射到米 | 无。不改 `openarm_quest_cartesian_control.cpp` |
| P2. 仿真遥操走同一条启动轨迹 | M | `run.sh` 的仿真分支；`prime_sim_arm()` | 仿真命令行含 `--startup-move` 和 `--no-sim-table-ready`。日志先出现 `startup dual-CSV`，再出现 `startup dual-CSV complete`，然后才 `bimanual running`。参考从全零连续走到末帧，单拍增量不超过过渡段的速度上限乘 `dt`。不调用 `set_home` 写 `TABLE_READY`。`OPENARM_QUEST_STARTUP_MOVE=0` 时 `qpos` 保持 0 | P1。与 PR #3 只在 `run.sh` 的不同段落相遇 |
| P3. 回放回位 | M | `control/openarm_teaching_replay.cpp`；必要时在 `src/controller/teaching_replay.hpp` 加纯函数步进 | 无 CAN：从末帧相差 1 rad 的姿态生成回位，每 5 ms 速度不超过 0.6 rad/s，速度增量不超过 `1.2 * 0.005`，15 秒内到位。超时路径打印 `DISABLE reason=return_timeout` 且在该行之后才允许失能标志。`phase=returning` 与 `return_left=` 出现在 `PROGRESS` 行。遍间 `aligning` 的目标仍是录像第一帧。仿真构造不再 `set_home` 到肘 π/2 | P1。不改 `mirror_teaching_arm.hpp` 的 CAN 检查。不要按 PR #2 把第一次停止做成原地失能 |
| P4. 控制台阶段 | S | `script/openarm_app/main_window.py`；`script/openarm_app/replay_dialog.py`；`tests/test_replay_dialog.py` | 未知 `phase` 不抛异常。`returning` 显示「正在回到起手姿势」。遥操停止的第一次等待不少于 22 秒；回放第二次信号不早于 15 秒。启动完成日志出现前，录制按钮不可用 | 日志格式以 P3 为准。可先合入「未知阶段不崩溃」，等待时间随 P3 一起改 |
| P5. 遥操四种模式的停止回位 | L | `control/openarm_quest_cartesian_control.cpp` 的双臂和单臂退出；`src/common.hpp` 的信号函数若要区分第二次信号 | 仿真：Grip 线程已停、跟随线程仍在时，参考走到 CSV 末帧，单拍不超过 S 曲线限制；然后进程退出。真机路径在到位之后才进入 `send_kd_only_release`。`OutputDisabled()` 为真时不生成回位参考。第二次信号跳过剩余点。启动途中的停止改为走向末帧，而不是把剩余 CSV 播完 | PR #4 的退出块先落地，或与它一起改。P2 的启动目标已经是同一末帧 |

建议顺序：P1 和 P4 的「未知阶段」可以马上做。P2、P3 并行。P5 等 PR #4。P4 的等待时间随 P3、P5 一起改。

不在这批任务里：改默认 MuJoCo 场景路径（健康检查 C2 / Q6）、把 IK 的 `q_nominal` 换成 CSV 末帧、同向示教 的初始姿态、把启动 CSV 再限速到 0.6 rad/s（那会改变真机今天的启动手感）。

## 8. 给负责人的问题

1. 起手姿势是否就是本文第 1 节的 CSV 最后一帧？若现场桌面姿态已经换过文件，是否仍以 `OPENARM_QUEST_STARTUP_FILE` 的最后一帧为准？
2. 遥操停止是否也要回位？今天真机遥操是原地保持约 6 秒再失能。本方案按「遥操和回放都先回位」设计。若遥操仍要原地保持，P5 可以不做，只做仿真启动和回放回位。
3. 回到起手姿势之后，遥操是否保留这 6 秒阻尼？回放按你的决定在到位后直接释放。末帧是否已经靠在桌面上、松力后不会砸下去，这次没有在真机上看过。
4. 仿真夹爪用回放那套弧度→米的映射，还是在起手时保持指尖 0（张开）？两种画面不一样。
5. `return_left=` 是否就是剩余秒数？
6. 关掉仿真窗口要不要看完回位？本方案默认要看完，可用环境变量关掉。
7. 同向示教、零重力、MoveIt 的仿真窗口继续停在 `TABLE_READY`，是否可以留到后面？

## 9. 这次没有验证的事项

- 没有编译，没有运行 `unittest`，没有打开 MuJoCo 或 Qt，因此没有看到瞬移和 S 曲线的画面。
- 没有把 CSV 末帧放到真机上，不知道它是否落在桌面支撑范围内，也不知道松力后的下沉。
- 没有确认 0.037 rad 夹爪在真机上的开合，也没有确认映射成约 0.028 m 后的仿真指尖是否看起来相同。
- 13 rad/s 的尖峰是相邻两行位置相减得到的，没有看当时电机是否跟上。
- 达妙固件在命令中断后的行为仍以健康检查 Q1 为准，未验证。
- `phase=returning` 这组字段没有出现在 `main` 或 PR #1–#8 的描述和评论里。本文按这次任务里控制与安全工程师的约定书写，没有读到他们仓库外的草稿。
- 控制台 8 秒后 `SIGTERM` 仍被信号函数捕获、界面可能停在「正在释放…」，是阅读 `_stop` 和 `signal_handler` 的结论，没有点过按钮。
