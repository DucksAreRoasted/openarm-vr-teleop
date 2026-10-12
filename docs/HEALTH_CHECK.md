# OpenArm VR 遥操作仓库健康检查

检查日期：2026-10-12。范围是本仓库的 C++ 控制核心（`src/`）、可执行入口（`control/`）、Quest 桥（`quest/`）、ROS 2 / MoveIt（`ros2/`）、启动脚本（`scripts/`、`run.sh`、`app.sh`、`install.sh`）、测试（`tests/`）和现有文档（`docs/`、`README.md`、`AGENTS.md`）。

本次只读代码，没有改控制逻辑，也没有连接 Quest、CAN 或真机。文中凡是写明「未验证」的结论，都是代码审阅无法单独证实的事项。

## 1. 架构总览

仓库从 Motion Trajectory Studio 拆出，默认目标是 Quest 3 遥操作 OpenArm 双臂。仿真走 MuJoCo，真机走达妙电机的 SocketCAN FD。重力补偿、KER 示教、同向示教、轨迹回放和 MoveIt 是并列入口，它们复用 KDL 动力学和部分 YAML，但并不共用同一条安全状态机。

### 1.1 模块怎么拼在一起

| 区域 | 职责 |
| --- | --- |
| `src/controller/` | 四种遥操作律：默认 `human_mimic`、`cartesian_precision`、`skeleton_proximal`，以及遗留的 `pika_style` 配置口。另有关节参考发生器、S 曲线、同向示教和夹爪跟随。 |
| `src/hardware/` | `TeleopHardware` 接口。`CanTeleopHardware` 转发到 vendored `third_party/openarm_can`；`MujocoTeleopHardware` 用同一接口驱动仿真。 |
| `src/io/` | UDP 文本协议：左右手柄（47912 / 47911）、骨骼姿态（47951）、骨骼 hint（47950）、身体姿态。录制和点动口在 `teleop_io.cpp`，绑定 `127.0.0.1`。 |
| `src/safety/` | `SafetyFilter`。设计上位于 MIT 下发之前，实现只处理非有限值和停止模式。 |
| `src/openarm_port/` | 打开 CAN、使能电机，并先发一串 `kp=kd=tau=0` 的中性帧。 |
| `control/` | 进程入口：`openarm_quest_cartesian_control.cpp`（约 5486 行，遥操主程序）、`openarm_zero_g.cpp`、`openarm_gravity_calib.cpp`、`openarm_gravity_model.cpp`（离线，不开 CAN）、`openarm_teaching_replay.cpp`、`openarm_read_motor_params.cpp`。 |
| `quest/` | HTTPS + WebSocket 页面把 Quest 手柄和肩肘骨骼转成 UDP。网页不显示相机。 |
| `script/openarm_app/` | PySide6 控制台。模式、CAN、相机、重力微调和录制都从这里拉起子进程。 |
| `scripts/` | CAN 配置、MoveIt、零重力、重力标定。`scripts/lib/` 是标定用的 Python 拟合。 |
| `ros2/src/openarm_moveit_config/` | MoveIt 2 配置。仿真用 `fake_arm.py`；实机用 `scripts/real_arm.cpp` 以约 100 Hz 跟踪 `FollowJointTrajectory`。 |
| `config/` | `follower.yaml`（MIT 增益）、`gravity_comp.yaml`、`ker.yaml`、相机配置。 |
| `assets/` | URDF 和桌面启动轨迹。MuJoCo XML 在 `models/OpenArm_V1/`，但 `run.sh` 的默认场景不指向这里。 |

`third_party/openarm_can` 是随仓库拷贝的 CAN 栈，没有记录上游 commit。CMake 把它编成静态库 `openarm_can_vr`。

### 1.2 Quest 追踪到关节指令的数据流

默认模式是 `human_mimic`，姿态映射是绝对标定：第一次握 Grip 锁定向，之后松开再握只重锚位置。

1. Quest 浏览器打开 `https://<控制电脑>:8443`（`run.sh` 把桥绑在 `0.0.0.0`）。页面经 WSS（默认 8442）把手柄姿态、Grip、Trigger、A/B 或 X/Y 发给 `quest/quest_bridge.py`。
2. 桥把每只手打成 CSV，UDP 发到本机 `47911`（右）和 `47912`（左）。骨骼模式再发 `47951`。桥到控制器的 UDP 目标默认是 `127.0.0.1`，但控制器侧的 socket 绑的是 `INADDR_ANY`。
3. `QuestReceiveThread` 约 90 Hz 排空 UDP，用本机 `steady_clock` 盖接收时间戳。控制律不使用包内的 Quest 时间戳做超时。
4. 另一条线程按控制周期跑逆解，把关节参考写入 `RobotSystemState`。默认律在 `HumanMimicController`：Grip 离合、虚拟臂骨骼、整臂 IK、`JointReferenceGenerator` 做速度和加速度限制，关节限位来自 URDF。
5. `FollowerControlThread` 以 `FREQUENCY`（1000 Hz）调用 `Control::unilateral_step()`。它读电机状态、可选地过 `SafetyFilter`、把参考打成 MIT `{kp, kd, q, dq, tau=0}`，再 `recv_all(200)`。跟随臂的重力不进前馈力矩，靠 `config/follower.yaml` 的 Kp 在电机里形成弹簧（近端 Kp 为 240）。
6. 松开 Grip、Quest 超时或 IK 连续失败时，控制器应冻结参考并要求重新握持。笛卡尔模式和骨骼模式按这个语义实现；默认的 `human_mimic` 没有守住，见发现 H1。
7. 停止进程时，遥操退出路径会先按测量姿态做阻尼保持（默认约 6 秒，可用 `OPENARM_QUEST_DAMP_SEC` 改），再 `disable_all()`。这是松手支撑，不是切断驱动电源。

同向示教、KER 和零重力不走 Quest。KER 要双扳机连续按住 2 秒才使能，输入中断超过 250 ms 会停双臂。同向示教在启动和每个周期检查 8 个电机的反馈年龄（150 ms）和 `error_code > 1`。零重力是 100 Hz、`kp=0`、`tau = ff_gain*(scale*G(q)+bias)+摩擦`。MoveIt 实机在 `real_arm.cpp` 里用 PD + 重力前馈 + 保持积分，规划侧速度上限在 `joint_limits.yaml` 里是 2 rad/s。

控制台里的「急停」勾选框是操作员确认：勾上才允许 MoveIt / 零重力连真机。它不读取硬件急停，也不在循环里切力矩。停止按钮结束子进程。自动重力微调的提示写明运动中要靠硬件急停。

## 2. 发现

严重度：high 表示按当前代码，真机在丢跟踪、丢 CAN 或错误使能时可能继续出力矩或自行恢复运动；medium 表示安全边界不完整、构建不可重复或维护风险大；low 表示死代码、注释漂移或局部质量问题。

### H1. 默认模式在超时和 IK 失败后会把仍按住的 Grip 当成新的握持

- 严重度：high
- 位置：`src/controller/human_mimic_controller.cpp`（超时约 2351–2390 行，IK 冻结约 2465–2476 行，`activate_clutch` 约 736–738 行）
- 说明：超时分支把 `last_grip_` 写成 `false`，并设置 `require_reclutch_`。下一帧如果人没有松开 Grip，`grip_rising` 为真，随手把 `require_reclutch_` 清掉并调用 `activate_clutch()`。该函数把 `solve_fail_count_` 置 0、`hold_active_` 置 false，离合重新接上。日志写的是 “re-grip required”，实际不需要物理上松开再握。IK 连续失败（`solve_fail_freeze_count`，默认 30）走同一条 `last_grip_ = false` 路径，失败计数会被下一次合成上升沿清零，冻结站不住。
- 对照：`CartesianController::handle_quest_timeout_recovery()` 保留上一帧 Grip，恢复后必须见到真实上升沿。`SkeletonProximalController` 在超时期间把 `last_grip_` 设成当前 Grip，按住不放不会形成上升沿。默认 `VR_TELEOP_MODE=human_mimic` 用的是有问题的那条。
- 未验证：没有在仿真里单步复现；结论来自这条分支的控制流。

### H2. 名为安全滤波的一层不限制位置、速度、加速度和实际力矩

- 严重度：high
- 位置：`src/safety/safety_filter.cpp`，`src/openarm_constants.hpp` 70–80 行，`src/common.hpp` 24–60 行，`src/controller/control.cpp` 的 `unilateral_step()` 跟随分支（约 570–580、725–727 行），`config/follower.yaml`
- 说明：
  - `SafetyConfig` 带了位置、速度、加速度、`max_first_step_delta`。`filter()` 把 `dt` 标成未使用，循环里只钳位 `effort`，而且仅当 `max_effort[i] > 0`。
  - `velocity_limit` 和 `effort_limit` 八个关节全是 0。注释写「暂时按请求解锁」以及「≤0 关闭反馈速度安全触发」。全仓库没有第二处实现这个速度触发。`make_default_safety_config()` 又对速度做 `min(limit, 2.0)`，结果仍是 0；仿真配置用 `max(limit, 12.0)`，两套数字都不会被滤波器读取。
  - `SafetyMode::Stop` 只有 `set_mode` 定义，没有任何调用点。
  - 尺寸不匹配时 `filter()` 把 `emergency_stop` 设为真并留下空的 `safe_cmd`。`unilateral_step()` 接着 `set_all_references(safe_ref)`。`RobotSystemState::set_all_references` 在长度不对时抛异常。1000 Hz 线程把异常打日志后继续，这一拍不发 MIT，电机保持上一帧。这不是锁存急停，也不会 `disable_all()`。
  - 跟随 MIT 的 `tau` 固定为 0。真正的力矩是电机内部的 `Kp*(q_des-q)`。近端 Kp 为 240。软件侧的力矩钳位即使以后接上，也钳不到这根弹簧。笛卡尔模式有 `max_tracking_error_rad = 0.25`；默认的 human_mimic 没有同等的跟踪误差钳位。
- IK 和 `JointReferenceGenerator` 会按 URDF 限位夹参考，所以正常遥操不会只靠这层滤波器。滤波器是 MIT 前的最后一闸，目前等于没闸。`openarm_constants.hpp` 里肘关节上限是 π，URDF `openarm_*_joint4` 的 upper 是 2.443461。这组更宽的常数只被拷进未使用的 `SafetyConfig`。若以后「把滤波器接上」却用这组常数，允许范围会宽于 URDF。

### H3. 遥操、零重力和 MoveIt 实机不检查 CAN 反馈是否新鲜

- 严重度：high
- 位置：`src/controller/control.cpp` `SeedHoldMeasured()`（约 791–848 行），`src/openarm_port/openarm_init.cpp`，`control/openarm_zero_g.cpp` `read_locked()` 与构造函数里的 20 次 `refresh`，`ros2/src/openarm_moveit_config/scripts/real_arm.cpp` 的 `ArmBus` 构造，`third_party/openarm_can/src/openarm/damiao_motor/dm_motor.cpp`（状态默认 `q=0`），对照 `src/controller/mirror_teaching_arm.hpp` `validate_feedback()`（约 212–227 行）
- 说明：电机对象在收到帧之前位置是 0，`last_state_received()` 为空。同向示教会在 1 秒内等齐 8 个电机，年龄超过 150 ms 或 `error_code > 1` 就拒绝启动并在运行中抛错停臂。另外三条真机路径没有等价检查。
  - 遥操初始化先 `enable_all()`，再发 `kp=0` 的中性帧（这一步本身不会把臂拉向 0）。随后 `SeedHoldMeasured()` 用「测量位置」发带 Kp/Kd 的保持帧，并且总是返回 true。没有回包时测量值仍是 0，近端就会以 Kp 240 指向零位。
  - 零重力和 MoveIt 实机同样刷新约 20 个周期后直接 `read()`。无回包时 `measured_` 保持 0，零重力按 `G(0)` 出力，MoveIt 从零位开始跟踪。
  - 运行中反馈停更后，遥操仍用最后一次 `q` 做逆解和夹爪判断；零重力继续按陈旧 `q` 算 `G(q)`。`write_canfd_frame` 的失败被忽略，发送侧也不知道命令是否上路。
  - 录制侧只在「指令关节长期不动」时打印一行 CAN 警告，不停止力矩。
- 未验证：达妙固件在命令中断后是保持、阻尼还是自行失能（协议里有 RID `TIMEOUT`，本仓库不写这个参数）。`error_code` 各数值的厂家定义也没有随仓库附带手册。

### H4. Quest 姿态通道对局域网开放，且没有来源校验

- 严重度：high
- 位置：`src/io/quest_pose_receiver.cpp`（绑定 `INADDR_ANY`，约 31–35 行），同样绑定方式的 `skeleton_pose_receiver.cpp`、`skeleton_hint_receiver.cpp`、`body_pose_receiver.cpp`；`quest/quest_bridge.py`（`--host` 默认 `0.0.0.0`，WebSocket 无身份）；`run.sh` 112–118 行
- 说明：能向控制机的 UDP 47911 / 47912 / 47951 发包的一方，可以写入 Grip=1 和任意位姿，从而在默认模式里接上离合并驱动关节参考。WebSocket 同样接受任意能连上 8442 的客户端，桥再把 JSON 转成上述 UDP。点动和录制口在 `teleop_io.cpp` 绑定 `127.0.0.1`，这一对端口没有对全网卡开放。
- 未验证：没有在网络上做注入试验。结论是套接字和解析器的行为。

### H5. 没有持续集成，C++ 回归默认不参与构建

- 严重度：medium
- 位置：仓库根目录没有 `.github/` 或其他 CI 配置；`CMakeLists.txt` 97–98 行 `skeleton_control_smoke` 为 `EXCLUDE_FROM_ALL`；`README.md` 末行只登记 `python -m unittest discover -s tests`
- 说明：`tests/` 有 15 个文件，覆盖重力拟合、同向参考、KER 映射和超时、回放对话框、相机、身体追踪、CAN 混帧解析、初始化中性帧等。这些测试大多用假头文件或纯 Python 复刻，不链接运行中的 `HumanMimicController`、`SafetyFilter` 或 CAN 年龄逻辑。Node 脚本 `tests/test_body_tracking.cjs`、`tests/test_quest_session.cjs` 不在 `unittest discover` 里。C++ 冒烟程序明确不打开 CAN，但普通 `cmake --build` 不会编译它。
- 未验证：本环境没有 `.venv` 和 `build/`，这次检查没有实际跑测试，不知道当前 `unittest` 是否通过。

### M1. `run.sh` 的默认仿真场景在仓库外面

- 严重度：medium
- 位置：`run.sh` 17 行，默认 `OPENARM_MUJOCO_XML=/home/ducks/motion-trajectory-studio/model_library/OpenArm_V1/scene_table.xml`。仓库内已有 `models/OpenArm_V1/scene_table.xml`
- 说明：未设置环境变量时，仿真在其他机器上会在编译之后因为找不到 XML 退出。`README.md` 和 `AGENTS.md` 已经写了这个默认值是机器相关路径。`app.sh` 只是把控制台转给 `script/teleop_app.py`，场景路径仍由下层启动器决定。

### M2. 安装与依赖没有锁到可复现的组合

- 严重度：medium
- 位置：`install.sh`，`requirements.txt`，`requirements-ker.txt`，`CMakeLists.txt` 16–50 行
- 说明：`install.sh` 不安装 README 列出的 apt 包，只建 `.venv` 并 `pip install`。`mujoco` 和 `websockets` 有精确版本，`numpy`、`PySide6`、`scipy` 等只有下界。`openarm-dataset` 安装失败会被吞掉。没有 lock 文件。CMake 把 `/opt/ros/humble` 追加进前缀路径，配置阶段必须能 `import mujoco`，否则整个工程包括离线的 `openarm_gravity_model` 都配不过。`find_library` 同时找 `libmujoco.so.3.12.0` 和 `3.11.0`，与 `requirements.txt` 的 3.12.0 不完全一致。KER 使用另一份 `requirements-ker.txt` 和 `.venv-ker`，主 `install.sh` 不装它。

### M3. Quest HTTPS 静态服务可以把目录读出 `quest/`

- 严重度：medium
- 位置：`quest/quest_bridge.py` `HTTPRequestHandler.do_GET`（约 57–84 行）
- 说明：请求路径去掉开头的 `/` 后直接 `os.path.join` 到 `quest/` 再 `open`。路径里的 `..` 没有先规范成实路径再限制在网页目录内。桥默认监听所有网卡，和 H4 是同一条暴露面。证书是运行时自签的，私钥路径 `quest/quest_key.pem` 已在 `.gitignore`。CORS 为 `*`。
- 未验证：没有发 HTTP 请求去读仓库外文件。

### M4. 软件停止是数秒阻尼释放；界面上的急停不是急停

- 严重度：medium
- 位置：`control/openarm_quest_cartesian_control.cpp` `send_kd_only_release()`（约 1181–1233 行，默认 6 秒），`script/openarm_app/main_window.py` 约 215、360、430 行，`scripts/run_moveit.sh`
- 说明：SIGINT / 正常退出会保持 MIT 刚度再失能，避免松力砸落。控制台勾选框只决定子进程是否带 `--real` 或 `use_real:=true`。自动标定进行中，停止按钮的提示要求改用硬件急停。进程被 SIGKILL、掉电或 CAN 适配器拔出时，这条阻尼路径不会跑完。
- 未验证：现场是否另有切断电机供电的硬件急停，以及它是否独立于这台电脑。

### M5. 重力模型和电机表在三条实机路径里各写一份

- 严重度：medium
- 位置：`control/openarm_zero_g.cpp`，`ros2/src/openarm_moveit_config/scripts/real_arm.cpp`，`src/controller/mirror_teaching_arm.hpp`，以及 `src/openarm_constants.hpp` 的 `DEFAULT_MOTOR_CONFIG`
- 说明：指尖质量、夹爪电机质量、相机质心、摩擦补缺、DM8009/DM4340/DM4310 和 CAN ID `0x01–0x08` / `0x11–0x18` 被复制多份。零重力腕部还有额外 Kd，MoveIt 另有一组 KP/KD/KI 和力矩上限。改一处模型不会自动进其他模式。`Control::unilateral_step()` 的 `ROLE_LEADER` 分支会算重力再把 MIT `tau` 写成 0；本仓库只构造 `ROLE_FOLLOWER`，这段是死路径。

### M6. 1000 Hz 线程不处于实时优先级，超时后会追拍

- 严重度：medium
- 位置：`src/periodic_timer_thread.hpp`（`before_start` 想设 `SCHED_FIFO` 50；周期落后时 `clock_nanosleep` 的绝对时间已在过去，会连续空转补周期），`control/openarm_quest_cartesian_control.cpp` `FollowerControlThread::before_start`（约 3731 行）只打印日志，没有调用基类
- 说明：真正的 MIT 线程把实时设置覆盖掉了。CAN `recv_all` 变慢时，周期线程不会丢拍或进入保持，而是背靠背补步。跟随步内部用固定 `Ts_ = 1/1000`，补拍时夹爪接触计时会比墙钟快。

### M7. 笛卡尔 IK 失败要累积到 1000 次才进入会退出进程的状态

- 严重度：medium
- 位置：`control/openarm_quest_cartesian_control.cpp` 约 817–822 行，`src/controller/cartesian_types.hpp` 92–93 行，致命错误处理约 4181–4184 行
- 说明：类型里的默认 `ik_fail_estop_count` 是 500，遥操入口改成 1000。该状态机挂在 `CartesianController` 上，默认 human_mimic 不走它。笛卡尔模式下，致命错误会把 `keep_running` 置 false，然后仍走 M4 的阻尼退出，而不是立刻失能。1000 Hz 下 1000 次大约是 1 秒持续失败。未在实机确认这个模式是否还有人用；README 写明默认是 human_mimic。

### M8. 文档覆盖操作，不覆盖安全边界和模块边界

- 严重度：medium
- 位置：`README.md`，`docs/` 下 7 篇中文操作说明，`AGENTS.md`
- 说明：重力标定、KER、同向示教、回放、肩肘追踪都有操作文档，和代码里的端口、Grip 语义、标定门槛大体一致。缺少一张和本文第 1 节相当的模块图，也没有写：哪些模式有 CAN 看门狗、软件停止要多长、姿态 UDP 没有认证、`SafetyFilter` 当前不钳位。`AGENTS.md` 写「Git history is unavailable in this checkout」，与当前仓库有 `main` 和 `origin/main` 不符。MoveIt 的 `joint_limits.yaml` 只写速度和加速度，位置限位依赖 URDF，文档没有把两套数字（URDF 肘上限 2.44 rad 与 `openarm_constants.hpp` 的 π）并排放出。

### L1. 控制器源文件已经大到难以单独审查

- 严重度：low
- 位置：`control/openarm_quest_cartesian_control.cpp` 5486 行，`src/controller/human_mimic_controller.cpp` 2725 行，`src/controller/cartesian_controller.cpp` 2070 行
- 说明：遥操入口同时包含参数解析、双臂线程、录制、LeRobot、点动、耦合、KER 和同向示教。H1 这类离合器错误需要顺着 200 行以后的第二处 `last_grip_ = false` 才能看全。没有格式化配置；CMake 打开了 `-Wall -Wextra -Wpedantic`。

### L2. vendored CAN 栈的发送失败和经典帧回调告警没有进控制状态

- 严重度：low
- 位置：`third_party/openarm_can/src/openarm/damiao_motor/dm_motor_device_collection.cpp` `send_command_to_device`，`dm_motor_device.cpp` 在 FD 设备上走错回调时只打 `WARNING`
- 说明：控制周期把发送当成成功。混帧解析本身有 `tests/test_can_mixed_frames.py`。上游版本未标记，以后同步补丁时无法对照。

### 做得较完整的部分

这些不是问题，审阅时确认过，避免后续重复排查：

- 同向示教有启动和运行中的 CAN 年龄 / 故障码检查，以及速度守卫。
- KER 双扳机 2 秒预压、映射校验、输入超时停双臂；`tests/test_ker.py` 覆盖了无硬件的协议部分。
- 笛卡尔与骨骼模式的 Quest 超时会冻结并要求真实的重新握持；短时丢帧先冻结再重新接入。
- 点动 / 录制 UDP 只听回环。重力标定文档描述了验证集、单次倍率变化上限和失败不覆盖配置；离线工具 `openarm_gravity_model` 的注释写明不开 CAN。
- 初始化在使能后先发 `kp=0` 的中性帧，避免沿用进程被强杀之前的旧目标。`tests/test_openarm_init_transport.py` 锁的是「中性帧且 CAN FD 与 debug 开关无关」。
- 证书私钥被 gitignore。生成的 `build/`、`.venv`、`recordings/` 不进源码树。

## 3. 并行开发计划

三条线按目录切开，可以同时做。共享文件只在下面点名的那一条里改。

### A. 控制与安全

拥有：`src/safety/`、`src/controller/human_mimic_controller.cpp`、`src/controller/human_mimic_types.hpp`、`src/controller/control.cpp`、`src/controller/control.hpp`、`src/openarm_constants.hpp`、`src/openarm_port/`、`control/openarm_zero_g.cpp`、`control/openarm_gravity_calib.cpp`、`control/openarm_quest_cartesian_control.cpp` 里跟随线程和退出路径、`ros2/src/openarm_moveit_config/scripts/real_arm.cpp`。

不改：`quest/`、`src/io/`、`docs/`、`tests/`（测试由 C 线补；A 只保证行为可被无硬件测试观察）。

| 任务 | 规模 | 依赖 | 完成标准 |
| --- | --- | --- | --- |
| A1. 修正 human_mimic 的重新握持。超时和 IK 冻结期间保持 `last_grip_` 与真实 Grip 一致；只有 Grip 松开后再按下才调用 `activate_clutch`。不要在冻结分支里把 `solve_fail_count_` 清零。 | S | 无 | 单测或离线夹具：Grip 按住穿过 150 ms 超时再恢复时离合保持断开；松开再按下才跟踪。IK 失败计数在 Grip 未松开时保持在阈值以上。 |
| A2. 抽出与同向示教相同语义的 CAN 反馈检查（年龄、从未收到、`error_code`），用到遥操 `SeedHoldMeasured`、零重力、重力标定运动和 `real_arm.cpp`。无新鲜反馈时不得发带 Kp 的保持，也不得按 `q=0` 算重力。运行中超过约定年龄则停止下发并走现有失能路径。 | M | 需要负责人确认年龄阈值和错误码含义（见问题 Q1、Q2）。在确认前可用同向示教的 150 ms 和 `error_code > 1` 作为临时同一标准，并在日志里写明。 | 伪造「位置一直为 0 且时间戳为空」的电机时，三条启动路径拒绝进入出力循环。反馈中途过期时循环退出并调用 `disable_all()`。同向示教原测试仍然通过。 |
| A3. 让 `SafetyFilter` 使用位置、速度和首步跳变，或删掉这些字段并改掉「MIT 前有安全层」的文件头注释。跟随路径增加 `|q_des - q_meas|` 上限，因为力矩发生在电机内部。给 `velocity_limit` / `effort_limit` 恢复非零值，或保留 0 但改成明确的「未启用」配置并让仿真和真机走同一解释。`SafetyMode::Stop` 要么接到停止路径并真正发零速度保持 / 失能，要么删除。 | M | A2 的停机路径。位置数必须来自 URDF，不要启用 `openarm_constants.hpp` 里更宽的肘上限，除非 Q3 另有决定。 | 单元测试：超限参考被夹紧；非有限值保持上一帧；Stop 会发一帧明确的安全命令而不是只 `return true`。真机 Kp 弹簧的跟踪误差有上限，且上限被测试锁住。 |
| A4. 跟随线程恢复周期超时策略：超时后保持上一参考并告警，而不是追拍。需要实时优先级时显式调用基类设置，失败时打印一次。 | S | 无 | 人为拉长 `on_timer` 时，后续拍的 `dt` 不按 1 ms 连补；日志能看到超时。 |

### B. Quest 与 ROS 2 集成

拥有：`quest/`、`src/io/`、`ros2/` 中除 `scripts/real_arm.cpp` 以外的文件（该文件归 A）、`scripts/run_moveit.sh` 的启动参数面。

不改：`src/safety/`、`human_mimic_controller.cpp`、`control.cpp`、`openarm_quest_cartesian_control.cpp`。姿态认证若要在 C++ 接收端核对令牌，把核对放在 `src/io/*_receiver.cpp`，不要改控制律。

| 任务 | 规模 | 依赖 | 完成标准 |
| --- | --- | --- | --- |
| B1. 姿态 UDP 默认只绑 `127.0.0.1`。若必须从另一台机器注入，用环境变量打开，并拒绝无共享令牌的包。WebSocket 使用同一令牌。 | M | Q4：控制电脑和 Quest 是否永远在同一台主机旁，令牌如何交到头显。 | 默认构建下，从非回环地址向 47911 发包不会更新 `latest()`。错误令牌被丢弃。现有 `tests/test_quest_session.cjs` 在本机回环上仍能走通，或被改成携带令牌。 |
| B2. HTTP 文件服务先 `realpath`，拒绝跑出 `quest/` 目录。 | S | 无 | 请求含 `..` 时返回 404，且不会打开 `quest/` 以外的文件。正常 `client.html` 仍返回 200。 |
| B3. 把 MoveIt 关节速度、加速度和 URDF 位置限值对一下，在 `joint_limits.yaml` 或启动日志里写明实机执行用的是哪一组。不要在这条线里改 `real_arm.cpp` 的力矩（归 A2/A3）。 | S | A 线合并前不要改 `real_arm.cpp`。 | 文档或启动日志列出 7 个关节的位置上下限来源。规划速度仍不超过现有 2 rad/s，除非 Q5 改指标。 |
| B4. 骨骼丢帧策略与 `docs/Quest肩肘骨骼追踪.md` 对齐并加一条桥侧测试：短丢帧冻结、超时后必须重新握 Grip。逻辑已在 `SkeletonProximalController`，测试放在 B 能触达的 Python/Node 桥和接收器；控制器夹具如果必须改 `src/controller/`，把该测试留给 C，避免和 A 抢文件。 | M | A1 不修改骨骼控制器，故无代码冲突。 | 桥或接收器测试覆盖年龄超过 `pose_max_age_ms`（默认 250）时样本不可用。文档中的端口和重新握持与代码一致。 |

### C. 测试、CI 与文档

拥有：`tests/`、`.github/`（或仓库选定的 CI）、`docs/`、`README.md`、`AGENTS.md`、`install.sh`、`requirements.txt`、`run.sh` 的默认 XML 路径、`CMakeLists.txt` 里仅测试目标注册。

不改：控制律和 CAN 发送。为 A/B 写的测试只链接现有接口或复制「可观察的纯函数」。若 A 尚未合并，C 可以先提交失败被标记的测试草稿，但不要为了让测试编译去改 `src/controller/`。

| 任务 | 规模 | 依赖 | 完成标准 |
| --- | --- | --- | --- |
| C1. CI 在干净环境安装 README 的构建依赖和锁定后的 Python 依赖，配置 CMake，编译 `quest_cartesian_control`、`openarm_gravity_model`、`skeleton_control_smoke`，并运行 `unittest discover -s tests` 与两个 `.cjs`。 | L | C2 的锁文件。不依赖真 CAN、显示和 ROS，MoveIt 作业单独标成可选。 | 一次公有 CI 记录为绿。失败日志能区分缺 apt、缺 Python 包和测试失败。 |
| C2. 给 `requirements.txt` 生成锁，或把下界收成已验证的上限。`install.sh` 在缺少 cmake / eigen / kdl 时打印 README 里的 apt 行并以非零退出。`run.sh` 默认 XML 改为 `models/OpenArm_V1/scene_table.xml`，仍允许 `OPENARM_MUJOCO_XML` 覆盖。 | M | Q6：默认场景用桌面场景还是空场景。 | 新克隆在设置了该默认值后，`run.sh --sim` 能过「XML 存在」这一关（未验证图形显示）。锁文件使两次安装的 `mujoco` 主版本一致。 |
| C3. 为 H1、H2、H3 补无硬件测试。human_mimic 若还不能单独链接，就在 A1 合并后追加；在此之前先为 `SafetyFilter` 和「反馈时间戳为空则拒绝保持」写测试，驱动 A 的接口而不是复制一份算法。 | M | A1–A3 的最终函数签名。 | CI 中这些测试失败能对应到回归。测试不打开 SocketCAN。 |
| C4. 把本文的数据流、模式与看门狗对照表、软件停止和硬件急停的差别收进 `README.md` 的短节，并删掉或更正 `AGENTS.md` 里「没有 git 历史」的句子。记录 `third_party/openarm_can` 对应的上游修订。 | S | Q7：若上游修订号负责人还不掌握，文档写「未标记」而不是猜。 | README 能回答：丢 Quest、丢 CAN、按停止键分别发生什么。新同事不需要读 5486 行入口才能知道默认模式。 |

建议合并顺序：C2 的路径修正和 C4 的文档可以马上做。A1 独立且小。A2/A3 与 B1 互不修改同一文件。C1 在 C2 之后。C3 跟在 A 的接口稳定之后。

## 4. 需要负责人决定的问题

1. **Q1.** 达妙电机现在的掉线行为是什么？固件 `TIMEOUT` 是否已写成失能或阻尼？软件看门狗应该在它之前多久动作？
2. **Q2.** `error_code > 1` 是否就是同向示教想拒绝的全部故障？0 和 1 在现用固件里分别代表什么？
3. **Q3.** 肘关节机械限位以 URDF 的 2.443 rad 为准，还是以 `openarm_constants.hpp` 的 π 为准？其他关节是否也有「软件常数宽于 URDF」需要保留的原因？
4. **Q4.** Quest 是否总是和控制器在同一台电脑、同一信任网络上？姿态通道要不要令牌，还是只绑回环就够？
5. **Q5.** MoveIt 实机 2 rad/s、遥操 S 曲线默认 0.6 rad/s、同向示教 1.5 rad/s，哪一个是可接受的最大关节速度？
6. **Q6.** 仿真默认场景是否改为仓库内 `models/OpenArm_V1/scene_table.xml`？该 XML 是否与真机桌面启动轨迹 `assets/steady_action_on_table.csv` 是同一套几何？
7. **Q7.** `third_party/openarm_can` 拷贝自哪一次上游提交？以后是否改为子模块？
8. **Q8.** 现场硬件急停是否切断电机电源，并且在软件死锁、CAN 适配器消失和进程被强杀时仍然有效？控制台是否要增加一个真正调用 `disable_all()` 的软件急停，而不仅是结束子进程？
9. **Q9.** `velocity_limit` / `effort_limit` 全 0 的「暂时解锁」是否仍是当前要求？若是，解锁到何时、由谁收回？
10. **Q10.** 默认 human_mimic 在 Quest 短时恢复后自动续上，是否曾经是有意的操作习惯？代码注释写的是必须重新握持。确认意图后再改 A1，避免「修好」一条操作员已经依赖的行为。

## 5. 这次没有验证的事项

- 没有编译 C++，没有运行 `unittest`，没有打开 MuJoCo、RViz 或 Qt。
- 没有 CAN 适配器、电机、Quest 头显和 ROS 2 Humble，因此丢总线后电机的真实反应、急停回路、左右 CAN 默认 `can1` / `can0` 是否仍是布线，都未在硬件上确认。
- 没有测量 1000 Hz 线程在真实总线上的超时比例。
- H1 的重新握持是控制流阅读结论，没有步进调试器跑过该函数。
- H4 / M3 没有对运行中的桥发包或请求路径。
