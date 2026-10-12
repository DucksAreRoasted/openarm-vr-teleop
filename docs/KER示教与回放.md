# OpenArm KER → OpenArm V1 示教与回放

KER 作为独立输入设备，经 USB（默认）或串口读取双臂编码器，驱动当前项目的 V1 双臂及夹爪。无需 Quest 在线。控制器复用现有重力补偿、反馈检查、关节限位、夹爪柔和保持及轨迹录制；没有新增 OpenArm 2.0 动力学或硬件配置。

## 安装

官方 `openarm-ker==0.3.0` SDK 要求 Python 3.11+。现有 Qt/C++ 环境可继续使用 Python 3.10；单独安装 KER 环境：

```bash
bash scripts/install_ker.sh
.venv-ker/bin/openarm-ker-cli ping
```

脚本优先用 `uv` 创建 Python 3.12 的 `.venv-ker`，没有 `uv` 时使用 `KER_SETUP_PYTHON`（默认 `python3.12`）。启动器自动选用 `.venv-ker/bin/python`，也可通过 `OPENARM_KER_PYTHON` 指向其他装有 SDK 的 Python。

USB 连接依赖 libusb 和设备访问权限；按[官方 SDK 文档](https://github.com/enactic/openarm_ker)配置本机。启动器不会修改系统 USB 权限、固件或编码器零点。

若日志出现 `[Errno 13] Access denied (insufficient permissions)`，表示系统已发现设备，但运行程序的用户没有 USB 访问权限。在项目目录执行一次：

```bash
sudo bash scripts/configure_ker_usb.sh
.venv-ker/bin/python script/ker_bridge.py --probe
```

脚本为调用 `sudo` 的普通用户安装 `/etc/udev/rules.d/70-openarm-ker.rules`，仅匹配 KER 正常模式的 `303a:4002`，授予该用户读写权限并重新触发对应设备规则；不需要以 root 运行整个控制台。如仍被拒绝，重新插拔 KER USB 后重试。可先用 `bash scripts/configure_ker_usb.sh --print-rule "$USER"` 查看规则内容。`--probe` 仅读取设备元数据并关闭连接，不启动动作流或从臂；启动器也会在编译和启动从臂前进行这项检查。

## 标定 V1 映射

配置位于 `config/ker.yaml`，也可通过 `OPENARM_KER_CONFIG` 指向另一份文件。默认 KER 通道顺序参考[官方 Dora KER 节点](https://github.com/enactic/dora-openarm-ker)：右臂 0～6、右夹爪 7，左臂 8～14、左夹爪 15；编码器输入单位为度。

每侧七关节的目标计算方式为：

```text
V1 目标角度 = signs × radians(KER 编码器角度 − zero_degrees) + offset_rad
```

夹爪通过 `gripper_open_degrees` 和 `gripper_closed_degrees` 归一化到 V1 的 −0.98（开）～0.62（闭）rad；仿真时由控制器转换成 0～0.044 m。

先在控制台点击 **“标定 KER 双扳机…”**（先停止并释放正在运行的控制器）。完全松开两侧扳机并保持不动半秒，点击采集松开位置；再将两侧扳机同时压到底并保持不动半秒，点击采集并保存。窗口实时显示左右角度，只启动 SDK 读数，不连接 UDP 控制器或 CAN。两个端点均使用多个新鲜样本的中位数，读数波动超过 2°、行程不足 5°或断流会拒绝采集/保存。标定后关闭窗口，再启动 KER 仿真并同时压到底保持 2 秒。

默认 ±60° 是初始估计，官方 SDK 并未承诺这个扳机行程。不能将日志中的某个静止角度直接当作压到底端点。标定会保存实际松开/压下角度到当前 `OPENARM_KER_CONFIG` 文件，并备份为同目录的 `.before-trigger-calibration` 后缀文件；不会修改七关节方向/零点或 `calibrated` 状态。按动扳机时读数没有变化会导致标定失败，此时应检查实物编码器，而不是调低启动阈值。

默认配置是待核对的起点，`calibrated: false`，会阻止真机启动。先连接 KER，选择控制台“KER 仿真”，逐关节核对左右通道、方向、零点以及夹爪开闭，调整配置；尤其要核对肘关节的零位和可动范围。确认与当前实物配对一致后，才将 `calibrated` 改为 `true`。更换设备或调整编码器零位后须重新核对。

双扳机确认前，超出 V1 URDF 限位的目标会保持电机未使能，并以 `WAITING_POSE` 打印左右臂、关节号、目标角度及限位范围，同时重置双扳机计时。调整到合法姿态后需重新保持双扳机 2 秒。已使能后的超限会终止跟随，不会静默裁剪大幅超限。桥接启动时的 `first_sample` 日志包含原始角度（度）和映射目标（弧度），可结合超限关节定位通道、方向或零点配置；不要通过放宽 URDF 限位绕过标定。

```bash
# 不接设备，仅检查映射配置
.venv/bin/python script/ker_bridge.py --check
# 真机配置检查，不连接 USB 或 CAN
.venv/bin/python script/ker_bridge.py --check --real
# 无 KER 硬件的仿真演示，不使用标定文件中的关节映射
./run.sh --ker-demo
# 接入真实 KER，驱动仿真 V1
./run.sh --ker --sim
# 标定完成、CAN 已配置后，驱动真实 V1
./run.sh --ker --real
```

命令行真机启动沿用原有要求：先核对急停及左右 CAN（默认左 `can1`、右 `can0`），并完成 CAN 配置。Qt 控制台选择“KER 真机”时沿用现有 CAN 配置和安全确认流程。

## 录制与回放

1. 选择“KER 仿真”或完成标定后的“KER 真机”，启动时保持 KER 手臂姿态不动，同时压下左右扳机并连续保持 2 秒。确认通过前从臂不使能；通过后才从实测位置慢速接入第一帧。
2. 出现 `TRACKING` 后，“数据采集”开放录制。填写批次，点击“开始录制”，操作 KER 双臂和夹爪。
3. 完成后停止录制，标记成功或失败，再点击主控制区“停止并释放”。
4. 打开“动作回放…”或“数据管理 → 回放此条”。回放读取已记录的 V1 下发目标，不再应用 KER 映射，也不需要 KER 在线。

片段仍保存在 `recordings/sessions/<批次>/<序号>/`。除现有分臂、合并 CSV 及观察/动作数据外，KER 片段还保存 `collection_mode.json`（采集方式、控制频率、夹爪单位）及 `ker_mapping.yaml` 标定快照。回放窗口根据 KER 元数据选择源夹爪单位，仍可手动调整。相机和数据集导出沿用现有采集流程。

控制频率 200 Hz；接入首帧使用 0.3 rad/s、0.6 rad/s²，正常跟随七关节使用 0.6 rad/s、1.2 rad/s²。快速移动 KER 时，从臂可能滞后，应按从臂实际位置操作。夹爪沿用回放控制器的限速与接触保持。

双扳机启动确认在 C++ 控制器中执行，对 KER 仿真和真机均强制生效。扳机使用配置中的左右夹爪通道，达到各自标定开→闭行程的 90% 判定为压下。两侧连续同时压下满 2 秒才通过；松开任一侧或有效输入间隔超过 250 ms 都会清零计时。收到有效数据但尚未通过确认时，可以持续等待，不会因等待超过 15 秒自动启动。

这是每次启动的确认门控，确认通过后会保持本次会话的启动状态，松开扳机可正常操作夹爪，不会停止跟随。断连退出或重启控制器后，必须再次完成双扳机确认。`--ker-demo` 会用仿真输入模拟双扳机压下，同样等待 2 秒。桥接协议使用带独立左右扳机状态的 `KER2`，旧 `KER1` 输入被拒绝，不能绕过确认。

输入桥接只向本机 `127.0.0.1:47932` 发送目标。控制器校验版本、完整列数、有限数值、序号、时间及关节限位，超过 250 ms 无有效新输入会停止并失能双臂。启动时 15 秒收不到有效输入则退出，不使能电机；对齐超时为 120 秒。桥接与控制器任一退出，启动器会停止另一端。断连后须检查连接并重新启动，SDK 自动恢复串流被关闭。

## 环境变量

| 变量 | 默认值 | 用途 |
|---|---|---|
| `OPENARM_KER_CONFIG` | `config/ker.yaml` | V1 映射配置 |
| `OPENARM_KER_PYTHON` | `.venv-ker/bin/python`，不存在则使用项目 Python | SDK 独立环境 |
| `OPENARM_KER_PORT` | `47932` | 本机桥接输入端口 |
| `OPENARM_KER_TRANSPORT` | `usb` | `usb` 或 `serial` |
| `OPENARM_KER_SERIAL_PORT` | `/dev/ttyACM0` | 串口设备 |

`OPENARM_MUJOCO_XML`、`OPENARM_GRAVITY_YAML` 和左右从臂 CAN 环境变量继续生效。

## 验证记录

- `cmake --build build --target quest_cartesian_control openarm_teaching_replay -j2`：通过；保留已有 `diff.hpp` 的有符号/无符号比较警告。
- `.venv/bin/python -m unittest discover -s tests -p 'test_ker.py'`：12 项通过，覆盖映射、标定拦截、数据校验、新旧帧、失联处理、权限错误提示、只读设备检查、双扳机门控、准备姿态超限重置计时、使能后超限停止及队列中的松开状态（USB/UDP 使用替身）。
- `.venv/bin/python -m unittest discover -s tests -p 'test_*replay*.py'`：9 项通过，包括 KER 模式、录制对齐门控和回放夹爪单位识别。
- `bash -n run.sh scripts/install_ker.sh`、Python 语法检查及独立 SDK 导入：通过。
- 使用 `/tmp/ker_recording_smoke` 离线测试程序直接调用控制与录制组件，在无渲染 MuJoCo 中完成合成 KER 目标跟随、录制、合并、成功标签、标定快照及保存轨迹回放，临时片段已清理。此项未经过 USB/UDP 链路，未验证相机或数据集导出。
- 双扳机改动后，使用 `/tmp/ker_recording_smoke` 验证合成时间戳达到 2 秒前门控保持关闭，确认通过后再创建仿真硬件并完成跟随和录制；日志保存在 `/tmp/ker-trigger-smoke.log`。真实扳机行程仍需按实物标定验证。

当前执行环境禁止创建 UDP socket，因此真实 KER 到控制器的通信链路未完成运行验证；USB 设备、V1 实物映射和真机力矩稳定性也未实测。现场应先用真实 KER 驱动仿真、逐关节完成标定，再进行真机验证。

## 仿真启动后没有反应

`WAITING_TRIGGERS` 表示正在等待双扳机连续压下 2 秒，不是已经进入跟随。仿真窗口在等待期间显示准备姿态；真机电机仍仅在安全门通过后使能。日志 `[ker-bridge] triggers` 每秒显示左右原始编码器角度、按标定行程计算的百分比与 `pressed=(左,右)`；两侧均须达到 90%。日志 `[ker] gate` 显示输入新鲜度、姿态校验和连续保持秒数。若一侧始终不到 90%，先核对 `config/ker.yaml` 中该侧扳机通道、松开/压到底角度及正负方向，不要通过关闭安全门绕过。`WAITING_POSE` 表示关节映射超出限位，需要核对方向和零点。进入 `ARMED` 后会慢速对齐，出现 `TRACKING` 才开始实时跟随。

若按动扳机而百分比始终不变，检查日志 `device_frame` 和后续 `device_seq`：序号递增只证明设备帧更新，不保证编码器位置更新。桥接会读取固件的 `error_mask` / `errors` 并在故障时停止；重复序号不会刷新控制心跳。没有故障标记但扳机仍不变化时，需要核对通道映射和 KER 本身的编码器读数，不能据此自动修改扳机行程。关节限位和扳机确认是两项独立条件，均通过才可启动。

标定窗口现在显示全部 16 通道的当前角度与累计变化，并把设备元数据、协议 schema、设备时间戳、原始角度和故障字段保存到 `/tmp/openarm_ker_inspect.jsonl`，每次打开窗口覆盖上次检测记录。保持关节不动，只反复按左扳机，观察 CH15；点击“清零通道变化统计”后只按右侧，观察 CH07。如果扳机对应通道不变、其他通道随按压变化，检查映射/接线，不能自动把其他关节当成扳机。如果所有通道都不随按压变化，但转动一个关节时该关节读数会更新，问题范围落在夹爪传感器、机械传动、接线或设备固件；没有故障标记不能排除这些问题。若设备时间戳不更新或出现断流错误，则先处理数据流连接。独立命令 `.venv-ker/bin/python script/ker_bridge.py --inspect` 也可采集同一份原始日志，不连接从臂。

### 扳机仅在手腕外撇时可读（2026-10-10 现场记录）

19:25 的只读采样包含 739 帧、37.47 秒。CH07 范围约 −61.44～0.49°，CH15 约 −0.10～61.80°，两侧扳机确实能够上报完整行程；随后控制器日志已出现 `ARMED` 和 `TRACKING`。因此不能再将这一现象归因于默认 ±60° 行程过大。

但采样开始后 8.283～11.890 秒期间，CH05～CH15 连续 3.607 秒完全不变，CH00～CH04 同期持续变化，设备时间戳前进 3,606,002；15.134 秒后另有约 1.520 秒的相同现象。所有帧的 `errors` 均为 false，只能说明固件未报告错误，不能证明下游编码器读数新鲜。结合“手腕外撇时恢复”的现场观察，优先排查右腕 CH05/J6 编码器附近及其下游线束、接插件、传感器机械连接和固件；仅凭这些日志还不能确定哪个部件损坏。官方 CLI 对串联 RS-485 编码器描述了单个模块不响应导致下游无法收到触发的情况，见 [上游说明](https://github.com/enactic/openarm_ker/blob/main/src/openarm_ker/cli.py)。

停止控制器、只开只读窗口，保持扳机压下并缓慢改变右腕姿态，观察 CH05、CH06、CH07 和左臂通道是否同时停止/恢复。若只是腕部运动、不按扳机就能令夹爪通道大幅变化，应另查机械耦合、编码器安装和映射。需要操作接插件时先断开 KER 供电，再检查右腕活动区域线束是否被拉紧、夹住或接触松动。确认读数在各种腕部姿态下连续更新后再进行遥操。

### FW 2.0.0 扳机已经按下但两秒确认失败

如果日志已经显示左右约 100%、`pressed=(1,1)`，随后出现无新数据超时，应检查输入流连续性。桥接现在对固定版本 SDK 0.3.0 的 USB 接收加入端点单包适配，避免一次请求 4096 字节造成积累或部分传输超时；协议帧仍由 SDK 自身的缓存与校验逻辑拼接。适配在连接前应用，握手与流式接收均有效。上游实现见 [KER SDK](https://github.com/enactic/openarm_ker/blob/main/src/openarm_ker/ker_stream.py)。

该固件样本提供 `timestamp` 而没有 `seq`，桥接会检测重复设备时间戳并记录 `device_timestamp`。静止时角度相同是正常情况；时间戳重复的帧不会累计到双扳机确认或刷新控制心跳。启动日志应出现 `USB read mode: endpoint-sized packets`；更新后停止并重新启动 KER 仿真即可加载。仍保留双扳机至少 90% 且连续 2 秒、关节限位及断流停止行为。

回归命令：`.venv-ker/bin/python -m unittest discover -s tests -p test_ker.py`。覆盖 USB 单包读取、FW 时间戳去重和模拟 USB 数据经真实 SDK 解析器、桥接映射、C++ 仿真安全门的完整两秒确认流程；不连接真实 USB 或 CAN。

本次扳机行程标定验证：上述回归共 22 项通过，包含非 ±60° 行程、反向行程、静止/不稳定读数拒绝、只读模式不创建 UDP，以及实测形状的行程数据经 SDK 解析到 C++ 两秒门控。Qt 窗口离屏验证采集与保存流程。由于执行环境无法访问 KER USB，实物端点仍须由操作人员在标定窗口采集，不能声称硬件问题已经验证解决。
