# OpenArm VR 遥操作

从 Motion Trajectory Studio 拆出的 Quest 3 / OpenArm 双臂遥操作。不依赖编辑器，仿真和真机都从本目录启动。

默认控制模式是 `human_mimic`，姿态映射是绝对标定：第一次握住 Grip 锁定手柄到夹爪的朝向，之后松开再握只重锚位置。握住 Grip 后同时按 A+B（右）或 X+Y（左）可以重新标定朝向。

## 依赖

```bash
sudo apt install build-essential cmake libeigen3-dev liborocos-kdl-dev \
  libkdl-parser-dev liburdfdom-dev libyaml-cpp-dev libglfw3-dev \
  libgl1-mesa-dev can-utils openssl
./install.sh
```

`install.sh` 会在本目录创建 `.venv`，安装 `mujoco` 和 `websockets`。

## 运行

```bash
./app.sh            # Qt 控制台（推荐）
./run.sh            # 直接开 MuJoCo 仿真 + Quest WebXR 桥
./run.sh --real     # 直接开真机双臂，默认 can1=左、can0=右
./scripts/run_moveit.sh --sim    # MoveIt 仿真：RViz 规划，模型跟着走
./scripts/run_moveit.sh --real   # MoveIt 实机：规划后执行到 CAN 双臂
./scripts/run_zero_g.sh          # 零重力：重力补偿托住双臂，可徒手慢拖
./run.sh --mirror   # 右臂徒手示教，左臂同向跟随（含夹爪），无需 Quest
./scripts/run_gravity_tune.sh    # 单独打开关节微调窗口
./run.sh --no-bridge
```

### `./app.sh` 控制台功能（PySide6 / Qt6）

| 分区 | 功能 |
| --- | --- |
| **模式** | 仿真遥操 · 真机遥操 · MoveIt · 零重力 · 同向示教 |
| **连接** | 左右 CAN / 波特率、急停确认、Quest 地址 |
| **相机 USB** | 头/左腕/右腕口选择、刷新、三路预览 |
| **启停** | 启动 / 停止并释放；读取电机参数（不使能） |
| **重力补偿** | **自动微调**（在已有方案上采数、验证后调整倍率/偏置）· **手动微调**（逐关节调节、恢复模板、热加载和保存） |
| **数据采集** | 仿真/真机遥操及同向示教：开始/停止/成功/失败/重录；**数据管理…** 浏览/删除/改标签/重采 |
| **日志** | 当前子进程输出 |

MoveIt：不勾急停 = 仿真（只动 RViz）；勾选后配 CAN 并执行到真机。MoveIt 与零重力不录遥操数据，需 ROS 2 Humble + MoveIt 2。

RViz 默认使用系统图形驱动，显示刷新率设为 30 FPS。若报 `Unable to create a suitable GLXContext`，先检查 NVIDIA 驱动：`cat /sys/module/nvidia/version` 是当前运行版本，`modinfo -F version nvidia` 是磁盘模块版本；更新驱动后两者不一致时，应先停止并释放机械臂，再重启电脑加载新模块。临时排查可用 `OPENARM_MOVEIT_RVIZ_RENDERER=software ./scripts/run_moveit.sh --sim`，仅 RViz 使用 Mesa 软件渲染，显示性能会降低。日志出现 `You can start planning now!` 表示规划后端已就绪；若同时报 GLXContext 错误，失败的是 RViz 显示窗口。

「零重力」使用 KDL `G(q)` 与 `gravity_comp.yaml`。自动标定默认在当前可用参数上精修关节重力倍率/偏置：保留质量/质心、夹爪、相机、已知负载、远端增益、摩擦和前馈增益，不先清空参数。

固定模板为 `config/inf.backup`。界面只保留自动微调和手动微调，按钮均带悬停说明。手动窗口的“恢复模板”载入左右臂及全局可调参数，执行热加载或保存后生效；其余质量/质心参数保留。“热加载试调”也会写入当前配置，关闭窗口时可选择回退；“永久写入”保存为后续基准。模板文件不会被修改。

```bash
./scripts/calibrate_gravity.py                 # 航点停稳采数 + 自动精修
./scripts/calibrate_gravity.py --skip-motion --dry-run  # 新版日志离线预览，不动臂、不写参数
./scripts/tune_gravity_comp.py --post-id --dry-run      # 同一 KDL 模型残差验证
./scripts/calibrate_gravity.py --identify --seed       # 显式旧版质量/质心实验路径
```

默认精修调用离线 C++ `openarm_gravity_model`，直接复用零重力的 Dynamics 与 YAML 解释，包含夹爪电机、相机、末端位置和逐关节远端增益。采数记录实际 `tau_g` 与 `ff_gain`；目标为 `tau_g + Kp·e + Kd·ė`，低速下仍可能受未建模静摩擦影响。不会再额外乘 0.95 折扣；保存的倍率/偏置显式计入保留的前馈增益。

拟合按整段姿态分出验证集，每阶段至少3个有效姿态、每个至少15个静止样本，两臂阶段均须完整。用位置差分速度筛选（≤0.05 rad/s）；缺乏激励的关节保留原值。单次倍率变化最多20%、偏置最多0.08 Nm，只有未参与拟合姿态的误差改善且未明显恶化任一点才采用候选。此验证检查力矩残差，不保证实际松手漂移或拖动力；仍需实机验收。

采数中止、旧日志缺少 `tau_g/ff_gain`、物理模型不匹配、拟合失败或配置被同时修改时，不覆盖现有配置。成功写回前保留备份，并原子替换文件。静止采数不可靠地辨识运动摩擦，默认保持已有摩擦；旧完整质量/质心辨识仍是实验选项，不能等同于默认精修。

**同向示教**：右臂使用当前重力补偿，可单手拖动；左臂肩部前后摆臂 j1、腕部上下俯仰 j7 的角度和速度取右臂反号以修正转轴方向，其余对应关节同号，目标受左臂自身限位约束，夹爪同向。启动不回零、不播放桌面启动轨迹，左臂先慢速对齐，再以 200 Hz 使用右臂实测速度前馈随动。随动上限为 1.5 rad/s、6 rad/s²，夹爪为 2 rad/s、8 rad/s²。夹爪目标结合左侧反馈限制领先距离，闭合受阻后柔和保持，回开即释放，正常夹持不会直接中止双臂。读取当前 `gravity_comp.yaml`，支持手动微调热加载。

选择同向示教 → 确认 CAN 和安全条件 → 启动 → 等待左臂对齐 → 开始录制 → 单手拖动右臂及夹爪 → 停止录制并标记成功/失败。完整操作和数据字段见 [同向示教与数据采集](docs/同向示教与数据采集.md)。

**OpenArm KER 示教**：控制台新增“KER 仿真 / KER 真机”，支持 USB 或串口 KER 操作当前 V1 双臂及夹爪，录制后可自动回放。双扳机必须同时压下并连续保持 2 秒，才会使能从臂并慢速对齐。先运行 `bash scripts/install_ker.sh` 安装独立 SDK 环境，再在仿真核对 `config/ker.yaml` 的关节方向、零点及夹爪行程；真机须完成映射标定。无设备可用 `./run.sh --ker-demo` 演示。详见 [KER 示教与回放](docs/KER示教与回放.md)。

**动作回放（Quest 遥操 / KER / 手拖示教）**：停录并标注，停止并释放控制器后，点击“动作回放…”或“数据管理 → 回放此条”，可让双臂及夹爪自动重放录制动作，回放时无需输入设备在线。支持仿真/真机、0.25/0.5/1 倍、暂停/继续、停止释放；先慢速接入首帧，结束后保持末帧。操作和文件要求见 [动作录制与轨迹回放](docs/拖拽示教与轨迹回放.md)。

**Quest 手柄采集 / 循环回放**：通过 `./app.sh` 启动并填写批次名后，左摇杆短按开始/停止录制，停录后右摇杆短按成功、长按失败，左摇杆长按丢弃并重录（长按 1 秒后松开）。A/B、X/Y 保持夹爪功能。回放窗口可设播放次数，0 为持续循环；每遍之间保持使能并慢速返回首帧，须先在仿真核对返回路径。

日志：`recordings/gravity/`；电机参数：`recordings/motor_params/`。真机启动会调 `scripts/configure_can.sh`（默认左 `can1`、右 `can0`，1M / 5M CAN FD），需 sudo。

仿真会打开 MuJoCo 窗口。Quest 浏览器打开 `https://<这台电脑的IP>:8443`（证书在 `quest/`）。左手 UDP `47912`，右手 `47911`。

**肘部与腕部追踪**：控制台勾选“Quest 肘部与腕部追踪”，或使用 `./run.sh --skeleton --sim`。持续追踪肘部和腕部，肩点仅在进入沉浸模式时校准；需要浏览器提供 WebXR 身体关节数据。短暂丢帧冻结并重新接入，持续丢失后需重新握持 Grip。逆解限制约 60 Hz，关节参考仍按原控制频率更新。先验证仿真，操作与限制见 [Quest 肩肘骨骼追踪](docs/Quest肩肘骨骼追踪.md)。

VR 网页仅用于手柄和肩肘骨骼追踪，不显示实时摄像头画面，也不会自动启动相机采集。沉浸模式不创建网页悬浮层，仅允许透视 AR；按 Quest 系统菜单退出沉浸模式后，可在网页停止追踪。头部及腕部相机仍可通过控制台独立预览，并沿用原有录制流程。

真机启动前先把 CAN 配好，并确认急停可用：

```bash
sudo ./scripts/configure_can.sh can1 can0 1000000 5000000
./run.sh --real
```

VR 真机遥操默认先沿 `assets/steady_action_on_table.csv` 进入桌面姿态，再接受 Grip。设 `OPENARM_QUEST_STARTUP_MOVE=0` 可跳过。

## 常用环境变量

| 变量 | 默认 | 作用 |
| --- | --- | --- |
| `VR_TELEOP_MODE` | `human_mimic` | `human_mimic` / `cartesian_precision` / `skeleton_proximal` / `pika_style` |
| `VR_ORIENTATION` | `absolute` | `absolute` 或 `incremental` |
| `OPENARM_LEFT_FOLLOWER_CAN` | `can1` | 左臂 CAN |
| `OPENARM_RIGHT_FOLLOWER_CAN` | `can0` | 右臂 CAN |
| `OPENARM_HM_CLUTCH_START_DEG` | `2` | 握持后转过这么多度才开始跟姿态 |
| `OPENARM_SCURVE_VMAX` | `0.6` | 启动/CSV/MoveIt S 曲线最大关节速度 (rad/s) |
| `OPENARM_SCURVE_AMAX` | `1.2` | S 曲线最大加速度 (rad/s²) |
| `OPENARM_SCURVE_JMAX` | `6` | S 曲线最大加加速度 (rad/s³) |
| `OPENARM_MOVEIT_SCURVE` | `1` | MoveIt 实机：段内 S 形加减速（保持原航点时刻）；`0` 回退线性 |

按钮示教不需要头显。向 `127.0.0.1:47931` 发 UDP：

```text
hold|release,<left|right>,<x+|x-|y+|y-|z+|z-|rx+|rx-|ry+|ry-|rz+|rz->
couple,on|off
```

平移和旋转都在机器人机体系。录制控制口是 `47930`。`./app.sh` 里用「数据采集」分区；`./run.sh` 额外打开 `http://127.0.0.1:8765` 网页面板。流程相同：开始 → 停止 → 标成功/失败，或重录（删本条、序号复用）。

**数据管理**（数据采集 →「数据管理…」）：浏览 `recordings/sessions/` 下批次与片段，删除片段/整批、改成功/失败、手动导出；遥操运行中可对当前批次 **重采此条**（UDP `rerecord,<batch>,<NN>`，下次开始复用该序号）。录制中或待标注时禁止改当前批次。

打过标签的片段会导出为 [OpenArm Dataset v0.4](https://docs.openarm.dev/dataset/)（`recordings/sessions/<batch>/openarm_dataset/`），并尽量转成 LeRobot（`.../lerobot/`）：

```bash
# 手动重导某个 batch（遥操标完成功/失败后也会自动跑）
.venv/bin/python script/openarm_dataset_export.py --batch recordings/sessions/<name>

# 已安装 openarm-dataset 时也可用官方工具
openarm-dataset-convert recordings/sessions/<name>/openarm_dataset /tmp/lerobot \
  --format lerobot_v3.0 --fps 30
```

可选：`pip install openarm-dataset`（官方读/写与 `openarm-dataset-convert`）。未安装时本地会用 `lerobot_collect.py` 从 CSV 回退生成 LeRobot。`OPENARM_LEROBOT=0` 跳过导出；`OPENARM_LEROBOT_TASK` / `OPENARM_LEROBOT_FPS` / `OPENARM_LEROBOT_REPO` 可改任务名、帧率和 repo id。

日志写在 `recordings/hm_logs/`。Apache-2.0，见 `LICENSE-APACHE` 和 `third_party/openarm_can/LICENSE.txt`。

自动标定离线回归检查：`.venv/bin/python -m unittest discover -s tests -v`（不启用 CAN）。
