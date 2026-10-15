# OpenArm VR 遥操作

从 Motion Trajectory Studio 拆出的 Quest 3 / OpenArm 双臂遥操作。不依赖编辑器，仿真和真机都从本目录启动。

默认控制模式是宇树参考 `unitree_style`，也可选择 PIKA 参考 `pika_style`，姿态映射是绝对标定：第一次握住 Grip 锁定手柄到夹爪的朝向，之后松开再握只重锚位置。握住 Grip 后同时按 A+B（右）或 X+Y（左）可以重新标定朝向。

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
./run.sh --print-paths   # 只打印 URDF 和仿真场景路径，不编译、不启动
./run.sh --real     # 直接开真机双臂，默认 can1=左、can0=右
./scripts/run_moveit.sh --sim    # MoveIt 仿真：RViz 规划，模型跟着走
./scripts/run_moveit.sh --real   # MoveIt 实机：规划后执行到 CAN 双臂
./scripts/run_zero_g.sh          # 零重力：重力补偿托住双臂，可徒手慢拖
./run.sh --no-bridge
```

### `./app.sh` 控制台功能（PySide6 / Qt6）

| 分区 | 功能 |
| --- | --- |
| **模式** | 仿真遥操 · 真机遥操 · MoveIt · 零重力 |
| **连接** | 左右 CAN / 波特率、急停确认、Quest 地址 |
| **相机 USB** | 头/左腕/右腕口选择、刷新、三路预览 |
| **启停** | 启动 / 停止并释放；读取电机参数（不使能） |
| **重力补偿** | ① **使用 URDF 初值**（不动臂，恢复模型基准）→ ② **自动微调**（动臂两侧采数，调整倍率/偏置）→ ③ **惯量标定**（不动臂，用②的日志辨识质量/质心）；均直接覆盖当前配置，不另存备份 |
| **数据采集** | 仿真/真机遥操：开始/停止/成功/失败/重录；**数据管理…** 浏览/删除/改标签/重采 |
| **日志** | 底部实时日志，可拖动高度或打开独立大窗口；级别配色、过滤、查找、暂停、错误计数和完整文件归档，见[使用说明](docs/实时日志使用说明.md) |

重力补偿学习入口：[重力补偿原理与实践教程](docs/重力补偿原理与实践教程.md)，包含模型计算、MIT 前馈、各模式差异、标定方法和负载下垂排查。

MoveIt：不勾急停 = 仿真（只动 RViz）；勾选后配 CAN 并执行到真机。选中 MoveIt 后出现「软硬」区：启动前选正常刚度、中等刚度、偏低刚度或接触柔顺。真机运行中可切换、手拖或软急停（保持使能）。停止并释放仍会失能，手臂会下落。MoveIt 与零重力不录遥操数据，需 ROS 2 Humble + MoveIt 2。

P3 使用低刚度、`integral: off`，不自动积累静差补偿。此前试验的缓慢积分补偿已撤回：低刚度下可能出现蓄力突破静摩擦、滑动后再积累的反复抖动。P3 仍可能存在摩擦或重力补偿残差造成的回位误差，需要按关节检查；目前需要位置保持时优先使用已验证的 P1。到位阈值是执行容差，不代表最终精度。

MoveIt 实机的重力项现在与零重力共用 `g_scale` / `ff_gain`，启动和热加载均使用同一标定输出，模型尊重夹爪/相机安装标记与显式零摩擦；`tracking_g_scale` / `moveit_g_scale` 仍供 Quest 使用，不再额外放大 MoveIt 重力。新 MoveIt 轨迹会清空普通积分、冻结积分和衰减状态，避免 P2 恢复上一姿态的偏置；同一姿态从 P1 切到 P2 仍保留冻结补偿。

P2/P3 默认只给 J6 添加平滑位置弹簧 `restore_tau_max * tanh(error / restore_width)`，上限分别为 0.12/0.20 Nm，宽度 0.025 rad；零误差时额外力矩为零，持续受推时不会随时间积累。J6 的局部位置刚度分别约为 11.8/12 Nm/rad，误差较大时额外力矩饱和，目标附近会比原来稍硬。额外阻尼分别为 0.10/0.15 Nm·s/rad（最终 kd 均为 0.30）；所有新参数随切档平滑变化，软保持/手拖立即移除额外弹簧。可在 `impedance_profiles.yaml` 中把该档位的 `restore_tau_max` 和 `restore_kd` 全设为 0 后重启，恢复原来的弹簧/阻尼行为。日志增加 `spring1..7`（限幅前请求值）、实际 `gravity_gain`，`kd1..7` 包含额外阻尼。有限恢复力仍可能留下静摩擦误差；仿真通过不代表真机已验证，首次需检查 J6 轻推释放、持续受推及静止保持。

RViz 默认使用系统图形驱动，显示刷新率设为 30 FPS。若报 `Unable to create a suitable GLXContext`，先检查 NVIDIA 驱动：`cat /sys/module/nvidia/version` 是当前运行版本，`modinfo -F version nvidia` 是磁盘模块版本；更新驱动后两者不一致时，应先停止并释放机械臂，再重启电脑加载新模块。临时排查可用 `OPENARM_MOVEIT_RVIZ_RENDERER=software ./scripts/run_moveit.sh --sim`，仅 RViz 使用 Mesa 软件渲染，显示性能会降低。日志出现 `You can start planning now!` 表示规划后端已就绪；若同时报 GLXContext 错误，失败的是 RViz 显示窗口。

「零重力」使用 KDL `G(q)` 与 `gravity_comp.yaml`。自动标定默认在当前可用参数上精修关节重力倍率/偏置：保留质量/质心、夹爪、相机、已知负载、远端增益、摩擦和前馈增益，不先清空参数。

「自动微调」的肩部轨迹包含垂直附近 4 个姿态（J1 前后小角度、不同肘部角度），每个姿态仍从正反两侧接近。垂直中心姿态独立留出验证；新采数缺少任一垂直姿态的双向停稳数据时不覆盖配置。移动速度保持初始同步 0.10 rad/s、姿态间 0.20 rad/s；优先选择较近的接近侧。硬保持确认双臂停稳且跟踪误差足够小时可提前结束；软持采数至少 1.2 秒，只有双臂位置与保持力矩连续 1 秒稳定才提前结束，否则采满原来的 3 秒。理想停稳时，32 姿态主体约 8.3 分钟（原 28 姿态约 10 分钟）；不含初始同步、编译和拟合，停稳较慢时会延长。必须清空标定工作区，遥操场景里的桌子不属于标定轨迹可避让的障碍。

可用 `build/openarm_gravity_calib --preview-plan` 导出轨迹 JSON，此命令不连接 CAN、不驱动电机、不写采数日志。`scripts/calibrate_gravity.py --dry-run` 仍会驱动机械臂采数，只是不写配置；仅 `--skip-motion --dry-run` 是已有日志的离线预览。

控制台默认左右夹爪均已安装。拆下夹爪时，在 `gravity_comp.yaml` 中把 `left_gripper_present` / `right_gripper_present` 设为 `false`（缺省 `true`，重启生效，仅零重力和自动微调读取）：该侧只注册 7 个臂关节，模型去掉夹爪电机、指尖及夹持负载质量，腕部相机补偿保留。

**惯量标定**：在最近一次自动微调采到的两侧接近日志（`recordings/gravity/gravity_*.csv`，含 `approach` 列）上，辨识各连杆质量倍率与质心偏移，不动臂。点击后先显示当前配置、URDF 原值和本次辨识的留出姿态误差对比；确认后才写入（直接覆盖当前配置，远端增益/倍率/偏置归一），下次启动生效。命令行等价于 `scripts/identify_gravity.py <日志> [--dry-run]`。

控制器停止后点击“使用 URDF 初值”，会从 `OPENARM_TELEOP_URDF`（默认 `assets/v1.urdf`）离线计算名义重力，直接覆盖当前 `gravity_comp.yaml`：恢复质量/质心与逐关节重力修正为 URDF 模型初值，前馈增益设为 0.95。夹爪/相机安装状态、负载和摩擦参数保留；无需连接机械臂，也不会驱动机械臂，下次启动生效。“使用 URDF 初值”“自动微调”“惯量标定”都以当前 `gravity_comp.yaml` 为起点（可用 `OPENARM_GRAVITY_YAML` 指向其他配置）。界面不再提供手动微调窗口；个别参数需要修改时直接编辑 YAML，下次启动生效。

```bash
./scripts/calibrate_gravity.py                 # 航点停稳采数 + 自动精修
./scripts/calibrate_gravity.py --skip-motion --dry-run  # 新版日志离线预览，不动臂、不写参数
./scripts/tune_gravity_comp.py --post-id --dry-run      # 同一 KDL 模型残差验证
./scripts/calibrate_gravity.py --identify --seed       # 显式旧版质量/质心实验路径
```

默认精修调用离线 C++ `openarm_gravity_model`，直接复用零重力的 Dynamics 与 YAML 解释，包含夹爪电机、相机、末端位置和逐关节远端增益。采数记录实际 `tau_g` 与 `ff_gain`；目标为 `tau_g + Kp·e + Kd·ė`，低速下仍可能受未建模静摩擦影响。不会再额外乘 0.95 折扣；保存的倍率/偏置显式计入保留的前馈增益。

拟合按整段姿态分出验证集，每阶段至少3个有效姿态、每个至少15个静止样本，两臂阶段均须完整。用位置差分速度筛选（≤0.05 rad/s）；缺乏激励的关节保留原值。单次倍率变化最多20%、偏置最多0.08 Nm，只有未参与拟合姿态的误差改善且未明显恶化任一点才采用候选。此验证检查力矩残差，不保证实际松手漂移或拖动力；仍需实机验收。

采数中止、旧日志缺少 `tau_g/ff_gain`、物理模型不匹配、拟合失败或配置被同时修改时，不覆盖现有配置。成功时原子替换文件，不另存备份；需要保留某版参数请自行复制 `gravity_comp.yaml`。静止采数不可靠地辨识运动摩擦，默认保持已有摩擦；旧完整质量/质心辨识仍是实验选项，不能等同于默认精修。

**动作回放（Quest 遥操）**：停录并标注，停止并释放控制器后，点击“动作回放…”或“数据管理 → 回放此条”。先点击“到达起手位置”，双臂完整执行你的 Quest 起手轨迹，再平滑衔接到录制首帧并保持；到位后再点击“开始回放”，无需输入设备在线。支持仿真/真机、0.25/0.5/1 倍、暂停/继续、停止释放；仿真按录制时间预览，真机受安全限速约束。结束后保持末帧，可“返回录制首帧”后再次手动开始。操作和文件要求见 [动作录制与轨迹回放](docs/动作录制与轨迹回放.md)。

**Quest 跟随方式**：控制台可选择宇树参考（默认）或 PIKA 参考；先使用仿真核对。命令行可用 `VR_TELEOP_MODE=unitree_style ./run.sh --sim`。支持同一合成输入下的双臂对照测试，见 [模式说明与测试](docs/Quest宇树参考模式.md)。

**Quest 手柄采集 / 循环回放**：通过 `./app.sh` 启动并填写批次名后，左摇杆短按开始/停止录制，停录后右摇杆短按成功、按住 1.5 秒标记失败。左摇杆按住 1.5 秒把当前条移到 `recordings/trash/`（按住时轻振，松开可取消），10 秒内再按住或点「撤销丢弃」可恢复。A/B、X/Y 保持夹爪功能。回放窗口可设播放次数，0 为持续循环；仿真每遍直接回到首帧；真机每遍之间保持使能并慢速返回首帧，需单独检查返回路径。

日志：`recordings/gravity/`；电机参数：`recordings/motor_params/`。真机启动会调 `scripts/configure_can.sh`（默认左 `can1`、右 `can0`，1M / 5M CAN FD），需 sudo。

仿真会打开 MuJoCo 窗口。默认场景是仓库内 `models/OpenArm_V1/scene_table.xml`，按 `run.sh` 所在目录解析，因此换到别的工作目录启动也找得到。`OPENARM_MUJOCO_XML` 可改场景。若本机仍有旧路径 `/home/ducks/motion-trajectory-studio/model_library/OpenArm_V1/scene_table.xml` 且没有设置该变量，则继续用旧文件。

默认使用宇树参考模式，控制台可切换为 PIKA 参考；也可设置 `VR_TELEOP_MODE=pika_style`。详见 [宇树参考模式](docs/Quest宇树参考模式.md)。

Quest 浏览器打开 `https://<这台电脑的IP>:8443`（证书在 `quest/`）。网页 `8443` 和 WebSocket `8442` 仍监听局域网，头显从另一台设备连入。桥再把左手 UDP `47912`、右手 `47911` 发给控制程序；控制端这两个端口 默认只绑定 `127.0.0.1`。桥和控制程序要在同一台电脑，否则控制端收不到姿态，日志里会一直停在等待 UDP。跨机时设置 `OPENARM_QUEST_UDP_HOST`：控制端填 `0.0.0.0`（监听所有网卡）或本机网卡地址，桥填控制电脑的 IPv4。发给 `0.0.0.0` 的包仍留在桥所在的电脑上。

VR 网页用于手柄追踪，不显示实时摄像头画面，也不会自动启动相机采集。沉浸模式不创建网页悬浮层，仅允许透视 AR；按 Quest 系统菜单退出沉浸模式后，可在网页停止追踪。头部及腕部相机仍可通过控制台独立预览，并沿用原有录制流程。

真机启动前先把 CAN 配好，并确认急停可用：

```bash
sudo ./scripts/configure_can.sh can1 can0 1000000 5000000
./run.sh --real
```

VR 遥操默认先到达起手姿态，再接受 Grip 和录制。仿真和真机均先从当前姿态平滑接入 `assets/steady_action_on_table.csv` 的第一帧，再按文件时间戳播放完整轨迹，最后保持末帧。仿真夹爪角度转换为位移。`OPENARM_QUEST_STARTUP_FILE` 可指定启动文件，`OPENARM_QUEST_STARTUP_TRANSITION_SEC` 设置接入首帧时长（默认 5 秒）；设 `OPENARM_QUEST_STARTUP_MOVE=0` 可跳过。

实机 Quest 遥操默认在 MIT 位置/速度控制上叠加重力前馈：根据实测关节角计算 `G(q)`，复用 `config/gravity_comp.yaml` 的左右臂倍率、偏置、质量/质心、相机及夹爪安装质量，使用 `tracking_g_scale`（当前起始值 1.00，独立于 MoveIt 的零重力标定倍率；静止下垂或被顶起时在 0.95–1.08 内按 0.02 调整）。起手动作、运动、松握保持和 Quest 断流保持均持续补偿；首次输出用 1 秒平滑渐入，七轴前馈限幅分别为 20/20/7/7/2/2/2 Nm，夹爪无重力前馈，也不叠加摩擦补偿。标定文件在启动时加载，修改后重启遥操。夹爪安装开关在此影响质量模型，遥操的电机注册仍保持原有双夹爪要求。

Quest 短暂超时后，持续握住的自动恢复沿用断流时保持的关节指令，避免以已下沉的实测角度重新设定更低目标；主动松握再握仍按所选模式重新锚定。CAN 反馈失效或安全停机仍按原保护逻辑失能，重力前馈不覆盖停机。`GetLastCommandEfforts` 与轨迹记录中的 MIT `effort` 反映实际发送的前馈力矩；这不等于电机内部 PD 与前馈相加后的总力矩。

## 常用环境变量

| 变量 | 默认 | 作用 |
| --- | --- | --- |
| `VR_TELEOP_MODE` | `unitree_style` | `unitree_style` / `pika_style` |
| `VR_ORIENTATION` | `absolute` | `absolute` 或 `incremental` |
| `OPENARM_QUEST_UDP_HOST` | `127.0.0.1` | Quest 桥发往控制程序的 UDP 地址，控制端绑定同一值，手柄采集快捷键 `47934` 也用它。`0.0.0.0` 让控制端监听所有网卡；跨机时桥要填控制电脑的 IPv4，不要填 `0.0.0.0`。 |
| `OPENARM_LEFT_FOLLOWER_CAN` | `can1` | 左臂 CAN |
| `OPENARM_RIGHT_FOLLOWER_CAN` | `can0` | 右臂 CAN |
| `OPENARM_QUEST_GRAVITY_FF` | `1` | 实机 Quest 重力前馈开关；`0` 回退为原 PD 控制，仿真不启用 |
| `OPENARM_GRAVITY_YAML` | `config/gravity_comp.yaml` | 共用重力标定文件 |
| `OPENARM_QUEST_GRAVITY_GAIN` | YAML `tracking_g_scale` | 遥操前馈倍率，允许 0–2 |
| `OPENARM_QUEST_GRAVITY_RAMP_SEC` | `1` | 首次前馈渐入时间，允许 0.1–10 秒 |
| `OPENARM_HM_CLUTCH_START_DEG` | `2` | 握持后转过这么多度才开始跟姿态 |
| `OPENARM_SCURVE_VMAX` | `0.6` | 启动/CSV/MoveIt S 曲线最大关节速度 (rad/s) |
| `OPENARM_SCURVE_AMAX` | `1.2` | S 曲线最大加速度 (rad/s²) |
| `OPENARM_SCURVE_JMAX` | `6` | S 曲线最大加加速度 (rad/s³) |
| `OPENARM_MOVEIT_SCURVE` | `1` | MoveIt 实机：段内 S 形加减速（保持原航点时刻）；`0` 回退线性 |
| `OPENARM_MUJOCO_XML` | 仓库内桌面场景 | 仿真场景。未设置时：旧的 `/home/ducks/.../scene_table.xml` 若存在则用它，否则用 `models/OpenArm_V1/scene_table.xml`。`run.sh` 与 `script/replay_teaching.py` 共用这条规则 |
| `OPENARM_TELEOP_URDF` | `assets/v1.urdf` | 遥操 URDF，相对 `run.sh` 所在目录 |

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

自动标定离线回归检查：`.venv/bin/python -m unittest discover -s tests -v`（不启用 CAN）。安全相关阈值和哪些用例在 main 上仍会跳过，见 [安全测试](docs/SAFETY_TESTS.md)。离合测试需要先编译 `human_mimic_clutch_test`。GitHub Actions（`.github/workflows/tests.yml`）在 push 和 pull request 上跑这套测试以及 Node 检查，不连接真机。
