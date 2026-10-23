# OpenArm Teleop 安装与使用说明书

面向第一次安装和操作本软件的用户。项目文件夹名称为 `openarm-teleop`；下文命令除特别说明外，均在包含 `app.sh`、`install.sh` 和 `CMakeLists.txt` 的项目根目录执行。

本软件使用 Quest 手柄操作 OpenArm 双臂，提供 Qt 控制台、MuJoCo 仿真、CAN 真机控制、动作录制和回放、数据集导出、重力标定及可选 MoveIt 规划。基础遥操作不需要 ROS；MoveIt 单独依赖 ROS 2 Humble。

## 一、准备工作

建议从 Ubuntu 22.04 桌面环境部署，尤其是需要 ROS 2 Humble 时。工程依赖 Linux SocketCAN、C++17、CMake 3.22 及以上和 Python 虚拟环境。其他 Linux 版本需自行核对系统包与 Python wheel 的兼容性；本文不提供 Windows/macOS 原生真机安装流程。

| 使用方式 | 准备设备 |
| --- | --- |
| 安装、仿真窗口和回放检查 | Linux 电脑、桌面图形环境、可用 OpenGL |
| VR 仿真遥操 | 上述电脑、Quest 头显及左右手柄、同一局域网 |
| 真机双臂遥操 | 上述设备、OpenArm 双臂及夹爪、CAN FD 适配器、供电、硬件急停 |
| 图像采集 | USB 相机，项目包含 Gemini 335 支持 |
| MoveIt | ROS 2 Humble、MoveIt 2、colcon |

没有机械臂也可以先验证仿真、Quest 和录制流程；没有 Quest 时可检查仿真窗口或回放已有片段。

**真机启动会执行起手轨迹。** 首次启动前必须核对安装方向、零位、CAN 映射、负载和运动范围。软件限位不能代替碰撞检测。停止并释放会失能，机械臂可能下落，操作前应准备可靠支撑。松开 Grip、暂停、软件停止和硬件急停的作用不同。

## 二、获取与安装

### 2.1 获取完整项目

从维护者取得完整项目，或用团队提供的真实地址替换下面的占位地址：

```bash
git clone <团队提供的仓库SSH地址> openarm-teleop
cd openarm-teleop
```

使用压缩包时，解压后进入项目根目录即可。应包含 `third_party/openarm_can/`、`models/OpenArm_V1/`、`assets/v1.urdf` 和 `assets/steady_action_on_table.csv`。缺文件时先取得完整版本。

### 2.2 系统依赖

在采用 apt 的 Ubuntu 系统执行，需要 sudo 权限：

```bash
sudo apt update
sudo apt install -y build-essential cmake python3 python3-venv python3-dev \
  libeigen3-dev liborocos-kdl-dev libkdl-parser-dev liburdfdom-dev \
  libyaml-cpp-dev libglfw3-dev libgl1-mesa-dev can-utils iproute2 openssl
```

需要视频采集或导出时再装：

```bash
sudo apt install -y ffmpeg
```

找不到系统包时检查发行版、软件源和启用的源组件，不要忽略 CMake 依赖错误继续操作。

### 2.3 Python 环境

```bash
./install.sh
.venv/bin/python -c "import mujoco, PySide6, websockets, aiortc, numpy; print('Python dependencies OK')"
```

安装脚本创建本目录 `.venv`，安装 `requirements.txt` 中的 MuJoCo、PySide6、WebSocket/WebRTC 和数据处理包，并尝试安装可选 `openarm-dataset`。可选包失败时会提示；基础依赖失败则安装中止。

不必手动激活环境，启动脚本优先使用项目 `.venv/bin/python`。手动装包用 `.venv/bin/python -m pip`，避免装入其他解释器。

### 2.4 编译

```bash
cmake -S . -B build -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build -j
```

内存不足时改为 `cmake --build build -j2`。`run.sh` 会自动配置并编译遥操目标，首次启动可能较慢。

### 2.5 可选 IPOPT 逆解

控制台使用宇树参考 `unitree_style`。基础 Python 依赖不包含带 CasADi 绑定的 Pinocchio；求解器不可用时程序可回退原有 LM 逆解。需要 IPOPT 时，在已安装 conda 的前提下创建独立环境：

```bash
conda create -n openarm-unitree python=3.10 pinocchio=3.1.0 numpy=1.26.4 casadi -c conda-forge
conda run -n openarm-unitree python -c "import casadi; from pinocchio import casadi; print('IK dependencies OK')"
```

不要把该环境的 NumPy 1.26 配置混入项目要求 NumPy 2 的 `.venv`。启动后以日志 `IPOPT 已接入` 判断实际启用情况；解释器查找与模式说明见 [Quest 宇树参考模式](docs/Quest宇树参考模式.md)。可用 `OPENARM_UNITREE_IPOPT=0 ./run.sh --sim` 关闭此路径。

### 2.6 文件夹改名或移动后的处理

旧 `.venv` 内的脚本与 `build/CMakeCache.txt` 可能保存原绝对路径。出现旧路径或 CMake 目录不一致错误时，先停止控制程序，将 `.venv`、`build` 移出项目备份，再执行 2.3、2.4 重建。MoveIt 的 `ros2/build`、`ros2/install`、`ros2/log` 也可能需要移出重建。

保留 `recordings/` 和 `config/`，不要为修复构建删除录制与标定。桌面快捷方式、编辑器工作区、外部脚本中的旧路径也要更新。

## 三、第一次启动仿真

推荐图形控制台：

```bash
./app.sh
```

1. 在遥操作页选择“仿真遥操”。
2. 点击“启动”，查看日志并等待编译、仿真窗口和起手动作完成。
3. 确认 MuJoCo 显示双臂，没有启动错误。
4. 连接 Quest 后先小幅检查单侧、另一侧及夹爪。
5. 完成后点击“停止并释放”，确认回到停止状态。

命令行入口：

```bash
./run.sh --sim
./run.sh --print-paths
```

第一条启动仿真、Quest 桥和录制网页面板，终端 Ctrl+C 退出。第二条仅打印模型路径，不编译、不启动。`./run.sh --no-bridge` 可不启动桥，适合已有独立桥的情况。

默认场景是 `models/OpenArm_V1/scene_table.xml`；若机器上仍存在历史路径 `/home/ducks/motion-trajectory-studio/model_library/OpenArm_V1/scene_table.xml`，则优先使用旧场景。可明确指定仓库版本：

```bash
OPENARM_MUJOCO_XML="$PWD/models/OpenArm_V1/scene_table.xml" ./run.sh --sim
```

仿真流程成功不能证明真机力矩稳定、无碰撞或循环返回安全。

## 四、连接 Quest 与操作

### 4.1 连接步骤

1. 电脑与 Quest 接入能够互访的同一局域网，避免启用设备隔离的访客 Wi-Fi。
2. 电脑执行 `hostname -I`，选取 Quest 可访问的局域网 IPv4，例如 `192.168.1.50`。
3. 启动遥操，使 Quest 桥运行。
4. Quest 浏览器打开 `https://192.168.1.50:8443`，替换为实际 IP。
5. 桥使用 `quest/` 下证书，缺失时尝试用 OpenSSL 生成。首次出现自签名证书提示时，核对是本机地址后按浏览器提示继续；WebSocket 证书被拒绝时，可访问同主机 `https://192.168.1.50:8442` 确认证书，再回到 8443。8442 不提供普通网页内容。
6. 按页面实际按钮开始追踪/进入沉浸模式，授予权限，确认左右手数据更新。

网页用于手柄追踪，不显示实时相机画面。沉浸模式使用透视 AR，没有网页悬浮层；需操作网页停止追踪时先从系统菜单退出沉浸模式。WebXR 需要安全上下文，不要把 HTTPS 改成普通 HTTP。

### 4.2 手柄操作表

| 输入 | 功能 |
| --- | --- |
| 握住 Grip（侧握把） | 对应一侧跟随手柄 |
| 松开 Grip | 退出跟随并保持参考位置，不等于失能 |
| 前扳机 | 扣下闭合夹爪，松开张开 |
| 右 Grip + A+B | 重新标定右侧朝向 |
| 左 Grip + X+Y | 重新标定左侧朝向 |

默认绝对朝向标定：第一次握住 Grip 锁定手柄到夹爪的朝向关系，后续松开再握只重锚位置。首次把手柄放在舒适位置，小幅确认方向，再扩大动作范围；等待起手完成后再握 Grip 和录制。

姿态优先走 WebRTC，未连通时回退 WebSocket；不使用公网 STUN。电脑上的桥再通过本机 UDP 将左右手姿态交给控制程序。

## 五、真实机械臂操作

### 5.1 启动前

先完成仿真流程，由熟悉设备的人确认供电、机械固定、关节零位、负载、夹爪和硬件急停。清空起手轨迹及后续动作的工作区。默认真机遥操要求双夹爪电机；拆下夹爪后仅修改重力模型不代表电机注册也兼容。

默认 **左臂 can1、右臂 can0**，编号不保证对应插口顺序。按接线核实，不要同时使能双臂来试错。

### 5.2 CAN 配置

```bash
ip -br link
sudo ./scripts/configure_can.sh can1 can0 1000000 5000000
ip -details link show can1
ip -details link show can0
```

脚本参数依次为左接口、右接口、仲裁波特率、数据波特率；默认 CAN FD 1 Mbit/s / 5 Mbit/s，硬件需支持并匹配。脚本先关闭接口再重配，不要在控制程序运行时执行。

控制台可设置左右 CAN。直接 `run.sh --real` 不自动配置接口，应先执行配置脚本；不同接口名可用以下示例（替换为真实名称）：

```bash
OPENARM_LEFT_FOLLOWER_CAN=can_left OPENARM_RIGHT_FOLLOWER_CAN=can_right ./run.sh --real
```

### 5.3 正常启动与结束

1. `./app.sh`，选择“真机遥操”，核对左右 CAN。
2. 确认硬件急停有效，再完成界面急停确认。
3. 核对 `config/gravity_comp.yaml` 与本机夹爪、相机及负载一致，不直接套用其他机械臂的标定。
4. 准备支撑，清空工作区，点击“启动”，等待使能和起手轨迹完成。
5. 连接 Quest，小幅验证单侧方向及夹爪后再操作双臂。
6. 结束前停止录制、完成标签，把手臂放到可支撑位置，再“停止并释放”。

紧急情况下使用已验证的硬件急停。Quest 断流保持不能代替急停；CAN 故障或安全停机可能失能，需防止下落。不要同时运行遥操、回放、零重力、标定或 MoveIt 控制同一组电机。

## 六、录制与数据管理

### 6.1 一段任务的采集

1. 控制台填写批次名，例如 `pick_cube_20261023`，建议用字母、数字和下划线。
2. 启动仿真/真机遥操，等待起手完成；需要图像时先检查相机预览。
3. 点击“开始录制”，执行完整任务。
4. 点击“停止”，再标记成功或失败；待标注时先处理当前条。
5. 需要重录时使用“丢弃重录”或数据管理中的“重采此条”。
6. 在数据页检查片段、标签和导出日志，不能只凭手柄震动认定保存成功。

### 6.2 手柄快捷键

仅通过 `./app.sh` 启动并填写批次后支持；单独 `./run.sh` 不提供这组控制台快捷键。

| 操作 | 功能 |
| --- | --- |
| 左摇杆短按并在 0.4 秒内松开 | 开始/停止录制 |
| 停录后右摇杆短按 | 成功 |
| 停录后右摇杆按住 1.5 秒 | 失败 |
| 停录后左摇杆按住 1.5 秒 | 移入回收站，下次复用序号 |
| 丢弃后 10 秒内再长按，或点“撤销丢弃” | 恢复刚丢弃片段 |

长按超过 0.4 秒但未到 1.5 秒就松开会取消，录制中不能丢弃。断连或追踪丢失后先松开再按。回收站为 `recordings/trash/`；不要把数据页所有删除操作都视为可撤销丢弃。

数据页可浏览批次/片段、改标签、重采、导出及回放。录制中或待标注时限制编辑当前批次。平滑会生成 `processed/smooth_…/` 副本，保留源数据；先仿真检查，平滑后的指令不能当作新的真实观测。

命令行模式还可访问 `http://127.0.0.1:8765` 使用本机录制面板。

## 七、动作回放

1. 停录并标注，停止并释放遥操控制器。
2. 数据页选片段，点击“回放此条”，进入回放页。
3. 先选仿真，并按来源选夹爪单位：真机为角度，仿真为位移。
4. 点击“到达起手位置”，等待起手轨迹和录制首帧衔接完成。
5. 点击“开始回放”，可选 0.25、0.5、1 倍，支持暂停/继续/停止。
6. 播完保持末帧；再次播放可先返回录制首帧再手动开始。切换动作或设置前先停止并释放。

回放不要求 Quest 在线。真机需重新确认 CAN、急停、负载和工作区；限速保护可能使实际播放时间变长。播放慢速选项不代表起手轨迹也按同倍率缩放。

次数为 0 表示持续循环。真机每遍之间保持使能并返回首帧，**返回路径不是录制路径，也没有碰撞检测**；仿真直接回首帧，不能验证真机返回过渡。还需考虑物体复位和首尾衔接。

离线检查与仿真示例，路径换成实际片段：

```bash
.venv/bin/python script/replay_teaching.py recordings/sessions/demo/01 --real --check
.venv/bin/python script/replay_teaching.py recordings/sessions/demo/01 --sim
```

第一条只检查文件，不使能；第二条仿真，命令行默认自动播放。详见 [动作录制与轨迹回放](docs/动作录制与轨迹回放.md)。

## 八、相机与数据集导出

控制台刷新 USB 列表，将头、左腕、右腕映射到对应相机并逐个预览。`config/cameras.yaml` 中原有 USB 路径与序列号属于原设备，应按本机更新。Quest 网页不会自动启动相机采集。

项目提供 UVC 彩色节点及 Orbbec SDK 路径。`pyorbbecsdk` 不在基础依赖中，需要 SDK 时安装与设备和 Python 匹配的版本，并在实际采集解释器中验证导入。部分彩色采集可通过 UVC 工作；具体以预览和日志为准。检查 USB 带宽、供电、线缆和设备占用。

先录短片段，确认输出真实包含图像与时间戳；能录关节轨迹不代表相机正常。

打过标签的片段触发导出，批次目录中通常包含：

```text
recordings/sessions/<批次>/
├── <序号>/             原始片段与元数据
├── openarm_dataset/    OpenArm Dataset 输出
└── lerobot/            依赖齐全且转换成功时的 LeRobot 输出
```

手动重导：

```bash
.venv/bin/python script/openarm_dataset_export.py --batch recordings/sessions/demo
```

需要官方工具时可显式安装并转换：

```bash
.venv/bin/python -m pip install openarm-dataset
.venv/bin/openarm-dataset-convert recordings/sessions/demo/openarm_dataset /tmp/openarm-demo-lerobot \
  --format lerobot_v3.0 --fps 30
```

官方工具不可用时项目会尝试本地 CSV 转 LeRobot 回退路径。检查日志和实际文件，目录存在不代表导出完整。`OPENARM_LEROBOT=0` 跳过相关自动导出；`OPENARM_LEROBOT_TASK`、`OPENARM_LEROBOT_FPS`、`OPENARM_LEROBOT_REPO` 分别设置任务、帧率、仓库标识。

## 九、零重力与标定

零重力是模型重力前馈辅助，仍可能漂移或有摩擦残差，不能代替机械支撑。此功能连接真机，初次应由熟悉设备的人操作。

| 功能 | 是否运动 | 用途 |
| --- | --- | --- |
| 使用 URDF 初值 | 否 | 恢复模型基准，覆盖重力配置的对应参数 |
| 自动微调 | 是 | 多姿态采数，精修倍率/偏置 |
| 惯量标定 | 否 | 用已有日志辨识质量/质心，预览确认后写入 |
| 摩擦辨识 | 是 | 逐关节正反扫描，拟合摩擦 |

自动微调访问较大姿态范围，不执行桌面避障，必须清空标定工作区。部分操作直接覆盖配置且不另存备份，开始前自行复制 `config/gravity_comp.yaml`，记录硬件安装状态。

**`--dry-run` 不一定不动臂。** `scripts/calibrate_gravity.py --dry-run` 仍会驱动采数，仅不写配置。使用已有日志离线预览才用：

```bash
.venv/bin/python scripts/calibrate_gravity.py --skip-motion --dry-run
```

零重力入口是控制台模式或 `./scripts/run_zero_g.sh`。操作细节及验收见 [重力补偿原理与实践教程](docs/重力补偿原理与实践教程.md) 和 [自动重力标定流程说明](docs/自动重力标定流程说明.md)。拟合改善不等于真机已验收。

## 十、可选 MoveIt 规划

脚本要求 `/opt/ros/humble/setup.bash`。先按团队方式安装 ROS 2 Humble，ROS apt 软件源配置好后可装：

```bash
sudo apt install -y ros-humble-moveit python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2/src --ignore-src -r -y
./scripts/run_moveit.sh --sim
```

rosdep 首次使用需管理员完成 `rosdep init`，再由普通用户 `rosdep update`。脚本会 colcon 构建项目 ROS 包并启动 RViz。选择规划组、设置目标，先 Plan 检查，再在仿真 Execute。MoveIt 仿真是 ROS/RViz 模型，与 MuJoCo 遥操是不同入口。

控制台 MoveIt 不勾急停确认时是仿真；完成硬件准备并确认后才进入真机。命令行 `./scripts/run_moveit.sh --real` 要求 CAN 已配置。首次真机规划应由熟悉 RViz 和硬件的人执行。

软硬档位支持正常、中等、偏低和接触柔顺，运行中可手拖或软急停。软急停保持使能，停止并释放会失能。低刚度可能有静差，规划容差不代表最终精度，具体限制见主 README。MoveIt 和零重力不录制 Quest 遥操数据。

## 十一、配置与网络速查

| 路径 | 内容 |
| --- | --- |
| `config/gravity_comp.yaml` | 重力、负载与标定 |
| `config/follower.yaml` | 跟随器参数，读取行为依具体模式 |
| `config/cameras.yaml` | 相机映射 |
| `assets/v1.urdf` | 默认机器人模型 |
| `assets/steady_action_on_table.csv` | 默认起手动作 |
| `recordings/sessions/` | 片段和数据集 |
| `recordings/trash/` | 丢弃重录回收站 |
| `recordings/hm_logs/` | 控制器日志 |
| `recordings/gravity/` | 标定及摩擦日志 |
| `recordings/motor_params/` | 电机参数读取结果 |

日志支持展开、过滤、查找和完整归档，见 [实时日志使用说明](docs/实时日志使用说明.md)。电机页读取 EEPROM 不使能、不运动，但仍需正确 CAN，不应与其他控制程序争用设备。

环境变量在启动前设置。初学者统一在停止后修改配置，再重启控制，不在运行中随意调整增益。

| 变量 | 默认/用途 |
| --- | --- |
| `OPENARM_LEFT_FOLLOWER_CAN` / `OPENARM_RIGHT_FOLLOWER_CAN` | 左 can1 / 右 can0 |
| `OPENARM_MUJOCO_XML` | 仿真场景，建议绝对路径 |
| `OPENARM_TELEOP_URDF` | 默认 assets/v1.urdf |
| `OPENARM_GRAVITY_YAML` | 默认 config/gravity_comp.yaml |
| `VR_TELEOP_MODE` | 默认 unitree_style；控制台固定此模式 |
| `VR_ORIENTATION` | 默认 absolute |
| `OPENARM_QUEST_STARTUP_FILE` | 起手 CSV |
| `OPENARM_QUEST_STARTUP_TRANSITION_SEC` | 首帧接入默认 5 秒 |
| `OPENARM_QUEST_STARTUP_MOVE` | 默认 1，0 跳过起手，不建议初学者改变 |
| `OPENARM_QUEST_UDP_HOST` | 默认 127.0.0.1，姿态 UDP 地址 |
| `OPENARM_QUEST_GRAVITY_FF` | 真机 Quest 前馈默认开启，0 关闭 |

| 端口 | 用途 |
| --- | --- |
| TCP 8443 | Quest HTTPS，桥监听局域网 |
| TCP 8442 | 加密 WebSocket/信令，桥监听局域网 |
| UDP 47912 / 47911 | 左/右姿态，控制端默认只绑定本机 |
| UDP 47930 | 录制控制 |
| UDP 47931 | 按钮示教 |
| UDP 47934 | 控制台手柄采集快捷键 |
| TCP 8765 | 命令行录制网页，本机访问 |

WebRTC 还需要局域网直接数据通道连通，只放行两个 TCP 端口不一定能启用；不通可回退 WebSocket。按组织策略放行必要访问，不把控制端口映射到公网。

推荐桥与控制程序同机。跨机时控制端 `OPENARM_QUEST_UDP_HOST` 填本机网卡地址或 `0.0.0.0`，桥端填控制电脑实际 IPv4。桥发往 `0.0.0.0` 不会将数据送到另一台电脑。

## 十二、故障排查

| 现象 | 处理 |
| --- | --- |
| 脚本 Permission denied | 检查解压是否丢执行位，可先用 bash install.sh / bash app.sh，再由维护者修复权限 |
| Python 缺模块 | 确认项目 .venv 解释器，重新安装并查看 pip 首个错误 |
| CMake 找不到 MuJoCo | 用 .venv/bin/python 验证 import，重新指定 Python3_EXECUTABLE |
| CMake 指向旧目录 | 停止后将旧 build 移出备份再配置，移动过的 .venv 也需重建 |
| 编译 Killed | 降为 -j2，检查内存和磁盘 |
| Qt 无法连接 display | 在桌面图形会话启动，按 Qt 插件实际错误补依赖 |
| 场景/网格缺失 | run.sh --print-paths，核对模型完整性和旧环境变量 |
| Quest 打不开网页 | 核对局域网 IP、HTTPS、桥、8443、防火墙和 Wi-Fi 隔离 |
| 网页有但无追踪 | 检查沉浸模式、授权、手柄可见性、8442 连接和证书 |
| 一直等待 UDP | 检查桥、左右端口和地址；跨机不能用默认环回地址 |
| 手柄动但不跟随 | 等起手完成、握对应 Grip，检查追踪及安全状态，不绕过故障保护 |
| 快捷键无响应 | 使用 app.sh、先填批次，检查 47934；断连后松开再按 |
| 找不到 CAN | 检查适配器、驱动、USB、ip -br link，确认实际接口名 |
| CAN 无反馈/BUS-OFF | 停止后检查供电、线序、终端电阻、FD 速率和映射，不反复使能试错 |
| 下垂、漂移或抖动 | 支撑并停止，核对负载、模型和本机标定，不先靠提高增益掩盖问题 |
| 相机无画面 | 重新选 USB/序列号，查 SDK/UVC、权限、占用、供电和带宽 |
| 录制有但未导出 | 检查标签、OPENARM_LEROBOT、转换日志、依赖、磁盘 |
| 回放数据非法 | 检查递增时间、有限数值、限位、夹爪单位，保留原文件定位 |
| MoveIt 缺 ROS | 检查 /opt/ros/humble/setup.bash 和 ROS/MoveIt 依赖 |
| RViz GLXContext 错误 | 查驱动/桌面环境，可用下面软件渲染定位显示问题 |

```bash
OPENARM_MOVEIT_RVIZ_RENDERER=software ./scripts/run_moveit.sh --sim
```

软件渲染性能较低。`You can start planning now!` 表示规划后端就绪，不代表 RViz 显示成功。处理驱动或重启前先停止并支撑机械臂。

报障时提供使用模式、复现步骤、首个错误前后日志、系统/Python 版本、CAN 状态或场景路径，不发送令牌、私钥及无关敏感录制。

## 十三、日常操作与维护

每次开始：检查机械固定、线缆、工作区和急停，核对 CAN、负载与配置，选择正确模式，等起手完成后小范围验证，再正式采集。

每次结束：停录并标注，确认数据保存，停止相机等任务，将手臂可靠支撑，停止并释放并确认控制器退出，再按硬件规定断电。

更新前停止控制，备份重要批次与配置，按团队发布流程更新，重新安装变化的依赖并编译，先验证仿真。备份时记录负载和硬件状态，不把他机参数直接覆盖本机标定。

维护者可执行无 CAN 的 Python 回归检查：

```bash
.venv/bin/python -m unittest discover -s tests -v
```

部分测试依赖预先编译 C++ 目标。测试通过不代替实机验收，范围见 [安全测试](docs/SAFETY_TESTS.md)。软件采用 Apache-2.0，见 `LICENSE-APACHE`，第三方 CAN 代码见 `third_party/openarm_can/LICENSE.txt`。

更多控制参数、模式限制及实现说明见 [主 README](README.md)。
