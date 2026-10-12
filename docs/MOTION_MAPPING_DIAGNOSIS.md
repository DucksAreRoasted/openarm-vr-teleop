# VR 遥操作：手移动得更远、机械臂走不到

日期：2026-10-12。基线是 `main` 的 `73dc004`（`feat(ui): 优化主界面与回放对话框布局`）。本文只读代码，不改控制、配置或界面行为。

对照过的材料：

- [PR #1](https://github.com/DucksAreRoasted/openarm-vr-teleop/pull/1) `docs/HEALTH_CHECK.md`（分支 `cursor/repo-health-check-a4e9`），下文用 A1–A4、B1 指那份计划里的任务。
- [PR #2](https://github.com/DucksAreRoasted/openarm-vr-teleop/pull/2) `docs/TELEOP_RECORDING_PLAN.md`（分支 `cursor/teleop-recording-plan-9925`）。那份文档解释跟手延迟；本文解释**停下来以后手臂仍然比手短**，以及运动过程中为什么会少走一截。
- 进行中的安全改动：[PR #3](https://github.com/DucksAreRoasted/openarm-vr-teleop/pull/3)（Quest UDP 默认回环）、[PR #4](https://github.com/DucksAreRoasted/openarm-vr-teleop/pull/4)（CAN 反馈新鲜度、`SafetyFilter`、周期超时不补拍）。

主人确认的现象是欠幅：同样一段动作，机器人末端走过的距离小于操作者的手。默认模式是 `human_mimic`（`control/openarm_quest_cartesian_control.cpp` 里 `teleop_mode` 的初值，界面不勾「Quest 肘部与腕部追踪」时走这条）。下面的排序按这条路径、并且按「会让手臂走不到目标」来排。笛卡尔和骨骼模式单独写。

本次没有编译、没有仿真、没有接 Quest、CAN 或真机。标了「已核对」的句子能在当前源码里对上路径和常数。标了「待测量」的句子需要主人按第 6 节在真机或仿真里读一次数。

## 0. 最可能让手臂比手短的三件事

1. **两连杆到达球把腕部目标夹短，逆解跟踪的是夹短之后的点。** 手的位移按 1:1 加到末端，但肩到腕的距离不允许超过 `L1+L2−0.002` m。再往外伸，手还在走，腕部目标停在球面上，末端就停短。`L1`、`L2` 在第一次握 Grip 时按当时的肩–肘、肘–腕弦长量出来并锁死；弯曲着握，这颗球比手臂真正能伸直的范围更小。已核对，见第 4.1 节。少多少取决于握持时手臂有多弯，待测量。
2. **关节限位把解停在机械止点上。** 真机肩 `joint2`（下标 1）单侧只有约 10°：右臂下限 −0.175 rad，左臂上限 +0.175 rad。仿真把这一侧放到约 ±60°，并把肩偏航再放宽。肘 `joint4` 是 0～2.443 rad，腕 `joint6` 是 ±0.785 rad。顶到限位后，位置任务留下残差，末端走不到 1:1 目标。限位数字已核对；主人这次的动作有没有顶到，看日志里的 `lim=` / `at_lim=`，待测量。真机比仿真更短，优先查这一条。
3. **加权逆解可以在姿态或连续性争抢时留下位置残差；运动过程中还有速度斜坡。** 位置权重 12、姿态权重 8，解是折中，不是硬约束。肘关节目标最快 4.5 rad/s，关节参考发生器小误差时间常数约 71 ms。真机 MIT 的 `dq` 被写成 0，近端 `Kd=3` 会在运动中把臂往回拉。手停下并保持约 1 秒后，斜坡和发生器应基本追上；若这时末端仍然短一截，原因在前两条或 `e_wrist` 残差。常数已核对；残差有多大，待测量。

平移增益 `pos_scale` 是 **1.0**，仓库里没有环境变量能改它。轴映射是正交的，不改变三维长度。Quest 页面用 `local-floor`，按米转发，页面里没有第二个比例。所以「到处都少一截、而且 5 cm 和 30 cm 的比例一样」不会来自默认的 `human_mimic` 增益。那种恒定比例要先确认是不是开了骨骼模式（第 5.2 节）。

## 1. 映射管线

默认 `human_mimic`，Grip 按住之后：

```text
Quest 浏览器 quest/client.html
  immersive-ar，参考空间 local-floor（失败则 local）
  gripSpace 的 position / quaternion，单位米，60 Hz 上限
        | WSS :8442   JSON，桥不缩放
quest/quest_bridge.py
        | UDP 127.0.0.1:47912 左 / 47911 右
        | CSV: 时间戳,grip,trigger,px,py,pz,qx,qy,qz,qw,...
QuestReceiveThread 约 90 Hz
  用本机 steady_clock 盖 receive_timestamp_ns，只留最新一包
        |
HumanMimicController::step()     参考线程名义 1000 Hz
  1. Grip 上升沿 activate_clutch
       hand_ref = 当前手柄位置
       robot_ref = 当前手 TCP 的正运动学（Dynamics 末端是 hand_tcp）
       第一次成功时锁 L1、L2，并标定 wrist_to_tcp
  2. 未超过 3 mm 且转角未超过 2°：只更新 hand_ref，臂保持
  3. delta_vr = 手柄位置 − hand_ref          （默认世界系）
     delta_robot = M * delta_vr * pos_scale   （pos_scale = 1）
     再按前后 / 侧向 / 上下夹紧
     p_tcp_des = robot_ref + delta_robot
  4. 腕目标 = p_tcp_des − R * wrist_to_tcp
     若肩到腕超出 [短限, L1+L2−0.002]，沿径向夹回
     tip_for_ik 由夹回后的腕加回工具向量
  5. 整臂 DIFF-QP，最多 12 次；再对 q_target 做关节斜坡
  6. JointReferenceGenerator：v = clamp(k*(q_target−q_ref), ±qdot_max)
        |
fill_reference：七个臂关节 velocity = 0，tau 字段为 0
        |
Follower 1000 Hz，MIT {kp, kd, q, dq=0, tau=0}
  仿真 kp>0 时直接写 MuJoCo qpos
  真机靠电机里的 Kp 弹簧跟上，重力不进前馈
```

笛卡尔模式在第 3 步改成「握持瞬间手柄本体坐标系里的位移」，后面是 DLS 速度伺服，没有两连杆球。骨骼模式不用手柄的平移，肩肘方向来自身体追踪。

轴映射在 `HumanMimicController::vr_to_robot_axes()`（`src/controller/human_mimic_controller.cpp` 约 293–304 行），笛卡尔同一矩阵（约 509–518 行）：

```text
机器人 +X = −Quest Z     （Quest 的水平朝前，是 −Z）
机器人 +Y = −Quest X     （Quest +X 向右）
机器人 +Z = +Quest Y     （Quest +Y 朝上）
```

`det(M)=+1`，是旋转。三维距离在乘 `pos_scale` 之前保持不变。`mirror_translation_y` 默认 false。

## 2. 已经核对过、不会单独造成「按比例变短」的部分

| 项目 | 位置 | 结论 |
| --- | --- | --- |
| 平移增益 | `human_mimic_types.hpp` `pos_scale = 1.0`；`make_cartesian_config()` 显式写成 1.0 | 默认两条笛卡尔类路径都是 1:1。全仓库没有 `OPENARM_HM_POS_SCALE`。要改比例必须改源码。 |
| 姿态增益 | human_mimic `rot_scale = 1.0`；笛卡尔在 `make_cartesian_config()` 写成 **0.6** | 只缩小转角。笛卡尔模式下手腕转得比手柄少，平移长度不受这个 0.6 影响。 |
| 工具长度是否把锚点锚错 | `select_leaf_link()` 优先 `openarm_*_hand_tcp`；URDF 该关节为 link7 沿 +Z **0.186 m**；`wrist_to_tcp` 默认 `(0,0,0.186)`，首次握持若测量值落在 0.02–0.40 m 会写回 | 握持锚点和逆解任务是同一个 TCP。增量 0 时目标与当前 TCP 重合。这不是欠幅来源。 |
| Quest 页面比例 | `quest/client.html` 约 285、345–369、410 行；`quest_bridge.py` 原样 `%.6f` | `local-floor` 的 position 按米发送。页面没有缩放系数。60 Hz 上限只丢帧，不改变已经发出的坐标。 |
| 位置低通 | `fine_motion_hold_enabled = false`；`joint_output_filter_enabled = false` | 平移没有低通。朝向 SLERP 的 `tau_orientation = 0.006` s 不改变位置长度。 |
| 软到达对 TCP 的直接作用 | `solve_base_skeleton()` 约 1391–1456 行与 2029–2043 行 | 进度软顶（弯曲时多留 45 mm 余量）只改肘圆上的腕点。送给逆解的 TCP 只用硬球裁剪。日志里的 `d` 可以小于 `d_req`，这本身不表示末端目标被缩短。 |

## 3. 离合：零点怎么取，会吃掉多少距离

位置每次按下 Grip 都重新锚（`activate_clutch()` 约 605–614 行）。朝向默认 `ABSOLUTE_CALIBRATED`：第一次握锁定手与 TCP 的旋转关系，之后再握只重锚位置。

会让末端少走的离合行为，都是**几毫米或一次重锚偏移**，不是随动作变大的比例：

| 行为 | 常数 | 对距离的影响 |
| --- | --- | --- |
| 握下后先空等 | `clutch_settle_ticks = 1`（约 1 ms） | 这一拍把 `hand_ref` 跟到当前手柄，位移被吃掉。 |
| 开始跟踪前的死区 | `clutch_start_motion_m = 0.003`，`clutch_start_rotation_deg = 2` | 平移不到 3 mm 且转角不到 2° 时，持续把 `hand_ref` 跟上手柄，臂不动。 |
| 越过死区后再吃一拍 | `clutch_track_settle_ticks = 1` | 越过 3 mm 的那一拍位置增量也被吸收。 |
| 再握时锚到指令而不是实测 | `reanchor_to_commanded = true`，`max_reanchor_correction_rad = 0.45` | 重力下垂不会在每次再握时被锁进新零点。之后的增量仍相对指令 TCP 按 1:1 加。臂如果已经比指令短，再握不会把这段缺口补上。 |

3 mm 死区能使 20 cm 动作少大约 1.5%。主人描述的差距若明显大于此，原因在到达球、限位或残差，不在离合零点漂移。

超时或连续逆解失败会把 `last_grip_` 写成 false（健康检查 H1）。Grip 仍按住时下一帧会被当成新的握持，零点被刷新。那是安全问题，也会在失败恢复时把「已经走出的增量」清零，看起来像臂突然不再跟手。A1 修的是必须松开再握，本文的遥测不要在那两条分支里改 `last_grip_`。

## 4. 默认 human_mimic：会让目标本身变短，或让臂追不上目标

### 4.1 到达球（欠幅的第一来源）

`solve_base_skeleton()` 约 2029–2043 行：

```text
d_req = |腕目标 − 肩|
d_lo  = |L1 − L2| + reach_margin
d_hi  = L1 + L2 − reach_margin
d_req 超出 [d_lo, d_hi] 时，腕沿肩–腕方向夹回
tip_for_ik = 夹回后的腕 + R * wrist_to_tcp
```

运行时 `reach_margin_m = 0.002`（`OPENARM_HM_REACH_MARGIN`，`make_human_mimic_config()` 约 336 行）。类型里的默认 0.06 被入口覆盖，不要拿 0.06 做判断。

`L1`、`L2` 在 `activate_clutch()` 约 636–649 行：配置里两者 ≤0，于是用当前正运动学的 `|肘−肩|`、`|腕−肘|`。`lock_segment_lengths` 默认 true，第一次成功之后不再重测。弯曲姿态的弦长小于伸直时的骨长，球的半径被锁小，之后即使关节还能展开，目标也不再要求展到真实极限。

URDF 关节原点长度（弦长的上界，不是运行时的 L1/L2）：

| 段 | 关节原点长度（米，近似） |
| --- | --- |
| link1→link2，link2→link3，link3→link4 | 0.067 + 0.073 + 0.157 ≈ 0.30 |
| link4→link5，link5→link6，link6→link7 | 0.101 + 0.126 + 0.038 ≈ 0.26 |

肩到腕的两连杆上限大约 0.56 m，再减去锁死时没展开的那一截。TCP 在腕之外 0.186 m，但裁剪发生在腕上：工具朝外时，末端目标会跟着腕一起停短。

从半屈的握持再向前 30 cm，若腕的剩余行程只有十几厘米，多出来的手部位移不会出现在 `tip_for_ik` 上。这是饱和，不是比例。5 cm 的小动作若离球面还远，这一条不该触发；30 cm 的前伸很容易触发。第 6 节用三个距离把这两者分开。

工作空间盒子在球之后才起作用，阈值是相对握持 TCP 的分量，不是相对肩：

| 方向 | 运行时默认 | 环境变量 |
| --- | --- | --- |
| 向后（`forward_axis` 负方向，默认机器人 +X） | 0.60 m | `OPENARM_HM_MAX_OFFSET` |
| 向前和水平侧向 | 0.85 m | `OPENARM_HM_MAX_OFFSET_FWD` |
| 向上 / 向下 | 0.75 m / 0.80 m | `OPENARM_HM_MAX_OFFSET_UP` / `_DOWN` |

类型默认（向后 0.45、向前 0.45、上 0.55、下 0.80）被入口覆盖。夹紧是分量独立的（约 2514–2546 行）。手的合位移一旦有一个分量顶到盒子，机器人的合位移就严格更短。向外伸时，0.85 m 的盒子通常比第 4.1 节的球更晚碰到。长距离的上提、侧移、后拉，盒子可以先碰到。日志里的 `fwd_off`、`up_off` 顶满对应阈值时，就是这一条。

### 4.2 关节限位（真机比仿真更容易欠幅）

逆解之后 `q_target` 被夹进 `q_min_` / `q_max_`。来源是 URDF，仿真另有加宽（`make_human_mimic_config()` 约 574–604 行）：

| 关节 | URDF（真机） | 仿真额外加宽 |
| --- | --- | --- |
| 右 `joint2` 下标 1 | 下限 **−0.175 rad（约 −10°）**，上限 3.32 rad | 下限默认 −1.047 rad（约 −60°），`OPENARM_HM_QMIN_1_R` |
| 左 `joint2` | 上限 **+0.175 rad（约 +10°）**，下限 −3.32 rad | 上限默认 +1.047 rad，`OPENARM_HM_QMAX_1_L` |
| 肩偏航下标 0 | 约 −200°～+80° 这一侧不对称 | 再放到 ±2.5 rad 的紧侧 |
| 肘 `joint4` 下标 3 | 0～2.443 rad | 不加宽 |
| 腕 `joint6` 下标 5 | ±0.785 rad（约 ±45°） | 不加宽 |
| 腕 `joint7` 下标 6 | ±1.571 rad | 不加宽 |

入口注释写明：这 10° 会挡住大臂摆到身后和向前伸直。仿真能走到的末端位置，真机可能在肩关节先顶死。顶死之后位置误差留在 `e_wrist`。human_mimic 没有笛卡尔那条 `|q_des−q_meas|≤0.30` 的跟踪夹紧；欠幅来自 IK 限位，不是 `SafetyFilter`（健康检查 H2：滤波器目前不夹位置）。

A3 若启用安全滤波，位置上下限必须用这组 URDF，不要用 `openarm_constants.hpp` 里更宽的肘上限 π，也不要把仿真的 ±60° 抄进真机。

### 4.3 逆解残差、每步斜坡、发生器

这些项让臂在手还在动的时候少走一截。手完全停下约 1 秒后，斜坡和发生器应追上 `q_target`。若停稳后 `e_wrist` 仍有厘米级，目标关节本身就没放到 1:1 的 TCP 上。

| 环节 | 常数 | 欠幅方式 |
| --- | --- | --- |
| DIFF-QP | `ik_max_iters = 12`，`ik_kp_pos = 1`，近端每步 ≤0.10 rad，远端 ≤0.30 rad | 单拍可能不收敛，下一拍从 `q_target` 接着做。手静止后应能收敛，除非目标不可达。 |
| 位置对姿态 | `w_wrist_pos = 12`，`w_hand_ori = 8`，连续性 1.6 | 腕关节顶满而姿态仍差时，位置被牺牲一部分。腕饱和时姿态权会降到约 0.35（`wrist_sat_ori_scale`），用来把位置救回来，但是折中仍然存在。 |
| `q_target` 斜坡 | `q_target_slew_max_radps = 10`；肘下标 3 另限 **4.5 rad/s** | 肘打不开时，末端在到达球之前就落后。停手后以不超过 4.5 rad/s 补上，通常远小于 1 秒，除非限位或球已经把目标截断。`elbow_joint_slew_down_scale` 默认 1，展开并不额外变慢，除非设置了 `OPENARM_HM_ELBOW_Q_SLEW_DOWN`。 |
| 关节参考发生器 | `k_joint = 14`，`qdot_max = 6` rad/s，`qddot_max = 80` rad/s² | 小误差时间常数 `1/14≈71` ms。这是 `q_ref` 落后于 `q_target`，停稳后会消失。 |
| 输出低通 | 默认关 | 打开后还有 14 Hz、3.2 rad/s。默认路径上它不参与欠幅。 |

`tip_priority_enabled` 默认 false。软到达因此不会再额外关掉肘任务。肘旋回有 2 rad/s 的 `elbow_cmd_slew_radps`，它改变肘往哪边弯，不直接把 TCP 目标缩短。

### 4.4 真机弹簧落后于指令（仿真看不到）

`fill_reference()` 约 1094–1110 行把每个臂关节的 `velocity` 写成 0。`config/follower.yaml` 的 `Kp = [240,240,240,240,24,31,25,16]`，`Kd = [3,3,3,3,0.2,0.2,0.2,0.2]`，MIT 的 `tau` 为 0。跟随臂的重力不进前馈。

运动中，近端 `Kd*(0−dq)` 与运动方向相反。停下以后，剩余的是重力相对 Kp 的静差：近端大约 `τ_g/240` 弧度。几牛·米对应大约 0.01–0.05 rad，末端大约 1–2 cm。这是估算，**待测量**。它解释不了「少了一大截」，也解释不了仿真里的欠幅。

`src/hardware/mujoco_teleop_hardware.cpp` 的 `send_arm()`：`kp>0` 时直接 `qpos = cmd.q`。仿真末端贴着关节参考走，没有 Kd，也没有重力静差。仿真也短，说明短在 `tip_for_ik` 或 `q_target`；只有真机短，说明短在电机跟踪。

50–150 ms 的 Quest 包被标成陈旧后，控制器不再用新姿态更新目标，关节参考继续走向最后一次 `q_target`（`quest_stale_sec = 0.05`，`quest_timeout_sec = 0.15`）。手已经停、包仍新鲜时，这一条不造成永久欠幅。Wi-Fi 延迟本身不缩放坐标。

## 5. 另外两种模式

### 5.1 cartesian_precision

平移仍然是 `pos_scale = 1`，矩阵与 human_mimic 相同。差别是位移取在**握持瞬间的手柄本体坐标系**里（`update_cartesian_target_from_vr()` 约 851–855 行）：`delta_body = R_grip⁻¹ * (p − p_grip)`，再乘 M。三维长度仍保持。若只拿「房间里向前」和「机器人 +X」比，握持时手柄是斜的，单轴投影会更短。比距离时用两端点的三维合位移。

会截断或落后的常数（`make_cartesian_config()`）：

| 项 | 值 | 作用 |
| --- | --- | --- |
| 相对握持点的球 | `max_teleop_offset_m = 0.55` | 手再远，TCP 目标停在 0.55 m |
| 水平半径 / 高度 | `max_radius = 0.68`，`z ∈ [0.10, 0.90]` | 第二个位置夹紧 |
| 平滑器 | 速度上限 `1e6` | 等于关掉，不制造欠幅 |
| DLS | `kp_pos = 9` 1/s，线速度上限 **1.5 m/s**，关节 **2.2 rad/s** | 手速 0.3 m/s 时，跟踪滞后大约 `0.3/9≈3` cm。停下后时间常数约 111 ms，应补上 |
| 跟踪误差夹紧 | `max_tracking_error_rad = 0.30`，`clamp_tracking_error()` | 指令关节被拉回实测 ±0.30 rad。真机跟不上时，**指令本身被缩短**。仿真 `qpos` 贴着指令，这条夹紧几乎不触发 |
| 姿态 | `rot_scale = 0.6`，腕部相对握持姿态的 ROM 约 90° / 70° / 40° | 只限制转角 |

笛卡尔模式的永久欠幅，优先看 0.55 m 的球和关节软限位；运动中的少走一截，优先看 1.5 m/s 和 2.2 rad/s。

### 5.2 skeleton_proximal（界面勾选会走进来）

`run.sh --skeleton` 把 `VR_TELEOP_MODE` 设为 `skeleton_proximal`。控制台勾选「Quest 肘部与腕部追踪」会给 `run.sh` 加上 `--skeleton`。这条路径**不用手柄位置驱动末端**。肩肘方向来自 WebXR 身体点（`quest/body_tracking.py` 把向量归一化，人体骨长只做 0.08–0.65 m 的合法性检查，不参与缩放）。

`SkeletonProximalConfig`：

```text
elev_robot = elev0 + 1.00 * (elev_human − elev0)     夹在约 −0.20～1.35 rad
azim_robot = azim0 + 0.85 * (azim_human − azim0)
bend_robot = bend0 + 0.85 * (bend_human − bend0)     夹在约 0.45～2.20 rad
肘目标 = 肩 + L1 * u
L1、L2 = 握持时机器人自己的 |肘−肩|、|腕−肘|
```

人手移动 20 cm 时，机器人腕移动大约 `(机器人骨长 / 人手骨长) × 角度增益`。角度增益在方位和弯曲上已经是 0.85，骨长通常也更短，所以**小动作也会按一个小于 1 的比例变短**。这和 human_mimic 的「伸出去才饱和」不同。腕的位置不是任务，远端只跟手柄姿态。

另外：近端斜坡 2 rad/s，`k_joint = 10`，`qdot_max = 3.5`。运动中比默认模式更拖。

若第 6 节的 5 cm 与 15 cm 比例都明显小于 1，并且状态行是 `skel-prox` 而不是 `human-mimic`，原因就是这一节，不必再查 `pos_scale`。

## 6. 测量：把「比手少」拆开

先看启动日志里的 `Teleop mode`。`HUMAN_MIMIC` 用下面的表。`SKELETON_PROXIMAL` 直接按第 5.2 节记录骨长和三个增益。仿真和真机各做一次；仿真也短，就不要先改 Kp/Kd。

现有日志已经能回答一半，不必等新字段：

- 每秒状态在 `recordings/hm_logs/quest_<时间>/status.log`，今天有 `clutch=`、`age_ms=`、`udp=`，没有位移。
- 同目录 `left_hm.log` / `right_hm.log` 默认就写（`debug_log = true`）。行里有 `d_req`、`d`、`dmax`、`fwd_off`、`up_off`、`e_wrist`、`e_ori`、`lim=`、`at_lim=`。握持那一行有 `L1=`、`L2=`。

`d < d_req` 多半是肘的软顶，不代表 TCP 被缩短。TCP 被硬球截断的判据是 **`d_req > dmax`**（`dmax = L1+L2−0.002`）。

### 6.1 手和臂的三段距离

人把右（或左）手柄握在胸前，臂保持明显弯曲。按下 Grip，等 1 秒。然后只平移、尽量不转手腕，沿机器人前方做三次，每次结束后**完全停住 1 秒**再读日志最后一条 `act`：

| 次序 | 手的位移（尺，沿运动方向） | 停 1 秒后要记的数 |
| --- | --- | --- |
| A | 约 5 cm | `d_req`，`dmax`，`e_wrist`，`fwd_off`，`lim`，`at_lim`，L1/L2 |
| B | 约 15 cm | 同上 |
| C | 约 30 cm，或直到臂看起来伸直 / 某关节不再动 | 同上，并看 `fwd_off` 是否顶到 0.85、`up_off` 是否顶到 0.75 或 −0.80 |

手柄合位移用握持帧和停稳帧的 `(px,py,pz)` 做差再取模。页面在进了 AR 之后不刷新屏幕上的坐标，所以这段要么暂时加一行打印（第 7 节的字段），要么在桥的现有每秒日志之外，让控制器把 `px,py,pz` 打进 `act` 行。尺量的是房间里的手，UDP 的模是 Quest 报告的米。两者差超过 10%，才把 Quest 运行时比例当成候选（第 6.3 节）。

期望，按代码推论，尚未在机器上读到：

| 读数 | 说明 |
| --- | --- |
| A 的 `d_req < dmax`，`e_wrist < 5 mm`，`at_lim = 0` | 5 cm 的目标没有被球或限位截断。停稳后末端应接近 5 cm。若真机末端仍短 1–2 cm 而仿真不短，看第 4.4 节的静差。 |
| A、B 的「末端/手」比例几乎相同且明显小于 1，模式却是 human_mimic，UDP 与尺一致 | 与「增益为 1、截断是饱和」不符。先核对是不是看了单轴、或停稳不足 1 秒仍在吃 4.5 rad/s 的肘斜坡。 |
| C 出现 `d_req > dmax`，`e_wrist` 很小 | 目标被球夹短。末端少走的量大约是腕超出 `dmax` 的那一截（工具朝向与运动方向一致时最接近）。这就是第 0 节第 1 条。 |
| `lim < 0.05` 且 `at_lim > 0`，同时 `e_wrist` 为厘米级 | 关节顶死，位置没跟上。真机若顶在下标 1，就是肩 joint2 的 10°。仿真同一动作若 `at_lim = 0` 且末端更远，与第 4.2 节一致。 |
| 刚停时短、1 秒后 `e_wrist` 降到几毫米且末端补上 | 欠幅来自肘 4.5 rad/s 或发生器，不是永久截断。 |
| `fwd_off` 等于 0.85 或 `up_off` 顶到 ±上下限，而 `d_req` 仍小于 `dmax` | 工作空间盒子先夹紧。 |

真机读末端之前，看 CAN 反馈年龄。PR #4 / A2 落地前，遥操循环不检查年龄，电机位置默认 0。年龄超过 150 ms 或从未收到时，用实测关节算出的末端不能拿来和手比。

### 6.2 不带头显的对照

向 `127.0.0.1:47911` 发 Grip=1 的 CSV，位置沿 Quest −Z（映射到机器人 +X）以 0.05 m 的台阶增加，每步停 200 ms。PR #3 把姿态口收成回环之后，这条注入仍然有效。比较 `p_tcp_des` 的模和 0.05×步数。在到达 `dmax` 或盒子之前，指令模应与注入模一致（允许 3 mm 死区）。越过之后指令模停止增长。这样把 Quest 无线和页面从问题里拿掉。

## 7. 界面要显示的量：控制侧需要打出的字段

今天的 1 Hz 行只有 `clutch`、`grip`、`age_ms`、`udp`。`e_wrist` 和 `d_req` 在 `*_hm.log`，界面读不到。下面这组按臂输出，建议 10 Hz，文本键值，方便控制台以后直接显示「映射比例、离合、手与臂的位移差」。先加字段，不要为了显示去改映射。

已有、只需从控制器读出的：`clutch_active()`、`last_wrist_err()`、握持日志里的 L1/L2。其余要在 `step()` 里留下只读量。

| 字段 | 含义 | 界面怎么用 |
| --- | --- | --- |
| `mode` | `human_mimic` / `cartesian_precision` / `skeleton_proximal` / `pika_style` | 比例的解释取决于模式 |
| `clutch` | `off` / `settle` / `idle` / `track` / `frozen` | 离合状态。`idle` 表示还在 3 mm 死区里 |
| `pos_scale` | 当前平移增益，默认 1 | 「映射比例」的配置值。骨骼模式没有这个增益，改显示三个角度增益 |
| `hand_m` | 相对本次 `hand_ref` 的手柄位移模，米 | 手移动了多少 |
| `cmd_m` | 夹紧之后 `|p_tcp_des − robot_ref|` | 控制器真正要求的末端位移 |
| `tcp_m` | `|FK(q_target) − 握持时的 TCP|` | 逆解关节对应的末端位移 |
| `meas_m` | `|FK(q_meas) − 握持时的 TCP|` | 臂实际走了多少。CAN 不新鲜时填空并置 `meas_stale=1`，禁止用 0 位姿 |
| `ratio_cmd` | `cmd_m / hand_m`（`hand_m < 5 mm` 时为空） | 映射比例。1 附近表示目标没被夹短 |
| `ratio_tcp` | `tcp_m / hand_m` | 逆解相对手的比例。低于 `ratio_cmd` 表示解没跟上目标 |
| `ratio_meas` | `meas_m / hand_m` | 臂相对手的比例。低于 `ratio_tcp` 表示电机没跟上指令 |
| `gap_cmd_m` | `hand_m − cmd_m` | 手与指令的位移差，正值即欠幅 |
| `gap_meas_m` | `hand_m − meas_m` | 手与臂的位移差 |
| `d_req_m` `dmax_m` `reach_clip` | 肩–腕请求、硬上限、`d_req>dmax` 则为 1 | 解释比例为什么掉下去 |
| `box_clip` | `fwd_off` 或 `up_off` 或侧向顶到第 4.1 节的阈值则为 1，并带 `fwd_off` `up_off` | 工作空间夹紧 |
| `e_wrist_m` `e_ori_rad` | 现有逆解残差 | 停稳后仍大，就是第 0 节第 3 条 |
| `lim_rad` `lim_joint` `at_lim` | 现有限位裕度 | 哪个关节顶死 |
| `q_lag_rad` | `max \|q_target − q_ref\|` | 发生器还落后多少。停稳后应趋近 0 |
| `L1_m` `L2_m` | 锁死的弦长 | 和 URDF 上界约 0.30 / 0.26 比较，能看出球是不是在弯曲时被锁小 |
| `can_age_ms` | 最旧的一个臂关节反馈年龄 | 给 `meas_*` 做守门 |
| 骨骼模式另加 | `elev_gain` `azim_gain` `bend_gain`（0.85/0.85）、人体肩–肘与肘–腕长度、机器人 L1/L2 | 这个模式的「映射比例」是角度增益和骨长比，不是 `pos_scale` |

`ratio_cmd`、`ratio_tcp`、`ratio_meas` 分开之后，界面可以显示一条比例，并用颜色或第二行指出短在「目标被夹」「逆解没跟上」还是「电机没跟上」。不要把三者乘成一个无法追查的数。

## 8. 修改任务，以及和正在做的安全改动怎么错开

不在本文件里改代码。顺序假定 A1（human_mimic 必须松开再握）先合并；遥测可以只加只读量，不碰 `last_grip_` / `activate_clutch()`。

| 任务 | 规模 | 文件 | 和安全改动的关系 |
| --- | --- | --- | --- |
| T1. 打出第 7 节的字段，10 Hz 一行，默认进现有 `hm_logs`，`OPENARM_HM_STATUS_CONSOLE=1` 时上控制台 | S | `src/controller/human_mimic_controller.hpp` 增加只读访问；`human_mimic_controller.cpp` 的 `step()` / `maybe_log()`；`control/openarm_quest_cartesian_control.cpp` 里现有的 `human-mimic` 状态行。骨骼、笛卡尔各加自己的比例字段 | 与 A1 同一个 cpp。只增加局部变量和一行输出，不改冻结分支里的 `last_grip_`。`meas_*` 在 CAN 年龄未知或超过 A2 的 150 ms 时标记 `meas_stale`，避免 PR #4 之前把默认 `q=0` 显示成「臂在零位、比例为 0」。不新开 UDP 端口，避免和 B1 / PR #3 的回环绑定缠在一起。 |
| T2. 控制台显示模式、`clutch`、`pos_scale` 或骨骼增益、`ratio_cmd`、`gap_meas_m` | S | `script/openarm_app/main_window.py` 解析子进程日志 | 不改 C++ 控制律。解析失败时保持现在的日志窗，不要假装比例是 1。 |
| T3. 到达球用伸直骨长，而不是弯曲握持的弦长；硬裁剪时打 `reach_clip=1`。软顶继续只作用于肘 | M | `human_mimic_controller.cpp` 的 `activate_clutch()` 与 `solve_base_skeleton()`；必要时 `human_mimic_types.hpp` 增加「最大骨长」而不是把 `pos_scale` 改掉 | 等 A1 合并后再改这两处函数。不要用放宽 URDF 限位换行程：A3 的位置限必须来自 URDF。球仍然要留在奇异点外那 2 mm，避免伸直后腕关节失去姿态能力。 |
| T4. 用 T1 的 `lim_joint` 确认真机前伸是否顶在下标 1。若是，由负责人决定机械上能否加大 joint2 行程；软件在确认前保持 URDF | S（只确认）/ M（若改限位） | 日志即可。改限位才会动 `assets/v1.urdf` 与仿真里的 `OPENARM_HM_QMIN_1_R` / `QMAX_1_L` 默认 | 仿真 ±60° 不能在 A3 里被当成真机安全限。PR #4 的滤波若夹位置，用 URDF 的 −0.175 / +0.175，不要用仿真覆盖值。 |
| T5. 停稳后 `e_wrist` 仍大于约 1 cm、且没有 `reach_clip` 也没有 `at_lim` 时，再把位置残差做成更硬的约束（提高 `w_wrist_pos` 或在腕饱和时进一步降低姿态权）。一次只改一个数 | M | `human_mimic_types.hpp`，`human_mimic_ik_solver.cpp` | 更硬的位置跟踪会加大 `|q_des−q_meas|`。放在 A3 的跟踪误差上限有测试里的具体弧度之后。不要为了补距离关掉那个上限。 |
| T6. 真机欠幅只出现在 `ratio_meas`、`ratio_tcp≈1` 时，再考虑把发生器的 `qdot` 送进 MIT `dq`（遥操作方案里的 L3） | M | `joint_reference_generator.hpp`，`fill_reference()` | 与 A3 绑在一起：`dq` 必须对应夹紧之后的 `q`。A2 规定没有新鲜 CAN 就不发带 Kp 的命令；T6 不绕过发送条件。仿真 `qpos` 跟随看不出这项。 |
| T7. 骨骼模式在界面上单独显示骨长比和 0.85 的方位/弯曲增益；若希望末端跟手柄位置，那是换映射，不是把 `pos_scale` 调到 1 | L | `skeleton_proximal_controller.cpp`，`skeleton_proximal_types.hpp`，`quest/body_tracking.py` | 不在 A1 的文件集合里，可以并行，但不要顺手改 human_mimic 的到达球。丢骨骼后的重新握持保持现状（健康检查里骨骼模式已经要求真的松开再握）。 |

不要做的事：把 `pos_scale` 调到大于 1 去「补偿」到达球或限位，那会在小球动作上超调，并在球面上把目标推得更不可达。不要打开 `OPENARM_HM_Q_FILT`。不要在 A3 之前把 `qdot_max` 再往 6 rad/s 以上加。

## 9. 这次没有验证的事项

- 没有在仿真或真机上做第 6 节的 5 / 15 / 30 cm 试验，因此第 0 节的排序是按代码路径的饱和点，不是按一次实测的比例。
- 没有用尺核对 Quest `local-floor` 的米是否与房间一致。页面和桥都不缩放；若尺和 UDP 不一致，那是运行时或场地尺度，本仓库没有对应常数。
- 重力静差的 1–2 cm 是 `Kp=240` 的量级估计，不是电流测量。
- 没有单步 H1 的重新握持。离合表里的 3 mm 来自常数和 `hand_ref` 的赋值，不是示波器。
- L1/L2 在弯曲握持时比伸直弦长短多少，取决当时姿态，日志里的 `L1=` `L2=` 才能给出这次会话的球半径。
