# 2026-10-12 分支整合记录

初次整合分支：`integration/20261012`，基线为 `main` 的 `73dc004`。
开始前已刷新 origin，原工作区无未提交改动。初次在独立目录完成整合，随后合入包含设备支持清理提交 `de11537` 的本地 `main`；未推送远端。

## 分支范围

| 来源分支 | 原提交 | 纳入内容 |
| --- | --- | --- |
| `cursor/quest-udp-loopback-b57a` | `89cb1cd` | Quest UDP 默认回环监听，采集快捷键沿用地址配置 |
| `cursor/can-freshness-safetyfilter-timeout-6345` | `a73c70c` | CAN 反馈新鲜度、URDF 位置过滤、周期超时不补拍 |
| `test/integration-0001` | `b17500b` | 已有的上述安全与 UDP 整合历史 |
| `cursor/ci-safety-tests-0f82` | `e50d5e5` | 无硬件 CI、安全契约测试、统一场景解析规则 |
| `cursor/qt-record-replay-ux-dd1e` | `9fe3f62` | 就绪后开放录制、重录确认、回放圈数、停止超时处理 |
| `cursor/repo-health-check-a4e9` | `45e25f4` | 仓库健康检查快照 |
| `cursor/teleop-recording-plan-9925` | `22514f6` | 跟手与录制回放方案快照 |
| `cursor/left-hand-mapping-diagnosis-8e8e` | `4db4bcf` | 左臂映射诊断快照 |
| `cursor/motion-mapping-diagnosis-8fd9` | `2dfc442` | 位移与行程诊断快照 |
| `cursor/sim-start-pose-plan-455c` | `ead7758` | 共用起手姿势规划快照 |

文档分支的规划未自动转化为控制实现。例如 IK 降频、录制异步写盘、关节速度前馈、左右映射修改和共用起手姿势，仍是后续任务。

## 合并修正

- README 同时保留 UDP 地址说明与仿真场景回退规则。
- 采集测试同时采用新的就绪／确认流程与显式回环测试地址。
- 测试契约里的 CAN 年龄常量改名，避免与生产代码重名；保留生产阈值一致性断言。
- 就绪日志作为完整一行输出，避免启动期并发日志使界面漏识别就绪。
- 显式导入 `websockets.exceptions.ConnectionClosed`，修复断连时异常类型查询出错，并增加回归测试。
- Quest 页面、README 与操作说明同步重录确认流程；历史诊断文档标注为快照。

## 使用变化

启动完成前禁止录制。左摇杆短按开始／结束，停录后右摇杆短按成功、长按失败。
重录是左摇杆按住 1.5 秒，到点把当前条移入回收站；10 秒内再按住一次可撤销。
PC 的“丢弃重录”按钮也会要求确认。

循环播放、首帧慢速接入、暂停／继续和结束后保持末帧的行为保留。
回放进度显示当前遍数与总次数，持续循环显示 `∞`。

## 验证

在 Ubuntu 22.04、ROS Humble 以及现有 Python 虚拟环境上执行：

```bash
cmake -S . -B build -DPython3_EXECUTABLE=/home/ducks/openarm-vr-teleop/.venv/bin/python -DCMAKE_PREFIX_PATH=/opt/ros/humble
cmake --build build -j2
/home/ducks/openarm-vr-teleop/.venv/bin/python -m unittest discover -s tests -v
node tests/test_body_tracking.cjs
node tests/test_quest_session.cjs
bash -n run.sh
git diff --check
```

- 默认 C++ 全量构建通过，包含遥操、回放、零重力、标定及离合测试。保留原有编译警告。
- ROS 2 实机适配器 `real_arm` 单独配置与编译通过：`cmake -S ros2/src/openarm_moveit_config -B /tmp/openarm-integration-ros-build -DCMAKE_PREFIX_PATH=/opt/ros/humble -DPython3_EXECUTABLE=/usr/bin/python3`，随后运行 `cmake --build /tmp/openarm-integration-ros-build --target real_arm -j2`。未运行 MoveIt 或连接真机。
- 合并前历史验证：Python 共 94 项：92 项通过，2 项因当前环境缺少可选设备 SDK 跳过。CAN、安全过滤和周期截止时间测试实际运行，未跳过。
- 两组 Node 测试、Shell 语法和差异检查通过。
- 无窗口 MuJoCo：桥接 → Qt → 控制器的开始／停止、成功／失败、二次确认重录、序号复用和 OpenArm Dataset 导出通过。数据使用自动清理的临时目录。
- 无窗口 MuJoCo：两遍回放、返回首帧途中暂停／继续、持续循环及停止释放通过。
- Qt 离屏截图位于 `/tmp/openarm-integration-console.png` 和 `/tmp/openarm-integration-replay.png`，仅作为本次本地验证产物。
- 本次临时仿真验证脚本位于 `/tmp/openarm_integration_capture_smoke.py` 和 `/tmp/openarm_integration_repeat_smoke.py`，未纳入版本控制。

没有连接 Quest、CAN 或真机，也没有验证真实相机采集、真机闭环稳定性和碰撞安全。
GitHub Actions 的 Ubuntu 24.04 / ROS Jazzy 环境需推送后由远端执行，本次本地测试不代表远端 CI 已通过。

## 合入 main 后验证

- 保留 `de11537` 的设备支持清理结果，移除整合分支带入的过时说明及专用测试；当前源码无该设备控制入口。
- 冲突处理保留 Quest 就绪后录制、二次确认重录、UDP 地址说明及场景路径查询。
- `cmake -S . -B build -DPython3_EXECUTABLE="$PWD/.venv/bin/python"` 和 `cmake --build build -j2` 通过。
- Python 71 项测试全部通过：首次全套运行时离合测试因并行构建尚未完成而跳过，构建完成后单独运行 `test_human_mimic_clutch.py` 通过。
- 两项 Node 检查、Shell 语法、差异检查通过。
- 无窗口仿真采集就绪、录制／停录、成功／失败、二次确认重录、导出，以及有限／无限循环、暂停／继续和停止释放通过。
- 合并后的临时采集脚本为 `/tmp/openarm_main_merge_capture_smoke.py`，回放脚本为 `/tmp/openarm_repeat_smoke.py`；未连接真实设备。

## 切换到此版本

```bash
git switch main
cmake -S . -B build -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build -j2
./app.sh
```

切换后需重新编译，避免沿用旧可执行文件；Quest 页面也需刷新以更新快捷键提示。

## Quest 默认方案与追踪清理

用户确认宇树与 PIKA 可替换旧默认后，默认切换为 `unitree_style`，控制台并列提供 `pika_style`。移除“肘部与腕部追踪”入口，以及其 WebXR 身体追踪、骨骼 UDP 接收、HybrIK 提示、专用控制器和 MuJoCo 骨骼显示。旧版手柄模式与虚拟手臂 IK 仍可使用。上文身体追踪测试为合并时的历史记录，相应测试文件现已删除。

清理后的验证：全量 C++ 构建通过；Python 65 项测试通过；`test_quest_session.cjs`、Shell 语法及差异检查通过。宇树、PIKA 双臂合成 Quest UDP 仿真均完成三轴移动、旋转、不可达伸展、松握重接及断流恢复，左右臂参考均在 URDF 限位内。新默认下端到端录制、停止、成功/失败标注、确认重录、序号复用和导出通过。没有运行真机。

用户进一步确认仅保留宇树与 PIKA 后，移除旧版 `human_mimic` 和 `cartesian_precision` 的 UI/CLI 入口；删除独立 Cartesian DLS 控制器、旧模式配置和 Kalman 滤波器。Quest 输入与遥测状态移至 `quest_types.hpp`。`HumanMimicController` 等内部名称仍代表宇树与 PIKA 共用的 IK、离合及参考生成实现，不再是可选运行模式。

仅保留两种模式后的验证：全量构建、Python 70 项、Node 会话测试、Shell 语法和差异检查通过；旧模式参数已确认会立即拒绝。宇树/PIKA 的双臂轨迹和单臂握持、移动、超时冻结、正常退出均通过，双臂参考在 URDF 限位内。录制/标注/重录/导出通过。按当前预览接口验证两次循环、暂停/继续、无限循环和停止通过。所有验证均未连接真机。

## 同向示教移除

移除控制台及 `run.sh` 的同向示教入口，删除右臂主导左臂的映射和控制循环。真机回放需要的反馈时效检查、速度保护、重力前馈和关节跟踪误差限制整理到 `ReplayArm`，回放继续按双臂 CSV 指令播放。旧操作文档与专用映射测试已删除。

验证：全量 C++ 构建、Python 66 项既有测试及 1 项新增的回放反馈测试、Quest 会话 Node 检查与 `bash -n run.sh` 均通过。控制台 Quest 采集经仿真完成录制、成功/失败标注、重录与导出；回放预览完成两遍循环、暂停继续和无限循环停止。`--mirror`、`--mirror-teach` 已拒绝。真机回放控制代码仅编译和无硬件契约检查，未在 CAN 实机运行。
