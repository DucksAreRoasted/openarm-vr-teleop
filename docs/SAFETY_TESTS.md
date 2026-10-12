# 安全相关测试

阈值只写在 `tests/safety_contract.hpp`。Python 从这份头文件读出同一组数，测试里不要再抄一份。

| 常量 | 值 | 含义 |
| --- | --- | --- |
| `kQuestTimeoutSec` | 0.15 s | Quest 数据超时后 `human_mimic` 断开离合 |
| `kSolveFailFreezeCount` | 30 | 连续 IK 失败超过该次数后断开离合（比较符是 `>`） |
| `kCanFeedbackMaxAgeMs` | 150 ms | 反馈年龄超过该值视为无效（等于 150 ms 仍算有效） |
| `kCanErrorCodeMaxOk` | 1 | `error_code` 大于该值视为故障。**待硬件确认** |
| `kElbowUrdfUpperRad` | 2.443461 rad | URDF `openarm_*_joint4` 的上限，不是 `openarm_constants.hpp` 里的 π |

不测 `|q_des - q_meas|` 的 0.25 rad 跟随上限，也不测速度/力矩上限。这两项目前是 0，表示不限制。

## 已经在 main 上直接断言的行为

- **离合现状**（`human_mimic_clutch_test`，链接真实的 `HumanMimicController`，不开 CAN）：Quest 超时、无效包，或 IK 连续失败超过 30 次后离合断开。Grip 仍按住时，下一帧被当成上升沿重新接上，并且 `activate_clutch()` 把 IK 失败计数清零。短于超时的掉帧不会断开。真正松开 Grip 再按住才会重新接上；松开后保持松开则保持断开。这是 weiqiang 确认要保留的行为。
- **SafetyFilter 非有限值**：NaN / ±Inf 不会下发。位置保持上一帧有限指令。有限且在限幅内的指令照常通过。
- **CAN 分类规则**：用达妙 `Motor` 对象检查「从未收到回包」「年龄超过 150 ms」「`error_code > 1`」都算无效；`error_code` 为 0 或 1 且回包新鲜则有效。同向示教 `validate_feedback()` 里的字面量和这份契约一致。`error_code > 1` 仍标注待硬件确认。
- **肘关节 URDF**：左右 `openarm_*_joint4` 的 upper 等于契约里的 2.443461 rad。

## 在 main 上跳过、等别的 PR 的行为

这些用例调用 `skipTest`，CI 保持绿色。对应修复合并后，跳过条件不再成立，同一条断言会开始执行。

| 测试 | 原因 | 解除条件 |
| --- | --- | --- |
| `test_out_of_range_reference_clamps_to_urdf_elbow` | `SafetyFilter::filter()` 还不读位置上下限 | 超限参考被夹紧。夹紧上限是 URDF 肘关节 2.443461 rad。测试里有一条超过 URDF、但仍小于 π 的指令，用来区分 URDF 和 `openarm_constants.hpp` |
| `test_default_safety_config_elbow_follows_urdf` | `make_default_safety_config()` 的肘上限仍是 π | 默认安全配置的位置上限改为 URDF |
| `test_startup_refuses_invalid_feedback` | `SeedHoldMeasured`、零重力、MoveIt `real_arm` 还不检查反馈 | 启动时无效反馈（年龄、`error_code > 1`、从未回包）拒绝进入出力循环 |
| `test_runtime_stale_feedback_disables` | 上述三条运行路径没有反馈看门狗 | 运行中反馈过期则停止下发，并调用现有 `disable_all()` |

## 怎么跑

```bash
./install.sh
# Ubuntu 24.04 还需要 ROS Jazzy 的 kdl_parser（以及 ament_cmake）。
# 22.04 把下面的 jazzy 换成 humble。
sudo apt-get install -y ros-jazzy-kdl-parser ros-jazzy-ament-cmake ros-jazzy-ament-cmake-libraries
export CXX=g++
export CMAKE_PREFIX_PATH=/opt/ros/jazzy
export AMENT_PREFIX_PATH=/opt/ros/jazzy
cmake -S . -B build -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build --target human_mimic_clutch_test -j
.venv/bin/python -m unittest discover -s tests -v
node tests/test_body_tracking.cjs
node tests/test_quest_session.cjs
```

`human_mimic_clutch_test` 只编译 human_mimic 这条控制链，不链接 MuJoCo，也不打开 CAN。这样 CI 不必编过整份 `vr_control_core`。

`run.sh` 和 `script/replay_teaching.py` 的场景路径不依赖当前工作目录，规则相同：

```bash
cd /tmp && /path/to/openarm-vr-teleop/run.sh --print-paths
```

持续集成见 `.github/workflows/tests.yml`。它在 push 和 pull request 上安装依赖、编译离合测试，并跑上面的 Python 与 Node 检查。不连接 Quest、CAN 或真机。
