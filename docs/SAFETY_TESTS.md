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
- **SafetyFilter 非有限值**：NaN / ±Inf 不会下发。位置保持上一帧有限指令。有限且在限幅内的指令照常通过。这条用当前 `main` 上的 `filter()` 直接断言。
- **CAN 分类规则**：用达妙 `Motor` 对象检查「从未收到回包」「年龄超过 150 ms」「`error_code > 1`」都算无效；`error_code` 为 0 或 1 且回包新鲜则有效。同向示教 `validate_feedback()` 里的字面量和这份契约一致。`error_code > 1` 仍标注待硬件确认。
- **肘关节 URDF**：左右 `openarm_*_joint4` 的 upper 等于契约里的 2.443461 rad。

## PR #4 的纯函数

这些测试调用 PR #4 头文件里的函数，不打开 CAN，也不起周期线程。当前分支还没有这些声明时，用例 `skip`，CI 保持绿色。声明一出现就编译并断言，不需要把本分支叠到 #4 上。

| 测试 | 调用 | 断言 |
| --- | --- | --- |
| `test_evaluate_can_feedback_and_null_arm` | `evaluate_can_feedback`、`can_feedback_allows_torque(nullptr)` | 空样本和纪元时间戳无效；年龄等于上限仍有效，超过则过期；`error_code` 等于上限接受，更大拒绝且优先于过期；未来时间戳不当作过期；空指针不允许进入出力 |
| `test_production_thresholds_match_contract` | `openarm_constants.hpp` | 生产代码里的年龄和 `error_code` 上限与 `safety_contract.hpp` 相同 |
| `test_urdf_clamp_hold_stop_and_size_mismatch` | `urdf_arm_position_limit`、`hold_previous_joint`、`filter`、`make_explicit_stop_command` | 肘夹到 URDF 2.443461（含大于 URDF 但仍小于 π 的指令）；夹爪索引不夹；非有限值保持整关节；`Stop` 和尺寸不匹配锁存停止，速度和力矩为 0，且不抛异常 |
| `test_late_deadline_is_one_period_after_now` | `plan_next_deadline` | 迟到后下一拍是 now 加一个周期，不补跑错过的回调 |

不测 `|q_des - q_meas|` 的 0.25 rad。速度/力矩上限仍为 0 表示不限制；只检查 0 不会把数值夹掉。

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
