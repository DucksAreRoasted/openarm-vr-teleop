# 实机遥操重力前馈与断流恢复

2026-10-12：Quest 实机跟随控制默认将基于实测角度的重力前馈叠加到 MIT PD 指令。配置及开关见 [README](../README.md)。模型采用独立的 body_link0 → link7 链，复用零重力标定中的质量/质心、相机、夹爪安装质量、倍率与偏置，不将 IK 的 hand_tcp 固定偏移误用为质量坐标。双臂模型均在使能任一实臂前初始化。

此前实际 MIT 前馈始终为零。现有 `/tmp/openarm_vr_teleop_app.log` 中 CAN 双臂 PIKA 会话还出现了握持期间反复 Quest 超时（约 150 ms）、恢复后从 measured 重新锚定，以及控制周期超时。这些是排查线索，不能单凭日志断定用户感受到的每次下坠原因。

无硬件回归复现了断流恢复的目标降低：给实测关节角引入 0.06 rad 滞后，超时保持后持续握住恢复，旧代码将实测角设为新目标。新代码沿用保持指令，连续三次恢复均不降低目标。真实松握再握仍保留所选模式的锚定行为。该修复不改变超时阈值，也不跳过 CAN 故障与安全停机失能。

验证命令（均不连接电机）：

```bash
cmake -S . -B build -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build -j 4 --target teleop_gravity_test human_mimic_clutch_test quest_cartesian_control unitree_style_test openarm_teaching_replay openarm_zero_g
.venv/bin/python -m unittest discover -s tests -p 'test_teleop_gravity.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_human_mimic_clutch.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_unitree_style.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_can_feedback_contract.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_sim_startup.py' -v
git diff --check
```

结果：编译通过，以上 11 项 Python 测试通过，差异空白检查通过。前馈测试在内存硬件后端验证实际 MIT 打包，覆盖左右臂、实测角计算、渐入、限幅、保持及运动、重设保持不重启渐入、异常姿态与显式安全停机清零、停止锁存，以及遥测与实际发送力矩一致。仿真起手测试覆盖两种遥操模式与原始轨迹中间帧、时长及中断。

未执行实臂，也未改写标定文件。物理补偿效果、负载是否与当前 YAML 一致、以及下坠是否还有电机故障/CAN 断流原因需要实机验收；运行前须确认急停与左 can1、右 can0 映射。
