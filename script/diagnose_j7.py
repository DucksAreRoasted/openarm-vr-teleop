#!/usr/bin/env python3
"""Prepare passive captures and analyze follower diagnostics; never opens CAN."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shlex
import shutil
import statistics
import time

ROOT = Path(__file__).resolve().parents[1]


def prepare(directory, label, source):
    directory.mkdir(parents=True, exist_ok=False)
    files = [ROOT / 'config/follower.yaml', ROOT / 'config/gravity_comp.yaml', ROOT / 'assets/v1.urdf']
    if source:
        files.append(Path(source).resolve())
    snapshots = []
    for i, path in enumerate(files):
        target = directory / f'{i}_{path.name}'
        shutil.copy2(path, target)
        snapshots.append({'path': str(path), 'snapshot': target.name,
                          'sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
    env = {k: v for k, v in __import__('os').environ.items()
           if k.startswith(('OPENARM_QUEST_', 'OPENARM_GRIPPER_', 'OPENARM_GRAVITY_', 'OPENARM_CONTROL_'))}
    (directory / 'metadata.json').write_text(json.dumps(
        {'created_unix': time.time(), 'label': label, 'files': snapshots, 'environment_at_prepare': env,
         'note': 'Capture settings in the same shell; label should include side, load kg and mode.'},
        indent=2, ensure_ascii=False)+'\n')
    print(f'export OPENARM_CONTROL_DIAGNOSTICS_DIR={shlex.quote(str(directory.resolve()))}')
    print('export OPENARM_CONTROL_DIAGNOSTICS=1')


def percentile(values, fraction):
    values = sorted(values)
    return values[min(len(values)-1, int((len(values)-1)*fraction))] if values else None


def summarize(rows):
    """Candidate evidence only: torque feedback is not an external force sensor."""
    if not rows:
        return {'samples': 0, 'findings': ['无有效 J7 样本']}
    error = [abs(r['cmd_q']-r['q']) for r in rows]
    source_lag = [abs(r['source_q']-r['cmd_q']) for r in rows]
    static = [r for r in rows if abs(r['dq']) < .05 and abs(r['cmd_dq']) < .05]
    soft = [r for r in static if abs(r['cmd_q']-r['q']) > math.radians(3)]
    ff_clamped = [r for r in rows if abs(r['gravity_requested']) > r['ff_limit']+1e-6]
    protocol = [r for r in rows if abs(r['ff']) > r['protocol_limit'] and r['protocol_limit'] > 0]
    faults = [r for r in rows if r['status'] > 1 or r['age_ms'] > 150]
    findings = []
    if soft:
        findings.append('低速/静止时仍有 >3° 跟踪误差：负载模型、摩擦或位置刚性不足候选；需要同姿态空载/带载对照')
    if ff_clamped:
        findings.append('重力前馈发生限幅；这不是 PD+FF 总力矩限幅')
    if protocol:
        findings.append('FF 超出电机协议编码范围；检查传输裁剪')
    if faults:
        findings.append('存在反馈超时或电机状态异常；先调查 CAN/驱动日志')
    if percentile(source_lag, .95) > .03:
        findings.append('轨迹目标与实际 MIT 指令存在参考过滤延迟；与机械跟踪误差分别处理')
    if not findings:
        findings.append('当前样本未达到诊断阈值；不能证明重载问题已消失')
    return {
        'samples': len(rows), 'duration_s': rows[-1]['mono']-rows[0]['mono'],
        'kp_values': sorted({r['kp'] for r in rows}), 'kd_values': sorted({r['kd'] for r in rows}),
        'tracking_error_p95_deg': math.degrees(percentile(error, .95)),
        'tracking_error_max_deg': math.degrees(max(error)),
        'source_command_lag_p95_deg': math.degrees(percentile(source_lag, .95)),
        'static_samples': len(static), 'static_error_over_3deg_samples': len(soft),
        'gravity_ff_clamped_samples': len(ff_clamped), 'protocol_overrange_samples': len(protocol),
        'fault_or_stale_samples': len(faults), 'cycle_dt_p99_ms': 1000*percentile([r['dt'] for r in rows], .99),
        'max_rotor_temperature': max(r['rotor'] for r in rows),
        'max_estimated_total_Nm': max(abs(r['total_estimate']) for r in rows),
        'max_reported_torque_Nm': max(abs(r['torque']) for r in rows),
        'findings': findings,
    }


def analyze(directory):
    reports = {}
    for path in sorted(directory.glob('*.csv')):
        with path.open(newline='') as file:
            reader = csv.DictReader(file)
            if 'gravity_requested' not in (reader.fieldnames or []):
                continue
            rows = []
            invalid = 0
            for row in reader:
                try:
                    values = {k: float(v) for k, v in row.items()}
                    if not all(math.isfinite(v) for v in values.values()):
                        raise ValueError('non-finite')
                    if values['joint'] == 7:
                        rows.append(values)
                except (ValueError, TypeError):
                    invalid += 1
        # Report both phases, so startup cannot hide a playing-only failure.
        reports[path.name] = {'all': summarize(rows), 'playing': summarize([r for r in rows if r['phase']==2]),
                              'invalid_rows': invalid}
        stats = Path(str(path)+'.stats')
        reports[path.name]['writer_status'] = stats.read_text() if stats.exists() else '采集中或异常退出；丢帧计数未知'
    return reports


def legacy(directory):
    """Historical q action/observation audit. No invented measured torques/gains."""
    path = directory / 'lerobot_episode.csv'
    with path.open(newline='') as file:
        rows = list(csv.DictReader(file))
    errors = [abs(float(r['r_act6'])-float(r['r_obs6'])) for r in rows]
    return {'source': str(path), 'samples': len(rows), 'right_J7_tracking_max_deg': math.degrees(max(errors)),
            'right_J7_tracking_p95_deg': math.degrees(percentile(errors, .95)),
            'old_replay_kp': 8, 'teleop_default_kp': 25,
            'PD_at_6deg_old_Nm': 8*math.radians(6), 'PD_at_6deg_aligned_Nm': 25*math.radians(6),
            'limitation': '历史文件为遥操录制；没有实机回放逐关节反馈、确切历史增益、温度或负载重量，不能据此判定回放限幅/驱动故障。'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='command', required=True)
    p = subs.add_parser('prepare')
    p.add_argument('--directory', type=Path, default=ROOT/'recordings/control_diagnostics'/time.strftime('%Y%m%d_%H%M%S'))
    p.add_argument('--label', default='right J7; mode/load kg unknown')
    p.add_argument('--source', type=Path)
    for mode in ('analyze', 'legacy'):
        p = subs.add_parser(mode)
        p.add_argument('directory', type=Path)
        p.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.command == 'prepare':
        prepare(args.directory, args.label, args.source)
        return
    report = analyze(args.directory) if args.command == 'analyze' else legacy(args.directory)
    result = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(result)
    print(result, end='')


if __name__ == '__main__':
    main()
