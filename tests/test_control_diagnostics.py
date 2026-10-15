import csv
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('diagnose_j7', ROOT/'script/diagnose_j7.py')
diagnose = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnose)


class ControlDiagnosticsTests(unittest.TestCase):
    def test_background_writer_drains_and_accounts_for_loss(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = directory/'writer.cpp'
            binary = directory/'writer'
            source.write_text(r'''
#include <io/control_diagnostics.hpp>
int main(int argc, char** argv) {
    if(argc!=2) return 2;
    setenv("OPENARM_CONTROL_DIAGNOSTICS_DIR", argv[1], 1);
    setenv("OPENARM_CONTROL_DIAGNOSTICS", "1", 1);
    auto logger=ControlDiagnostics::create("right_arm");
    if(!logger) return 3;
    ControlDiagnostics::Frame f;
    f.phase=2;
    for(auto& j:f.joints) {
        j.cmd_q=.1; j.q=0; j.kp=25; j.ff=.4;
        j.ff_limit=2; j.protocol_limit=10;
    }
    for(int i=0;i<1000;++i) { f.mono=i*.01; logger->submit(f); }
    logger.reset();
    setenv("OPENARM_CONTROL_DIAGNOSTICS", "0", 1);
    return ControlDiagnostics::create("off") ? 4 : 0;
}
''')
            subprocess.run(['g++', '-std=c++17', '-pthread', '-I', str(ROOT/'src'),
                            str(source), '-o', str(binary)], check=True, capture_output=True)
            subprocess.run([str(binary), str(directory)], check=True, capture_output=True, cwd=ROOT)
            path = next(directory.glob('right_arm*.csv'))
            with path.open() as file:
                rows = list(csv.DictReader(file))
            stats = dict(line.split('=') for line in Path(str(path)+'.stats').read_text().splitlines())
            self.assertEqual(len(rows) % 7, 0)
            self.assertEqual(len(rows)//7+int(stats['dropped_frames']), 1000)
            self.assertEqual(stats['write_failed'], '0')
            self.assertAlmostEqual(float(rows[6]['pd']), 2.5)
            self.assertAlmostEqual(float(rows[6]['total_estimate']), 2.9)
            report = diagnose.analyze(directory)[path.name]
            self.assertGreater(report['playing']['static_error_over_3deg_samples'], 0)
            self.assertEqual(report['playing']['gravity_ff_clamped_samples'], 0)

    def test_separates_reference_lag_from_motor_error_and_faults(self):
        row = dict(mono=1, cmd_q=0, source_q=.2, q=0, cmd_dq=0, dq=0,
                   gravity_requested=3, ff_limit=2, ff=2, protocol_limit=10,
                   status=1, age_ms=2, dt=.005, kp=25, kd=.2,
                   rotor=40, total_estimate=2, torque=2)
        report = diagnose.summarize([row])
        self.assertEqual(report['tracking_error_max_deg'], 0)
        self.assertGreater(report['source_command_lag_p95_deg'], 10)
        self.assertEqual(report['gravity_ff_clamped_samples'], 1)
        self.assertEqual(report['fault_or_stale_samples'], 0)
        self.assertEqual(report['protocol_overrange_samples'], 0)
        row.update(status=9, ff=11)
        report = diagnose.summarize([row])
        self.assertEqual(report['fault_or_stale_samples'], 1)
        self.assertEqual(report['protocol_overrange_samples'], 1)


if __name__ == '__main__':
    unittest.main()
