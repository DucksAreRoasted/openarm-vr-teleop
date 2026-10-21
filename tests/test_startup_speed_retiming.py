"""Replay the actual startup wrist burst with a simplified inertial follower.

This catches the logged overspeed mechanism; it is not physical validation.
"""
from pathlib import Path
import hashlib
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class StartupSpeedRetimingTests(unittest.TestCase):
    def test_actual_startup_burst_has_tracking_headroom(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'retiming.cpp'
            binary = Path(directory) / 'retiming'
            source.write_text(r'''
#include <controller/startup_trajectory.hpp>
#include <cassert>
#include <iostream>

double peak(CsvReplay& replay, double inertia) {
    double q=replay.sample_at(0)->at(14).position, dq=0, maximum=0;
    double target=q, velocity=0;
    // 200 Hz MIT commands, 2 kHz inertial integration. Gravity is cancelled.
    for (int n=0; n*.0005<replay.duration(); ++n) {
        if (n%10==0) {
            const auto sample=replay.sample_at(n*.0005);
            target=sample->at(14).position;
            velocity=sample->at(14).velocity;
        }
        const double torque=std::clamp(25*(target-q)+.2*(velocity-dq),-10.,10.);
        dq+=(torque-.02*dq)/inertia*.0005;
        q+=dq*.0005;
        maximum=std::max(maximum,std::abs(dq));
    }
    return maximum;
}
int main(int argc, char** argv) {
    assert(argc==2);
    CsvReplay original, retimed;
    assert(original.load(argv[1],16) && retimed.load(argv[1],16));
    const auto first=original.sample_at(0)->at(14).position;
    const auto last=original.sample_at(original.duration())->at(14).position;
    std::vector<double> limits(16);
    for (size_t i=0;i<16;++i) limits[i]=.65*replay_feedback_speed_limit(i%8);
    retimed.retime_speed_limits(limits);
    assert(retimed.duration()>original.duration());
    assert(retimed.duration()<original.duration()+1); // local slowing, not half speed
    assert(retimed.sample_at(0)->at(14).position==first);
    assert(retimed.sample_at(retimed.duration())->at(14).position==last);
    for (double t=0;t<retimed.duration();t+=.001) {
        auto sample=retimed.sample_at(t);
        for (size_t j=0;j<16;++j) assert(std::abs(sample->at(j).velocity)<=limits[j]+1e-9);
    }
    assert(replay_feedback_speed_limit(6)==3); // hard stop is retained
    // The startup file is the left recording mirrored onto the right, so the
    // old right-wrist burst above 3 rad/s is gone.
    assert(peak(original,.01)<3);
    for (double inertia : {.003,.006,.01,.03}) {
        const double before=peak(original,inertia), after=peak(retimed,inertia);
        std::cout << "inertia=" << inertia << " original_peak=" << before
                  << " retimed_peak=" << after << std::endl;
        assert(after<3);
        assert(after<=before+1e-9);
    }
    // Nonpositive limits are rejected rather than disabling protection.
    bool rejected=false;
    limits[6]=0;
    try { retimed.retime_speed_limits(limits); }
    catch (const std::runtime_error&) { rejected=true; }
    assert(rejected);
}
''')
            startup = ROOT / 'assets/steady_action_on_table.csv'
            digest = hashlib.sha256(startup.read_bytes()).digest()
            subprocess.run(['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-I', str(ROOT/'src'),
                            str(source), str(ROOT/'src/io/teleop_io.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary), str(startup)], check=True)
            self.assertEqual(hashlib.sha256(startup.read_bytes()).digest(), digest)

    def test_startup_right_arm_mirrors_left(self):
        # j3 flips with the shoulders and wrist. Leaving it unchanged swings
        # both forearms to the same side.
        sign = [-1, -1, -1, 1, -1, -1, -1, 1]
        lines = (ROOT / "assets/steady_action_on_table.csv").read_text().splitlines()
        self.assertTrue(lines[0].startswith("time,"))
        for line in lines[1:]:
            cells = line.split(",")
            for joint, factor in enumerate(sign):
                for offset in range(3):
                    left = float(cells[1 + joint * 3 + offset])
                    right = float(cells[1 + (8 + joint) * 3 + offset])
                    self.assertAlmostEqual(right, factor * left, places=6)
