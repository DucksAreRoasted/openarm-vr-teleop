"""Replay rejects invalid motor feedback before the next command."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ReplayFeedbackTests(unittest.TestCase):
    def test_feedback_bounds_and_nonfinite_values(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "feedback.cpp"
            binary = Path(directory) / "feedback"
            source.write_text(r'''
#include <controller/replay_feedback.hpp>
#include <limits>
#include <cassert>
#include <stdexcept>
int main() {
    int rejected = 0;
    auto reject = [&](size_t joint, double q, double dq, double tau) {
        try { validate_replay_feedback("left", joint, q, dq, tau, -1.57, 1.57); }
        catch (const std::runtime_error&) { ++rejected; }
    };
    for (const auto* side : {"left", "right"}) {
        for (size_t joint=0; joint<8; ++joint) {
            const double limit = joint < 6 ? 2 : joint == 6 ? 3 : 10;
            assert(replay_feedback_speed_limit(joint) == limit);
            for (double direction : {-1., 1.}) {
                validate_replay_feedback(side, joint, 0, direction*limit, 0, -1.57, 1.57);
                reject(joint, 0, direction*(limit+.01), 0);
            }
        }
        for (double speed : {-2.63004, -2.04396, 2.04396, 2.63004}) {
            validate_replay_feedback(side, 6, 0, speed, 0, -1.57, 1.57);
        }
    }
    reject(6, 1.66, 0.0, 0.0);
    reject(6, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0);
    reject(6, 0.0, 0.0, std::numeric_limits<double>::infinity());
    reject(6, std::numeric_limits<double>::quiet_NaN(), 0, 0);
    reject(8, 0, 0, 0);
    return rejected == 37 ? 0 : 1;
}
''')
            subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"),
                            str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
