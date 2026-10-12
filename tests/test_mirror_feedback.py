"""Regression for hand-driven master speed triggering the follower's protection."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class MirrorFeedbackTests(unittest.TestCase):
    def test_master_input_and_follower_protection(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "feedback.cpp"
            binary = Path(directory) / "feedback"
            source.write_text(r'''
#include <controller/mirror_feedback.hpp>
#include <controller/mirror_reference.hpp>
#include <limits>
int main() {
    // Replay the exact right-wrist sample which stopped the real session.
    validate_mirror_feedback("right", true, 6, -.0181201, -2.16117, 1.2674, -1.5708, 1.5708);
    int rejected=0;
    auto reject=[&](bool master, double q, double dq, double tau) {
        try { validate_mirror_feedback(master ? "right" : "left", master, 6,
                                       q, dq, tau, -1.5708, 1.5708); }
        catch(const std::runtime_error& error) {
            if(std::string(error.what()).find("speed_limit=")==std::string::npos) return;
            ++rejected;
        }
    };
    reject(false, -.0181201, -2.16117, 1.2674); // powered follower still stops
    reject(true, 0, 4.01, 0);
    reject(true, 0, -4.01, 0);
    reject(true, 1.7, 0, 0);
    reject(true, 0, std::numeric_limits<double>::quiet_NaN(), 0);
    reject(true, 0, 0, std::numeric_limits<double>::infinity());
    if(rejected!=6) return 1;
    // Fast master input must not bypass command slew limits.
    MirrorReference::Pose lo,hi,master{},speed{};
    lo.fill(-2); hi.fill(2);
    MirrorReference reference({},lo,hi);
    reference.step({}, {}, .005);
    double previous_velocity=0;
    for(int n=0;n<100;++n) {
        master[6]=-2.16117*n*.005; speed[6]=-2.16117;
        reference.step(master,speed,.005);
        double v=reference.velocity()[6];
        if(std::abs(v)>1.5+1e-9 || std::abs(v-previous_velocity)>.03+1e-9) return 2;
        previous_velocity=v;
    }
}
''')
            subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"),
                            str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
