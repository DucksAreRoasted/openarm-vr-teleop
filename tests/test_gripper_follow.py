"""Fast hand input, blocked jaws, contact release, and recorded command bounds."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class GripperFollowTests(unittest.TestCase):
    def test_fast_input_contact_and_release(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "gripper.cpp"
            binary = Path(directory) / "gripper"
            source.write_text(r'''
#include <controller/gripper_follow.hpp>
#include <cassert>
#include <limits>
int main() {
    GripperFollow free;
    double measured=-.7;
    for(int n=0;n<100;++n) {
        measured+=.005;
        auto command=free.step(measured+.05, measured+.04, 1, measured, 1, -.98,.62,.005);
        assert(!command.holding && !command.limited && command.velocity==1);
    }
    // Simulated units have a much smaller stroke: never exceed those limits.
    GripperFollow sim;
    auto command=sim.step(.044,.5,2,0,0,0,.044,.005);
    assert(command.position==.044 && command.velocity==0);
    bool rejected=false;
    try { free.step(0,0,0,std::numeric_limits<double>::quiet_NaN(),0,-.98,.62,.005); }
    catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);
}
''')
            subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"),
                            str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
