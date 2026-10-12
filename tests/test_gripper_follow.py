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
#include <controller/mirror_reference.hpp>
#include <cassert>
#include <limits>
int main() {
    using P=MirrorReference::Pose;
    P lo,hi; lo.fill(-2); hi.fill(2); lo[7]=-.98; hi[7]=.62;
    P left{},right{},speed{},actual_speed{};
    left[7]=right[7]=-.76276; // measured left position from the failing log
    MirrorReference ref(left,lo,hi);
    ref.step(right,speed,left,actual_speed,.005);
    bool limited=false;
    for(int n=0;n<800;++n) {
        right[7]=std::min(.62,-.76276+2.48352*n*.005);
        speed[7]=right[7]<.62 ? 2.48352 : 0;
        auto q=ref.step(right,speed,left,actual_speed,.005);
        assert(std::abs(q[7]-left[7])<=.2+1e-9);
        assert(q[7]>=lo[7] && q[7]<=hi[7]);
        limited=limited || ref.gripper_limited();
    }
    assert(limited && ref.gripper_holding() && ref.tracking());
    assert(std::abs(ref.step(right,speed,left,actual_speed,.005)[7]-left[7]-.05)<1e-9);
    // User opens slightly: release even though right angle is still much larger
    // than the left contact position; continued opening must not re-close the jaw.
    for(int n=0;n<20;++n) {
        right[7]=.58-.002*n; speed[7]=-.4;
        auto q=ref.step(right,speed,left,actual_speed,.005);
        assert(!ref.gripper_holding() && q[7]<left[7]);
    }
    // Opening obstruction is also lead-limited, without being labelled contact.
    MirrorReference opening(left,lo,hi);
    right[7]=-.98; speed[7]=-3;
    for(int n=0;n<800;++n) {
        auto q=opening.step(right,speed,left,actual_speed,.005);
        assert(std::abs(q[7]-left[7])<=.2+1e-9);
        assert(!opening.gripper_holding());
    }
    // A moving, unobstructed jaw should keep following instead of latching hold.
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
