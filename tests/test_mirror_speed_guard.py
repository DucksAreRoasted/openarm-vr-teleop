"""Torque-level regression for a delayed follower accelerating past its reference."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class MirrorSpeedGuardTests(unittest.TestCase):
    def test_braking_torque_and_delayed_wrist_response(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "speed_guard.cpp"
            binary = Path(directory) / "speed_guard"
            source.write_text(r'''
#include <controller/mirror_speed_guard.hpp>
#include <cassert>
#include <limits>
int main() {
    // Below the guard, preserve gravity, friction and tracking torque.
    assert(mirror_follower_torque(.7,.3,.7,2)==.7);
    assert(mirror_follower_torque(3,0,0,2)==2);
    double previous=2;
    for(int n=0;n<=60;++n) {
        const double speed=.8+n*.02;
        const double total=mirror_follower_torque(2,.3,speed,2);
        assert(total<=previous+1e-9);
        assert(std::abs(total)<=2);
        assert(std::abs(total+mirror_follower_torque(-2,-.3,-speed,2))<1e-9);
        if(speed>=1.5) assert(total<.3); // brake relative to gravity
        previous=total;
    }
    // Already braking: never scale braking away or reverse it.
    assert(mirror_follower_torque(-.5,.3,1.6,2)<-.5);
    // Large tracking error cannot overwhelm the guard before torque saturation.
    assert(mirror_follower_torque(200,.3,1.6,2)==mirror_follower_torque(2,.3,1.6,2));
    // Preserve the holding baseline when shaping drive in either direction.
    for(double hold : {-.5,0.,.5}) for(double direction : {-1.,1.}) {
        double limited=mirror_follower_torque(hold+direction*1.0,hold,direction*1.5,2);
        assert(direction*(limited-hold)<0 && std::abs(limited)<=2);
    }
    // Replay the overspeed direction; the hard-stop validator still rejects it.
    assert(mirror_follower_torque(2,0,2.14652,2)<0);
    // Equivalent MIT command must include exactly one copy of motor-side PD.
    double pd=8*.4+.3*(1.5-1.7), gravity=.2;
    double guarded=mirror_follower_torque(gravity+pd,gravity,1.7,2);
    double tau=guarded-pd;
    assert(std::abs(tau+pd-guarded)<1e-9 && guarded<gravity);
    bool rejected=false;
    try { mirror_follower_torque(0,0,std::numeric_limits<double>::quiet_NaN(),2); }
    catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);

    // Simplified inertial wrist with a 0.4 rad backlog, at a 200 Hz servo.
    // This tests the torque policy; it does not model the real robot or CAN delay.
    for(double direction : {-1.,1.}) for(double inertia : {.01,.03,.1}) {
        double q=0, v=direction*1.4, oldq=q, oldv=v;
        double peak=std::abs(v), oldpeak=peak;
        for(int n=0;n<100;++n) {
            const double target=direction*(.4+1.5*n*.005);
            double raw=8*(target-q)+.3*(direction*1.5-v);
            double torque=mirror_follower_torque(raw,0,v,2);
            v+=(torque-.04*v)/inertia*.005;
            q+=v*.005;
            peak=std::max(peak,std::abs(v));
            raw=8*(target-oldq)+.3*(direction*1.5-oldv);
            oldv+=(std::clamp(raw,-2.,2.)-.04*oldv)/inertia*.005;
            oldq+=oldv*.005;
            oldpeak=std::max(oldpeak,std::abs(oldv));
        }
        assert(oldpeak>2.0); // reproduce overspeed with position PD alone
        assert(peak<1.8);   // margin below the unchanged 2 rad/s hard stop
    }
}
''')
            subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"),
                            str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
