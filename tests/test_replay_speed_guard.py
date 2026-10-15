"""Torque-level regression for a delayed follower accelerating past its reference."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ReplaySpeedGuardTests(unittest.TestCase):
    def test_braking_torque_and_delayed_wrist_response(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "speed_guard.cpp"
            binary = Path(directory) / "speed_guard"
            source.write_text(r'''
#include <controller/replay_speed_guard.hpp>
#include <cassert>
#include <limits>
int main() {
    // Below the guard, preserve gravity, friction and tracking torque.
    assert(replay_follower_torque(.7,.3,.7,2)==.7);
    assert(replay_follower_torque(3,0,0,2)==2);
    // Historical wrist policy admits additional load torque but still clamps
    // both directions and preserves braking and realizable MIT feedforward.
    const double wrist_limit=kReplayLegacyTotalEffortLimits[6];
    for(double direction : {-1.,1.}) {
        assert(replay_follower_torque(direction*2.5,.4*direction,0,wrist_limit)==direction*2.5);
        assert(replay_follower_torque(direction*100,.4*direction,0,wrist_limit)==direction*wrist_limit);
        assert(direction*replay_follower_torque(direction*100,0,direction*1.7,wrist_limit)<0);
        for(double error : {-.45,0.,.45}) {
            double pd=8*error+.3*(.6-direction*1.7);
            double total=replay_follower_torque(.5+pd,.5,direction*1.7,wrist_limit);
            double ff=total-pd;
            assert(std::abs(total)<=wrist_limit);
            assert(std::abs(ff)<10); // wrist MIT protocol FF range
            assert(std::abs(ff+pd-total)<1e-9);
        }
    }
    double previous=2;
    for(int n=0;n<=60;++n) {
        const double speed=.8+n*.02;
        const double total=replay_follower_torque(2,.3,speed,2);
        assert(total<=previous+1e-9);
        assert(std::abs(total)<=2);
        assert(std::abs(total+replay_follower_torque(-2,-.3,-speed,2))<1e-9);
        if(speed>=1.5) assert(total<.3); // brake relative to gravity
        previous=total;
    }
    // Already braking: never scale braking away or reverse it.
    assert(replay_follower_torque(-.5,.3,1.6,2)<-.5);
    // Large tracking error cannot overwhelm the guard before torque saturation.
    assert(replay_follower_torque(200,.3,1.6,2)==replay_follower_torque(2,.3,1.6,2));
    // Preserve the holding baseline when shaping drive in either direction.
    for(double hold : {-.5,0.,.5}) for(double direction : {-1.,1.}) {
        double limited=replay_follower_torque(hold+direction*1.0,hold,direction*1.5,2);
        assert(direction*(limited-hold)<0 && std::abs(limited)<=2);
    }
    // Replay the overspeed direction; the hard-stop validator still rejects it.
    assert(replay_follower_torque(2,0,2.14652,2)<0);
    // Equivalent MIT command must include exactly one copy of motor-side PD.
    double pd=8*.4+.3*(1.5-1.7), gravity=.2;
    double guarded=replay_follower_torque(gravity+pd,gravity,1.7,2);
    double tau=guarded-pd;
    assert(std::abs(tau+pd-guarded)<1e-9 && guarded<gravity);
    bool rejected=false;
    try { replay_follower_torque(0,0,std::numeric_limits<double>::quiet_NaN(),2); }
    catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);

    // Simplified inertial wrist with a 0.4 rad backlog, at a 200 Hz servo.
    // This tests the torque policy; it does not model the real robot or CAN delay.
    for(double limit : {2.,wrist_limit})
    for(double direction : {-1.,1.}) for(double inertia : {.01,.03,.1}) {
        double q=0, v=direction*1.4, oldq=q, oldv=v;
        double peak=std::abs(v), oldpeak=peak;
        for(int n=0;n<100;++n) {
            const double target=direction*(.4+1.5*n*.005);
            double raw=8*(target-q)+.3*(direction*1.5-v);
            double torque=replay_follower_torque(raw,0,v,limit);
            v+=(torque-.04*v)/inertia*.005;
            q+=v*.005;
            peak=std::max(peak,std::abs(v));
            raw=8*(target-oldq)+.3*(direction*1.5-oldv);
            oldv+=(std::clamp(raw,-limit,limit)-.04*oldv)/inertia*.005;
            oldq+=oldv*.005;
            oldpeak=std::max(oldpeak,std::abs(oldv));
        }
        assert(oldpeak>2.0); // reproduce overspeed with position PD alone
        if(limit==2.) assert(peak<1.8); // preserve the previous-limit regression
        // Higher available torque changes the transient. Retain at least
        // 0.1 rad/s margin to the unchanged 2 rad/s stop in this stress model.
        assert(peak<kReplayFollowerSpeedStop-.1);
    }
}
''')
            subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"),
                            str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
