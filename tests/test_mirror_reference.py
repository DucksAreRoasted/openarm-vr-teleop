"""Offline checks for OpenArm same-direction mapping, slew limits and rejection paths."""
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


class MirrorReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        source = Path(cls.temp.name) / "test.cpp"
        binary = Path(cls.temp.name) / "test"
        source.write_text(r'''
#include <controller/mirror_reference.hpp>
#include <iostream>
#include <limits>
int main() {
    using P = MirrorReference::Pose;
    P lo, hi; lo.fill(-1); hi.fill(1);
    MirrorReference ref({}, lo, hi);
    P right{.4,.3,.2,.5,.2,.1,.3,.4};
    for (double s : MirrorReference::signs) std::cout << s << " ";
    std::cout << "\n";
    for (int n=0;n<1500;++n) {
        if(n==700) for (auto& q:right) q=-q;
        const auto q=ref.step(right,.01);
        for (double v:q) std::cout<<v<<" ";
        for (double v:ref.velocity()) std::cout<<v<<" ";
        std::cout<<"\n";
    }
    int rejected=0;
    try { ref.step(right,.2); } catch (const std::runtime_error&) { ++rejected; }
    right[2]=std::numeric_limits<double>::quiet_NaN();
    try { ref.step(right,.01); } catch (const std::runtime_error&) { ++rejected; }
    right[2]=std::numeric_limits<double>::infinity();
    try { ref.step(right,.01); } catch (const std::runtime_error&) { ++rejected; }

    // Smooth trajectory has nonzero velocity and acceleration, starting aligned.
    MirrorReference follow({},lo,hi);
    constexpr double pi=3.141592653589793;
    double qerr=0, verr=0, aerr=0, oldv=0;
    for (int n=0;n<2000;++n) {
        const double t=n*.005, omega=pi;
        const double pos=.2*(1-std::cos(omega*t));
        const double vel=.2*omega*std::sin(omega*t);
        const double acc=.2*omega*omega*std::cos(omega*t);
        P master{}, speed{};
        master.fill(pos); speed.fill(vel);
        const auto q=follow.step(master,speed,.005);
        const double v=follow.velocity()[0];
        if(n>200) {
            qerr=std::max(qerr,std::abs(q[0]+pos));
            verr=std::max(verr,std::abs(v+vel));
            aerr=std::max(aerr,std::abs((v-oldv)/.005+acc));
        }
        oldv=v;
    }
    std::cerr<<qerr<<" "<<verr<<" "<<aerr<<"\n";
    P bad_speed{};bad_speed[1]=std::numeric_limits<double>::quiet_NaN();
    try { follow.step({},bad_speed,.005); } catch (const std::runtime_error&) { ++rejected; }
    // Master position beyond the follower stroke is clipped; master hardware
    // feedback validation separately checks the master own joint limits.
    MirrorReference grip({},lo,hi);
    P hand{}; hand[7]=1.05;
    for(int n=0;n<2000;++n) grip.step(hand,.005);
    if(std::abs(grip.step(hand,.005)[7]-1.0)>1e-4) return 2;
    hand[7]=-1.05;
    for(int n=0;n<2000;++n) grip.step(hand,.005);
    if(std::abs(grip.step(hand,.005)[7]+1.0)>1e-4) return 3;
    return rejected==4 ? 0 : 1;
}
''')
        subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"), str(source), "-o", str(binary)], check=True)
        result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
        lines = result.stdout.splitlines()
        cls.tracking_errors = np.fromstring(result.stderr, sep=" ")
        cls.signs = np.fromstring(lines[0], sep=" ")
        cls.samples = np.array([np.fromstring(line, sep=" ") for line in lines[1:]])

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_rate_acceleration_and_reversal(self):
        q, v = self.samples[:, :8], self.samples[:, 8:]
        vmax = np.array([1.5]*7 + [2.0])
        amax = np.array([6.0]*7 + [8.0])
        self.assertTrue(np.all(np.abs(v) <= vmax + 1e-6))
        self.assertTrue(np.all(np.abs(np.diff(v, axis=0))/.01 <= amax + 1e-3))
        self.assertTrue(np.all(np.abs(np.diff(q, axis=0))/.01 <= vmax + 1e-3))
        self.assertTrue(np.all(np.abs(q[0]) < .0002))
        goal = -np.array([.4,.3,.2,.5,.2,.1,.3,.4])*self.signs
        np.testing.assert_allclose(q[-1], goal, atol=1e-4)

    def test_velocity_and_acceleration_tracking(self):
        position, velocity, acceleration = self.tracking_errors
        self.assertLess(position, .004)  # < 0.25 degrees at 0.63 rad/s master speed
        self.assertLess(velocity, .015)
        self.assertLess(acceleration, .05)

    def test_urdf_fore_aft_direction(self):
        joints = {j.attrib["name"]: j for j in ET.parse(ROOT / "assets/v1.urdf").getroot().findall("joint")}

        def tcp(side, shoulder, elbow):
            transform = np.eye(4)
            names = [f"openarm_{side}_openarm_body_link0_joint"] + [f"openarm_{side}_joint{i}" for i in range(1, 8)]
            for index, name in enumerate(names):
                joint = joints[name]
                origin = joint.find("origin")
                step = np.eye(4)
                step[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
                step[:3, 3] = np.fromstring(origin.get("xyz"), sep=" ")
                transform = transform @ step
                if index:
                    angle = shoulder if index == 1 else elbow if index == 4 else 0
                    axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                    step = np.eye(4)
                    step[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
                    transform = transform @ step
            return transform[:3, 3]

        # Body X is fore/aft. Positive/negative j1 movements must agree in X,
        # across different elbow bends, despite opposite left/right j1 axes.
        for elbow in (.4, 1.2, 2.0):
            for shoulder in (-.2, .1, .3):
                right_delta = tcp("right", shoulder, elbow) - tcp("right", 0, elbow)
                left_delta = tcp("left", self.signs[0] * shoulder, elbow) - tcp("left", 0, elbow)
                self.assertGreater(right_delta[0] * left_delta[0], 0)
                self.assertAlmostEqual(right_delta[0], left_delta[0], places=6)

    def test_same_direction_with_asymmetric_urdf_limits(self):
        np.testing.assert_array_equal(self.signs, [-1,1,1,1,1,1,-1,1])
        joints = {j.attrib["name"]: j for j in ET.parse(ROOT / "assets/v1.urdf").getroot().findall("joint")}
        def limits(side, attribute):
            return [float(joints[f"openarm_{side}_joint{i}"].find("limit").get(attribute))
                    for i in range(1, 8)] + ([-.98] if attribute == "lower" else [.62])
        lo, hi = limits("left", "lower"), limits("left", "upper")
        rlo, rhi = limits("right", "lower"), limits("right", "upper")
        def cpp(values):
            return "{" + ",".join(str(v) for v in values) + "}"
        source = Path(self.temp.name) / "limits.cpp"
        binary = Path(self.temp.name) / "limits"
        source.write_text(r'''
#include <controller/mirror_reference.hpp>
#include <cassert>
int main() {
    using P=MirrorReference::Pose;
    P lo=LEFT_LO, hi=LEFT_HI, rlo=RIGHT_LO, rhi=RIGHT_HI;
    MirrorReference ref({},lo,hi);
    for(int n=0;n<101;++n) {
        P right{};
        for(int i=0;i<8;++i) right[i]=rlo[i]+(rhi[i]-rlo[i])*n/100.;
        auto goal=ref.target_pose(right);
        for(int i=0;i<8;++i) assert(goal[i]==std::clamp(MirrorReference::signs[i]*right[i],lo[i],hi[i]));
    }
    // The right shoulder has a large positive stroke absent on the left.
    // It must stay at the left limit, not throw or feed forward at the boundary.
    P home{}; home[1]=hi[1];
    MirrorReference shoulder(home,lo,hi);
    shoulder.step(home,{},.005);
    P right=home, speed{}; right[1]=hi[1]+.5; speed[1]=1;
    for(int n=0;n<200;++n) {
        auto q=shoulder.step(right,speed,.005);
        assert(q[1]==hi[1] && shoulder.velocity()[1]==0);
    }
    right[1]=hi[1]; speed[1]=1;
    shoulder.step(right,speed,.005);
    assert(shoulder.velocity()[1]==0);
    right[1]=hi[1]-.05; speed[1]=-.2;
    auto q=shoulder.step(right,speed,.005);
    assert(q[1]<hi[1] && shoulder.velocity()[1]<0);
}
'''.replace("LEFT_LO",cpp(lo)).replace("LEFT_HI",cpp(hi))
            .replace("RIGHT_LO",cpp(rlo)).replace("RIGHT_HI",cpp(rhi)))
        subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"), str(source), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)

    def test_all_joint_axes_and_gripper_pitch_in_body_frame(self):
        joints = {j.attrib["name"]: j for j in ET.parse(ROOT / "assets/v1.urdf").getroot().findall("joint")}

        def fk(side, q):
            transform = np.eye(4)
            axes = []
            names = [f"openarm_{side}_openarm_body_link0_joint"] + [f"openarm_{side}_joint{i}" for i in range(1, 8)]
            for index, name in enumerate(names):
                joint = joints[name]
                origin = joint.find("origin")
                step = np.eye(4)
                step[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
                step[:3, 3] = np.fromstring(origin.get("xyz"), sep=" ")
                transform = transform @ step
                if index:
                    axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                    axes.append(transform[:3, :3] @ axis)
                    step = np.eye(4)
                    step[:3, :3] = Rotation.from_rotvec(axis*q[index-1]).as_matrix()
                    transform = transform @ step
            # A point along the gripper centre line exposes wrist pitch movement;
            # the joint7 origin alone does not move when rotating joint7.
            tip = transform[:3, 3] + .12*transform[:3, 2]
            return tip, transform[:3, :3], np.array(axes)

        signs = self.signs[:7]
        right_limits, mapped_left_limits = [], []
        for side, output in (("right", right_limits), ("left", mapped_left_limits)):
            for i in range(7):
                limit = joints[f"openarm_{side}_joint{i+1}"].find("limit")
                pair = np.array([float(limit.get("lower")), float(limit.get("upper"))])
                output.append(np.sort(pair*(signs[i] if side == "left" else 1)))
        lower = np.maximum(np.array(right_limits)[:, 0], np.array(mapped_left_limits)[:, 0])
        upper = np.minimum(np.array(right_limits)[:, 1], np.array(mapped_left_limits)[:, 1])
        home = (lower+upper)/2
        l0, rl0, _ = fk("left", signs*home)
        r0, rr0, _ = fk("right", home)
        offset = l0-r0
        rng = np.random.default_rng(72)
        for right in rng.uniform(lower, upper, (100, 7)):
            left_tip, left_rotation, left_axes = fk("left", signs*right)
            right_tip, right_rotation, right_axes = fk("right", right)
            # URDF includes rounded +/-1.5708 base rotations (about 7e-6 rad).
            np.testing.assert_allclose(left_axes*signs[:, None], right_axes, atol=2e-5)
            np.testing.assert_allclose(left_tip-right_tip, offset, atol=2e-5)
            np.testing.assert_allclose(left_rotation@rl0.T, right_rotation@rr0.T, atol=2e-5)

        # Both grippers must pitch up/down in Z together. The old j7 same-sign
        # mapping must fail this assertion, while j8 opening stays same-sign.
        right = np.zeros(7)
        right[3] = 1.2
        for pitch in (-.2, .2):
            tilted = right.copy()
            tilted[6] = pitch
            dz_right = fk("right", tilted)[0][2]-fk("right", right)[0][2]
            dz_left = fk("left", signs*tilted)[0][2]-fk("left", signs*right)[0][2]
            old = signs*tilted
            old[6] = pitch
            dz_old = fk("left", old)[0][2]-fk("left", signs*right)[0][2]
            self.assertGreater(dz_right*dz_left, 0)
            self.assertLess(dz_right*dz_old, 0)
            self.assertAlmostEqual(dz_right, dz_left, places=6)
        self.assertEqual(self.signs[7], 1)


if __name__ == "__main__":
    unittest.main()
