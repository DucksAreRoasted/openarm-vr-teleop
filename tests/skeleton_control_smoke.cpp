// Offline V1 simulation regression: never opens CAN or a network socket.
#include <controller/skeleton_proximal_controller.hpp>
#include <hardware/mujoco_teleop_hardware.hpp>
#include <openarm_constants.hpp>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

double distance(const std::vector<JointState>& a, const std::vector<JointState>& b,
                size_t begin = 0, size_t end = 8) {
    double result = 0;
    for (size_t i = begin; i < end; ++i) result = std::max(result, std::abs(a[i].position-b[i].position));
    return result;
}

void check_arm(const std::string& urdf, const std::string& side,
               const std::shared_ptr<MujocoSimWorld>& world) {
    Dynamics dynamics(urdf, "openarm_body_link0", "openarm_"+side+"_link7");
    require(dynamics.Init(), "dynamics init");
    SkeletonProximalConfig cfg;
    cfg.urdf_path = urdf;
    cfg.shoulder_link = "openarm_"+side+"_link1";
    cfg.elbow_link = "openarm_"+side+"_link4";
    cfg.wrist_link = "openarm_"+side+"_link7";
    cfg.debug_log = false;
    cfg.q_nominal.assign(TABLE_READY_POSITION.begin(), TABLE_READY_POSITION.begin()+7);
    SkeletonProximalController control(&dynamics, cfg);
    require(control.Init(), "controller init");
    MujocoTeleopHardware sim(world, side);
    auto home = cfg.q_nominal;
    home[0] += .05; // First sample must preserve measured pose, not the nominal.
    sim.set_home(home, .02);
    SkeletonPoseArm body;
    body.u_upper = Eigen::Vector3d(.3,.9,.2).normalized();
    body.u_fore = Eigen::Vector3d(.7,.1,.6).normalized();
    body.L1 = .3; body.L2 = .25; body.conf = 1;
    QuestPoseSample sample;
    sample.valid = true;
    std::vector<JointState> current(8), ref;
    std::vector<double> lo, hi;
    require(dynamics.GetJointPositionLimits(&lo,&hi), "URDF limits");
    auto tick = [&](bool fresh = true, bool valid_body = true, double age = 0., double controller_age_sec = 0.) {
        auto arm = sim.read_arm(); auto grip = sim.read_gripper();
        for (size_t i=0;i<7;++i) current[i].position = arm[i].position;
        current[7].position = grip[0].position;
        sample.receive_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count() - (fresh ? static_cast<int64_t>(controller_age_sec*1e9) : 1000000000LL);
        control.set_skeleton_pose(body, valid_body, age);
        require(control.step(current, sample, .005, &ref), "control step");
        require(ref.size()==8, "reference size");
        std::vector<openarm::damiao_motor::MITParam> cmds(7);
        for (size_t i=0;i<7;++i) {
            require(std::isfinite(ref[i].position), "finite reference");
            require(ref[i].position >= lo[i]-1e-8 && ref[i].position <= hi[i]+1e-8, "URDF limits respected");
            cmds[i].q = ref[i].position; cmds[i].kp = 1.;
        }
        sim.send_arm(cmds);
        std::vector<openarm::damiao_motor::MITParam> gripper(1);
        gripper[0].q = ref[7].position;
        sim.send_gripper(gripper); sim.recv_all(0);
        return ref;
    };
    sample.valid = false;
    const auto initial = tick();
    sample.valid = true;
    require(distance(initial,current)<1e-12, "startup must hold measured pose including gripper");
    sample.grip_active = true;
    for (int i=0;i<60;++i) tick();
    require(control.clutch_active(), "grip activates");
    require(distance(initial,ref)<1e-5, "stationary clutch must not jump");
    body.u_upper = Eigen::Vector3d(.6,.6,.5).normalized();
    body.u_fore = Eigen::Vector3d(-.2,.3,.9).normalized();
    for (int i=0;i<120;++i) tick();
    require(distance(initial,ref,0,4)>.005, "skeleton moves proximal joints");
    sample.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.25,Eigen::Vector3d::UnitY()));
    const auto before_wrist = ref;
    for (int i=0;i<80;++i) tick();
    require(distance(before_wrist,ref,4,7)>.001, "orientation moves wrist");
    sample.grip_active = false;
    const auto released = tick();
    for (int i=0;i<30;++i) tick();
    require(distance(released,ref)<1e-12, "release stops outstanding target motion");
    sample.grip_active = true; tick();
    const auto brief_loss = tick(true,false);
    require(control.clutch_active(), "brief body loss keeps clutch while holding");
    body.u_upper = Eigen::Vector3d(.5,.6,.6).normalized();
    sample.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitY()));
    require(distance(brief_loss,tick())<1e-5, "brief body recovery reanchors without jump");
    require(control.clutch_active(), "brief body loss automatically recovers");
    sample.trigger=1.; sample.button_close=true;
    const auto brief_controller = tick(true,true,0.,.1);
    for (int i=0;i<10;++i) tick(true,true,0.,.1);
    require(distance(brief_controller,ref)<1e-12, "soft controller gap freezes including buttons");
    sample.trigger=0.; sample.button_close=false;
    sample.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.55,Eigen::Vector3d::UnitY()));
    require(distance(brief_controller,tick())<1e-5, "soft controller recovery reanchors without jump");
    require(control.clutch_active(), "brief controller loss automatically recovers");
    const auto lost = tick(true,false);
    for (int i=0;i<30;++i) tick(true,false);
    for (int i=0;i<20;++i) tick();
    require(!control.clutch_active(), "body recovery must require regrip");
    require(distance(lost,ref)<1e-12, "body loss freezes arm and wrist");
    sample.grip_active = false; tick(); sample.grip_active = true; tick();
    require(control.clutch_active(), "regrip resumes");
    const auto stale_body = tick(true,true,251.);
    require(!control.clutch_active(), "old skeleton stops clutch");
    require(distance(stale_body,tick())<1e-12, "fresh skeleton cannot auto resume");
    sample.trigger=1.; sample.button_close=true;
    const auto timed_out = tick(false);
    for (int i=0;i<30;++i) tick(false);
    require(distance(timed_out,ref)<1e-12, "stale trigger and gripper buttons must freeze");
    sample.valid=false; tick();
    require(distance(timed_out,ref)<1e-12, "invalid controller must freeze");
    sample.valid=true; sample.trigger=0.; sample.button_close=false;
    tick();
    require(!control.clutch_active(), "controller reconnect must require regrip");
    sample.grip_active=false; tick(); sample.grip_active=true; tick();
    require(control.clutch_active(), "controller recovery regrip");
    body.conf=0.;
    for (int i=0;i<30;++i) tick();
    require(!control.clutch_active(), "sustained zero arm confidence must stop clutch");
    body.conf=1.; tick();
    require(!control.clutch_active(), "confidence recovery cannot auto resume");
    std::cout << "PASS " << side << ": startup, stationary grip, proximal/wrist movement, limits, release, loss, recovery, brief-gap hold/reanchor, timeout buttons\n";
}

int main(int argc, char** argv) {
    try {
        require(argc==3, "usage: skeleton_control_smoke URDF SCENE_XML");
        setenv("OPENARM_MUJOCO_HEADLESS","1",1);
        auto world = std::make_shared<MujocoSimWorld>(argv[2],"skeleton regression");
        check_arm(argv[1],"left",world);
        check_arm(argv[1],"right",world);
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n'; return 1;
    }
}
