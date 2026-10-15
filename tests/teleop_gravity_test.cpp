// Runs the production MIT packing path against an in-memory backend, never CAN.
#include <controller/control.hpp>
#include <controller/replay_arm.hpp>
#include <common.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using Mit = openarm::damiao_motor::MITParam;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

class MemoryHardware : public TeleopHardware {
public:
    std::vector<MotorState> arm = std::vector<MotorState>(7);
    std::vector<Mit> command, gripper;
    int sends = 0;
    bool disabled = false;
    std::vector<MotorState> read_arm() override { return arm; }
    std::vector<MotorState> read_gripper() override { return {{0, 0, 0}}; }
    void send_arm(const std::vector<Mit>& value) override { command = value; ++sends; }
    void send_gripper(const std::vector<Mit>& value) override { gripper = value; }
    void recv_all(int) override {}
    void disable_all() override { disabled = true; }
    size_t arm_motor_count() const override { return 7; }
    size_t gripper_motor_count() const override { return 1; }
};

void test_side(const std::string& urdf, const std::string& side) {
    auto dynamics = std::make_shared<Dynamics>(urdf, "openarm_body_link0",
                                               "openarm_" + side + "_link7");
    require(dynamics->Init(), "dynamics init");
    auto hardware = std::make_shared<MemoryHardware>();
    hardware->arm[1].position = 0.4;
    hardware->arm[3].position = 1.2;
    auto state = std::make_shared<RobotSystemState>(7, 1);
    Control control(hardware, dynamics.get(), dynamics.get(), state, .001,
                    ROLE_FOLLOWER, side + "_arm", 7, 1);
    const std::vector<double> kp(8, 30), kd(8, 1), zero(8, 0);
    control.SetParameter(kp, kd, zero, zero, zero, zero);
    require(control.SeedHoldMeasured(), "initial measured hold failed");
    require(control.unilateral_step(), "uncompensated step failed");
    for (const auto& frame : hardware->command) require(frame.tau == 0, "FF enabled by default in Control");

    GravityTune tune;
    tune.scale.fill(1.1);
    tune.bias.fill(.15);
    // Friction is deliberately large: teleop must apply gravity only.
    tune.fc.fill(100);
    tune.fv.fill(100);
    control.SetFollowerGravity(dynamics, tune, .95, 1.0);
    require(control.unilateral_step(), "ramp first frame failed");
    const auto first = hardware->command;
    for (int step = 1; step < 1000; ++step) {
        require(control.unilateral_step(), "gravity hold failed");
    }
    std::array<double, 7> q{}, gravity{};
    for (size_t i = 0; i < 7; ++i) q[i] = hardware->arm[i].position;
    dynamics->GetGravity(q.data(), gravity.data());
    const auto effort = control.GetLastCommandEfforts();
    const auto frame = control.GetLastMitCommand();
    bool nonzero = false;
    for (size_t i = 0; i < 7; ++i) {
        const double expected = std::clamp(.95 * (1.1 * gravity[i] + .15),
                                          -kTauFfLimits[i], kTauFfLimits[i]);
        require(std::abs(hardware->command[i].tau - expected) < 1e-9, "wrong measured-q gravity");
        require(std::abs(first[i].tau) <= std::abs(expected) * .000004, "startup torque jumped");
        require(effort[i] == hardware->command[i].tau && frame[i].effort == effort[i],
                "telemetry does not match MIT torque");
        nonzero = nonzero || std::abs(expected) > .1;
    }
    require(nonzero, "test pose has no gravity");
    require(hardware->gripper[0].tau == 0 && effort[7] == 0, "gravity applied to gripper");
    require(control.SeedHoldMeasured(), "reseed hold failed");
    for (size_t i = 0; i < 7; ++i) {
        require(std::abs(hardware->command[i].tau - effort[i]) < 1e-9, "reseed reset support ramp");
    }

    // Moving the reference must not evaluate G at the commanded pose.
    auto reference = state->get_all_references();
    for (size_t i = 0; i < 7; ++i) {
        reference[i].position += .1;
        reference[i].velocity = .2;
        reference[i].effort = 123;
        hardware->arm[i].velocity = .3;
    }
    state->set_all_references(reference);
    require(control.unilateral_step(), "moving step failed");
    for (size_t i = 0; i < 7; ++i) {
        require(std::abs(hardware->command[i].tau - effort[i]) < 1e-9, "FF uses target or friction");
        require(hardware->command[i].kp == 30 && hardware->command[i].dq == .2,
                "feedforward changed PD tracking");
    }

    // Calibrated torque has hard per-joint bounds even with extreme biases.
    tune.bias.fill(10000);
    control.SetFollowerGravity(dynamics, tune, 1, .1);
    for (int i = 0; i < 100; ++i) require(control.unilateral_step(), "clamped FF failed");
    for (size_t i = 0; i < 7; ++i) {
        require(std::abs(hardware->command[i].tau - kTauFfLimits[i]) < 1e-9, "FF exceeds limit");
    }

    hardware->arm[1].position = std::numeric_limits<double>::quiet_NaN();
    require(!control.unilateral_step(), "non-finite gravity input accepted");
    require(hardware->disabled && control.OutputDisabled(), "invalid model did not latch stop");
    for (const auto& command : hardware->command) {
        require(command.kp == 0 && command.kd == 0 && command.tau == 0,
                "fault retained gravity torque");
    }
    for (double tau : control.GetLastCommandEfforts()) {
        require(tau == 0, "stop telemetry retained old gravity torque");
    }
    const int stopped_sends = hardware->sends;
    require(!control.unilateral_step() && stopped_sends == hardware->sends,
            "latched output sent another frame");
}

void test_explicit_stop(const std::string& urdf) {
    auto dynamics = std::make_shared<Dynamics>(urdf, "openarm_body_link0", "openarm_left_link7");
    require(dynamics->Init(), "stop-test dynamics init");
    auto hardware = std::make_shared<MemoryHardware>();
    auto state = std::make_shared<RobotSystemState>(7, 1);
    Control control(hardware, dynamics.get(), dynamics.get(), state, .001,
                    ROLE_FOLLOWER, "left_arm", 7, 1);
    const std::vector<double> gains(8, 10), zero(8, 0);
    control.SetParameter(gains, gains, zero, zero, zero, zero);
    control.SetFollowerGravity(dynamics, GravityTune{}, 1, .1);
    auto filter = std::make_unique<SafetyFilter>(make_default_safety_config(8));
    auto* safety = filter.get();
    control.SetSafetyFilter(std::move(filter));
    require(control.SeedHoldMeasured() && control.unilateral_step(), "pre-stop hold failed");
    safety->set_mode(SafetyMode::Stop);
    require(!control.unilateral_step() && hardware->disabled, "explicit stop ignored");
    for (const auto& frame : hardware->command) {
        require(frame.kp == 0 && frame.kd == 0 && frame.tau == 0, "stop leaked gravity");
    }
}


// Exercise ReplayArm::send and Quest's shared follower setup on identical states.
// Also simulate a constant external load; no CAN or physical motors are opened.
void test_replay_alignment(const std::string& urdf) {
    unsetenv("OPENARM_QUEST_MIT_SCALE");
    setenv("OPENARM_QUEST_GRAVITY_FF", "0", 1);
    for (const std::string side : {"left", "right"}) {
        auto dynamics = std::make_shared<Dynamics>(urdf, "openarm_body_link0", "openarm_"+side+"_link7");
        require(dynamics->Init(), "aligned dynamics init");
        auto hardware = std::make_shared<MemoryHardware>();
        hardware->arm[3].position=1.2;
        ReplayArm replay(side, hardware, dynamics, GravityTune{}, GravityModel{});
        auto twin_hardware = std::make_shared<MemoryHardware>();
        twin_hardware->arm=hardware->arm;
        auto state = std::make_shared<RobotSystemState>(7, 1);
        Control teleop(twin_hardware, dynamics.get(), dynamics.get(), state, .005,
                       ROLE_FOLLOWER, side+"_arm", 7, 1);
        configure_follower_control(&teleop, load_follower_params(), 8, false, side+"_arm");
        require(teleop.SeedHoldMeasured(), "aligned teleop seed");
        auto target = replay.pose();
        target[6]+=.14;
        for (int n=0; n<200; ++n) {
            replay.read(); replay.send(target, {});
            std::vector<JointState> reference(8);
            for (size_t i=0; i<8; ++i) reference[i].position=target[i];
            state->set_all_references(reference);
            require(teleop.unilateral_step(), "aligned teleop step");
            for (size_t i=0; i<7; ++i) {
                const auto& a=hardware->command[i]; const auto& b=twin_hardware->command[i];
                require(a.kp==b.kp && a.kd==b.kd && a.q==b.q && a.dq==b.dq && a.tau==b.tau,
                        "replay and teleop MIT frames differ");
            }
        }
        const auto& frame=hardware->command[6];
        require(frame.kp==25 && frame.kd==.2, "J7 did not use follower gains");
        require(frame.kp*(frame.q-hardware->arm[6].position)+frame.tau > 3.4,
                "replay still cancels PD at its former 3 Nm total cap");
        // 1 Nm constant load, I=.03 kg m^2, viscous damping=.4 Nm s/rad.
        // Equilibrium should be error=load/Kp=.04 rad, versus .125 at old Kp=8.
        for (int n=0; n<2000; ++n) {
            replay.read(); replay.send(target, {});
            const auto c=hardware->command[6];
            for (int sub=0; sub<20; ++sub) {
                auto& motor=hardware->arm[6];
                const double torque=c.kp*(c.q-motor.position)+c.kd*(c.dq-motor.velocity)+c.tau;
                motor.velocity+=(torque-1-.4*motor.velocity)/.03*.00025;
                motor.position+=motor.velocity*.00025;
                motor.effort=torque;
            }
        }
        require(std::abs(target[6]-hardware->arm[6].position-.04)<.002,
                "loaded virtual J7 does not reach expected aligned equilibrium");
        require(std::abs(hardware->arm[6].velocity)<.01, "loaded virtual J7 did not settle");
    }
    unsetenv("OPENARM_QUEST_GRAVITY_FF");
}


void test_replay_gravity_alignment(const std::string& urdf) {
    unsetenv("OPENARM_QUEST_GRAVITY_FF");
    GravityTune left, right; GravityModel model;
    require(load_gravity_yaml("config/gravity_comp.yaml", &left, &right, &model), "load calibration");
    for (const std::string side : {"left", "right"}) {
        const auto& tune=side=="left" ? left : right;
        auto dynamics=std::make_shared<Dynamics>(urdf, "openarm_body_link0", "openarm_"+side+"_link7");
        configure_follower_gravity_model(dynamics.get(), model, side);
        require(dynamics->Init(), "gravity model init");
        auto hw=std::make_shared<MemoryHardware>(); hw->arm[3].position=1.2;
        ReplayArm replay(side, hw, dynamics, tune, model);
        auto twin=std::make_shared<MemoryHardware>(); twin->arm=hw->arm;
        auto state=std::make_shared<RobotSystemState>(7,1);
        Control teleop(twin, dynamics.get(), dynamics.get(), state, .005, ROLE_FOLLOWER, side+"_arm",7,1);
        configure_follower_control(&teleop, load_follower_params(),8,false,side+"_arm");
        teleop.SetFollowerGravity(dynamics,tune,model.tracking_ff_gain,1);
        require(teleop.SeedHoldMeasured(),"gravity seed");
        const auto target=replay.pose();
        for(int n=0;n<220;++n) {
            replay.read(); replay.send(target,{});
            std::vector<JointState> refs(8);
            for(size_t i=0;i<8;++i) refs[i].position=target[i];
            state->set_all_references(refs);
            require(teleop.unilateral_step(),"gravity twin step");
            for(size_t i=0;i<7;++i) {
                const auto& a=hw->command[i]; const auto& b=twin->command[i];
                require(a.kp==b.kp && a.kd==b.kd && a.q==b.q && a.dq==b.dq && a.tau==b.tau,
                        "gravity ramp or MIT command differs between replay and teleop");
            }
        }
        const int sends=hw->sends;
        hw->arm[6].velocity=2.63004;
        replay.read(); replay.send(target,{});
        require(hw->sends==sends+1,"valid startup J7 speed was rejected");
        hw->arm[6].velocity=3.01;
        bool stopped=false;
        try { replay.read(); replay.send(target,{}); } catch(const std::runtime_error&) { stopped=true; }
        require(stopped && hw->sends==sends+1,"overspeed sent another replay command");
    }
}

void test_replay_overspeed_disables_both(const std::string& urdf) {
    GravityTune tune; GravityModel model;
    auto left_hw=std::make_shared<MemoryHardware>();
    auto right_hw=std::make_shared<MemoryHardware>();
    auto left_dynamics=std::make_shared<Dynamics>(urdf,"openarm_body_link0","openarm_left_link7");
    auto right_dynamics=std::make_shared<Dynamics>(urdf,"openarm_body_link0","openarm_right_link7");
    require(left_dynamics->Init() && right_dynamics->Init(), "stop dynamics init");
    int left_sends=0, right_sends=0;
    bool stopped=false;
    try {
        ReplayArm left("left",left_hw,left_dynamics,tune,model);
        ReplayArm right("right",right_hw,right_dynamics,tune,model);
        left_sends=left_hw->sends; right_sends=right_hw->sends;
        right_hw->arm[6].velocity=-3.01;
        left.read(); right.read();
        left.send(left.pose(),{}); right.send(right.pose(),{});
    } catch (const std::runtime_error&) { stopped=true; }
    require(stopped,"J7 overspeed did not stop");
    require(left_hw->disabled && right_hw->disabled,"overspeed did not disable both arms");
    require(left_hw->sends==left_sends && right_hw->sends==right_sends,"overspeed sent new commands");
}

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    try {
        test_side(argv[1], "left");
        test_side(argv[1], "right");
        test_explicit_stop(argv[1]);
        test_replay_alignment(argv[1]);
        test_replay_gravity_alignment(argv[1]);
        test_replay_overspeed_disables_both(argv[1]);
        std::cout << "teleop_gravity_test: ok\n";
    } catch (const std::exception& error) {
        std::cerr << "teleop_gravity_test: " << error.what() << '\n';
        return 1;
    }
}
