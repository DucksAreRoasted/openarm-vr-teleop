#include <controller/gravity_calibration_plan.hpp>
#include <cassert>
#include <iostream>
#include <set>

using namespace gravity_calibration;

int main() {
    std::set<int> ids;
    for (const auto& point : kLeftWaypoints) {
        assert(ids.insert(point.traj_id).second);
        for (int sign : {-1, 1}) {
            const Pose offset = approach_offset(point.q, sign);
            const Pose mirrored = mirror_right(offset);
            assert(mirror_right(mirrored) == offset);
            for (size_t j = 0; j < 7; ++j) {
                assert(offset[j] >= kLeftLower[j] + kLimitMargin - 1e-9);
                assert(offset[j] <= kLeftUpper[j] - kLimitMargin + 1e-9);
                assert(point.q[j] >= kLeftLower[j] && point.q[j] <= kLeftUpper[j]);
            }
            const auto order = approach_order(offset, mirrored, point.q);
            assert(order[0] == sign && order[1] == -sign);
            const double duration = ramp_duration(offset, point.q, TRANSIT_JOINT_SPEED);
            for (int step = 0; step <= 100; ++step) {
                const double t = step / 100.0;
                for (size_t j = 0; j < 7; ++j) {
                    const double speed = std::abs(point.q[j] - offset[j]) * 6 * t * (1 - t) / duration;
                    assert(speed <= TRANSIT_JOINT_SPEED + 1e-9);
                }
            }
        }
    }
    for (int id : {350, 351, 352, 353}) assert(ids.count(id));
    Pose zero{}, q{}, torque{};
    StableHoldWindow stable;
    bool complete = false;
    for (int i = 0; i <= 121; ++i) {
        complete = stable.update(i * 0.01, true, zero, zero, zero, zero);
        if (i < 120) assert(!complete);
    }
    assert(complete);
    // Motion on either arm and a slowly drifting plateau cannot finish early.
    StableHoldWindow drifting, noisy_torque;
    for (int i = 0; i < 300; ++i) {
        q[0] = i * 0.00015;  // 0.015 rad/s: quiet-speed test alone would pass.
        assert(!drifting.update(i * 0.01, true, zero, q, zero, zero));
        torque[5] = (i % 2) ? 0.03 : -0.03;
        assert(!noisy_torque.update(i * 0.01, true, zero, zero, torque, zero));
    }
    // Two encoder counts every 10 ms used to look faster than 0.05 rad/s.
    SettleSpeed dither;
    for (int i = 0; i < 40; ++i) {
        Pose noisy{};
        noisy[3] = (i % 2) ? 0.0008 : -0.0008;
        dither.push(i * 0.01, noisy);
    }
    assert(dither.speed() < 0.05);
    SettleSpeed creep;
    for (int i = 0; i < 40; ++i) {
        Pose moving{};
        moving[0] = i * 0.01 * 0.08;
        creep.push(i * 0.01, moving);
    }
    assert(creep.speed() > 0.05 && creep.speed() < 0.12);
    SettleSpeed spike;
    Pose held{};
    for (int i = 0; i < 5; ++i) spike.push(i * 0.01, held);
    held[1] = 0.01;
    for (int i = 0; i < 30; ++i) spike.push(0.05 + i * 0.01, held);
    assert(spike.speed() < 0.05);
    SettleSpeed fresh;
    fresh.push(0.0, zero);
    assert(fresh.speed() > 1e6);

    assert(!stable.update(1.22, false, zero, zero, zero, zero));
    assert(!stable.update(1.23, true, zero, zero, zero, zero));
    StableHoldWindow sparse;
    for (int i = 0; i < 100; ++i) {
        assert(!sparse.update(i * 0.1, true, zero, zero, zero, zero));
    }
    // Emit production waypoints for the independent MuJoCo geometry smoke test.
    for (const auto& point : kLeftWaypoints) {
        std::cout << point.traj_id;
        for (double value : point.q) std::cout << ',' << value;
        std::cout << '\n';
    }
}
