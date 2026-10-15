"""Repeated return planning must not stall the real-arm reference heartbeat."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ReplayPreparationJobTests(unittest.TestCase):
    def test_slow_repeated_planning_holds_pose_without_relaxing_deadlines(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'job.cpp'
            binary = Path(directory) / 'job'
            source.write_text(r'''
#include <controller/replay_preparation_job.hpp>
#include <cassert>
#include <thread>
int main(int argc, char** argv) {
    assert(argc==2);
    TeachingTrajectory trajectory;
    trajectory.load(argv[1]);
    const auto held=trajectory.sample(trajectory.duration()).position;
    const auto target=trajectory.sample(0).position;
    TeachingTrajectory::Pose lo{}, hi{};
    lo.fill(-10); hi.fill(10);
    TeachingReplayReference reference(held,lo,hi);
    ReplayPreparation active;
    ReplayPreparationJob job;
    const auto servo_thread=std::this_thread::get_id();
    for (int cycle=0; cycle<3; ++cycle) {
        std::promise<void> allow_build;
        auto release=allow_build.get_future();
        job.start([&, release=std::move(release)]() mutable {
            assert(std::this_thread::get_id()!=servo_thread);
            // Model a slow disk/CSV preparation while the powered servo holds.
            release.wait();
            ReplayPreparation prepared(argv[1],.2,.1);
            prepared.reset(held,target,false,false);
            return prepared;
        });
        for (int heartbeat=0; heartbeat<80; ++heartbeat) {
            assert(job.busy());
            assert(!job.take(&active)); // polling cannot wait for the builder
            TeachingTrajectory::Sample goal{};
            goal.position=held;
            reference.step(goal,held,{},.005,true);
            assert(reference.position()==held);
        }
        allow_build.set_value();
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
        while (!job.take(&active)) {
            assert(std::chrono::steady_clock::now()<deadline);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        assert(!job.busy());
        assert(!job.take(&active));
        assert(active.startup_duration()==0);
        assert(!active.startup_active(0));
        assert(active.sample(0).position==held);
        const auto final=active.sample(active.duration()).position;
        for (size_t i=0;i<16;++i) assert(std::abs(final[i]-target[i])<1e-9);
    }
    job.start([]() -> ReplayPreparation { throw std::runtime_error("bad source"); });
    bool error=false;
    for (int n=0;n<1000 && !error;++n) {
        try { job.take(&active); }
        catch (const std::runtime_error&) { error=true; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(error); // worker errors reach the caller's normal motor-stop unwind
    TeachingTrajectory::Sample goal{}; goal.position=held;
    bool deadline_preserved=false;
    try { reference.step(goal,held,{},.051,true); }
    catch (const std::runtime_error&) { deadline_preserved=true; }
    assert(deadline_preserved); // never silence a genuine stalled servo
}
''')
            subprocess.run(['c++', '-std=c++17', '-O2', '-pthread', '-I', str(ROOT/'src'),
                            str(source), str(ROOT/'src/io/teleop_io.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary), str(ROOT/'assets/steady_action_on_table.csv')], check=True, timeout=10)
