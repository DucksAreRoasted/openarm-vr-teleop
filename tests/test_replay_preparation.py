"""Full startup waypoints and endpoint transitions, independent of hardware."""
from pathlib import Path
import csv
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ReplayPreparationTests(unittest.TestCase):
    def test_waypoints_connections_and_reset(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            startup = directory / 'startup.csv'
            with startup.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time'] + [name for i in range(16) for name in (f'q{i}', f'dq{i}')])
                for instant, angle in ((0, 0), (1, .4), (2, 0)):
                    writer.writerow([instant] + [value for i in range(16) for value in (angle if i == 0 else 0, .123 if i == 0 and instant == 1 else 0)])
            source = directory / 'preparation.cpp'
            binary = directory / 'preparation'
            source.write_text(r'''
#include <controller/replay_preparation.hpp>
#include <cassert>
int main(int argc, char** argv) {
    assert(argc == 2);
    TeachingTrajectory::Pose origin{}, target{}, lo{}, hi{};
    lo.fill(-10); hi.fill(10);
    ReplayPreparation prep(argv[1], .2, .1);
    prep.reset(origin, origin, false);
    assert(std::abs(prep.duration()-2.1) < 1e-9);
    assert(std::abs(prep.sample(1).position[0]-.4) < 1e-9);
    assert(std::abs(prep.sample(1).velocity[0]-.123) < 1e-9);
    assert(prep.sample(2.1).position[0] == 0);
    assert(std::string(prep.phase(1)) == "startup");
    assert(std::string(prep.phase(2.1)) == "connecting");
    target[0] = .3;
    origin[0] = -.3;
    prep.reset(origin, target, false);
    assert(std::abs(prep.startup_duration()-2.3) < 1e-9);
    assert(prep.sample(0).position == origin);
    assert(std::abs(prep.sample(1.2).position[0]-.4) < 1e-9);
    auto end = prep.sample(prep.duration());
    assert(std::abs(end.position[0]-.3) < 1e-9);
    assert(std::abs(end.velocity[0]) < 1e-9);
    // Same samples as the Quest CsvReplay, including recorded velocity and the
    // transition. A second sample at an earlier time must also work after reset.
    CsvReplay quest;
    assert(quest.load(argv[1], 16));
    std::vector<JointState> initial(16);
    initial[0].position = -.3;
    assert(quest.prepend_transition_from(initial, 16, .2));
    TeachingReplayReference reference(origin, lo, hi);
    for (double t : {0., .1, .2, .7, 1.2, 2.19, .1}) {
        auto expected = quest.sample_at(t);
        auto actual = prep.sample(t);
        assert(expected);
        reference.follow_startup(actual, .005);
        for (size_t i = 0; i < 16; ++i) {
            assert(std::abs(actual.position[i]-(*expected)[i].position) < 1e-9);
            assert(std::abs(actual.velocity[i]-(*expected)[i].velocity) < 1e-9);
            assert(reference.position()[i] == actual.position[i]);
            assert(reference.velocity()[i] == actual.velocity[i]);
        }
    }
    assert(prep.sample(2.25).velocity[0] == 0); // shared end hold
    bool rejected = false;
    auto invalid = prep.sample(0);
    invalid.position[0] = 11;
    try { reference.follow_startup(invalid, .005); }
    catch (const std::runtime_error&) { rejected = true; }
    assert(rejected);
    // Returning must bypass startup waypoints even when a startup file exists.
    prep.reset(target, target, false, false);
    assert(prep.startup_duration() == 0);
    assert(prep.duration() == 0);
    assert(!prep.startup_active(0));
    assert(prep.sample(0).position == target);
    prep.reset(origin, target, false, false);
    for (int n=0; n<=100; ++n) {
        const auto sample=prep.sample(prep.duration()*n/100.);
        assert(sample.position[0] >= origin[0]-1e-9);
        assert(sample.position[0] <= target[0]+1e-9);
        assert(std::abs(sample.velocity[0]) <= .6+1e-9);
    }
    // A fresh preparation still executes the original startup trajectory.
    prep.reset(origin, target, false);
    assert(prep.startup_duration() > 2);
    ReplayPreparation direct;
    direct.reset(origin, target, false);
    assert(direct.sample(0).position == origin);
    assert(direct.sample(direct.duration()).position == target);

}
''')
            subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-I', str(ROOT / 'src'),
                            str(source), str(ROOT / 'src/io/teleop_io.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary), str(startup)], check=True)

    def test_mirrored_startup_gathers_the_right_arm_first(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            startup = directory / 'startup.csv'
            with startup.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time'] + [name for i in range(16) for name in (f'q{i}', f'dq{i}')])
                for instant, angle in ((0, 0), (1, .2)):
                    row = [instant]
                    for joint in range(16):
                        value = angle if joint == 0 else -angle if joint == 8 else 0
                        row.extend((value, 0))
                    writer.writerow(row)
            source = directory / 'gather.cpp'
            binary = directory / 'gather'
            source.write_text(r'''
#include <controller/replay_preparation.hpp>
#include <cassert>
#include <cmath>
int main(int argc, char** argv) {
    assert(argc == 2);
    TeachingTrajectory::Pose current{}, target{};
    current[0] = .1;
    current[8] = .1; // not the mirror; mirror of the left shoulder is -.1
    ReplayPreparation prep(argv[1], .4, 0);
    prep.reset(current, target, false);
    const auto start = prep.sample(0);
    assert(std::abs(start.position[0]-.1) < 1e-6);
    assert(std::abs(start.position[8]-.1) < 1e-6);
    const auto gathered = prep.sample(.4);
    assert(std::abs(gathered.position[0]-.1) < 1e-3);
    assert(std::abs(gathered.position[8]+.1) < 1e-3);
    bool mirrored_motion = false;
    for (int step = 1; step <= 40; ++step) {
        const auto sample = prep.sample(prep.startup_duration()*step/40.);
        if (sample.position[0] > .15) {
            mirrored_motion = true;
            assert(std::abs(sample.position[8]+sample.position[0]) < 1e-3);
        }
    }
    assert(mirrored_motion);
}
''')
            subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-I', str(ROOT / 'src'),
                            str(source), str(ROOT / 'src/io/teleop_io.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary), str(startup)], check=True)

    def test_launcher_uses_quest_startup_configuration(self):
        import os
        import sys
        from unittest.mock import Mock, patch
        sys.path.insert(0, str(ROOT / 'script'))
        import replay_teaching
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'recording.csv'
            source.touch()
            custom = Path(directory) / 'custom.csv'
            override = Path(directory) / 'override.csv'
            for environment, extra, expected in (
                ('', [], ROOT / 'assets/steady_action_on_table.csv'),
                (str(custom), [], custom),
                (str(custom), ['--startup-file', str(override)], override),
            ):
                with self.subTest(expected=expected), \
                     patch.dict(os.environ, {'OPENARM_QUEST_STARTUP_FILE': environment}), \
                     patch.object(sys, 'argv', ['replay_teaching.py', str(source), '--sim', '--wait-for-start', *extra]), \
                     patch.object(replay_teaching, 'build'), \
                     patch.object(replay_teaching.os, 'getpgrp', return_value=os.getpid()), \
                     patch.object(replay_teaching.signal, 'signal'), \
                     patch.object(replay_teaching.subprocess, 'Popen', return_value=Mock(wait=Mock(return_value=0))) as launch:
                    self.assertEqual(replay_teaching.main(), 0)
                    command = launch.call_args.args[0]
                    self.assertEqual(command[command.index('--startup-file')+1], str(expected.resolve()))
                    self.assertEqual(float(command[command.index('--speed')+1]), 1.0)
