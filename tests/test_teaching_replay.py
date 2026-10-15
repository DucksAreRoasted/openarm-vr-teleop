"""Offline trajectory rejection, interpolation and replay command constraints."""
from pathlib import Path
import csv
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script"))
from replay_teaching import resolve_source


class TeachingReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.directory = Path(cls.temp.name)
        source = cls.directory / "replay.cpp"
        cls.binary = cls.directory / "replay"
        source.write_text(r'''
#include <controller/teaching_replay.hpp>
#include <cassert>
int main(int argc,char** argv) {
    TeachingTrajectory trajectory;
    if(argc>2 && std::string(argv[2])=="rounding") {
        trajectory.load(argv[1]);
        TeachingTrajectory::Pose lo,hi;lo.fill(-2);hi.fill(2);
        lo[0]=3e-7;hi[0]=.2-3e-7;
        assert(trajectory.validate_limits(lo,hi)==2);
        assert(trajectory.sample(0).position[0]==lo[0]);
        assert(trajectory.sample(2).position[0]==hi[0]);
        assert(trajectory.sample(1).position[0]>=lo[0]);
        assert(trajectory.sample(1).position[0]<=hi[0]);
        for(bool lower:{false,true}) {
            trajectory.load(argv[1]);lo[0]=lower ? 2e-6 : -2;hi[0]=lower ? 2 : .2-2e-6;
            bool rejected=false;
            try {trajectory.validate_limits(lo,hi);} catch(const std::exception&) {rejected=true;}
            assert(rejected);
        }
        return 0;
    }
    if(argc>2) {
        try { trajectory.load(argv[1]); } catch(const std::exception&) { return 0; }
        return 1;
    }
    trajectory.load(argv[1]);
    assert(trajectory.size()==2 && trajectory.duration()==2);
    auto middle=trajectory.sample(1);
    assert(std::abs(middle.position[0]-.1)<1e-9);
    assert(std::abs(middle.velocity[0]-.1)<1e-9); // recorded dq=999 is ignored
    assert(trajectory.sample(2).velocity[0]==0);
    trajectory.convert_grippers(false,true);
    assert(std::abs(trajectory.sample(0).position[7]-(.18/1.6*.044))<1e-9);
    TeachingTrajectory::Pose lo,hi; lo.fill(-2);hi.fill(2);
    for(size_t i:{7u,15u}) {lo[i]=0;hi[i]=.044;}
    trajectory.validate_limits(lo,hi);
    bool rejected=false;
    hi[0]=.05;
    try {trajectory.validate_limits(lo,hi);} catch(const std::exception&) {rejected=true;}
    assert(rejected);hi[0]=2;
    auto measured=trajectory.sample(0).position;
    TeachingTrajectory::Pose oldv{};
    TeachingReplayReference reference(measured,lo,hi);
    for(int n=0;n<1000;++n) {
        auto goal=trajectory.sample(std::min(2.,n*.005));
        reference.step(goal,measured,oldv,.005,false);
        for(size_t i=0;i<16;++i) if(i%8!=7) {
            assert(std::abs(reference.velocity()[i])<=.6+1e-9);
            assert(std::abs(reference.velocity()[i]-oldv[i])<=.006+1e-9);
        }
        measured=reference.position();oldv=reference.velocity();
    }
    assert(std::abs(measured[0]-.2)<1e-4);
}
''')
        subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"), str(source), "-o", str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def write_csv(self, name="good.csv", count=16, times=(10, 12)):
        path = self.directory / name
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["time"] + [field for i in range(count) for field in (f"q{i}", f"dq{i}", f"tau{i}")])
            for frame, instant in enumerate(times):
                row = [instant]
                for i in range(count):
                    q = -.8 if i%8==7 else 1.5 if i%8==3 else .2*frame if i==0 else 0
                    row.extend((q,999,100))
                writer.writerow(row)
        return path

    def test_interpolation_units_and_command_limits(self):
        subprocess.run([str(self.binary), str(self.write_csv())], check=True)

    def test_rounding_at_limits_is_clamped_but_larger_violations_rejected(self):
        path = self.write_csv("rounding.csv")
        original = path.read_bytes()
        subprocess.run([str(self.binary), str(path), "rounding"], check=True)
        self.assertEqual(path.read_bytes(), original)

    def test_reject_corrupt_trajectory(self):
        for label, times in (("duplicate",(0,0)), ("reversed",(2,1)), ("negative",(-1,1))):
            path = self.write_csv(label+".csv", times=times)
            subprocess.run([str(self.binary), str(path), "reject"], check=True)
        original = self.write_csv().read_text()
        for label, text in (("nan",original.replace("100", "nan", 1)),
                            ("truncated",original.rsplit(",",1)[0]),
                            ("bad_header",original.replace("q15", "q99"))):
            path = self.directory / (label+".csv")
            path.write_text(text)
            subprocess.run([str(self.binary), str(path), "reject"], check=True)

    def test_split_episode_and_invalid_row(self):
        episode = self.directory / "episode"
        for side in ("left", "right"):
            path = episode / f"real_teleop_split/{side}/real_teleop_{side}.csv"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(self.write_csv(side+".csv", count=8, times=(0,2)).read_text())
        merged = resolve_source(episode, self.directory)
        with merged.open() as handle:
            self.assertEqual(len(next(csv.reader(handle))),49)
        left = episode / "real_teleop_split/left/real_teleop_left.csv"
        left.write_text(left.read_text().replace("100", "nan", 1))
        with self.assertRaises(ValueError):
            resolve_source(episode, self.directory)


if __name__ == "__main__":
    unittest.main()
