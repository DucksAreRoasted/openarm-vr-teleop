"""run.sh resolves the MuJoCo scene from the script location, not the cwd."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script"))
from replay_teaching import resolve_scene_xml
RUN = ROOT / "run.sh"
LEGACY = Path("/home/ducks/motion-trajectory-studio/model_library/OpenArm_V1/scene_table.xml")
REPO_XML = ROOT / "models" / "OpenArm_V1" / "scene_table.xml"


def _paths(cwd: Path, extra_env: dict[str, str] | None = None) -> dict[str, str]:
    env = os.environ.copy()
    env.pop("OPENARM_MUJOCO_XML", None)
    env.pop("OPENARM_TELEOP_URDF", None)
    if extra_env:
        env.update(extra_env)
    completed = subprocess.run(
        [str(RUN), "--print-paths"],
        cwd=cwd,
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )
    parsed = {}
    for line in completed.stdout.splitlines():
        key, value = line.split("=", 1)
        parsed[key] = value
    return parsed


class RunShPathTests(unittest.TestCase):
    def test_help_from_another_directory(self):
        completed = subprocess.run(
            [str(RUN), "--help"],
            cwd="/tmp",
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("--print-paths", completed.stdout)
        self.assertNotIn("unknown argument", completed.stdout)

    def test_scene_resolves_from_another_directory(self):
        from_tmp = _paths(Path("/tmp"))
        from_root = _paths(ROOT)
        self.assertEqual(from_tmp, from_root)
        self.assertEqual(from_tmp["ROOT"], str(ROOT))
        self.assertEqual(from_tmp["URDF"], str(ROOT / "assets" / "v1.urdf"))
        self.assertTrue(Path(from_tmp["URDF"]).is_file())
        if LEGACY.is_file():
            self.assertEqual(from_tmp["XML"], str(LEGACY))
        else:
            self.assertEqual(from_tmp["XML"], str(REPO_XML))
        self.assertTrue(Path(from_tmp["XML"]).is_file())

    def test_environment_overrides_scene_and_urdf(self):
        scene = "/tmp/openarm-scene-override.xml"
        urdf = "/tmp/openarm-urdf-override.urdf"
        parsed = _paths(
            Path("/tmp"),
            {"OPENARM_MUJOCO_XML": scene, "OPENARM_TELEOP_URDF": urdf},
        )
        self.assertEqual(parsed["XML"], scene)
        self.assertEqual(parsed["URDF"], urdf)
        self.assertEqual(parsed["ROOT"], str(ROOT))

    def test_replay_uses_the_same_scene_as_run_sh(self):
        previous = os.environ.pop("OPENARM_MUJOCO_XML", None)
        try:
            self.assertEqual(resolve_scene_xml(), _paths(Path("/tmp"))["XML"])
            os.environ["OPENARM_MUJOCO_XML"] = "/tmp/openarm-replay-scene.xml"
            self.assertEqual(resolve_scene_xml(), "/tmp/openarm-replay-scene.xml")
            overridden = _paths(ROOT, {"OPENARM_MUJOCO_XML": "/tmp/openarm-replay-scene.xml"})
            self.assertEqual(resolve_scene_xml(), overridden["XML"])
        finally:
            if previous is None:
                os.environ.pop("OPENARM_MUJOCO_XML", None)
            else:
                os.environ["OPENARM_MUJOCO_XML"] = previous


if __name__ == "__main__":
    unittest.main()
