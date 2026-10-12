# Repository Guidelines

## Project Structure & Module Organization

- `src/` contains the C++ control core: controllers, hardware adapters, pose I/O, safety filters, and OpenArm integration.
- `control/` contains executable entry points for teleoperation, gravity compensation, calibration, and motor inspection.
- `script/openarm_app/` implements the PySide6 console; other `script/` modules handle recording, cameras, and dataset export.
- `scripts/` contains launchers, CAN setup, and gravity calibration utilities; `scripts/lib/` holds shared Python helpers.
- `quest/` provides the WebXR client and Python bridge. `config/` stores YAML settings; `assets/` stores the URDF and startup trajectory.
- `ros2/src/openarm_moveit_config/` contains MoveIt integration. `third_party/openarm_can/` is vendored CAN code. Keep generated builds and recordings out of source changes.

## Build, Test, and Development Commands

Install the system dependencies listed in `README.md`, then run:

```bash
./install.sh                    # Create .venv and install Python dependencies
cmake -S . -B build -DPython3_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build -j           # Build C++ executables
./app.sh                        # Launch the Qt console
./run.sh --sim                  # Build and launch simulation plus Quest bridge
./scripts/run_moveit.sh --sim   # Launch MoveIt simulation (ROS 2 required)
```

`run.sh` and `script/replay_teaching.py` resolve the MuJoCo scene from the script directory (`models/OpenArm_V1/scene_table.xml`) unless `OPENARM_MUJOCO_XML` is set. If the historical `/home/ducks/.../scene_table.xml` file is still present, that path remains the default.

## Coding Style & Naming Conventions

Use four-space indentation in C++ and Python, matching nearby code. C++ uses C++17, `snake_case` files/functions, and PascalCase types. Python uses `snake_case` functions/modules and PascalCase classes. Preserve shell quoting and `set -euo pipefail`. No formatter or linter configuration is supplied; CMake enables `-Wall -Wextra -Wpedantic`.

## Testing Guidelines

`.github/workflows/tests.yml` runs `unittest discover -s tests` and the Node checks on push and pull request. There is no coverage threshold. Rebuild changed C++ targets and smoke-test affected behavior in simulation. For UI or recording changes, verify start/stop, labeling, and export using disposable recordings. Document commands, results, and hardware dependencies when reporting validation.

## Commit & Pull Request Guidelines

Git history is unavailable in this checkout, so existing commit conventions cannot be verified. Use short, imperative subjects describing one focused change. PRs should explain the behavior change, list validation commands, link relevant issues, and include screenshots for UI changes.

## Hardware & Configuration

Validate control changes in simulation before real-arm execution. Confirm the emergency stop and CAN mapping before using `--real` (defaults: left `can1`, right `can0`). Do not commit credentials, private keys, or generated recordings.
