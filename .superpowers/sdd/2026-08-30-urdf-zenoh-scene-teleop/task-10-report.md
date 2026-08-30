# Task 10 report

## Scoped verification

Command (from `PICO_tracker/`):

```text
pixi run python -m pytest src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py -q
```

Output:

```text
.........                                                                [100%]
=============================== warnings summary ===============================
.pixi/envs/default/lib/python3.11/site-packages/hppfcl/__init__.py:3
  Warning: Please update your 'hppfcl' imports to 'coal'

-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html
9 passed, 1 warning in 0.39s
```

Command:

```text
pixi run python -m py_compile src/spd_vr/spd_vr/session_state.py src/spd_vr/spd_vr/viewer_window.py src/spd_vr/spd_vr/viewer.py src/spd_vr/spd_vr/simulator.py src/spd_vr/spd_vr/runtime.py && pixi run python -c "import spd_vr.session_state, spd_vr.viewer_window, spd_vr.viewer, spd_vr.simulator, spd_vr.runtime; print('imports=ok')"
```

Output:

```text
imports=ok
```

Command:

```text
pixi run python -m spd_vr.viewer --headless --ticks 480 --auto-start
```

Output:

```text
headless=True ticks=480 simulated_seconds=1.000000 finite=True synthetic=True
```

## Implementation evidence

- `SessionController.apply(ControlFrame)` owns the canonical `ControlSequenceGate` lifecycle for START, PAUSE, RESUME, REALIGN, RESET and SHUTDOWN; exact duplicate frames are no-ops, rollback/conflicting duplicates are rejected, and RESUME/REALIGN require a fresh alignment.
- `PlantController` owns the only complete 54-DoF `MjModel`/`MjData` in the viewer path. Arm and tracking callbacks only replace bounded latest mailboxes; physics boundaries consume new frames, isolate left/right validity, retain the last target during stale HOLD, clamp finite controls, and call `mujoco.mj_step` at the model timestep.
- `ViewerRuntime` uses independent absolute-deadline 480 Hz physics and 60 Hz render scheduling. Rendering is skipped when delayed rather than catching up, and headless mode does not open GLFW/MuJoCo viewer resources. `ViewerWindow` sends Q/Escape shutdown once.
- `runtime.py` is an explicit hardware-free mock episode path; ROS/rclpy live input, UDP arm receiver parameters, and live mailbox wiring were removed from runtime/simulator. The existing `UnifiedSimulator` remains available for verified artifact/replay consumers, with its UDP receiver removed.

## Artifact boundary and Task7 blocker

The production viewer path is fail-closed: when no explicit synthetic fixture is selected, it calls `verify_artifacts` for the generated model manifest, source URDF, output XML and hashes before loading the full plant. The headless smoke above intentionally uses the named in-code `PlantController.synthetic_fixture()` because the authoritative generated artifacts are unavailable; `synthetic=True` is printed and is not authoritative robot/artifact evidence.

Task7 remains externally blocked by the authoritative source quality gate: direct CoACD on `Link_Base.STL` with seed 0, max 16 pieces and max 64 vertices measured arm/base p95 surface error `0.036785362 m`, above the fixed `0.003 m` threshold. No collision threshold or fallback was widened or added.
