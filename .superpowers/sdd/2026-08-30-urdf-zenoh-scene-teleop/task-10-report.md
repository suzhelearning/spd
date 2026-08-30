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
pixi run python -m spd_vr.viewer --headless --synthetic --ticks 480 --auto-start
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

## Review round 1 verification

Scoped compilation:

```text
pixi run python -m py_compile src/spd_vr/spd_vr/session_state.py src/spd_vr/spd_vr/viewer_window.py src/spd_vr/spd_vr/viewer.py src/spd_vr/spd_vr/simulator.py src/spd_vr/spd_vr/runtime.py src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py
```

Output:

```text
(no output; exit 0)
```

Scoped tests:

```text
pixi run python -m pytest src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py -q

Output:

```text
..........                                                               [100%]
=============================== warnings summary ===============================
.pixi/envs/default/lib/python3.11/site-packages/hppfcl/__init__.py:3
  Warning: Please update your 'hppfcl' imports to 'coal'

-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html
10 passed, 1 warning in 0.40s
```

Headless full-plant smoke:

```text
pixi run python -m spd_vr.viewer --headless --synthetic --ticks 480 --auto-start
```

Output:

```text
headless=True ticks=480 simulated_seconds=1.000000 finite=True synthetic=True
```

The production viewer remains fail-closed at the verified artifact boundary. The
authoritative Task7 five-artifact set is still unavailable because the fixed
CoACD quality gate for `Link_Base.STL` measured arm/base p95 error
`0.036785362 m` versus the `0.003 m` threshold; this round adds no fallback or
threshold relaxation.

## Review round 2 verification

Scoped compilation:

```text
pixi run python -m py_compile src/spd_vr/spd_vr/session_state.py src/spd_vr/spd_vr/viewer_window.py src/spd_vr/spd_vr/viewer.py src/spd_vr/spd_vr/simulator.py src/spd_vr/spd_vr/runtime.py src/spd_vr/spd_vr/zenoh_transport.py src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py
```

Output:

```text
(no output; exit 0)
```

Scoped tests:

```text
pixi run python -m pytest src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py -q
```

Output:

```text
.............                                                            [100%]
=============================== warnings summary ===============================
.pixi/envs/default/lib/python3.11/site-packages/hppfcl/__init__.py:3
  Warning: Please update your 'hppfcl' imports to 'coal'

-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html
13 passed, 1 warning in 0.40s
```

Headless full-plant smoke:

```text
pixi run python -m spd_vr.viewer --headless --synthetic --ticks 480 --auto-start
```

Output:

```text
headless=True ticks=480 simulated_seconds=1.000000 finite=True synthetic=True
```

The production artifact blocker is unchanged: `Link_Base.STL` still fails the
fixed CoACD gate at arm/base p95 `0.036785362 m` versus `0.003 m`; production
viewer remains fail-closed and this round adds no synthetic fallback to that
path.

## Review round 3 verification

Scoped compilation:

```text
pixi run python -m py_compile src/spd_vr/spd_vr/session_state.py src/spd_vr/spd_vr/viewer_window.py src/spd_vr/spd_vr/viewer.py src/spd_vr/spd_vr/simulator.py src/spd_vr/spd_vr/runtime.py src/spd_vr/spd_vr/zenoh_transport.py src/spd_vr/spd_vr/arm_ik.py src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py src/spd_vr/test/test_arm_ik.py
```

Output:

```text
(no output; exit 0)
```

Scoped tests:

```text
pixi run python -m pytest src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py src/spd_vr/test/test_arm_ik.py -q
```

Output:

```text
...................                                                      [100%]
=============================== warnings summary ===============================
.pixi/envs/default/lib/python3.11/site-packages/hppfcl/__init__.py:3
  Warning: Please update your 'hppfcl' imports to 'coal'

test/test_arm_ik.py: 10 warnings
  Warning: "polish" is deprecated. Please use "polishing" instead.

test/test_arm_ik.py::test_dual_controller_keeps_side_hold_isolated
test/test_arm_ik.py::test_dual_controller_keeps_side_hold_isolated
test/test_arm_ik.py::test_dual_controller_keeps_side_hold_isolated
  Warning: The default value of raise_error will change to True in the future.

19 passed, 14 warnings in 0.42s
```

Headless full-plant smoke:

```text
pixi run python -m spd_vr.viewer --headless --synthetic --ticks 480 --auto-start
```

Output:

```text
headless=True ticks=480 simulated_seconds=1.000000 finite=True synthetic=True
```

The production artifact boundary remains fail-closed. Task7's authoritative
five-artifact set is still blocked by `Link_Base.STL` CoACD arm/base p95
`0.036785362 m` versus the fixed `0.003 m` threshold; no wire expansion,
fallback, or threshold relaxation was added.

## Review round 3 final evidence

The final round used the scoped command below:

```text
pixi run python -m py_compile src/spd_vr/spd_vr/session_state.py src/spd_vr/spd_vr/viewer_window.py src/spd_vr/spd_vr/viewer.py src/spd_vr/spd_vr/simulator.py src/spd_vr/spd_vr/runtime.py src/spd_vr/spd_vr/zenoh_transport.py src/spd_vr/spd_vr/arm_ik.py src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py src/spd_vr/test/test_arm_ik.py
```

```text
(no output; exit 0)
```

```text
pixi run python -m pytest src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py src/spd_vr/test/test_arm_ik.py -q
```

```text
...................                                                      [100%]
=============================== warnings summary ===============================
.pixi/envs/default/lib/python3.11/site-packages/hppfcl/__init__.py:3
  Warning: Please update your 'hppfcl' imports to 'coal'

test/test_arm_ik.py: 10 warnings
  Warning: "polish" is deprecated. Please use "polishing" instead.

test/test_arm_ik.py::test_dual_controller_keeps_side_hold_isolated
test/test_arm_ik.py::test_dual_controller_keeps_side_hold_isolated
test/test_arm_ik.py::test_dual_controller_keeps_side_hold_isolated
  Warning: The default value of raise_error will change to True in the future.

19 passed, 14 warnings in 0.42s
```

```text
pixi run python -m spd_vr.viewer --headless --synthetic --ticks 480 --auto-start
```

```text
headless=True ticks=480 simulated_seconds=1.000000 finite=True synthetic=True
```

The authoritative artifact blocker is unchanged: `Link_Base.STL` CoACD
arm/base p95 is `0.036785362 m`, above `0.003 m`. Production remains
manifest/hash verified and fail-closed; synthetic mode is explicit and is not
authoritative artifact evidence.

## Review round 4 final evidence

```text
pixi run python -m py_compile src/spd_vr/spd_vr/session_state.py src/spd_vr/spd_vr/viewer_window.py src/spd_vr/spd_vr/viewer.py src/spd_vr/spd_vr/simulator.py src/spd_vr/spd_vr/runtime.py src/spd_vr/spd_vr/zenoh_transport.py src/spd_vr/spd_vr/arm_ik.py src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py src/spd_vr/test/test_arm_ik.py src/spd_vr/test/test_zenoh_transport.py
```

```text
(no output; exit 0)
```

```text
pixi run python -m pytest src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py src/spd_vr/test/test_arm_ik.py src/spd_vr/test/test_zenoh_transport.py -q
```

```text
............................                                             [100%]
=============================== warnings summary ===============================
.pixi/envs/default/lib/python3.11/site-packages/hppfcl/__init__.py:3
  Warning: Please update your 'hppfcl' imports to 'coal'

test/test_arm_ik.py: 10 warnings
  Warning: "polish" is deprecated. Please use "polishing" instead.

test/test_arm_ik.py::test_dual_controller_keeps_side_hold_isolated
test/test_arm_ik.py::test_dual_controller_keeps_side_hold_isolated
test/test_arm_ik.py::test_dual_controller_keeps_side_hold_isolated
  Warning: The default value of raise_error will change to True in the future.

28 passed, 14 warnings in 0.55s
```

```text
pixi run python -m spd_vr.viewer --headless --synthetic --ticks 480 --auto-start
```

```text
headless=True ticks=480 simulated_seconds=1.000000 finite=True synthetic=True
```

The Task7 artifact blocker remains unchanged: `Link_Base.STL` CoACD arm/base
p95 `0.036785362 m` exceeds the fixed `0.003 m` threshold. The production
viewer remains manifest/hash verified and fail-closed; synthetic mode remains
explicit and non-authoritative.
