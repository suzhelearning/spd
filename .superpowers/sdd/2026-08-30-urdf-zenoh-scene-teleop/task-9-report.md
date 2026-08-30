# Task 9 report

## Scoped verification

Command (from `PICO_tracker/`):

```text
.pixi/envs/default/bin/python -m pytest ../wuji-retargeting/tests/test_reduced_robot.py src/spd_vr/test/test_retarget_pair.py -q
```

Output:

```text
.....                                                                    [100%]
=============================== warnings summary ===============================
.pixi/envs/default/lib/python3.11/site-packages/hppfcl/__init__.py:3
  Warning: Please update your 'hppfcl' imports to 'coal'
  
-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html
5 passed, 1 warning in 0.36s
```

Command:

```text
.pixi/envs/default/bin/python -m py_compile ../wuji-retargeting/wuji_retargeting/robot.py ../wuji-retargeting/wuji_retargeting/opt/base.py src/spd_vr/spd_vr/retarget_pair.py && .pixi/envs/default/bin/python -c "from wuji_retargeting.robot import RobotWrapper; from wuji_retargeting.opt.base import BaseOptimizer; from spd_vr.retarget_pair import WujiRetargetPair; print('imports ok')"
```

Output:

```text
imports ok
```

## Implementation evidence

- `RobotWrapper` loads the authoritative full URDF, validates unique/existing side-specific active names, locks inactive joints at `pin.neutral(full_model)`, and exposes a 20-DoF reduced model.
- Direct reduced-model tests consume `assets/tianji_wuji2/tianji_wuji2.urdf` directly; they do not depend on a Task7 generated artifact.
- `BaseOptimizer` passes `optimizer.active_joint_names` to `RobotWrapper`.
- `WujiRetargetPair.from_manifest` validates manifest schema, authoritative URDF filename/hash, exact left/right 20-joint lists, overrides config URDF/active names, removes the obsolete MJCF live path, and maps outputs in manifest order.
- Pair processing clamps finite targets to source robot limits and keeps side failures/inactive inputs isolated; epoch changes reset both filters.

## Task7 blocker

Task7's five-artifact generation remains externally blocked by the authoritative source quality gate: direct CoACD on `Link_Base.STL` (seed 0, max 16 pieces, max 64 vertices) measured arm/base p95 surface error `0.036785362 m`, above the fixed `0.003 m` threshold. Task9's direct URDF reduced-model tests remain runnable and passing independently of that unavailable generated artifact.
