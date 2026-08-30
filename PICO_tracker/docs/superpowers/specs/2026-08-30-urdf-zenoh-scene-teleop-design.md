# URDF-First Zenoh Scene Teleoperation Design

**Date:** 2026-08-30

**Status:** Approved for implementation planning

## Goal

Use `/home/current/syz/spd/assets/tianji_wuji2/tianji_wuji2.urdf` as the only structural and physical model source for a real-PICO, ROS-free teleoperation path. PICO optical hand tracking drives the complete Tianji dual-arm and bilateral Wuji2 model in a PC MuJoCo window. This feature controls simulation only and never sends commands to physical Tianji or Wuji2 hardware.

The first delivered scene is an empty free-teleoperation scene with a ground plane. Session control uses the PC keyboard. PICO-to-robot alignment is established from a user-held neutral pose at startup and after explicit reset.

## Current-State Facts

The authoritative URDF contains:

- 80 links;
- 79 joints: 25 fixed and 54 revolute;
- 62 collision mesh references and 62 visual mesh references;
- the same manufacturer STL for visual and collision on all 62 meshed links;
- approximately 754,533 collision-source triangles;
- the complete Tianji base and bilateral arm assembly;
- `JointWuji2_L`, `JointWuji2_R`, `l_wrist`, `r_wrist`, and both five-finger Wuji2 chains.

The current SPD-VR model builder is hybrid: it starts from a Tianji MJCF, appends standalone Wuji2 MJCF files, and uses the integrated URDF only for mount transforms and limits. The current live runtime imports `rclpy` through `LiveInputMailbox`, has no Zenoh input, and does not launch a unified MuJoCo operator window. The current live startup script still references old ROS processes and an undefined `optical_inner` command. These paths are not the target architecture.

## Scope

### In scope

- A deterministic URDF-to-MuJoCo compiler.
- Manufacturer STL visual geometry without simplification.
- Multi-convex collision geometry derived from the URDF collision meshes.
- A full 54-DoF simulated plant and a 14-DoF arm IK projection generated from the same URDF.
- A PXREARobotSDK-to-Zenoh bridge for existing PICO custom tracking frames.
- A Zenoh-connected Tianji QP IK process using URDF-derived wrist sites.
- A Python MuJoCo viewer/plant that drives arms and both Wuji2 hands.
- Per-side neutral-pose alignment, validity, stale HOLD, and solver HOLD.
- PC keyboard start/pause, re-alignment, reset, and exit.
- Pixi-managed build, launch, status, stop, and verification commands.
- Mock and real-PICO end-to-end validation.

### Out of scope

- Physical Tianji or Wuji2 actuator output.
- ROS, DDS, ROS messages, or ROS launch files in the new live path.
- PICO-headset rendering or stereo scene return to the Android/XR app.
- Task objects, scoring, recording workflows, or the six-scene/17-task registry in the first delivery.
- Optical-hand gesture session controls.
- Silent primitive collision fallback.
- Keeping the old hybrid generated model as a compatibility alias.

Legacy ROS tools outside the new SPD-VR live entry point remain untouched.

## Architecture

The compiler produces two MuJoCo models from one URDF:

```text
assets/tianji_wuji2/tianji_wuji2.urdf
        |
        +-- unified_plant.xml   54-DoF dual arms + bilateral hands
        `-- arm_ik.xml          14-DoF arm projection + wrist target sites
```

The live path uses three application processes and no separate Zenoh router:

```text
PICO XR App
    |
    | ADB reverse + RoboticsService + PXREARobotSDK custom bytes
    v
pico_zenoh_bridge (C++)
    |  spd/vr/v1/tracking
    +--------------------------+
    v                          v
tianji_zenoh_ik (C++)     spd_vr_viewer (Python)
    ^                          |
    | spd/vr/v1/control        | Wuji retarget + single MuJoCo owner
    |                          |
    +--------------------------+
    | spd/vr/v1/arm_targets    v
    +-------------------- unified MjModel/MjData
```

`pico_zenoh_bridge` listens as a Zenoh peer at `tcp/127.0.0.1:7447`. The IK and viewer processes explicitly connect to that endpoint. A missing peer produces HOLD, not queued replay. No `zenohd`, ROS daemon, or DDS discovery service is required.

## URDF Compiler

### Source authority

The integrated URDF is authoritative for:

- link and joint topology;
- fixed mount chains;
- link inertial values;
- joint axes, origins, limits, and dynamics;
- visual and collision mesh identity and transforms;
- left/right wrist body frames;
- arm and hand joint ordering.

Existing Tianji and Wuji2 MJCF files are not structural inputs. MJCF-only data is limited to generated actuator, solver, contact, site, lighting, ground, and camera settings owned by the compiler.

### Validation before generation

Compilation fails before writing final artifacts when any of the following is true:

- the URDF has zero or multiple roots;
- link or joint names are duplicated;
- a parent/child reference is missing or the graph is disconnected;
- a mesh file is missing or has a non-finite transform/scale;
- mass is non-positive, or an inertia is non-finite/non-physical outside the narrow source-derived fixed-link policy below;
- a revolute joint has missing, non-finite, or inverted limits;
- the required `l_wrist` or `r_wrist` chain is absent;
- the revolute counts are not 54 overall and 14 for the arm projection.

The authoritative file contains 14 geometry-free fixed frames with no `<inertial>`, which remain massless frames, and two fixed TCP links with positive `0.05 kg` mass but an exact zero inertia tensor. The compiler interprets only `TCP_Link_L/R` as source-declared fixed point masses: it transforms each mass through its fixed joint, combines it into the direct parent `Link7_L/R` mass, center of mass, and tensor with the parallel-axis theorem, revalidates the combined inertia, and emits no separate TCP inertial. The manifest records the original mass, transform, destination link, and combined result. A missing inertial is accepted only on a fixed link with no visual/collision geometry. Every other missing or non-physical inertia is a build failure. This preserves the exact URDF mass and transform while avoiding invented epsilon or geometry-derived inertias.

### Visual geometry

Visual geoms use the manufacturer STL files exactly as referenced by the URDF. They are not decimated or converted to primitives. Generated MJCF mesh paths are relative and portable within the workspace output layout.

The URDF also contains 24 named `*_axis_[0-2]` cylinder visuals used as coordinate debug markers. They are not manufacturer meshes and are omitted from the operator scene to avoid axis clutter; the manifest records every omission. Any other primitive visual and every primitive collision geometry is rejected rather than silently substituted.

### Collision geometry

MuJoCo mesh collision uses a convex hull, so passing a non-convex manufacturer mesh directly would fill concavities such as finger gaps. Collision geometry is therefore generated from the same URDF mesh with deterministic CoACD multi-convex decomposition:

- fixed random seed;
- at most 16 convex pieces per link;
- at most 64 hull vertices per piece;
- cache key includes STL SHA-256, URDF scale, CoACD version, and all decomposition parameters;
- cache writes use a temporary directory followed by atomic rename;
- a failed decomposition never falls back to a box, capsule, or one-hull approximation;
- adjacent parent-child links are excluded from self-collision;
- non-adjacent arm, cross-arm, palm, and finger pairs remain eligible for collision.

Every output piece must be finite, non-empty, manifold enough for MuJoCo compilation, and have positive volume. Deterministic surface sampling checks the generated union against the source mesh. Bilateral hand links require p95 bidirectional surface distance at or below 1.5 mm; arm/base links require at or below 3 mm. Exceeding the limit is a build failure.

### Generated artifacts

The clean-cutover output is:

```text
PICO_tracker/src/spd_vr/generated/
├── unified_plant.xml
├── arm_ik.xml
├── model_manifest.yaml
├── collision_manifest.yaml
└── actuator_calibration.yaml
```

`unified_plant.xml` has 54 revolute DoFs and all fixed links required for appearance and collision. `arm_ik.xml` is a projection of the same source graph with 14 revolute arm DoFs and fixed chains through `l_wrist` and `r_wrist`. It includes sites named `l_wrist_target` and `r_wrist_target` at the corresponding URDF wrist body origins.

The manifests record compiler version, URDF hash, every STL hash, output hashes, joint/link maps, source transforms, decomposition parameters and metrics, actuator mapping, wrist sites, and model dimensions. Both runtime processes reject a source or output hash mismatch.

The previous `tianji_wuji2_spd.xml` hybrid artifact is removed after every consumer and test migrates.

## Zenoh Tracking Contract

### Source decoding

The bridge links the installed `PXREARobotSDK.h` and `libPXREARobotSDK.so`. It receives `PXREADeviceCustomMessage` bytes and reuses the existing PICO frame IDs:

- `0x05`: optional head pose;
- `0x06`: world reset;
- `0x38`: left optical hand;
- `0x39`: right optical hand.

Each hand payload remains `active:uint8`, `scale:float32`, and 26 OpenXR-ordered `xyz + quaternion_xyzw` poses. The bridge pairs left and right only when source timestamp and tracking epoch match. A newer timestamp invalidates an incomplete older pair. Device reconnect and world reset increment the epoch and clear all pending state.

### Atomic tracking frame

`spd/vr/v1/tracking` uses a fixed 1,540-byte little-endian binary frame:

```text
uint32 magic                 // "SVT1"
uint16 version               // 1
uint16 flags                 // left_active, right_active, head_valid
uint32 payload_size          // 1540
uint32 crc32                 // bytes [16, 1540)
uint64 sequence
uint64 tracking_epoch
int64  source_timestamp_ns
int64  bridge_monotonic_ns
float32 left_scale
float32 right_scale
float32 head_pose[7]         // xyz + quaternion_xyzw; zero when invalid
float32 left_hand[26][7]
float32 right_hand[26][7]
```

The decoder rejects wrong magic, version, size, CRC, non-finite values, non-positive epoch, non-monotonic sequence/timestamp, non-positive scale, or non-unit quaternions outside the normalization tolerance. Head validity never gates a valid paired hand frame.

Zenoh tracking subscribers use latest-only semantics. Old frames are not replayed after reconnect.

## Arm Target and Control Contracts

`spd/vr/v1/arm_targets` carries the existing 272-byte arm-target v2 schema. Its semantic fields remain:

- sequence, epoch, source and send timestamps;
- left and right q/qdot vectors;
- independent left/right validity;
- independent HOLD reason;
- reserved-byte validation.

One canonical C++ implementation and one canonical Python implementation share golden byte vectors. UDP-specific wrappers are removed from the new live path after all users migrate.

`spd/vr/v1/control` is a small versioned binary command frame with sequence, monotonic timestamp, command enum, and CRC. Supported commands are START, PAUSE, RESUME, REALIGN, RESET, and SHUTDOWN. Control publication is reliable and ordered; duplicate sequence IDs are idempotent.

Status is diagnostic JSON under:

```text
spd/vr/v1/status/bridge
spd/vr/v1/status/ik
spd/vr/v1/status/viewer
```

Status JSON is never consumed as a control input.

## Runtime Components

### `pico_zenoh_bridge`

Responsibilities:

- initialize/deinitialize PXREARobotSDK exactly once;
- select one online device or require an explicit serial when ambiguous;
- decode, validate, pair, and publish atomic tracking frames;
- reserve monotonic sequence and tracking epoch state;
- expose callback rate, pair drops, invalid frames, SDK state, Zenoh state, and source latency;
- publish no arm, hand actuator, or physical hardware command.

The SDK callback copies the SDK-owned bytes into a bounded queue and returns immediately. Parsing and Zenoh publication happen outside the callback. Queue overflow drops the oldest tracking candidate and increments a counter.

### `tianji_zenoh_ik`

Responsibilities:

- load and verify `arm_ik.xml` plus manifest;
- subscribe to tracking and control;
- derive each Wrist from optical hand joint index 1;
- maintain independent left/right neutral alignment and validity;
- run the existing Tianji QP IK at 200 Hz;
- publish independent arm targets and HOLD reasons;
- never instantiate a GUI or physical device output.

### `spd_vr_viewer`

Responsibilities:

- load and verify `unified_plant.xml` plus manifests;
- own the only live `MjModel`/`MjData` pair;
- subscribe to tracking, arm targets, and control acknowledgements;
- run Wuji retarget once for each new hand frame;
- apply latest accepted targets only at 480 Hz physics tick boundaries;
- render the PC operator window at 60 Hz without making rendering the physics clock;
- show the ground plane and no task objects in the first delivery;
- expose keyboard controls and an operator HUD;
- never send real hardware commands.

## Alignment and Control State

Each side has an independent state:

```text
DISCONNECTED
  -> WAITING_INPUT
  -> STABILIZING
  -> ALIGNED
  -> HOLD_STALE | HOLD_INACTIVE | HOLD_SOLVER
  -> ALIGNED
```

STABILIZING requires 10 consecutive accepted Wrist frames. The window rejects a frame when translation changes by more than 0.02 m or orientation changes by more than 0.15 rad from the preceding accepted sample.

At successful neutral alignment for side `s`:

```text
T_robot_from_pico,s = T_urdf_wrist,s,neutral * inverse(T_pico_wrist,s,neutral)
```

The default position scale is 1.0 and is recorded in status and run metadata. Left and right alignment are independent. One side can be ALIGNED while the other is inactive or stabilizing.

A tracking epoch change, source timestamp rollback, explicit REALIGN, or RESET invalidates both alignments. A side-specific active loss invalidates only that side's live target and keeps its last accepted actuator target.

Tracking and arm targets become stale after 50 ms without a newly accepted frame. Stale, inactive, malformed, or solver-failed input produces `valid=false` with a specific HOLD reason. HOLD retains the last valid target; it never jumps to zero, neutral, or an unverified new pose.

Wuji retarget output is reordered by the generated manifest and clamped to URDF limits. Failure or inactivity on one hand does not block the opposite hand.

## Keyboard and Viewer Behavior

The PC MuJoCo viewer owns session controls:

- Space: START, PAUSE, or RESUME;
- R: REALIGN both sides;
- N: RESET q, qvel, ctrl, simulation time, and both alignments;
- Q or Escape: SHUTDOWN.

While paused, physics, retarget, QP publication, recording hooks, and target application are frozen. Health/status reception may continue. Resume requires a fresh alignment window; stale pre-pause input is never applied.

The HUD shows:

- SDK device and Zenoh connection state;
- tracking and arm-target rates;
- source-to-viewer and bridge-to-viewer latency;
- sequence drops and invalid-frame counters;
- left/right alignment state and HOLD reason;
- physics step p95/max;
- active contacts and collision warnings;
- source and generated model hash status.

## Startup and Shutdown

Pixi tasks are the supported operator interface:

```bash
pixi run spd-model
pixi run pico-adb -- --offline
pixi run spd-teleop
pixi run spd-teleop-status
pixi run spd-teleop-stop
```

`spd-model` compiles and validates both models. `spd-teleop` verifies that:

- `adb.sh status` reports the expected reverse;
- RoboticsService and the selected PICO are available;
- the official SDK header/library load;
- CoACD, MuJoCo, Zenoh, and display dependencies are present;
- source and generated hashes match;
- `127.0.0.1:7447` is free;
- no existing SPD-VR session is active.

The startup script creates exactly three tmux windows:

```text
pico_zenoh_bridge
tianji_zenoh_ik
spd_vr_viewer
```

The clean-cutover script does not start or mention old PICO driver, M0, optical/SMPL ROS bridges, Manus, DDS, or ROS environment variables.

Shutdown first publishes SHUTDOWN, then waits for viewer, IK, and bridge in that order. It escalates only after a bounded graceful timeout and validates process identity before signaling. It does not stop an ADB supervisor it did not start.

## Failure Handling

- SDK disconnect: bridge remains alive and publishes disconnected status; downstream enters HOLD.
- Zenoh disconnect: latest control targets are invalidated immediately; reconnect requires fresh sequence and alignment.
- One hand inactive: only that side enters HOLD_INACTIVE.
- Tracking/target stale: corresponding side enters HOLD_STALE after 50 ms.
- QP failure: only the failed side enters HOLD_SOLVER.
- Epoch/reset: pending pairs, latest frames, and both alignments are cleared.
- Model/manifest/hash mismatch: process refuses startup before opening the viewer or SDK.
- Collision compilation failure or quality threshold failure: `spd-model` fails; no primitive fallback.
- Viewer close or Q/Escape: orderly control shutdown; no physical output exists to drain.

## Dependencies

Pixi owns all new open-source dependencies. Expected additions include:

- MuJoCo Python/C++ runtime already used by the repository;
- Python `eclipse-zenoh`;
- Zenoh Pico C/C++ library and CMake target;
- CoACD Python bindings;
- mesh parsing and deterministic surface-sampling support;
- existing `wuji-retargeting` editable package.

The official SDK remains an external system dependency. Its root is configurable through `PXREA_SDK_ROOT`, with `/opt/apps/roboticsservice/SDK` as the discovered workstation default. Build and runtime diagnostics print the exact header and shared library selected.

## Verification

### URDF/compiler tests

- full topology, unique names, one root, connected graph;
- 54 unified and 14 arm revolute DoFs;
- fixed-chain wrist transforms;
- visual mesh identity and relative-path portability;
- limits, inertials, actuator mapping, and source hashes;
- deterministic collision cache and output hashes;
- collision p95 surface-distance gates;
- both MJCF files load and expose required joint/site names;
- source edits invalidate generated manifests.

### Cross-language protocol tests

- C++ encode to Python decode and Python encode to C++ decode;
- checked-in golden vectors for tracking, arm target, and control;
- wrong magic/version/size/CRC;
- NaN/Inf, invalid quaternion, timestamp rollback, epoch transition;
- left/right timestamp mismatch and incomplete-pair replacement;
- duplicate control sequence idempotence.

### State and control tests

- 10-frame independent alignment;
- left-only/right-only active and movement;
- translation/orientation jump rejection;
- stale/inactive/solver HOLD without zero jump;
- pause freezes physics and target publication;
- reset clears q/qvel/ctrl/time/alignment;
- epoch transition rejects old data;
- latest-only behavior under publisher overload.

### Hardware-free integration

A fake PXREA callback emits at least 12 paired frames through real Zenoh sessions. Tests verify:

- left Wrist movement changes only left arm target;
- right Wrist rotation changes only right arm target;
- finger motion changes only the corresponding Wuji2 joints;
- all 54 plant positions, velocities, and controls remain finite;
- viewer headless/offscreen mode renders the manufacturer meshes;
- process shutdown leaves no Zenoh peer, tmux session, or child process.

### Performance and collision

- physics runs at 480 Hz with step-time p95 below 2.083 ms on the target workstation;
- operator rendering runs at 60 Hz without controlling the physics clock;
- collision proxies compile deterministically and remain under the approved geometric-error gates;
- ground, cross-arm, palm, and finger contacts remain finite and do not produce explosive energy or NaNs;
- tracking overload causes bounded drops, not unbounded memory growth.

### Real PICO acceptance

With RoboticsService, ADB reverse, and the XR app running:

- the bridge receives continuous paired optical hands;
- 10 stable frames align both wrists without a target jump;
- left/right wrist motion drives the matching simulated arm only;
- each five-finger hand follows optical articulation through Wuji retarget;
- occluding one hand holds only that side;
- Space freezes and resume requires fresh alignment;
- R realigns, N resets, and Q exits cleanly;
- no ROS process starts and no new live module imports ROS;
- no physical Tianji/Wuji2 command is emitted.

## Clean Cutover

Implementation migrates every new SPD-VR live caller to the generated manifests and Zenoh contracts. It removes obsolete hybrid model consumers, UDP-only live arm wiring, and old six-window live startup commands. It does not add compatibility aliases or a second live protocol. Legacy ROS applications outside the new `spd-teleop` path remain available but are not dependencies of this feature.
