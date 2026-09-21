# tianji_spd_interfaces

ROS 2 wire interface for the Tianji teleop → SPD simulation boundary.

`JointCommand` is an absolute 54-DoF position target in radians. The fixed wire
order is left arm 7, right arm 7, left hand 20, right hand 20. The publisher
must send the complete name and position arrays on every frame.

The publisher is maintained in `tianji_teleop-ros2`; this package contains only
the matching message definition needed to build the subscriber's local overlay.
It contains no publisher, device input, IK, or retargeting implementation.
The definition must remain byte-for-byte identical to the upstream
`src/interfaces/tianji_spd_interfaces/msg/JointCommand.msg`.

Topic: `/spd/tianji_wuji2/v1/joint_command`; `schema_version=1`,
`robot_config="tianji_wuji2_v1"`. QoS is BEST_EFFORT / KEEP_LAST(1) / VOLATILE.
Timestamps are command-generation UTC, not receipt time. The subscriber checks
freshness again when applying a target and requires explicit local enable.

The finalized upstream entry is `bash bash/run_pico_hand_sim.sh --height-m 1.75`
in the teleoperation workspace (use the operator's measured height).
It publishes at most 60 Hz on Fast DDS, domain 120, localhost by default.
Its `--headless` option disables only its auxiliary viewer; publishing continues.
Calibration, following, stale input, braking and Home trajectories belong to
that publisher. None of those source transitions grants SPD local authorization.
