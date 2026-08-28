from types import SimpleNamespace

import numpy as np

from spd_vr.episode import EpisodeCommandType, EpisodeController, EpisodeState


class _Task:
    def reset(self, seed):
        return SimpleNamespace(manifest=lambda: {"scene": "test", "seed": seed}, objects=[])


class _Simulator:
    def __init__(self, *, contact=False):
        self.paused = False
        self.contact = contact
        self.data = SimpleNamespace(
            qpos=np.zeros(1), qvel=np.zeros(1), act=None, ctrl=np.zeros(1),
            mocap_pos=np.zeros((0, 3)), mocap_quat=np.zeros((0, 4)), time=0.0,
        )
        self.physics_hz = 480
        self.set_paused_calls = []

    def set_paused(self, value):
        self.paused = bool(value)
        self.set_paused_calls.append(self.paused)

    def has_task_object_contact(self):
        return self.contact


class _Recorder:
    def __init__(self):
        self.manifest = None
        self.finished = 0
        self.discarded = []
    def start_episode(self, episode_id, manifest):
        self.manifest = manifest
    def finish_episode(self):
        self.finished += 1

    def discard_episode(self, reason):
        self.discarded.append(reason)


def test_checkpoint_contact_guard_and_revert_skip_semantics_are_unchanged():
    simulator = _Simulator(contact=True)
    recorder = _Recorder()
    controller = EpisodeController(simulator, _Task(), recorder=recorder)
    controller.enqueue(EpisodeCommandType.START)
    assert controller.process_one() == "started"
    controller.enqueue(EpisodeCommandType.CHECKPOINT)
    assert controller.process_one() == "checkpoint rejected while hand-object contact is active"

    simulator.contact = False
    controller.enqueue(EpisodeCommandType.CHECKPOINT)
    assert controller.process_one() == "checkpointed"
    simulator.data.qpos[0] = 12.0
    controller.enqueue(EpisodeCommandType.REVERT)
    assert controller.process_one() == "reverted"
    assert simulator.data.qpos[0] == 0.0

    controller.enqueue(EpisodeCommandType.SKIP)
    assert controller.process_one() == "skipped"
    assert controller.state is EpisodeState.IDLE
    assert recorder.discarded == ["operator_skip"]


def test_pause_binds_simulator_and_manifest_copies_teleop_metadata():
    simulator = _Simulator()
    recorder = _Recorder()
    metadata = {"wrist_position_scale": 1.25, "nested": {"stable": 10}}
    controller = EpisodeController(simulator, _Task(), recorder=recorder, run_metadata=metadata)
    controller.enqueue(EpisodeCommandType.START)
    assert controller.process_one() == "started"
    metadata["nested"]["stable"] = 99
    assert recorder.manifest["teleop"] == {"wrist_position_scale": 1.25, "nested": {"stable": 10}}

    controller.enqueue(EpisodeCommandType.PAUSE)
    assert controller.process_one() == "paused"
    assert simulator.paused is True
    paused_epoch = controller.state_epoch
    controller.enqueue(EpisodeCommandType.RESUME)
    assert controller.process_one() == "resumed"
    assert simulator.paused is False
    assert controller.state is EpisodeState.RECORDING
    assert controller.state_epoch == paused_epoch + 1
    assert simulator.set_paused_calls == [True, False]
