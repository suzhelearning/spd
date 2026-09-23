"""Physics-thread collection lifecycle; disk lifecycle operations run off-thread."""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from datetime import date
import json
import time
from pathlib import Path
from typing import Any, Callable
from uuid import uuid4

import numpy as np

from data_collector.config import CollectionConfig, PHYSICS_HZ
from data_collector.recorder import EpisodeRecorder
from data_collector.trajectory import TrajectorySource
from interfaces.ros_joint_command import MAX_AGE_NS


class CollectionSession:
    """One recorder shared by local controls and ROS; call only on physics thread."""

    def __init__(
        self, config: CollectionConfig, plant: Any, executor: Any,
        task_manifest: dict,
    ) -> None:
        if plant.physics_hz != PHYSICS_HZ:
            raise ValueError(f"collection requires {PHYSICS_HZ} Hz physics")
        self.config = config
        self.plant = plant
        self.executor = executor
        self.task_manifest = {**task_manifest, "collection_config": config.as_dict()}
        self.source = TrajectorySource(plant, self.task_manifest)
        self.recorder = EpisodeRecorder(config.data_dir, queue_size=config.writer_queue_size)
        self.collector_id = uuid4().hex
        self.operation_id = ""
        self.operation = ""
        self.state = "idle"
        self.message = "Ready; explicit local control enable is required before start"
        self.error = ""
        self.episode_path = ""
        self.last_saved_path = ""
        self.completed_episodes = 0
        self.state_frames = 0
        self.on_transition: Callable[[], None] | None = None
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="spd-record-control")
        self._job: Future | None = None
        self._started_ns = 0
        self._ended_ns = 0
        self._first_tick: int | None = None
        self._last_tick: int | None = None
        self._closed = False
        self._physics_paused = False
        self.control_paused = False  # Unified control gate, independent of episode lifecycle.
        self._checkpoint: dict[str, Any] | None = None
        self._skip_confirmation = False

    @property
    def physics_paused(self) -> bool:
        """Freeze integration and command application, not ROS or operator controls."""
        return self._physics_paused or self.control_paused

    @property
    def checkpoint_targets(self) -> np.ndarray | None:
        """Immutable canonical-54 retained targets, detached from checkpoint state."""
        return self._checkpoint["targets"].view() if self._checkpoint is not None else None

    def _capture_checkpoint(self) -> dict[str, Any]:
        snapshot = self.plant.capture_checkpoint()
        return {
            "plant": snapshot,
            # A bytes-backed view cannot be made writable by a ghost consumer.
            "targets": np.frombuffer(snapshot.targets.tobytes(), dtype=snapshot.targets.dtype),
            "contacts": self.source.capture_contact_state(),
            "frames": self.state_frames,
            "first_tick": self._first_tick, "last_tick": self._last_tick,
        }

    def snapshot(self) -> dict:
        elapsed = ((self._ended_ns or time.monotonic_ns()) - self._started_ns) * 1e-9 if self._started_ns else 0.0
        return {
            "collector_id": self.collector_id, "operation_id": self.operation_id,
            "operation": self.operation, "state": self.state, "message": self.message,
            "error": self.error, "episode_path": self.episode_path,
            "last_saved_path": self.last_saved_path, "state_frames": self.state_frames,
            "elapsed_s": max(0.0, elapsed),
            "max_frames": self.config.max_frames,
            "physics_paused": self.physics_paused,
            "checkpoint_frames": self._checkpoint["frames"] if self._checkpoint else None,
            "skip_confirmation": self._skip_confirmation,
        }

    def replace_scene(self, plant: Any, executor: Any, task_manifest: dict) -> None:
        """Rebind only after a completed episode; never migrate checkpoints."""
        if self._closed or self.state != "idle" or self._job is not None or self.recorder.is_busy:
            raise RuntimeError("scene replacement requires an idle, closed episode")
        if plant.physics_hz != PHYSICS_HZ:
            raise ValueError(f"collection requires {PHYSICS_HZ} Hz physics")
        manifest = {**task_manifest, "collection_config": self.config.as_dict()}
        source = TrajectorySource(plant, manifest)
        self.plant, self.executor = plant, executor
        self.task_manifest, self.source = manifest, source
        self._checkpoint = None
        self.cancel_skip_confirmation()

    def _transition(self, state: str, message: str) -> None:
        self.cancel_skip_confirmation()
        self.state, self.message = state, message
        print("SPD collection: " + json.dumps(self.snapshot(), ensure_ascii=False), flush=True)
        if self.on_transition is not None:
            self.on_transition()

    def _start_rejection(self) -> str:
        mailbox = self.executor.mailbox
        if not mailbox.enabled:
            return "Enable aligned external control locally before recording"
        candidate = mailbox.latest
        if candidate is None or not (candidate.ready_mask & ~self.executor.hold_mask):
            return "No currently ready, unheld command group"
        if time.time_ns() - candidate.stamp_ns > MAX_AGE_NS:
            return "External command is stale; no currently ready command group"
        return ""

    def cancel_skip_confirmation(self) -> None:
        """Any other operator action or lifecycle transition cancels pending skip."""
        if self._skip_confirmation:
            self.message = "Skip confirmation cancelled"
        self._skip_confirmation = False

    def request_local(self, operation: str) -> tuple[bool, dict]:
        """Local pause pedal is explicit authorization; ROS requests never use this gate."""
        if operation == "revert_skip":
            if self._closed or self._job is not None or self.state not in {"recording", "paused"}:
                return self.request("revert")
            if self._checkpoint is not None:
                return self.request("revert")
            if self._skip_confirmation:
                return self.request("skip")
            self._skip_confirmation = True
            self.message = "No checkpoint: press d again to confirm skipping this episode; another control cancels"
            if self.on_transition is not None:
                self.on_transition()
            return False, {"collector_id": self.collector_id, "operation_id": "", "message": self.message}
        self.cancel_skip_confirmation()
        if operation != "pause_toggle":
            return self.request(operation)
        if self.state != "paused":
            return self.request("pause")
        if self._closed or self._job is not None:
            return self.request("resume")
        # Recheck even if somebody enabled while paused: the latest candidate
        # must still align with the retained/restored targets at this press.
        if not self.executor.authorize(True):
            reason = self.executor.mailbox.last_reject_reason or "No fresh aligned command candidate"
            return False, {"collector_id": self.collector_id, "operation_id": "", "message": reason}
        accepted, payload = self.request("resume")
        if not accepted or self.state != "recording":
            self.executor.authorize(False)
        return accepted, payload

    def request(self, operation: str) -> tuple[bool, dict]:
        """Accept on the physics thread; disk mutations run on the control worker."""
        self.cancel_skip_confirmation()
        reason = ""
        operations = {"start", "save", "discard", "checkpoint", "pause", "resume", "revert", "skip"}
        if self._closed:
            reason = "Collector is shutting down"
        elif operation not in operations:
            reason = f"Unknown collection operation: {operation}"
        elif self._job is not None:
            reason = f"Collector busy: {self.state}"
        elif operation == "start":
            if self.state not in {"idle", "error"}:
                reason = "An episode is already open"
            elif self.recorder.is_busy:
                reason = "Previous episode could not be closed; restart the collector"
            else:
                reason = self._start_rejection()
        elif self.state not in {"recording", "paused"}:
            reason = "No episode is open"
        elif operation == "pause" and self.state != "recording":
            reason = "Episode is already paused"
        elif operation == "resume":
            reason = "Episode is not paused" if self.state != "paused" else self._start_rejection()
        elif operation == "revert" and self._checkpoint is None:
            reason = "No checkpoint in this episode"
        elif operation == "checkpoint":
            if self.source.has_hand_object_contact():
                reason = "Checkpoint rejected: a hand is in contact with a task object"
        elif operation == "save":
            if self.state == "recording" and not self.executor.mailbox.enabled:
                reason = "Control disabled; episode must be preserved as partial, not saved"
            elif not self.state_frames:
                reason = "Waiting for the first actual whole-scene trajectory sample"
        if reason:
            return False, {"collector_id": self.collector_id, "operation_id": "", "message": reason}

        self.operation_id = uuid4().hex
        self.operation = operation
        try:
            if operation == "start":
                episode_id = uuid4().hex
                episode_dir = self.config.data_dir / date.today().strftime("%Y%m%d")
                self.episode_path = str(episode_dir / f"episode_{episode_id}.partial.h5")
                self._started_ns = self._ended_ns = 0
                self.state_frames = 0
                self._first_tick = self._last_tick = None
                self._physics_paused = True
                self.source.reset_contacts()
                # Capture before the first motion or asynchronous disk preparation.
                # Automatic checkpoint zero intentionally bypasses the contact gate.
                self._checkpoint = self._capture_checkpoint()
                self.error = ""
                self._job = self._worker.submit(
                    self.recorder.start_episode, episode_id, self.task_manifest,
                    model_bytes=self.source.model_bytes, metadata=self.source.metadata,
                    output_dir=episode_dir,
                )
                self._transition("preparing", "Opening episode")
            elif operation == "checkpoint":
                self._checkpoint = self._capture_checkpoint()
                self._transition(self.state, f"Checkpoint completed at {self.state_frames} frames")
            elif operation == "pause":
                self._physics_paused = True
                self.executor.clear()
                self._transition("paused", "Physics and recording paused; local s requests one-second recovery")
            elif operation == "resume":
                self._physics_paused = False
                self._transition("recording", "Resume completed")
            elif operation == "revert":
                self._physics_paused = True
                self.executor.clear()
                self._job = self._worker.submit(self.recorder.truncate_frames, self._checkpoint["frames"])
                self._transition("reverting", "Removing failed suffix; physics and command application frozen")
            elif operation == "save":
                self._finish(success=True)
            else:
                self._physics_paused = True
                self.executor.clear()
                self._ended_ns = time.monotonic_ns()
                self._job = self._worker.submit(self.recorder.discard_episode)
                self._transition("discarding", "Skipping current episode" if operation == "skip" else "Discarding episode")
        except Exception as exc:
            self._abort(str(exc))
        return True, {"collector_id": self.collector_id, "operation_id": self.operation_id,
                      "message": f"{operation} accepted; completion is reported on status"}

    def _finish(self, *, success: bool) -> None:
        if self.physics_paused:
            self.executor.clear()
        self._ended_ns = time.monotonic_ns()
        self._job = self._worker.submit(self.recorder.finish_episode, success=success)
        self._transition("saving", "Saving explicitly successful episode" if success else
                         "State sample limit reached; saving success=False")

    def _abort(self, reason: str) -> None:
        self.error = reason
        self._ended_ns = time.monotonic_ns()
        self._job = self._worker.submit(self._preserve_partial, reason, self.episode_path)
        self._transition("aborting", f"{reason}; preserving partial episode")

    def _preserve_partial(self, reason: str, path: str) -> str:
        self.recorder.abort_episode(reason)
        return path if path and Path(path).is_file() else ""

    def poll(self) -> None:
        """Complete finished disk work without waiting on the physics thread."""
        if self._job is not None and self._job.done():
            previous = self.state
            job, self._job = self._job, None
            try:
                result = job.result()
                if previous == "preparing":
                    reason = "Collector shutdown" if self._closed else self._start_rejection()
                    if reason:
                        self._abort(reason)
                        return
                    self._started_ns = time.monotonic_ns()
                    self._physics_paused = False
                    self._transition("recording", "Ready to sample whole-scene physical trajectories")
                elif previous == "saving":
                    self.episode_path = self.last_saved_path = str(result)
                    self._checkpoint = None
                    self._physics_paused = False
                    self.completed_episodes += 1
                    self._transition("idle", f"Saved episode: {result}")
                elif previous == "discarding":
                    self.episode_path = ""
                    self._checkpoint = None
                    self._physics_paused = False
                    self.completed_episodes += 1
                    self._transition("idle", "Episode discarded")
                elif previous == "reverting":
                    checkpoint = self._checkpoint
                    self.plant.restore_checkpoint(checkpoint["plant"])
                    self.source.restore_contact_state(checkpoint["contacts"])
                    self.state_frames = checkpoint["frames"]
                    self._first_tick = checkpoint["first_tick"]
                    self._last_tick = checkpoint["last_tick"]
                    self.executor.clear()
                    self._transition("paused", "Checkpoint restored; coordinator may begin automatic recovery")
                elif previous == "aborting":
                    self.episode_path = result
                    self._transition("error", f"{self.error}; partial episode preserved")
            except Exception as exc:
                if previous == "aborting":
                    self.error = f"{self.error}; abort failed: {exc}"
                    self._transition("error", self.error)
                else:
                    self._abort(str(exc))
        if self.state in {"recording", "paused"}:
            if self.recorder.error is not None:
                self._abort(str(self.recorder.error))
            elif self.state == "recording" and not self.executor.mailbox.enabled:
                self._abort("Control disabled")

    def tick(self, step: Any, *, recovery: int = 0) -> None:
        """Record actual state; recovery labels are 0=normal, 1=start, 2=resume, 3=rewind."""
        if self.state != "recording":
            return
        try:
            if not step.finite:
                raise ValueError("non-finite simulation state")
            if self._last_tick is not None and step.tick != self._last_tick + 1:
                raise ValueError("missing or repeated physics tick during trajectory collection")
            self._last_tick = step.tick
            self.source.observe_contacts()
            if self._first_tick is None:
                self._first_tick = step.tick
            offset = step.tick - self._first_tick
            if offset % (PHYSICS_HZ // self.config.state_rate_hz) == 0:
                self.recorder.append_frame(self.source.capture(step.tick, time.monotonic_ns()), recovery=recovery)
                self.state_frames += 1
            if offset == 0:
                self._transition("recording", "Recording whole-scene state and hand-object contacts")
            if self.config.max_frames and self.state_frames >= self.config.max_frames:
                self._finish(success=False)
        except Exception as exc:
            self._abort(str(exc))

    def close(self) -> None:
        """Wait pending work, then abort any still-open episode without success."""
        self._closed = True
        if self._job is not None:
            try:
                self._job.result()
            except Exception:
                pass  # poll records the operation error and schedules partial preservation.
            self.poll()
        if self.state in {"recording", "paused"}:
            self._abort("Collector shutdown")
        if self._job is not None:
            try:
                self._job.result()
            except Exception:
                pass
            self.poll()
        self._worker.shutdown(wait=True)
