"""Physics-thread collection lifecycle; disk lifecycle operations run off-thread."""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from datetime import date
import json
import time
from pathlib import Path
from typing import Any, Callable
from uuid import uuid4

from spd_vr.cameras.camera import CAMERA_NAMES
from spd_vr.data_collector.config import CollectionConfig, PHYSICS_HZ
from spd_vr.data_collector.recorder import EpisodeRecorder
from spd_vr.interfaces.ros_joint_command import MAX_AGE_NS


class CollectionSession:
    """One recorder shared by local controls and ROS; call only on physics thread."""

    def __init__(
        self, config: CollectionConfig, plant: Any, executor: Any,
        task_manifest: dict, camera_factory: Callable[[], Any],
    ) -> None:
        if plant.physics_hz != PHYSICS_HZ:
            raise ValueError(f"collection requires {PHYSICS_HZ} Hz physics")
        self.config = config
        self.plant = plant
        self.executor = executor
        self.task_manifest = {**task_manifest, "collection_config": config.as_dict()}
        self._camera_factory = camera_factory
        self.camera = None
        self.recorder = EpisodeRecorder(config.data_dir, camera_names=CAMERA_NAMES,
                                        queue_size=config.writer_queue_size)
        self.collector_id = uuid4().hex
        self.operation_id = ""
        self.operation = ""
        self.state = "idle"
        self.message = "Ready; explicit local control enable is required before start"
        self.error = ""
        self.episode_path = ""
        self.last_saved_path = ""
        self.state_frames = 0
        self.on_transition: Callable[[], None] | None = None
        self.camera_frames = dict.fromkeys(CAMERA_NAMES, 0)
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="spd-record-control")
        self._job: Future | None = None
        self._started_ns = 0
        self._ended_ns = 0
        self._first_tick: int | None = None
        self._closed = False

    def snapshot(self) -> dict:
        elapsed = ((self._ended_ns or time.monotonic_ns()) - self._started_ns) * 1e-9 if self._started_ns else 0.0
        return {
            "collector_id": self.collector_id, "operation_id": self.operation_id,
            "operation": self.operation, "state": self.state, "message": self.message,
            "error": self.error, "episode_path": self.episode_path,
            "last_saved_path": self.last_saved_path, "state_frames": self.state_frames,
            "camera_frames": dict(self.camera_frames), "elapsed_s": max(0.0, elapsed),
            "max_frames": self.config.max_frames,
        }

    def _transition(self, state: str, message: str) -> None:
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

    def request(self, operation: str) -> tuple[bool, dict]:
        """Accept quickly, never wait for disk and never authorize movement."""
        reason = ""
        if self._closed:
            reason = "Collector is shutting down"
        elif operation not in {"start", "save", "discard"}:
            reason = f"Unknown collection operation: {operation}"
        elif self._job is not None or self.state in {"preparing", "saving", "discarding", "aborting"}:
            reason = f"Collector busy: {self.state}"
        elif operation == "start":
            if self.state not in {"idle", "error"}:
                reason = "An episode is already recording"
            elif self.recorder.is_busy:
                reason = "Previous episode could not be closed; restart the collector"
            else:
                reason = self._start_rejection()
        elif self.state != "recording":
            reason = "No episode is recording"
        elif operation == "save" and not self.executor.mailbox.enabled:
            reason = "Control disabled; episode must be preserved as partial, not saved"
        elif operation == "save" and not self.state_frames:
            reason = "Waiting for the first actual simulation state/RGB samples"
        if reason:
            print(f"SPD collection rejected {operation}: {reason}", flush=True)
            return False, {"collector_id": self.collector_id, "operation_id": "", "message": reason}

        self.operation_id = uuid4().hex
        self.operation = operation
        if operation == "start":
            episode_id = uuid4().hex
            episode_dir = self.config.data_dir / date.today().strftime("%Y%m%d")
            self.episode_path = str(episode_dir / f"episode_{episode_id}.partial.h5")
            self._started_ns = self._ended_ns = 0
            self.state_frames = 0
            self.camera_frames = dict.fromkeys(CAMERA_NAMES, 0)
            self._first_tick = None
            self.error = ""
            self._job = self._worker.submit(
                self.recorder.start_episode, episode_id, self.task_manifest,
                output_dir=episode_dir,
            )
            self._transition("preparing", "Opening episode")
        elif operation == "save":
            self._finish(success=True)
        else:
            self._ended_ns = time.monotonic_ns()
            self._job = self._worker.submit(self.recorder.discard_episode)
            self._transition("discarding", "Discarding episode")
        return True, {"collector_id": self.collector_id, "operation_id": self.operation_id,
                      "message": f"{operation} accepted; completion is reported on status"}

    def _finish(self, *, success: bool) -> None:
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
        """Complete finished work without waiting; renderer creation stays here."""
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
                    if self.camera is None:
                        self.camera = self._camera_factory()
                    self._started_ns = time.monotonic_ns()
                    self._transition("recording", "Ready to sample actual simulation state and RGB")
                elif previous == "saving":
                    self.episode_path = self.last_saved_path = str(result)
                    self._transition("idle", f"Saved episode: {result}")
                elif previous == "discarding":
                    self.episode_path = ""
                    self._transition("idle", "Episode discarded")
                elif previous == "aborting":
                    self.episode_path = result
                    self._transition("error", f"{self.error}; partial episode preserved")
            except Exception as exc:
                if previous == "aborting":
                    self.error = f"{self.error}; abort failed: {exc}"
                    self._transition("error", self.error)
                else:
                    self._abort(str(exc))
        if self.state == "recording":
            if self.recorder.error is not None:
                self._abort(str(self.recorder.error))
            elif not self.executor.mailbox.enabled:
                self._abort("Control disabled")

    def tick(self, step: Any) -> None:
        """Sample the current MuJoCo state and rendered RGB, independently of commands."""
        if self.state != "recording":
            return
        try:
            if self._first_tick is None:
                self._first_tick = step.tick
            offset = step.tick - self._first_tick
            if offset % (PHYSICS_HZ // self.config.state_rate_hz) == 0:
                qpos = self.plant.joint_command_positions()
                stamp = time.monotonic_ns()
                self.recorder.append_arm_qpos(stamp, qpos[:14])
                self.recorder.append_hand_qpos(stamp, qpos[14:])
                self.state_frames += 1
            if offset % (PHYSICS_HZ // self.config.camera_rate_hz) == 0:
                frames = self.camera.capture(step.sim_time_ns)
                stamp = time.monotonic_ns()
                for name in CAMERA_NAMES:
                    self.recorder.append_image(name, stamp, frames[name].rgb)
                    self.camera_frames[name] += 1
            if offset == 0:
                self._transition("recording", "Recording actual state and RGB")
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
        if self.state == "recording":
            self._abort("Collector shutdown")
        if self._job is not None:
            try:
                self._job.result()
            except Exception:
                pass
            self.poll()
        self._worker.shutdown(wait=True)
        if self.camera is not None:
            for renderer in self.camera._renderers.values():
                renderer.close()
