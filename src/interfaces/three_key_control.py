"""Local recording owner: upstream keeps publishing while SPD pauses/rewinds."""
from __future__ import annotations

import time

from interfaces.ros_joint_command import MAX_AGE_NS


class ThreeKeyControl:
    def __init__(self, collection, executor) -> None:
        self.collection = collection
        self.executor = executor
        self.stage = "idle"
        self.freeze = True
        self.notice = "Upstream r calibrates / s follows; local r starts an episode"
        self._save_confirmation = None
        self._session_id = ""
        self._recovery = 0
        self._rewind_deadline_ns = 0

    @property
    def freeze(self) -> bool:
        return self.collection.control_paused

    @freeze.setter
    def freeze(self, value: bool) -> None:
        self.collection.control_paused = value

    @property
    def recovery(self) -> int:
        """Annotation for the last applied physics tick, including the endpoint."""
        return self._recovery if self.executor.transition_frame else 0

    def _set_stage(self, stage: str, notice: str) -> None:
        if self.stage != stage:
            self._save_confirmation = None
        self.stage, self.notice = stage, notice
        print(f"SPD keys [{stage}]: {notice}", flush=True)

    def cancel_confirmation(self) -> None:
        self._save_confirmation = None

    def _ready_reason(self) -> str:
        candidate = self.executor.mailbox.latest
        if candidate is None:
            return "Waiting for upstream targets: calibrate with r and start with s upstream"
        if not candidate.ready_mask & 1:
            return "Upstream arm targets are not ready"
        if not 0 <= time.time_ns() - candidate.stamp_ns <= MAX_AGE_NS:
            return "Upstream joint candidate is stale"
        return ""

    def _pause(self, reason: str) -> None:
        self.freeze = True
        if self.collection.state == "recording":
            self.collection.request("pause")
        self.executor.clear()
        self._recovery = 0
        self.cancel_confirmation()
        stage = "paused" if self.collection.state == "paused" else self.collection.state
        if stage == "preparing":
            stage = "preparation_failed"  # Wait for the in-flight writer to preserve the partial.
        self._set_stage(stage, reason)

    def _start_episode(self) -> None:
        reason = self._ready_reason()
        if reason:
            self.notice = reason
            return
        self.freeze = True
        if not self.executor.authorize_transition():
            self.notice = self.executor.mailbox.last_reject_reason
            return
        self._session_id = self.executor.mailbox.authorized_session
        accepted, response = self.collection.request("start")
        if not accepted:
            self.executor.clear()
            self.notice = response["message"]
            return
        self._set_stage("preparing", "Opening episode with checkpoint 0; no motion until ready")

    def _begin_recovery(self, kind: int) -> None:
        reason = self._ready_reason()
        if reason or not self.executor.authorize_transition():
            self._pause(reason or self.executor.mailbox.last_reject_reason)
            return
        self._session_id = self.executor.mailbox.authorized_session
        if self.collection.state == "paused":
            accepted, response = self.collection.request("resume")
            if not accepted:
                self._pause(response["message"])
                return
        if self.collection.state != "recording":
            self._pause("Collector is not ready for the transition")
            return
        self._recovery = kind
        self.freeze = False
        self._set_stage("blending", "1 second live-target transition; r/s/d ignored; recovery samples labelled")

    def key(self, key: str) -> None:
        """Consume a local key. Transition keys are dropped, never deferred."""
        key = key.lower()
        if self.stage in {"blending", "preparing", "preparation_failed", "reverting", "rewind_wait", "saving", "aborting"}:
            return
        if key != "r":
            self.cancel_confirmation()
        if key == "r":
            if self.stage == "idle" and self.collection.state == "idle":
                self._start_episode()
            elif self.stage == "recording" and self.collection.state == "recording":
                _, response = self.collection.request("checkpoint")
                self.notice = response["message"]
            elif self.stage == "paused" and self.collection.state == "paused":
                context = (self.collection.operation_id, self.collection.state_frames)
                if self._save_confirmation != context:
                    self._save_confirmation = context
                    self.notice = "Press r again to confirm successful completion; s resumes, d rewinds"
                    return
                self.cancel_confirmation()
                accepted, response = self.collection.request("save")
                self.notice = response["message"]
                if accepted:
                    self.executor.clear()
                    self._set_stage("saving", "Saving successful episode; next task waits for r")
        elif key == "s":
            if self.stage == "recording":
                self._pause("Paused: checkpoint hand ghost shown; s resumes, d rewinds, r then r saves")
            elif self.stage == "paused" and self.collection.state == "paused":
                self._begin_recovery(2)
        elif key == "d":
            if self.stage == "paused" and self.collection.state == "paused":
                accepted, response = self.collection.request("revert")
                self.notice = response["message"]
                if accepted:
                    self.freeze = True
                    self._set_stage("reverting", "Restoring checkpoint; automatic 1 second recovery follows")
        else:
            self.cancel_confirmation()

    def poll(self) -> None:
        if self.stage in {"preparing", "recording", "blending"}:
            candidate = self.executor.mailbox.latest
            reason = self._ready_reason()
            if not self.executor.mailbox.enabled:
                reason = self.executor.mailbox.last_reject_reason or "Motion authorization was revoked"
            elif candidate is not None and candidate.session_id != self._session_id:
                reason = "Upstream session changed"
            elif self.executor.hold_mask & 1:
                reason = reason or "Arm targets are held"
            if reason:
                self._pause(reason + "; explicit s required after recovery")
                return
        if self.stage == "preparing":
            if self.collection.state == "recording":
                self._begin_recovery(1)
            elif self.collection.state != "preparing":
                self._pause(f"Collector is {self.collection.state}")
        elif self.stage == "blending":
            if self.collection.state != "recording":
                self._pause(f"Collector is {self.collection.state}")
            elif not self.executor.transition_active:
                self._recovery = 0
                self._set_stage("recording", "r checkpoint; s pause; d is available only while paused")
        elif self.stage == "recording":
            if self.collection.state != "recording":
                self._pause(f"Collector is {self.collection.state}")
        elif self.stage == "reverting":
            if self.collection.state == "paused":
                # Restore clears the old mailbox. Wait for a genuinely fresh
                # post-restore packet instead of requiring a 60Hz packet on
                # the very next 480Hz physics iteration.
                self._rewind_deadline_ns = time.monotonic_ns() + MAX_AGE_NS
                self._set_stage("rewind_wait", "Checkpoint restored; waiting for fresh target to auto-resume")
            elif self.collection.state != "reverting":
                self._pause(f"Rewind did not complete: {self.collection.message}")
        elif self.stage == "rewind_wait":
            candidate = self.executor.mailbox.latest
            if candidate is not None and candidate.session_id != self._session_id:
                self._pause("Upstream session changed during rewind; explicit s required")
            elif not self._ready_reason():
                self._begin_recovery(3)
            elif time.monotonic_ns() >= self._rewind_deadline_ns:
                self._pause("No fresh target after rewind; explicit s required")
        elif self.stage in {"saving", "aborting", "preparation_failed"}:
            self.freeze = True
            if self.collection.state == "idle":
                self._set_stage("idle", self.collection.message + "; r starts the next episode")
            elif self.collection.state == "error":
                self._set_stage("error", self.collection.message + "; frozen, restart required")
            elif self.collection.state == "aborting" and self.stage != "aborting":
                self._set_stage("aborting", self.collection.message)

    def close(self) -> None:
        self._pause("Local collector stopping; upstream continues independently")
