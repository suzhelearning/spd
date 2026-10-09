"""Single physics-thread authority for input binding, collection and scene lifecycle.

The teleop worker never changes MuJoCo state. Every bind/rewind is issued while
physics is frozen, and generation barriers exclude pre-rewind worker results.
"""
from __future__ import annotations

import time
from uuid import uuid4

from interfaces.ros_joint_command import JointCommandSnapshot, MAX_AGE_NS


class CollectionControl:
    INPUT_TIMEOUT_NS = 120_000_000
    ENTRY_NS = 500_000_000

    def __init__(self, app):
        self.app = app
        self.stage = "idle"
        self.notice = "双手放在腰间准备位；r 开始"
        self.recovery = 0
        self.control_flags = 0
        self._session = uuid4().hex
        self._generation = -1
        self._sequence = -1
        self._kind = 0
        self._entry_until = 0
        self._manual_pause_pending = False
        self._external_session = None
        self.freeze = True

    @property
    def collection(self):
        return self.app.collection

    @property
    def executor(self):
        return self.app.executor

    @property
    def teleop(self):
        return self.app.teleop

    @property
    def freeze(self):
        return self.collection.control_paused

    @freeze.setter
    def freeze(self, value):
        self.collection.control_paused = bool(value)

    def _stage(self, stage, notice):
        changed = stage != self.stage or notice != self.notice
        self.stage, self.notice = stage, notice
        if changed:
            print(f"SPD [{stage}]: {notice}", flush=True)

    def _request(self, operation):
        accepted, payload = self.collection.request(operation)
        if not accepted:
            self.notice = payload["message"]
        return accepted

    def _halt(self):
        self.freeze = True
        self.recovery = 0
        if self.teleop is not None:
            self.teleop.pause()
        if self.collection.state == "recording":
            self._request("pause")
        self.executor.clear()

    def _pause(self, automatic=False):
        self.freeze = True
        if self.collection.state == "preparing":
            # Keep the stationary bound reference alive until asynchronous open
            # completes; then pause before the first physical integration step.
            self._manual_pause_pending = True
            return
        self._manual_pause_pending = False
        if automatic and self.collection.state == "recording":
            self.collection.capture_auto_checkpoint()
        self._halt()
        if self.collection.state not in {"recording", "paused"}:
            return
        if automatic:
            self._stage("auto_paused", "跟踪丢失：保持当前姿态；摆好现实姿态后按 r 重新接手")
        else:
            self._stage("paused", "人工暂停：s 重新绑定继续；r 保存整条并进入下一条；d 丢弃整条并进入下一条")

    def _fail(self, reason):
        self._halt()
        self._stage("error", f"已冻结：{reason}；未完成数据保留，退出检查后重新启动")

    def _external_ready(self):
        candidate = self.executor.mailbox.latest
        if candidate is None or not candidate.ready_mask & 1:
            return False
        age = time.time_ns() - candidate.stamp_ns
        return 0 <= age <= MAX_AGE_NS

    def _begin_bind(self, kind):
        if self.teleop is not None:
            snapshot = self.teleop.snapshot()
            if snapshot.fault:
                self._fail(snapshot.fault)
                return False
            self.freeze = True
            self.executor.clear()
            self._generation = self.teleop.rebind(self.app.plant.joint_command_targets())
            self._sequence = -1
        else:
            self.freeze = True
        self._kind = kind
        self._stage("binding" if kind == 1 else "rebinding", "参考对齐中，机器人保持不动")
        return True

    def _accept_local(self, snapshot, now):
        if snapshot.generation != self._generation or snapshot.sequence <= self._sequence:
            return False
        age = now - snapshot.generated_ns
        if not 0 <= age <= MAX_AGE_NS:
            return False
        if snapshot.mode not in {"bound", "follow"}:
            return False
        # This is the timestamp of a freshly generated bounded reference, NOT a
        # fabricated input timestamp. Original input age is checked separately.
        command = JointCommandSnapshot.from_values(
            session_id=f"{self._session}:{self._generation}", sequence=snapshot.sequence,
            ready_mask=7, position_rad=snapshot.position_rad,
            stamp_ns=time.time_ns() - age,
        )
        if not self.executor.mailbox.receive(command):
            self._fail(self.executor.mailbox.last_reject_reason)
            return False
        self._sequence = snapshot.sequence
        self.control_flags = snapshot.control_flags
        return True

    def _binding_ready(self, snapshot):
        if self.teleop is not None:
            return (snapshot.generation == self._generation and snapshot.mode == "bound"
                    and snapshot.arms_valid and not snapshot.needs_rebind
                    and self.executor.mailbox.latest is not None)
        return self._external_ready()

    def _authorize(self):
        if self.teleop is not None:
            return self.executor.authorize(True)
        # External publishers retain their explicit entry blend. Local binding
        # already starts at retained targets, avoiding a second approach controller.
        return self.executor.authorize_transition()

    def _activate(self, now):
        if not self._authorize():
            self._fail(self.executor.mailbox.last_reject_reason)
            return
        if self.collection.state == "paused" and not self._request("resume"):
            self._fail(self.notice)
            return
        if self.collection.state != "recording":
            self._fail("采集器尚未准备好")
            return
        if self.teleop is not None:
            self.teleop.follow(self._generation)
        else:
            self._external_session = self.executor.mailbox.latest.session_id
        self._entry_until = now + self.ENTRY_NS
        self.recovery = self._kind
        self.freeze = False
        self._stage("recording", "运动录制：s 人工暂停；r 更新检查点；d 回退并自动重新接手")

    def key(self, key):
        key = key.lower()
        if key == "q":
            self._halt()
            self.app.request_stop()
            return
        if key not in {"r", "s", "d"}:
            return
        if self.stage == "idle":
            if key == "r" and self.collection.state == "idle":
                self._begin_bind(1)
            return
        if self.stage == "auto_paused":
            if key == "r":
                ready = self.teleop.snapshot().can_bind if self.teleop is not None else self._external_ready()
                if not ready:
                    self.notice = "跟踪尚未稳定；保持头和双腕可见，摆好姿态后再按 r 重新接手"
                    return
                self._begin_bind(4)
            return
        if self.stage in {"binding", "rebinding", "preparing"}:
            if key == "s":
                if self.collection.state == "preparing":
                    self._manual_pause_pending = True
                elif self.collection.state == "idle":
                    self._halt()
                    self._stage("idle", "开始已取消；r 重新开始")
                else:
                    self._pause(automatic=self._kind == 4)
            return
        # Disk operations never defer ordinary keys into a later state.
        if self.stage not in {"recording", "paused"}:
            return
        if self.stage == "paused":
            if key == "s":
                self._begin_bind(2)
            elif key == "r":
                if self.collection.state_frames == 0:
                    self.notice = "尚无实际采集帧，不能保存成功"
                    return
                self._halt()
                if self._request("save"):
                    self._stage("saving", "关闭并校验示范文件；成功后生成下一条场景，Home 等待 r")
            elif key == "d":
                self._halt()
                if self._request("discard"):
                    self._stage("discarding", "丢弃整条文件；完成后生成下一条场景，Home 等待 r")
            return
        if key == "s":
            self._pause()
        elif key == "r" and self._request("checkpoint"):
            self.notice = f"当前检查点已更新：{self.collection.state_frames} 帧；继续运动录制"
        elif key == "d":
            self._halt()
            if self._request("revert"):
                self._stage("reverting", "恢复检查点并裁掉失败后缀；稳定输入后自动重新绑定续采，无需额外按键")

    def poll(self):
        now = time.monotonic_ns()
        snapshot = self.teleop.snapshot() if self.teleop is not None else None
        if snapshot is not None and snapshot.fault and self.stage != "error":
            self._fail(snapshot.fault)
            return
        if self.collection.state in {"error", "aborting"} and self.stage != "error":
            self._fail(self.collection.error or self.collection.message)
            return
        if self.stage == "error":
            self.freeze = True
            return
        if self.stage == "recording" and self.collection.state in {"saving", "idle"}:
            # A configured sample limit can finish between coordinator polls.
            self._halt()
            self._stage("saving", "采集帧数达到上限，等待文件完成")
        if self.stage in {"binding", "rebinding", "preparing", "recording"} and snapshot is not None:
            self._accept_local(snapshot, now)
            if self.stage == "error":
                return
        if self.stage in {"saving", "discarding"}:
            self.freeze = True
            if self.collection.state == "idle":
                self._halt()
                self._generation = self._sequence = -1
                self._external_session = None
                self._kind = self._entry_until = 0
                self._manual_pause_pending = False
                try:
                    self.app.next_task_after_episode()
                except Exception as exc:
                    self._fail(str(exc))
                    return
                self.control_flags = self.recovery = 0
                self._stage("idle", "整条已结束；下一条场景已在 Home，双手放在腰间，r 重新绑定开始")
            return
        if self.stage == "reverting":
            if self.collection.state == "paused":
                self._begin_bind(3)
            return
        if self.stage in {"binding", "rebinding"}:
            if self._binding_ready(snapshot):
                if self._kind == 1:
                    if not self._authorize():
                        self._fail(self.executor.mailbox.last_reject_reason)
                    elif self._request("start"):
                        self._stage("preparing", "建立初始检查点并打开数据文件；机器人不动")
                    else:
                        self._fail(self.notice)
                else:
                    self._activate(now)
            return
        if self.stage == "preparing":
            if self.collection.state != "recording":
                return
            if self._manual_pause_pending:
                self._manual_pause_pending = False
                self._pause()
                return
            if snapshot is not None and (snapshot.needs_rebind or not snapshot.arms_valid):
                self._pause(automatic=True)
                return
            self._activate(now)
            return
        if self.stage != "recording":
            return
        if snapshot is not None:
            lost = (snapshot.needs_rebind or snapshot.input_ns <= 0
                    or now - snapshot.input_ns > self.INPUT_TIMEOUT_NS
                    or now - snapshot.generated_ns > MAX_AGE_NS)
            if lost:
                self._pause(automatic=True)
                return
            if not snapshot.arms_valid:
                self.control_flags |= 1
        else:
            candidate = self.executor.mailbox.latest
            if (not self._external_ready() or not self.executor.mailbox.enabled
                    or candidate.session_id != self._external_session):
                self._pause(automatic=True)
                self.notice = "外部目标失效：现场已冻结；发布端对齐后按 r 重新接手"
                return
        if not self.executor.mailbox.enabled or self.executor.hold_mask & 1:
            self._pause(automatic=True)
            return
        if self.collection.state != "recording":
            self._fail(f"意外采集状态：{self.collection.state}")
            return
        self.recovery = self._kind if now < self._entry_until or self.executor.transition_active else 0

    def close(self):
        self._halt()
