"""Single physics-thread authority for input binding, collection and scene lifecycle.

The teleop worker never changes MuJoCo state. Every bind/rewind is issued while
physics is frozen, and generation barriers exclude pre-rewind worker results.
"""
from __future__ import annotations

import math
import time
from uuid import uuid4

import numpy as np

from interfaces.ros_joint_command import JointCommandSnapshot, MAX_AGE_NS


class CollectionControl:
    INPUT_TIMEOUT_NS = 120_000_000
    ENTRY_NS = 500_000_000
    HOME_TIMEOUT_NS = 30_000_000_000

    def __init__(self, app):
        self.app = app
        self.stage = "idle"
        self.notice = "双手放在腰间准备位；r 开始，s 保存，d 回退"
        self.recovery = 0
        self.control_flags = 0
        self._session = uuid4().hex
        self._generation = -1
        self._sequence = -1
        self._kind = 0
        self._entry_until = 0
        self._discard_confirmation = False
        self._manual_pause_pending = False
        self._external_session = None
        self._home_started = 0
        self._home_settled = None
        self._home_origin = None
        self._home_duration = 0.0
        self._home_sequence = 0
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
        if stage != self.stage:
            self.cancel_confirmation()
        self.stage, self.notice = stage, notice
        if changed:
            print(f"SPD [{stage}]: {notice}", flush=True)

    def cancel_confirmation(self):
        self._discard_confirmation = False

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
        if automatic and self.collection.state == "recording":
            self.collection.capture_auto_checkpoint()
        self._halt()
        if self.collection.state not in {"recording", "paused"}:
            return
        if automatic:
            self._stage("auto_paused", "跟踪丢失：保持当前姿态；摆好现实姿态后按 r 重新接手，s 保存")
        else:
            self._stage("paused", "人工暂停：空格重新接手；d 回退；s 保存；x 放弃")

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
        if snapshot.mode not in {"bound", "follow", "home", "home_done"}:
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
        self._stage("recording", "r 存检查点；s 成功保存；d 回退；空格暂停")

    def key(self, key):
        key = key.lower()
        if key == "q":
            self._halt()
            self.app.request_stop()
            return
        if self.stage == "auto_paused":
            if key == "r":
                if not self.teleop.snapshot().can_bind:
                    self.notice = "跟踪尚未稳定；保持头和双腕可见，摆好姿态后再按 r 重新接手"
                    return
                self._begin_bind(4)
                return
            if key in {"d", " "}:
                self.notice = "失跟踪后请先按 r 重新接手；不回退、不更新保存点，接手后 r 存点、d 回退"
                return
        if key != "x":
            self.cancel_confirmation()
        if key == " ":
            if self.stage == "returning_home":
                self._halt()
                self._stage("home_paused", "回 Home 已暂停；空格继续，q 退出")
                return
            if self.stage == "home_paused":
                self._begin_home_motion()
                return
            if self.stage in {"reverting", "saving", "discarding"}:
                self._manual_pause_pending = True
                self.notice = "当前磁盘操作完成后保持暂停；不会自动运动"
                return
            if self.stage in {"recording", "binding", "rebinding", "auto_paused", "preparing", "rewind_wait"}:
                if self.stage == "binding" and self.collection.state == "idle":
                    self._halt()
                    self._stage("idle", "开始已取消；r 重新开始")
                else:
                    self._pause()
            elif self.stage == "paused":
                self._begin_bind(2)
            return
        if self.stage in {"binding", "preparing", "rebinding", "reverting", "saving", "discarding", "returning_home", "error"}:
            return
        if key == "r":
            if self.stage == "idle" and self.collection.state == "idle":
                self._begin_bind(1)
            elif self.stage == "recording" and self._request("checkpoint"):
                self.notice = f"当前保存点已更新：{self.collection.state_frames} 帧"
        elif key == "d" and self.collection.state in {"recording", "paused"}:
            self._halt()
            if self._request("revert"):
                self._kind = 3
                self._stage("reverting", "恢复完整检查点并裁掉失败分支；随后自动重新接手")
        elif key == "s" and self.collection.state in {"recording", "paused"}:
            if self.collection.state_frames == 0:
                self.notice = "尚无实际采集帧，不能保存成功"
                return
            self._halt()
            if self._request("save"):
                self._stage("saving", "关闭并校验示范文件；成功后直接随机新任务，Home 等待 r")
        elif key == "x" and self.collection.state in {"recording", "paused"}:
            if self._discard_confirmation:
                self._halt()
                if self._request("discard"):
                    self._stage("discarding", "放弃本条；回 Home 后重做同一任务和初始场景")
            else:
                self._pause()
                self._discard_confirmation = True
                self.notice = "再次按 x 确认放弃整条；空格继续、d 回退、s 保存"

    def _start_home(self):
        """Discard only: return physically Home before rebuilding the same task."""
        self.freeze = True
        self.executor.clear()
        if self.teleop is not None:
            self.teleop.pause()
        self.app.begin_home_return()
        self.freeze = True
        if self._manual_pause_pending:
            self._manual_pause_pending = False
            self._stage("home_paused", "准备场景已冻结；空格开始回 Home，q 退出")
        else:
            self._begin_home_motion()

    def _begin_home_motion(self):
        self.freeze = True
        self.executor.clear()
        # Scene construction may take longer than an entire motion segment.
        # Start the return clock only after the new physics owner exists.
        self._home_started = time.monotonic_ns()
        self._home_settled = None
        self._sequence = -1
        origin = self.app.plant.joint_command_targets()
        if self.teleop is not None:
            self._generation = self.teleop.home(origin, self.app.home_targets)
        else:
            self._home_origin = origin.copy()
            distance = float(np.max(np.abs(self.app.home_targets - origin)))
            # Quintic rest-to-rest reference with conservative global v/a/j caps.
            self._home_duration = max(1.0, 1.875 * distance / .6,
                                      math.sqrt(5.774 * distance / 1.2),
                                      (60 * distance / 6.0) ** (1 / 3))
            self._home_sequence = 0
        self._stage("returning_home", "准备场景：平滑回 Home，不录入示范")

    def _poll_home(self, snapshot, now):
        if now - self._home_started > self.HOME_TIMEOUT_NS:
            error = np.max(np.abs(self.app.plant.joint_command_positions()[:14] - self.app.home_targets[:14]))
            velocity = np.abs(self.app.plant.joint_command_velocities())
            mode = snapshot.mode if snapshot is not None else "local_home"
            self._fail(f"回 Home 超时：轨迹={mode}，物理 tick={self.app.plant.tick}，"
                       f"双臂误差={error:.4f} rad，速度={np.max(velocity[:14]):.4f}/"
                       f"{np.max(velocity[14:]):.4f} rad/s；不瞬移、不切任务")
            return
        done = False
        if self.teleop is None:
            elapsed = (now - self._home_started) * 1e-9
            u = min(1.0, elapsed / self._home_duration)
            w = u ** 3 * (10 + u * (-15 + 6 * u))
            self._home_sequence += 1
            command = JointCommandSnapshot.from_values(
                session_id=f"{self._session}:home", sequence=self._home_sequence, ready_mask=7,
                position_rad=self._home_origin + w * (self.app.home_targets - self._home_origin),
            )
            if not self.executor.mailbox.receive(command):
                self._fail(self.executor.mailbox.last_reject_reason)
                return
            done = u == 1.0
        else:
            if snapshot.fault:
                self._fail(snapshot.fault)
                return
            if snapshot.generation != self._generation or snapshot.mode not in {"home", "home_done"}:
                return
            if now - snapshot.generated_ns > MAX_AGE_NS:
                self._fail("Home 轨迹生成中断")
                return
            done = snapshot.mode == "home_done"
        if not self.executor.mailbox.enabled:
            if self.executor.mailbox.latest is None:
                return
            if not self.executor.authorize(True):
                self._fail(self.executor.mailbox.last_reject_reason)
                return
        self.freeze = False
        measured = self.app.plant.joint_command_positions()
        velocity = self.app.plant.joint_command_velocities()
        # Compliant finger contacts exhibit high-frequency velocity chatter even
        # at neutral Home. Require a bounded physical-position envelope over a
        # full settling window, rather than restarting on each solver impulse.
        settled = (done and np.isfinite(measured).all() and np.isfinite(velocity).all()
                   and np.max(np.abs(measured[:14] - self.app.home_targets[:14])) < .1
                   and np.max(np.abs(velocity[:14])) < .05)
        if not settled:
            self._home_settled = None
        elif self._home_settled is None:
            self._home_settled = (now, measured.copy(), measured.copy())
        else:
            started, low, high = self._home_settled
            np.minimum(low, measured, out=low)
            np.maximum(high, measured, out=high)
            if now - started >= 150_000_000:
                span = high - low
                if np.max(span[:14]) > .005 or np.max(span[14:]) > .02:
                    self._home_settled = (now, measured.copy(), measured.copy())
                    return
                self._halt()
                self.app.complete_home_return()
                self.freeze = True
                self.control_flags = self.recovery = 0
                self._stage("idle", "Home 已就绪；双手放在腰间，r 开始下一条")

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
        if self.stage in {"binding", "rebinding", "preparing", "recording", "returning_home"} and snapshot is not None:
            self._accept_local(snapshot, now)
            if self.stage == "error":
                return
        if self.stage == "returning_home":
            self._poll_home(snapshot, now)
            return
        if self.stage in {"saving", "discarding"}:
            self.freeze = True
            if self.collection.state == "idle":
                if self.stage == "saving":
                    self._halt()
                    self._generation = self._sequence = -1
                    self._external_session = None
                    self._kind = self._entry_until = 0
                    self._manual_pause_pending = False
                    self.app.next_task_after_save()
                    self.control_flags = self.recovery = 0
                    self._stage("idle", "文件已保存并校验；全新随机任务已在 Home，双手放在腰间，r 重新绑定开始")
                else:
                    self._start_home()
            return
        if self.stage == "reverting":
            if self.collection.state == "paused":
                if self._manual_pause_pending:
                    self._manual_pause_pending = False
                    self._stage("paused", "检查点已恢复并保持暂停；空格重新接手")
                else:
                    self._stage("rewind_wait", "检查点已恢复；等待稳定输入，自动重新接手")
            return
        if self.stage == "rewind_wait":
            if snapshot is not None and snapshot.can_bind:
                self._begin_bind(self._kind)
            elif snapshot is None and self._external_ready():
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
                self._pause()
                self.notice = "外部目标失效：现场已冻结；发布端对齐后按空格恢复"
                return
        if not self.executor.mailbox.enabled or self.executor.hold_mask & 1:
            self._pause(automatic=snapshot is not None)
            return
        if self.collection.state != "recording":
            self._fail(f"意外采集状态：{self.collection.state}")
            return
        self.recovery = self._kind if now < self._entry_until or self.executor.transition_active else 0

    def close(self):
        self._halt()
