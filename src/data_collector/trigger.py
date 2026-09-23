"""Foreground ROS-only operator client for the existing SPD collector."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any

from interfaces.keyboard_control import KEY_COMMANDS, read_key, terminal_input


STATUS_MAX_AGE = 2.0  # Collector heartbeat is at most 0.5 seconds apart.
STATES = {"idle", "preparing", "recording", "paused", "reverting", "saving", "discarding", "aborting", "error"}
STRING_FIELDS = (
    "collector_id", "operation_id", "operation", "state", "message", "error",
    "episode_path", "last_saved_path", "last_outcome",
)
COUNT_FIELDS = ("state_frames", "max_frames")
FINAL_STATES = {
    "start": {"recording"}, "save": {"idle"}, "discard": {"idle"},
    "checkpoint": {"recording", "paused"}, "pause": {"paused"},
    "resume": {"recording"}, "revert": {"paused"}, "skip": {"idle"},
}
CONFIRMATION_FIELDS = (
    "collector_id", "episode_path", "operation_id", "state", "checkpoint_frames",
    "skip_confirmation", "message", "error",
)


class ClientError(RuntimeError):
    """The collector outcome cannot safely be treated as successful."""


class ClientExit(Exception):
    """Leave the client without issuing another collector operation."""


def positive_seconds(value: str) -> float:
    try:
        seconds = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("timeout must be positive finite seconds") from exc
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("timeout must be positive finite seconds")
    return seconds


def json_object(payload: str) -> dict[str, Any]:
    def unique_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate field {key!r}")
            result[key] = value
        return result

    try:
        result = json.loads(payload, object_pairs_hook=unique_pairs)
    except (ValueError, TypeError, RecursionError) as exc:
        raise ClientError(f"Malformed collector JSON: {exc}") from exc
    if not isinstance(result, dict):
        raise ClientError("Malformed collector JSON: expected an object")
    return result


def parse_status(payload: str) -> dict[str, Any]:
    status = json_object(payload)
    fields = {*STRING_FIELDS, *COUNT_FIELDS, "elapsed_s", "physics_paused",
              "checkpoint_frames", "auto_checkpoint_frames", "skip_confirmation"}
    if set(status) != fields:
        raise ClientError("Malformed collector status: unexpected or missing fields")
    if any(not isinstance(status[key], str) for key in STRING_FIELDS):
        raise ClientError("Malformed collector status: text fields must be strings")
    if not status["collector_id"] or status["state"] not in STATES:
        raise ClientError("Malformed collector status: invalid identity or state")
    if any(type(status[key]) is not int or status[key] < 0 for key in COUNT_FIELDS):
        raise ClientError("Malformed collector status: counts must be nonnegative integers")
    if type(status["physics_paused"]) is not bool:
        raise ClientError("Malformed collector status: physics_paused must be a boolean")
    if type(status["skip_confirmation"]) is not bool:
        raise ClientError("Malformed collector status: skip_confirmation must be a boolean")
    for field in ("checkpoint_frames", "auto_checkpoint_frames"):
        count = status[field]
        if count is not None and (type(count) is not int or count < 0):
            raise ClientError(f"Malformed collector status: {field} must be null or a nonnegative integer")
    if status["last_outcome"] not in {"", "saved", "discarded"}:
        raise ClientError("Malformed collector status: invalid last_outcome")
    elapsed = status["elapsed_s"]
    try:
        valid_elapsed = type(elapsed) in (int, float) and math.isfinite(elapsed) and elapsed >= 0
    except OverflowError:
        valid_elapsed = False
    if not valid_elapsed:
        raise ClientError("Malformed collector status: elapsed_s must be finite and nonnegative")
    return status


def show_status(status: dict[str, Any]) -> None:
    # JSON quoting prevents collector text from injecting terminal control sequences.
    print(
        f"state={status['state']} operation={json.dumps(status['operation'])} "
        f"collector_id={json.dumps(status['collector_id'])} "
        f"operation_id={json.dumps(status['operation_id'])}\n"
        f"  state_frames={status['state_frames']} "
        f"elapsed_s={status['elapsed_s']:.3f} max_frames={status['max_frames']}\n"
        f"  physics_paused={status['physics_paused']} checkpoint_frames={status['checkpoint_frames']} "
        f"auto_checkpoint_frames={status['auto_checkpoint_frames']} last_outcome={json.dumps(status['last_outcome'])}\n"
        f"  episode_path={json.dumps(status['episode_path'])} "
        f"last_saved_path={json.dumps(status['last_saved_path'])}\n"
        f"  message={json.dumps(status['message'])} error={json.dumps(status['error'])}",
        flush=True,
    )


class CollectionTrigger:
    def __init__(self, node: Any, interactive: bool) -> None:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        from data_collector.ros_control import SERVICE_NAMES, STATUS_TOPIC

        self.node = node
        self.interactive = interactive
        self.ros = rclpy
        self.request_type = Trigger.Request
        self.service_names = SERVICE_NAMES
        self.status_topic = STATUS_TOPIC
        self.clients = {name: node.create_client(Trigger, path) for name, path in SERVICE_NAMES.items()}
        self.status: dict[str, Any] | None = None
        self.received_at = 0.0
        self.received_count = 0
        self.status_error = ""
        self.collector_id = ""
        self.transition = None
        self.observed: dict[str, dict[str, Any]] | None = None
        self.skip_confirmation: tuple[Any, ...] | None = None
        self.subscription = node.create_subscription(
            String, STATUS_TOPIC, self.on_status,
            QoSProfile(
                depth=1, history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def on_status(self, message: Any) -> None:
        try:
            status = parse_status(message.data)
            if self.collector_id and status["collector_id"] != self.collector_id:
                raise ClientError("Collector identity changed; outcome unknown. Reconnect explicitly.")
        except ClientError as exc:
            self.skip_confirmation = None
            self.status_error = str(exc)
            return
        if self.skip_confirmation != tuple(status[key] for key in CONFIRMATION_FIELDS):
            self.skip_confirmation = None
        self.collector_id = status["collector_id"]
        self.status = status
        self.received_at = time.monotonic()
        self.received_count += 1
        if self.observed is not None:
            self.observed[status["operation_id"]] = status
        transition = tuple(status[key] for key in ("collector_id", "operation_id", "state", "error"))
        if transition != self.transition:
            show_status(status)
            self.transition = transition

    def graph_problem(self) -> str:
        publishers = self.node.get_publishers_info_by_topic(self.status_topic)
        if len(publishers) > 1:
            raise ClientError("Ambiguous collector: multiple status publishers; no further commands sent.")
        providers: dict[str, list[tuple[str, str]]] = {name: [] for name in self.service_names}
        for node_name, namespace in self.node.get_node_names_and_namespaces():
            services = self.node.get_service_names_and_types_by_node(node_name, namespace)
            for command, service_path in self.service_names.items():
                for name, types in services:
                    if name == service_path:
                        if types != ["std_srvs/srv/Trigger"]:
                            raise ClientError(f"Unexpected service type for {service_path}")
                        providers[command].append((node_name, namespace))
        for command, owners in providers.items():
            if len(owners) > 1:
                raise ClientError(f"Ambiguous collector: multiple providers for {self.service_names[command]}")
        if not publishers:
            return "status publisher missing"
        publisher = publishers[0]
        if publisher.topic_type != "std_msgs/msg/String":
            raise ClientError("Unexpected collector status topic type")
        owner = (publisher.node_name, publisher.node_namespace)
        for command, owners in providers.items():
            if not owners or not self.clients[command].service_is_ready():
                return f"service not ready: {self.service_names[command]}"
            if owners[0] != owner:
                raise ClientError("Collector services and status have different ROS node owners")
        return ""

    def step(self, *, waiting: bool = True) -> None:
        if not self.ros.ok():
            raise ClientExit
        self.ros.spin_once(self.node, timeout_sec=0.05)
        if self.status_error:
            raise ClientError(self.status_error)
        if waiting and self.interactive:
            key = read_key()
            if key:
                self.skip_confirmation = None
            if key == "q":
                raise ClientExit
            if key in KEY_COMMANDS:
                print("Operation pending; key ignored. q exits this client only.", flush=True)

    def require_fresh_status(self) -> None:
        if self.status is None or time.monotonic() - self.received_at > STATUS_MAX_AGE:
            raise ClientError("Collector status is missing or stale; outcome unknown; no automatic retry.")

    def wait_ready(self, deadline: float) -> None:
        problem = "waiting for live status heartbeat"
        next_graph_check = 0.0
        graph_problem = "discovery pending"
        while time.monotonic() < deadline:
            self.step()
            now = time.monotonic()
            if now >= next_graph_check:
                graph_problem = self.graph_problem()
                next_graph_check = now + 0.25
            problem = graph_problem or "waiting for live status heartbeat"
            # One retained transient-local sample is not proof the collector is alive.
            if not graph_problem and self.received_count >= 2:
                self.require_fresh_status()
                return
        raise ClientError(f"Collector unavailable: {problem}; no command sent.")

    def execute(self, command: str, timeout: float) -> None:
        if command != "discard":
            self.skip_confirmation = None
        deadline = time.monotonic() + timeout
        self.wait_ready(deadline)
        if command == "status":
            show_status(self.status)
            if self.status["state"] == "error":
                raise ClientError("Collector reports an error; see status above.")
            return
        problem = self.graph_problem()
        if problem:
            raise ClientError(f"Collector unavailable: {problem}; no command sent.")
        self.require_fresh_status()
        if command == "pause_toggle":
            command = "resume" if self.status["state"] == "paused" else "pause"
        elif command == "discard" and self.interactive:
            if self.status["state"] not in {"recording", "paused"}:
                self.skip_confirmation = None
                print("x requires a recording or paused episode; no command sent.", flush=True)
                return
            context = tuple(self.status[key] for key in CONFIRMATION_FIELDS)
            if self.skip_confirmation != context:
                self.skip_confirmation = context
                print("Tap x again to confirm discarding this episode; another key or state change cancels.",
                      flush=True)
                return
            self.skip_confirmation = None
        collector_id = self.collector_id
        previous_saved_path = self.status["last_saved_path"]
        self.observed = {}
        future = self.clients[command].call_async(self.request_type())
        print(f"Requested {command}; acceptance is not completion. No automatic retry.", flush=True)
        operation_id = ""
        next_graph_check = 0.0
        try:
            while time.monotonic() < deadline:
                self.step()
                self.require_fresh_status()
                now = time.monotonic()
                if now >= next_graph_check:
                    problem = self.graph_problem()
                    if problem:
                        raise ClientError(f"Collector lost: {problem}; operation outcome unknown.")
                    next_graph_check = now + 0.5
                if not operation_id and future.done():
                    response = future.result()
                    if response is None:
                        raise ClientError("Service returned no response; operation outcome unknown.")
                    details = json_object(response.message)
                    if set(details) != {"collector_id", "operation_id", "message"} or any(
                        not isinstance(value, str) for value in details.values()
                    ):
                        raise ClientError("Malformed service response; operation outcome unknown.")
                    if details["collector_id"] != collector_id:
                        raise ClientError("Service response is from a different collector; outcome unknown.")
                    if not response.success:
                        raise ClientError(f"{command} rejected: {json.dumps(details['message'])}")
                    operation_id = details["operation_id"]
                    if not operation_id:
                        raise ClientError("Accepted response has no operation_id; outcome unknown.")
                    print(f"Accepted operation_id={json.dumps(operation_id)}; awaiting final state.", flush=True)
                if operation_id:
                    status = self.observed.get(operation_id)
                    if status is None:
                        continue
                    if status["collector_id"] != collector_id or status["operation"] != command:
                        raise ClientError("Matching operation has inconsistent identity; outcome unknown.")
                    if status["state"] == "error" or status["error"]:
                        show_status(status)
                        raise ClientError(f"{command} failed; see collector error above.")
                    final_states = FINAL_STATES[command]
                    auto_finished = (
                        command in {"start", "resume", "checkpoint"} and status["state"] == "idle"
                        and status["state_frames"] > 0 and bool(status["last_saved_path"])
                        and status["last_saved_path"] != previous_saved_path
                    )
                    if status["state"] in final_states or auto_finished:
                        show_status(status)
                        print(f"{command} completed (matching collector and operation).", flush=True)
                        return
            raise ClientError(f"{command} timed out; outcome unknown. Inspect status; do not retry blindly.")
        finally:
            self.observed = None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command", choices=(*FINAL_STATES, "status"))
    parser.add_argument("--timeout", type=positive_seconds, default=30.0, metavar="SECONDS")
    args = parser.parse_args(argv)
    interactive = args.command is None
    if interactive and not sys.stdin.isatty():
        parser.error("interactive mode needs a terminal; use --command " + "|".join((*FINAL_STATES, "status")))
    if interactive:
        print(
            "Low-level ROS collection client (no motion authorization)\n"
            "  Default spd-sim exposes status only: use --command status.\n"
            "  With a separately enabled control service: r checkpoint, s save, d revert, Space pause/resume, x discard.\n"
            "  Normal operator controls belong in the SPD window/terminal, not this client.\n"
            "  q / Ctrl+C exits this client only; no save or discard on exit.\n"
            "Commands wait for collector completion; unknown outcomes are never retried.",
            flush=True,
        )
    node = None
    rclpy = None
    initialized = False
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node(f"spd_collection_trigger_{os.getpid()}")
        with terminal_input(interactive):
            client = CollectionTrigger(node, interactive)
            if args.command:
                client.execute(args.command, args.timeout)
                return 0
            client.wait_ready(time.monotonic() + args.timeout)
            next_graph_check = 0.0
            while True:
                client.step(waiting=False)
                client.require_fresh_status()
                if time.monotonic() >= next_graph_check:
                    problem = client.graph_problem()
                    if problem:
                        raise ClientError(f"Collector unavailable: {problem}")
                    next_graph_check = time.monotonic() + 0.5
                key = read_key()
                if key and key != "x":
                    client.skip_confirmation = None
                if key == "q":
                    raise ClientExit
                command = KEY_COMMANDS.get(key)
                if command:
                    client.execute(command, args.timeout)
    except (ClientExit, EOFError, KeyboardInterrupt):
        print("Client exiting only. Already submitted operations may still complete; no exit operation sent.")
        return 130 if sys.exc_info()[0] is KeyboardInterrupt else 0
    except Exception as exc:
        print(f"Collection client failed: {exc}\nNo automatic retry or exit operation sent.", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
