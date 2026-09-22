"""Spawn-isolated EGL workers with a shared, dynamically consumed episode queue."""
from __future__ import annotations

import multiprocessing as mp
import signal
import time
import traceback
from dataclasses import dataclass
from multiprocessing.connection import Connection, wait
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .config import BatchConfig, RenderSettings


class BatchRenderError(RuntimeError):
    """A failed batch, including its completed and unfinished job accounting."""

    def __init__(self, message: str, summary: dict[str, Any]):
        super().__init__(message)
        self.summary = summary


@dataclass
class _Worker:
    index: int
    gpu_id: int
    process: Any
    connection: Connection
    ready: bool = False
    stopped: bool = False
    current_job: int | None = None


def _worker_main(
    index: int, gpu_id: int, threads: int, expected_gpu_name: str | None,
    settings: RenderSettings, check_only: bool, jobs: Any, events: Connection,
) -> None:
    # The parent alone handles Ctrl+C and owns the lifetime of every child.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    current_job = None
    try:
        from .gpu import configure_device, probe_device

        configure_device(gpu_id, threads)
        device_info = probe_device(gpu_id, expected_gpu_name)
        if not check_only:
            from .renderer import render_episode

        events.send({"event": "ready", "worker": index, "device": device_info})
        if not check_only:
            while True:
                job = jobs.get()
                if job is None:
                    break
                current_job, source, destination = job
                events.send({"event": "started", "worker": index, "job": current_job})
                started = time.monotonic()
                result = render_episode(Path(source), Path(destination), settings, device_info)
                if not isinstance(result, dict) or result.get("status") not in {"rendered", "skipped"}:
                    raise RuntimeError("render_episode returned no rendered/skipped status")
                events.send({
                    "event": "result", "worker": index, "job": current_job,
                    "elapsed_s": time.monotonic() - started, "result": result,
                })
                current_job = None
        events.send({"event": "stopped", "worker": index})
    except BaseException as exc:
        try:
            events.send({
                "event": "error", "worker": index, "job": current_job,
                "error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc(),
            })
        finally:
            raise
    finally:
        events.close()


def _discover(config: BatchConfig) -> list[tuple[Path, Path]]:
    source_root = Path(config.input_dir).expanduser().resolve()
    output_root = Path(config.output_dir).expanduser().resolve()
    if not source_root.exists():
        raise ValueError(f"Input does not exist: {source_root}")
    if output_root.exists() and not output_root.is_dir():
        raise ValueError(f"Output must be a directory: {output_root}")
    single = source_root.is_file()
    if single:
        if source_root.suffix != ".h5" or source_root.name.endswith(".partial.h5"):
            raise ValueError("Explicit input must be a completed .h5 episode, not a .partial.h5 file")
        relative_root = source_root.parent
        if source_root.is_relative_to(output_root):
            raise ValueError("Output directory must not contain the input episode")
        sources = [source_root]
    else:
        if not source_root.is_dir():
            raise ValueError(f"Input is not an episode file or directory: {source_root}")
        if output_root.is_relative_to(source_root) or source_root.is_relative_to(output_root):
            raise ValueError("Input and output directories must be disjoint (neither may contain the other)")
        relative_root = source_root
        sources = sorted(
            path for path in source_root.rglob("episode_*.h5")
            if path.is_file() and not path.name.endswith((".partial.h5", ".render.h5"))
        )
    if not sources:
        raise ValueError(f"No completed-looking episode_*.h5 inputs found in {source_root}")
    discovered = []
    seen_sources: set[Path] = set()
    seen_outputs: set[Path] = set()
    for source in sources:
        resolved_source = source.resolve()
        if not resolved_source.is_relative_to(relative_root):
            raise ValueError(f"Input symlink escapes the input root: {source}")
        if resolved_source in seen_sources:
            raise ValueError(f"Duplicate source episode (possibly a symlink): {source}")
        destination = output_root / source.relative_to(relative_root).with_suffix(".render.h5")
        resolved_destination = destination.resolve()
        if not resolved_destination.is_relative_to(output_root):
            raise ValueError(f"Output symlink escapes the output root: {destination}")
        if resolved_destination == resolved_source or resolved_destination in seen_outputs:
            raise ValueError(f"Unsafe or duplicate output destination: {destination}")
        if not single and resolved_destination.is_relative_to(source_root):
            raise ValueError(f"Output destination overlaps input: {destination}")
        seen_sources.add(resolved_source)
        seen_outputs.add(resolved_destination)
        discovered.append((resolved_source, destination))
    return discovered


def _shutdown(workers: list[_Worker], jobs: Any, *, abort: bool) -> None:
    if abort:
        for worker in workers:
            if worker.process.is_alive():
                worker.process.terminate()
    deadline = time.monotonic() + 5.0
    for worker in workers:
        worker.process.join(max(0.0, deadline - time.monotonic()))
    for worker in workers:
        if worker.process.is_alive():
            worker.process.kill()
    for worker in workers:
        worker.process.join()
        worker.connection.close()
        worker.process.close()
    # An aborted consumer must never leave us waiting for the queue feeder.
    jobs.cancel_join_thread()
    jobs.close()


def run_batch(config: BatchConfig, *, check_only: bool = False) -> dict[str, Any]:
    """Probe every selected worker, then render all episodes without automatic retries.

    Errors raise BatchRenderError carrying the same JSON-ready summary as success.
    Check-only never inspects or creates input/output dataset paths.
    """
    started = time.monotonic()
    summary: dict[str, Any] = {
        "status": "running", "check_only": check_only, "gpu_ids": list(config.gpu_ids),
        "workers_per_gpu": config.workers_per_gpu, "devices": [], "total": 0,
        "rendered": 0, "skipped": 0, "failed": 0, "episodes": [], "errors": [],
    }
    workers: list[_Worker] = []
    tasks: list[tuple[Path, Path]] = []
    completed: set[int] = set()
    jobs = None
    failure: BaseException | None = None
    try:
        if not config.gpu_ids or any(type(gpu) is not int or gpu < 0 for gpu in config.gpu_ids):
            raise ValueError("GPU IDs must be a nonempty sequence of nonnegative EGL device indices")
        if len(set(config.gpu_ids)) != len(config.gpu_ids):
            raise ValueError("Duplicate GPU IDs are not allowed; use workers_per_gpu instead")
        if type(config.workers_per_gpu) is not int or config.workers_per_gpu < 1:
            raise ValueError("workers_per_gpu must be a positive integer")
        if type(config.threads_per_worker) is not int or config.threads_per_worker < 1:
            raise ValueError("threads_per_worker must be a positive integer")
        if not 0 < config.startup_timeout_s < float("inf"):
            raise ValueError("startup_timeout_s must be finite and positive")
        if not check_only:
            tasks = _discover(config)
        summary["total"] = len(tasks)
        context = mp.get_context("spawn")
        jobs = context.Queue()
        startup_deadline = time.monotonic() + config.startup_timeout_s
        for gpu_id in config.gpu_ids:
            for _ in range(config.workers_per_gpu):
                receiver, sender = context.Pipe(duplex=False)
                index = len(workers)
                process = context.Process(
                    name=f"egl-{gpu_id}-worker-{index}", target=_worker_main,
                    args=(index, gpu_id, config.threads_per_worker, config.expected_gpu_name,
                          config.settings, check_only, jobs, sender),
                )
                try:
                    process.start()
                except BaseException:
                    receiver.close()
                    sender.close()
                    raise
                sender.close()
                workers.append(_Worker(index, gpu_id, process, receiver))

        submitted = False
        stopping = check_only
        shutdown_deadline: float | None = None
        open_connections = {worker.connection: worker for worker in workers}
        while True:
            ready_count = sum(worker.ready for worker in workers)
            if ready_count != len(workers) and time.monotonic() >= startup_deadline:
                missing = [worker.index for worker in workers if not worker.ready]
                raise RuntimeError(f"GPU startup timed out after {config.startup_timeout_s:g}s; workers {missing}")
            if ready_count == len(workers) and not submitted:
                submitted = True
                for job_id, (source, destination) in enumerate(tasks):
                    jobs.put((job_id, str(source), str(destination)))
            if submitted and len(completed) == len(tasks) and not stopping:
                stopping = True
                shutdown_deadline = time.monotonic() + 10.0
                for _ in workers:
                    jobs.put(None)
            if check_only and submitted and shutdown_deadline is None:
                shutdown_deadline = time.monotonic() + 10.0
            if shutdown_deadline is not None and time.monotonic() >= shutdown_deadline:
                raise RuntimeError("Workers did not exit within 10s after completing their work")
            handles = list(open_connections) + [
                worker.process.sentinel for worker in workers if worker.process.exitcode is None
            ]
            if handles:
                signalled = wait(handles, timeout=0.2)
                for connection in list(open_connections):
                    if connection not in signalled:
                        continue
                    worker = open_connections[connection]
                    try:
                        event = connection.recv()
                    except EOFError:
                        del open_connections[connection]
                        continue
                    kind = event["event"]
                    if event["worker"] != worker.index:
                        raise RuntimeError("Worker event identity mismatch")
                    if kind == "ready":
                        if worker.ready:
                            raise RuntimeError(f"Worker {worker.index} reported readiness twice")
                        worker.ready = True
                        summary["devices"].append({"worker": worker.index, **event["device"]})
                    elif kind == "started":
                        job_id = event["job"]
                        if (not submitted or worker.current_job is not None or job_id in completed
                                or not 0 <= job_id < len(tasks)
                                or any(other.current_job == job_id for other in workers)):
                            raise RuntimeError(f"Invalid job assignment from worker {worker.index}: {job_id}")
                        worker.current_job = job_id
                    elif kind == "result":
                        job_id = event["job"]
                        if worker.current_job != job_id or job_id in completed:
                            raise RuntimeError(f"Unexpected result from worker {worker.index}: {job_id}")
                        result = event["result"]
                        status = result["status"]
                        if status not in {"rendered", "skipped"}:
                            raise RuntimeError(f"Invalid render result status: {status}")
                        source, destination = tasks[job_id]
                        summary["episodes"].append({
                            "job": job_id, "source": str(source), "output": str(destination),
                            "worker": worker.index, "gpu_id": worker.gpu_id, "status": status,
                            "elapsed_s": event["elapsed_s"], "result": result,
                        })
                        summary[status] += 1
                        completed.add(job_id)
                        worker.current_job = None
                    elif kind == "stopped":
                        if not stopping or worker.current_job is not None or not worker.ready:
                            raise RuntimeError(f"Worker {worker.index} stopped before completing its work")
                        worker.stopped = True
                    elif kind == "error":
                        summary["errors"].append(event)
                        raise RuntimeError(f"GPU {worker.gpu_id}, worker {worker.index}: {event['error']}")
                    else:
                        raise RuntimeError(f"Unknown worker event: {kind}")
            for worker in workers:
                exitcode = worker.process.exitcode
                if exitcode is not None:
                    # Drain already-sent pipe events before classifying an exit.
                    if worker.connection in open_connections and worker.connection.poll():
                        continue
                    if exitcode != 0 or not worker.stopped:
                        raise RuntimeError(
                            f"GPU {worker.gpu_id}, worker {worker.index} exited unexpectedly "
                            f"with code {exitcode}; active job={worker.current_job}"
                        )
            if all(worker.stopped and worker.process.exitcode == 0 for worker in workers):
                if len(completed) != len(tasks):
                    raise RuntimeError("Workers exited with unfinished episodes")
                break
        summary["status"] = "complete"
    except BaseException as exc:
        failure = exc
        summary["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        message = "Interrupted by user" if isinstance(exc, KeyboardInterrupt) else f"{type(exc).__name__}: {exc}"
        summary["errors"].append({"error": message})
        active = {worker.current_job: worker for worker in workers if worker.current_job is not None}
        for job_id, (source, destination) in enumerate(tasks):
            if job_id in completed:
                continue
            worker = active.get(job_id)
            summary["episodes"].append({
                "job": job_id, "source": str(source), "output": str(destination), "status": "failed",
                "worker": worker.index if worker else None, "gpu_id": worker.gpu_id if worker else None,
                "error": "Batch aborted without retry; completion not confirmed. "
                         "Inspect any partial/lock files before explicit cleanup and rerun.",
            })
            summary["failed"] += 1
    finally:
        if jobs is not None:
            _shutdown(workers, jobs, abort=failure is not None)
        summary["elapsed_s"] = time.monotonic() - started
        summary["episodes"].sort(key=lambda item: item["job"])
        summary["devices"].sort(key=lambda item: item["worker"])
    if failure is not None:
        raise BatchRenderError(summary["errors"][-1]["error"], summary) from failure
    return summary


__all__ = ["BatchRenderError", "run_batch"]
