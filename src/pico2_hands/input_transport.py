"""Exclusive local PICO input ownership and non-destructive ADB forwarding."""
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import subprocess
import tempfile


def ensure_adb_forward(port):
    """Reuse or create one unambiguous forwarding rule; never replace an owner."""
    def adb(*arguments):
        try:
            return subprocess.run(
                ["adb", *arguments], check=True, capture_output=True,
                text=True, timeout=5).stdout
        except FileNotFoundError as error:
            raise RuntimeError("ADB is required for local PICO input; install adb first") from error
        except subprocess.TimeoutExpired as error:
            raise RuntimeError("ADB preflight timed out; check the USB connection") from error
        except subprocess.CalledProcessError as error:
            raise RuntimeError(f"ADB preflight failed: {error.stderr.strip()}") from error

    devices = {}
    for line in adb("devices").splitlines():
        fields = line.split()
        if len(fields) >= 2 and not line.startswith("List of devices"):
            devices[fields[0]] = fields[1]
    serial = os.environ.get("ANDROID_SERIAL")
    if not serial:
        if len(devices) != 1:
            raise RuntimeError("Connect one PICO headset and authorize USB debugging; "
                               "with multiple devices set ANDROID_SERIAL explicitly")
        serial = next(iter(devices))
    if devices.get(serial) != "device":
        raise RuntimeError(f"PICO {serial} is not authorized/online; "
                           "accept USB debugging in the headset")

    endpoint = f"tcp:{port}"
    expected = [serial, endpoint, endpoint]
    forwards = [line.split() for line in adb("forward", "--list").splitlines()]
    owners = [row for row in forwards if len(row) == 3 and row[1] == endpoint]
    if owners:
        if owners != [expected]:
            raise RuntimeError(f"ADB {endpoint} already forwards to another device/port; "
                               "refusing to replace it")
        print(f"ADB ready: {serial} {endpoint} -> {endpoint} (reused)", flush=True)
        return
    adb("-s", serial, "forward", "--no-rebind", endpoint, endpoint)
    forwards = [line.split() for line in adb("forward", "--list").splitlines()]
    if expected not in forwards:
        raise RuntimeError(f"ADB did not retain the requested {endpoint} forwarding rule")
    print(f"ADB ready: {serial} {endpoint} -> {endpoint} (created)", flush=True)


@contextmanager
def input_lock(port):
    path = Path(tempfile.gettempdir()) / f"tianji-pico2-sim-{os.getuid()}-{port}.lock"
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        if os.fstat(fd).st_uid != os.getuid():
            raise RuntimeError("input lock is owned by another user")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("another PICO2 simulation owns this input port") from exc
        yield
    finally:
        os.close(fd)
