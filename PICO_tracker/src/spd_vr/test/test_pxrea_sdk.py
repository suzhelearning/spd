import ctypes
import gc
import weakref

import pytest

from spd_vr.pxrea_sdk import (
    BoundedCallbackQueue,
    CallbackEvent,
    PXREAClient,
    PXREADevCustomMessage,
    PXREAError,
)


class FakeFn:
    def __init__(self, result=0):
        self.result = result
        self.argtypes = None
        self.restype = None
        self.calls = []

    def __call__(self, *args):
        self.calls.append(args)
        return self.result


class FakeLibrary:
    def __init__(self, init_result=0):
        self.PXREAInit = FakeFn(init_result)
        self.PXREADeinit = FakeFn(0)
        self.PXREASendCustomMessage = FakeFn(0)


def test_custom_message_matches_sdk_abi():
    assert PXREADevCustomMessage.devID.offset == 0
    assert PXREADevCustomMessage.dataSize.offset == 32
    assert PXREADevCustomMessage.dataPtr.offset == 40
    assert ctypes.sizeof(PXREADevCustomMessage) == 48


def test_init_failure_does_not_deinit():
    lib = FakeLibrary(init_result=7)
    with pytest.raises(PXREAError, match="PXREAInit failed: 7"):
        PXREAClient(lib).__enter__()
    assert len(lib.PXREADeinit.calls) == 0


def test_context_calls_deinit_once_and_keeps_callback_alive():
    lib = FakeLibrary()
    client = PXREAClient(lib)
    with client:
        callback_ref = weakref.ref(client.callback)
        assert callback_ref() is not None
        assert lib.PXREAInit.argtypes[1] is client.callback_type
    del callback_ref
    client.close()
    client.close()
    gc.collect()
    assert len(lib.PXREAInit.calls) == 1
    assert len(lib.PXREADeinit.calls) == 1
    assert client.callback is None


def test_callback_copies_only_bounded_payload():
    queue = BoundedCallbackQueue()
    client = PXREAClient(FakeLibrary(), queue=queue)
    oversized = ctypes.create_string_buffer(b"x" * 2049)
    message = PXREADevCustomMessage(b"FAKE", 2049, ctypes.cast(oversized, ctypes.POINTER(ctypes.c_char)))
    client._callback(None, 0, 0, ctypes.byref(message))
    assert queue.qsize() == 0
    assert client.status()["dropped_oversize"] == 1


def test_queue_drops_oldest_on_64_slot_overflow():
    queue = BoundedCallbackQueue(max_items=64, max_bytes=2048)
    for index in range(65):
        assert queue.put(CallbackEvent("FAKE", bytes([index])))
    assert queue.qsize() == 64
    assert queue.get().data == b"\x01"
