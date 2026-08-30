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
    PXREA_CALLBACK_MASK,
    PXREA_DEVICE_CONNECT,
    PXREA_DEVICE_CUSTOM,
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
    def __init__(self, init_result=0, deinit_result=0):
        self.PXREAInit = FakeFn(init_result)
        self.PXREADeinit = FakeFn(deinit_result)
        self.PXREASendCustomMessage = FakeFn(0)


def test_custom_message_matches_sdk_abi():
    assert PXREADevCustomMessage.devID.offset == 0
    assert PXREADevCustomMessage.dataSize.offset == 32
    assert PXREADevCustomMessage.dataPtr.offset == 40
    assert ctypes.sizeof(PXREADevCustomMessage) == 48


def test_init_uses_custom_and_lifecycle_callback_mask():
    lib = FakeLibrary()
    client = PXREAClient(lib)
    assert client.flags == PXREA_CALLBACK_MASK
    assert PXREA_CALLBACK_MASK & PXREA_DEVICE_CUSTOM
    with client:
        assert lib.PXREAInit.calls[0][2] == PXREA_CALLBACK_MASK


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
    message = PXREADevCustomMessage(
        b"FAKE", 2049, ctypes.cast(oversized, ctypes.POINTER(ctypes.c_char))
    )
    client._callback(None, PXREA_DEVICE_CUSTOM, 0, ctypes.byref(message))
    assert queue.qsize() == 0
    assert client.status()["dropped_oversize"] == 1
    client._callback(None, PXREA_DEVICE_CONNECT, 0, None)
    lifecycle = queue.get()
    assert lifecycle is not None
    assert lifecycle.event_type == PXREA_DEVICE_CONNECT
def test_queue_drops_oldest_on_64_slot_overflow():
    queue = BoundedCallbackQueue(max_items=64, max_bytes=2048)
    for index in range(65):
        assert queue.put(CallbackEvent("FAKE", bytes([index])))
    assert queue.qsize() == 64
    assert queue.get().data == b"\x01"
def test_deinit_failure_keeps_client_open_for_retry():
    lib = FakeLibrary(deinit_result=9)
    client = PXREAClient(lib)
    client.__enter__()
    with pytest.raises(PXREAError, match="PXREADeinit failed: 9"):
        client.close()
    assert client.callback is not None
    assert client._initialized is True
    lib.PXREADeinit.result = 0
    client.close()
    assert client.callback is None
    assert len(lib.PXREADeinit.calls) == 2
