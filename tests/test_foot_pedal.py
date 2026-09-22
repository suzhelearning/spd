"""Pedal gesture and evdev teardown regressions without input hardware."""
from contextlib import contextmanager
import errno
import os
from pathlib import Path
import stat
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from interfaces import foot_pedal as pedal


def event_bytes(event_type, code, value, timestamp_ns):
    seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
    return pedal.INPUT_EVENT.pack(seconds, nanoseconds // 1_000, event_type, code, value)


class PedalGestureTests(unittest.TestCase):
    def test_three_distinct_legal_key_codes_are_required(self):
        self.assertEqual(pedal.parse_pedal_keys(' 30, 31, 32 '), (30, 31, 32))
        for text in ('37,25,37', '0,25,48', '37,25,768', '-1,25,48',
                     '37,25', '37,25,48,49', 'k,p,b', '37,,48'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                pedal.parse_pedal_keys(text)
        with self.assertRaises(ValueError):
            pedal.PedalGesture(keys=(True, 25, 48))

    def test_binary_events_emit_only_on_full_release(self):
        gesture = pedal.PedalGesture()
        data = b''.join(event_bytes(pedal.EV_KEY, code, value, stamp) for code, value, stamp in (
            (37, 1, 1_000_000), (37, 2, 2_000_000), (37, 0, 3_000_000),
            (25, 1, 4_000_000), (25, 0, 5_000_000)))
        self.assertEqual([gesture.feed(*event) for event in pedal.decode_events(data)],
                         [None, None, 'checkpoint', None, 'pause_toggle'])

    def test_right_short_and_exact_threshold_long_never_revert_before_release(self):
        for duration_ns, expected in ((999_999_000, 'revert'), (1_000_000_000, 'skip')):
            with self.subTest(duration_ns=duration_ns):
                gesture = pedal.PedalGesture()
                self.assertIsNone(gesture.feed(pedal.EV_KEY, 48, 1, 10_000_000_000))
                # A repeated down must not reset the press start; autorepeat
                # cannot emit either operation, even at the long threshold.
                self.assertIsNone(gesture.feed(pedal.EV_KEY, 48, 1, 10_500_000_000))
                self.assertIsNone(gesture.feed(pedal.EV_KEY, 48, 2, 10_750_000_000))
                self.assertEqual(gesture.feed(pedal.EV_KEY, 48, 0,
                                              10_000_000_000 + duration_ns), expected)
                self.assertIsNone(gesture.feed(pedal.EV_KEY, 48, 0, 12_000_000_000))

    def test_initially_held_and_missing_down_cannot_act(self):
        gesture = pedal.PedalGesture(held=(48,))
        for code, value in ((48, 1), (48, 2), (48, 0), (37, 0), (25, 2), (25, 0)):
            self.assertIsNone(gesture.feed(pedal.EV_KEY, code, value, 1_000_000))
        self.assertIsNone(gesture.feed(pedal.EV_KEY, 48, 1, 2_000_000))
        self.assertEqual(gesture.feed(pedal.EV_KEY, 48, 0, 3_000_000), 'revert')

    def test_unconfigured_keys_and_nonkey_events_do_not_create_gestures(self):
        gesture = pedal.PedalGesture(keys=(30, 31, 32))
        for event in ((pedal.EV_KEY, 48, 1, 0), (pedal.EV_KEY, 48, 0, 1),
                      (2, 30, 1, 2), (pedal.EV_KEY, 30, 0, 3)):
            self.assertIsNone(gesture.feed(*event))
        self.assertIsNone(gesture.feed(pedal.EV_KEY, 31, 1, 4))
        self.assertEqual(gesture.feed(pedal.EV_KEY, 31, 0, 5), 'pause_toggle')

    def test_event_loss_permanently_cancels_pending_gesture(self):
        gesture = pedal.PedalGesture()
        gesture.feed(pedal.EV_KEY, 48, 1, 0)
        with self.assertRaises(OSError):
            gesture.feed(pedal.EV_SYN, pedal.SYN_DROPPED, 0, 1)
        for event in ((pedal.EV_KEY, 48, 0, 2), (pedal.EV_KEY, 48, 1, 3),
                      (pedal.EV_KEY, 48, 0, 4)):
            self.assertIsNone(gesture.feed(*event))

    def test_backwards_clock_cancels_instead_of_reverting(self):
        gesture = pedal.PedalGesture()
        gesture.feed(pedal.EV_KEY, 48, 1, 10)
        with self.assertRaises(OSError):
            gesture.feed(pedal.EV_KEY, 48, 0, 9)
        self.assertIsNone(gesture.feed(pedal.EV_KEY, 37, 1, 11))
        self.assertIsNone(gesture.feed(pedal.EV_KEY, 37, 0, 12))

    def test_incomplete_binary_event_is_rejected(self):
        with self.assertRaises(OSError):
            list(pedal.decode_events(b'\x00' * (pedal.INPUT_EVENT.size - 1)))


class FootPedalReaderTests(unittest.TestCase):
    @contextmanager
    def fake_device(self, *, held=(), supported=pedal.DEFAULT_KEYS, failed_request=None):
        """Use a real pipe reader with only device-opening/ioctls substituted."""
        reader, writer = os.pipe2(os.O_NONBLOCK | os.O_CLOEXEC)
        operations, errors, grabs = [], [], []
        failed = threading.Event()
        received = threading.Event()

        def ioctl(fd, request, argument, mutate=True):
            if request == failed_request:
                raise OSError(errno.EACCES, 'injected ioctl failure')
            if request == pedal._EVIOCGRAB:
                grabs.append(argument)
            elif request in (pedal._EVIOCGBIT_TYPES, pedal._EVIOCGBIT_KEYS, pedal._EVIOCGKEY):
                codes = ((pedal.EV_KEY,) if request == pedal._EVIOCGBIT_TYPES else
                         supported if request == pedal._EVIOCGBIT_KEYS else held)
                for code in codes:
                    argument[code // 8] |= 1 << (code % 8)
            return 0

        def callback(operation):
            operations.append(operation)
            received.set()

        def on_error(message):
            errors.append(message)
            failed.set()

        device = pedal.FootPedal(Path('/fake/pedal'), callback, on_error)
        try:
            with patch.object(pedal.os, 'open', return_value=reader), \
                    patch.object(pedal.os, 'fstat', return_value=SimpleNamespace(st_mode=stat.S_IFCHR)), \
                    patch.object(pedal.fcntl, 'ioctl', side_effect=ioctl):
                try:
                    yield device, writer, operations, errors, grabs, failed, received
                finally:
                    device.close()
                    device.close()
            with self.assertRaises(OSError) as closed:
                os.fstat(reader)
            self.assertEqual(closed.exception.errno, errno.EBADF)
        finally:
            try:
                os.close(writer)
            except OSError as exc:
                if exc.errno != errno.EBADF:
                    raise

    def test_dropped_packet_cancels_release_and_releases_grab(self):
        with self.fake_device() as (device, writer, operations, errors, grabs, failed, _):
            device.start()
            stamp = time.monotonic_ns() + 1_000_000
            os.write(writer, b''.join((
                event_bytes(pedal.EV_KEY, 48, 1, stamp),
                event_bytes(pedal.EV_SYN, pedal.SYN_DROPPED, 0, stamp + 1_000),
                event_bytes(pedal.EV_KEY, 48, 0, stamp + 2_000))))
            self.assertTrue(failed.wait(1.0), 'reader did not report lost events')
            device.close()
            self.assertEqual(operations, [])
            self.assertEqual(len(errors), 1)
            self.assertEqual(grabs, [1, 0])

    def test_disconnect_cancels_pending_press_and_reports_once(self):
        with self.fake_device() as (device, writer, operations, errors, grabs, failed, _):
            device.start()
            stamp = time.monotonic_ns() + 1_000_000
            os.write(writer, event_bytes(pedal.EV_KEY, 48, 1, stamp))
            os.close(writer)
            self.assertTrue(failed.wait(1.0), 'reader did not report disconnect')
            device.close()
            self.assertEqual(operations, [])
            self.assertEqual(len(errors), 1)
            self.assertEqual(grabs, [1, 0])

    def test_initial_held_state_ignores_release_then_accepts_new_cycle(self):
        with self.fake_device(held=(48,)) as (device, writer, operations, errors, grabs, _, received):
            device.start()
            stamp = time.monotonic_ns() + 1_000_000
            os.write(writer, b''.join(event_bytes(pedal.EV_KEY, code, value, stamp + offset)
                                     for code, value, offset in (
                                         (48, 0, 0), (48, 1, 1_000), (48, 0, 2_000))))
            self.assertTrue(received.wait(1.0), 'reader did not emit the fresh cycle')
            device.close()
            self.assertEqual(operations, ['revert'])
            self.assertEqual(errors, [])
            self.assertEqual(grabs, [1, 0])

    def test_queued_prestart_cycle_cannot_act(self):
        with self.fake_device() as (device, writer, operations, errors, _, _, received):
            old_stamp = time.monotonic_ns() - 1_000_000_000
            os.write(writer, b''.join((
                event_bytes(pedal.EV_KEY, 48, 1, old_stamp),
                event_bytes(pedal.EV_KEY, 48, 0, old_stamp + 1_000))))
            device.start()
            stamp = time.monotonic_ns() + 1_000_000
            os.write(writer, b''.join((
                event_bytes(pedal.EV_KEY, 25, 1, stamp),
                event_bytes(pedal.EV_KEY, 25, 0, stamp + 1_000))))
            self.assertTrue(received.wait(1.0), 'reader did not emit the fresh cycle')
            device.close()
            self.assertEqual(operations, ['pause_toggle'])
            self.assertEqual(errors, [])

    def test_unsupported_key_fails_before_grab(self):
        with self.fake_device(supported=(37, 25)) as (device, _, operations, errors, grabs, _, _):
            with self.assertRaises(ValueError):
                device.start()
            self.assertEqual(grabs, [])
            self.assertEqual(operations, [])
            self.assertEqual(errors, [])

    def test_setup_failure_after_grab_releases_it(self):
        with self.fake_device(failed_request=pedal._EVIOCGKEY) as (device, _, _, errors, grabs, _, _):
            with self.assertRaises(OSError):
                device.start()
            self.assertEqual(grabs, [1, 0])
            self.assertEqual(errors, [])

    def test_thread_start_failure_releases_grab_and_descriptor(self):
        with self.fake_device() as (device, _, _, errors, grabs, _, _):
            with patch.object(pedal.threading.Thread, 'start', side_effect=RuntimeError('cannot start')):
                with self.assertRaises(RuntimeError):
                    device.start()
            self.assertEqual(grabs, [1, 0])
            self.assertEqual(errors, [])

    def test_regular_file_is_rejected_without_leaking_descriptor(self):
        with tempfile.NamedTemporaryFile() as source:
            opened = []
            real_open = os.open

            def tracking_open(*args):
                fd = real_open(*args)
                opened.append(fd)
                return fd

            device = pedal.FootPedal(Path(source.name), lambda _: None, lambda _: None)
            with patch.object(pedal.os, 'open', side_effect=tracking_open):
                with self.assertRaises(ValueError):
                    device.start()
            device.close()
            with self.assertRaises(OSError) as closed:
                os.fstat(opened[0])
            self.assertEqual(closed.exception.errno, errno.EBADF)


if __name__ == '__main__':
    unittest.main()
