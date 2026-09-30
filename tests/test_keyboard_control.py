"""Terminal and headless window regressions for shared keyboard controls."""
import os
import pty
import select
import termios
import threading
import unittest
from unittest.mock import MagicMock, patch

from interfaces.keyboard_control import ControlTerminal, read_key, terminal_input


class KeyboardControlTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.previous = termios.tcgetattr(self.slave)
        self.addCleanup(os.close, self.slave)
        self.addCleanup(os.close, self.master)

    def test_key_without_newline_and_context_error_restores_terminal(self):
        with self.assertRaisesRegex(RuntimeError, "operator failure"):
            with terminal_input(True, self.slave):
                self.assertTrue(termios.tcgetattr(self.slave)[3] & termios.ISIG)
                self.assertEqual(read_key(self.slave), "")
                os.write(self.master, b"R")
                self.assertTrue(select.select([self.slave], [], [], 1)[0])
                self.assertEqual(read_key(self.slave), "r")
                raise RuntimeError("operator failure")
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)

    def test_terminal_dispatches_raw_controls_without_newlines(self):
        received = []
        changed = threading.Event()

        def receive(key):
            received.append(key)
            changed.set()

        terminal = ControlTerminal(receive, fd=self.slave)
        self.addCleanup(terminal.close)
        terminal.start()
        for key in (b"R", b"s", b"D"):
            changed.clear()
            os.write(self.master, key)
            self.assertTrue(changed.wait(1))
            self.assertEqual(received[-1], key.decode().lower())
        changed.clear()
        os.write(self.master, b" xX q")
        self.assertTrue(changed.wait(1))
        terminal.close()
        self.assertEqual(received, ["r", "s", "d", "q"])
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)

    def test_terminal_adjacent_keys_remain_individual_presses(self):
        received = []
        done = threading.Event()

        def receive(key):
            received.append(key)
            if key == "q":
                done.set()

        terminal = ControlTerminal(receive, fd=self.slave)
        self.addCleanup(terminal.close)
        terminal.start()
        os.write(self.master, b"rssdq")
        self.assertTrue(done.wait(1))
        terminal.close()
        self.assertEqual(received, ["r", "s", "s", "d", "q"])
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)

    def test_terminal_ignores_escape_sequences_without_quitting_or_discarding(self):
        received = []
        terminal = ControlTerminal(received.append, fd=self.slave)
        self.addCleanup(terminal.close)
        terminal.start()
        thread = terminal._thread
        os.write(self.master, b"\x1b[A\x1b[B\x1b[C\x1b[D\x1bOD\x1b[1;5D\x1b[15~\x1b[[D\x1brdq")
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(received, ["r", "d", "q"])
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)

    def test_terminal_eof_does_not_drop_a_preceding_press(self):
        reader, writer = os.pipe()
        self.addCleanup(os.close, reader)
        os.write(writer, b"s")
        os.close(writer)
        received = []
        terminal = ControlTerminal(received.append, fd=reader)
        self.addCleanup(terminal.close)
        terminal.start()
        thread = terminal._thread
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(received, ["s"])

    def test_viewer_normalizes_logical_presses_and_ignores_obsolete_keys(self):
        from simulation.viewer_window import ViewerWindow

        received = []
        window = ViewerWindow(headless=True, joint_control=received.append)
        self.addCleanup(window.close)
        for key in ("R", "s", ord("D"), "KEY_R", "s", "s", "d",
                    " ", 32, "KEY_SPACE", "space", "x", "r+s", "s+d", "q", "q", "r"):
            window.on_key(key)
        self.assertEqual(received, ["r", "s", "d", "r", "s", "s", "d", "q"])

    def test_viewer_exit_notifies_only_once(self):
        from simulation.viewer_window import ViewerWindow

        for exit_key in ("q", "Q", "escape", "KEY_ESCAPE", "esc", 27, 256):
            with self.subTest(exit_key=exit_key):
                received = []
                shutdown = []
                window = ViewerWindow(
                    headless=True, joint_control=received.append,
                    shutdown=lambda: shutdown.append(True),
                )
                self.addCleanup(window.close)
                window.on_key(exit_key)
                window.on_key(exit_key)
                window.close()
                window.on_key("r")
                self.assertEqual(received, ["q"])
                self.assertEqual(shutdown, [])

    def test_viewer_shutdown_fallback_and_programmatic_close(self):
        from simulation.viewer_window import ViewerWindow

        shutdown = []
        window = ViewerWindow(headless=True, shutdown=lambda: shutdown.append(True))
        self.addCleanup(window.close)
        window.on_key("r")
        window.on_key("escape")
        window.on_key("q")
        window.close()
        self.assertEqual(shutdown, [True])
        window = ViewerWindow(headless=True, shutdown=lambda: shutdown.append(True))
        window.close()
        window.on_key("q")
        self.assertEqual(shutdown, [True])

    def test_glfw_dispatches_only_press_and_notifies_close_once(self):
        import mujoco
        from simulation.split_view import SplitViewRenderer
        from simulation.viewer_window import ViewerWindow

        model = mujoco.MjModel.from_xml_string("<mujoco/>")
        received = []
        window = ViewerWindow(headless=True, joint_control=received.append)
        self.addCleanup(window.close)
        renderer = SplitViewRenderer(model, split_view=False, render_header=lambda *args: None,
                                     on_key=window.on_key)
        glfw = MagicMock(PRESS=1, RELEASE=0, REPEAT=2, KEY_Q=81, KEY_ESCAPE=256)
        glfw.get_cursor_pos.return_value = (0, 0)

        def poll_events():
            key_callback = glfw.set_key_callback.call_args.args[1]
            for key in (ord("R"), ord("S"), ord("D")):
                key_callback(None, key, 0, glfw.REPEAT, 0)
                key_callback(None, key, 0, glfw.RELEASE, 0)
                key_callback(None, key, 0, glfw.PRESS, 0)
            glfw.set_window_focus_callback.call_args.args[1](None, False)
            self.assertEqual(received, ["r", "s", "d"])
            key_callback(None, glfw.KEY_Q, 0, glfw.REPEAT, 0)
            key_callback(None, glfw.KEY_ESCAPE, 0, glfw.RELEASE, 0)
            self.assertEqual(received, ["r", "s", "d"])
            key_callback(None, glfw.KEY_Q, 0, glfw.PRESS, 0)
            glfw.set_window_close_callback.call_args.args[1](None)
            key_callback(None, glfw.KEY_Q, 0, glfw.PRESS, 0)

        glfw.poll_events.side_effect = poll_events
        with patch.dict("sys.modules", {"glfw": glfw}), \
                patch.object(mujoco, "MjrContext"), patch.object(mujoco, "mjr_setBuffer"):
            renderer._render_loop(model)
        self.assertEqual(received, ["r", "s", "d", "q"])

    def test_close_restores_terminal_and_joins_idle_reader(self):
        terminal = ControlTerminal(lambda key: None, fd=self.slave)
        self.addCleanup(terminal.close)
        terminal.start()
        thread = terminal._thread
        terminal.close()
        self.assertFalse(thread.is_alive())
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)
        terminal.close()

    def test_eof_restores_terminal_and_ends_reader(self):
        terminal = ControlTerminal(lambda key: None, fd=self.slave)
        self.addCleanup(terminal.close)
        terminal.start()
        thread = terminal._thread
        # Keep the PTY inspectable: canonical VEOF produces a zero-length read.
        attrs = termios.tcgetattr(self.slave)
        attrs[3] |= termios.ICANON
        termios.tcsetattr(self.slave, termios.TCSANOW, attrs)
        os.write(self.master, attrs[6][termios.VEOF])
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)

    def test_failed_reader_start_restores_terminal(self):
        terminal = ControlTerminal(lambda key: None, fd=self.slave)
        with patch.object(threading.Thread, "start", side_effect=RuntimeError("cannot start")):
            with self.assertRaisesRegex(RuntimeError, "cannot start"):
                terminal.start()
        terminal.close()
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)


if __name__ == "__main__":
    unittest.main()
