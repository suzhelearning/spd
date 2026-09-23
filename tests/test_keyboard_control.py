"""Real terminal regressions for immediate, shared keyboard controls."""
import os
import pty
import select
import termios
import threading
import unittest
from unittest.mock import patch

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


    def test_close_restores_terminal_and_joins_idle_reader(self):
        terminal = ControlTerminal(lambda key: None, lambda operation: None, fd=self.slave)
        self.addCleanup(terminal.close)
        terminal.start()
        thread = terminal._thread
        terminal.close()
        self.assertFalse(thread.is_alive())
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)
        terminal.close()

    def test_eof_restores_terminal_and_ends_reader(self):
        terminal = ControlTerminal(lambda key: None, lambda operation: None, fd=self.slave)
        self.addCleanup(terminal.close)
        terminal.start()
        thread = terminal._thread
        # A PTY hangup makes tcgetattr fail, so produce a real zero-length tty
        # read with canonical VEOF instead, keeping the descriptor inspectable.
        attrs = termios.tcgetattr(self.slave)
        attrs[3] |= termios.ICANON
        termios.tcsetattr(self.slave, termios.TCSANOW, attrs)
        os.write(self.master, attrs[6][termios.VEOF])
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)

    def test_failed_reader_start_restores_terminal(self):
        terminal = ControlTerminal(lambda key: None, lambda operation: None, fd=self.slave)
        with patch.object(threading.Thread, "start", side_effect=RuntimeError("cannot start")):
            with self.assertRaisesRegex(RuntimeError, "cannot start"):
                terminal.start()
        terminal.close()
        self.assertEqual(termios.tcgetattr(self.slave), self.previous)



if __name__ == "__main__":
    unittest.main()
