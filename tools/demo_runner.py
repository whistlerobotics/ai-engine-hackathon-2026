"""Drive the existing policy runner's terminal controls through a PTY."""

import codecs
from collections import deque
import errno
import fcntl
import os
import pty
import re
import signal
import struct
import subprocess
import termios
from threading import Thread
from time import monotonic, sleep


ROWS, COLUMNS = 35, 120
CSI = re.compile(r"\x1b\[([0-?]*)([ -/]*)([@-~])")
PHASES = {
    "idle": "IDLE",
    "moving to the start pose": "HOMING",
    "at the start pose": "HOMED",
    "running an episode": "REPLAYING",
}


class RunnerScreen:
    def __init__(self):
        self.active = False
        self.rows = [list(" " * COLUMNS) for _ in range(ROWS)]
        self.row = self.column = 0
        self.pending = ""
        self.plain = ""

    def feed(self, chunk):
        data = self.pending + chunk
        self.pending = ""
        i = 0
        while i < len(data):
            char = data[i]
            if char == "\x1b":
                match = CSI.match(data, i)
                if match:
                    self.control(match[1], match[3])
                    i = match.end()
                    continue
                if data[i:] == "\x1b" or data.startswith("\x1b[", i):
                    self.pending = data[i:]
                    break
                i += 2
                continue
            if not self.active:
                self.plain = (self.plain + char)[-8192:]
            elif char == "\r":
                self.column = 0
            elif char == "\n":
                self.row += 1
            elif char >= " ":
                if self.row < ROWS and self.column < COLUMNS:
                    self.rows[self.row][self.column] = char
                self.column += 1
            i += 1

    def control(self, args, command):
        if args == "?1049":
            self.active = command == "h"
            self.plain = ""
            return
        if command not in "HfGCJ" or args.startswith(("?", ">")):
            return
        fields = args.split(";")
        first = int(fields[0] or "1")
        if command in "Hf":
            self.row = max(0, first - 1)
            self.column = max(0, int(fields[1] or "1") - 1) if len(fields) > 1 else 0
        elif command == "G":
            self.column = max(0, first - 1)
        elif command == "C":
            self.column += first
        elif command == "J" and first == 2:
            self.rows = [list(" " * COLUMNS) for _ in range(ROWS)]

    def phase(self):
        if not self.active:
            return None
        header = "".join(self.rows[2])
        if "REAL ARM" not in header:
            return None
        row = "".join(self.rows[5])
        label = re.search(r"\bPhase\s+([^│]+)", row)
        if label is None:
            return None
        phase = PHASES.get(label[1].strip())
        if phase == "IDLE" and not re.search(r"\bArm\s+●\s+live\b", row):
            return None
        return phase


class RealRunner:
    def __init__(self, command, log, cwd, env):
        master, slave = pty.openpty()
        try:
            fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", ROWS, COLUMNS, 0, 0))
            self.process = subprocess.Popen(
                command, cwd=cwd, env={**env, "TERM": "xterm-256color"},
                stdin=slave, stdout=slave, stderr=slave, start_new_session=True,
            )
        except BaseException:
            os.close(master)
            raise
        finally:
            os.close(slave)
        self.master = master
        self.events = deque()
        self.screen = RunnerScreen()
        self.reader = Thread(target=self.read, args=(log,), daemon=True)
        self.reader.start()

    def read(self, log):
        decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        last_phase = None
        with log.open("wb") as stream:
            while True:
                try:
                    chunk = os.read(self.master, 8192)
                except OSError as error:
                    if error.errno != errno.EIO:
                        raise
                    break
                if not chunk:
                    break
                stream.write(chunk)
                stream.flush()
                self.screen.feed(decoder.decode(chunk))
                phase = self.screen.phase()
                if phase is not None and phase != last_phase:
                    self.events.append(phase)
                    last_phase = phase
        self.events.extend(line.strip() for line in self.screen.plain.splitlines())

    def lines(self):
        while self.events:
            yield self.events.popleft()

    def send(self, command):
        key = {"home": b"\r", "run": b"\r", "quit": b"q", "cancel": b"\x03"}[command]
        os.write(self.master, key)

    def stop(self):
        if self.process.poll() is None:
            deadline = monotonic() + 4
            try:
                self.send("cancel")
                while self.process.poll() is None and monotonic() < deadline:
                    if self.screen.phase() in {"IDLE", "HOMED"}:
                        self.send("quit")
                    sleep(0.05)
            except OSError:
                pass
            if self.process.poll() is None:
                self.signal_group(signal.SIGTERM)
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.signal_group(signal.SIGKILL)
                    self.process.wait()
        self.reader.join(timeout=1)
        os.close(self.master)

    def signal_group(self, signum):
        try:
            os.killpg(self.process.pid, signum)
        except ProcessLookupError:
            pass
