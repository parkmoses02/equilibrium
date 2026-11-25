"""Snapshot-driven command routing for double_upright; no serial/GUI dependency."""
import math

POSES = {"r": "DD", "e": "DU", "w": "UD", "q": "UU"}
STATES = {"ZERO", "OFF", "READY", "ARM", "SWING", "HOLD", "WAIT", "DAMP", "BRAKE"}


class ConsoleState:
    def __init__(self):
        self.phase, self.src, self.dest = "", "-", "-"
        self.pending = None
        self.values = None
        self.last = -math.inf

    def feed(self, line, now):
        if not line.startswith("DP1,"):
            if (line.startswith(("STOP_CODE=", "OFF", "BLOCK", "rst:", "Saved DD angles"))
                    or " refused:" in line or " not started:" in line):
                self.__init__()  # wait for a fresh authoritative snapshot
            return False
        try:
            _, stamp, phase, src, dest, *fields = line.split(",")
            values = tuple(map(float, fields))
            if (not 0 <= int(stamp) < 2**32 or phase not in STATES or len(values) != 7
                    or not all(map(math.isfinite, values))
                    or src not in (*POSES, "-") or dest not in (*POSES, "-")):
                return False
            if phase in ("ARM", "SWING") and (src not in POSES or dest not in POSES or src == dest):
                return False
            if phase == "HOLD" and src not in ("e", "w", "q"):
                return False
            if phase in ("READY", "WAIT") and src != "r":
                return False
            if phase in ("READY", "WAIT", "HOLD", "OFF", "ZERO") and dest != "-":
                return False
        except ValueError:
            return False
        if (phase, src) != (self.phase, self.src):
            self.pending = None
        self.phase, self.src, self.dest = phase, src, dest
        self.values, self.last = values, now
        return True

    def fresh(self, now):
        return 0 <= now - self.last < 0.6

    def command(self, dest, now):
        if not self.fresh(now) or self.pending:
            return None
        if dest == "z":
            return "z" if self.phase in ("ZERO", "OFF", "READY") else None
        if self.phase not in ("READY", "WAIT", "HOLD") or dest not in POSES or dest == self.src:
            return None
        return ("r" if self.phase == "READY" else "") + dest

    def request(self, dest, now):
        cmd = self.command(dest, now)
        if cmd:
            self.pending = dest
        return cmd
