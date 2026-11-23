"""Decode recorded Rev.L poses. Angles are absolute, in radians from upright."""
from collections import Counter
from dataclasses import dataclass, field
import math
from typing import NamedTuple


TARGET = {"q": (0, 0), "w": (0, math.pi), "e": (math.pi, 0), "r": (math.pi, math.pi)}
PHASE = {"q": "UU", "w": "UD", "e": "DU", "r": "DD", "s": "DD → DU", "u": "DD → UU"}


class Sample(NamedTuple):
    us: int
    mode: str
    x: float
    base: float
    tip: float


@dataclass
class Recording:
    samples: list
    reason: str
    times: list = field(init=False)

    def __post_init__(self):
        start = self.samples[0].us
        self.times = [p.us - start for p in self.samples]


def clock_offset(poses, swing):
    # POSE and CSV use the same micros() snapshot but different time origins.
    # At a shared tick the ramp position is the same, rounded to 5 and 6 places.
    offsets = Counter()
    for p in poses:
        if p.mode in ("s", "u"):
            offsets.update(
                p.us - s.us for s in swing
                if abs(p.x - s.x) <= 0.000006
                and abs(math.remainder(p.base - s.base, 2*math.pi)) < .02
                and abs(math.remainder(p.tip - s.tip, 2*math.pi)) < .02)
    return offsets.most_common(1)[0][0] if offsets else None


def load_recordings(path):
    runs = []
    poses, swing, trace = [], [], []
    selected, caught, reason = "q", False, ""
    swing_phase = "s"
    columns = {}
    section = ""

    def finish():
        nonlocal caught, reason, section, columns
        # When the clock offset is unavailable keep the continuous POSE recording.
        offset = clock_offset(poses, swing) if poses and swing else None
        dense_swing = swing if offset is not None or not poses else []
        frames = {p.us: p for p in poses}
        for p in dense_swing:
            t = p.us + (offset or 0)
            frames[t] = p._replace(us=t)
        if trace:
            # Hold-only traces count from arm. After a catch, trace time zero
            # is the last swing frame. Each trace is finer than the POSE stream.
            if not caught or not frames:
                frames = {s.us: s for s in trace}
            elif dense_swing:
                start = dense_swing[-1].us + (offset or 0)
                frames.update({start+s.us: s._replace(us=start+s.us) for s in trace})
        if frames:
            runs.append(Recording([frames[t] for t in sorted(frames)], reason))
        poses.clear()
        swing.clear()
        trace.clear()
        caught, reason, section, columns = False, "", "", {}

    with path.open(encoding="utf-8-sig", errors="replace") as stream:
        for raw in stream:
            line = raw.strip()
            if not line:
                continue
            if line.startswith(("ACTIVE ", "rst:")):
                finish()
                swing_phase = "u" if line.startswith("ACTIVE DD->UU") else "s"
                if line.startswith("rst:"):
                    selected = "q"
            elif line.startswith("Selected ") and line[9:10] in TARGET:
                selected = line[9]
            elif line in ("CATCH DU -> e", "CATCH UU -> q"):
                caught, selected = True, line[-1]
            elif line == "OFF" or line.startswith("OFF:"):
                if not reason:
                    reason = line
                section = ""
            elif "POSE," in line:
                try:
                    _, stamp, mode, x, base, tip = line[line.index("POSE,"):].split(",")
                    t = int(stamp)
                    if poses:
                        t = poses[-1].us + (t - poses[-1].us) % (1 << 32)
                    sample = Sample(t, mode, float(x), float(base), float(tip))
                    if mode in PHASE and all(map(math.isfinite, sample[2:])):
                        poses.append(sample)
                        if mode in TARGET:
                            selected = mode
                except ValueError:
                    continue
            elif line in ("RQ_CSV_BEGIN", "RE_CSV_BEGIN"):
                swing_phase = "u" if line.startswith("RQ_") else "s"
            elif line.startswith(("step,t_us,", "elapsed_us,dt_us,")):
                if line.startswith("step,t_us,") and swing:
                    finish()  # Concatenated CSV exports may omit ACTIVE markers.
                columns = {name: i for i, name in enumerate(line.split(","))}
                section = "swing" if "t_us" in columns else "trace"
            elif line in ("RE_CSV_END", "RQ_CSV_END"):
                section = ""
            elif section and line[0].isdigit():
                parts = line.split(",")
                try:
                    if section == "swing":
                        t, x, a, b = [parts[columns[k]] for k in
                                      ("t_us", "ramp_x_m", "base_rad", "tip_rad")]
                        sample = Sample(int(t), swing_phase, float(x), float(a), float(b))
                        dest = swing
                    else:
                        t, x, a, b = [parts[columns[k]] for k in
                                      ("elapsed_us", "ramp_x_mm", "base_error_deg", "tip_error_deg")]
                        base, tip = TARGET[selected]
                        sample = Sample(int(t), selected, float(x)/1000,
                                        base+math.radians(float(a)), tip+math.radians(float(b)))
                        dest = trace
                    if all(map(math.isfinite, sample[2:])):
                        dest.append(sample)
                except (ValueError, IndexError, KeyError):
                    continue
    finish()
    return runs
