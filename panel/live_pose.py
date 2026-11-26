"""Compare the firmware's planned motion with the live PlatformIO monitor log."""

from collections import deque
import argparse
import math
import re
import subprocess
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk


FRAME_US = 5_000
ZERO_POSE = (0.0, math.pi, math.pi)
POSE_CODES = dict(zip("rqwe", ("DD", "UU", "UD", "DU")))
ROUTES = {
    "s": ("DD", "DU"), "u": ("DD", "UU"), "d": ("DD", "UD"),
    "t": ("UD", "DU"), "v": ("DU", "UD"), "y": ("UU", "UD"),
    "z": ("UU", "DU"), "k": ("DU", "UU"), "h": ("UD", "UU"),
}
START = re.compile(r"(DD|UU|UD|DU)->(DD|UU|UD|DU) start x_mm=([-+0-9.]+)")
SWING_START = re.compile(r"ACTIVE DD->(UU|UD|DU)\b")
TELEMETRY = re.compile(r"\b(POSE|DP1),[^\s]+")
FRAME = re.compile(r"^\s*\{([^{}]+)\},?\s*$", re.MULTILINE)


def load_trajectories(project):
    folder = project / "src" / "double_pendulum"
    routes = list(ROUTES.values()) + [(source, "DD") for source in ("UU", "UD", "DU")]
    trajectories = {}
    for source, target in routes:
        stem = target if source == "DD" else f"{source}_{target}"
        header = folder / f"{stem.lower()}_frames.h"
        frames = [tuple(float(value.strip().rstrip("f")) for value in fields.split(","))
                  for fields in FRAME.findall(header.read_text(encoding="utf-8"))]
        if len(frames) < 2 or any(len(frame) != 11 for frame in frames):
            raise ValueError(f"기준 궤적을 읽을 수 없습니다. {header}")
        trajectories[source, target] = frames
    return trajectories


class Comparison:
    def __init__(self, trajectories):
        self.trajectories = trajectories
        self.reset()

    def reset(self, zeroed=False):
        self.reference = self.actual = ZERO_POSE if zeroed else None
        self.status = "영점 설정 완료 · DD" if zeroed else "새 자세 데이터 대기"
        self.reference_status = "영점 · DD" if zeroed else "전환 시작 대기"
        self.last_hold = "DD" if zeroed else None
        self.route = self.start_us = self.origin = None
        self.last_data = None
        self.controller = None
        self.frame_index = 0

    def start(self, route, origin=None):
        self.route, self.origin, self.start_us = route, origin, None
        self.frame_index = 0
        self.controller = None
        self.reference = self.trajectories[route][0][:3]
        if origin is not None:
            self.reference = (self.reference[0] + origin, *self.reference[1:])
        self.reference_status = " → ".join(route) + " · 시작"

    def advance(self, stamp, measured_x):
        if self.start_us is None:
            self.start_us = stamp
            if self.origin is None:
                self.origin = measured_x
        frames = self.trajectories[self.route]
        # ESP32 micros() is an unsigned 32-bit clock and wraps during long sessions.
        elapsed = (stamp - self.start_us) & 0xFFFFFFFF
        frame = min(elapsed / FRAME_US, len(frames) - 1)
        lower = int(frame)
        self.frame_index = lower
        x, base, tip = frames[lower][:3]
        if lower + 1 < len(frames):
            next_x, next_base, next_tip = frames[lower + 1][:3]
            fraction = frame - lower
            x += fraction * (next_x - x)
            base += fraction * math.remainder(next_base - base, math.tau)
            tip += fraction * math.remainder(next_tip - tip, math.tau)
        self.reference = (self.origin + x, base, tip)
        label = " → ".join(self.route)
        self.reference_status = f"{label} · {frame * FRAME_US / 1e6:.2f} s"
        if lower == len(frames) - 1:
            self.reference_status += " · 계획 종료"

    def finish(self, caught=False):
        if self.route and caught:
            self.controller = self.route[1]
            x, base, tip = self.trajectories[self.route][-1][:3]
            self.reference = ((self.origin if self.origin is not None else 0.0) + x, base, tip)
            self.reference_status = " → ".join(self.route) + " · 계획 종료"
        elif self.route and not self.reference_status.endswith("계획 종료"):
            self.reference_status += " · 정지"
        self.route = None

    def feed(self, line):
        if "Saved DD angles and cart ramp origin" in line:
            self.reset(zeroed=True)
            return
        if line.startswith("rst:"):
            self.reset()
            return
        match = START.search(line)
        if match:
            route = match.group(1, 2)
            if route in self.trajectories:
                self.start(route, float(match.group(3)) / 1000.0)
            return
        match = SWING_START.search(line)
        if match:
            origin = self.actual[0] if self.last_hold == "DD" and self.actual is not None else None
            self.start(("DD", match[1]), origin)
            return
        if line.startswith("CATCH "):
            self.finish(caught=True)
            return
        if line == "OFF" or line.startswith(("OFF:", "STOP_CODE=")):
            self.finish()
            self.status = line
            self.controller = None
            return
        match = TELEMETRY.search(line)
        if not match:
            return
        fields = match[0].split(",")
        try:
            stamp = int(fields[1])
            if fields[0] == "POSE" and len(fields) == 6:
                code = fields[2]
                pose = tuple(map(float, fields[3:6]))
                hold = POSE_CODES.get(code)
                active_hold = hold
                route = ROUTES.get(code)
                if code == "n":
                    route = self.route if self.route and self.route[1] == "DD" else (self.last_hold, "DD")
                label = hold or (" → ".join(route) if route in self.trajectories else code)
            elif fields[0] == "DP1" and len(fields) == 12:
                phase, source, target = fields[2:5]
                if phase == "ZERO":
                    self.reset()
                    self.status = "영점 설정 대기"
                    return
                pose = tuple(map(float, fields[5:8]))
                hold = POSE_CODES.get(source) if phase in {"HOLD", "READY", "WAIT", "ARM"} else None
                active_hold = POSE_CODES.get(source) if phase in {"HOLD", "WAIT", "ARM", "DAMP"} else None
                route = (POSE_CODES.get(source), POSE_CODES.get(target)) if phase == "SWING" else None
                label = " → ".join(route) if route in self.trajectories else phase
            else:
                return
        except (ValueError, IndexError):
            return
        if not all(map(math.isfinite, pose)):
            return
        self.actual = pose
        self.status = label
        self.last_data = time.monotonic()
        if route in self.trajectories:
            if route != self.route:
                self.start(route)
            self.advance(stamp, pose[0])
        elif hold:
            self.finish(caught=True)
            self.last_hold = hold
        if route not in self.trajectories:
            self.controller = active_hold
            if fields[0] == "DP1" and phase in {"OFF", "DAMP"}:
                self.finish()


# Normalized design weights (check_model.py, dd-uu/design.py and dd-du/model.js).
# Swing design multiplies both Q and R by the 5 ms sample period.
BASE_Q = (12, 60, 60, 2, 2, 2)
HOLD_WEIGHTS = {"DU": ((220, 500, 900, 60, 8, 4), 1),
                "UD": (BASE_Q, 1), "DD": (BASE_Q, 1), "UU": ((4, *BASE_Q), 0.3)}
STATES = ("x   카트 위치", "θ₁   첫 링크 각도", "θ₂   끝 링크 각도",
          "v   카트 속도", "ω₁   첫 링크 각속도", "ω₂   끝 링크 각속도")


def load_hold_gains(project):
    source = (project / "src/double_pendulum/upright.h").read_text(encoding="utf-8")
    gains = {}
    for code, pose in POSE_CODES.items():
        match = re.search(rf"\bK_{code}\s*\[\s*\]\s*=\s*\{{([^}}]+)\}}", source)
        if not match:
            raise ValueError(f"{pose} 홀드 게인을 찾을 수 없습니다.")
        row = tuple(float(v.strip().rstrip("f")) for v in match[1].split(","))
        if len(row) != 6:
            raise ValueError(f"{pose} 홀드 게인 차원이 다릅니다.")
        gains[pose] = row
    integral = re.search(r"\bKI_Q\s*=\s*([-\d.eE+]+)f?\s*;", source)
    if not integral:
        raise ValueError("UU 위치 적분 게인을 찾을 수 없습니다.")
    gains["UU"] = (float(integral[1]), *gains["UU"])
    return gains


class GainPanel:
    def __init__(self, parent, gains):
        self.gains = gains
        card = ttk.Frame(parent, style="Card.TFrame", padding=14)
        card.grid(row=0, column=0, sticky="nsew", padx=5)
        ttk.Label(card, text="제어 가중치와 게인", style="Heading.TLabel").pack(anchor="w")
        self.mode = ttk.Label(card, style="Status.TLabel")
        self.mode.pack(anchor="w", pady=(3, 14))
        table = ttk.Frame(card, style="Card.TFrame")
        table.pack(fill="x")
        table.columnconfigure(0, weight=1)
        self.cells = []
        for j, title in enumerate(("상태 · K 열 순서", "Q 대각값")):
            ttk.Label(table, text=title, style="Status.TLabel").grid(row=0, column=j, sticky="w")
        for i in range(7):
            cells = []
            for j in range(2):
                label = ttk.Label(table, style="Readout.TLabel")
                label.grid(row=i+1, column=j, sticky="w", pady=2)
                cells.append(label)
            self.cells.append(cells)
        self.r = ttk.Label(card, style="Readout.TLabel")
        self.r.pack(anchor="w", pady=(12, 8))
        self.k_title = ttk.Label(card, style="Status.TLabel")
        self.k_title.pack(anchor="w")
        self.k = ttk.Label(card, style="Readout.TLabel", font=("TkFixedFont", 10))
        self.k.pack(anchor="w", pady=(3, 6))
        self.note = ttk.Label(card, style="Readout.TLabel", wraplength=330)
        self.note.pack(anchor="w")
        self.last = None

    def update(self, model):
        key = (model.route, model.frame_index, model.controller)
        if key == self.last:
            return
        self.last = key
        labels = STATES
        note = ""
        if model.route:
            q, r = BASE_Q, 1
            k = model.trajectories[model.route][model.frame_index][5:]
            mode = " → ".join(model.route) + " · 궤적 추종"
            note = "5 ms 기준 프레임의 설계 K · Q와 R은 시간 정규화 값"
            if model.route == ("DD", "DU"):
                note += " · 위치항 미사용"
        elif model.controller:
            pose = model.controller
            q, r = HOLD_WEIGHTS[pose]
            k = self.gains[pose]
            mode = pose + {"UU": " · LQI 홀드", "DD": " · LQR 감쇠"}.get(pose, " · LQR 홀드")
            if pose == "UU":
                labels = ("∫x dt   위치 적분", *STATES)
        else:
            mode, q, r, k = "활성 제어 대기", (), None, ()
        self.mode.configure(text=mode)
        for i, (state, weight) in enumerate(self.cells):
            state.configure(text=labels[i] if i < len(labels) else "")
            weight.configure(text=f"{q[i]:g}" if i < len(q) else ("—" if i < len(labels) else ""))
        self.r.configure(text=f"R   입력 가중치     [ {r:g} ]" if r is not None else "R   입력 가중치     —")
        self.k_title.configure(text=f"K   (1 × {len(k)})" if k else "K")
        self.k.configure(text="[ " + "  ".join(f"{v:.2f}" for v in k) + " ]" if k else "—")
        self.note.configure(text=note)


class PosePanel:
    def __init__(self, parent, column, title):
        card = ttk.Frame(parent, style="Card.TFrame", padding=12)
        card.grid(row=0, column=column, sticky="nsew", padx=5)
        ttk.Label(card, text=title, style="Heading.TLabel").pack(anchor="w")
        self.status_label = ttk.Label(card, style="Status.TLabel")
        self.status_label.pack(anchor="w", pady=(2, 8))
        self.canvas = tk.Canvas(card, width=750, height=460, bg="#101722", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.detail = ttk.Label(card, style="Readout.TLabel", anchor="center")
        self.detail.pack(fill="x", pady=(6, 0))
        self.history = deque()
        self.last_stamp = None
        self.elapsed = 0.0
        self.graphs = []
        for title in ("링크 1 절대각", "링크 2 절대각"):
            graph = tk.Canvas(card, height=110, bg="#101722", highlightthickness=0)
            graph.pack(fill="x", pady=(6, 0))
            graph.bind("<Configure>", lambda event: self.draw_graphs())
            self.graphs.append((title, graph))
        self.poses = (None, None)
        self.scale = None
        self.canvas.bind("<Configure>", self.resize)

    def resize(self, event):
        width, height = event.width, event.height
        self.cx, self.cy = width / 2, height * .48
        # Fit both displayed links at either 300 mm rail limit.
        self.scale = max(1, min((width - 54) / 1.13, (height - 70) / .53))
        self.ruler_y = self.cy + 40
        c = self.canvas
        c.delete("all")
        left, right = self.cx - .30 * self.scale, self.cx + .30 * self.scale
        c.create_line(left - 20, self.cy + 16, right + 20, self.cy + 16, fill="#b7c8da", width=4)
        c.create_line(self.cx, self.cy + 16, self.cx, self.cy + 23, fill="#58a6ff", width=2)
        c.create_line(left, self.ruler_y, right, self.ruler_y, fill="#c8d5e3")
        for mm in range(-300, 301, 25):
            px = self.cx + mm * self.scale / 1000
            major = mm % 100 == 0
            color = "#58a6ff" if mm == 0 else "#e3edf8"
            c.create_line(px, self.ruler_y, px, self.ruler_y + (9 if major else 4), fill=color)
            if major:
                c.create_text(px, self.ruler_y + 19, text=str(mm), fill=color, font=("TkDefaultFont", 9))
        self.draw_pose()

    def update(self, reference, actual, status):
        if self.status_label.cget("text") != status:
            self.status_label.configure(text=status)
        if self.poses != (reference, actual):
            self.poses = (reference, actual)
            self.draw_pose()

    def draw_pose(self):
        self.canvas.delete("motion")
        if self.scale is None:
            return
        reference, actual = self.poses
        if reference is not None:
            self.draw_body(reference, ghost=True)
        if actual is not None:
            self.draw_body(actual, ghost=False)
        def reading(label, pose):
            if pose is None:
                return label + "   —"
            x, base, tip = pose
            angles = [math.degrees(math.remainder(a, math.tau)) for a in (base, tip)]
            return f"{label}   카트 {1000*x:+.1f} mm    첫 링크 {angles[0]:+.1f}°    끝 링크 {angles[1]:+.1f}°"
        self.detail.configure(text=reading("실제 · 카트 추정", actual) + "\n" + reading("기준", reference))

    def add_sample(self, stamp, pose):
        if self.last_stamp is not None:
            delta = (stamp - self.last_stamp) & 0xFFFFFFFF
            if delta > 2_000_000:
                self.history.clear()
                self.elapsed = 0.0
            else:
                self.elapsed += delta / 1_000_000
        self.last_stamp = stamp
        angles = tuple(math.degrees(math.remainder(a, math.tau)) for a in pose[1:])
        self.history.append((self.elapsed, *angles))
        while self.history and self.elapsed - self.history[0][0] > 10:
            self.history.popleft()
        self.draw_graphs()

    def draw_graphs(self):
        for index, (title, graph) in enumerate(self.graphs, 1):
            graph.delete("all")
            width, height = graph.winfo_width(), graph.winfo_height()
            left, right, top, bottom = 48, width - 12, 20, height - 22
            if right <= left or bottom <= top:
                continue
            graph.create_text(8, 8, text=title + " (°)", fill="#f3f8ff", anchor="w",
                              font=("TkDefaultFont", 10, "bold"))
            for angle in (-180, 0, 180):
                y = top + (180 - angle) / 360 * (bottom - top)
                graph.create_line(left, y, right, y, fill="#4a5b70")
                graph.create_text(left - 5, y, text=str(angle), fill="#d5e3f2", anchor="e")
            graph.create_text(left, height - 4, text="-10초", fill="#d5e3f2", anchor="sw")
            graph.create_text(right, height - 4, text="현재", fill="#d5e3f2", anchor="se")
            points = []
            previous = None
            for t, a1, a2 in self.history:
                angle = (a1, a2)[index - 1]
                x = right - (self.elapsed - t) / 10 * (right - left)
                y = top + (180 - angle) / 360 * (bottom - top)
                if previous is not None and abs(angle - previous) > 180:
                    if len(points) >= 4:
                        graph.create_line(*points, fill="#ffdb4d", width=2)
                    points = []
                points.extend((x, y))
                previous = angle
            if len(points) >= 4:
                graph.create_line(*points, fill="#ffdb4d", width=2)
            if self.history:
                graph.create_text(right, 8, text=f"{self.history[-1][index]:+.1f}°",
                                  fill="#ffdb4d", anchor="e", font=("TkDefaultFont", 10, "bold"))

    def draw_body(self, pose, ghost):
        c = self.canvas
        x, base, tip = pose
        # Tk Canvas has no per-item alpha. Stipple lets the rail show through the reference.
        area = {"stipple": "gray50"} if ghost else {}
        tag = ("motion", "reference" if ghost else "actual")
        edge = "#748795" if ghost else "#ecf0f1"
        px, py = self.cx + x * self.scale, self.cy
        c.create_polygon(px, self.ruler_y, px - 4, self.ruler_y - 6, px + 4, self.ruler_y - 6,
                         fill="#78858e" if ghost else "#f85149", tags=tag, **area)
        for wheel in (px - 16, px + 16):
            c.create_oval(wheel - 5, py + 8, wheel + 5, py + 18,
                          fill="#45505d" if ghost else "#34495e", outline=edge, tags=tag, **area)
        c.create_rectangle(px - 26, py - 12, px + 26, py + 11,
                           fill="#516c83" if ghost else "#2980b9", outline=edge, tags=tag, **area)
        joints = [(px, py, 3)]
        for length, angle, radius in ((.13, base, 5), (.135, tip, 9)):
            nx = px + self.scale * length * math.sin(angle)
            ny = py - self.scale * length * math.cos(angle)
            c.create_line(px, py, nx, ny, width=7 if ghost else 6, fill="#a5b2c1" if ghost else "#ffe05a", capstyle="round", tags=tag, **area)
            joints.append((nx, ny, radius))
            px, py = nx, ny
        for px, py, radius in joints:
            c.create_oval(px - radius, py - radius, px + radius, py + radius,
                          fill="#75818e" if ghost else "#e74c3c", outline=edge, tags=tag, **area)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path)
    parser.add_argument("--project-dir", type=Path, default=Path.cwd())
    args = parser.parse_args()
    project = args.project_dir.resolve()
    logs = list((project / "logs").glob("device-monitor-*.log"))
    path = args.log or max(logs, key=lambda file: file.stat().st_mtime, default=None)
    if path is None or not path.is_file():
        parser.error("PlatformIO Monitor(log2file)를 시작하거나 현재 로그 경로를 지정하세요.")
    try:
        model = Comparison(load_trajectories(project))
        gains = load_hold_gains(project)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    root = tk.Tk()
    root.title("Equilibrium | Live pose 비교")
    root.geometry("1450x950")
    root.minsize(1050, 760)
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", background="#161b22", foreground="#e6edf3")
    style.configure("Card.TFrame", background="#1e222d")
    style.configure("Heading.TLabel", background="#1e222d", font=("TkDefaultFont", 12, "bold"))
    style.configure("Status.TLabel", background="#1e222d", foreground="#8abfff")
    style.configure("Readout.TLabel", background="#1e222d", foreground="#e6edf3")
    style.configure("TButton", background="#30363d", foreground="#e6edf3", padding=(8, 4))
    style.map("TButton", background=[("active", "#424b57")])
    toolbar = ttk.Frame(root, padding=(10, 8))
    toolbar.pack(fill="x")
    ttk.Label(toolbar, text=path.name).pack(side="left")
    ttk.Button(toolbar, text="실험 재생", command=lambda: subprocess.Popen(
        [sys.executable, "-m", "panel.replay", str(path.resolve())], cwd=project)).pack(side="right")
    ttk.Button(toolbar, text="DU 이중진자 LQR", command=lambda: subprocess.Popen(
        [sys.executable, "-m", "panel.du_lqr"], cwd=project)).pack(side="right", padx=5)
    panels = ttk.Frame(root, padding=(5, 0, 5, 10))
    panels.pack(fill="both", expand=True)
    panels.columnconfigure(0, minsize=390)
    panels.columnconfigure(1, weight=1)
    panels.rowconfigure(0, weight=1)
    sidebar = GainPanel(panels, gains)
    overlay = PosePanel(panels, 1, "실제 자세 · 노란 실선     기준 궤적 · 반투명 회색")

    with path.open("rb") as stream:
        # Recover the current state when opened after zeroing or after motion started.
        end = stream.seek(0, 2)
        if stream.seek(max(0, end - 131072)):
            stream.readline()
        pending = b""

        def poll():
            nonlocal pending
            lines = (pending + stream.read(131072)).split(b"\n")
            pending = lines.pop()
            for line in lines:
                text = line.decode(errors="replace").strip()
                previous = model.last_data
                model.feed(text)
                if model.last_data is not None and model.last_data != previous:
                    match = TELEMETRY.search(text)
                    if match:
                        overlay.add_sample(int(match[0].split(",")[1]), model.actual)
            status = model.status
            if model.last_data is not None and time.monotonic() - model.last_data > 1:
                status += " · 새 데이터 대기"
            sidebar.update(model)
            overlay.update(model.reference, model.actual, status + "    |    " + model.reference_status)
            root.after(25, poll)

        poll()
        root.mainloop()


if __name__ == "__main__":
    main()
