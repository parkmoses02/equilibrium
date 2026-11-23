"""Replay a saved PlatformIO monitor log: python -m panel.replay [path]."""
from bisect import bisect_right
import math
from pathlib import Path
import sys

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

if __package__:
    from .replay_log import PHASE, load_recordings
else:
    from replay_log import PHASE, load_recordings


LOG_DIR = Path(__file__).resolve().parents[1] / "logs"


class ReplayPanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Equilibrium · 이중진자 실험 재생")
        self.resize(880, 650)
        self.runs = []
        self.path = None
        self.initialized = False
        layout = QtWidgets.QVBoxLayout(self)
        files = QtWidgets.QHBoxLayout()
        layout.addLayout(files)
        open_button = QtWidgets.QPushButton("로그 열기…")
        open_button.clicked.connect(self.choose_file)
        files.addWidget(open_button)
        self.filename = QtWidgets.QLabel()
        files.addWidget(self.filename, 1)
        self.trials = QtWidgets.QComboBox()
        self.trials.currentIndexChanged.connect(self.select_trial)
        files.addWidget(self.trials)

        view = pg.PlotWidget(background="#f3f6f5")
        layout.addWidget(view, 1)
        view.setAspectLocked()
        view.setMouseEnabled(x=False, y=False)
        view.setMenuEnabled(False)
        view.hideButtons()
        view.hideAxis("left")
        view.hideAxis("bottom")
        view.setRange(xRange=(-0.61, 0.61), yRange=(-0.36, 0.36), padding=0)
        view.plot([-.34, .34], [0, 0], pen=pg.mkPen("#85949b", width=3))
        for x in (-.3, 0, .3):
            view.plot([x, x], [-.009, .009], pen=pg.mkPen("#85949b"))
            tick = pg.TextItem(f"{x*1000:.0f} mm", color="#50606b", anchor=(.5, 0))
            tick.setPos(x, -.015)
            view.addItem(tick)
        self.cart = view.plot(pen=None, symbol="s", symbolSize=18, symbolBrush="#253747")
        self.base = view.plot(pen=pg.mkPen("#187c8e", width=6), symbol="o", symbolSize=8,
                              symbolBrush="#187c8e")
        self.tip = view.plot(pen=pg.mkPen("#c55d22", width=5), symbol="o", symbolSize=9,
                             symbolBrush="#c55d22")
        self.pose = QtWidgets.QLabel()
        layout.addWidget(self.pose)
        self.timeline = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.timeline.setAccessibleName("재생 위치")
        self.timeline.sliderPressed.connect(self.pause)
        self.timeline.valueChanged.connect(self.draw)
        layout.addWidget(self.timeline)

        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        self.play = QtWidgets.QPushButton("재생")
        self.play.clicked.connect(self.toggle)
        controls.addWidget(self.play)
        self.speed = QtWidgets.QComboBox()
        for rate in (.25, .5, 1, 2, 4):
            self.speed.addItem(f"{rate:g}×", rate)
        self.speed.setCurrentIndex(2)
        self.speed.currentIndexChanged.connect(self.anchor)
        controls.addWidget(self.speed)
        self.time_label = QtWidgets.QLabel()
        controls.addWidget(self.time_label)
        controls.addStretch()
        self.reason = QtWidgets.QLabel()
        self.reason.setWordWrap(True)
        self.reason.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.reason)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self.advance)
        self.clock = QtCore.QElapsedTimer()
        self.select_trial(-1)

    def showEvent(self, event):
        super().showEvent(event)
        if not self.initialized:
            self.initialized = True
            for path in sorted(LOG_DIR.glob("*.log"), key=lambda p: p.name, reverse=True):
                if self.open_log(path):
                    break

    def choose_file(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "실험 로그 열기", str(self.path.parent if self.path else LOG_DIR),
            "실험 기록 (*.log *.csv *.txt);;모든 파일 (*)")
        if filename:
            self.open_log(Path(filename))

    def open_log(self, path):
        self.initialized = True
        self.pause()
        try:
            self.runs = load_recordings(path)
            error = ""
        except OSError as exc:
            self.runs = []
            error = str(exc)
        self.path = path
        self.filename.setText(path.name)
        self.filename.setToolTip(str(path))
        self.trials.blockSignals(True)
        self.trials.clear()
        for number, run in enumerate(self.runs, 1):
            self.trials.addItem(f"{number}회차 · {run.times[-1]/1e6:.2f} s")
        index = len(self.runs) - 1
        self.trials.setCurrentIndex(index)
        self.trials.blockSignals(False)
        self.select_trial(index)
        if error:
            self.reason.setText(error)
        return bool(self.runs)

    def select_trial(self, index):
        self.pause()
        self.run = self.runs[index] if index >= 0 else None
        duration = math.ceil(self.run.times[-1]/1000) if self.run else 0
        self.timeline.setRange(0, duration)
        self.timeline.setValue(0)
        self.timeline.setEnabled(duration > 0)
        self.play.setEnabled(duration > 0)
        if self.run:
            self.reason.setText("회차 종료 기록 · " + (self.run.reason or "정지 사유 미기록"))
            self.draw()
        else:
            for item in (self.cart, self.base, self.tip):
                item.clear()
            self.pose.clear()
            self.time_label.clear()
            self.reason.setText("재생할 자세 기록이 없습니다. 다른 실험 로그를 열어 주세요.")

    def pause(self):
        self.timer.stop()
        self.play.setText("재생")

    def anchor(self):
        self.start_ms = self.timeline.value()
        self.clock.start()

    def toggle(self):
        if self.timer.isActive():
            self.pause()
        elif self.run:
            if self.timeline.value() == self.timeline.maximum():
                self.timeline.setValue(0)
            self.anchor()
            self.play.setText("일시정지")
            self.timer.start()

    def advance(self):
        target = self.start_ms + round(self.clock.elapsed() * self.speed.currentData())
        self.timeline.setValue(min(target, self.timeline.maximum()))
        if target >= self.timeline.maximum():
            self.pause()

    def draw(self):
        if not self.run:
            return
        t = self.timeline.value() * 1000
        i = bisect_right(self.run.times, t) - 1
        p = self.run.samples[i]
        # Both rods are logged in world coordinates; positive angles tilt right.
        elbow_x = p.x + .16 * math.sin(p.base)
        elbow_y = .16 * math.cos(p.base)
        self.cart.setData([p.x], [0])
        self.base.setData([p.x, elbow_x], [0, elbow_y])
        self.tip.setData([elbow_x, elbow_x + .165*math.sin(p.tip)],
                         [elbow_y, elbow_y + .165*math.cos(p.tip)])
        base = math.degrees(math.remainder(p.base, 2*math.pi))
        tip = math.degrees(math.remainder(p.tip, 2*math.pi))
        self.pose.setText(f"{PHASE[p.mode]}   ·   카트 추정 위치 {1000*p.x:+.1f} mm   ·   "
                          f"베이스 {base:+.2f}°   ·   끝 링크 {tip:+.2f}°   (위쪽 0°)")
        self.time_label.setText(f"{min(t, self.run.times[-1])/1e6:.3f} / {self.run.times[-1]/1e6:.3f} s")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    viewer = ReplayPanel()
    if len(sys.argv) > 1:
        viewer.open_log(Path(sys.argv[1]))
    viewer.show()
    sys.exit(app.exec_())
