"""Interactive continuous-time LQR gain calculator for the double-pendulum DU pose."""
from pathlib import Path
import sys

import numpy as np
from PyQt5 import QtCore, QtWidgets

if not __package__:  # Also support launching directly from the live pose viewer.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.double_pendulum.check_model import DU_Q, DEFAULT_R, feedback_gain


class DuLqrPanel(QtWidgets.QWidget):
    states = (
        "카트 위치 x (m)", "베이스 각도 오차 θ₁ (rad)", "끝 링크 각도 오차 θ₂ (rad)",
        "카트 속도 v (m/s)", "베이스 각속도 ω₁ (rad/s)", "끝 링크 각속도 ω₂ (rad/s)",
    )

    def __init__(self):
        super().__init__()
        self.setWindowTitle("DU 이중진자 · LQR 이득")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        layout.setContentsMargins(24, 20, 24, 20)
        title = QtWidgets.QLabel("DU 균형 제어 · LQR 이득")
        font = title.font()
        font.setPointSize(16)
        font.setBold(True)
        title.setFont(font)
        layout.addWidget(title)
        layout.addWidget(QtWidgets.QLabel("6개 상태의 DU 이득을 계산합니다. 펌웨어는 각도·속도 이득만 제어에 사용합니다."))

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(20)
        grid.setVerticalSpacing(8)
        layout.addLayout(grid)
        for column, heading in enumerate(("상태", "Q 대각 가중치", "계산된 K")):
            grid.addWidget(QtWidgets.QLabel(heading), 0, column)
        self.q_inputs = []
        self.gain_labels = []
        for row, (state, weight) in enumerate(zip(self.states, DU_Q), 1):
            label = QtWidgets.QLabel(state)
            field = QtWidgets.QLineEdit(str(weight))
            field.setAccessibleName("Q · " + state)
            label.setBuddy(field)
            result = QtWidgets.QLabel()
            result.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            result.setMinimumWidth(100)
            result.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            grid.addWidget(label, row, 0)
            grid.addWidget(field, row, 1)
            grid.addWidget(result, row, 2)
            self.q_inputs.append(field)
            self.gain_labels.append(result)

        self.r_input = QtWidgets.QLineEdit(str(DEFAULT_R))
        self.r_input.setAccessibleName("R · 카트 가속도")
        grid.addWidget(QtWidgets.QLabel("카트 가속도 입력 가중치 R"), 7, 0)
        grid.addWidget(self.r_input, 7, 1)
        self.message = QtWidgets.QLabel()
        self.message.setStyleSheet("color: #96321b;")
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        for field in [*self.q_inputs, self.r_input]:
            field.textChanged.connect(self.recalculate)
        self.recalculate()

    def recalculate(self):
        try:
            q = [float(field.text()) for field in self.q_inputs]
            r = float(self.r_input.text())
        except ValueError:
            self.show_error("Q와 R의 각 칸에 숫자를 입력하세요.")
            return
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                gain = feedback_gain("e", q, r)
        except ValueError as exc:
            self.show_error(str(exc))
            return
        except (np.linalg.LinAlgError, FloatingPointError):
            self.show_error("이 가중치로 안정화 이득을 계산할 수 없습니다. Q와 R을 확인하세요.")
            return
        self.message.clear()
        for label, value in zip(self.gain_labels, gain):
            label.setText(f"{value:.6f}")

    def show_error(self, message):
        for label in self.gain_labels:
            label.setText("—")
        self.message.setText(message)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = DuLqrPanel()
    window.show()
    sys.exit(app.exec_())
