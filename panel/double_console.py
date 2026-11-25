"""Equilibrium · double pendulum operator console. python -m panel.double_console"""
from collections import deque
from datetime import datetime
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports
from .double_state import ConsoleState, POSES

LABEL = {"ZERO": "모터 OFF · 영점 필요", "OFF": "모터 OFF · DD 출발각 밖",
         "READY": "DD · 모터 OFF", "WAIT": "DD · 모터 ON / 감쇠 대기",
         "ARM": "출발 조건 대기", "SWING": "전환 중", "HOLD": "유지 제어",
         "DAMP": "제어 하강 / 감쇠", "BRAKE": "레일 보호 제동"}
COLORS = ("#197989", "#c25721")
LOGS = Path(__file__).resolve().parents[1] / "logs"


class Console:
    def __init__(self, root):
        self.root = root
        root.title("Equilibrium  |  이중진자 제어")
        root.geometry("1080x695")
        self.port = self.log = None
        self.model = ConsoleState()
        self.buffer = b""
        self.supported = False
        self.samples = deque(maxlen=250)
        body = ttk.Frame(root, padding=14)
        body.pack(fill="both", expand=True)
        bar = ttk.Frame(body)
        bar.pack(fill="x")
        ttk.Label(bar, text="EQUILIBRIUM   /   DOUBLE", font=("Segoe UI", 14, "bold")).pack(side="left", padx=(0, 20))
        self.ports = ttk.Combobox(bar, width=15)
        self.ports.pack(side="left")
        ttk.Button(bar, text="포트 검색", command=self.scan).pack(side="left")
        self.link = ttk.Button(bar, text="연결", command=self.toggle)
        self.link.pack(side="left")
        self.status = ttk.Label(body, font=("Malgun Gothic", 15, "bold"))
        self.status.pack(anchor="w", pady=(16, 4))
        self.metrics = ttk.Label(body)
        self.metrics.pack(anchor="w")
        self.plot = tk.Canvas(body, bg="#f6f8f7", highlightthickness=0)
        self.plot.pack(fill="both", expand=True, pady=8)
        self.plot.bind("<Configure>", lambda _: self.draw())
        actions = ttk.Frame(body)
        actions.pack(fill="x")
        self.buttons = {}
        for key, name in [("z", "중앙 DD 영점"), *POSES.items()]:
            btn = ttk.Button(actions, text=name, command=lambda k=key: self.send(k))
            btn.pack(side="left", padx=2)
            self.buttons[key] = btn
        self.stop = tk.Button(actions, text="즉시 출력 OFF", bg="#9c302f", fg="white",
                              command=lambda: self.send("!"))
        self.stop.pack(side="right")
        self.events = tk.Text(body, height=5, bg="#172735", fg="#eff5fa", state="disabled")
        self.events.pack(fill="x", pady=(9, 0))
        self.scan()
        self.note("카트를 중앙에 놓고 두 링크를 아래로 늘어뜨린 후 영점을 설정하세요.")
        self.refresh()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(25, self.poll)

    def scan(self):
        devices = [p.device for p in list_ports.comports()]
        self.ports["values"] = devices
        if not self.ports.get() and devices:
            self.ports.set(devices[0])

    def note(self, msg):
        self.events.config(state="normal")
        self.events.insert("end", datetime.now().strftime("%H:%M:%S  ") + msg + "\n")
        if int(self.events.index("end-1c").split(".")[0]) > 120:
            self.events.delete("1.0", "20.0")
        self.events.see("end")
        self.events.config(state="disabled")

    def detach(self):
        if self.port:
            self.port.close()
        if self.log:
            self.log.close()
        self.port = self.log = None
        self.buffer = b""
        self.supported = False
        self.model = ConsoleState()
        self.samples.clear()
        self.refresh()
        self.draw()

    def can_leave(self):
        return (not self.port or (self.model.fresh(time.monotonic()) and not self.model.pending
                and self.model.phase in ("ZERO", "OFF", "READY")) or messagebox.askyesno(
                    "연결 종료", "연결을 끊어도 모터는 꺼지지 않습니다. 그래도 연결을 종료할까요?"))

    def close(self):
        if self.can_leave():
            self.detach()
            self.root.destroy()

    def toggle(self):
        if self.port:
            if self.can_leave():
                self.detach()
            return
        try:
            s = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.1)
            s.dtr = s.rts = False
            s.port = self.ports.get().strip()
            s.open()
            self.port = s
            s.reset_input_buffer()
            LOGS.mkdir(exist_ok=True)
            path = LOGS / datetime.now().strftime("double_panel_%Y%m%d_%H%M%S_%f.log")
            self.log = path.open("xb", buffering=8192)
            self.note("연결됨 · 보드 상태 수신 대기 · 로그: " + path.name)
        except (OSError, ValueError) as exc:
            self.detach()
            self.note("연결 실패: " + str(exc))
        self.refresh()

    def send(self, key):
        if not self.port:
            return
        if key == "z" and not messagebox.askyesno("DD 영점 설정", "카트가 레일 중앙이고 두 링크가 모두 아래로 정지해 있나요?"):
            return
        cmd = ("!" if self.supported else None) if key == "!" else self.model.request(key, time.monotonic())
        if not cmd:
            return
        try:
            data = (cmd + "\n").encode("ascii")
            if self.port.write(data) != len(data):
                raise OSError("명령 일부만 전송됨")
            if key == "!":
                self.model.pending = "!"
            self.note("보냄: " + cmd + " · 보드 응답 대기")
        except OSError as exc:
            self.detach()
            self.note("송신 실패 · 실제 장치 상태 확인: " + str(exc))
        self.refresh()

    def poll(self):
        if self.port:
            try:
                raw = self.port.read(min(self.port.in_waiting, 65536))
                if raw:
                    self.log.write(raw)
                    lines = (self.buffer + raw).split(b"\n")
                    self.buffer = lines.pop()
                    if len(self.buffer) > 8192:
                        raise OSError("정상적인 줄 단위 데이터가 아닙니다")
                    updated = False
                    for line in lines:
                        updated |= self.receive(line.decode(errors="replace").strip())
                    if updated:
                        self.draw()
            except (OSError, ValueError) as exc:
                self.detach()
                self.note("수신 중단 · 모터 정지는 확인되지 않았습니다: " + str(exc))
        self.refresh()
        self.root.after(25, self.poll)

    def receive(self, line):
        now = time.monotonic()
        if self.model.feed(line, now):
            self.supported = True
            if self.model.phase != "ZERO":
                _, b, t, *_ = self.model.values
                self.samples.append((now, math.degrees(b), math.degrees(t)))
            else:
                self.samples.clear()
            return True
        elif line and not (line[0].isdigit() or line.startswith(("POSE,", "DP1,", "step,"))):
            self.note(line[:400])
        return False

    def refresh(self):
        m = self.model
        now = time.monotonic()
        fresh = m.fresh(now)
        for k, btn in self.buttons.items():
            btn.config(state="normal" if self.port and m.command(k, now) else "disabled")
        self.stop.config(state="normal" if self.port and self.supported else "disabled")
        self.link.config(text="연결 해제" if self.port else "연결")
        if fresh:
            state = LABEL[m.phase]
            if m.phase in ("ARM", "SWING"):
                state = f"{POSES[m.src]} → {POSES[m.dest]}   |   {state}"
            elif m.phase == "HOLD":
                state = f"{POSES[m.src]}   |   {state}"
            if m.pending:
                state += "   ·   응답 대기: " + POSES.get(m.pending, m.pending)
        else:
            state = ("미연결" if not self.port else "상태 미확인 · DP1 수신 대기 / 자세 명령 잠금")
        self.status.config(text=state)
        if fresh and m.phase != "ZERO":
            x,b,t,wb,wt,v,a = m.values
            active = m.phase not in ("READY", "OFF", "BRAKE")
            rates = f"각속도 {wb:+.2f} / {wt:+.2f} rad/s    램프 속도 {v:+.3f} m/s    제한 후 가속도 명령 {a:+.2f} m/s²"
            self.metrics.config(text=f"카트 추정 {x*1000:+.1f} mm    각도 {math.degrees(b):+.1f}° / {math.degrees(t):+.1f}°    " +
                                (rates if active else "속도·가속도 표시 대기"))
        else:
            self.metrics.config(text="유효한 실시간 값 없음 · 카트 위치는 독립 센서 실측이 아닌 램프 추정값입니다.")

    def draw(self):
        c = self.plot
        c.delete("all")
        w,h = c.winfo_width(),c.winfo_height()
        if min(w,h)<20:
            return
        mid=w*.48
        cx,cy=mid*.50,h*.51
        scale=min((mid-24)/1.30,(h-60)/.70)
        c.create_text(12,16,anchor="w",text="장치 자세   /   위쪽 0°", fill="#324456")
        c.create_line(cx-.445*scale,cy,cx+.445*scale,cy,width=3,fill="#85949b")
        for pos in (-.30,0,.30):
            px=cx+pos*scale
            c.create_line(px,cy-4,px,cy+4)
            c.create_text(px,cy+15,text=f"{pos*1000:.0f}",fill="#697786")
        if self.model.values and self.model.phase!="ZERO":
            x,b,t,*_=self.model.values
            px,py=cx+scale*x,cy
            c.create_rectangle(px-10,py-5,px+10,py+5,fill="#283946")
            for length,angle,color in zip((.16,.165),(b,t),COLORS):
                nx,ny=px+scale*length*math.sin(angle),py-scale*length*math.cos(angle)
                c.create_line(px,py,nx,ny,width=5,fill=color)
                c.create_oval(nx-4,ny-4,nx+4,ny+4,fill=color,outline="")
                px,py=nx,ny
        x0,x1=mid+34,w-16
        y0,y1=42,h-30
        c.create_text(x0,16,anchor="w",text="절대각 · 최근 10초     ① 베이스",fill=COLORS[0])
        c.create_text(x0+185,16,anchor="w",text="② 끝 링크",fill=COLORS[1])
        for deg in (-180,0,180):
            y=(y0+y1)/2-deg*(y1-y0)/360
            c.create_line(x0,y,x1,y,fill="#d0dadd")
            c.create_text(x0-3,y,anchor="e",text=str(deg),fill="#697786")
        if not self.samples:
            return
        end=self.samples[-1][0]
        for j,color in enumerate(COLORS,1):
            prev=None
            segment=[]
            for row in self.samples:
                ts,angle=row[0],row[j]
                if ts<end-10: continue
                px=x1-(end-ts)*(x1-x0)/10
                py=(y0+y1)/2-angle*(y1-y0)/360
                if prev and (ts-prev[0]>=.6 or abs(angle-prev[1])>=180):
                    if len(segment)>=4:
                        c.create_line(*segment,fill=color,width=2)
                    segment=[]
                segment.extend((px,py))
                prev=ts,angle
            if len(segment)>=4:
                c.create_line(*segment,fill=color,width=2)


if __name__ == "__main__":
    root = tk.Tk()
    Console(root)
    root.mainloop()
