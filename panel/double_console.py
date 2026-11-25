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
COLORS = ("#007f78", "#c55a26")
PAPER, INK, MUTED = "#eef2f2", "#192f38", "#556970"
LOGS = Path(__file__).resolve().parents[1] / "logs"


class Console:
    def __init__(self, root):
        self.root = root
        root.title("Equilibrium  |  이중진자 제어")
        root.geometry("1100x740")
        root.minsize(790, 590)
        self.port = self.log = None
        self.model = ConsoleState()
        self.buffer = b""
        self.supported = False
        self.samples = deque(maxlen=250)
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TButton", font=("Malgun Gothic", 10), padding=(10, 5))
        style.configure("Pose.TButton", font=("Segoe UI", 11, "bold"), padding=(12, 5))
        style.configure("TFrame", background=PAPER)
        root.configure(bg=PAPER)
        bar = tk.Frame(root, bg=INK, padx=18, pady=10)
        bar.pack(fill="x")
        tk.Label(bar, text="Equilibrium", font=("Segoe UI", 18, "bold"), bg=INK, fg="white").pack(side="left")
        tk.Label(bar, text=" /   이중 역진자", font=("Malgun Gothic", 10), bg=INK, fg="#b8cbca").pack(side="left")
        self.link = ttk.Button(bar, text="연결", command=self.toggle)
        self.link.pack(side="right")
        ttk.Button(bar, text="포트 검색", command=self.scan).pack(side="right", padx=5)
        self.ports = ttk.Combobox(bar, width=12)
        self.ports.pack(side="right", padx=3)
        body = ttk.Frame(root, padding=(18, 9, 18, 10))
        body.pack(fill="both", expand=True)
        strip = tk.Frame(body, bg=PAPER)
        strip.pack(fill="x", pady=(0, 7))
        tk.Label(strip,text="제어 상태",bg=PAPER,fg=MUTED).pack(side="left",padx=(0,10))
        self.status = tk.Label(strip,bg=PAPER,fg=INK,font=("Malgun Gothic", 14, "bold"))
        self.status.pack(side="left")
        cards=tk.Frame(body,bg=PAPER)
        cards.pack(fill="x")
        self.readouts=[]
        for i,(title,color) in enumerate(zip(
                ("첫 링크 각도","끝 링크 각도","카트 위치 · 추정","램프 속도","가속도 명령"),
                (*COLORS,INK,INK,INK))):
            cards.columnconfigure(i,weight=1,uniform="card")
            card=tk.Frame(cards,bg="white",padx=12,pady=5)
            card.grid(row=0,column=i,sticky="nsew",padx=(0,5 if i<4 else 0))
            tk.Label(card,text=title,bg="white",fg=color,font=("Malgun Gothic",9)).pack(anchor="w")
            value=tk.Label(card,text="—",bg="white",fg=color,font=("Segoe UI",15,"bold"))
            value.pack(anchor="w")
            detail=tk.Label(card,bg="white",fg=MUTED,font=("Malgun Gothic",9))
            detail.pack(anchor="w")
            self.readouts.append((value,detail))
        self.plot = tk.Canvas(body, bg="white", highlightthickness=0)
        self.plot.pack(fill="both", expand=True, pady=8)
        self.plot.bind("<Configure>", lambda _: self.draw())
        actions = ttk.Frame(body)
        actions.pack(fill="x")
        self.buttons = {}
        tk.Label(actions,text="목표 자세",bg=PAPER,fg=INK,font=("Malgun Gothic",10,"bold")).pack(side="left",padx=(0,8))
        for key, name in [*POSES.items(), ("z", "중앙 DD 영점")]:
            btn = ttk.Button(actions, text=name, width=11 if key=="z" else 5,
                             style="TButton" if key=="z" else "Pose.TButton",command=lambda k=key: self.send(k))
            btn.pack(side="left", padx=2)
            self.buttons[key] = btn
        self.stop = tk.Button(actions, text="즉시 출력 OFF", bg="#9c302f", fg="white",
                              command=lambda: self.send("!"))
        self.stop.pack(side="right")
        tk.Label(body,text="이벤트 기록",bg=PAPER,fg=MUTED).pack(anchor="w",pady=(7,1))
        self.events = tk.Text(body, height=3, bg=INK, fg="#e2ecea", state="disabled",bd=0,padx=9,pady=4)
        self.events.pack(fill="x")
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
            vals=(f"{math.degrees(b):+.1f}°",f"{math.degrees(t):+.1f}°",f"{x*1000:+.1f} mm",
                  f"{v:+.3f} m/s" if active else "—", f"{a:+.2f} m/s²" if active else "—")
            rates=(f"{wb:+.2f} rad/s",f"{wt:+.2f} rad/s") if active else ("위쪽 0°",)*2
        else:
            vals=("—",)*5
            rates=("위쪽 0°",)*2
        for (value,detail),v,d in zip(self.readouts,vals,(*rates,"영점 기준","드라이버 추정","제한 적용 후")):
            value.config(text=v)
            detail.config(text=d)

    def draw(self):
        c = self.plot
        c.delete("all")
        w,h = c.winfo_width(),c.winfo_height()
        if min(w,h)<20:
            return
        mid=w*.48
        cx,cy=mid*.50,h*.51
        scale=min((mid-24)/1.30,(h-60)/.70)
        c.create_rectangle(0,0,mid,h,fill="#f8fbfa",outline="")
        c.create_line(mid,0,mid,h,fill="#dbe4e3")
        c.create_text(15,19,anchor="w",text="장치 자세",font=("Malgun Gothic",11,"bold"),fill=INK)
        c.create_text(15,h-15,anchor="w",text="레일 좌표  /  mm",fill=MUTED)
        c.create_line(cx,38,cx,h-30,dash=(3,5),fill="#ceddda")
        c.create_line(cx-.445*scale,cy,cx+.445*scale,cy,width=4,fill="#a7bab8",capstyle="round")
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
                c.create_line(px,py,nx,ny,width=6,fill=color,capstyle="round")
                c.create_oval(nx-5,ny-5,nx+5,ny+5,fill="white",outline=color,width=3)
                px,py=nx,ny
        x0,x1=mid+48,w-18
        y0,y1=48,h-32
        c.create_text(mid+15,19,anchor="w",text="각도 기록",font=("Malgun Gothic",11,"bold"),fill=INK)
        for j,name in enumerate(("첫 링크","끝 링크")):
            lx=x1-132+j*72
            c.create_line(lx,19,lx+12,19,width=3,fill=COLORS[j])
            c.create_text(lx+16,19,anchor="w",text=name,fill=COLORS[j])
        for sec in (-10,-5,0):
            px=x1+sec*(x1-x0)/10
            c.create_line(px,y0,px,y1,fill="#e4ecea",dash=(2,4))
            c.create_text(px,y1+14,anchor="e" if sec==0 else "center",text=f"{sec} s",fill=MUTED)
        for deg in (-180,-90,0,90,180):
            y=(y0+y1)/2-deg*(y1-y0)/360
            c.create_line(x0,y,x1,y,fill="#abbdb9" if deg==0 else "#e4ecea")
            c.create_text(x0-5,y,anchor="e",text=f"{deg}°",fill=MUTED)
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
