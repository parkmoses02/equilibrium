"""Read the PlatformIO monitor log (never opens the board or sends commands)."""
import argparse
import math
import time
import tkinter as tk
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path)
    args = parser.parse_args()
    logs = list(Path("logs").glob("*.log"))
    path = args.log or (max(logs, key=lambda p: p.stat().st_mtime) if logs else None)
    if not path or not path.is_file():
        parser.error("Start PlatformIO Monitor with log2file first, or specify the active log path.")
    root = tk.Tk()
    root.title("Equilibrium | Live pose")
    c = tk.Canvas(root, width=800, height=620, bg="#f7faf9")
    c.pack()
    status = tk.StringVar(value="Waiting for control telemetry: " + path.name)
    tk.Label(root, textvariable=status).pack()
    c.create_line(44,300,756,300,width=3)
    cart = c.create_rectangle(0,0,0,0,fill="#263849")
    rods = [c.create_line(0,0,0,0,width=7,fill=color) for color in ("#277f96","#d57c39")]
    last = 0
    pending = b""
    with path.open("rb") as f:
        f.seek(0,2)
        def poll():
            nonlocal last,pending
            lines=(pending+f.read(65536)).split(b"\n")
            pending=lines.pop()
            for raw in lines:
                line=raw.decode(errors="replace").strip()
                if line.startswith("OFF"):
                    last=0
                    status.set(line+" | last pose frozen")
                if "POSE," not in line: continue
                try:
                    _,stamp,mode,x,a,b=line[line.index("POSE,"):].split(",")
                    x,a,b=map(float,(x,a,b))
                    if not all(map(math.isfinite,(x,a,b))): continue
                    px,py=400+800*x,300
                    c.coords(cart,px-17,py-7,px+17,py+7)
                    for rod,length,angle in zip(rods,(.16,.165),(a,b)):
                        nx,ny=px+800*length*math.sin(angle),py-800*length*math.cos(angle)
                        c.coords(rod,px,py,nx,ny)
                        px,py=nx,ny
                    status.set(f"{mode} | cart estimate {1000*x:+.1f} mm | base {math.degrees(a):+.1f}° | tip {math.degrees(b):+.1f}°")
                    last=time.monotonic()
                except ValueError: continue
            if last and time.monotonic()-last>1:
                status.set("No recent telemetry | last pose frozen")
                last=0
            root.after(40,poll)
        poll()
        root.mainloop()


if __name__=="__main__":
    main()
