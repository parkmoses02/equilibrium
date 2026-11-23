"""Estimate equivalent joint torque residuals from one complete RE_CSV block.

Usage: python simulation/dd-du/inverse_residual.py monitor-excerpt.txt
Requires numpy and scipy. Ramp velocity is a proxy, not a cart measurement.
"""
import csv
import re
import sys
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter


def analyze(path):
    text = Path(path).read_text(encoding="utf-8-sig")
    blocks = re.findall(r"RE_CSV_BEGIN\s*\n(.*?)\nRE_CSV_END", text, re.S)
    if len(blocks) != 1:
        raise ValueError("Select exactly one complete RE_CSV_BEGIN/END block")
    rows = list(csv.DictReader(blocks[0].splitlines()))
    col = lambda name: np.array([float(row[name]) for row in rows])
    if not np.array_equal(col("step"), np.arange(337)):
        raise ValueError("Expected swing frames 0..336 in order")
    t = col("t_us") * 1e-6
    if not np.all(np.diff(t) > 0):
        raise ValueError("Nonmonotone timestamps")
    grid = np.linspace(t[0], t[-1], len(t))
    dt = grid[1]-grid[0]
    a,b,v = [np.interp(grid,t,y) for y in
             (np.unwrap(col("base_rad")),np.unwrap(col("tip_rad")),col("ramp_v_m_s"))]
    header = Path(__file__).resolve().parents[2]/"src/double_pendulum/du_frames.h"
    ref = np.array([[float(x.rstrip("f")) for x in row.split(",")] for row in
                    re.findall(r"^\s*\{([^{}]+)\},", header.read_text(), re.M)])
    err = np.column_stack((col("base_rad")[:-1],col("tip_rad")[:-1]))-ref[:,1:3]
    print("nominal tracking angular RMS deg:", np.sqrt(np.mean(np.rad2deg(np.angle(np.exp(1j*err)))**2,axis=0)))
    B,S,g=.002475,.0128,9.80665
    C=.16*B
    J2=B*g*(.792172/(2*np.pi))**2
    J1=S*g*(.930958/(2*np.pi))**2-J2-2*C
    for win in (11,21,31):
        def filt(x,n): return savgol_filter(x,win,3,deriv=n,delta=dt)
        w,z,aa,bb,u=filt(a,1),filt(b,1),filt(a,2),filt(b,2),filt(v,1)
        h,s=C*np.cos(a-b),C*np.sin(a-b)
        r1=J1*aa+h*bb-(S-B)*(g*np.sin(a)-u*np.cos(a))+s*z*z
        r2=h*aa+J2*bb-B*(g*np.sin(b)-u*np.cos(b))-s*w*w
        # Generalized absolute-angle torques => physical base and elbow torques.
        torques=np.column_stack((r1+r2,r2))[16:-16]
        rates=np.column_stack((w,z-w))[16:-16]
        drag=-np.sum(torques*rates,axis=0)/np.sum(rates**2,axis=0)
        print("window", win,"joint torque RMS [mNm]",1000*np.sqrt(np.mean(torques**2,axis=0)),
              "unconstrained viscous slopes [Nm s/rad]",drag)


if __name__=="__main__":
    analyze(sys.argv[1])
