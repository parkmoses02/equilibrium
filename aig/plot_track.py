#!/usr/bin/env python3
"""
Plot a CSV written by pend_cam (board) on the PC.

  python3 plot_track.py track.csv            -> track.png
  python3 plot_track.py track.csv -o fig.png

Needs: numpy, matplotlib
Panels: angle (tip / middle), unwrapped angle, per-frame processing time + capture latency,
        and a text summary (found counts, dropped frames, tip-vs-middle agreement).
"""
import argparse, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def col(rows, name):
    out = []
    for r in rows:
        v = r.get(name, "")
        out.append(float(v) if v not in ("", None) else np.nan)
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv)))
    t = col(rows, "t_s")
    at, am = col(rows, "angle_tip_deg"), col(rows, "angle_mid_deg")
    unw = col(rows, "angle_unwrapped_deg")
    proc, lat = col(rows, "proc_ms"), col(rows, "lat_ms")
    seq = col(rows, "seq")

    n = len(t)
    both = ~np.isnan(at) & ~np.isnan(am)
    d = np.abs((at[both] - am[both] + 180) % 360 - 180)
    gaps = int(np.nansum(np.diff(seq) - 1)) if n > 1 else 0
    lines = ["frames %d   duration %.2f s" % (n, t[-1] - t[0] if n > 1 else 0),
             "tip found %d, middle found %d, no angle %d" % ((~np.isnan(at)).sum(), (~np.isnan(am)).sum(),
                                                            (np.isnan(at) & np.isnan(am)).sum()),
             "dropped frames (seq gaps) %d" % gaps]
    if both.any():
        lines.append("tip vs middle: median %.2f deg, p95 %.2f deg" % (np.median(d), np.percentile(d, 95)))
    if (~np.isnan(proc)).any():
        lines.append("processing: mean %.2f ms, max %.2f ms" % (np.nanmean(proc), np.nanmax(proc)))
    if (~np.isnan(lat)).any():
        lines.append("capture->processing latency: mean %.2f ms" % np.nanmean(lat))
    print("\n".join(lines))

    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(t, at, ".", ms=4, label="tip"); ax[0].plot(t, am, ".", ms=4, label="middle")
    ax[0].set_ylabel("angle [deg]\n0 = up, + = right"); ax[0].legend(loc="upper right"); ax[0].grid(alpha=.3)
    ax[1].plot(t, unw, "-"); ax[1].set_ylabel("unwrapped [deg]"); ax[1].grid(alpha=.3)
    ax[2].plot(t, proc, "-", label="processing [ms]")
    if (~np.isnan(lat)).any():
        ax[2].plot(t, lat, "-", label="capture latency [ms]")
    ax[2].set_ylabel("ms"); ax[2].set_xlabel("time [s]"); ax[2].legend(loc="upper right"); ax[2].grid(alpha=.3)
    fig.suptitle(a.csv + "\n" + " | ".join(lines[:3]), fontsize=9)
    fig.tight_layout()
    out = a.out or a.csv.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=130)
    print("wrote", out)


if __name__ == "__main__":
    main()
