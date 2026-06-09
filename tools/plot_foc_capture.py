#!/usr/bin/env python3
"""Plot foc_capture variables from a dump.

Input (positional, default: foc_capture.bin):
    *.bin   raw ring dump  (decoded via decode_foc_capture.load; reads *.bin.head)
    *.csv   CSV from `tools/foc_capture.sh decode`
    -       read CSV from stdin

By default draws a stacked, time-aligned multi-panel view tuned for the FOC inner
loop: phase currents, dq meas-vs-setpoint, SVM duties, electrical velocity, Vbus
(plus the dropped-shunt phase if best-2-of-3 selection was active). Saves a PNG;
add --show for an interactive window.

For the high-speed lumpiness hunt, two views answer the key question:
  - the default stack: are Id/Iq themselves lumpy (control/measurement), or are
    they clean while the phase currents are lumpy (voltage-vector quantization)?
  - --xy: the Iα-Iβ current-vector locus — a clean spin is a circle; lumpiness
    shows up as a distorted / polygonal locus.

Examples:
    tools/plot_foc_capture.py foc.bin
    tools/plot_foc_capture.py foc.csv --show
    tools/plot_foc_capture.py foc.bin --xy
    tools/plot_foc_capture.py foc.bin --cols Ia,Ib,Ic,Iq_meas
    tools/foc_capture.sh decode foc.bin | tools/plot_foc_capture.py - --show

Sample rate for the time axis defaults to 32 kHz (PWM_CYCLES_PER_ISR=3); override
with --rate if you change the fast-loop rate.
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np

# Allow `import decode_foc_capture` when run as a script from anywhere.
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# Named panels: group -> (y-axis label, [(column, linestyle), ...]). Columns absent
# from the input are silently skipped, so this works across capture-struct versions.
GROUPS = {
    "iabc":  ("Phase I (A)",       [("Ia", "-"), ("Ib", "-"), ("Ic", "-")]),
    "idq":   ("dq I (A)",          [("Id_meas", "-"), ("Iq_meas", "-"),
                                    ("Id_set", "--"), ("Iq_set", "--")]),
    "iab":   ("Clarke αβ I (A)",   [("Ialpha", "-"), ("Ibeta", "-")]),
    "mod":   ("mod dq",            [("mod_d", "-"), ("mod_q", "-"), ("mod_norm", "-")]),
    "modab": ("mod αβ",            [("mod_alpha", "-"), ("mod_beta", "-")]),
    "svm":   ("SVM duty (0–1)",    [("svm_A", "-"), ("svm_B", "-"), ("svm_C", "-")]),
    "vint":  ("PI ∫ (V)",          [("Vd_int", "-"), ("Vq_int", "-")]),
    "vel":   ("elec vel (Hz)",     [("vel_hz", "-")]),
    "phase": ("enc phase (0–1)",   [("phase_01", "-")]),
    "vbus":  ("Vbus (V)",          [("Vbus", "-")]),
    "drop":  ("shunt drop",        [("shunt_drop", "-")]),
}
# Captured |mod| is clamped to 0.99·√3/2 ≈ 0.857 (SVM linear-modulation limit, foc.c);
# drawn as a reference line on the mod panel — but only once |mod| approaches it, so it
# doesn't squash the y-axis on low-modulation (low-speed) captures.
SVM_MOD_LIMIT = 0.99 * (3.0 ** 0.5) / 2.0
DEFAULT_GROUPS = ["iabc", "idq", "mod", "vel", "phase", "vbus"]


def load_columns(src):
    """Return (names, {name: ndarray}) from a .bin, .csv, or '-' (stdin CSV)."""
    if src != "-" and Path(src).suffix == ".bin":
        import decode_foc_capture as dec
        data = dec.load(src)
        return list(data.dtype.names), {n: np.asarray(data[n]) for n in data.dtype.names}
    import pandas as pd
    df = pd.read_csv(sys.stdin if src == "-" else src)
    return list(df.columns), {c: df[c].to_numpy() for c in df.columns}


def make_x(cols, mode, rate):
    """Build the shared x-axis array + label."""
    if mode in ("ms", "s") and "loop_count" in cols:
        t = (cols["loop_count"].astype(np.float64) - float(cols["loop_count"][0])) / rate
        return (t * 1e3, "time (ms)") if mode == "ms" else (t, "time (s)")
    if mode == "loop" and "loop_count" in cols:
        return cols["loop_count"], "loop_count"
    return np.arange(len(next(iter(cols.values())))), "sample"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?", default="foc_capture.bin",
                    help="*.bin, *.csv, or - for stdin CSV (default: foc_capture.bin)")
    ap.add_argument("--rate", type=float, default=32000.0,
                    help="fast-loop sample rate in Hz for the time axis (default 32000)")
    ap.add_argument("--x", choices=["ms", "s", "sample", "loop"], default="ms",
                    help="x-axis units (default ms)")
    ap.add_argument("--cols", help="comma-separated columns to plot on one panel "
                                    "(overrides --groups)")
    ap.add_argument("--groups", help="comma-separated group names (default: %s; "
                    "available: %s)" % (",".join(DEFAULT_GROUPS), ",".join(GROUPS)))
    ap.add_argument("--xy", action="store_true",
                    help="also save the Iα-Iβ current-vector locus")
    ap.add_argument("--save", help="output PNG (default: <input>.png, /tmp for stdin)")
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    ap.add_argument("--list-cols", action="store_true",
                    help="print available columns and exit")
    args = ap.parse_args()

    names, cols = load_columns(args.input)
    if args.list_cols:
        print(", ".join(names))
        return 0
    n = len(cols[names[0]]) if names else 0
    if n == 0:
        sys.stderr.write("no samples in input (head=0?). arm → spin → stop → dump first; "
                         "see tools/foc_capture.sh\n")
        return 1

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")  # headless-safe; only need a display for --show
    import matplotlib.pyplot as plt

    x, xlabel = make_x(cols, args.x, args.rate)

    if args.cols:
        panels = [("selected", [(c, "-") for c in args.cols.split(",")])]
    else:
        gnames = args.groups.split(",") if args.groups else list(DEFAULT_GROUPS)
        if not args.groups and "shunt_drop" in cols and np.any(cols["shunt_drop"] != -1):
            gnames.append("drop")  # selection was active → show which phase got dropped
        for g in gnames:
            if g not in GROUPS:
                sys.stderr.write(f"unknown group '{g}' (have: {', '.join(GROUPS)})\n")
                return 2
        panels = [GROUPS[g] for g in gnames]

    # Drop columns/panels not present in this capture.
    panels = [(t, [(c, s) for c, s in ser if c in cols]) for t, ser in panels]
    panels = [p for p in panels if p[1]]
    if not panels:
        sys.stderr.write("nothing to plot — requested columns not in input "
                         "(try --list-cols)\n")
        return 1

    fig, axes = plt.subplots(len(panels), 1, sharex=True,
                             figsize=(12, max(2.0 * len(panels), 3.0)), squeeze=False)
    axes = axes[:, 0]
    title = f"{os.path.basename(args.input)}  |  {n} samples"
    if "loop_count" in cols:
        title += f"  |  ~{n / args.rate * 1e3:.1f} ms @ {args.rate / 1e3:.0f} kHz"
    fig.suptitle(title)
    for ax, (ylabel, series) in zip(axes, panels):
        drop_only = len(series) == 1 and series[0][0] == "shunt_drop"
        for c, style in series:
            # shunt_drop is a categorical phase index, not a continuous signal — hold
            # each value until it changes (steps-post) rather than interpolating.
            ds = "steps-post" if c == "shunt_drop" else "default"
            ax.plot(x, cols[c], style, linewidth=1.0, label=c, drawstyle=ds)
        if drop_only:  # label the y ticks with the phase letters
            ax.set_yticks([-1, 0, 1, 2])
            ax.set_yticklabels(["none", "A", "B", "C"])
            ax.set_ylim(-1.4, 2.4)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8, ncol=max(1, len(series)))
    axes[-1].set_xlabel(xlabel)
    fig.tight_layout(rect=(0, 0, 1, 0.98))

    save = args.save or ("/tmp/foc_capture.png" if args.input == "-"
                         else str(Path(args.input).with_suffix(".png")))
    fig.savefig(save, dpi=120)
    sys.stderr.write(f"wrote {save}\n")

    if args.xy:
        if "Ialpha" in cols and "Ibeta" in cols:
            figxy, axxy = plt.subplots(figsize=(6, 6))
            axxy.plot(cols["Ialpha"], cols["Ibeta"], "-", linewidth=0.7)
            axxy.set_aspect("equal", "box")
            axxy.grid(True, alpha=0.3)
            axxy.set(xlabel="Iα (A)", ylabel="Iβ (A)",
                     title="Current-vector locus (clean spin → circle)")
            xysave = save[:-4] + ".xy.png" if save.endswith(".png") else save + ".xy.png"
            figxy.savefig(xysave, dpi=120)
            sys.stderr.write(f"wrote {xysave}\n")
        else:
            sys.stderr.write("--xy: Ialpha/Ibeta not in input, skipping locus\n")

    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
