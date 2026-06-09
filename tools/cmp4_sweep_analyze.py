#!/usr/bin/env python3
"""Analyze a CMP4 sweep (tools/cmp4_sweep.sh) and recommend the ADC trigger position.

The sweep holds a fixed PWM duty vector and steps the HRTIM Timer A CMP4 ADC-trigger
across the period, recording mean/variance of the three (pre-CM-removal) phase currents
at each position — an equivalent-time-sampled reconstruction of the intra-period current
waveform. Each phase's PWM switching edge sits at a KNOWN counter tick:

    E_x = (1 - duty_x) * PWM_CNTR        (up-slope; duty from duties.bin)

but *appears* in the reconstruction at X* = E_x - D, where D is the total trigger->sample
delay (trigger latency + S&H aperture centroid + oversampling-burst centroid). So:

    D = E_x - X*        ->   recommended CMP4 = PWM_CNTR - D    (burst centered on crest)

X* is located as the |gradient|-weighted centroid of the step/kick the edge leaves in that
phase's mean-current curve. The 4x oversampling burst smears the step into a staircase
~ +/-1.5*Tconv wide; the centroid lands mid-staircase, which is exactly the burst centroid —
so D needs no analytic correction terms at all.

Usage:
    python3 tools/cmp4_sweep_analyze.py cmp4_sweep_out/ [--save out.png] [--show]
    python3 tools/cmp4_sweep_analyze.py --selftest

Outputs the per-edge delay table, the recommendation, and (with matplotlib) a two-panel
figure: reconstructed waveforms with fiducials, and the per-phase variance map.
"""
import argparse
import glob
import os
import re
import struct
import sys

import numpy as np

PWM_CNTR = 24000
HRTIM_TICK_NS = 1.0 / 4.608  # ns per tick (144 MHz * 32)
TCONV_TICKS = 1920           # one 12-bit conversion @ 36 MHz ADC clock, in HRTIM ticks
CMP4_MIN, CMP4_MAX = 64, PWM_CNTR - 64

REC_FMT = "<II4f3f"  # adcsweep: req_n, count, sum[A,B,C,Vbus], sumsq[A,B,C]
REC_SIZE = struct.calcsize(REC_FMT)


def load_sweep(outdir):
    """Read cmp4_*.bin + duties.bin + meta.txt -> dict of arrays."""
    files = glob.glob(os.path.join(outdir, "cmp4_*.bin"))
    if not files:
        sys.exit(f"error: no cmp4_*.bin in {outdir}")
    rows = []
    for f in files:
        m = re.search(r"cmp4_(\d+)\.bin$", f)
        if not m:
            continue
        blob = open(f, "rb").read()
        if len(blob) != REC_SIZE:
            sys.exit(f"error: {f} is {len(blob)} B, expected {REC_SIZE}")
        req_n, count, sA, sB, sC, sV, qA, qB, qC = struct.unpack(REC_FMT, blob)
        if count == 0:
            continue
        mean = np.array([sA, sB, sC]) / count
        var = np.array([qA, qB, qC]) / count - mean**2
        rows.append((int(m.group(1)), mean, np.maximum(var, 0.0), sV / count))
    rows.sort(key=lambda r: r[0])
    cmp4 = np.array([r[0] for r in rows], dtype=float)
    means = np.stack([r[1] for r in rows])   # (N, 3)
    varis = np.stack([r[2] for r in rows])   # (N, 3)
    vbus = np.array([r[3] for r in rows])

    duties_path = os.path.join(outdir, "duties.bin")
    duties = np.array(struct.unpack("<3f", open(duties_path, "rb").read(12)))

    meta = {}
    meta_path = os.path.join(outdir, "meta.txt")
    if os.path.exists(meta_path):
        for line in open(meta_path):
            if "=" in line:
                k, v = line.strip().split("=", 1)
                meta[k] = v
    return dict(cmp4=cmp4, means=means, varis=varis, vbus=vbus, duties=duties, meta=meta)


def edge_ticks(duties):
    """Up-slope switching-edge position per phase, in counter ticks (see pwm.c mapping)."""
    return (1.0 - duties) * PWM_CNTR


def find_edge_appearance(cmp4, mean, e_tick, lead=9000, lag=500, clip_frac=0.3):
    """|gradient|-weighted centroid of the edge feature in one phase's mean curve.

    Searches [e_tick - lead, e_tick + lag] (the feature leads its fiducial by D > 0).
    Returns (X*, weight_sum) or (None, 0) if the window has no usable signal.
    """
    grad = np.abs(np.gradient(mean, cmp4))
    # Light 3-point smoothing: the edge feature is a ~3*Tconv-wide staircase (many sweep steps),
    # so smoothing costs no localization but knocks down single-point noise spikes.
    grad = np.convolve(grad, [0.25, 0.5, 0.25], mode="same")
    sel = (cmp4 >= e_tick - lead) & (cmp4 <= e_tick + lag)
    if sel.sum() < 5:
        return None, 0.0
    w = grad[sel].copy()
    x = cmp4[sel]
    thresh = clip_frac * w.max()
    w[w < thresh] = 0.0
    if w.sum() <= 0:
        return None, 0.0
    return float((w * x).sum() / w.sum()), float(w.sum())


def analyze(data, lead=9000, lag=500):
    cmp4, means, duties = data["cmp4"], data["means"], data["duties"]
    edges = edge_ticks(duties)
    results = []
    for p, name in enumerate("ABC"):
        xstar, wsum = find_edge_appearance(cmp4, means[:, p], edges[p], lead, lag)
        if xstar is None:
            results.append((name, edges[p], None, None, 0.0))
            continue
        d = edges[p] - xstar
        results.append((name, edges[p], xstar, d, wsum))

    usable = [(d, w) for (_, _, _, d, w) in results if d is not None and d > 0]
    if not usable:
        return results, None, None
    dvals = np.array([d for d, _ in usable])
    wvals = np.array([w for _, w in usable])
    d_mean = float((dvals * wvals).sum() / wvals.sum())
    rec = int(round(np.clip(PWM_CNTR - d_mean, CMP4_MIN, CMP4_MAX)))
    return results, d_mean, rec


def report(data, results, d_mean, rec):
    duties = data["duties"]
    print(f"duties: A={duties[0]:.4f} B={duties[1]:.4f} C={duties[2]:.4f}   "
          f"Vbus={data['vbus'].mean():.2f} V   points={len(data['cmp4'])}")
    print(f"{'phase':>5} {'edge E [tick]':>14} {'seen X* [tick]':>15} "
          f"{'D [tick]':>9} {'D [ns]':>8}")
    for name, e, xstar, d, _ in results:
        if d is None:
            print(f"{name:>5} {e:>14.0f} {'(not found)':>15} {'-':>9} {'-':>8}")
        else:
            print(f"{name:>5} {e:>14.0f} {xstar:>15.0f} {d:>9.0f} {d * HRTIM_TICK_NS:>8.1f}")
    if rec is None:
        print("\nNO USABLE EDGES — check the waveform plot; widen --lead/raise mod and re-sweep.")
        return
    print(f"\ncombined D = {d_mean:.0f} ticks = {d_mean * HRTIM_TICK_NS:.1f} ns "
          f"(burst centroid incl. latency, aperture, 1.5*Tconv)")
    print(f"RECOMMENDED: #define ADC_TRIG_CMP4_TICKS {rec}u   (Firmware/Inc/config.h)")
    # Variance sanity: the recommendation should sit in a quiet region.
    cmp4, varis = data["cmp4"], data["varis"]
    vtot = varis.sum(axis=1)
    i_rec = int(np.argmin(np.abs(cmp4 - rec)))
    print(f"total current variance @ recommended: {vtot[i_rec]:.3e} A^2 "
          f"(sweep min {vtot.min():.3e}, max {vtot.max():.3e})")
    if vtot[i_rec] > 3 * vtot.min():
        print("WARNING: variance at the recommendation is well above the sweep minimum —")
        print("         eyeball the plots before trusting the fiducial fit.")


def plot(data, results, rec, save=None, show=False):
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not available — skipping plots)", file=sys.stderr)
        return
    cmp4, means, varis = data["cmp4"], data["means"], data["varis"]
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    colors = dict(zip("ABC", ("C0", "C1", "C2")))
    for p, name in enumerate("ABC"):
        ax1.plot(cmp4, means[:, p], color=colors[name], label=f"I{name} mean")
        ax2.semilogy(cmp4, np.maximum(varis[:, p], 1e-12), color=colors[name],
                     label=f"I{name} var")
    for name, e, xstar, d, _ in results:
        ax1.axvline(e, color=colors[name], ls=":", alpha=0.7)
        if xstar is not None:
            ax1.axvline(xstar, color=colors[name], ls="--", alpha=0.7)
    if rec is not None:
        for ax in (ax1, ax2):
            ax.axvline(rec, color="k", lw=1.5, label="recommended CMP4" if ax is ax1 else None)
    ax1.axvline(PWM_CNTR, color="gray", lw=1, alpha=0.5)
    ax1.set_ylabel("mean current [A]")
    ax1.legend(ncol=4, fontsize=8)
    ax1.set_title("CMP4 sweep: reconstructed intra-period waveform "
                  "(dotted = true edge E, dashed = edge as seen X*)")
    ax2.set_ylabel("variance [A$^2$]")
    ax2.set_xlabel("CMP4 trigger position [HRTIM ticks]  (24000 = crest)")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130)
        print(f"wrote {save}")
    if show:
        plt.show()


def selftest():
    """Synthetic sweep with known D -> assert the estimator recovers it."""
    rng = np.random.default_rng(8)
    d_true = 3100.0
    cmp4 = np.arange(300.0, 23701.0, 200.0)
    duties = np.array([0.545, 0.487, 0.468])  # alpha+beta-ish fixed vector
    edges = edge_ticks(duties)
    means = np.zeros((len(cmp4), 3))
    substeps = (np.arange(4) - 1.5) * TCONV_TICKS  # 4x burst sub-sample offsets
    for p in range(3):
        base = 0.9 * (1 if p == 0 else -0.45)          # locked-rotor DC levels
        for i, x in enumerate(cmp4):
            acc = 0.0
            for s in substeps:                          # burst average, sample at x + D + s
                t = x + d_true + s
                v = base + 0.004 * np.sin(2 * np.pi * t / PWM_CNTR)  # mild ripple
                for q in range(3):                      # each edge kicks every phase
                    amp = 0.12 if q == p else 0.03
                    if t >= edges[q]:
                        v += amp
                acc += v
            # Noise scaled for the real dwell: per-tick sigma ~16 mA / sqrt(1024) ~ 0.5 mA.
            means[i, p] = acc / 4 + rng.normal(0, 0.0005)
    data = dict(cmp4=cmp4, means=means,
                varis=np.full((len(cmp4), 3), 1e-4), vbus=np.full(len(cmp4), 8.0),
                duties=duties, meta={})
    results, d_mean, rec = analyze(data)
    err = abs(d_mean - d_true)
    print(f"selftest: D_true={d_true:.0f}  D_est={d_mean:.0f}  err={err:.0f} ticks "
          f"({err * HRTIM_TICK_NS:.1f} ns)  rec={rec}")
    report(data, results, d_mean, rec)
    ok = err < 150  # 150 ticks ~= 33 ns
    print("selftest:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("outdir", nargs="?", help="sweep output dir (from tools/cmp4_sweep.sh)")
    ap.add_argument("--lead", type=int, default=9000,
                    help="search window lead before each edge fiducial [ticks]")
    ap.add_argument("--lag", type=int, default=500, help="search window lag after [ticks]")
    ap.add_argument("--save", help="write the figure to this PNG")
    ap.add_argument("--show", action="store_true", help="open the figure interactively")
    ap.add_argument("--selftest", action="store_true",
                    help="run the estimator on synthetic data with known D")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())
    if not args.outdir:
        ap.error("outdir required (or --selftest)")

    data = load_sweep(args.outdir)
    results, d_mean, rec = analyze(data, args.lead, args.lag)
    report(data, results, d_mean, rec)
    if args.save or args.show:
        plot(data, results, rec, save=args.save, show=args.show)
    elif rec is not None:
        default_png = os.path.join(args.outdir, "cmp4_sweep.png")
        plot(data, results, rec, save=default_png)


if __name__ == "__main__":
    main()
