#!/usr/bin/env python3
"""Offline fit for the back-EMF coast-down encoder-offset calibration.

Input: one or more backemf_capture.bin (+ .meta.json sidecars) from tools/backemf_capture.sh.
Pass BOTH a forward and a reverse run for the gold-standard bidirectional fit:

    python3 tools/backemf_cal_fit.py fwd.bin rev.bin --divider 0.3884 --plot

Method: docs/backemf_coastdown_calibration.md — per sample
    delta_i = wrap(atan2(s*v_d, s*v_q)),   s = sign(omega_e)
then ONE pooled linear regression  delta = Delta + k*omega_e  across all runs:
    Delta (intercept) = genuine offset error [elec rad]
    k     (slope)     = lumped sensing delay [s]

Pooling notes:
  * Every run's theta_e is recomputed from its raw encoder samples against the FIRST run's
    phase offset (the "reference baseline"), so runs taken with different boot offsets (e.g.
    different single-point-cal lock pole-pairs — offsets are only meaningful mod 1/p turn)
    pool cleanly. The suggested offset applies ON TOP OF run[0]'s recorded offset.
  * A single one-sided run has a narrow |omega_e| band far from zero: Delta and k are nearly
    collinear and the intercept is a long extrapolation (k uncertainty ~ +/-20 us at typical
    spans). Forward + reverse brackets omega_e = 0 and fixes both. Prefer two runs.

Firmware-convention mirrors (keep in sync):
    encoder.c:encoder_get_phase():  theta_e = wrap_pm_pi(2*pi*p*(pos_turns - offset_turns)) * dir
                                    omega_e = 2*pi*p * dir * omega_mech
    bldc_math.c:park_transform():   v_d =  va*cos + vb*sin ;  v_q = vb*cos - va*sin
    backemf_cal.h:backemf_sample_t: '<HHHHfI' = enc_raw, va, vb, vc, omega_mech, loop_count (16 B)

Clarke here is the full 3-term amplitude-invariant form (NOT the firmware's 2-term one): the
floating-neutral common mode must be rejected without assuming va+vb+vc = 0.

The suggested-offset SIGN follows encoder_phase_cal_finish(); per the method doc §7 verify once
empirically (tools/backemf_capture.sh poke-offset to perturb by +5 deg elec, re-run, fit must
move by -5 deg) before trusting automated write-back.
"""

import argparse
import json
import struct
import sys
from pathlib import Path

import numpy as np

SAMPLE_FMT = "<HHHHfI"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)  # 16
assert SAMPLE_SIZE == 16


def wrap_pi(x):
    return (np.asarray(x) + np.pi) % (2 * np.pi) - np.pi


def circ_mean(x):
    return float(np.arctan2(np.sin(x).mean(), np.cos(x).mean()))


def load(bin_path: Path):
    raw = bin_path.read_bytes()
    meta_path = Path(str(bin_path) + ".meta.json")
    if not meta_path.exists():
        sys.exit(f"error: {meta_path} not found (dump with tools/backemf_capture.sh dump)")
    meta = json.loads(meta_path.read_text())

    n = meta["idx"]
    cap = len(raw) // SAMPLE_SIZE
    if n > cap:
        print(f"warning: {bin_path}: meta idx={n} > capacity {cap}; clamping", file=sys.stderr)
        n = cap
    if n == 0:
        sys.exit(f"error: {bin_path}: 0 samples captured")

    arr = np.frombuffer(raw[: n * SAMPLE_SIZE], dtype=np.dtype([
        ("enc_raw", "<u2"), ("va", "<u2"), ("vb", "<u2"), ("vc", "<u2"),
        ("omega_mech", "<f4"), ("loop_count", "<u4"),
    ]))
    return arr, meta


def preprocess(bin_path: Path, args):
    """Per-run unpack, zero-cal, Clarke, gates. Returns a dict of per-sample arrays + meta."""
    arr, meta = load(bin_path)
    t = (arr["loop_count"] - arr["loop_count"][0]) / args.loop_hz
    pos = arr["enc_raw"].astype(np.float64) / args.cpr
    om = arr["omega_mech"].astype(np.float64)

    # Per-channel zero from this run's mean: over many electrical cycles the back-EMF averages
    # to ~0, so the mean is the channel's DC level. Per-run because the bias can drift per boot.
    v = np.stack([arr["va"], arr["vb"], arr["vc"]]).astype(np.float64)
    zeros = v.mean(axis=1)
    va, vb, vc = v - zeros[:, None]

    valpha = (2.0 / 3.0) * (va - 0.5 * vb - 0.5 * vc)
    vbeta = (vb - vc) / np.sqrt(3.0)
    vmag = np.hypot(valpha, vbeta)
    gate = (np.abs(om) >= args.min_revs) & (vmag >= args.min_vmag)

    return {
        "name": bin_path.name, "meta": meta, "t": t, "pos": pos, "om": om,
        "va": va, "vb": vb, "vc": vc, "valpha": valpha, "vbeta": vbeta,
        "vmag": vmag, "gate": gate, "zeros": zeros,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bins", nargs="*", default=["backemf_capture.bin"], type=Path,
                    help="capture file(s); pass fwd AND rev for the bidirectional fit")
    ap.add_argument("--pole-pairs", type=int, default=6)
    ap.add_argument("--cpr", type=int, default=16384)
    ap.add_argument("--loop-hz", type=float, default=32000.0, help="fast-loop rate for the timebase")
    ap.add_argument("--min-revs", type=float, default=4.0, help="|omega_mech| gate [rev/s] (SNR floor)")
    ap.add_argument("--min-vmag", type=float, default=15.0, help="|v_alphabeta| gate [ADC counts]")
    ap.add_argument("--vref", type=float, default=3.3, help="ADC full-scale volts (for reporting only)")
    ap.add_argument("--divider", type=float, default=1.0,
                    help="PHx_V divider ratio V_pin/V_phase (1.0 = unknown; angles unaffected)")
    ap.add_argument("--force-dir", type=float, choices=(1.0, -1.0), default=None,
                    help="override encoder direction (default: auto-pick the self-consistent one)")
    ap.add_argument("--csv", action="store_true", help="dump per-sample CSV to stdout and exit")
    ap.add_argument("--plot", action="store_true", help="write <first-bin>.png diagnostics")
    ap.add_argument("--show", action="store_true", help="plt.show() interactively (implies --plot)")
    args = ap.parse_args()
    if not args.bins:
        args.bins = [Path("backemf_capture.bin")]

    p = args.pole_pairs
    runs = [preprocess(b, args) for b in args.bins]

    # Common reference baseline = run[0]'s offset. theta_e is pure math on the logged raw encoder
    # angle, so every run can be (re)evaluated against this one offset regardless of what its
    # firmware boot had loaded. Mod-1/p differences (different lock pole-pairs) wrap away.
    off0 = runs[0]["meta"]["phase_offset_turns"]
    fw_dirs = {r["meta"]["encoder_dir"] for r in runs}
    if len(fw_dirs) > 1:
        print(f"warning: runs were taken with different firmware encoder_dir values {fw_dirs}; "
              f"raw data is dir-independent so pooling is still valid", file=sys.stderr)

    # Concatenate
    pos = np.concatenate([r["pos"] for r in runs])
    om = np.concatenate([r["om"] for r in runs])
    valpha = np.concatenate([r["valpha"] for r in runs])
    vbeta = np.concatenate([r["vbeta"] for r in runs])
    vmag = np.concatenate([r["vmag"] for r in runs])
    gate = np.concatenate([r["gate"] for r in runs])
    run_id = np.concatenate([np.full(len(r["pos"]), i) for i, r in enumerate(runs)])
    # Stitched time axis for plotting only (gap between runs is not physical)
    t_plot, t0 = [], 0.0
    for r in runs:
        t_plot.append(r["t"] + t0)
        t0 += r["t"][-1] + 0.05
    t_plot = np.concatenate(t_plot)

    ng = int(gate.sum())
    if ng == 0:
        sys.exit("error: no samples pass the gates")

    def delta_for(dir_val):
        theta = wrap_pi(2 * np.pi * p * (pos - off0)) * dir_val
        w_e = 2 * np.pi * p * dir_val * om
        c, s_ = np.cos(theta), np.sin(theta)
        vd = valpha * c + vbeta * s_
        vq = vbeta * c - valpha * s_
        sgn = np.sign(w_e)
        return theta, w_e, vd, vq, wrap_pi(np.arctan2(sgn * vd, sgn * vq))

    # Direction self-consistency: right dir -> delta ~constant -> circular resultant R ~ 1; wrong
    # dir -> delta sweeps at ~2x electrical rate -> R ~ 0. Catches the boot-default encoder_dir
    # (dir-cal is NOT persisted across reboots).
    R = {dv: float(np.hypot(np.sin(delta_for(dv)[4][gate]).mean(),
                            np.cos(delta_for(dv)[4][gate]).mean())) for dv in (+1.0, -1.0)}
    auto_dir = max(R, key=R.get)
    use_dir = args.force_dir if args.force_dir is not None else auto_dir
    print(f"direction self-consistency R (1=constant delta, 0=sweeping): "
          f"dir=+1: {R[1.0]:.3f}, dir=-1: {R[-1.0]:.3f} -> using dir={use_dir:+.0f}"
          f"{' (forced)' if args.force_dir is not None else ''}")
    if max(R.values()) < 0.5:
        print("warning: NEITHER direction gives a consistent delta — suspect swapped phase-voltage", file=sys.stderr)
        print("         channels, wrong --pole-pairs, or garbage data. Do not trust this fit.", file=sys.stderr)

    theta_e, omega_e, v_d, v_q, delta = delta_for(use_dir)

    if args.csv:
        print("run,t,enc_turns,omega_mech,valpha,vbeta,vmag,theta_e,omega_e,v_d,v_q,delta,gated")
        for i in range(len(pos)):
            print(f"{run_id[i]},{t_plot[i]:.6f},{pos[i]:.6f},{om[i]:.4f},"
                  f"{valpha[i]:.2f},{vbeta[i]:.2f},{vmag[i]:.2f},"
                  f"{theta_e[i]:.5f},{omega_e[i]:.2f},{v_d[i]:.2f},{v_q[i]:.2f},{delta[i]:.5f},{int(gate[i])}")
        return

    for i, r in enumerate(runs):
        m, z = r["meta"], r["zeros"]
        print(f"run[{i}] {r['name']}: {len(r['pos'])} samples ({int(r['gate'].sum())} gated), "
              f"offset={m['phase_offset_turns']:.6f} turns, dir={m['encoder_dir']:+.0f}, "
              f"omega_cut={m['omega_cut_revs']:+.1f} rev/s, zeros A/B/C={z[0]:.1f}/{z[1]:.1f}/{z[2]:.1f}")
    print(f"reference baseline: run[0] offset = {off0:.6f} turns "
          f"(all runs re-evaluated against it; suggestion applies on top of it)")

    if ng < 20:
        sys.exit("error: too few gated samples for a fit — spin faster / lower --min-revs")

    # Pooled fit of wrapped residuals about the circular mean (no np.unwrap: with scatter it
    # accumulates a random walk and fabricates a huge slope).
    we, draw = omega_e[gate], delta[gate]
    cmean = circ_mean(draw)
    dl = cmean + wrap_pi(draw - cmean)
    A = np.vstack([np.ones_like(we), we]).T
    (Delta, k), *_ = np.linalg.lstsq(A, dl, rcond=None)
    resid = dl - (Delta + k * we)
    Delta = float(wrap_pi(Delta))

    sides = "fwd+rev" if (we.min() < 0 < we.max()) else "ONE-SIDED"
    print(f"\npooled fit over {ng} samples, omega_e span [{we.min():.0f}, {we.max():.0f}] rad/s ({sides}):")
    if sides == "ONE-SIDED":
        print("  note: one-sided span -> Delta/k are nearly collinear and the intercept is a long")
        print("        extrapolation. Add a run in the other direction for a trustworthy fit.")
    print(f"  Delta (intercept) = {Delta:+.5f} rad elec = {np.degrees(Delta):+.2f} deg elec")
    print(f"  k (slope)         = {k*1e6:+.2f} us  (lumped sensing delay; "
          f"{np.degrees(abs(k)*np.abs(we).max()):.2f} deg at max speed)")
    print(f"  residual sigma    = {np.degrees(resid.std()):.3f} deg elec "
          f"(angle-domain ripple lives here: eccentricity/LUT content + noise)")

    # Per-run intercepts holding the POOLED k fixed — the §4.3 agreement gate, now on a common
    # baseline AND a common slope, so it isolates genuine run-to-run baseline disagreement.
    for i, r in enumerate(runs):
        m = gate & (run_id == i)
        if m.sum() >= 10:
            di = circ_mean(wrap_pi(delta[m] - k * omega_e[m]))
            print(f"  run[{i}] intercept (pooled k) = {np.degrees(wrap_pi(di)):+.2f} deg elec")

    delta_mech_turns = (Delta / use_dir) / (2 * np.pi * p)
    print(f"\nsuggested encoder phase offset (mech turns, on run[0]'s baseline):")
    print(f"  reference : {off0:+.6f}")
    print(f"  + delta   : {delta_mech_turns:+.6f}  ({Delta:+.4f} rad elec / dir / 2pi*p)")
    print(f"  new       : {off0 + delta_mech_turns:+.6f}   << VERIFY SIGN once per method doc §7")

    if args.divider != 1.0:
        psi = vmag[gate] * (args.vref / 4095.0) / args.divider / np.abs(we)
        print(f"\npsi_m estimate: {psi.mean()*1e6:.1f} uV*s (flux linkage, amplitude-invariant)")

    if args.plot or args.show:
        import matplotlib
        if not args.show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 2, figsize=(12, 9))
        for ch, lbl in ((np.concatenate([r["va"] for r in runs]), "A"),
                        (np.concatenate([r["vb"] for r in runs]), "B"),
                        (np.concatenate([r["vc"] for r in runs]), "C")):
            ax[0, 0].plot(t_plot, ch, lw=0.6, label=lbl)
        ax[0, 0].set(title="phase voltages (zeroed counts; runs stitched)", xlabel="t [s]")
        ax[0, 0].legend()
        col = run_id[gate] if len(runs) > 1 else t_plot[gate]
        sc = ax[0, 1].scatter(valpha[gate], vbeta[gate], c=col, s=2, cmap="coolwarm")
        ax[0, 1].set(title="alpha-beta hodograph (gated) — want a circle",
                     xlabel="v_alpha", ylabel="v_beta")
        ax[0, 1].set_aspect("equal")
        fig.colorbar(sc, ax=ax[0, 1], label="run" if len(runs) > 1 else "t [s]")
        ax[1, 0].scatter(we, np.degrees(wrap_pi(dl)), s=2, c=run_id[gate], cmap="coolwarm")
        wl = np.linspace(we.min(), we.max(), 100)
        ax[1, 0].plot(wl, np.degrees(wrap_pi(Delta + k * wl)), "k-", lw=1.5, label="pooled fit")
        ax[1, 0].axvline(0, color="gray", lw=0.5)
        ax[1, 0].set(title=f"delta vs omega_e: Delta={np.degrees(Delta):+.2f}deg, k={k*1e6:+.1f}us",
                     xlabel="omega_e [rad/s]", ylabel="delta [deg elec]")
        ax[1, 0].legend()
        ax[1, 1].scatter(np.abs(we), vmag[gate], s=2, c=run_id[gate], cmap="coolwarm")
        ax[1, 1].set(title="|v| vs |omega_e| — want a line through 0",
                     xlabel="|omega_e| [rad/s]", ylabel="|v_alphabeta| [counts]")
        fig.tight_layout()
        out = str(args.bins[0]) + ".png"
        fig.savefig(out, dpi=120)
        print(f"\nwrote {out}")
        if args.show:
            plt.show()


if __name__ == "__main__":
    main()
