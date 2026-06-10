#!/usr/bin/env python3
"""6f_e ripple decomposition: trapezoidal back-EMF vs dead-time, from a speed sweep.

Feed it foc_capture dumps taken at SEVERAL different steady speeds (fixed small Iq is ideal,
velocity mode is fine). For each capture it synchronously demodulates the 6th-electrical-harmonic
ripple in Id/Iq against motor_phase, then fits |i6| vs omega_e:

    |i6|(omega_e) ~= slope * omega_e + intercept
      slope     -> back-EMF-harmonic drive (e5,e7 are speed-proportional voltages across ~R)
      intercept -> speed-independent drive (dead-time-class voltage error)

Overlays the prediction from the coast-measured BEMF ratio:  i6_pred = (e5/e1)*psi1*omega_e / R.

Usage:
    tools/foc_capture.sh dump sweep_15revs.bin     # at each speed
    python3 tools/ripple_sweep.py sweep_*.bin --plot

Defaults for this motor: --e5-ratio 0.0753 --psi 172.9e-6 --R 0.098 (coast-cal 2026-06-09).
"""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


def load_capture(bin_path: Path):
    # Reuse the canonical decoder (owns the struct layout) rather than duplicating DTYPE here.
    csv = subprocess.run(
        [sys.executable, str(Path(__file__).parent / "decode_foc_capture.py"), str(bin_path)],
        capture_output=True, text=True, check=True).stdout
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
        f.write(csv)
        name = f.name
    return np.genfromtxt(name, delimiter=",", names=True)


def demod6(d):
    """LS demod of Id/Iq at 6*theta_e. Returns (f_e[Hz], iq_mean, i6_mag, id6, iq6)."""
    th = d["motor_phase"]
    we = np.abs(d["motor_vel"]).mean()
    A = np.vstack([np.cos(6 * th), np.sin(6 * th), np.ones_like(th)]).T

    def mag(x):
        sol, *_ = np.linalg.lstsq(A, x, rcond=None)
        return float(np.hypot(sol[0], sol[1]))

    id6, iq6 = mag(d["Id_meas"]), mag(d["Iq_meas"])
    return we / (2 * np.pi), float(d["Iq_meas"].mean()), float(np.hypot(id6, iq6)), id6, iq6


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bins", nargs="+", type=Path, help="foc_capture dumps at different speeds")
    ap.add_argument("--e5-ratio", type=float, default=0.0753, help="coast-measured e5/e1 (voltage ratio)")
    ap.add_argument("--psi", type=float, default=172.9e-6, help="fundamental flux linkage [V*s/rad]")
    ap.add_argument("--R", type=float, default=0.098, help="phase resistance [ohm]")
    ap.add_argument("--L", type=float, default=3.2e-6, help="phase inductance [H] (for Z at 5f_e)")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    rows = []
    for b in args.bins:
        d = load_capture(b)
        f_e, iq, i6, id6, iq6 = demod6(d)
        we = 2 * np.pi * f_e
        rows.append((b.name, we, f_e, iq, i6, id6, iq6))

    rows.sort(key=lambda r: r[1])
    print(f"{'capture':<28}{'f_e[Hz]':>8}{'rev/s':>7}{'Iq[mA]':>8}{'|i6|[mA]':>10}{'Id6':>7}{'Iq6':>7}")
    for name, we, f_e, iq, i6, id6, iq6 in rows:
        print(f"{name:<28}{f_e:>8.0f}{f_e/6:>7.1f}{iq*1e3:>8.0f}{i6*1e3:>10.1f}{id6*1e3:>7.1f}{iq6*1e3:>7.1f}")

    we_a = np.array([r[1] for r in rows])
    i6_a = np.array([r[4] for r in rows])
    if len(rows) >= 2:
        A = np.vstack([we_a, np.ones_like(we_a)]).T
        (slope, intercept), *_ = np.linalg.lstsq(A, i6_a, rcond=None)
        # Predicted BEMF-driven slope (R-dominated; Z(5f_e) refinement shown alongside)
        slope_pred = args.e5_ratio * args.psi / args.R
        print(f"\nfit: |i6| = {slope*1e6:.1f} uA/(rad/s) * omega_e + {intercept*1e3:.1f} mA")
        print(f"  BEMF-driven prediction (coast e5, R-only): {slope_pred*1e6:.1f} uA/(rad/s)")
        print(f"  -> slope/predicted = {slope/slope_pred:.2f} "
              f"(~1 = trapezoid explains the speed-dependent part)")
        print(f"  -> intercept = dead-time-class floor; 10.4 ns GaN predicts ~"
              f"{(4/np.pi)*(10.4e-9*96e3*6.0 + 10.4e-9*96e3*2.0)/5/args.R*1e3:.0f} mA")
    else:
        print("\n(need >=2 speeds for the slope/intercept decomposition)")

    if args.plot or args.show:
        import matplotlib
        if not args.show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(we_a, i6_a * 1e3, "o", label="measured |i6|")
        wl = np.linspace(0, we_a.max() * 1.1, 50)
        if len(rows) >= 2:
            ax.plot(wl, (slope * wl + intercept) * 1e3, "-", label=f"fit: {slope*1e6:.0f} uA/(rad/s) + {intercept*1e3:.0f} mA")
        ax.plot(wl, args.e5_ratio * args.psi / args.R * wl * 1e3, "--",
                label="BEMF prediction (coast e5, through 0)")
        ax.set(xlabel="omega_e [rad/s]", ylabel="|i6| [mA]",
               title="6f_e current ripple vs speed: slope=trapezoid, floor=dead-time")
        ax.legend()
        ax.grid(alpha=0.3)
        out = "ripple_sweep.png"
        fig.tight_layout()
        fig.savefig(out, dpi=120)
        print(f"wrote {out}")
        if args.show:
            plt.show()


if __name__ == "__main__":
    main()
