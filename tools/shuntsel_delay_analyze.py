#!/usr/bin/env python3
"""Measure the duty->sample pipeline lag K from a torque-step foc_capture dump,
and recommend SHUNT_SEL_DELAY_TICKS (= K - 1). Worklog open question Q2-A.

Method (see tools/shuntsel_delay.sh, which produces the dump):
  - The command tick T_cmd is where `Iq_set` steps — the same tick the FOC PI's P-term moves
    the SVM duties, which are latched to hardware at the end of that tick (`svm_*` cross-check).
  - The response tick T_resp is where `Iq_meas` first departs its pre-step noise band. The
    sample read at tick M reflects the duty written at tick M-K, so K = T_resp - T_cmd.
  - The shunt-selection history ring is pushed at the END of each tick (slot `head` at tick M
    holds the duty written at M-1), hence SHUNT_SEL_DELAY_TICKS = K - 1.

Prediction from the timing analysis (config.h comment): K = 2 -> DELAY = 1. This script measures
it. `loop_count` is checked for gaps (dropped ticks would corrupt the count).

Usage:
    python3 tools/shuntsel_delay_analyze.py shuntsel_step.bin
    python3 tools/shuntsel_delay_analyze.py --selftest
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from decode_foc_capture import load  # noqa: E402  (shared ring-unwrap + layout)

MIN_BASELINE = 16   # ticks of pre-step data required for a usable noise estimate
FIT_TICKS = 30      # post-step ticks used by the onset fit
EXPOSURE0 = 2.0 / 3.0  # new-duty exposure (ticks) of the FIRST affected sample: the JDR read
                       # lags one crest (1/3 tick) and the duty went active one tick after the
                       # write -> that sample's crest sits ~2/3 tick into the new duty.


def fit_onset(y, sigma):
    """Grid-fit a delayed first-order rise to the post-step Iq excursion.

    y[K] = Iq_meas(t_cmd + K) - baseline.  Model: 0 for K < K0, else
    A * (1 - exp(-(K - K0 + EXPOSURE0) / tau)).  The closed current loop makes tau ~= the
    current-controller time constant (1/CURRENT_CONTROLLER_BANDWIDTH ~= 8 ticks at 32 kHz),
    NOT the open-loop L/R — far too slow for single-sample threshold detection, hence the fit.
    Returns (k0_best, tau, A, sse_by_k0).
    """
    n = min(len(y), FIT_TICKS)
    yy = y[:n]
    sse_by_k0 = {}
    best = None
    for k0 in range(0, 7):
        ks = np.arange(k0, n)
        e = ks - k0 + EXPOSURE0
        pre = yy[:k0]
        best_k0 = None
        for tau in np.geomspace(2.0, 40.0, 80):
            basis = 1.0 - np.exp(-e / tau)
            denom = float(basis @ basis)
            if denom <= 0:
                continue
            a = float(basis @ yy[k0:]) / denom
            resid = yy[k0:] - a * basis
            sse = float(resid @ resid) + float(pre @ pre)
            if best_k0 is None or sse < best_k0[0]:
                best_k0 = (sse, tau, a)
        sse_by_k0[k0] = best_k0[0]
        if best is None or best_k0[0] < best[0]:
            best = (best_k0[0], k0, best_k0[1], best_k0[2])
    _, k0, tau, a = best
    return k0, tau, a, sse_by_k0


def analyze(data):
    lc = data["loop_count"].astype(np.int64)
    gaps = np.flatnonzero(np.diff(lc) != 1)
    if gaps.size:
        print(f"WARNING: {gaps.size} loop_count gap(s) in the capture (first at row {gaps[0]}) —"
              " tick arithmetic below may be off; re-grab the capture.")

    iq_set = data["Iq_set"]
    iq_meas = data["Iq_meas"].astype(np.float64)

    # All setpoint edges (the orchestrator makes ~5, alternating sign; older captures have 1).
    dset = np.diff(iq_set)
    edge_rows = np.flatnonzero(np.abs(dset) > 1e-6) + 1  # first row WITH each new setpoint
    if edge_rows.size == 0:
        sys.exit("error: Iq_set never changes in this capture — was the step applied while "
                 "the capture was RUNNING?")

    # Cross-check on the first usable edge: SVM duties move the same tick (P-term acts now).
    svm = np.stack([data["svm_A"], data["svm_B"], data["svm_C"]], axis=1)

    # Align + average all edges, each normalized by its own size and sign and referenced to
    # its own local pre-edge baseline. Averaging is what buys enough SNR to pin the onset:
    # a single edge's first affected sample is only ~ (1-exp(-EXPOSURE0/tau)) ~ 2% of the step.
    pre, post = 8, 21
    acc = np.zeros(post)
    cnt = np.zeros(post, dtype=int)
    used = 0
    svm_moved = False
    sigmas = []
    for i, te in enumerate(edge_rows):
        if te < pre:
            continue
        nxt = edge_rows[i + 1] if i + 1 < len(edge_rows) else len(iq_meas)
        win = min(post, int(nxt - te), len(iq_meas) - int(te))
        if win < 8:
            continue
        delta = float(iq_set[te] - iq_set[te - 1])
        local = iq_meas[te - pre:te]
        mu = float(local.mean())
        sigmas.append(float(local.std()) / abs(delta))
        yk = (iq_meas[te:te + win] - mu) / delta
        acc[:win] += yk
        cnt[:win] += 1
        used += 1
        if np.abs(svm[te] - svm[te - 1]).max() > 1e-4:
            svm_moved = True
    if used == 0:
        sys.exit("error: no usable edges (need >= 8 baseline ticks before an edge and >= 8 "
                 "after) — capture window misaligned?")
    valid = cnt > 0
    y = np.where(valid, acc / np.maximum(cnt, 1), 0.0)[valid]
    sigma_norm = float(np.mean(sigmas)) / np.sqrt(used)

    k0, tau, amp, sse_by_k0 = fit_onset(y, sigma_norm)

    profile = {kk: float(y[kk]) / amp if amp else float("nan")
               for kk in range(0, min(6, len(y)))}

    return dict(n_edges=used, k=k0, tau=tau, amp=amp, sse_by_k0=sse_by_k0,
                sigma_norm=sigma_norm, step=float(np.abs(dset).max()),
                svm_moved_same_tick=svm_moved, profile=profile, n=len(iq_meas))


def report(r):
    print(f"capture: {r['n']} ticks   {r['n_edges']} edge(s) averaged   "
          f"step {r['step']:.3f} A   normalized noise {r['sigma_norm']:.1%} of step")
    print(f"svm duties moved on the command tick: {'yes' if r['svm_moved_same_tick'] else 'NO (?)'}")
    print("\nresponse profile (fraction of fitted step amplitude):")
    for kk, frac in r["profile"].items():
        marker = "  <-- fitted onset" if kk == r["k"] else ""
        print(f"  K={kk}: {frac:+7.1%}{marker}")
    print("\nonset fit (delayed first-order, all candidate onsets):")
    sse_min = min(r["sse_by_k0"].values())
    for k0, sse in sorted(r["sse_by_k0"].items()):
        marker = "  <-- best" if sse == sse_min else ""
        print(f"  K0={k0}: SSE={sse:.3e}{marker}")
    k = r["k"]
    print(f"\nfitted closed-loop tau = {r['tau']:.1f} ticks "
          f"(expect ~{1e6 / 4000 / 31.25:.0f} from CURRENT_CONTROLLER_BANDWIDTH; "
          "much larger => check encoder cal / gains)")

    # Confidence: how decisively does the best onset beat the runner-up?
    ranked = sorted(r["sse_by_k0"].items(), key=lambda kv: kv[1])
    margin = ranked[1][1] / ranked[0][1] if ranked[0][1] > 0 else float("inf")
    if margin < 1.3:
        a, b = ranked[0][0], ranked[1][0]
        print(f"\nAMBIGUOUS: K0={a} and K0={b} fit nearly equally (SSE ratio {margin:.2f}, "
              "want > 1.3).")
        print(f"  candidates: DELAY={a - 1} or DELAY={b - 1}. Do NOT pick yet — re-run "
              "tools/shuntsel_delay.sh (it averages 5 edges; a single edge rarely resolves this)")
        if r["n_edges"] == 1:
            print("  (this capture has only 1 edge — that is exactly the underpowered case)")
        return

    print(f"measured pipeline lag K = {k} tick(s), margin {margin:.1f}x  "
          f"(sample read at tick M reflects duty written at M-{k})")
    delay = k - 1
    if 0 <= delay <= 3:
        print(f"RECOMMENDED: #define SHUNT_SEL_DELAY_TICKS {delay}   (Firmware/Inc/config.h)")
        if delay != 1:
            print("NOTE: differs from the reasoned default (1) — double-check with a repeat run.")
    else:
        print(f"K={k} maps to DELAY={delay}, outside the 0..3 ring window — something is off "
              "(loop_count gaps? capture/step misaligned?). Re-run before trusting.")


def selftest():
    """Synthesize step captures with known K in the REALISTIC regime and check the estimator.

    Realistic = the response follows the closed current loop (tau ~ 8-10 ticks, set by
    CURRENT_CONTROLLER_BANDWIDTH), with single-tick noise of a few % of the step during motion.
    This is the regime the 2026-06-09 bench runs exposed: a naive 5-sigma threshold detector
    triggers 1-2 ticks late and jitters run-to-run; the onset fit must not.
    """
    rng = np.random.default_rng(11)

    def synth(k_true, tau_true, edges):
        """Multi-edge capture: alternating setpoint, response = sum of delayed first-order
        transitions toward the current setpoint (good approximation for tau-spaced edges)."""
        n = 256
        edge_ticks = [40 + 33 * i for i in range(edges)]
        iq_set = np.full(n, 0.05, np.float64)
        for i, te in enumerate(edge_ticks):
            iq_set[te:] = 0.30 if i % 2 == 0 else 0.05
        iq_meas = np.full(n, 0.05, np.float64)
        t = np.arange(n)
        for i, te in enumerate(edge_ticks):
            target = 0.30 if i % 2 == 0 else 0.05
            prev = 0.05 if i % 2 == 0 else 0.30
            rise = t >= te + k_true
            expo = (t[rise] - (te + k_true)) + EXPOSURE0
            # superpose the transition: move the remaining gap toward the new target
            iq_meas[rise] += (target - prev) * (1.0 - np.exp(-expo / tau_true)) \
                - (0.0 if i == 0 else 0.0)
        # crude re-baseline: clip the superposition error by recomputing relative shape only
        iq_meas += rng.normal(0, 0.0075, n)
        moving = t >= edge_ticks[0]
        iq_meas[moving] += rng.normal(0, 0.008, moving.sum())
        svm = np.full(n, 0.5, np.float64)
        for i, te in enumerate(edge_ticks):
            svm[te:] = 0.55 if i % 2 == 0 else 0.5
        return dict(loop_count=np.arange(1000, 1000 + n, dtype=np.uint32),
                    Iq_set=iq_set.astype(np.float32),
                    Iq_meas=iq_meas.astype(np.float32),
                    svm_A=svm.astype(np.float32),
                    svm_B=np.full(n, 0.5, np.float32),
                    svm_C=np.full(n, 0.45, np.float32))

    ok_all = True
    cases = [(1, 8.0, 5), (2, 8.0, 5), (3, 8.0, 5), (2, 10.5, 5), (2, 6.0, 5), (2, 8.0, 1)]
    for k_true, tau_true, edges in cases:
        r = analyze(synth(k_true, tau_true, edges))
        good = r["k"] == k_true
        ok_all &= good
        print(f"selftest K_true={k_true} tau={tau_true} edges={edges}: "
              f"K_est={r['k']} tau_est={r['tau']:.1f} n_edges={r['n_edges']} "
              f"{'PASS' if good else 'FAIL'}")
    print("selftest:", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bin", nargs="?", help="foc_capture dump from tools/shuntsel_delay.sh")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())
    if not args.bin:
        ap.error("bin required (or --selftest)")
    report(analyze(load(args.bin)))


if __name__ == "__main__":
    main()
