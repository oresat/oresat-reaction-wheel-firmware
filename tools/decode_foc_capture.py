#!/usr/bin/env python3
"""Decode a foc_capture_buf ring dump to chronological CSV on stdout.

Reads two files (the second is auto-discovered next to the first):
    BIN      - raw foc_capture_buf contents (FOC_CAPTURE_LEN × sizeof(foc_capture_t))
    BIN.head - the foc_capture_idx value, written by foc_capture.sh dump

The .head file holds the total-writes-since-arm count, which the decoder uses
to unwrap the ring:
  head == 0           → no samples captured, exits with empty CSV
  0 < head <= LEN     → buffer was stopped before fully wrapping; emit
                        buf[0:head] in chronological order
  head > LEN          → ring fully wrapped; oldest sample is at
                        buf[head % LEN], newest at buf[(head-1) % LEN]
                        Emits LEN samples in chronological order.

If .head is missing, the decoder falls back to "trim trailing all-zero
records" mode (suitable for legacy non-ring dumps).

Layout MUST stay in sync with Firmware/Inc/foc_capture.h. The assert at the
top trips loudly if the dtype gets out of sync.

Usage: python3 tools/decode_foc_capture.py foc_capture.bin > foc_capture.csv
"""
import csv
import sys
from pathlib import Path

import numpy as np

DTYPE = np.dtype(
    [
        ("loop_count", "<u4"),
        ("Ia", "<f4"), ("Ib", "<f4"), ("Ic", "<f4"),
        ("Ialpha", "<f4"), ("Ibeta", "<f4"),
        ("Id_meas", "<f4"), ("Iq_meas", "<f4"),
        ("Id_set", "<f4"), ("Iq_set", "<f4"),
        ("motor_phase", "<f4"),
        ("motor_vel", "<f4"),
        ("Vd_int", "<f4"), ("Vq_int", "<f4"),
        ("mod_d", "<f4"), ("mod_q", "<f4"),
        ("mod_alpha", "<f4"), ("mod_beta", "<f4"),
        ("svm_A", "<f4"), ("svm_B", "<f4"), ("svm_C", "<f4"),
        ("Vbus", "<f4"),
        ("shunt_drop", "<i4"),
    ]
)
assert DTYPE.itemsize == 92, f"struct size drift: {DTYPE.itemsize}"


def load(bin_path):
    """Read a foc_capture .bin (+ optional sibling .bin.head) and return the
    samples as a chronological structured ndarray (oldest first), handling the
    ring-buffer unwrap. Shared by main() and tools/plot_foc_capture.py so the
    unwrap logic lives in exactly one place."""
    bin_path = Path(bin_path)
    head_path = bin_path.with_suffix(bin_path.suffix + ".head")

    buf = np.fromfile(bin_path, dtype=DTYPE)
    LEN = len(buf)

    if head_path.exists():
        head = int(head_path.read_text().strip())
        if head == 0:
            data = buf[:0]
        elif head <= LEN:
            data = buf[:head]
        else:
            # Ring fully wrapped: rotate so oldest sample (at head % LEN) is first
            start = head % LEN
            data = np.concatenate([buf[start:], buf[:start]])
        sys.stderr.write(f"head={head}, emitting {len(data)} samples\n")
    else:
        # Fallback: no head file, trim trailing all-zero records
        nonzero = np.where(buf["loop_count"] != 0)[0]
        data = buf if not len(nonzero) else buf[: nonzero[-1] + 1]
        sys.stderr.write(f"no .head file; trimming to {len(data)} non-zero samples\n")
    return data


def main() -> int:
    if len(sys.argv) != 2:
        sys.stderr.write(__doc__)
        return 2
    data = load(sys.argv[1])
    w = csv.writer(sys.stdout)
    w.writerow(DTYPE.names)
    for row in data:
        w.writerow([row[name] for name in DTYPE.names])
    return 0


if __name__ == "__main__":
    sys.exit(main())
