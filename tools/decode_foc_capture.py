#!/usr/bin/env python3
"""Decode a foc_capture_buf dump (raw little-endian struct array) to CSV on stdout.

Layout MUST stay in sync with Firmware/Inc/foc_capture.h's foc_capture_t.
Each record is 88 bytes (22 × 4-byte fields). No padding because all fields
are 4-byte aligned.

Usage: python3 tools/decode_foc_capture.py foc_capture.bin > foc_capture.csv
"""
import csv
import sys

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
    ]
)
assert DTYPE.itemsize == 88, f"struct size drift: {DTYPE.itemsize}"


def main() -> int:
    if len(sys.argv) != 2:
        sys.stderr.write(__doc__)
        return 2
    data = np.fromfile(sys.argv[1], dtype=DTYPE)
    # Drop trailing all-zero records (capture stopped early)
    nonzero = np.where(data["loop_count"] != 0)[0]
    if len(nonzero):
        data = data[: nonzero[-1] + 1]
    w = csv.writer(sys.stdout)
    w.writerow(DTYPE.names)
    for row in data:
        w.writerow([row[name] for name in DTYPE.names])
    return 0


if __name__ == "__main__":
    sys.exit(main())
