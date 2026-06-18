#!/usr/bin/env python3
"""Offline verification of the V.22bis transmitter, independent of the (not yet
complete) receiver. Coherently demodulates a forced-data-mode TX capture and
compares the recovered symbols against the exact expected sequence computed from
the same PRBS the C dumper uses.

  make: gcc test_tx_dump.c v22bis_tx.o -lm -o test_tx_dump
  run : ./test_tx_dump 2400 calling > /tmp/tx.s16
        python3 tools/tx_offline_check.py 2400 /tmp/tx.s16 1200
  (last arg is the carrier of the captured signal: 1200 for a calling TX,
   2400 for an answering TX). Prints SER and PASS/FAIL.
"""
import numpy as np
import sys

rate = int(sys.argv[1]); fname = sys.argv[2]; fc = float(sys.argv[3])
x = np.fromfile(fname, dtype='<i2').astype(np.float64)
fs, baud = 8000.0, 600.0
phase_steps = [1, 0, 2, 3]
CONST = np.array([1+1j, 3+1j, 1+3j, 3+3j, -1+1j, -1+3j, -3+1j, -3+3j,
                  -1-1j, -3-1j, -1-3j, -3-3j, 1-1j, 1-3j, 3-1j, 3-3j])

# Expected symbol indices: replicate the C TX chain exactly.
prbs = 0x1234
def src():
    global prbs
    prbs ^= (prbs << 13) & 0xffffffff; prbs ^= prbs >> 17; prbs ^= (prbs << 5) & 0xffffffff
    return prbs & 1
reg = 0; ones = 0
def scr():
    global reg, ones
    bit = src()
    if ones >= 64: bit ^= 1; ones = 0
    out = (bit ^ (reg >> 13) ^ (reg >> 16)) & 1
    reg = ((reg << 1) | out) & 0xffffffff
    ones = ones + 1 if out else 0
    return out
state = 0; exp = []
for _ in range(1500):
    b = (scr() << 1) | scr(); state = (state + phase_steps[b]) & 3
    inner = ((scr() << 1) | scr()) if rate == 2400 else 1
    exp.append((state << 2) | inner)
exp = np.array(exp)

n = np.arange(len(x))
bb = x * np.exp(-1j * 2 * np.pi * fc * n / fs)
def rrc_proto(beta, sps, span):
    N = int(sps * span); t = (np.arange(N) - (N - 1) / 2) / sps; out = np.zeros(N)
    for i, tt in enumerate(t):
        if abs(tt) < 1e-9: out[i] = 1 + beta * (4 / np.pi - 1)
        elif beta > 0 and abs(abs(tt) - 1 / (4 * beta)) < 1e-9:
            out[i] = (beta / np.sqrt(2)) * ((1 + 2 / np.pi) * np.sin(np.pi / (4 * beta)) +
                                            (1 - 2 / np.pi) * np.cos(np.pi / (4 * beta)))
        else:
            out[i] = (np.sin(np.pi * tt * (1 - beta)) + 4 * beta * tt * np.cos(np.pi * tt * (1 + beta))) / \
                     (np.pi * tt * (1 - (4 * beta * tt) ** 2))
    return out / np.sqrt(np.sum(out ** 2))
mf = np.convolve(bb, rrc_proto(0.75, fs / baud, 9), 'same')

Ts = fs / baud
def decode_at(offset):
    idxs = np.arange(offset, len(mf) - 2, Ts)
    syms = (np.interp(idxs, np.arange(len(mf)), mf.real)
            + 1j * np.interp(idxs, np.arange(len(mf)), mf.imag))
    p = np.mean(np.abs(syms[20:]) ** 2)
    if p < 1e-9: return None, 1.0
    syms = syms * np.sqrt(10.0 / p)
    d = np.abs(syms[:, None] - CONST[None, :])
    return np.argmin(d, axis=1), np.mean(np.min(d, axis=1) ** 2)

best = None
for off in np.arange(0, Ts, 0.25):
    meas, err = decode_at(off)
    if meas is not None and (best is None or err < best[2]):
        best = (off, meas, err)
off, meas, evm = best

def ser_at_lag(lag):
    a, b = meas, exp
    if lag >= 0: a = a[lag:]; b = b[:len(a)]
    else: b = b[-lag:]; a = a[:len(b)]
    m = min(len(a), len(b))
    if m < 200: return 1.0, 0
    return np.mean(a[20:m] != b[20:m]), m - 20
bestlag = min(range(0, 30), key=lambda L: ser_at_lag(L)[0])
ser, ncmp = ser_at_lag(bestlag)
print(f"rate={rate} fc={fc:.0f} offset={off:.2f} EVM={evm:.4f} lag={bestlag} "
      f"symbols={ncmp} SER={ser:.4f}  {'PASS' if ser < 1e-3 else 'FAIL'}")
sys.exit(0 if ser < 1e-3 else 1)
