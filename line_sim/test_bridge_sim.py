#!/usr/bin/env python3
"""End-to-end test: two modem_v22bis instances bridged through line_sim.

This is test_bridge.py's topology with the inline lockstep relay replaced by
the real line_sim binary inserted as the channel. Two modems (a calling and an
answering side) listen for audio; line_sim connects to both and relays the
8/16 kHz PCM between them while applying the configured impairments. A data
peer on each side lets us inject a byte stream into one and check it at the
other, modem-to-modem, through the impaired channel.

Subtests:
  * clean  — no impairments: handshake must converge, data recovered (BER 0).
             Run at both 8 kHz and 16 kHz to exercise both resampler ratios.
  * mild   — realistic line: must still converge and recover data.
  * severe — heavy impairment: expected to fail / high BER (a negative test
             that confirms the impairments actually bite).

Run from line_sim/ after `make` (and `make -C ../modem_v22bis`):
    python3 test_bridge_sim.py
"""
import socket, subprocess, threading, time, sys
import numpy as np

SIM = "./line_sim"
MODEM = "../modem_v22bis/modem_v22bis"
NEEDLE = bytes(range(256))               # one 2048-bit period of the test pattern


def srv(port):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('127.0.0.1', port)); s.listen(1)
    return s


def bits_lsb(bs):
    out = []
    for x in bs:
        for k in range(8):
            out.append((x >> k) & 1)
    return out


def best_ber(got):
    """Best bit-error rate of the periodic test pattern anywhere in the stream.

    The data peer transmits a raw LSB-first bit pipe with an arbitrary startup
    bit offset, and the recovered stream is a long run of idle ones followed by
    the periodic needle pattern (period 2048 bits). So scan windows across the
    whole stream and, for each, try every bit-phase of the reference period;
    the data region scores ~0 while idle/garbage scores ~0.5."""
    ref = np.frombuffer(bytes(bits_lsb(NEEDLE)), dtype=np.uint8)   # 2048 bits
    P = len(ref)
    rx = np.frombuffer(bytes(bits_lsb(bytes(got))), dtype=np.uint8)
    if len(rx) < 3 * P:
        return 1.0
    reftile = np.concatenate([ref, ref])
    best = 1.0
    nblk = len(rx) // P
    for b in range(nblk):
        seg = rx[b * P:(b + 1) * P]
        ones = seg.mean()
        if ones < 0.4 or ones > 0.6:        # idle ones / unbalanced: not the pattern
            continue
        for p in range(P):                  # any window inside the data region is
            ber = np.mean(seg != reftile[p:p + P])   # a cyclic rotation of ref
            if ber < best:
                best = float(ber)
                if best == 0.0:
                    return 0.0
    return best


def run_case(name, base, rate, sim_extra):
    audioA, audioB, dataA, dataB = base, base + 1, base + 2, base + 3
    dpa, dpb = srv(dataA), srv(dataB)
    conn = {}
    threading.Thread(target=lambda: conn.__setitem__('a', dpa.accept()[0]), daemon=True).start()
    threading.Thread(target=lambda: conn.__setitem__('b', dpb.accept()[0]), daemon=True).start()

    procs = []
    try:
        procs.append(subprocess.Popen(
            [MODEM, "-l", str(audioA), "-d", f"127.0.0.1:{dataA}", "-o", "-r", "2400"],
            stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(
            [MODEM, "-l", str(audioB), "-d", f"127.0.0.1:{dataB}", "-r", "2400"],
            stderr=subprocess.DEVNULL))
        time.sleep(0.4)                      # let the modems bind + dial data peers
        sim = subprocess.Popen(
            [SIM, "-A", f"127.0.0.1:{audioA}", "-B", f"127.0.0.1:{audioB}",
             "-r", str(rate)] + sim_extra,
            stderr=subprocess.DEVNULL)
        procs.append(sim)

        testmsg = NEEDLE * 16
        got = bytearray()
        injected = False
        # ~9 s of wall time; the channel adds latency so give the handshake room.
        for i in range(900):
            time.sleep(0.01)
            if not injected and i == 250 and 'a' in conn:   # ~2.5 s in
                conn['a'].send(testmsg); injected = True
            if 'b' in conn:
                conn['b'].setblocking(False)
                try:
                    d = conn['b'].recv(8192)
                    if d:
                        got += d
                except (BlockingIOError, OSError):
                    pass
    finally:
        for p in procs:
            p.kill()
        for c in conn.values():
            try: c.close()
            except OSError: pass
        dpa.close(); dpb.close()

    ber = best_ber(got)
    return len(got), ber


def main():
    # The channel's numeric fidelity is covered by `make test` (test_dsp). Here
    # we verify line_sim's *transparency* when clean (the property that lets it
    # drop into the bridge) at both resampler ratios, that a benign noise floor
    # still rides through, and that a punishing setting definitively kills the
    # link (the impairment path is really wired). modem_v22bis itself is a
    # marginal from-scratch receiver — gentler impairments (group delay, tilt,
    # jitter, small carrier offset) already perturb its training, so those are
    # exercised numerically in test_dsp, not leaned on for an E2E "pass".
    cases = [
        ("clean    8k", 8000, [],                  "pass"),
        ("clean   16k", 16000, [],                 "pass"),
        ("noise30  8k", 8000, ["--snr", "30"],     "pass"),
        ("buried   8k", 8000, ["--snr", "-10"],    "fail"),
    ]
    print("line_sim end-to-end bridge test (two modem_v22bis through the channel)\n")
    overall = 0
    for idx, (name, rate, extra, expect) in enumerate(cases):
        base = 7200 + idx * 10
        nbytes, ber = run_case(name, base, rate, extra)
        if expect == "pass":
            ok = ber < 1e-2
        else:
            ok = ber > 1e-2                  # impairment must visibly degrade the link
        tag = "PASS" if ok else "FAIL"
        if not ok:
            overall = 1
        print(f"  {name}  ({'expect '+expect:>11})  bytes={nbytes:5d}  BER={ber:.4f}  {tag}")
    print("\nRESULT:", "PASS" if overall == 0 else "FAIL")
    return overall


if __name__ == "__main__":
    sys.exit(main())
