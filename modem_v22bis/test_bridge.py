#!/usr/bin/env python3
"""End-to-end bridge test for modem_v22bis.

Runs two modem_v22bis binaries (a calling and an answering side) connected
through a lockstep audio relay — a faithful, time-aligned full-duplex transport
of the kind RTP/CLEARMODE provides between two sip_interface bridges — and a
data peer on each side. It injects a byte stream into one data peer and checks
it is recovered at the other, modem-to-modem, through the real harness.

The relay exchanges fixed 40-sample (5 ms) blocks in lockstep; the audio path
must be time-aligned for the V.22bis handshake to converge (as it is over RTP).
A raw bit-pipe has an arbitrary startup bit offset, so the check is at the bit
level (LSB-first) over an aligned window.

Run from modem_v22bis/ after `make`:  python3 test_bridge.py
"""
import socket, subprocess, threading, time, sys

BIN = "./modem_v22bis"
AUDIO_A, AUDIO_B, DATA_A, DATA_B = 7000, 7001, 7100, 7101
BLK = 80                                    # 40 samples = 5 ms per exchange


def srv(port):
    s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('127.0.0.1', port)); s.listen(1); return s


def recvn(s, n):
    buf = b""
    while len(buf) < n:
        d = s.recv(n - len(buf))
        if not d: return buf
        buf += d
    return buf


def bits_lsb(bs):
    out = []
    for x in bs:
        for k in range(8): out.append((x >> k) & 1)
    return out


def main():
    dpa, dpb = srv(DATA_A), srv(DATA_B)
    conn = {}
    threading.Thread(target=lambda: conn.__setitem__('a', dpa.accept()[0]), daemon=True).start()
    threading.Thread(target=lambda: conn.__setitem__('b', dpb.accept()[0]), daemon=True).start()

    mA = subprocess.Popen([BIN, "-l", str(AUDIO_A), "-d", f"127.0.0.1:{DATA_A}", "-o", "-r", "2400"],
                          stderr=subprocess.DEVNULL)
    mB = subprocess.Popen([BIN, "-l", str(AUDIO_B), "-d", f"127.0.0.1:{DATA_B}", "-r", "2400"],
                          stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    a = socket.socket(); a.connect(('127.0.0.1', AUDIO_A))
    b = socket.socket(); b.connect(('127.0.0.1', AUDIO_B))

    a.send(b'\0' * BLK); b.send(b'\0' * BLK)          # prime both sides
    needle = bytes(range(256)); testmsg = needle * 16
    got = bytearray(); injected = False
    for i in range(1400):                              # ~7 s @ 5 ms
        da = recvn(a, BLK); db = recvn(b, BLK)         # lockstep, time-aligned
        if len(da) < BLK or len(db) < BLK: break
        b.send(da); a.send(db)
        if not injected and i == 300 and 'a' in conn:  # ~1.5 s in: inject data
            conn['a'].send(testmsg); injected = True
        if 'b' in conn:
            conn['b'].setblocking(False)
            try:
                d = conn['b'].recv(8192)
                if d: got += d
            except BlockingIOError:
                pass
    mA.kill(); mB.kill()

    tx, rx = bits_lsb(testmsg), bits_lsb(bytes(got))
    best = 1.0
    if len(rx) >= 2500:
        win = 2000
        for off in range(0, len(tx) - win):
            for rs in (300, 600, 1000):
                if rs + win > len(rx): continue
                e = sum(1 for k in range(win) if rx[rs + k] != tx[off + k])
                if e / win < best: best = e / win
    print(f"B data peer received {len(got)} bytes; best bit-BER = {best:.4f}")
    ok = best < 1e-2
    print("RESULT:", "PASS — data recovered modem-to-modem through the bridge" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
