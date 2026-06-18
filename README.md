# sip_modem


⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️  This is slopware ⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️

A small C suite that bridges a **SIP/RTP voice call** to a **classic FSK modem
data channel** (ITU-T V.21 or V.23), exposing the decoded byte stream as a plain
TCP connection. It lets you place or receive a SIP call and talk to a modem-era
service (BBS, Bildschirmtext/BTX centre, terminal) over IP.

> **Public domain.** This is free and unencumbered software released into the
> public domain — see [License](#license) at the bottom.

## Architecture

Two independent programs, connected by a raw-audio TCP link. Audio on the wire is
**signed 16-bit little-endian PCM, mono, 8 kHz**.

```
   SIP/RTP                  s16le 8 kHz audio              bytes
  (UDP 5060)                  over TCP                  over TCP
      │                          │                          │
      ▼                          ▼                          ▼
┌────────────────┐  RTP⇄PCM  ┌──────────────┐  modem    ┌──────────────┐
│ sip_interface  │──────────▶│  modem_fsk   │──────────▶│  data peer   │
│  (SIP + RTP,   │◀──────────│ (V.21/V.23   │◀──────────│ (BBS, BTX,   │
│   G.711 aLaw)  │           │  modulation) │           │  telnet, …)  │
└────────────────┘           └──────────────┘           └──────────────┘
   registers to              listens for audio           listens for the
   a SIP registrar,          (-l), demodulates to         byte stream and
   dials -c host:port        bytes, dials -d host:port     drives the service
   for audio
```

- **`sip_interface`** registers to a SIP registrar (MD5 digest auth), answers
  inbound INVITEs, and bridges each call's RTP audio (G.711 A-law, PT 8, 20 ms
  frames) to a TCP peer as s16le PCM. That TCP peer is `modem_fsk`.
- **`modem_fsk`** listens for the PCM audio, demodulates it to a byte stream, and
  dials out to the actual data service. It modulates bytes from the service back
  into audio. One unified program covers V.21 (300 bps full-duplex), Bell 103
  (300 bps full-duplex, the US counterpart of V.21), and V.23 (1200/75 bps
  asymmetric); pick with `-M`.
- The **data peer** is anything that speaks bytes over TCP (not part of this repo).

## TCP sample transfer protocol

The link between `sip_interface` and `modem_fsk` (the middle arrow above) is a
plain, **continuous, unframed byte stream** of audio samples — there is no
packetisation, length prefix, or sequence number on the wire. Each side simply
reads and writes raw sample bytes; cadence and framing live in the timing, not
in markers. (The optional [`-H` stream header](#stream-header--h) is the one
exception: a single fixed prefix sent *once* before the stream begins.)

### Connection & framing

- **Direction.** `sip_interface` is the TCP **client** — it dials `-c host:port`
  once the call has media. `modem_fsk` is the TCP **server** (`-l port`,
  one accepted socket per session). The socket is **bidirectional / full-duplex**
  and non-blocking, with `TCP_NODELAY` set so 20 ms frames are not Nagle-delayed.
- **Sample format** depends on the negotiated SIP codec, and is the *same in both
  directions* for a given call:

  | Call codec | TCP sample format | Bytes per 20 ms frame |
  |------------|-------------------|-----------------------|
  | PCMA (G.711, default) | s16le, mono, **8 kHz** | 320 (160 samples) |
  | G.722 (`-g`) | s16le, mono, **16 kHz** | 640 (320 samples) |
  | CLEARMODE (`-T`) | **raw octets** (no PCM), 8000 B/s | 160 |

  `sip_interface` decodes inbound RTP to this format and encodes the outbound
  direction back from it; `modem_fsk` only understands **s16le / 8 kHz / mono**
  (so G.722 and CLEARMODE streams need a different consumer).

### The two directions

- **RX (RTP → TCP), "what the caller sent."** For every inbound RTP packet
  (~one per 20 ms), `sip_interface` decodes the payload (A-law→PCM, G.722→PCM,
  or raw octets for CLEARMODE) and **writes one frame** down the TCP socket.
- **TX (TCP → RTP), "what to play to the caller."** On a fixed **20 ms tick**,
  `sip_interface` pulls up to one frame's worth of bytes from the TCP socket,
  encodes it, and emits one RTP packet (160 octets, either codec). If fewer than
  a full frame's bytes are buffered, it takes what's there and **zero-pads** the
  rest (CLEARMODE pads with `0xFF`).

### Clocking, idle fill & back-pressure

- **Idle fill keeps the consumer clocked.** If no RTP arrived during a 20 ms
  window, `sip_interface` still pushes one frame of **silence** down the TCP
  side (`0x00` PCM, or `0xFF` for CLEARMODE) so the demodulator's sample clock
  never stalls. This is suppressed by `-i` (ignore inbound RTP).
- **Back-pressure (TX, TCP → RTP).** `sip_interface` reads from TCP only when its
  small staging buffer (2560 B, ≥ 4 frames) has room, otherwise it leaves the
  bytes in the kernel socket buffer — that is what naturally throttles a fast
  sender. A slow data peer simply fills the pipe and waits.
- **Lossy on overrun (RX, RTP → TCP).** The RTP→TCP writes are non-blocking and
  **drop excess bytes on `EAGAIN`** rather than block, deliberately preserving
  the real-time audio cadence: if the TCP consumer can't keep up, samples are
  lost instead of the call stalling. (The one-shot `-H` header is the exception —
  it is written reliably, waiting up to 5 s, since losing it would desync the
  framing.)
- **Teardown.** Either side closing the TCP connection ends the bridge; a hard
  socket error closes it too. The SIP/RTP call is torn down with a BYE.

## Build

Per subproject, or all at once from the top level:

```sh
make              # builds sip_interface/, modem_fsk/, modem_v22bis/, line_sim/
make clean
```

Requires a C99 compiler, `-lm`, and `-lpthread`. No external dependencies. The
`modem_v22bis` interop test (`make test` in that subdir) additionally links
`libspandsp` — it is optional and not part of the default build.

## Components

| Path | What it is |
|------|------------|
| `sip_interface/` | SIP UA + RTP bridge. `sip_interface.c` (REGISTER, event loop), `call.c` (per-call thread, SDP/RTP, ACK/BYE), `alaw.c` (G.711 A-law), `g722.c` (G.722 wideband, built-in). |
| `modem_fsk/` | Combined V.21/Bell 103/V.23 modem. `fsk.c`/`fsk.h` (parameterized modulator + Goertzel demodulator + framers), `modem_fsk.c` (TCP session bridge). |
| `modem_v22bis/` | ITU-T V.22bis modem (2400 bps 16-QAM / 1200 bps QPSK, 600 baud). `v22bis_tx.c` (scrambler, differential 16-QAM, RRC shaper, carrier + training state machine), `v22bis_rx.c` (matched filter, AGC, Gardner timing, adaptive LMS equalizer, carrier PI loop, slicer, descrambler, handshake), `v22bis.h`, `v22bis_rrc.h` (generated filter taps), `modem_v22bis.c` (TCP session bridge). |
| `line_sim/` | Voiceband line/channel simulator placed between two modems. `channel.c` (per-direction impairment pipeline at 48 kHz), `resample.c` (polyphase 8k/16k↔48k), `filters.c` (biquads, all-pass, Hilbert), `line_sim.c` (lockstep relay + clock slip). Configurable frequency response, group-delay, non-linear distortion, noise, frequency offset, phase jitter, A-law, clock slip. See `line_sim/README.md` for the standards behind the impairment values. |

## Usage

### sip_interface

```
sip_interface -u sip:user@registrar -p password -c host:port
              [-P sip_port] [-e expires] [-i] [-m N] [-H] [-D target] [-L]
```

- `-u` SIP AoR (`sip:user@host`)   `-p` password
  - The password may instead be supplied in the `SIP_PASSWORD` environment
    variable (used when `-p` is omitted). Prefer this: a command-line `-p`
    is visible to any local user via `ps aux` / `/proc/<pid>/cmdline`.
- `-c` TCP host:port to bridge audio with (this is `modem_fsk`'s `-l` port)
- `-P` local SIP port (default 5060)   `-e` register expiry seconds (default 120)
- `-i` ignore inbound RTP (don't send RTP-derived bytes down TCP)
- `-m` max concurrent calls (0 = unlimited)
- `-H` prefix the audio connection with a [stream header](#stream-header--h)
  carrying the received INVITE before the PCM (**inbound calls only**)
- `-g` offer/accept **G.722** wideband (RTP PT 9), preferred over PCMA when both
  are available; falls back to PCMA otherwise. With G.722 the TCP audio stream is
  **16 kHz** s16le instead of 8 kHz (a `-H` header then declares `16000`). Default
  off — PCMA/8 kHz only. The codec is built-in (no external library). Note
  `modem_fsk` only handles 8 kHz, so it rejects a G.722 (16 kHz) stream — use a
  wideband consumer for G.722 calls.
- `-T` **CLEARMODE** (RFC 4040): exclusively negotiate a **transparent 64 kbit/s**
  channel (dynamic RTP PT, `CLEARMODE/8000`). No codec — the TCP side then carries
  **raw octets** (8000 B/s), not PCM; idle fill is `0xFF`. Rejects the call (415)
  if the peer won't do CLEARMODE; **overrides `-g`**. A `-H` header declares
  `codec=3` (transparent). For clear-channel/ISDN data bridging — **not** for the
  FSK modem (`modem_fsk` would reject it). Default off.
- `-D` dial-out: place one outbound call to `target` instead of registering,
  bridge it, then exit when it ends or fails. `target` may be a full `sip:` URI,
  `user@host`, or a bare user/number (uses the `-u` registrar domain). Handles
  401/407 digest auth via `-u`/`-p`. The backend TCP socket (`-c`) is opened
  only once the far end signals media (183 Session Progress / early media, or
  the 200 OK answer) — not while the call is merely ringing; if that connect
  fails it aborts the call cleanly (CANCEL before answer, BYE after). Normal
  register/answer mode is unchanged when `-D` is absent.
- `-L` passive mode: **never register upstream**. Just listen on the SIP port,
  answer any inbound REGISTER with `200 OK` (accept-all — no password check and
  no binding store), and answer inbound INVITEs exactly as in normal mode,
  bridging their audio to the TCP backend. `-u` still supplies our identity (the
  `Contact` in our responses) and is used to pick the local IP; `-p` is unused.
  Useful as a lab/test endpoint behind a SIP proxy or test rig that registers
  against us and then sends calls, without needing credentials for — or a route
  to — an upstream registrar. Mutually exclusive with `-D`.

Dial-out example (call extension 30 and bridge it to a modem on :9000):

```sh
sip_interface -u sip:bridge@registrar.example -p secret -c 127.0.0.1:9000 -D 30
```

Passive example (answer calls into a modem on :9000 without registering):

```sh
sip_interface -u sip:bridge@127.0.0.1 -c 127.0.0.1:9000 -P 5060 -L
```

### modem_fsk

```
modem_fsk -l audio_port -d data_host:data_port
          [-M v21|v23|bell103] [-o] [-f FRAMING] [-A amp]
          [-w] [-W secs] [-C ms] [-B] [-m N] [-b BANNER] [-H]
```

- `-l` TCP port to accept audio on (s16le, 8 kHz) — `sip_interface` dials this
- `-d` TCP host:port of the data service
- `-M` `v21` (300 bps full-duplex), `bell103` (300 bps full-duplex, US), or `v23` (1200/75 bps asymmetric). Default `v23`
- `-o` originate side of a symmetric standard (`v21`/`bell103`; default answer)
- `-f` framing: `8N1`, `7E1`, `7N1`, `7O1`, … or `sync` (raw 8-bit stream). Default `8N1`
- `-A` TX amplitude (1..32767, default 16384)
- `-w` / `-W secs` wait for carrier before dialing the data peer (+ optional delay)
- `-C ms` mark-carrier lead-in before the first TX byte (default 300)
- `-B` transmit break (continuous space) after carrier until first RX byte (V.23)
- `-b` banner modulated to the caller before dialing (`\n \r \t \0 \\ \xHH` escapes)
- `-m` max concurrent sessions (0 = unlimited)
- `-H` expect a [stream header](#stream-header--h) (`sip_interface -H`) before
  the PCM; consume it (logging the INVITE) before demodulating

### modem_v22bis

A full-duplex **V.22bis** modem: 2400 bps (16-QAM) with 1200 bps (QPSK)
fallback, 600 baud, band-split with a 1200 Hz (calling) / 2400 Hz (answering)
carrier. Same TCP bridge shape as `modem_fsk` (listen for 8 kHz s16le audio,
dial out a byte/data peer).

```
modem_v22bis -l audio_port -d data_host:data_port
             [-o] [-r 1200|2400] [-G none|550|1800] [-A amp] [-m N] [-H]
```

- `-l` TCP port to accept audio on (s16le, 8 kHz) — `sip_interface` dials this
- `-d` TCP host:port of the data service
- `-o` calling (originate) side; default is the answering side
- `-r` maximum bit rate, `1200` or `2400` (default `2400`); the two modems
  negotiate down to 1200 during the handshake if either side won't do 2400
- `-G` guard tone (answering side only): `none`, `550`, or `1800` (default none)
- `-A` TX amplitude (1..32767, default 12000)
- `-m` max concurrent sessions (0 = unlimited)
- `-H` expect a [stream header](#stream-header--h) before the PCM

The modem runs the full ITU startup handshake automatically (carrier/timing/
equalizer acquisition, S1 rate negotiation, training), then streams the data
peer's bytes as a raw LSB-first bit stream (idle = scrambled ones), exactly like
`modem_fsk`'s `sync` framing. The data channel is bit-oriented and unframed, so
the receiver's byte boundaries depend on where bit transport begins (use an
async/start-stop convention at the application layer if you need byte framing).

**Bridging it:** because the carrier and timing must survive the audio path, run
the SIP leg in **CLEARMODE** (`sip_interface -T`, transparent 64 kbit/s — no
companding) so the modem signal is carried unchanged, e.g.:

```sh
modem_v22bis -l 9000 -d 127.0.0.1:9300 &           # answering side, data on :9300
sip_interface -L -u sip:v22@host -c 127.0.0.1:9000 -T   # passive, CLEARMODE audio
```

**Verification status.** Self-interop (two `modem_v22bis` instances) trains and
exchanges data with **0 BER at both 1200 and 2400 bps, both directions**, and is
robust to the 550/1800 Hz guard tones. It **interoperates with the spandsp
reference modem at 1200 bps** (both roles, bit-exact). The end-to-end bridge
(two binaries over a time-aligned audio relay + data peers, `test_bridge.py`)
passes at 0 BER. Known limitations: 2400 bps **rate negotiation against spandsp**
falls back to 1200 (a handshake-timing mismatch — spandsp commits to 2400 faster
than this modem reacts to its S1 burst), and **data over G.711/PCMA companding**
currently carries a high error rate (the handshake trains through A-law, but the
data does not survive it yet) — so bridge over **CLEARMODE** (clean transport),
where BER is 0. G.711 robustness is future hardening. Tests: `test_handshake.c` (self
interop, both rates), `test_channel.c` (G.711 + noise), `test_v22bis_interop.c`
(`make test`, vs spandsp), `test_bridge.py` (end-to-end). Filter taps are
regenerated by `tools/gen_rrc.py`.

### line_sim

A configurable voiceband **line/channel simulator** that sits between two modems
and relays their audio while injecting realistic PSTN/GSTN impairments. The
modems are audio servers (e.g. `modem_v22bis -l PORT`); `line_sim` connects as a
client to both and shuttles 8/16 kHz s16le PCM in 40-sample lockstep blocks,
processing internally at 48 kHz. With no impairment flags it is a transparent
relay (a degrading stand-in for `modem_v22bis/test_bridge.py`'s clean relay).

```
line_sim -A hostA:portA -B hostB:portB [-r 8000|16000] [-s seed]
         [--gain DB] [--freq-tilt DB] [--freq-bump F0,Q,DB] [--freq-taps FILE]
         [--gd-distortion US] [--thd DB] [--snr DB] [--noise-weight flat|cmsg]
         [--freq-offset HZ] [--jitter DEG,RATE] [--alaw] [--slip PROB]
         [--dir both|a2b|b2a]
```

- `-A` / `-B` the two modems' audio ports   `-r` external rate (8000 or 16000)
- `--gain` flat level; `--freq-tilt` / `--freq-bump` / `--freq-taps` amplitude
  (frequency-response) distortion; `--gd-distortion` group-delay (phase)
  distortion in µs; `--thd` non-linear (signal-to-harmonic) distortion;
  `--snr` additive noise; `--freq-offset` carrier shift; `--jitter` phase jitter;
  `--alaw` a G.711 round-trip; `--slip` random clock slip (bounded to ±10 samples)
- Impairments are symmetric by default; `--dir a2b|b2a` scopes the flags that
  follow it to one direction. Defaults = clean passthrough.

Example — a noisy, group-delay-distorted line with clock slip between two modems:

```sh
modem_v22bis -l 9000 -d 127.0.0.1:9100      &   # answering, data peer :9100
modem_v22bis -l 9001 -d 127.0.0.1:9101 -o   &   # calling,   data peer :9101
line_sim -A 127.0.0.1:9000 -B 127.0.0.1:9001 --snr 34 --gd-distortion 500 --slip 1e-4
```

Verified by `make test` (`test_dsp.c`: each impairment's achieved value matches
the request — SNR, THD, group delay, frequency response, resampler ripple,
clock-slip bound, lockstep count) and `test_bridge_sim.py` (two `modem_v22bis`
through the channel: clean is transparent at BER 0 for both 8 kHz and 16 kHz, a
30 dB noise floor rides through, a buried-signal setting kills the link). The
typical and maximum allowable impairment magnitudes from the standards
(7 CFR §1755.405, ITU-T G.712/G.132/G.133/G.225/O.131/V.56bis) are tabulated in
`line_sim/README.md`.

### Stream header (`-H`)

With `-H` on both ends, `sip_interface` prefixes the audio TCP connection with a
small framed header carrying the **received INVITE** before the PCM, so the
consumer learns who called / what was dialed. It is **opt-in on both sides** and
applies to **inbound calls only** (dial-out connections stay raw). Without `-H`
the stream is byte-for-byte raw s16le as before.

Layout — a fixed 12-byte prefix followed by the payload, then the PCM:

```
 0      1      2       3       4       5         6   7        8  9 10 11    12 ...
+------+------+-------+-------+-------+----------+-----------+--------------+--------+
| 'S'  | 'H'  | ver=2 | flags | codec | channels | samplerate|   length     |payload |
|      |      |       |       |  u8   |   u8     |  u16 BE   |   u32 BE     |        |
+------+------+-------+-------+-------+----------+-----------+--------------+--------+
                                                                            (INVITE)
```

The `codec`/`channels`/`samplerate` fields describe the PCM that follows, so the
consumer can validate it:

- `codec`: `0 = s16le` (linear 16-bit PCM, little-endian; used for PCMA and G.722
  calls — both are decoded to PCM). `3 = transparent` (raw octets, e.g. a
  CLEARMODE call's 64 kbit/s byte stream). Reserved: `1 = PCMU`, `2 = PCMA`.
- `channels`: `1 = mono`.
- `samplerate`: Hz, network byte order (e.g. `8000`).
- `flags` bit 0 (`0x01`) = **VOICE_FOLLOWS**: the PCM begins immediately after
  `payload`. Other bits reserved.
- `length`: payload byte count, network byte order.

The bridge always emits `s16le / 8000 Hz / mono` (it decodes the call's G.711 to
linear PCM). `modem_fsk -H` **closes the session** if the header advertises any
other format, since it can only demodulate 8 kHz mono s16le.

### One-shot launcher: `btx_bridge.sh`

For the BTX (V.23) case, `btx_bridge.sh` wires both programs together so you
don't start them by hand:

```sh
./btx_bridge.sh <sip:user@registrar> <password> <btx_host:port>
# e.g.
./btx_bridge.sh sip:btx@registrar.example secret 127.0.0.1:9300
```

It launches `modem_fsk -M v23 -f 8N1 -w` on a local audio port and points
`sip_interface` at it, registers the AoR, and bridges inbound calls to the BTX
centre at `<btx_host:port>`. Ctrl-C tears both down. Override `AUDIO_PORT`
(default 9000), `SIP_PORT` (default 5060), or `BANNER` via the environment.

### Example: answer SIP calls into a V.23 BTX service (manual)

```sh
# 1. modem on :9000, forwarding decoded bytes to a BTX centre on :9300
modem_fsk -M v23 -l 9000 -d 127.0.0.1:9300 -w -b 'BTX\r\n' &

# 2. SIP UA bridging call audio to the modem
sip_interface -u sip:btx@registrar.example -p secret -c 127.0.0.1:9000
```

A caller dialing the registered AoR is answered; their modem audio is demodulated
and piped to the BTX centre, whose responses are modulated back to the caller.
```

## License

This project is released into the **public domain**. You may copy, modify,
publish, use, compile, sell, or distribute it, in source or binary form, for
any purpose, commercial or non-commercial, and by any means.

The software is provided "as is", without warranty of any kind, express or
implied, including but not limited to the warranties of merchantability,
fitness for a particular purpose and noninfringement. In no event shall the
authors be liable for any claim, damages or other liability arising from,
out of or in connection with the software or its use.

For jurisdictions that do not recognize a public-domain dedication, this code
is alternatively available under the terms of [The Unlicense](https://unlicense.org).

