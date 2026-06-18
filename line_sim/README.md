# line_sim — voiceband line / channel simulator

`line_sim` is placed **between two modems** and relays the audio between them
while injecting the impairments a real PSTN/GSTN connection adds. It is the
deliberately-degrading counterpart to `modem_v22bis/test_bridge.py`'s bit-exact
relay: same lockstep transport, but with a configurable channel in the middle.

```
  modem A  ──audio──▶  line_sim  ──audio──▶  modem B
 (-l portA)  ◀──audio──  (impairs)  ◀──audio──  (-l portB)
```

The two modems are TCP **audio servers** (e.g. `modem_v22bis -l PORT`), exchanging
an un-framed stream of signed-16-bit little-endian mono PCM, one sample in → one
sample out. `line_sim` connects as a **client to both** and shuttles the audio in
fixed 40-sample (5 ms) lockstep blocks. Their separate data channels (`-d`) are
where you inject/recover the byte stream and are not `line_sim`'s concern.

Internally `line_sim` runs at **48 kHz** (48000/8000 = 6, 48000/16000 = 3 — exact
integer resampling), so the external rate can be 8 kHz or 16 kHz. Defaults are a
clean passthrough, so with no impairment flags it is a transparent relay.

## Build

```
make            # builds ./line_sim   (from the repo root: make line_sim)
make test       # builds and runs the DSP unit checks (test_dsp)
python3 test_bridge_sim.py    # end-to-end: two modem_v22bis through line_sim
```

## Usage

```
line_sim -A hostA:portA -B hostB:portB [-r 8000|16000] [-s seed] [impairments]
```

Example — insert a noisy, slightly group-delay-distorted line with clock slip
between an answering and a calling `modem_v22bis`:

```
modem_v22bis -l 9000 -d 127.0.0.1:9100        &   # answering, data peer on :9100
modem_v22bis -l 9001 -d 127.0.0.1:9101 -o     &   # calling,   data peer on :9101
line_sim -A 127.0.0.1:9000 -B 127.0.0.1:9001 \
         --snr 34 --gd-distortion 500 --slip 1e-4
```

### Impairments (all optional; default = clean passthrough)

| Flag | Effect |
| --- | --- |
| `--gain DB` | flat gain / attenuation |
| `--freq-tilt DB` | high-frequency tilt: gain@3400 − gain@300 (attenuation distortion) |
| `--freq-bump F0,Q,DB` | a peaking section at F0 Hz (repeatable, up to 4) |
| `--freq-taps FILE` | arbitrary FIR magnitude mask (whitespace floats, designed at 48 kHz) |
| `--gd-distortion US` | peak group-delay (phase) distortion, microseconds (band-edge − mid-band) |
| `--thd DB` | non-linear distortion: signal-to-harmonic-distortion ratio (smaller = worse) |
| `--snr DB` | additive noise, referenced to signal power |
| `--noise-weight flat\|cmsg` | flat or (approximate) C-message-weighted noise |
| `--freq-offset HZ` | carrier frequency shift |
| `--jitter DEG,RATE` | phase jitter: peak degrees at RATE Hz |
| `--alaw` | insert a G.711 A-law (PCMA) round-trip (mimics the SIP bridge codec) |
| `--slip PROB` | per-output-sample clock-slip probability; cumulative offset bounded to ±10 |
| `--dir both\|a2b\|b2a` | scope the impairment flags that follow it (default `both`) |

Impairments are **symmetric** by default. A `--dir a2b` (or `b2a`) switch makes
every impairment flag *after* it apply to that one direction only, until the next
`--dir`. Clock slip is always modelled independently per direction (the two
endpoints have physically independent sample clocks).

### How it works

Per direction, each external sample is upsampled to 48 kHz and run through the
chain — **gain → frequency response → group-delay all-pass → non-linear
distortion → frequency offset + phase jitter** — then downsampled back, given an
optional A-law round-trip and additive noise, clipped, and pushed through the
clock-slip stage into an elastic FIFO the relay drains in fixed blocks. The
resampler uses a long Kaiser-windowed-sinc polyphase low-pass (flat to 3400 Hz,
stopband from the first image at 4600 Hz); the frequency-response stage is RBJ
biquads; the phase-distortion stage is magnitude-flat second-order all-pass
sections; the non-linearity is a `tanh` soft-limiter numerically calibrated to
the requested THD; frequency offset and jitter rotate the analytic signal formed
by a 31-tap Hilbert FIR.

## Impairment standards — typical and maximum allowable values

These are the figures the impairment ranges are modelled on. The hard limits
below are from **7 CFR §1755.405** (US voiceband-data-transmission measurement
limits, the modern codification of the Bell/REA voice-grade-data practice); the
ITU-T recommendations define the measurement methods and international masks.
Use them as a guide for "typical" vs "stressful" settings.

| Impairment | `line_sim` flag | Typical (good line) | Maximum allowable | Source |
| --- | --- | --- | --- | --- |
| Signal-to-noise (C-notched) | `--snr` | 38–45 dB | **≥ 31 dB** (1004 Hz @ −13 dBm0) | 7 CFR §1755.405; ITU-T G.712 |
| Total / quantizing distortion (SNDR) | `--thd` | 35–36 dB | **≥ ~33 dB** central band (1020 Hz, C-message weighted) | ITU-T G.712; methods O.131/O.132/O.133 |
| Intermodulation / harmonic distortion | `--thd` | 45–50 dB | **≥ 40 dB** (2nd & 3rd order, 4-tone) | 7 CFR §1755.405 |
| Attenuation (frequency-response) distortion | `--freq-tilt`, `--freq-bump`, `--freq-taps` | ±1–3 dB over 500–2800 Hz | basic 3002: **−2 / +8 dB** (404–2804 Hz, ref 1004 Hz); C1/C2 tighter | ITU-T G.132; AT&T/Bell 3002 conditioning |
| Envelope (group) delay distortion | `--gd-distortion` | 100–500 µs | **≤ 1500 µs @ 604 Hz, ≤ 1000 µs @ 2804 Hz** | 7 CFR §1755.405; ITU-T G.133 |
| Phase jitter | `--jitter DEG,RATE` | 2–5° pk-pk | **≤ 10.0° pk-pk** (20–300 Hz), ≤ 6.5° pk-pk (4–300 Hz) | 7 CFR §1755.405 |
| Frequency offset / carrier shift | `--freq-offset` | < 1 Hz | **≤ ~5 Hz** end-to-end (≈ 2 Hz per carrier system) | ITU-T G.225 |
| Codec companding | `--alaw` | — | G.711 A-law (PCMA), 8-bit log | ITU-T G.711 |

Notes on the figures:

- **Attenuation-distortion C1/C2 cells.** The exact per-band dB masks for the
  Bell/AT&T C-type conditioning (C1, C2, …) vary slightly between the AT&T
  Pub. 41004 series, the FCC tariffs and the ITU-T G.132 international mask. The
  values above are the widely-reproduced "basic 3002" envelope; treat them as
  representative rather than authoritative, and consult a clean copy of the
  relevant tariff / G.132 for a specific conditioning class.
- **Frequency offset.** ITU-T G.225 bounds the carrier-frequency error
  introduced by FDM carrier equipment; ≈ 5 Hz end-to-end is the long-standing
  worst case the V-series modems are designed to tolerate. Typical modern
  digital connections are well under 1 Hz.
- **Reference test channels.** ITU-T **V.56 bis** ("Network transmission model
  for evaluating modem performance over 2-wire voice grade connections")
  combines attenuation distortion, group delay, noise, frequency offset, phase
  jitter and non-linearity into named test channels for benchmarking modem BER;
  `line_sim` lets you reconstruct such a channel from its component flags.

## Verification

- **`make test`** (`test_dsp.c`, no sockets) drives known stimuli through one
  channel and asserts the *achieved* impairment matches the *requested* value:
  resampler in-band ripple (< 0.6 dB, 8 kHz and 16 kHz), `--freq-tilt` accuracy,
  `--thd` (signal-to-distortion via harmonic analysis), `--snr`, `--gd-distortion`
  (group delay measured by phase finite-difference, with the all-pass magnitude
  confirmed flat), the clock-slip ±10 bound over 10⁷ samples, and the lockstep
  invariant (exactly 40 samples out per 40 in with slip off).
- **`python3 test_bridge_sim.py`** runs two `modem_v22bis` instances through
  `line_sim`: a clean channel must stay transparent (BER 0) at both 8 kHz and
  16 kHz, a 30 dB noise floor still rides through, and a punishing setting
  (signal buried in noise) destroys the link — confirming the impairment path is
  really in circuit. (The numeric fidelity of each impairment is the job of
  `test_dsp`; `modem_v22bis` is a marginal from-scratch receiver, so the gentler
  impairments are exercised there rather than relied on for an end-to-end pass.)

Sources: [7 CFR §1755.405](https://www.law.cornell.edu/cfr/text/7/1755.405) ·
[ITU-T G.712](https://www.itu.int/rec/T-REC-G.712) ·
[ITU-T G.132](https://www.itu.int/rec/T-REC-G.132) ·
[ITU-T G.133](https://www.itu.int/rec/T-REC-G.133) ·
[ITU-T G.225](https://www.itu.int/rec/T-REC-G.225) ·
[ITU-T O.131](https://www.itu.int/rec/T-REC-O.131) ·
[ITU-T V.56bis](https://www.itu.int/rec/T-REC-V.56bis) ·
[ITU-T G.711](https://www.itu.int/rec/T-REC-G.711)
