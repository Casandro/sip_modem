#!/usr/bin/env sh
# Regenerate the test voice clips in this directory.
#
# espeak (+ sox) is an AUTHORING-TIME tool only — NOT a runtime dependency of
# sip_interface / modem_fsk. The committed .wav files are the assets; this
# script just documents how they were produced. espeak is a formant synth, so
# the voice is intelligible but robotic — a better engine (espeak-ng, piper, …)
# would be needed for natural speech.
set -e
here=$(dirname "$0")
phrase="This is a test of the SIP modem bridge. One, two, three, four, five."

# Bridge-format clip: 8 kHz mono s16le — matches sip_interface's PCMA TCP side,
# so it can be piped straight into a call's -c peer.
espeak --stdout "$phrase" | sox - -r 8000 -c 1 -b 16 "$here/test_phrase.wav"
echo "wrote $here/test_phrase.wav (8 kHz)"

# Higher-fidelity clip: espeak's native 22.05 kHz, slower/tuned, peak-normalized
# (full audio band — not telephone-limited). For listening, not the bridge.
espeak -v en-us -s 150 -p 55 -g 3 --stdout "$phrase" \
    | sox - -b 16 "$here/test_phrase_hq.wav" gain -n -1
echo "wrote $here/test_phrase_hq.wav (22.05 kHz)"
