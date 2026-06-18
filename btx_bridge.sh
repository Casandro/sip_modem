#!/usr/bin/env bash
#
# btx_bridge.sh — bring up a SIP-to-BTX bridge with one command.
#
# Wires sip_interface -> modem_fsk (V.23, Bildschirmtext parameters) -> a BTX
# data peer. It registers the given AoR, answers inbound calls, demodulates the
# caller's V.23 modem audio, and connects the decoded byte stream to the BTX
# centre at the given TCP target (and modulates the centre's replies back).
#
# Usage:
#   ./btx_bridge.sh <sip:user@registrar> <password> <btx_host:port>
#   e.g.  ./btx_bridge.sh sip:btx@example.com s3cret 127.0.0.1:9300
#
# Optional environment overrides:
#   AUDIO_PORT  local TCP port bridging sip_interface <-> modem_fsk (default 9000)
#   SIP_PORT    local SIP/UDP port for sip_interface (default 5060)
#   BANNER      bytes modulated to the caller before dialing the centre,
#               e.g. BANNER='BTX\r\n' (supports \n \r \t \0 \\ \xHH escapes)

set -euo pipefail

usage() {
    echo "Usage: $0 <sip:user@registrar> <password> <btx_host:port>" >&2
    echo "  e.g. $0 sip:btx@example.com s3cret 127.0.0.1:9300" >&2
    exit 1
}

[ $# -eq 3 ] || usage
AOR=$1
PASSWORD=$2
TARGET=$3

# Target must look like host:port.
case "$TARGET" in
    *:*[0-9]) ;;
    *) echo "error: target must be host:port (got '$TARGET')" >&2; exit 1 ;;
esac

# BTX = V.23: 1200 bps forward (centre->terminal) / 75 bps back channel, 8N1.
AUDIO_PORT=${AUDIO_PORT:-9000}
SIP_PORT=${SIP_PORT:-5060}

# Locate the binaries relative to this script.
HERE=$(cd "$(dirname "$0")" && pwd)
MODEM="$HERE/modem_fsk/modem_fsk"
SIP="$HERE/sip_interface/sip_interface"
for bin in "$MODEM" "$SIP"; do
    if [ ! -x "$bin" ]; then
        echo "error: $bin not built — run 'make' first." >&2
        exit 1
    fi
done

MODEM_PID=""
SIP_PID=""
cleanup() {
    [ -n "$SIP_PID" ]   && kill "$SIP_PID"   2>/dev/null || true
    [ -n "$MODEM_PID" ] && kill "$MODEM_PID" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# V.23 BTX modem: answer side (1200 fwd / 75 back), 8N1, and wait for the
# terminal's 75 bps carrier before dialing the BTX centre.
modem_cmd=("$MODEM" -M v23 -f 8N1 -l "$AUDIO_PORT" -d "$TARGET" -w)
[ -n "${BANNER:-}" ] && modem_cmd+=(-b "$BANNER")

echo "modem_fsk (V.23 BTX) : listening :$AUDIO_PORT -> data peer $TARGET"
"${modem_cmd[@]}" &
MODEM_PID=$!

# Give the modem a moment to bind its listen port, then confirm it's alive.
sleep 0.5
if ! kill -0 "$MODEM_PID" 2>/dev/null; then
    echo "error: modem_fsk failed to start (is :$AUDIO_PORT already in use?)" >&2
    exit 1
fi

echo "sip_interface        : $AOR on SIP :$SIP_PORT -> 127.0.0.1:$AUDIO_PORT"
"$SIP" -u "$AOR" -p "$PASSWORD" -P "$SIP_PORT" -c "127.0.0.1:$AUDIO_PORT" &
SIP_PID=$!

echo "Bridge up (modem pid $MODEM_PID, sip pid $SIP_PID). Ctrl-C to stop."
wait "$SIP_PID"
