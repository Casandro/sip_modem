#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "channel.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <getopt.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>

/* ── line_sim ──────────────────────────────────────────────────────────────
 * A configurable voiceband line/channel simulator placed between two modems.
 * The modems are TCP audio servers (e.g. modem_v22bis -l PORT); line_sim
 * connects as a client to both, then relays 8/16 kHz s16le PCM between them in
 * fixed 40-sample (5 ms) lockstep blocks while injecting the impairments a
 * real PSTN/GSTN connection adds. Internally it runs at 48 kHz. Impairments
 * are symmetric by default; a --dir switch scopes flags to one direction.
 * See README.md for the standards behind the typical/maximum values. */

#define BLK_SAMPLES 40
#define BLK_BYTES   (BLK_SAMPLES * 2)

typedef enum { DIR_BOTH, DIR_A2B, DIR_B2A } scope_t;

/* ── blocking socket I/O ──────────────────────────────────────────────── */

static int tcp_connect(const char *host, int port) {
    char portstr[16];
    snprintf(portstr, sizeof(portstr), "%d", port);
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res) return -1;

    int fd = -1;
    for (struct addrinfo *ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) continue;
        if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) break;
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);
    if (fd >= 0) {
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    }
    return fd;
}

static int read_exact(int fd, void *buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = read(fd, (char *)buf + got, n - got);
        if (r == 0) return -1;
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        got += (size_t)r;
    }
    return 0;
}

static int write_exact(int fd, const void *buf, size_t n) {
    size_t put = 0;
    while (put < n) {
        ssize_t w = write(fd, (const char *)buf + put, n - put);
        if (w <= 0) { if (w < 0 && errno == EINTR) continue; return -1; }
        put += (size_t)w;
    }
    return 0;
}

/* ── CLI ──────────────────────────────────────────────────────────────── */

enum {
    OPT_GAIN = 256, OPT_TILT, OPT_BUMP, OPT_FIR, OPT_GD, OPT_THD,
    OPT_SNR, OPT_NWEIGHT, OPT_OFFSET, OPT_JITTER, OPT_ALAW, OPT_SLIP, OPT_DIR
};

static const struct option long_opts[] = {
    {"gain",         required_argument, 0, OPT_GAIN},
    {"freq-tilt",    required_argument, 0, OPT_TILT},
    {"freq-bump",    required_argument, 0, OPT_BUMP},
    {"freq-taps",    required_argument, 0, OPT_FIR},
    {"gd-distortion",required_argument, 0, OPT_GD},
    {"thd",          required_argument, 0, OPT_THD},
    {"snr",          required_argument, 0, OPT_SNR},
    {"noise-weight", required_argument, 0, OPT_NWEIGHT},
    {"freq-offset",  required_argument, 0, OPT_OFFSET},
    {"jitter",       required_argument, 0, OPT_JITTER},
    {"alaw",         no_argument,       0, OPT_ALAW},
    {"slip",         required_argument, 0, OPT_SLIP},
    {"dir",          required_argument, 0, OPT_DIR},
    {0, 0, 0, 0}
};

static void usage(const char *argv0) {
    fprintf(stderr,
"Usage: %s -A hostA:portA -B hostB:portB [-r 8000|16000] [-s seed] [options]\n"
"\n"
"Relays s16le PCM between two modem audio servers, injecting line impairments.\n"
"Defaults = clean passthrough. Impairments are symmetric unless preceded by\n"
"--dir; a --dir applies to all impairment flags that follow it.\n"
"\n"
"Connection / format:\n"
"  -A host:port   modem A audio endpoint (required)\n"
"  -B host:port   modem B audio endpoint (required)\n"
"  -r 8000|16000  external sample rate (default 8000)\n"
"  -s seed        PRNG seed for noise/slip (default fixed, reproducible)\n"
"\n"
"Impairments:\n"
"  --gain DB              flat gain/attenuation (default 0)\n"
"  --freq-tilt DB         high-freq tilt = gain@3400 - gain@300 (default 0)\n"
"  --freq-bump F0,Q,DB    peaking section (repeatable, up to %d)\n"
"  --freq-taps FILE       arbitrary FIR mask (whitespace floats, @48 kHz)\n"
"  --gd-distortion US     peak group-delay (phase) distortion, microseconds\n"
"  --thd DB               signal-to-harmonic-distortion ratio (smaller = worse)\n"
"  --snr DB               signal-to-noise ratio (additive)\n"
"  --noise-weight W       flat | cmsg  (C-message-weighted noise; default flat)\n"
"  --freq-offset HZ       carrier frequency shift\n"
"  --jitter DEG,RATE      phase jitter: peak degrees at RATE Hz\n"
"  --alaw                 insert a G.711 A-law (PCMA) round-trip\n"
"  --slip PROB            per-output-sample clock-slip probability (bound +-%d)\n"
"  --dir both|a2b|b2a     scope subsequent impairment flags (default both)\n",
        argv0, LS_MAX_BUMPS, SLIP_MAX);
    exit(1);
}

static int parse_hostport(const char *s, char *host, size_t hostsz, int *port) {
    const char *col = strrchr(s, ':');
    if (!col) return -1;
    size_t hlen = (size_t)(col - s);
    if (hlen == 0 || hlen >= hostsz) return -1;
    memcpy(host, s, hlen);
    host[hlen] = '\0';
    *port = atoi(col + 1);
    return (*port > 0 && *port <= 65535) ? 0 : -1;
}

int main(int argc, char **argv) {
    signal(SIGPIPE, SIG_IGN);

    char hostA[256] = "", hostB[256] = "";
    int  portA = 0, portB = 0;
    int  ext_rate = 8000;
    uint32_t seed = 0;

    chan_cfg_t a2b, b2a;
    memset(&a2b, 0, sizeof(a2b));
    memset(&b2a, 0, sizeof(b2a));
    scope_t dir = DIR_BOTH;

#define WANT_A2B (dir == DIR_BOTH || dir == DIR_A2B)
#define WANT_B2A (dir == DIR_BOTH || dir == DIR_B2A)
#define SCOPE(field, val) do { \
        if (WANT_A2B) a2b.field = (val); \
        if (WANT_B2A) b2a.field = (val); } while (0)

    int opt;
    while ((opt = getopt_long(argc, argv, "A:B:r:s:h", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'A':
            if (parse_hostport(optarg, hostA, sizeof(hostA), &portA)) usage(argv[0]);
            break;
        case 'B':
            if (parse_hostport(optarg, hostB, sizeof(hostB), &portB)) usage(argv[0]);
            break;
        case 'r':
            ext_rate = atoi(optarg);
            if (ext_rate != 8000 && ext_rate != 16000) {
                fprintf(stderr, "bad -r value (want 8000 or 16000)\n");
                return 1;
            }
            break;
        case 's':
            seed = (uint32_t)strtoul(optarg, NULL, 0);
            break;
        case OPT_GAIN:  SCOPE(gain_db, atof(optarg)); break;
        case OPT_TILT:  SCOPE(freq_tilt_db, atof(optarg)); break;
        case OPT_BUMP: {
            double f0, q, db;
            if (sscanf(optarg, "%lf,%lf,%lf", &f0, &q, &db) != 3) {
                fprintf(stderr, "bad --freq-bump (want F0,Q,DB)\n");
                return 1;
            }
            if (WANT_A2B && a2b.nbumps < LS_MAX_BUMPS) {
                a2b.bump[a2b.nbumps].f0 = f0; a2b.bump[a2b.nbumps].q = q;
                a2b.bump[a2b.nbumps].gain_db = db; a2b.nbumps++;
            }
            if (WANT_B2A && b2a.nbumps < LS_MAX_BUMPS) {
                b2a.bump[b2a.nbumps].f0 = f0; b2a.bump[b2a.nbumps].q = q;
                b2a.bump[b2a.nbumps].gain_db = db; b2a.nbumps++;
            }
            break;
        }
        case OPT_FIR:   SCOPE(fir_path, optarg); break;
        case OPT_GD:    SCOPE(gd_us, atof(optarg)); break;
        case OPT_THD:
            if (WANT_A2B) { a2b.has_thd = 1; a2b.thd_db = atof(optarg); }
            if (WANT_B2A) { b2a.has_thd = 1; b2a.thd_db = atof(optarg); }
            break;
        case OPT_SNR:
            if (WANT_A2B) { a2b.has_snr = 1; a2b.snr_db = atof(optarg); }
            if (WANT_B2A) { b2a.has_snr = 1; b2a.snr_db = atof(optarg); }
            break;
        case OPT_NWEIGHT: {
            int cmsg;
            if (strcmp(optarg, "flat") == 0) cmsg = 0;
            else if (strcmp(optarg, "cmsg") == 0) cmsg = 1;
            else { fprintf(stderr, "bad --noise-weight (want flat or cmsg)\n"); return 1; }
            SCOPE(noise_cmsg, cmsg);
            break;
        }
        case OPT_OFFSET: SCOPE(offset_hz, atof(optarg)); break;
        case OPT_JITTER: {
            double deg, rate;
            if (sscanf(optarg, "%lf,%lf", &deg, &rate) != 2) {
                fprintf(stderr, "bad --jitter (want DEG,RATE)\n");
                return 1;
            }
            if (WANT_A2B) { a2b.jitter_deg = deg; a2b.jitter_rate_hz = rate; }
            if (WANT_B2A) { b2a.jitter_deg = deg; b2a.jitter_rate_hz = rate; }
            break;
        }
        case OPT_ALAW:  SCOPE(alaw, 1); break;
        case OPT_SLIP:  SCOPE(slip_prob, atof(optarg)); break;
        case OPT_DIR:
            if (strcmp(optarg, "both") == 0) dir = DIR_BOTH;
            else if (strcmp(optarg, "a2b") == 0) dir = DIR_A2B;
            else if (strcmp(optarg, "b2a") == 0) dir = DIR_B2A;
            else { fprintf(stderr, "bad --dir (want both, a2b, or b2a)\n"); return 1; }
            break;
        case 'h':
        default:
            usage(argv[0]);
        }
    }
    if (!hostA[0] || !portA || !hostB[0] || !portB) usage(argv[0]);

    chan_t ch_a2b, ch_b2a;
    if (chan_init(&ch_a2b, &a2b, ext_rate, seed ? seed : 0xA2B0u) != 0) {
        fprintf(stderr, "channel A->B init failed (bad rate or FIR file?)\n");
        return 1;
    }
    if (chan_init(&ch_b2a, &b2a, ext_rate, (seed ? seed : 0xB2A0u) ^ 0xdeadu) != 0) {
        fprintf(stderr, "channel B->A init failed (bad rate or FIR file?)\n");
        return 1;
    }
    chan_reset(&ch_a2b);
    chan_reset(&ch_b2a);

    int A = tcp_connect(hostA, portA);
    if (A < 0) { fprintf(stderr, "connect to %s:%d failed\n", hostA, portA); return 1; }
    int B = tcp_connect(hostB, portB);
    if (B < 0) { fprintf(stderr, "connect to %s:%d failed\n", hostB, portB); close(A); return 1; }

    fprintf(stderr, "line_sim: A=%s:%d B=%s:%d rate=%d Hz, relaying %d-sample blocks\n",
            hostA, portA, hostB, portB, ext_rate, BLK_SAMPLES);

    fifo_t fifo_a, fifo_b;
    fifo_init(&fifo_a);
    fifo_init(&fifo_b);

    int16_t zero[BLK_SAMPLES];
    memset(zero, 0, sizeof(zero));

    /* Prime the modems: they emit only after receiving input. The V.22bis
     * handshake is round-trip-latency sensitive, so add as little buffering as
     * possible. A FIFO prefill is only needed as a cushion when clock slip can
     * drop samples; without slip the resampler is sample-exact (N in -> N out)
     * and no prefill is required. */
    int slip_any = (a2b.slip_prob > 0.0 || b2a.slip_prob > 0.0);
    if (write_exact(A, zero, BLK_BYTES) || write_exact(B, zero, BLK_BYTES)) {
        fprintf(stderr, "line_sim: prime write failed\n");
        close(A); close(B); return 1;
    }
    if (slip_any) {
        chan_process(&ch_a2b, zero, BLK_SAMPLES, &fifo_b);
        chan_process(&ch_b2a, zero, BLK_SAMPLES, &fifo_a);
    }

    int16_t ina[BLK_SAMPLES], inb[BLK_SAMPLES];
    int16_t outa[BLK_SAMPLES], outb[BLK_SAMPLES];

    for (;;) {
        if (read_exact(A, ina, BLK_BYTES)) break;
        if (read_exact(B, inb, BLK_BYTES)) break;

        chan_process(&ch_a2b, ina, BLK_SAMPLES, &fifo_b);
        chan_process(&ch_b2a, inb, BLK_SAMPLES, &fifo_a);

        int gb = fifo_pop(&fifo_b, outb, BLK_SAMPLES);
        for (; gb < BLK_SAMPLES; gb++) outb[gb] = 0;     /* underflow guard */
        int ga = fifo_pop(&fifo_a, outa, BLK_SAMPLES);
        for (; ga < BLK_SAMPLES; ga++) outa[ga] = 0;

        if (write_exact(B, outb, BLK_BYTES)) break;
        if (write_exact(A, outa, BLK_BYTES)) break;
    }

    fprintf(stderr, "line_sim: closing (A->B slip range [%d,%d], B->A [%d,%d])\n",
            ch_a2b.slip_min, ch_a2b.slip_max, ch_b2a.slip_min, ch_b2a.slip_max);
    close(A);
    close(B);
    return 0;
}
