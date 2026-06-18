#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "fsk.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>

/* ── Global config (set in main, read-only after) ─────────────────── */

typedef struct {
    int            listen_port;
    char           data_host[256];
    int            data_port;
    fsk_mode_t     mode;           /* -M: V.21, V.23, or Bell 103 (default V.23) */
    int            originate;      /* -o: originate side (v21/bell103 only) */
    fsk_framing_t  framing;
    int16_t        amplitude;
    int            wait_carrier;   /* -w: defer data dial-out until carrier */
    int            connect_delay_ms; /* -W: wait this long after CD before dialing (default 1000) */
    int            carrier_leadin_ms; /* -C: mark-carrier lead-in before the first TX byte (default 300) */
    int            break_after_cd; /* -B: send a TX break after CD, until first RX byte */
    int            max_sessions;   /* -m: cap on concurrent sessions; 0 = unlimited */
    uint8_t        banner[1024];   /* -b: bytes modulated to the caller before dialing */
    int            banner_len;     /* decoded banner length; 0 = no banner */
    int            expect_header;  /* -H: consume a stream header before the PCM */
} cfg_t;

static cfg_t g_cfg;

/* Live session count. Bumped in main before pthread_create, dropped in the
 * session thread on exit. Atomic because main and session threads both touch
 * it without holding a lock. */
static volatile int g_session_count;

/* ── Per-session state ────────────────────────────────────────────── */

typedef struct {
    int        audio_sock;          /* accepted, non-blocking */
    int        data_sock;           /* outbound to data_host:data_port, non-blocking */

    fsk_tx_t   tx;
    fsk_rx_t   rx;

    /* Pending bytes from the data peer, waiting to be modulated. */
    uint8_t    tx_q[1280];
    int        tx_q_len;

    /* Demodulated bytes waiting to be written to the data peer. */
    uint8_t    rx_q[1280];
    int        rx_q_len;

    /* Audio I/O staging (raw bytes off the wire; samples are int16). */
    uint8_t    in_stage[2];         /* assembling one int16 across reads */
    int        in_stage_len;
    uint8_t    out_buf[640];        /* accumulated TX bytes ready to flush */
    int        out_buf_len;

    /* Lifecycle of the optional carrier-triggered TX break:
     *   IDLE  -> ACTIVE on first carrier-detect (if -B is set)
     *   ACTIVE -> DONE on first decoded RX byte. Never re-arms. */
    int        break_state;

    /* Post-carrier connect delay (-w / -W). carrier_seen latches on the first
     * carrier-detect; connect_at is the monotonic deadline to dial. */
    int             carrier_seen;
    struct timespec connect_at;

    /* How many banner bytes have been handed to the TX queue so far.
     * The data peer isn't dialed until this reaches g_cfg.banner_len and
     * the queue has fully drained (i.e. the banner is on the wire). */
    int             banner_sent;

    /* Carrier lead-in: number of samples to hold the modulator in idle
     * (mark carrier) before it's allowed to pull the first byte from tx_q.
     * Gives the caller's receiver time to detect carrier and recover bit
     * timing, so the first byte (or banner byte) isn't lost. Counts down on
     * every produced sample — idle-mark output before any data arrives
     * counts toward it, so a quiet start incurs no extra latency. */
    int             tx_leadin;

    /* Stream-header consumption (-H). The peer (sip_interface -H) prefixes the
     * audio with an 8-byte header + INVITE payload before the PCM begins; we
     * read past it before treating bytes as samples. All zero-init via calloc. */
    int        hdr_done;            /* 1 once the header has been fully consumed */
    int        hdr_parsed;          /* 1 once the 12-byte prefix has been parsed */
    int        hdr_reject;          /* 1 = unsupported format; drop the session */
    int        hdr_have;            /* prefix bytes collected so far (0..12) */
    uint8_t    hdr_buf[12];
    int        hdr_payload_remaining;
    char       hdr_line[128];       /* captured INVITE request-line, for logging */
    int        hdr_line_len;
    int        hdr_line_done;
} session_t;
enum { BREAK_IDLE = 0, BREAK_ACTIVE, BREAK_DONE };

/* ── SIGPIPE / random / time helpers ──────────────────────────────── */

static unsigned int rng_seed(void) {
    static unsigned int counter = 0;
    return (unsigned int)time(NULL)
         ^ (unsigned int)getpid()
         ^ (unsigned int)(uintptr_t)pthread_self()
         ^ __sync_fetch_and_add(&counter, 1);
}

/* Monotonic deadline helpers (mirror sip_interface). */
static void ts_add_ms(struct timespec *ts, long ms) {
    ts->tv_nsec += ms * 1000000L;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec++;
    }
}

static long ms_until(const struct timespec *deadline) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (deadline->tv_sec  - now.tv_sec)  * 1000L
         + (deadline->tv_nsec - now.tv_nsec) / 1000000L;
}

/* ── Banner decoding ──────────────────────────────────────────────── */

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Decode C-style escapes in `src` into `dst` (capacity `cap` bytes).
 * Supports \n \r \t \0 \\ and \xHH (1 or 2 hex digits). An unknown escape
 * yields the literal following character. Returns the decoded byte count,
 * or -1 if it would overflow `dst`. */
static int decode_banner(const char *src, uint8_t *dst, int cap) {
    int n = 0;
    for (const char *p = src; *p; ) {
        if (n >= cap) return -1;
        if (*p != '\\') { dst[n++] = (uint8_t)*p++; continue; }
        p++;
        switch (*p) {
            case 'n':  dst[n++] = '\n'; p++; break;
            case 'r':  dst[n++] = '\r'; p++; break;
            case 't':  dst[n++] = '\t'; p++; break;
            case '0':  dst[n++] = '\0'; p++; break;
            case '\\': dst[n++] = '\\'; p++; break;
            case 'x': {
                p++;
                int hi = hexval(*p);
                if (hi < 0) return -1;          /* \x needs at least one digit */
                p++;
                int lo = hexval(*p);
                if (lo < 0) { dst[n++] = (uint8_t)hi; break; }
                p++;
                dst[n++] = (uint8_t)((hi << 4) | lo);
                break;
            }
            case '\0': dst[n++] = '\\'; break;  /* trailing backslash: literal */
            default:   dst[n++] = (uint8_t)*p++; break;
        }
    }
    return n;
}

/* ── TCP helpers (mirror sip_interface/call.c) ────────────────────── */

static int tcp_connect_data(int *out_sock) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;

    struct hostent *he = gethostbyname(g_cfg.data_host);
    if (!he) { close(s); return -1; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons((uint16_t)g_cfg.data_port);
    memcpy(&addr.sin_addr, he->h_addr_list[0], (size_t)he->h_length);

    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    int r = connect(s, (struct sockaddr *)&addr, sizeof(addr));
    if (r < 0 && errno != EINPROGRESS) { close(s); return -1; }
    if (r < 0) {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(s, &wfds);
        struct timeval tv = {5, 0};
        r = select(s + 1, NULL, &wfds, NULL, &tv);
        if (r <= 0) { close(s); return -1; }
        int err = 0;
        socklen_t sl = sizeof(err);
        if (getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &sl) < 0 || err != 0) {
            close(s); return -1;
        }
    }

    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    *out_sock = s;
    return 0;
}

/* Non-blocking write; drop on EAGAIN, close on hard error. */
static void tcp_write_nb(int *sock, const void *buf, size_t len) {
    if (*sock < 0) return;
    const uint8_t *p = (const uint8_t *)buf;
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t w = write(*sock, p, remaining);
        if (w > 0) {
            p += w;
            remaining -= (size_t)w;
        } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return;
        } else {
            close(*sock);
            *sock = -1;
            return;
        }
    }
}

/* ── Session ──────────────────────────────────────────────────────── */

/* Consume the optional stream header (-H) from the front of `buf` (n bytes):
 * a 12-byte prefix ('S' 'H' ver flags codec channels samplerate(u16 BE)
 * length(u32 BE)) followed by `length` payload bytes (the INVITE). Returns the
 * index in buf where the PCM begins; sets s->hdr_done once the whole header has
 * been consumed (or s->hdr_reject if the advertised format is unsupported).
 * Header and PCM may share a read, so this is index-based and resumes across
 * reads. modem_fsk only handles 8 kHz / mono / s16le. */
static int consume_header(session_t *s, const uint8_t *buf, int n) {
    int i = 0;

    /* Collect the fixed 12-byte prefix. */
    while (s->hdr_have < 12 && i < n)
        s->hdr_buf[s->hdr_have++] = buf[i++];
    if (s->hdr_have < 12) return i;           /* need more bytes next read */

    if (!s->hdr_parsed) {
        if (s->hdr_buf[0] != 'S' || s->hdr_buf[1] != 'H') {
            fprintf(stderr, "session %p: bad stream-header magic — treating "
                    "stream as raw audio\n", (void *)s);
            s->hdr_done = 1;
            return i;
        }
        unsigned ver      = s->hdr_buf[2];
        unsigned flags    = s->hdr_buf[3];
        unsigned codec    = s->hdr_buf[4];
        unsigned channels = s->hdr_buf[5];
        uint16_t rate_be;  memcpy(&rate_be, s->hdr_buf + 6, sizeof(rate_be));
        unsigned rate = ntohs(rate_be);
        uint32_t len_be;   memcpy(&len_be,  s->hdr_buf + 8, sizeof(len_be));
        s->hdr_payload_remaining = (int)ntohl(len_be);
        s->hdr_parsed = 1;
        fprintf(stderr, "session %p: stream header v%u: codec=%u %uHz %uch, "
                "%d-byte payload\n", (void *)s, ver, codec, rate, channels,
                s->hdr_payload_remaining);

        if (ver != 2) {
            fprintf(stderr, "session %p: unsupported stream-header version %u "
                    "— closing\n", (void *)s, ver);
            s->hdr_reject = 1;
            return i;
        }
        /* modem_fsk can only demodulate 8 kHz mono s16le. */
        if (codec != 0 || rate != 8000 || channels != 1) {
            fprintf(stderr, "session %p: unsupported stream format "
                    "(codec=%u %uHz %uch; need s16le 8000Hz 1ch) — closing\n",
                    (void *)s, codec, rate, channels);
            s->hdr_reject = 1;
            return i;
        }
        if (!(flags & 0x01))                  /* VOICE_FOLLOWS */
            fprintf(stderr, "session %p: header VOICE_FOLLOWS bit clear — "
                    "proceeding anyway\n", (void *)s);
    }

    /* Consume the payload, capturing just the first (request) line for the log. */
    while (s->hdr_payload_remaining > 0 && i < n) {
        uint8_t b = buf[i++];
        s->hdr_payload_remaining--;
        if (s->hdr_line_done) continue;
        if (b == '\r' || b == '\n') {
            if (s->hdr_line_len > 0) s->hdr_line_done = 1;
        } else if (s->hdr_line_len < (int)sizeof(s->hdr_line) - 1) {
            s->hdr_line[s->hdr_line_len++] = (char)b;
        }
    }
    if (s->hdr_payload_remaining == 0) {
        s->hdr_line[s->hdr_line_len] = '\0';
        fprintf(stderr, "session %p: INVITE: %s\n", (void *)s, s->hdr_line);
        s->hdr_done = 1;
    }
    return i;
}

/* One audio sample in → one audio sample out. Handles RX framing,
 * break-release on first byte, and TX modulation. */
static void process_sample(session_t *s, int16_t samp) {
    int rb = fsk_rx_sample(&s->rx, samp);
    if (rb >= 0 && rb <= 255) {
        if (s->break_state == BREAK_ACTIVE) {
            fsk_tx_set_break(&s->tx, 0);
            s->break_state = BREAK_DONE;
            fprintf(stderr, "session %p: TX break released (first RX byte)\n",
                    (void *)s);
        }
        if (s->rx_q_len < (int)sizeof(s->rx_q))
            s->rx_q[s->rx_q_len++] = (uint8_t)rb;
    }
    /* During the carrier lead-in, keep the modulator idle (mark carrier)
     * by handing it an empty queue, so the far-end receiver can lock on
     * before the first start bit. We pass a local zero length rather than
     * s->tx_q_len so no queued byte is consumed. */
    int  zero  = 0;
    int *txlen = &s->tx_q_len;
    if (s->tx_leadin > 0) {
        s->tx_leadin--;
        txlen = &zero;
    }
    int16_t out = fsk_tx_sample(&s->tx, s->tx_q, txlen);
    if (s->out_buf_len + 2 <= (int)sizeof(s->out_buf)) {
        memcpy(s->out_buf + s->out_buf_len, &out, 2);
        s->out_buf_len += 2;
    }
}

static void session_io_loop(session_t *s) {
    /* Banner and break are framed-channel features; in raw sync streaming
     * they have no meaning, so they are disabled. */
    int banner_enabled = g_cfg.banner_len > 0 && !g_cfg.framing.sync;
    int break_enabled  = g_cfg.break_after_cd && !g_cfg.framing.sync;

    while (s->audio_sock >= 0) {
        fd_set rfds, wfds;
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_SET(s->audio_sock, &rfds);

        /* Only poll data_sock readable if tx_q has room (back-pressure
         * the data peer via TCP rwin when our queue is full). */
        int tx_room = (int)sizeof(s->tx_q) - s->tx_q_len;
        if (s->data_sock >= 0 && tx_room > 0)
            FD_SET(s->data_sock, &rfds);
        /* Watch writability only when we have bytes to flush. */
        if (s->data_sock >= 0 && s->rx_q_len > 0)
            FD_SET(s->data_sock, &wfds);

        int maxfd = s->audio_sock;
        if (s->data_sock > maxfd) maxfd = s->data_sock;

        struct timeval tv = {1, 0};                 /* 1 s safety timeout */
        /* While waiting out the post-carrier delay, shorten the timeout so the
         * deferred dial-out fires promptly even if no audio arrives meanwhile. */
        if (g_cfg.wait_carrier && s->data_sock < 0 && s->carrier_seen) {
            long rem = ms_until(&s->connect_at);
            if (rem < 0) rem = 0;
            if (rem < 1000) { tv.tv_sec = rem / 1000; tv.tv_usec = (rem % 1000) * 1000; }
        }
        int sel = select(maxfd + 1, &rfds, &wfds, NULL, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;
            break;
        }

        /* AUDIO IN -> demodulate -> rx_q; per sample emit one TX sample. */
        if (FD_ISSET(s->audio_sock, &rfds)) {
            uint8_t buf[640];
            ssize_t n = read(s->audio_sock, buf, sizeof(buf));
            if (n <= 0) {
                if (n == 0) break;
                if (errno != EAGAIN && errno != EWOULDBLOCK) break;
            } else {
                int i = 0;
                /* Strip the optional stream header before the PCM begins. */
                if (g_cfg.expect_header && !s->hdr_done) {
                    i = consume_header(s, buf, (int)n);
                    if (s->hdr_reject) break;   /* unsupported format → end session */
                }
                while (i < n) {
                    int16_t samp;
                    if (s->in_stage_len == 1) {
                        s->in_stage[1] = buf[i++];
                        memcpy(&samp, s->in_stage, 2);
                        s->in_stage_len = 0;
                    } else if (i + 1 < n) {
                        memcpy(&samp, buf + i, 2);
                        i += 2;
                    } else {
                        /* Odd byte at the tail — carry to the next read. */
                        s->in_stage[0]   = buf[i++];
                        s->in_stage_len  = 1;
                        continue;
                    }
                    process_sample(s, samp);
                }
                /* Flush whenever we've got at least one frame worth. */
                if (s->out_buf_len > 0) {
                    tcp_write_nb(&s->audio_sock, s->out_buf, (size_t)s->out_buf_len);
                    s->out_buf_len = 0;
                }
            }
        }

        /* Things we do on the first carrier-detect. CD has built-in
         * hysteresis (~100 ms to assert), so these fire at most once and
         * only when there's genuinely a carrier on the line. */
        if (fsk_rx_carrier(&s->rx)) {
            if (g_cfg.wait_carrier && s->data_sock < 0 && !s->carrier_seen) {
                /* Latch the carrier and arm the post-carrier connect delay.
                 * The actual dial happens below once the deadline elapses. */
                s->carrier_seen = 1;
                clock_gettime(CLOCK_MONOTONIC, &s->connect_at);
                ts_add_ms(&s->connect_at, g_cfg.connect_delay_ms);
                fprintf(stderr,
                        "session %p: carrier detected, dialing data peer in %d ms\n",
                        (void *)s, g_cfg.connect_delay_ms);
            }
            if (break_enabled && s->break_state == BREAK_IDLE) {
                fsk_tx_set_break(&s->tx, 1);
                s->break_state = BREAK_ACTIVE;
                fprintf(stderr, "session %p: TX break started after carrier\n",
                        (void *)s);
            }
        }

        /* The link is "ready" immediately, or — with -w — once we've heard
         * the caller's carrier. The banner and the data dial-out both gate
         * on this. */
        int link_ready = !g_cfg.wait_carrier || s->carrier_seen;

        /* Banner: feed it into the TX queue so it's modulated out to the
         * caller. We don't dial the data peer until every banner byte has
         * been queued *and* the queue has drained (banner is on the wire). */
        if (banner_enabled && link_ready
            && s->banner_sent < g_cfg.banner_len) {
            int room = (int)sizeof(s->tx_q) - s->tx_q_len;
            int n = g_cfg.banner_len - s->banner_sent;
            if (n > room) n = room;
            if (n > 0) {
                memcpy(s->tx_q + s->tx_q_len, g_cfg.banner + s->banner_sent,
                       (size_t)n);
                s->tx_q_len    += n;
                s->banner_sent += n;
            }
        }
        int banner_done = !banner_enabled
                       || (s->banner_sent >= g_cfg.banner_len && s->tx_q_len == 0);

        /* Dial the data peer once due: link ready, banner fully sent, and —
         * with -w — the post-carrier delay elapsed. */
        if (s->data_sock < 0 && link_ready && banner_done
            && (!g_cfg.wait_carrier || ms_until(&s->connect_at) <= 0)) {
            if (tcp_connect_data(&s->data_sock) < 0) {
                fprintf(stderr, "session %p: data dial to %s:%d failed\n",
                        (void *)s, g_cfg.data_host, g_cfg.data_port);
                break;
            }
            fprintf(stderr, "session %p: data peer connected\n", (void *)s);
        }

        /* DATA IN -> tx_q (to be modulated next). */
        if (s->data_sock >= 0 && tx_room > 0 && FD_ISSET(s->data_sock, &rfds)) {
            ssize_t n = read(s->data_sock,
                             s->tx_q + s->tx_q_len,
                             (size_t)tx_room);
            if (n > 0) {
                s->tx_q_len += (int)n;
            } else if (n == 0) {
                close(s->data_sock); s->data_sock = -1;
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                close(s->data_sock); s->data_sock = -1;
            }
        }

        /* DATA OUT <- rx_q (demodulated bytes). */
        if (s->data_sock >= 0 && s->rx_q_len > 0 && FD_ISSET(s->data_sock, &wfds)) {
            ssize_t w = write(s->data_sock, s->rx_q, (size_t)s->rx_q_len);
            if (w > 0) {
                if (w < s->rx_q_len)
                    memmove(s->rx_q, s->rx_q + w, (size_t)(s->rx_q_len - w));
                s->rx_q_len -= (int)w;
            } else if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                close(s->data_sock); s->data_sock = -1;
            }
        }
    }
}

static void *session_thread(void *arg) {
    session_t *s = (session_t *)arg;

    /* Make audio non-blocking, disable Nagle. */
    int flags = fcntl(s->audio_sock, F_GETFL, 0);
    fcntl(s->audio_sock, F_SETFL, flags | O_NONBLOCK);
    int one = 1;
    setsockopt(s->audio_sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    /* Dial the data peer. With -w, defer until we hear a carrier on the
     * inbound audio. With -b, defer until the banner has been modulated
     * out. Both are handled inside the io loop. */
    int banner_enabled = g_cfg.banner_len > 0 && !g_cfg.framing.sync;
    s->data_sock = -1;
    if (!g_cfg.wait_carrier && !banner_enabled) {
        if (tcp_connect_data(&s->data_sock) < 0) {
            fprintf(stderr, "session %p: data dial-out to %s:%d failed\n",
                    (void *)s, g_cfg.data_host, g_cfg.data_port);
            close(s->audio_sock);
            free(s);
            __sync_sub_and_fetch(&g_session_count, 1);
            return NULL;
        }
    }

    /* Seed RNG (here only to keep parity with sip_interface). */
    (void)rng_seed();

    fsk_profile_t prof;
    fsk_profile_init(&prof, g_cfg.mode, g_cfg.originate);
    fsk_tx_init(&s->tx, &prof, &g_cfg.framing, g_cfg.amplitude);
    fsk_rx_init(&s->rx, &prof, &g_cfg.framing);

    /* Arm the mark-carrier lead-in (8 kHz sampling => 8 samples/ms). */
    s->tx_leadin = g_cfg.carrier_leadin_ms * 8;

    session_io_loop(s);

    if (s->audio_sock >= 0) close(s->audio_sock);
    if (s->data_sock  >= 0) close(s->data_sock);
    free(s);
    __sync_sub_and_fetch(&g_session_count, 1);
    return NULL;
}

/* ── main: listen/accept loop ─────────────────────────────────────── */

static void usage(const char *argv0) {
    fprintf(stderr,
        "Usage: %s -l audio_port -d data_host:data_port [-M v21|v23|bell103] [-o] [-f FRAMING]\n"
        "          [-A amp] [-w] [-W secs] [-C ms] [-B] [-m N] [-b BANNER]\n"
        "  -l  TCP port to listen on for incoming audio (s16le, 8 kHz)\n"
        "  -d  TCP host:port to dial out for the byte/data channel\n"
        "  -M  Modem standard: v21 (300 bps full-duplex), bell103 (300 bps\n"
        "      full-duplex, US), or v23 (1200/75 bps asymmetric). Default v23\n"
        "  -o  Originate side of a symmetric standard (v21/bell103).\n"
        "      v21 originate: TX 980/1180, RX 1650/1850.\n"
        "      bell103 originate: TX 1270/1070, RX 2225/2025 (mark/space).\n"
        "      Default: answer side (the swapped pair)\n"
        "  -f  Framing: 8N1, 7E1, 7N1, 7O1, …, or 'sync' (default 8N1)\n"
        "      sync = LSB-first raw 8-bit streaming, no start/stop/parity\n"
        "      (primarily a V.21 feature; disables -b/-B)\n"
        "  -A  TX amplitude (int16 magnitude, default 16384)\n"
        "  -w  Wait for carrier before dialing data peer\n"
        "  -W  Seconds to wait after carrier-detect before dialing (default 1; implies -w; 0 = immediate)\n"
        "  -C  Mark-carrier lead-in in ms before the first TX byte (default 300; 0 = none).\n"
        "      Lets the caller's receiver lock onto the carrier so the first byte isn't lost\n"
        "  -B  After carrier-detect, transmit continuous space (break) until the first\n"
        "      RX byte arrives. Intended for V.23's forward channel\n"
        "  -m  Max concurrent sessions (default 0 = unlimited)\n"
        "  -b  Banner modulated to the caller before dialing the data peer.\n"
        "      Supports \\n \\r \\t \\0 \\\\ and \\xHH escapes (max 1024 bytes)\n"
        "  -H  Expect a stream header (sip_interface -H) before the PCM: an\n"
        "      8-byte prefix + INVITE payload, consumed before demodulating\n",
        argv0);
    exit(1);
}

int main(int argc, char **argv) {
    signal(SIGPIPE, SIG_IGN);

    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.mode = FSK_V23;            /* default standard */
    g_cfg.amplitude = 16384;
    g_cfg.connect_delay_ms = 1000;   /* default 1 s wait after carrier-detect */
    g_cfg.carrier_leadin_ms = 300;   /* default mark-carrier lead-in before first TX byte */
    if (fsk_parse_framing("8N1", &g_cfg.framing) != 0) {
        fprintf(stderr, "internal: default framing parse failed\n");
        return 1;
    }

    int opt;
    while ((opt = getopt(argc, argv, "l:d:M:of:A:wW:C:Bm:b:H")) != -1) {
        switch (opt) {
        case 'l':
            g_cfg.listen_port = atoi(optarg);
            break;
        case 'd': {
            const char *hp = optarg;
            const char *col = strrchr(hp, ':');
            if (!col) usage(argv[0]);
            int hlen = (int)(col - hp);
            if (hlen <= 0 || hlen >= (int)sizeof(g_cfg.data_host)) usage(argv[0]);
            memcpy(g_cfg.data_host, hp, (size_t)hlen);
            g_cfg.data_host[hlen] = '\0';
            g_cfg.data_port = atoi(col + 1);
            if (g_cfg.data_port <= 0 || g_cfg.data_port > 65535) usage(argv[0]);
            break;
        }
        case 'M':
            if (strcasecmp(optarg, "v21") == 0)           g_cfg.mode = FSK_V21;
            else if (strcasecmp(optarg, "v23") == 0)      g_cfg.mode = FSK_V23;
            else if (strcasecmp(optarg, "bell103") == 0
                  || strcasecmp(optarg, "b103") == 0)     g_cfg.mode = FSK_BELL103;
            else { fprintf(stderr, "bad -M value (want v21, v23, or bell103)\n"); return 1; }
            break;
        case 'o':
            g_cfg.originate = 1;
            break;
        case 'f':
            if (fsk_parse_framing(optarg, &g_cfg.framing) != 0) {
                fprintf(stderr, "bad -f value (want 8N1, 7E1, …, or 'sync')\n");
                return 1;
            }
            break;
        case 'A': {
            int a = atoi(optarg);
            if (a < 1 || a > 32767) {
                fprintf(stderr, "bad -A amplitude (1..32767)\n");
                return 1;
            }
            g_cfg.amplitude = (int16_t)a;
            break;
        }
        case 'w':
            g_cfg.wait_carrier = 1;
            break;
        case 'W': {
            double secs = atof(optarg);
            if (secs < 0 || secs > 600) {
                fprintf(stderr, "bad -W value (seconds, 0..600)\n");
                return 1;
            }
            g_cfg.connect_delay_ms = (int)(secs * 1000.0 + 0.5);
            g_cfg.wait_carrier = 1;   /* a post-carrier delay implies waiting for carrier */
            break;
        }
        case 'C': {
            int ms = atoi(optarg);
            if (ms < 0 || ms > 10000) {
                fprintf(stderr, "bad -C value (ms, 0..10000)\n");
                return 1;
            }
            g_cfg.carrier_leadin_ms = ms;
            break;
        }
        case 'B':
            g_cfg.break_after_cd = 1;
            break;
        case 'm': {
            int m = atoi(optarg);
            if (m < 0) {
                fprintf(stderr, "bad -m value (must be >= 0)\n");
                return 1;
            }
            g_cfg.max_sessions = m;
            break;
        }
        case 'b': {
            int n = decode_banner(optarg, g_cfg.banner, (int)sizeof(g_cfg.banner));
            if (n < 0) {
                fprintf(stderr, "bad -b banner (too long or bad \\x escape)\n");
                return 1;
            }
            g_cfg.banner_len = n;
            break;
        }
        case 'H':
            g_cfg.expect_header = 1;
            break;
        default:
            usage(argv[0]);
        }
    }
    if (g_cfg.listen_port <= 0 || !g_cfg.data_host[0] || g_cfg.data_port == 0)
        usage(argv[0]);

    /* -o selects the side for the symmetric standards (V.21 / Bell 103);
     * it has no meaning for V.23 (host-side only). */
    int symmetric = (g_cfg.mode == FSK_V21 || g_cfg.mode == FSK_BELL103);
    if (g_cfg.originate && !symmetric) {
        fprintf(stderr, "-o (originate) applies only to -M v21 or -M bell103\n");
        return 1;
    }
    /* -B freezes the entire TX framer, which is wrong for a symmetric
     * full-duplex link (it would stall the reverse direction too). Warn
     * but proceed. */
    if (g_cfg.break_after_cd && symmetric)
        fprintf(stderr,
                "warning: -B (break) is intended for V.23; on a full-duplex "
                "link (v21/bell103) it stalls all outbound data\n");

    /* Init the shared sine LUT once. */
    fsk_init();

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return 1; }
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)g_cfg.listen_port);
    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }
    if (listen(srv, 8) < 0) { perror("listen"); return 1; }

    const char *mode_str = g_cfg.mode == FSK_V21     ? "v21"
                         : g_cfg.mode == FSK_BELL103 ? "bell103"
                                                     : "v23";
    const char *side_str = symmetric
                         ? (g_cfg.originate ? "originate" : "answer") : "-";
    if (g_cfg.framing.sync) {
        fprintf(stderr,
                "modem_fsk listening on :%d, data -> %s:%d, mode=%s side=%s framing=sync\n",
                g_cfg.listen_port, g_cfg.data_host, g_cfg.data_port,
                mode_str, side_str);
    } else {
        fprintf(stderr,
                "modem_fsk listening on :%d, data -> %s:%d, mode=%s side=%s framing=%d%c%d%s\n",
                g_cfg.listen_port, g_cfg.data_host, g_cfg.data_port,
                mode_str, side_str,
                g_cfg.framing.data_bits, g_cfg.framing.parity, g_cfg.framing.stop_bits,
                g_cfg.banner_len > 0 ? ", banner enabled" : "");
    }

    while (1) {
        struct sockaddr_in cli;
        socklen_t cl = sizeof(cli);
        int cfd = accept(srv, (struct sockaddr *)&cli, &cl);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            perror("accept"); break;
        }

        /* Enforce the concurrent-session cap before we spend any resources
         * on the new connection. Use a single atomic add so we don't race
         * two accepts past the limit. */
        if (g_cfg.max_sessions > 0) {
            int n = __sync_add_and_fetch(&g_session_count, 1);
            if (n > g_cfg.max_sessions) {
                __sync_sub_and_fetch(&g_session_count, 1);
                fprintf(stderr,
                        "modem_fsk: session cap %d reached, rejecting new audio connection\n",
                        g_cfg.max_sessions);
                close(cfd);
                continue;
            }
        } else {
            __sync_add_and_fetch(&g_session_count, 1);
        }

        session_t *s = calloc(1, sizeof(*s));
        if (!s) { close(cfd); __sync_sub_and_fetch(&g_session_count, 1); continue; }
        s->audio_sock = cfd;

        pthread_t th;
        if (pthread_create(&th, NULL, session_thread, s) != 0) {
            close(cfd); free(s); __sync_sub_and_fetch(&g_session_count, 1); continue;
        }
        pthread_detach(th);
    }
    return 0;
}
