#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "v22bis.h"
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

/* ── modem_v22bis ──────────────────────────────────────────────────────
 * Session harness for the V.22bis core, mirroring modem_fsk: listen for an
 * 8 kHz s16le audio connection, dial out a TCP byte/data peer, and bridge
 * the two — demodulate inbound audio to bytes for the data peer, modulate
 * the data peer's bytes back out as audio. Full-duplex: one output sample is
 * produced for every input sample, keeping the audio stream 1:1 paced. */

typedef struct {
    int        listen_port;
    char       data_host[256];
    int        data_port;
    int        calling_party;   /* -o: 1 = calling/originate, 0 = answering */
    int        max_bit_rate;    /* -r: 1200 or 2400 (default 2400)          */
    int        guard;           /* -G: V22_GUARD_* (answering only)         */
    int        amplitude;       /* -A                                       */
    int        max_sessions;    /* -m                                       */
    int        expect_header;   /* -H                                       */
} cfg_t;

static cfg_t g_cfg;
static volatile int g_session_count;

/* ── Per-session state ────────────────────────────────────────────────── */

typedef struct {
    int        audio_sock;          /* accepted, non-blocking          */
    int        data_sock;           /* outbound byte channel, non-block */

    v22bis_t   modem;

    /* TX byte FIFO (data peer -> modulator), consumed LSB-first by get_bit. */
    uint8_t    tx_q[4096];
    int        tx_q_len;            /* bytes valid in tx_q              */
    int        tx_rd;               /* read cursor into tx_q (bytes)   */
    uint8_t    tx_cur;              /* byte currently being shifted out */
    int        tx_cur_bits;         /* bits left in tx_cur (0..8)       */

    /* RX byte FIFO (demodulator -> data peer), assembled LSB-first by put_bit. */
    uint8_t    rx_q[4096];
    int        rx_q_len;
    uint8_t    rx_cur;              /* byte being assembled            */
    int        rx_cur_bits;         /* bits collected (0..8)           */

    uint8_t    in_stage[2];         /* odd audio byte carried across reads */
    int        in_stage_len;

    int        trained_logged;      /* one-shot "trained" log          */

    /* Stream-header consumption (-H), identical scheme to modem_fsk. */
    int        hdr_done, hdr_parsed, hdr_reject, hdr_have;
    uint8_t    hdr_buf[12];
    int        hdr_payload_remaining;
    char       hdr_line[128];
    int        hdr_line_len, hdr_line_done;
} session_t;

/* ── Bit I/O callbacks bound to the session FIFOs ─────────────────────── */

/* Next TX bit, LSB-first from the byte FIFO; 1 (idle ones) when empty. */
static int sess_get_bit(void *user) {
    session_t *s = (session_t *)user;
    if (s->tx_cur_bits == 0) {
        if (s->tx_rd >= s->tx_q_len) return 1;   /* idle: scrambled ones */
        s->tx_cur = s->tx_q[s->tx_rd++];
        s->tx_cur_bits = 8;
    }
    int bit = s->tx_cur & 1;
    s->tx_cur >>= 1;
    s->tx_cur_bits--;
    return bit;
}

/* One recovered data bit, assembled LSB-first into the RX byte FIFO. */
static void sess_put_bit(void *user, int bit) {
    session_t *s = (session_t *)user;
    s->rx_cur |= (uint8_t)((bit & 1) << s->rx_cur_bits);
    if (++s->rx_cur_bits == 8) {
        if (s->rx_q_len < (int)sizeof(s->rx_q))
            s->rx_q[s->rx_q_len++] = s->rx_cur;
        s->rx_cur = 0;
        s->rx_cur_bits = 0;
    }
}

/* ── TCP helpers (mirror modem_fsk) ───────────────────────────────────── */

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

static void tcp_write_nb(int *sock, const void *buf, size_t len) {
    if (*sock < 0) return;
    const uint8_t *p = (const uint8_t *)buf;
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t w = write(*sock, p, remaining);
        if (w > 0) { p += w; remaining -= (size_t)w; }
        else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
        else { close(*sock); *sock = -1; return; }
    }
}

/* ── Stream header (-H), same wire format as modem_fsk ────────────────── */

static int consume_header(session_t *s, const uint8_t *buf, int n) {
    int i = 0;
    while (s->hdr_have < 12 && i < n)
        s->hdr_buf[s->hdr_have++] = buf[i++];
    if (s->hdr_have < 12) return i;

    if (!s->hdr_parsed) {
        if (s->hdr_buf[0] != 'S' || s->hdr_buf[1] != 'H') {
            fprintf(stderr, "session %p: bad stream-header magic — treating "
                    "stream as raw audio\n", (void *)s);
            s->hdr_done = 1;
            return i;
        }
        unsigned ver = s->hdr_buf[2], flags = s->hdr_buf[3];
        unsigned codec = s->hdr_buf[4], channels = s->hdr_buf[5];
        uint16_t rate_be; memcpy(&rate_be, s->hdr_buf + 6, sizeof(rate_be));
        unsigned rate = ntohs(rate_be);
        uint32_t len_be; memcpy(&len_be, s->hdr_buf + 8, sizeof(len_be));
        s->hdr_payload_remaining = (int)ntohl(len_be);
        s->hdr_parsed = 1;
        fprintf(stderr, "session %p: stream header v%u: codec=%u %uHz %uch, "
                "%d-byte payload\n", (void *)s, ver, codec, rate, channels,
                s->hdr_payload_remaining);
        if (ver != 2 || codec != 0 || rate != 8000 || channels != 1) {
            fprintf(stderr, "session %p: unsupported stream format "
                    "(need v2 s16le 8000Hz 1ch) — closing\n", (void *)s);
            s->hdr_reject = 1;
            return i;
        }
        if (!(flags & 0x01))
            fprintf(stderr, "session %p: header VOICE_FOLLOWS bit clear — "
                    "proceeding anyway\n", (void *)s);
    }
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

/* ── Audio block processing ───────────────────────────────────────────── */

/* Demodulate ns input samples and modulate ns output samples (1:1), writing
 * the output back to the audio peer. */
static void process_block(session_t *s, const int16_t *in, int ns) {
    int16_t out[640];
    while (ns > 0) {
        int chunk = ns > 640 ? 640 : ns;
        v22bis_rx(&s->modem, in, chunk);
        v22bis_tx(&s->modem, out, chunk);
        tcp_write_nb(&s->audio_sock, out, (size_t)chunk * sizeof(int16_t));
        in += chunk;
        ns -= chunk;
    }
    if (!s->trained_logged && v22bis_rx_trained(&s->modem)) {
        s->trained_logged = 1;
        fprintf(stderr, "session %p: trained, %d bps\n",
                (void *)s, v22bis_current_bit_rate(&s->modem));
    }
}

static void session_io_loop(session_t *s) {
    while (s->audio_sock >= 0) {
        fd_set rfds, wfds;
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_SET(s->audio_sock, &rfds);

        int tx_room = (int)sizeof(s->tx_q) - s->tx_q_len;
        if (s->data_sock >= 0 && tx_room > 0) FD_SET(s->data_sock, &rfds);
        if (s->data_sock >= 0 && s->rx_q_len > 0) FD_SET(s->data_sock, &wfds);

        int maxfd = s->audio_sock;
        if (s->data_sock > maxfd) maxfd = s->data_sock;

        struct timeval tv = {1, 0};
        int sel = select(maxfd + 1, &rfds, &wfds, NULL, &tv);
        if (sel < 0) { if (errno == EINTR) continue; break; }

        /* AUDIO IN -> demod (+ modulate one out per in). */
        if (FD_ISSET(s->audio_sock, &rfds)) {
            uint8_t buf[640];
            ssize_t n = read(s->audio_sock, buf, sizeof(buf));
            if (n <= 0) {
                if (n == 0) break;
                if (errno != EAGAIN && errno != EWOULDBLOCK) break;
            } else {
                int i = 0;
                if (g_cfg.expect_header && !s->hdr_done) {
                    i = consume_header(s, buf, (int)n);
                    if (s->hdr_reject) break;
                }
                int16_t samples[320];
                int ns = 0;
                while (i < (int)n) {
                    int16_t samp;
                    if (s->in_stage_len == 1) {
                        s->in_stage[1] = buf[i++];
                        memcpy(&samp, s->in_stage, 2);
                        s->in_stage_len = 0;
                    } else if (i + 1 < (int)n) {
                        memcpy(&samp, buf + i, 2);
                        i += 2;
                    } else {
                        s->in_stage[0] = buf[i++];
                        s->in_stage_len = 1;
                        continue;
                    }
                    samples[ns++] = samp;
                    if (ns == 320) { process_block(s, samples, ns); ns = 0; }
                }
                if (ns > 0) process_block(s, samples, ns);
            }
        }

        /* DATA IN -> tx_q (compact first so the read cursor doesn't strand
         * space at the front). */
        if (s->data_sock >= 0 && FD_ISSET(s->data_sock, &rfds)) {
            if (s->tx_rd > 0) {
                memmove(s->tx_q, s->tx_q + s->tx_rd,
                        (size_t)(s->tx_q_len - s->tx_rd));
                s->tx_q_len -= s->tx_rd;
                s->tx_rd = 0;
            }
            int room = (int)sizeof(s->tx_q) - s->tx_q_len;
            if (room > 0) {
                ssize_t n = read(s->data_sock, s->tx_q + s->tx_q_len, (size_t)room);
                if (n > 0) s->tx_q_len += (int)n;
                else if (n == 0) { close(s->data_sock); s->data_sock = -1; }
                else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    close(s->data_sock); s->data_sock = -1;
                }
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

    int flags = fcntl(s->audio_sock, F_GETFL, 0);
    fcntl(s->audio_sock, F_SETFL, flags | O_NONBLOCK);
    int one = 1;
    setsockopt(s->audio_sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    v22bis_init(&s->modem, g_cfg.max_bit_rate, g_cfg.calling_party, g_cfg.guard,
                0 /* spandsp_compat */, sess_get_bit, s, sess_put_bit, s,
                g_cfg.amplitude);

    s->data_sock = -1;
    if (tcp_connect_data(&s->data_sock) < 0) {
        fprintf(stderr, "session %p: data dial-out to %s:%d failed\n",
                (void *)s, g_cfg.data_host, g_cfg.data_port);
        close(s->audio_sock);
        free(s);
        __sync_sub_and_fetch(&g_session_count, 1);
        return NULL;
    }

    session_io_loop(s);

    if (s->audio_sock >= 0) close(s->audio_sock);
    if (s->data_sock  >= 0) close(s->data_sock);
    free(s);
    __sync_sub_and_fetch(&g_session_count, 1);
    return NULL;
}

/* ── main: listen/accept loop ─────────────────────────────────────────── */

static void usage(const char *argv0) {
    fprintf(stderr,
        "Usage: %s -l audio_port -d data_host:data_port [-o] [-r 1200|2400]\n"
        "          [-G none|550|1800] [-A amp] [-m N] [-H]\n"
        "  -l  TCP port to listen on for incoming audio (s16le, 8 kHz)\n"
        "  -d  TCP host:port to dial out for the byte/data channel\n"
        "  -o  Calling (originate) side; default is the answering side\n"
        "  -r  Max bit rate: 1200 or 2400 (default 2400, negotiates down)\n"
        "  -G  Guard tone (answering side only): none, 550, or 1800 (default none)\n"
        "  -A  TX amplitude (int16 peak magnitude, default 12000)\n"
        "  -m  Max concurrent sessions (default 0 = unlimited)\n"
        "  -H  Expect a stream header (sip_interface -H) before the PCM\n",
        argv0);
    exit(1);
}

int main(int argc, char **argv) {
    signal(SIGPIPE, SIG_IGN);

    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.max_bit_rate = 2400;
    g_cfg.guard        = V22_GUARD_NONE;
    g_cfg.amplitude    = 12000;

    int opt;
    while ((opt = getopt(argc, argv, "l:d:or:G:A:m:H")) != -1) {
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
        case 'o':
            g_cfg.calling_party = 1;
            break;
        case 'r': {
            int r = atoi(optarg);
            if (r != 1200 && r != 2400) {
                fprintf(stderr, "bad -r value (want 1200 or 2400)\n");
                return 1;
            }
            g_cfg.max_bit_rate = r;
            break;
        }
        case 'G':
            if (strcasecmp(optarg, "none") == 0)      g_cfg.guard = V22_GUARD_NONE;
            else if (strcmp(optarg, "550") == 0)      g_cfg.guard = V22_GUARD_550;
            else if (strcmp(optarg, "1800") == 0)     g_cfg.guard = V22_GUARD_1800;
            else { fprintf(stderr, "bad -G value (want none, 550, or 1800)\n"); return 1; }
            break;
        case 'A': {
            int a = atoi(optarg);
            if (a < 1 || a > 32767) {
                fprintf(stderr, "bad -A amplitude (1..32767)\n");
                return 1;
            }
            g_cfg.amplitude = a;
            break;
        }
        case 'm': {
            int m = atoi(optarg);
            if (m < 0) { fprintf(stderr, "bad -m value (must be >= 0)\n"); return 1; }
            g_cfg.max_sessions = m;
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
    if (g_cfg.calling_party && g_cfg.guard != V22_GUARD_NONE) {
        fprintf(stderr, "-G (guard tone) applies only to the answering side\n");
        return 1;
    }

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return 1; }
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)g_cfg.listen_port);
    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }
    if (listen(srv, 8) < 0) { perror("listen"); return 1; }

    fprintf(stderr,
            "modem_v22bis listening on :%d, data -> %s:%d, side=%s maxrate=%d guard=%s\n",
            g_cfg.listen_port, g_cfg.data_host, g_cfg.data_port,
            g_cfg.calling_party ? "calling" : "answering", g_cfg.max_bit_rate,
            g_cfg.guard == V22_GUARD_NONE ? "none" :
            g_cfg.guard == V22_GUARD_550  ? "550"  : "1800");

    while (1) {
        struct sockaddr_in cli;
        socklen_t cl = sizeof(cli);
        int cfd = accept(srv, (struct sockaddr *)&cli, &cl);
        if (cfd < 0) { if (errno == EINTR) continue; perror("accept"); break; }

        if (g_cfg.max_sessions > 0) {
            int n = __sync_add_and_fetch(&g_session_count, 1);
            if (n > g_cfg.max_sessions) {
                __sync_sub_and_fetch(&g_session_count, 1);
                fprintf(stderr, "modem_v22bis: session cap %d reached, rejecting\n",
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
