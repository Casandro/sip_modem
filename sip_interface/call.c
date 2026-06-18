#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "call.h"
#include "sip_util.h"
#include "alaw.h"
#include "g722.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>

/* ── SIP helpers ───────────────────────────────────────────────────── */

/* Header parsing, method/body extraction, response-code, digest helpers, the
 * random-hex generator and the timespec math all live in sip_util.c. */

static void sip_send(server_t *srv, const char *buf, int len,
                     struct sockaddr_in *to) {
    sendto(srv->sip_sock, buf, (size_t)len, 0,
           (struct sockaddr *)to, sizeof(*to));
}

/* ── Codecs ───────────────────────────────────────────────────────── */

#define CODEC_PCMA      8       /* G.711 A-law, 8 kHz  (RTP PT 8) */
#define CODEC_G722      9       /* G.722 wideband, 16 kHz (RTP PT 9) */
#define CODEC_CLEARMODE 256     /* RFC 4040 transparent 64 kbit/s (dynamic PT);
                                 * internal id only, never a wire PT */
#define CLEARMODE_OFFER_PT 96   /* dynamic PT we put in our CLEARMODE offer */

/* Bytes carried on the TCP side per 20 ms frame: s16le PCM for PCMA/G.722,
 * raw 64 kbit/s octets for CLEARMODE. */
static int codec_tcp_bytes(int codec) {
    if (codec == CODEC_G722)      return 640;   /* 320 samples x 2 */
    if (codec == CODEC_CLEARMODE) return 160;   /* 160 raw octets */
    return 320;                                 /* 160 samples x 2 (PCMA) */
}
/* Rate declared in the stream header for the TCP-side data. */
static int codec_rate(int codec)      { return codec == CODEC_G722 ? 16000 : 8000; }
/* RTP timestamp increment per 20 ms (G.722 uses an 8 kHz RTP clock per
 * RFC 3551 §4.5.2, so both advance by 160). */
#define RTP_TS_INCR 160

/* ── RTP ──────────────────────────────────────────────────────────── */

typedef struct __attribute__((packed)) {
    uint8_t  vpxcc;
    uint8_t  mpt;
    uint16_t seq;
    uint32_t ts;
    uint32_t ssrc;
} rtp_hdr_t;

static void rtp_fill_hdr(rtp_hdr_t *h, uint16_t seq, uint32_t ts,
                          uint32_t ssrc, int pt) {
    h->vpxcc = 0x80;
    h->mpt   = (uint8_t)pt;     /* payload type (8=PCMA, 9=G722) */
    h->seq   = htons(seq);
    h->ts    = htonl(ts);
    h->ssrc  = htonl(ssrc);
}

/* ── SDP parser ───────────────────────────────────────────────────── */

/* Parse the audio m=/c=/a=rtpmap lines and pick a codec. With want_clearmode,
 * ONLY CLEARMODE (RFC 4040, dynamic PT found via rtpmap) is accepted; otherwise
 * G.722 (PT 9) is preferred when allow_g722, else PCMA (PT 8). On success sets
 * *codec (internal id) and *rtp_pt (wire payload type) and returns 1. */
static int sdp_parse(const char *body, char *remote_ip, int *port,
                     int allow_g722, int want_clearmode,
                     int *codec, int *rtp_pt) {
    if (!body) return 0;
    int has_pt8 = 0, has_pt9 = 0;
    int mpts[32], nm = 0;            /* payload types listed on the m= line */
    int clearmode_pt = -1;           /* PT mapped to CLEARMODE via rtpmap */
    *port = 0;
    remote_ip[0] = '\0';

    const char *p = body;
    while (*p) {
        if (p[0] == 'c' && p[1] == '=') {
            const char *ip = strstr(p, "IP4 ");
            if (ip) {
                ip += 4;
                const char *end = ip;
                while (*end && *end != '\r' && *end != '\n') end++;
                int n = (int)(end - ip);
                if (n > 63) n = 63;
                memcpy(remote_ip, ip, (size_t)n);
                remote_ip[n] = '\0';
            }
        } else if (p[0] == 'm' && p[1] == '=') {
            int pv;
            if (sscanf(p + 2, "audio %d", &pv) == 1) *port = pv;
            const char *pt = p;
            while (*pt && *pt != '\r' && *pt != '\n') pt++;
            char mline[256] = {0};
            int ml = (int)(pt - p);
            if (ml >= 256) ml = 255;
            memcpy(mline, p, (size_t)ml);
            char *tok = strtok(mline, " \t");
            tok = strtok(NULL, " \t"); /* port */
            tok = strtok(NULL, " \t"); /* RTP/AVP */
            while ((tok = strtok(NULL, " \t\r\n")) != NULL) {
                int v = atoi(tok);
                if (v == 8) has_pt8 = 1;
                else if (v == 9) has_pt9 = 1;
                if (nm < 32) mpts[nm++] = v;
            }
        } else if (strncmp(p, "a=rtpmap:", 9) == 0) {
            int pt; char name[32] = {0};
            if (sscanf(p + 9, "%d %31[^/\r\n]", &pt, name) == 2 &&
                strncasecmp(name, "CLEARMODE", 9) == 0)
                clearmode_pt = pt;
        }
        while (*p && *p != '\n') p++;
        if (*p) p++;
    }

    int has_clearmode = 0;
    if (clearmode_pt >= 0)
        for (int i = 0; i < nm; i++)
            if (mpts[i] == clearmode_pt) { has_clearmode = 1; break; }

    if (want_clearmode) {                       /* exclusive: only CLEARMODE */
        if (!has_clearmode) return 0;
        *codec = CODEC_CLEARMODE; *rtp_pt = clearmode_pt;
    } else if (allow_g722 && has_pt9) {
        *codec = CODEC_G722; *rtp_pt = 9;
    } else if (has_pt8) {
        *codec = CODEC_PCMA; *rtp_pt = 8;
    } else if (has_pt9) {                        /* answer may force G.722 */
        *codec = CODEC_G722; *rtp_pt = 9;
    } else {
        return 0;
    }

    return (*port > 0 && remote_ip[0] != '\0') ? 1 : 0;
}

/* ── Per-call context ─────────────────────────────────────────────── */

typedef struct {
    server_t        *srv;
    call_t          *call;

    /* SIP dialog */
    char             call_id[256];
    char             via[512];
    char             from[256];
    char             to[256];
    char             to_tag[32];
    char             cseq[64];

    /* Remote addresses */
    struct sockaddr_in sip_from;
    char               remote_rtp_ip[64];
    int                remote_rtp_port;

    /* RTP */
    int                rtp_sock;
    int                rtp_local_port;
    struct sockaddr_in remote_rtp_addr;
    uint16_t           rtp_seq;
    uint32_t           rtp_ts;
    uint32_t           rtp_ssrc;

    /* Negotiated codec (CODEC_PCMA / CODEC_G722 / CODEC_CLEARMODE) + the wire
     * RTP payload type (8/9 for those; the dynamic PT for CLEARMODE). */
    int                codec;
    int                rtp_pt;
    g722_enc_t         g722e;
    g722_dec_t         g722d;

    /* UAC (dial-out) state. Unused for inbound calls. */
    int                uac;
    char               target_uri[256];   /* sip:callee@host (Request-URI / To) */
    char               from_tag[24];
    char               branch[24];         /* current INVITE transaction branch */
    int                cseq_num;
    int                auth_tries;
    char               req_buf[2048];      /* last INVITE, for retransmission */
    int                req_len;

    /* TCP audio peer (signed 16-bit LE; 8 kHz PCMA / 16 kHz G.722) */
    int                tcp_sock;
    uint8_t            tcp_rx_buf[2560];  /* >= 4 G.722 frames (4 x 640) */
    int                tcp_rx_len;

    /* Last 200 OK to INVITE, kept for TU-level retransmission until ACK */
    char               ok_buf[2048];
    int                ok_buf_len;

    /* Raw received INVITE, for the optional stream header (-H, inbound). */
    char               invite_raw[8192];
    int                invite_raw_len;
} ctx_t;

/* ── SIP response builders ────────────────────────────────────────── */

static void send_provisional(ctx_t *c, int code, const char *phrase) {
    char buf[2048];
    int n = snprintf(buf, sizeof(buf),
        "SIP/2.0 %d %s\r\n"
        "Via: %s\r\n"
        "From: %s\r\n"
        "To: %s\r\n"
        "Call-ID: %s\r\n"
        "CSeq: %s\r\n"
        "Content-Length: 0\r\n"
        "\r\n",
        code, phrase,
        c->via, c->from, c->to, c->call_id, c->cseq);
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;  /* never send past buf */
    sip_send(c->srv, buf, n, &c->sip_from);
}

/* Reject an INVITE we won't accept (no media match, call cap, …): build a
 * minimal context from the request headers and send one final response. */
static void reject(server_t *srv, struct sockaddr_in *from,
                   const char *msg, int code, const char *phrase) {
    ctx_t stub;
    memset(&stub, 0, sizeof(stub));
    stub.srv      = srv;
    stub.sip_from = *from;
    sip_hdr(msg, "Call-ID", stub.call_id, sizeof(stub.call_id));
    sip_hdr(msg, "Via",     stub.via,     sizeof(stub.via));
    sip_hdr(msg, "From",    stub.from,    sizeof(stub.from));
    sip_hdr(msg, "To",      stub.to,      sizeof(stub.to));
    sip_hdr(msg, "CSeq",    stub.cseq,    sizeof(stub.cseq));
    send_provisional(&stub, code, phrase);
}

/* Bind a local RTP socket (ephemeral port) and pick an SSRC. Stores the
 * chosen local port in c->rtp_local_port. Returns 0 on success. */
static int rtp_open(ctx_t *c) {
    c->rtp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (c->rtp_sock < 0) return -1;
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port        = 0;
    if (bind(c->rtp_sock, (struct sockaddr *)&a, sizeof(a)) < 0) {
        close(c->rtp_sock); c->rtp_sock = -1; return -1;
    }
    socklen_t sl = sizeof(a);
    getsockname(c->rtp_sock, (struct sockaddr *)&a, &sl);
    c->rtp_local_port = ntohs(a.sin_port);
    c->rtp_ssrc = rng_seed() * 1664525u + 1013904223u;
    return 0;
}

/* Point remote_rtp_addr at the peer parsed into remote_rtp_ip/port. */
static void set_remote_rtp(ctx_t *c) {
    memset(&c->remote_rtp_addr, 0, sizeof(c->remote_rtp_addr));
    c->remote_rtp_addr.sin_family = AF_INET;
    c->remote_rtp_addr.sin_port   = htons((uint16_t)c->remote_rtp_port);
    inet_pton(AF_INET, c->remote_rtp_ip, &c->remote_rtp_addr.sin_addr);
}

/* Initialise codec state once c->codec is decided. */
static void codec_init(ctx_t *c) {
    if (c->codec == CODEC_G722) {
        g722_enc_init(&c->g722e);
        g722_dec_init(&c->g722d);
    }
}

/* Build our SDP into sdp; returns its length. With offer != 0 this is an
 * outbound offer; otherwise it's the answer carrying the single negotiated
 * c->codec. CLEARMODE (-T) is exclusive — offered/answered alone on its
 * dynamic PT. rtpmap clock is 8000 for all of PCMA/G.722/CLEARMODE. */
static int build_sdp(ctx_t *c, char *sdp, int cap, int offer) {
    time_t now = time(NULL);
    int hd = snprintf(sdp, (size_t)cap,
        "v=0\r\n"
        "o=- %ld %ld IN IP4 %s\r\n"
        "s=sip_interface\r\n"
        "c=IN IP4 %s\r\n"
        "t=0 0\r\n",
        (long)now, (long)now, c->srv->local_ip, c->srv->local_ip);
    if (hd < 0 || hd >= cap) return hd;

    if (c->srv->clearmode) {            /* exclusive transparent 64k */
        int pt = offer ? CLEARMODE_OFFER_PT : c->rtp_pt;
        return hd + snprintf(sdp + hd, (size_t)(cap - hd),
            "m=audio %d RTP/AVP %d\r\n"
            "a=rtpmap:%d CLEARMODE/8000\r\n"
            "a=sendrecv\r\n", c->rtp_local_port, pt, pt);
    }

    if (offer) {
        if (c->srv->allow_g722)
            return hd + snprintf(sdp + hd, (size_t)(cap - hd),
                "m=audio %d RTP/AVP 9 8\r\n"
                "a=rtpmap:9 G722/8000\r\n"
                "a=rtpmap:8 PCMA/8000\r\n"
                "a=sendrecv\r\n", c->rtp_local_port);
        return hd + snprintf(sdp + hd, (size_t)(cap - hd),
            "m=audio %d RTP/AVP 8\r\n"
            "a=rtpmap:8 PCMA/8000\r\n"
            "a=sendrecv\r\n", c->rtp_local_port);
    }
    return hd + snprintf(sdp + hd, (size_t)(cap - hd),
        "m=audio %d RTP/AVP %d\r\n"
        "a=rtpmap:%d %s\r\n"
        "a=sendrecv\r\n",
        c->rtp_local_port, c->codec, c->codec,
        c->codec == CODEC_G722 ? "G722/8000" : "PCMA/8000");
}

/* UAS: bind RTP, point at the caller's media, and answer with 200 OK. */
static int bind_rtp(ctx_t *c) {
    if (rtp_open(c) < 0) return -1;
    set_remote_rtp(c);
    codec_init(c);

    char to_with_tag[300];
    char to_tag[18];
    gen_hex(to_tag, 16);
    snprintf(to_with_tag, sizeof(to_with_tag), "%s;tag=%s", c->to, to_tag);
    strncpy(c->to_tag, to_tag, sizeof(c->to_tag) - 1);

    char sdp[512];
    int sdp_len = build_sdp(c, sdp, sizeof(sdp), 0);

    int n = snprintf(c->ok_buf, sizeof(c->ok_buf),
        "SIP/2.0 200 OK\r\n"
        "Via: %s\r\n"
        "From: %s\r\n"
        "To: %s\r\n"
        "Call-ID: %s\r\n"
        "CSeq: %s\r\n"
        "Contact: <sip:%s@%s:%d>\r\n"
        "Content-Type: application/sdp\r\n"
        "Content-Length: %d\r\n"
        "\r\n"
        "%s",
        c->via, c->from, to_with_tag, c->call_id, c->cseq,
        c->srv->local_user, c->srv->local_ip, c->srv->local_sip_port,
        sdp_len, sdp);
    if (n >= (int)sizeof(c->ok_buf)) n = (int)sizeof(c->ok_buf) - 1;
    c->ok_buf_len = n;
    sip_send(c->srv, c->ok_buf, n, &c->sip_from);
    return 0;
}

/* ── TCP peer connection ──────────────────────────────────────────── */

/* Non-blocking connect with a 5s timeout, trying each resolved address until
 * one succeeds. On success leaves the socket non-blocking with TCP_NODELAY.
 * Uses getaddrinfo (reentrant) rather than gethostbyname (shared static state),
 * since this runs concurrently in every call thread. */
static int tcp_connect(ctx_t *c) {
    char portstr[16];
    snprintf(portstr, sizeof(portstr), "%d", c->srv->tcp_port);

    struct addrinfo hints, *res = NULL, *ai;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_UNSPEC;       /* IPv4 or IPv6 */
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(c->srv->tcp_host, portstr, &hints, &res) != 0) return -1;

    int s = -1;
    for (ai = res; ai; ai = ai->ai_next) {
        s = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (s < 0) continue;

        int flags = fcntl(s, F_GETFL, 0);
        fcntl(s, F_SETFL, flags | O_NONBLOCK);

        int r = connect(s, ai->ai_addr, ai->ai_addrlen);
        if (r == 0) break;                       /* connected immediately */
        if (r < 0 && errno == EINPROGRESS) {
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(s, &wfds);
            struct timeval tv = {5, 0};
            if (select(s + 1, NULL, &wfds, NULL, &tv) > 0) {
                int err = 0;
                socklen_t sl = sizeof(err);
                if (getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &sl) == 0 && err == 0)
                    break;                       /* connected */
            }
        }
        close(s);                                /* this candidate failed */
        s = -1;
    }
    freeaddrinfo(res);
    if (s < 0) return -1;

    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    c->tcp_sock = s;
    return 0;
}

/* Non-blocking write. Closes the TCP socket on hard failure; returns
 * silently on EAGAIN (drops excess data so the audio cadence is preserved
 * even if the TCP peer is slow). */
static void tcp_write(ctx_t *c, const void *buf, size_t len) {
    if (c->tcp_sock < 0) return;
    const uint8_t *p = (const uint8_t *)buf;
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t w = write(c->tcp_sock, p, remaining);
        if (w > 0) {
            p += w;
            remaining -= (size_t)w;
        } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return;  /* drop the rest of this chunk */
        } else {
            close(c->tcp_sock);
            c->tcp_sock = -1;
            return;
        }
    }
}

/* ── Optional stream header (-H) ──────────────────────────────────────
 * 12-byte prefix + payload, written once before any PCM:
 *   'S' 'H' ver flags  codec  channels  samplerate(u16 BE)  length(u32 BE)
 *   payload[length]
 * flags bit0 = VOICE_FOLLOWS (PCM begins right after payload). codec/channels/
 * samplerate describe that PCM (here always s16le / mono / 8 kHz, since the
 * bridge decodes the call's G.711 to linear PCM). payload = received INVITE. */

#define STREAM_HDR_VERSION        2
#define STREAM_HDR_VOICE_FOLLOWS  0x01
#define STREAM_CODEC_S16LE        0     /* linear 16-bit PCM, little-endian */
#define STREAM_PCM_CHANNELS       1     /* mono */
#define STREAM_PCM_RATE           8000  /* Hz */

/* Reliably write all `len` bytes to the non-blocking TCP socket, waiting for
 * writability on EAGAIN (5 s cap). Unlike tcp_write (which drops on EAGAIN to
 * keep audio cadence), the one-shot header must not lose bytes. Returns 0 on
 * success, -1 on failure (socket closed). */
static int tcp_write_all(ctx_t *c, const void *buf, size_t len) {
    if (c->tcp_sock < 0) return -1;
    const uint8_t *p = (const uint8_t *)buf;
    size_t left = len;
    while (left > 0) {
        ssize_t w = write(c->tcp_sock, p, left);
        if (w > 0) { p += w; left -= (size_t)w; continue; }
        if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(c->tcp_sock, &wfds);
            struct timeval tv = {5, 0};
            if (select(c->tcp_sock + 1, NULL, &wfds, NULL, &tv) <= 0) break;
            continue;
        }
        break;  /* hard error */
    }
    if (left == 0) return 0;
    close(c->tcp_sock);
    c->tcp_sock = -1;
    return -1;
}

static void tcp_send_stream_header(ctx_t *c, const uint8_t *payload, int plen) {
    if (plen < 0) plen = 0;
    uint8_t hdr[12];
    hdr[0] = 'S';
    hdr[1] = 'H';
    hdr[2] = STREAM_HDR_VERSION;
    hdr[3] = STREAM_HDR_VOICE_FOLLOWS;
    hdr[4] = (c->codec == CODEC_CLEARMODE) ? 3 : STREAM_CODEC_S16LE; /* 3=raw octets */
    hdr[5] = STREAM_PCM_CHANNELS;
    uint16_t rate = htons((uint16_t)codec_rate(c->codec));
    memcpy(hdr + 6, &rate, sizeof(rate));
    uint32_t be = htonl((uint32_t)plen);
    memcpy(hdr + 8, &be, sizeof(be));
    if (tcp_write_all(c, hdr, sizeof(hdr)) < 0) return;
    if (plen > 0) tcp_write_all(c, payload, (size_t)plen);
}

/* ── RTP send ─────────────────────────────────────────────────────── */

static void rtp_send_payload(ctx_t *c, const uint8_t *payload, int len) {
    uint8_t pkt[12 + 320];
    if (len > 320) len = 320;
    rtp_fill_hdr((rtp_hdr_t *)pkt, c->rtp_seq, c->rtp_ts, c->rtp_ssrc, c->rtp_pt);
    memcpy(pkt + 12, payload, (size_t)len);
    sendto(c->rtp_sock, pkt, (size_t)(12 + len), 0,
           (struct sockaddr *)&c->remote_rtp_addr,
           sizeof(c->remote_rtp_addr));
    c->rtp_seq++;
    c->rtp_ts += RTP_TS_INCR;
}

/* ── I/O loop ─────────────────────────────────────────────────────── */

static void io_loop(ctx_t *c, int dispatch_fd) {
    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);
    ts_add_ms(&next_tick, 20);

    int got_rtp_in_tick = 0;

    while (1) {
        long wait_ms = ts_until_ms(&next_tick);
        if (wait_ms < 0) wait_ms = 0;

        struct timeval tv;
        tv.tv_sec  = wait_ms / 1000;
        tv.tv_usec = (wait_ms % 1000) * 1000;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(c->rtp_sock, &rfds);
        FD_SET(dispatch_fd, &rfds);
        /* Only poll TCP for reading when we have room — otherwise the fd
         * stays in the readable set and select() busy-loops. Letting the
         * kernel hold the bytes is what naturally back-pressures the
         * sender. */
        int tcp_room = c->tcp_sock >= 0
                       ? (int)sizeof(c->tcp_rx_buf) - c->tcp_rx_len : 0;
        if (tcp_room > 0) FD_SET(c->tcp_sock, &rfds);

        int maxfd = c->rtp_sock;
        if (dispatch_fd > maxfd) maxfd = dispatch_fd;
        if (c->tcp_sock > maxfd) maxfd = c->tcp_sock;
        select(maxfd + 1, &rfds, NULL, NULL, &tv);

        /* RX: receive RTP, decode aLaw → s16le, forward to TCP */
        if (FD_ISSET(c->rtp_sock, &rfds)) {
            uint8_t pkt[2048];
            struct sockaddr_in src;
            socklen_t sl = sizeof(src);
            ssize_t r = recvfrom(c->rtp_sock, pkt, sizeof(pkt), 0,
                                 (struct sockaddr *)&src, &sl);
            if (r > 12) {
                /* Symmetric RTP: track NAT-mapped source address */
                if (src.sin_addr.s_addr != c->remote_rtp_addr.sin_addr.s_addr ||
                    src.sin_port        != c->remote_rtp_addr.sin_port) {
                    c->remote_rtp_addr = src;
                }
                int pt = pkt[1] & 0x7F;
                if (pt == c->rtp_pt) {
                    int hdr_sz = 12 + ((pkt[0] & 0x0F) * 4);
                    int n = (int)r - hdr_sz;
                    if (n > 0) {
                        if (n > 320) n = 320;  /* clamp pathological frames */
                        if (!c->srv->ignore_rtp_rx) {
                            if (c->codec == CODEC_CLEARMODE) {
                                /* Transparent: raw octets straight to TCP. */
                                tcp_write(c, pkt + hdr_sz, (size_t)n);
                            } else {
                                int16_t pcm[640]; /* G.722: 320 octets -> 640 samples */
                                int ns;
                                if (c->codec == CODEC_G722) {
                                    ns = g722_decode(&c->g722d, pkt + hdr_sz, n, pcm);
                                } else {
                                    for (int i = 0; i < n; i++)
                                        pcm[i] = alaw_decode(pkt[hdr_sz + i]);
                                    ns = n;
                                }
                                tcp_write(c, pcm, (size_t)ns * sizeof(int16_t));
                            }
                        }
                        got_rtp_in_tick = 1;
                    }
                }
            }
        }

        /* TCP RX: accumulate s16le samples for the next RTP TX. tcp_room
         * is guaranteed > 0 here because we only put the fd in rfds when
         * there's space. */
        if (c->tcp_sock >= 0 && FD_ISSET(c->tcp_sock, &rfds)) {
            ssize_t r = read(c->tcp_sock,
                             c->tcp_rx_buf + c->tcp_rx_len,
                             (size_t)tcp_room);
            if (r > 0) {
                c->tcp_rx_len += (int)r;
            } else if (r == 0) {
                /* TCP peer closed */
                close(c->tcp_sock);
                c->tcp_sock = -1;
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                close(c->tcp_sock);
                c->tcp_sock = -1;
            }
        }

        /* Dispatch: BYE / retransmitted INVITE */
        if (FD_ISSET(dispatch_fd, &rfds)) {
            char msg[4096];
            ssize_t r = read(dispatch_fd, msg, sizeof(msg) - 1);
            if (r > 0) {
                msg[r] = '\0';
                char method[32];
                sip_method(msg, method, sizeof(method));
                if (strcasecmp(method, "BYE") == 0) {
                    /* Echo the BYE's own Via/From/To/Call-ID/CSeq in the 200
                     * OK — correct regardless of which side we are. */
                    reject(c->srv, &c->sip_from, msg, 200, "OK");
                    return;
                } else if (!c->uac && strcasecmp(method, "INVITE") == 0) {
                    /* UAS: a retransmitted INVITE (our 200 was lost) — resend it. */
                    sip_send(c->srv, c->ok_buf, c->ok_buf_len, &c->sip_from);
                } else if (c->uac && c->ok_buf_len > 0 &&
                           strncmp(msg, "SIP/2.0 2", 9) == 0) {
                    /* UAC: the far end retransmitted its 2xx because our ACK was
                     * lost — replay the ACK (RFC 3261 §13.2.2.4). */
                    sip_send(c->srv, c->ok_buf, c->ok_buf_len, &c->sip_from);
                }
            }
        }

        /* 20 ms tick: pull one frame of s16le from the TCP side, encode it,
         * and send one RTP packet (160 octets either codec). If no RTP came
         * in this window, push a frame of silence down the TCP side. */
        if (ts_until_ms(&next_tick) <= 0) {
            int want_bytes = codec_tcp_bytes(c->codec);  /* 320 PCMA / 640 G.722 / 160 CM */
            int take = c->tcp_rx_len < want_bytes ? c->tcp_rx_len : want_bytes;
            uint8_t payload[160];

            if (c->codec == CODEC_CLEARMODE) {
                /* Transparent: raw octets straight from TCP; pad idle with 0xFF. */
                memcpy(payload, c->tcp_rx_buf, (size_t)take);
                if (take < want_bytes)
                    memset(payload + take, 0xFF, (size_t)(want_bytes - take));
            } else {
                int16_t tx_pcm[320];
                memcpy(tx_pcm, c->tcp_rx_buf, (size_t)take);
                if (take < want_bytes)
                    memset((uint8_t *)tx_pcm + take, 0, (size_t)(want_bytes - take));
                if (c->codec == CODEC_G722)
                    g722_encode(&c->g722e, tx_pcm, want_bytes / 2, payload);  /* 320->160 */
                else
                    for (int i = 0; i < 160; i++)
                        payload[i] = alaw_encode(tx_pcm[i]);
            }
            if (c->tcp_rx_len > take)
                memmove(c->tcp_rx_buf, c->tcp_rx_buf + take,
                        (size_t)(c->tcp_rx_len - take));
            c->tcp_rx_len -= take;

            rtp_send_payload(c, payload, 160);

            if (!got_rtp_in_tick && !c->srv->ignore_rtp_rx) {
                if (c->codec == CODEC_CLEARMODE) {
                    uint8_t idle[160];
                    memset(idle, 0xFF, sizeof idle);
                    tcp_write(c, idle, sizeof idle);
                } else {
                    static const int16_t silence[320] = {0};
                    tcp_write(c, silence, (size_t)want_bytes);
                }
            }
            got_rtp_in_tick = 0;

            ts_add_ms(&next_tick, 20);
        }
    }
}

/* ── Call thread ──────────────────────────────────────────────────── */

static void *call_thread(void *arg) {
    ctx_t *c = (ctx_t *)arg;
    int dispatch_fd = c->call->dispatch_pipe[0];

    send_provisional(c, 100, "Trying");
    send_provisional(c, 180, "Ringing");

    /* Open the upstream audio socket BEFORE answering, so a failure here
     * surfaces to the caller as 500 instead of as a dead call. */
    if (tcp_connect(c) < 0) {
        send_provisional(c, 500, "Backend Unavailable");
        goto done;
    }

    /* Optionally announce the call to the peer with a framed header carrying
     * the received INVITE, before any PCM. */
    if (c->srv->stream_header)
        tcp_send_stream_header(c, (const uint8_t *)c->invite_raw, c->invite_raw_len);

    if (bind_rtp(c) < 0) {
        send_provisional(c, 500, "Server Internal Error");
        goto done;
    }

    /* Wait for ACK, retransmitting 200 OK per RFC 3261 §13.3.1.4 / §17.2.1.
     * Timer schedule: T1=500ms, doubling up to T2=4s, total 64*T1=32s. */
    {
        struct timespec next_tx, giveup;
        clock_gettime(CLOCK_MONOTONIC, &next_tx);
        giveup = next_tx;
        ts_add_ms(&next_tx, 500);
        ts_add_ms(&giveup,  32000);
        long rto_ms = 500;
        int acked = 0;

        while (!acked) {
            long wait_giveup = ts_until_ms(&giveup);
            if (wait_giveup <= 0) goto done;
            long wait_tx = ts_until_ms(&next_tx);
            long wait_ms = wait_tx < wait_giveup ? wait_tx : wait_giveup;
            if (wait_ms < 0) wait_ms = 0;

            struct timeval tv;
            tv.tv_sec  = wait_ms / 1000;
            tv.tv_usec = (wait_ms % 1000) * 1000;
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(dispatch_fd, &rfds);
            int r = select(dispatch_fd + 1, &rfds, NULL, NULL, &tv);
            if (r < 0) goto done;

            if (r > 0 && FD_ISSET(dispatch_fd, &rfds)) {
                char msg[4096];
                ssize_t rd = read(dispatch_fd, msg, sizeof(msg) - 1);
                if (rd <= 0) goto done;
                msg[rd] = '\0';
                char method[32];
                sip_method(msg, method, sizeof(method));
                if (strcasecmp(method, "ACK") == 0) {
                    acked = 1;
                    break;
                } else if (strcasecmp(method, "INVITE") == 0) {
                    sip_send(c->srv, c->ok_buf, c->ok_buf_len, &c->sip_from);
                }
            }

            if (ts_until_ms(&next_tx) <= 0) {
                sip_send(c->srv, c->ok_buf, c->ok_buf_len, &c->sip_from);
                rto_ms *= 2;
                if (rto_ms > 4000) rto_ms = 4000;
                clock_gettime(CLOCK_MONOTONIC, &next_tx);
                ts_add_ms(&next_tx, rto_ms);
            }
        }
    }

    io_loop(c, dispatch_fd);

done:
    if (c->tcp_sock >= 0) close(c->tcp_sock);
    if (c->rtp_sock >= 0) close(c->rtp_sock);
    close(dispatch_fd);
    free(c);
    return NULL;
}

/* ── Public API ───────────────────────────────────────────────────── */

call_t *call_start(server_t *srv,
                   const char *msg, int msglen,
                   struct sockaddr_in *from) {

    char call_id[256] = "", via[512] = "", fr[256] = "";
    char to[256] = "", cseq_val[64] = "";
    sip_hdr(msg, "Call-ID",  call_id, sizeof(call_id));
    sip_hdr(msg, "Via",      via,     sizeof(via));
    sip_hdr(msg, "From",     fr,      sizeof(fr));
    sip_hdr(msg, "To",       to,      sizeof(to));
    sip_hdr(msg, "CSeq",     cseq_val, sizeof(cseq_val));
    if (!call_id[0] || !via[0]) return NULL;

    /* Enforce the concurrent-call cap. Count live entries under the lock;
     * any already-terminated threads will be cleared by call_reap() on the
     * main loop's next pass. Reject overflow with 486 Busy Here. */
    if (srv->max_calls > 0) {
        pthread_mutex_lock(&srv->calls_mutex);
        int n = 0;
        for (call_t *cc = srv->calls; cc; cc = cc->next) n++;
        pthread_mutex_unlock(&srv->calls_mutex);
        if (n >= srv->max_calls) {
            reject(srv, from, msg, 486, "Busy Here");
            fprintf(stderr,
                    "sip_interface: call cap %d reached, rejecting INVITE with 486\n",
                    srv->max_calls);
            return NULL;
        }
    }

    const char *body = sip_body(msg);
    char remote_ip[64] = "";
    int  remote_port   = 0;
    int  codec         = CODEC_PCMA;
    int  rtp_pt        = CODEC_PCMA;
    if (!sdp_parse(body, remote_ip, &remote_port, srv->allow_g722,
                   srv->clearmode, &codec, &rtp_pt)) {
        reject(srv, from, msg, 415, "Unsupported Media Type");
        return NULL;
    }

    call_t *call = calloc(1, sizeof(call_t));
    ctx_t  *c    = calloc(1, sizeof(ctx_t));
    if (!call || !c) { free(call); free(c); return NULL; }

    if (pipe(call->dispatch_pipe) < 0) { free(call); free(c); return NULL; }

    strncpy(call->call_id, call_id, sizeof(call->call_id)-1);

    c->srv             = srv;
    c->call            = call;
    c->sip_from        = *from;
    c->rtp_sock        = -1;
    c->tcp_sock        = -1;
    strncpy(c->call_id,        call_id,     sizeof(c->call_id)-1);
    strncpy(c->via,            via,         sizeof(c->via)-1);
    strncpy(c->from,           fr,          sizeof(c->from)-1);
    strncpy(c->to,             to,          sizeof(c->to)-1);
    strncpy(c->cseq,           cseq_val,    sizeof(c->cseq)-1);
    strncpy(c->remote_rtp_ip,  remote_ip,   sizeof(c->remote_rtp_ip)-1);
    c->remote_rtp_port = remote_port;
    c->codec = codec;
    c->rtp_pt = rtp_pt;

    /* Keep the raw INVITE for the optional stream header (-H). */
    c->invite_raw_len = msglen < (int)sizeof(c->invite_raw)
                      ? msglen : (int)sizeof(c->invite_raw);
    if (c->invite_raw_len > 0)
        memcpy(c->invite_raw, msg, (size_t)c->invite_raw_len);

    pthread_create(&call->thread, NULL, call_thread, c);

    pthread_mutex_lock(&srv->calls_mutex);
    call->next  = srv->calls;
    srv->calls  = call;
    pthread_mutex_unlock(&srv->calls_mutex);

    return call;
}

/* ── Outbound call (UAC / dial-out) ───────────────────────────────── */

/* Build and send an INVITE (storing it in c->req_buf for retransmission).
 * If realm/nonce are given, add a digest Authorization (407 ⇒ proxy). */
static void send_invite(ctx_t *c, const char *realm, const char *nonce,
                        int proxy) {
    strcpy(c->branch, "z9hG4bK");
    gen_hex(c->branch + 7, 12);

    char auth[700] = "";
    if (realm && nonce && realm[0] && nonce[0] && c->srv->password[0]) {
        char ha1_in[512], ha2_in[320], resp_in[160];
        char ha1[33], ha2[33], resp[33];
        snprintf(ha1_in, sizeof(ha1_in), "%s:%s:%s",
                 c->srv->local_user, realm, c->srv->password);
        snprintf(ha2_in, sizeof(ha2_in), "INVITE:%s", c->target_uri);
        md5_hex(ha1_in, ha1);
        md5_hex(ha2_in, ha2);
        snprintf(resp_in, sizeof(resp_in), "%s:%s:%s", ha1, nonce, ha2);
        md5_hex(resp_in, resp);
        snprintf(auth, sizeof(auth),
            "%s: Digest username=\"%s\",realm=\"%s\",nonce=\"%s\","
            "uri=\"%s\",response=\"%s\",algorithm=MD5\r\n",
            proxy ? "Proxy-Authorization" : "Authorization",
            c->srv->local_user, realm, nonce, c->target_uri, resp);
    }

    char sdp[512];
    int sdp_len = build_sdp(c, sdp, sizeof(sdp), 1);   /* offer */

    int n = snprintf(c->req_buf, sizeof(c->req_buf),
        "INVITE %s SIP/2.0\r\n"
        "Via: SIP/2.0/UDP %s:%d;branch=%s;rport\r\n"
        "Max-Forwards: 70\r\n"
        "From: <sip:%s@%s>;tag=%s\r\n"
        "To: <%s>\r\n"
        "Call-ID: %s\r\n"
        "CSeq: %d INVITE\r\n"
        "Contact: <sip:%s@%s:%d>\r\n"
        "%s"
        "Content-Type: application/sdp\r\n"
        "Content-Length: %d\r\n"
        "\r\n"
        "%s",
        c->target_uri,
        c->srv->local_ip, c->srv->local_sip_port, c->branch,
        c->srv->local_user, c->srv->registrar_host, c->from_tag,
        c->target_uri, c->call_id, c->cseq_num,
        c->srv->local_user, c->srv->local_ip, c->srv->local_sip_port,
        auth, sdp_len, sdp);
    if (n >= (int)sizeof(c->req_buf)) n = (int)sizeof(c->req_buf) - 1;
    c->req_len = n;
    sip_send(c->srv, c->req_buf, n, &c->sip_from);
}

/* Send an ACK. to_hdr is the response's To line (carries the remote tag).
 * A 2xx ACK is its own transaction (new branch); a non-2xx ACK must reuse
 * the INVITE's branch. The ACK is retained in c->ok_buf so io_loop can
 * replay it if the far end retransmits its 2xx (RFC 3261 §13.2.2.4). */
static void send_ack(ctx_t *c, const char *to_hdr, int new_branch) {
    char branch[24];
    if (new_branch) { strcpy(branch, "z9hG4bK"); gen_hex(branch + 7, 12); }
    else { strncpy(branch, c->branch, sizeof(branch) - 1); branch[sizeof(branch)-1] = '\0'; }

    int n = snprintf(c->ok_buf, sizeof(c->ok_buf),
        "ACK %s SIP/2.0\r\n"
        "Via: SIP/2.0/UDP %s:%d;branch=%s;rport\r\n"
        "Max-Forwards: 70\r\n"
        "From: <sip:%s@%s>;tag=%s\r\n"
        "To: %s\r\n"
        "Call-ID: %s\r\n"
        "CSeq: %d ACK\r\n"
        "Content-Length: 0\r\n"
        "\r\n",
        c->target_uri,
        c->srv->local_ip, c->srv->local_sip_port, branch,
        c->srv->local_user, c->srv->registrar_host, c->from_tag,
        to_hdr, c->call_id, c->cseq_num);
    if (n >= (int)sizeof(c->ok_buf)) n = (int)sizeof(c->ok_buf) - 1;
    c->ok_buf_len = n;
    sip_send(c->srv, c->ok_buf, n, &c->sip_from);
}

/* CANCEL the in-progress INVITE (early dialog, before a final response).
 * Same branch / CSeq number as the INVITE; To carries no tag. */
static void send_cancel(ctx_t *c) {
    char buf[1024];
    int n = snprintf(buf, sizeof(buf),
        "CANCEL %s SIP/2.0\r\n"
        "Via: SIP/2.0/UDP %s:%d;branch=%s;rport\r\n"
        "Max-Forwards: 70\r\n"
        "From: <sip:%s@%s>;tag=%s\r\n"
        "To: <%s>\r\n"
        "Call-ID: %s\r\n"
        "CSeq: %d CANCEL\r\n"
        "Content-Length: 0\r\n"
        "\r\n",
        c->target_uri,
        c->srv->local_ip, c->srv->local_sip_port, c->branch,
        c->srv->local_user, c->srv->registrar_host, c->from_tag,
        c->target_uri, c->call_id, c->cseq_num);
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;  /* never send past buf */
    sip_send(c->srv, buf, n, &c->sip_from);
}

/* BYE an established dialog (after 200 OK). to_hdr is the answer's To line
 * (carries the remote tag); a fresh branch and the next CSeq are used. */
static void send_bye_uac(ctx_t *c, const char *to_hdr) {
    char branch[24];
    strcpy(branch, "z9hG4bK");
    gen_hex(branch + 7, 12);
    char buf[1024];
    int n = snprintf(buf, sizeof(buf),
        "BYE %s SIP/2.0\r\n"
        "Via: SIP/2.0/UDP %s:%d;branch=%s;rport\r\n"
        "Max-Forwards: 70\r\n"
        "From: <sip:%s@%s>;tag=%s\r\n"
        "To: %s\r\n"
        "Call-ID: %s\r\n"
        "CSeq: %d BYE\r\n"
        "Content-Length: 0\r\n"
        "\r\n",
        c->target_uri,
        c->srv->local_ip, c->srv->local_sip_port, branch,
        c->srv->local_user, c->srv->registrar_host, c->from_tag,
        to_hdr, c->call_id, ++c->cseq_num);
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;  /* never send past buf */
    sip_send(c->srv, buf, n, &c->sip_from);
}

static void *call_thread_uac(void *arg) {
    ctx_t *c = (ctx_t *)arg;
    int dispatch_fd = c->call->dispatch_pipe[0];

    /* The backend TCP socket is opened lazily — only once the far end signals
     * media (183 Session Progress / early media, or the 200 OK answer) — so we
     * don't tie up the data peer while the call is merely ringing. */
    if (rtp_open(c) < 0) { fprintf(stderr, "dial: RTP bind failed\n"); goto done; }

    c->cseq_num = 1;
    gen_hex(c->from_tag, 8);
    send_invite(c, NULL, NULL, 0);
    fprintf(stderr, "dial: INVITE -> %s\n", c->target_uri);

    /* INVITE client transaction: retransmit until a response (Timer A: T1
     * doubling to T2), give up at 32 s (Timer B); handle 401/407, await 200,
     * then ACK and bridge. */
    struct timespec next_tx, giveup;
    clock_gettime(CLOCK_MONOTONIC, &next_tx);
    giveup = next_tx;
    ts_add_ms(&next_tx, 500);
    ts_add_ms(&giveup, 32000);
    long rto = 500;
    int provisional = 0, established = 0, cancelling = 0;

    while (!established) {
        long wg = ts_until_ms(&giveup);
        if (wg <= 0) { fprintf(stderr, "dial: no answer (timeout)\n"); goto done; }
        long wt = provisional ? wg : ts_until_ms(&next_tx);
        long wms = wt < wg ? wt : wg;
        if (wms < 0) wms = 0;

        struct timeval tv;
        tv.tv_sec  = wms / 1000;
        tv.tv_usec = (wms % 1000) * 1000;
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(dispatch_fd, &rfds);
        int r = select(dispatch_fd + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) goto done;

        if (r > 0 && FD_ISSET(dispatch_fd, &rfds)) {
            char msg[4096];
            ssize_t rd = read(dispatch_fd, msg, sizeof(msg) - 1);
            if (rd <= 0) goto done;
            msg[rd] = '\0';

            int code = sip_response_code(msg);
            if (code == 0) continue;                 /* not an INVITE response */

            if (cancelling) {
                /* We've CANCELled; the INVITE still gets a final response that
                 * must be ACKed (otherwise the UAS retransmits it forever).
                 * Ignore the CANCEL's own 200 OK (CSeq method CANCEL). */
                char cseqm[64] = "";
                sip_hdr(msg, "CSeq", cseqm, sizeof(cseqm));
                if (!strstr(cseqm, "INVITE")) continue;
                char toh2[256] = "";
                sip_hdr(msg, "To", toh2, sizeof(toh2));
                if (code >= 300) {                   /* 487 Request Terminated */
                    send_ack(c, toh2, 0);            /* same branch as the INVITE */
                    fprintf(stderr, "dial: CANCEL confirmed (%d) — ACKed\n", code);
                    goto done;
                }
                if (code >= 200) {                   /* 2xx raced past the CANCEL */
                    send_ack(c, toh2, 1);
                    send_bye_uac(c, toh2);
                    fprintf(stderr, "dial: 2xx raced CANCEL — ACK+BYE\n");
                    goto done;
                }
                continue;                            /* 1xx: keep waiting */
            }

            if (code < 200) {                        /* 1xx provisional */
                provisional = 1;
                /* Early media — 183 Session Progress, or any provisional that
                 * carries an SDP answer — means voice is (about to be) flowing,
                 * so open the backend now. If that fails, CANCEL the INVITE. */
                const char *body = sip_body(msg);
                int early_media = (code == 183) || (body && strstr(body, "m=audio"));
                if (c->tcp_sock < 0 && early_media && !cancelling) {
                    if (tcp_connect(c) < 0) {
                        fprintf(stderr, "dial: backend %s:%d connect failed on "
                                "early media — CANCEL\n",
                                c->srv->tcp_host, c->srv->tcp_port);
                        send_cancel(c);
                        cancelling = 1;   /* now wait to ACK the 487 */
                    } else {
                        fprintf(stderr, "dial: early media — backend connected\n");
                    }
                }
                continue;
            }

            char toh[256] = "";
            sip_hdr(msg, "To", toh, sizeof(toh));

            if (code == 401 || code == 407) {
                send_ack(c, toh, 0);                 /* ACK the challenge */
                if (++c->auth_tries > 2) {
                    fprintf(stderr, "dial: authentication failed\n");
                    goto done;
                }
                char ah[600] = "";
                if (code == 407) sip_hdr(msg, "Proxy-Authenticate", ah, sizeof(ah));
                else             sip_hdr(msg, "WWW-Authenticate",   ah, sizeof(ah));
                char realm[256] = "", nonce[256] = "";
                parse_quoted(ah, "realm", realm, sizeof(realm));
                parse_quoted(ah, "nonce", nonce, sizeof(nonce));
                c->cseq_num++;
                send_invite(c, realm, nonce, code == 407);
                provisional = 0; rto = 500;
                clock_gettime(CLOCK_MONOTONIC, &next_tx);
                giveup = next_tx;
                ts_add_ms(&next_tx, 500);
                ts_add_ms(&giveup, 32000);
                continue;
            }
            if (code < 300) {                         /* 2xx: answered */
                const char *body = sip_body(msg);
                int acodec = CODEC_PCMA, artp_pt = CODEC_PCMA;
                if (!body || !sdp_parse(body, c->remote_rtp_ip,
                                        &c->remote_rtp_port,
                                        c->srv->allow_g722, c->srv->clearmode,
                                        &acodec, &artp_pt)) {
                    fprintf(stderr, "dial: 200 OK without usable SDP — BYE\n");
                    send_ack(c, toh, 1);
                    send_bye_uac(c, toh);
                    goto done;
                }
                c->codec = acodec;
                c->rtp_pt = artp_pt;
                codec_init(c);
                set_remote_rtp(c);
                send_ack(c, toh, 1);
                /* Open the backend now if early media hasn't already. If it
                 * fails, the call is up, so tear it down with a BYE. */
                if (c->tcp_sock < 0 && tcp_connect(c) < 0) {
                    fprintf(stderr, "dial: backend %s:%d connect failed after "
                            "answer — BYE\n", c->srv->tcp_host, c->srv->tcp_port);
                    send_bye_uac(c, toh);
                    goto done;
                }
                established = 1;
                fprintf(stderr, "dial: connected to %s (rtp %s:%d)\n",
                        c->target_uri, c->remote_rtp_ip, c->remote_rtp_port);
                break;
            }
            /* >= 300: final failure. ACK it (same branch) and stop. */
            send_ack(c, toh, 0);
            fprintf(stderr, "dial: call rejected (%d)\n", code);
            goto done;
        }

        if (!provisional && ts_until_ms(&next_tx) <= 0) {
            sip_send(c->srv, c->req_buf, c->req_len, &c->sip_from);
            rto *= 2;
            if (rto > 4000) rto = 4000;
            clock_gettime(CLOCK_MONOTONIC, &next_tx);
            ts_add_ms(&next_tx, rto);
        }
    }

    io_loop(c, dispatch_fd);

done:
    if (c->tcp_sock >= 0) close(c->tcp_sock);
    if (c->rtp_sock >= 0) close(c->rtp_sock);
    close(dispatch_fd);
    free(c);
    return NULL;
}

call_t *call_dial(server_t *srv) {
    call_t *call = calloc(1, sizeof(call_t));
    ctx_t  *c    = calloc(1, sizeof(ctx_t));
    if (!call || !c) { free(call); free(c); return NULL; }

    if (pipe(call->dispatch_pipe) < 0) { free(call); free(c); return NULL; }

    c->srv      = srv;
    c->call     = call;
    c->uac      = 1;
    c->rtp_sock = -1;
    c->tcp_sock = -1;
    c->sip_from = srv->proxy_addr;          /* send requests to the proxy */

    /* Normalize the target into a sip: URI. */
    const char *t = srv->dial_target;
    if (strncasecmp(t, "sip:", 4) == 0)
        snprintf(c->target_uri, sizeof(c->target_uri), "%s", t);
    else if (strchr(t, '@'))
        snprintf(c->target_uri, sizeof(c->target_uri), "sip:%s", t);
    else
        snprintf(c->target_uri, sizeof(c->target_uri), "sip:%s@%s",
                 t, srv->registrar_host);

    char tok[24];
    gen_hex(tok, 16);
    snprintf(call->call_id, sizeof(call->call_id), "%s@%s", tok, srv->local_ip);
    strncpy(c->call_id, call->call_id, sizeof(c->call_id) - 1);

    if (pthread_create(&call->thread, NULL, call_thread_uac, c) != 0) {
        close(call->dispatch_pipe[0]);
        close(call->dispatch_pipe[1]);
        free(call);
        free(c);
        return NULL;
    }

    pthread_mutex_lock(&srv->calls_mutex);
    call->next = srv->calls;
    srv->calls = call;
    pthread_mutex_unlock(&srv->calls_mutex);
    return call;
}

int call_dispatch(server_t *srv, const char *msg, int msglen) {
    char call_id[256] = "";
    sip_hdr(msg, "Call-ID", call_id, sizeof(call_id));
    if (!call_id[0]) return 0;

    pthread_mutex_lock(&srv->calls_mutex);
    call_t *c = srv->calls;
    while (c) {
        if (strcmp(c->call_id, call_id) == 0) {
            write(c->dispatch_pipe[1], msg, (size_t)msglen);
            pthread_mutex_unlock(&srv->calls_mutex);
            return 1;
        }
        c = c->next;
    }
    pthread_mutex_unlock(&srv->calls_mutex);
    return 0;
}

void call_reap(server_t *srv) {
    pthread_mutex_lock(&srv->calls_mutex);
    call_t **pp = &srv->calls;
    while (*pp) {
        call_t *c = *pp;
        if (pthread_tryjoin_np(c->thread, NULL) == 0) {
            close(c->dispatch_pipe[1]);
            *pp = c->next;
            free(c);
        } else {
            pp = &c->next;
        }
    }
    pthread_mutex_unlock(&srv->calls_mutex);
}
