#define _GNU_SOURCE
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "call.h"
#include "sip_util.h"
#include "alaw.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>

/* sip_interface drives the SIP side of the bridge. It runs in one of three
 * modes: register with an upstream registrar and answer inbound calls
 * (default), place one outbound call and exit (-D), or listen and answer
 * without registering (-L). Per-call SIP/RTP work lives in call.c; shared
 * parsing/auth/time helpers live in sip_util.c. */

/* ── Local IP detection ───────────────────────────────────────────── */

/* Discover the local source IP the kernel would use to reach `dest` by
 * connecting a throwaway UDP socket (no packets sent). Returns 0 on
 * success, -1 on any failure. */
static int get_local_ip(const char *dest, char *out, int outlen) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return -1;
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port   = htons(5060);
    if (inet_pton(AF_INET, dest, &a.sin_addr) != 1) { close(s); return -1; }
    if (connect(s, (struct sockaddr *)&a, sizeof(a)) < 0) { close(s); return -1; }
    socklen_t sl = sizeof(a);
    if (getsockname(s, (struct sockaddr *)&a, &sl) < 0) { close(s); return -1; }
    close(s);
    const char *ip = inet_ntoa(a.sin_addr);
    if (!ip) return -1;
    strncpy(out, ip, (size_t)outlen - 1);
    out[outlen - 1] = '\0';
    return 0;
}

/* ── REGISTER (outbound, to an upstream registrar) ────────────────── */

/* RFC 3261 §17.1.2 default timer values */
#define T1_MS    500
#define T2_MS    4000
#define TIMER_F  32000   /* 64 * T1: non-INVITE client transaction timeout */

/* Build a REGISTER into srv->reg_buf, send it, and arm the retransmission
 * timers. If realm/nonce are supplied (after a 401/407 challenge), include a
 * digest Authorization header. */
static void do_register(server_t *srv, struct sockaddr_in *reg_addr,
                        const char *realm, const char *nonce) {
    char branch[18] = "z9hG4bK";
    gen_hex(branch + 7, 10);
    char tag[12];
    gen_hex(tag, 10);

    char uri[300];
    snprintf(uri, sizeof(uri), "sip:%s", srv->registrar_host);

    char auth[512] = "";
    if (realm && nonce && srv->password[0]) {
        char ha1_in[512], ha2_in[300], resp_in[128];
        char ha1[33], ha2[33], resp[33];
        snprintf(ha1_in, sizeof(ha1_in), "%s:%s:%s",
                 srv->local_user, realm, srv->password);
        snprintf(ha2_in, sizeof(ha2_in), "REGISTER:%s", uri);
        md5_hex(ha1_in, ha1);
        md5_hex(ha2_in, ha2);
        snprintf(resp_in, sizeof(resp_in), "%s:%s:%s", ha1, nonce, ha2);
        md5_hex(resp_in, resp);
        snprintf(auth, sizeof(auth),
            "Authorization: Digest username=\"%s\",realm=\"%s\","
            "nonce=\"%s\",uri=\"%s\",response=\"%s\",algorithm=MD5\r\n",
            srv->local_user, realm, nonce, uri, resp);
    }

    int n = snprintf(srv->reg_buf, sizeof(srv->reg_buf),
        "REGISTER %s SIP/2.0\r\n"
        "Via: SIP/2.0/UDP %s:%d;branch=%s;rport\r\n"
        "From: <sip:%s@%s>;tag=%s\r\n"
        "To: <sip:%s@%s>\r\n"
        "Call-ID: reg-%s@%s\r\n"
        "CSeq: %d REGISTER\r\n"
        "Contact: <sip:%s@%s:%d>\r\n"
        "Max-Forwards: 70\r\n"
        "Expires: %d\r\n"
        "%s"
        "Content-Length: 0\r\n"
        "\r\n",
        uri,
        srv->local_ip, srv->local_sip_port, branch,
        srv->local_user, srv->registrar_host, tag,
        srv->local_user, srv->registrar_host,
        srv->reg_callid, srv->local_ip,
        srv->cseq++,
        srv->local_user, srv->local_ip, srv->local_sip_port,
        srv->reg_expires,
        auth);
    if (n >= (int)sizeof(srv->reg_buf)) n = (int)sizeof(srv->reg_buf) - 1;
    srv->reg_buf_len = n;
    srv->reg_addr    = *reg_addr;
    sendto(srv->sip_sock, srv->reg_buf, (size_t)n, 0,
           (struct sockaddr *)reg_addr, sizeof(*reg_addr));

    /* Arm retransmission timers (RFC 3261 §17.1.2 timer E, give-up timer F) */
    srv->reg_pending = 1;
    srv->reg_rto_ms  = T1_MS;
    clock_gettime(CLOCK_MONOTONIC, &srv->reg_next_tx);
    srv->reg_giveup = srv->reg_next_tx;
    ts_add_ms(&srv->reg_next_tx, T1_MS);
    ts_add_ms(&srv->reg_giveup,  TIMER_F);
}

static void retransmit_register(server_t *srv) {
    sendto(srv->sip_sock, srv->reg_buf, (size_t)srv->reg_buf_len, 0,
           (struct sockaddr *)&srv->reg_addr, sizeof(srv->reg_addr));
    srv->reg_rto_ms *= 2;
    if (srv->reg_rto_ms > T2_MS) srv->reg_rto_ms = T2_MS;
    clock_gettime(CLOCK_MONOTONIC, &srv->reg_next_tx);
    ts_add_ms(&srv->reg_next_tx, srv->reg_rto_ms);
}

/* ── Request routing (shared by passive and answer modes) ─────────── */

/* Route a received SIP request: deliver an in-dialog message (ACK, BYE, a
 * retransmitted INVITE) to its call by Call-ID, or start a new call for a
 * brand-new INVITE. REGISTER is handled by the caller, not here. */
static void route_request(server_t *srv, const char *msg, int len,
                          struct sockaddr_in *from) {
    char method[32];
    sip_method(msg, method, sizeof(method));
    if (strcasecmp(method, "INVITE") == 0) {
        if (!call_dispatch(srv, msg, len))
            call_start(srv, msg, len, from);
    } else {
        call_dispatch(srv, msg, len);
    }
}

/* ── Dial-out mode (-D) ───────────────────────────────────────────── */

/* Place one outbound call and pump SIP responses to the call thread until
 * it finishes, then return. No registration is performed. Returns 0 once
 * the call has ended (or failed). */
static int run_dialout(server_t *srv) {
    if (!call_dial(srv)) {
        fprintf(stderr, "dial-out: failed to start call\n");
        return 1;
    }
    for (;;) {
        pthread_mutex_lock(&srv->calls_mutex);
        int active = srv->calls != NULL;
        pthread_mutex_unlock(&srv->calls_mutex);
        if (!active) break;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(srv->sip_sock, &rfds);
        struct timeval tv = {0, 200000};   /* 200 ms reap/poll cadence */
        int r = select(srv->sip_sock + 1, &rfds, NULL, NULL, &tv);
        if (r > 0 && FD_ISSET(srv->sip_sock, &rfds)) {
            char msg[8192];
            struct sockaddr_in from;
            socklen_t sl = sizeof(from);
            ssize_t len = recvfrom(srv->sip_sock, msg, sizeof(msg) - 1, 0,
                                   (struct sockaddr *)&from, &sl);
            if (len > 0) { msg[len] = '\0'; call_dispatch(srv, msg, (int)len); }
        }
        call_reap(srv);
    }
    fprintf(stderr, "dial-out: call ended\n");
    return 0;
}

/* ── Passive mode (-L) ────────────────────────────────────────────── */

/* Answer an inbound REGISTER with 200 OK (accept-all registrar). No password
 * check and no binding store: we just acknowledge so a UA/proxy considers
 * itself registered against us. Echoes Via/From/Call-ID/CSeq/Contact, copies
 * To (adding a tag if it lacks one), and reflects the requested Expires. */
static void respond_register_ok(server_t *srv, const char *msg,
                                struct sockaddr_in *from) {
    char via[512] = "", f[256] = "", to[256] = "", callid[256] = "";
    char cseq[64] = "", contact[256] = "", exp_hdr[64] = "";
    sip_hdr(msg, "Via",     via,     sizeof(via));
    sip_hdr(msg, "From",    f,       sizeof(f));
    sip_hdr(msg, "To",      to,      sizeof(to));
    sip_hdr(msg, "Call-ID", callid,  sizeof(callid));
    sip_hdr(msg, "CSeq",    cseq,    sizeof(cseq));
    sip_hdr(msg, "Contact", contact, sizeof(contact));
    sip_hdr(msg, "Expires", exp_hdr, sizeof(exp_hdr));
    int expires = exp_hdr[0] ? atoi(exp_hdr) : srv->reg_expires;

    /* Add a To-tag if the request did not carry one (RFC 3261 §8.2.6.2). */
    char to_with_tag[300];
    if (strstr(to, ";tag=")) {
        snprintf(to_with_tag, sizeof(to_with_tag), "%s", to);
    } else {
        char tag[18];
        gen_hex(tag, 16);
        snprintf(to_with_tag, sizeof(to_with_tag), "%s;tag=%s", to, tag);
    }

    char resp[2048];
    int n = snprintf(resp, sizeof(resp),
        "SIP/2.0 200 OK\r\n"
        "Via: %s\r\n"
        "From: %s\r\n"
        "To: %s\r\n"
        "Call-ID: %s\r\n"
        "CSeq: %s\r\n"
        "%s%s%s"
        "Expires: %d\r\n"
        "Content-Length: 0\r\n"
        "\r\n",
        via, f, to_with_tag, callid, cseq,
        contact[0] ? "Contact: " : "", contact[0] ? contact : "",
        contact[0] ? "\r\n" : "",
        expires);
    /* snprintf returns the would-be length; never hand a value past the buffer
     * to sendto, or it over-reads the stack and leaks it to the sender. */
    if (n >= (int)sizeof(resp)) n = (int)sizeof(resp) - 1;
    if (n > 0)
        sendto(srv->sip_sock, resp, (size_t)n, 0,
               (struct sockaddr *)from, sizeof(*from));
}

/* Listen on the SIP port without registering upstream. Accept inbound REGISTER
 * with 200 OK and answer inbound INVITEs, bridging their audio to the TCP peer
 * exactly like answer mode. Runs forever. */
static int run_passive(server_t *srv) {
    fprintf(stderr, "Passive mode: listening on port %d (no registration)\n",
            srv->local_sip_port);
    for (;;) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(srv->sip_sock, &rfds);
        struct timeval tv = {0, 200000};   /* 200 ms reap/poll cadence */
        int r = select(srv->sip_sock + 1, &rfds, NULL, NULL, &tv);
        if (r > 0 && FD_ISSET(srv->sip_sock, &rfds)) {
            char msg[8192];
            struct sockaddr_in from;
            socklen_t sl = sizeof(from);
            ssize_t len = recvfrom(srv->sip_sock, msg, sizeof(msg) - 1, 0,
                                   (struct sockaddr *)&from, &sl);
            if (len > 0) {
                msg[len] = '\0';
                if (sip_response_code(msg) <= 0) {   /* a request, not a response */
                    char method[32];
                    sip_method(msg, method, sizeof(method));
                    if (strcasecmp(method, "REGISTER") == 0)
                        respond_register_ok(srv, msg, &from);
                    else
                        route_request(srv, msg, (int)len, &from);
                }
                /* responses are ignored in passive mode */
            }
        }
        call_reap(srv);
    }
    return 0;
}

/* ── Register/answer mode (default) ───────────────────────────────── */

/* Handle a SIP response while registering. Any response to our pending
 * REGISTER stops its retransmission; a 401/407 drives the digest handshake
 * (with exponential backoff once challenges repeat), and a 2xx records the
 * granted lifetime. realm/nonce hold the latest challenge for reuse on renewal;
 * *registered tracks whether we currently hold a registration. */
static void handle_register_response(server_t *srv, const char *msg, int code,
                                     struct sockaddr_in *reg_addr,
                                     char *realm, char *nonce, int *registered) {
    char cseq[64] = "";
    sip_hdr(msg, "CSeq", cseq, sizeof(cseq));
    int is_register_resp = (strstr(cseq, "REGISTER") != NULL);
    if (is_register_resp) srv->reg_pending = 0;

    if (code == 401 || code == 407) {
        char auth_hdr[512] = "";
        if (!sip_hdr(msg, "WWW-Authenticate", auth_hdr, sizeof(auth_hdr)))
            sip_hdr(msg, "Proxy-Authenticate", auth_hdr, sizeof(auth_hdr));
        parse_quoted(auth_hdr, "realm", realm, 256);
        parse_quoted(auth_hdr, "nonce", nonce, 256);
        if (!is_register_resp) return;

        srv->auth_fail_count++;
        if (srv->auth_fail_count <= 1) {
            /* First challenge in a row — the expected handshake. */
            do_register(srv, reg_addr, realm, nonce);
        } else {
            /* Repeated challenges: back off. 1s, 2s, 4s … 60s cap. */
            long backoff_ms = 1000L << (srv->auth_fail_count - 2);
            if (backoff_ms > 60000L || backoff_ms <= 0) backoff_ms = 60000L;
            clock_gettime(CLOCK_MONOTONIC, &srv->auth_retry_at);
            ts_add_ms(&srv->auth_retry_at, backoff_ms);
            strncpy(srv->auth_retry_realm, realm, sizeof(srv->auth_retry_realm) - 1);
            srv->auth_retry_realm[sizeof(srv->auth_retry_realm) - 1] = '\0';
            strncpy(srv->auth_retry_nonce, nonce, sizeof(srv->auth_retry_nonce) - 1);
            srv->auth_retry_nonce[sizeof(srv->auth_retry_nonce) - 1] = '\0';
            srv->auth_retry_pending = 1;
            fprintf(stderr, "REGISTER auth failed %d times — backing off %ldms\n",
                    srv->auth_fail_count, backoff_ms);
        }
    } else if (code >= 200 && code < 300 && is_register_resp) {
        *registered = 1;
        srv->auth_fail_count = 0;
        srv->auth_retry_pending = 0;
        char exp_hdr[64] = "";
        sip_hdr(msg, "Expires", exp_hdr, sizeof(exp_hdr));
        int exp = exp_hdr[0] ? atoi(exp_hdr) : srv->reg_expires;
        srv->reg_deadline = time(NULL) + exp;
        fprintf(stderr, "Registered. Expires in %ds.\n", exp);
    }
}

/* Register with the upstream registrar, keep the registration fresh, and
 * answer inbound calls. Runs forever. */
static int run_register_answer(server_t *srv, struct sockaddr_in *reg_addr) {
    do_register(srv, reg_addr, NULL, NULL);

    char realm[256] = "", nonce[256] = "";
    int registered = 0;
    srv->reg_deadline = time(NULL) + srv->reg_expires;

    for (;;) {
        time_t now = time(NULL);
        /* Re-register before expiry. 60s margin for long registrations, but
         * never more than half the lifetime for short ones (e.g. -e 60), or
         * the renewal instant would be in the past and busy-loop. */
        long margin = srv->reg_expires > 120 ? 60 : srv->reg_expires / 2;
        if (margin < 1) margin = 1;
        time_t renew_at = srv->reg_deadline - margin;
        long wait_ms = (renew_at - now) * 1000L;
        if (wait_ms < 1) wait_ms = 1;

        /* Shrink the wait to whichever transaction/backoff timer fires first. */
        if (srv->reg_pending) {
            long w_tx = ts_until_ms(&srv->reg_next_tx);
            long w_gu = ts_until_ms(&srv->reg_giveup);
            if (w_tx < 0) w_tx = 0;
            if (w_gu < 0) w_gu = 0;
            if (w_tx < wait_ms) wait_ms = w_tx;
            if (w_gu < wait_ms) wait_ms = w_gu;
        }
        if (srv->auth_retry_pending) {
            long w_ar = ts_until_ms(&srv->auth_retry_at);
            if (w_ar < 0) w_ar = 0;
            if (w_ar < wait_ms) wait_ms = w_ar;
        }

        struct timeval tv;
        tv.tv_sec  = wait_ms / 1000;
        tv.tv_usec = (wait_ms % 1000) * 1000;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(srv->sip_sock, &rfds);
        int r = select(srv->sip_sock + 1, &rfds, NULL, NULL, &tv);

        if (r == 0) {
            /* A timer fired — figure out which and service it. */
            if (srv->auth_retry_pending && ts_until_ms(&srv->auth_retry_at) <= 0) {
                /* Backoff elapsed — retry REGISTER with stored creds. */
                srv->auth_retry_pending = 0;
                do_register(srv, reg_addr, srv->auth_retry_realm,
                            srv->auth_retry_nonce);
            } else if (srv->reg_pending && ts_until_ms(&srv->reg_giveup) <= 0) {
                /* Timer F: 32s without a response. Drop the transaction and
                 * start a fresh REGISTER (re-using cached creds if any). */
                fprintf(stderr, "REGISTER timed out — restarting transaction\n");
                srv->reg_pending = 0;
                do_register(srv, reg_addr, registered ? realm : NULL,
                            registered ? nonce : NULL);
            } else if (srv->reg_pending && ts_until_ms(&srv->reg_next_tx) <= 0) {
                /* Timer E: retransmit the pending REGISTER. */
                retransmit_register(srv);
            } else if (!srv->reg_pending && time(NULL) >= renew_at) {
                /* Renewal: only when the instant has actually arrived and no
                 * REGISTER is already in flight. A clamped short wake-up must
                 * not trigger a spurious registration. */
                do_register(srv, reg_addr, registered ? realm : NULL,
                            registered ? nonce : NULL);
            }
            continue;
        }

        if (!FD_ISSET(srv->sip_sock, &rfds)) { call_reap(srv); continue; }

        char msg[8192];
        struct sockaddr_in from;
        socklen_t sl = sizeof(from);
        ssize_t len = recvfrom(srv->sip_sock, msg, sizeof(msg) - 1, 0,
                               (struct sockaddr *)&from, &sl);
        if (len <= 0) continue;
        msg[len] = '\0';

        int code = sip_response_code(msg);
        if (code > 0)
            handle_register_response(srv, msg, code, reg_addr,
                                     realm, nonce, &registered);
        else
            route_request(srv, msg, (int)len, &from);

        call_reap(srv);
    }
    return 0;
}

/* ── Startup ──────────────────────────────────────────────────────── */

static void usage(const char *argv0) {
    fprintf(stderr,
        "Usage: %s -u sip:user@registrar -p password -c host:port [-P sip_port]\n"
        "          [-e expires] [-i] [-m N] [-D target] [-L]\n"
        "  -u  SIP AoR (sip:user@host)\n"
        "  -p  SIP password (or set $SIP_PASSWORD to keep it out of `ps aux`)\n"
        "  -P  Local SIP port (default 5060)\n"
        "  -e  Registration expires seconds (default 120)\n"
        "  -c  TCP host:port to bridge audio with (s16le, 8 kHz)\n"
        "  -i  Ignore incoming RTP audio (no RTP-derived bytes sent down TCP)\n"
        "  -m  Max concurrent calls (default 0 = unlimited)\n"
        "  -H  Prefix the audio connection with a framed header carrying the\n"
        "      received INVITE before the PCM (inbound calls only). See README\n"
        "  -g  Offer/accept G.722 wideband (PT 9, preferred); the TCP audio is\n"
        "      then 16 kHz s16le. Default off (PCMA/8 kHz only)\n"
        "  -T  CLEARMODE (RFC 4040): exclusively negotiate a transparent 64 kbit/s\n"
        "      channel — the TCP side carries raw octets, not PCM. Overrides -g;\n"
        "      not for the FSK modem. Default off\n"
        "  -D  Dial-out: place one outbound call to target (a sip: URI, user@host,\n"
        "      or bare user/number using the -u registrar domain) instead of\n"
        "      registering. Bridges the call, then exits when it ends or fails\n"
        "  -L  Passive mode: never register upstream. Listen on the SIP port,\n"
        "      answer inbound REGISTER with 200 OK (accept-all), and answer\n"
        "      inbound INVITEs as usual. -u still supplies our identity\n",
        argv0);
    exit(1);
}

/* Parse argv into srv (exits via usage() on any malformed/missing option). */
static void parse_args(int argc, char **argv, server_t *srv) {
    int opt;
    while ((opt = getopt(argc, argv, "u:p:P:e:im:c:D:HgTL")) != -1) {
        switch (opt) {
        case 'u': {
            const char *aor = optarg;          /* sip:user@host */
            const char *at = strchr(aor, '@');
            const char *colon = strchr(aor, ':');
            if (!at || !colon) usage(argv[0]);
            int ulen = (int)(at - colon - 1);
            if (ulen > 0 && ulen < 128) {
                memcpy(srv->local_user, colon + 1, (size_t)ulen);
                srv->local_user[ulen] = '\0';
            }
            strncpy(srv->registrar_host, at + 1, sizeof(srv->registrar_host) - 1);
            break;
        }
        case 'p': strncpy(srv->password, optarg, sizeof(srv->password) - 1); break;
        case 'P': srv->local_sip_port = atoi(optarg); break;
        case 'e': srv->reg_expires = atoi(optarg); break;
        case 'i': srv->ignore_rtp_rx = 1; break;
        case 'H': srv->stream_header = 1; break;
        case 'g': srv->allow_g722 = 1; break;
        case 'T': srv->clearmode = 1; break;
        case 'L': srv->passive_mode = 1; break;
        case 'm': {
            int m = atoi(optarg);
            if (m < 0) { fprintf(stderr, "bad -m value (must be >= 0)\n"); exit(1); }
            srv->max_calls = m;
            break;
        }
        case 'D':
            strncpy(srv->dial_target, optarg, sizeof(srv->dial_target) - 1);
            srv->dial_mode = 1;
            break;
        case 'c': {
            const char *hp = optarg;
            const char *col = strrchr(hp, ':');
            if (!col) usage(argv[0]);
            int hlen = (int)(col - hp);
            if (hlen <= 0 || hlen >= (int)sizeof(srv->tcp_host)) usage(argv[0]);
            memcpy(srv->tcp_host, hp, (size_t)hlen);
            srv->tcp_host[hlen] = '\0';
            srv->tcp_port = atoi(col + 1);
            if (srv->tcp_port <= 0 || srv->tcp_port > 65535) usage(argv[0]);
            break;
        }
        default:
            usage(argv[0]);
        }
    }

    /* If no -p was given, fall back to the SIP_PASSWORD environment variable.
     * Preferred over -p in practice: the env var keeps the secret out of the
     * argv that any user can read via `ps aux` / /proc/<pid>/cmdline. */
    if (!srv->password[0]) {
        const char *env = getenv("SIP_PASSWORD");
        if (env) {
            strncpy(srv->password, env, sizeof(srv->password) - 1);
            srv->password[sizeof(srv->password) - 1] = '\0';
        }
    }

    if (!srv->local_user[0] || !srv->registrar_host[0] ||
        !srv->tcp_host[0] || srv->tcp_port == 0)
        usage(argv[0]);
}

/* Resolve the registrar, detect our local IP, pick a REGISTER Call-ID, and
 * bind the SIP socket. Fills *reg_addr. Returns 0 on success, -1 on failure. */
static int server_setup(server_t *srv, struct sockaddr_in *reg_addr) {
    /* Resolve to IPv4: the REGISTER/Via/Contact/SDP path is IPv4-only and the
     * registrar address is used as a sockaddr_in throughout. getaddrinfo fills
     * in family/port/addr (here, port 5060). */
    memset(reg_addr, 0, sizeof(*reg_addr));
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    int gai = getaddrinfo(srv->registrar_host, "5060", &hints, &res);
    if (gai != 0 || !res) {
        fprintf(stderr, "Cannot resolve %s: %s\n",
                srv->registrar_host, gai_strerror(gai));
        return -1;
    }
    *reg_addr = *(struct sockaddr_in *)res->ai_addr;
    freeaddrinfo(res);

    if (get_local_ip(inet_ntoa(reg_addr->sin_addr),
                     srv->local_ip, sizeof(srv->local_ip)) < 0) {
        fprintf(stderr, "Cannot determine local IP for %s\n", srv->registrar_host);
        return -1;
    }

    /* Random Call-ID for REGISTER (stable across re-registrations, unique per run) */
    gen_hex(srv->reg_callid, sizeof(srv->reg_callid) - 1);

    srv->sip_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family      = AF_INET;
    local.sin_addr.s_addr = INADDR_ANY;
    local.sin_port        = htons((uint16_t)srv->local_sip_port);
    if (bind(srv->sip_sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        perror("bind SIP"); return -1;
    }
    return 0;
}

int main(int argc, char **argv) {
    /* Ignore SIGPIPE so a write to a closed pipe (dead child stdin or a
     * dispatch pipe whose reader just exited) returns EPIPE instead of
     * killing the whole process. */
    signal(SIGPIPE, SIG_IGN);

    server_t srv;
    memset(&srv, 0, sizeof(srv));
    srv.local_sip_port = 5060;
    srv.reg_expires    = 120;
    srv.cseq           = 1;
    pthread_mutex_init(&srv.calls_mutex, NULL);

    parse_args(argc, argv, &srv);

    struct sockaddr_in reg_addr;
    if (server_setup(&srv, &reg_addr) < 0) return 1;

    alaw_init();

    /* Dial-out mode: place one outbound call to the proxy (the -u registrar)
     * and exit when it ends. No registration. */
    if (srv.dial_mode) {
        srv.proxy_addr = reg_addr;
        return run_dialout(&srv);
    }

    /* Passive mode: listen and answer, but never register upstream. */
    if (srv.passive_mode)
        return run_passive(&srv);

    /* Default: register upstream and answer inbound calls. */
    return run_register_answer(&srv, &reg_addr);
}
