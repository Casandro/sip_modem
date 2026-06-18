#ifndef CALL_H
#define CALL_H

#include <stdint.h>
#include <time.h>
#include <netinet/in.h>
#include <pthread.h>

typedef struct call {
    char              call_id[256];
    int               dispatch_pipe[2];   /* main→thread SIP text */
    pthread_t         thread;
    struct call      *next;
} call_t;

typedef struct server {
    int               sip_sock;
    char              local_ip[64];
    char              local_user[128];
    char              registrar_host[256];
    int               local_sip_port;
    int               reg_expires;
    time_t            reg_deadline;
    int               cseq;
    char              reg_callid[24];   /* random Call-ID token, stable across re-registrations */
    char              password[128];
    char              tcp_host[256];
    int               tcp_port;
    int               ignore_rtp_rx;  /* if set: never write RTP-derived bytes to TCP */
    int               max_calls;      /* concurrent-call cap; 0 = unlimited */
    int               stream_header;  /* -H: prefix audio conn with INVITE header (inbound) */
    int               allow_g722;     /* -g: offer/accept G.722 wideband (PT 9) */
    int               clearmode;      /* -T: exclusive CLEARMODE (RFC 4040) transparent 64k */

    /* Dial-out mode (-D): place one outbound call instead of registering. */
    int               dial_mode;      /* 1 = dial-out, 0 = register/answer */
    char              dial_target[256]; /* target as given on the command line */
    struct sockaddr_in proxy_addr;    /* where outbound requests are sent */

    /* Passive mode (-L): never register upstream; accept inbound REGISTER with
     * 200 OK and answer inbound INVITEs. */
    int               passive_mode;

    call_t           *calls;
    pthread_mutex_t   calls_mutex;

    /* REGISTER retransmission state (RFC 3261 §17.1.2 timers E/F) */
    char              reg_buf[2048];
    int               reg_buf_len;
    struct sockaddr_in reg_addr;
    struct timespec   reg_next_tx;
    struct timespec   reg_giveup;
    long              reg_rto_ms;
    int               reg_pending;

    /* 401/407 challenge backoff state */
    int               auth_fail_count;
    int               auth_retry_pending;
    struct timespec   auth_retry_at;
    char              auth_retry_realm[256];
    char              auth_retry_nonce[256];
} server_t;

/* Start a new call from a received INVITE message.
 * Returns the new call_t, or NULL if the INVITE is rejected immediately. */
call_t *call_start(server_t *srv, const char *msg, int msglen,
                   struct sockaddr_in *from);

/* Place an outbound call (UAC) to srv->dial_target via srv->proxy_addr,
 * bridging its audio to the TCP peer. Returns the new call_t, or NULL on
 * setup failure. The call runs in its own thread; responses are delivered
 * via call_dispatch() (matched by Call-ID), exactly like inbound dialogs. */
call_t *call_dial(server_t *srv);

/* Dispatch an in-dialog SIP message to the matching call by Call-ID.
 * Returns 1 if dispatched, 0 if no matching call. */
int call_dispatch(server_t *srv, const char *msg, int msglen);

/* Join and free any completed call threads. */
void call_reap(server_t *srv);

#endif
