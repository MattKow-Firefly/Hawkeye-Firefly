#include "net_mp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <ws2tcpip.h>
#include <process.h>
#define SOCK_CLOSE(s) closesocket(s)
#else
#include <unistd.h>
#include <fcntl.h>
#include <pwd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define SOCK_CLOSE(s) close(s)
#endif

#define MP_MAGIC          0x484B4D50u  // 'H''K''M''P'
#define MP_VERSION        3            // bumped: added laser/hit/lost messages
#define MP_MSG_STATE      2            // periodic position beacon
#define MP_MSG_LASER      3            // a fired laser (origin + direction)
#define MP_MSG_HIT        4            // "my laser hit you" (target_session)
#define MP_MSG_LOST       5            // "I lost" (sender reached 0 health)
// Adaptive broadcast rate: a slow discovery beacon when no peers are present,
// ramping to full rate (frame-capped, ~60 Hz) once someone else is online.
#define MP_BEACON_INTERVAL 1.0          // ~1 Hz discovery broadcast (always)
#define MP_ACTIVE_INTERVAL (1.0/120.0)  // up to ~60 Hz unicast to peers (frame-capped)
#define MP_PEER_TIMEOUT    3.0          // drop peers silent this long

// Wire packet — fixed layout, sent raw. Mirrors the hil_state_t fields the
// receiver needs to place and orient the remote drone.
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t session_id;
    uint8_t  sysid;
    uint8_t  mav_type;
    uint8_t  valid;        // 1 = position fields are meaningful, 0 = beacon only
    uint8_t  _pad;
    char     name[MP_NAME_LEN];
    int32_t  lat;          // degE7
    int32_t  lon;          // degE7
    int32_t  alt;          // mm (AMSL)
    float    quat[4];      // w, x, y, z
    int16_t  vx, vy, vz;   // cm/s, NED
    uint16_t ind_airspeed; // cm/s
    uint16_t true_airspeed;// cm/s
    uint64_t time_usec;
    int32_t  score;
    int32_t  health;
    // Event payloads (meaningful per message type; zero for MP_MSG_STATE):
    float    fire_origin[3];  // MP_MSG_LASER: world-space spawn point
    float    fire_dir[3];     // MP_MSG_LASER: unit direction
    uint32_t target_session;  // MP_MSG_HIT: which peer was hit
} mp_packet_t;
#pragma pack(pop)

static int set_nonblocking(sock_t s) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(s, FIONBIO, &mode);
#else
    int flags = fcntl(s, F_GETFL, 0);
    return fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
}

// A best-effort unique id for this process/run — enough to distinguish peers
// on a LAN and to filter out our own looped-back broadcasts.
static uint32_t make_session_id(void) {
#ifdef _WIN32
    return ((uint32_t)_getpid() << 16) ^ (uint32_t)GetTickCount();
#else
    return ((uint32_t)getpid() << 16) ^ (uint32_t)time(NULL);
#endif
}

// The current user's login name, for use as the default player name.
static const char *login_name(void) {
    const char *n;
#ifdef _WIN32
    n = getenv("USERNAME");
#else
    n = getenv("USER");
    if (!n || !n[0]) n = getenv("LOGNAME");
    if (!n || !n[0]) {
        struct passwd *pw = getpwuid(getuid());
        if (pw) n = pw->pw_name;
    }
#endif
    return (n && n[0]) ? n : "hawkeye";
}

int mp_init(mp_t *mp, uint16_t port, uint8_t self_sysid, const char *name) {
    memset(mp, 0, sizeof(*mp));
    mp->port = port;
    mp->self_sysid = self_sysid;
    mp->session_id = make_session_id();
    mp->sock = SOCK_INVALID;
    snprintf(mp->self_name, sizeof(mp->self_name), "%s",
             (name && name[0]) ? name : login_name());

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "[mp] WSAStartup failed: %d\n", WSAGetLastError());
        return -1;
    }
#endif

    mp->sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (mp->sock == SOCK_INVALID) {
        perror("[mp] socket");
        return -1;
    }

    // Allow multiple instances on one host (handy for testing) and let us
    // send to the subnet broadcast address.
    int one = 1;
    setsockopt(mp->sock, SOL_SOCKET, SO_REUSEADDR, (char *)&one, sizeof(one));
    if (setsockopt(mp->sock, SOL_SOCKET, SO_BROADCAST, (char *)&one, sizeof(one)) < 0) {
        perror("[mp] SO_BROADCAST");
    }
    set_nonblocking(mp->sock);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(mp->sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[mp] bind");
        SOCK_CLOSE(mp->sock);
        mp->sock = SOCK_INVALID;
        return -1;
    }

    return 0;
}

void mp_set_local(mp_t *mp, const hil_state_t *state, uint8_t sysid,
                  uint8_t mav_type, int32_t score, int32_t health) {
    mp->local_state = *state;
    if (sysid) mp->self_sysid = sysid;
    mp->self_mav_type = mav_type;
    mp->self_score = score;
    mp->self_health = health;
}

// Fill our current drone state/score/health into a STATE packet.
static void mp_fill_state(mp_t *mp, mp_packet_t *pkt) {
    memset(pkt, 0, sizeof(*pkt));
    pkt->magic      = MP_MAGIC;
    pkt->version    = MP_VERSION;
    pkt->type       = MP_MSG_STATE;
    pkt->session_id = mp->session_id;
    pkt->sysid      = mp->self_sysid;
    pkt->mav_type   = mp->self_mav_type;
    pkt->valid      = mp->local_state.valid ? 1 : 0;
    memcpy(pkt->name, mp->self_name, MP_NAME_LEN);

    pkt->lat           = mp->local_state.lat;
    pkt->lon           = mp->local_state.lon;
    pkt->alt           = mp->local_state.alt;
    memcpy(pkt->quat, mp->local_state.quaternion, sizeof(pkt->quat));
    pkt->vx            = mp->local_state.vx;
    pkt->vy            = mp->local_state.vy;
    pkt->vz            = mp->local_state.vz;
    pkt->ind_airspeed  = mp->local_state.ind_airspeed;
    pkt->true_airspeed = mp->local_state.true_airspeed;
    pkt->time_usec     = mp->local_state.time_usec;
    pkt->score         = mp->self_score;
    pkt->health        = mp->self_health;
}

// Send a packet to the subnet broadcast address (discovery / fallback).
static void mp_send_broadcast(mp_t *mp, const mp_packet_t *pkt) {
    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family      = AF_INET;
    dst.sin_port        = htons(mp->port);
    dst.sin_addr.s_addr = htonl(INADDR_BROADCAST);  // 255.255.255.255
    sendto(mp->sock, (const char *)pkt, sizeof(*pkt), 0,
           (struct sockaddr *)&dst, sizeof(dst));
}

// Unicast a packet to every peer whose address we know. On Wi-Fi, unicast
// frames get 802.11 link-layer ACK + retransmit, so this is far more reliable
// than broadcast on a weak connection. Returns the number of peers sent to.
static int mp_send_peers(mp_t *mp, const mp_packet_t *pkt) {
    int sent = 0;
    for (int i = 0; i < MP_MAX_PEERS; i++) {
        mp_peer_t *pe = &mp->peers[i];
        if (!pe->used || !pe->addr_valid) continue;
        struct sockaddr_in dst;
        memcpy(&dst, pe->addr, sizeof(dst));  // copy out (alignment-safe)
        sendto(mp->sock, (const char *)pkt, sizeof(*pkt), 0,
               (struct sockaddr *)&dst, sizeof(dst));
        sent++;
    }
    return sent;
}

// Find the peer slot for a session id, or claim a free one. Returns NULL if full.
static mp_peer_t *mp_peer_for(mp_t *mp, uint32_t session_id) {
    for (int i = 0; i < MP_MAX_PEERS; i++)
        if (mp->peers[i].used && mp->peers[i].session_id == session_id)
            return &mp->peers[i];
    for (int i = 0; i < MP_MAX_PEERS; i++) {
        if (!mp->peers[i].used) {
            memset(&mp->peers[i], 0, sizeof(mp->peers[i]));
            mp->peers[i].used = true;
            mp->peers[i].session_id = session_id;
            return &mp->peers[i];
        }
    }
    return NULL;
}

static void mp_recv(mp_t *mp, double now) {
    mp_packet_t pkt;
    struct sockaddr_in src;
    socklen_t srclen = sizeof(src);

    for (;;) {
        int n = recvfrom(mp->sock, (char *)&pkt, sizeof(pkt), 0,
                         (struct sockaddr *)&src, &srclen);
        if (n <= 0) break;
        if (n < (int)sizeof(pkt)) continue;               // runt / foreign packet
        if (pkt.magic != MP_MAGIC || pkt.version != MP_VERSION) continue;
        if (pkt.session_id == mp->session_id) continue;    // our own broadcast

        switch (pkt.type) {
        case MP_MSG_LASER:
            // A peer's shot — buffer it for the caller to spawn visually.
            if (mp->rx_fire_count < MP_MAX_EVENTS) {
                mp_fire_event_t *e = &mp->rx_fires[mp->rx_fire_count++];
                memcpy(e->origin, pkt.fire_origin, sizeof(e->origin));
                memcpy(e->dir, pkt.fire_dir, sizeof(e->dir));
            }
            continue;

        case MP_MSG_HIT:
            if (pkt.target_session == mp->session_id) mp->rx_hits++;
            continue;

        case MP_MSG_LOST:
            mp->rx_lost = true;
            continue;

        case MP_MSG_STATE:
            break;      // fall through to the state handling below

        default:
            continue;
        }

        mp_peer_t *pe = mp_peer_for(mp, pkt.session_id);
        if (!pe) continue;                                 // peer table full

        bool is_new = (pe->last_seen == 0.0);
        pe->sysid    = pkt.sysid;
        pe->mav_type = pkt.mav_type;
        memcpy(pe->name, pkt.name, MP_NAME_LEN);
        pe->name[MP_NAME_LEN - 1] = '\0';
        pe->last_seen = now;

        // Learn this peer's address so we can unicast to them.
        memcpy(pe->addr, &src, sizeof(struct sockaddr_in));
        pe->addr_valid = true;

        pe->state.lat = pkt.lat;
        pe->state.lon = pkt.lon;
        pe->state.alt = pkt.alt;
        memcpy(pe->state.quaternion, pkt.quat, sizeof(pkt.quat));
        pe->state.vx = pkt.vx;
        pe->state.vy = pkt.vy;
        pe->state.vz = pkt.vz;
        pe->state.ind_airspeed  = pkt.ind_airspeed;
        pe->state.true_airspeed = pkt.true_airspeed;
        pe->state.time_usec     = pkt.time_usec;
        pe->state.valid         = pkt.valid ? true : false;
        pe->score               = pkt.score;
        pe->health              = pkt.health;

        // Receive-rate accounting.
        if (pe->rx_t0 == 0.0) pe->rx_t0 = now;
        pe->rx_count++;

        if (is_new)
            printf("[mp] peer joined: %s (sysid %u, session %08x)\n",
                   pe->name, pe->sysid, pe->session_id);
    }
}

void mp_poll(mp_t *mp, double now) {
    if (mp->sock == SOCK_INVALID) return;

    // Reset per-poll received-event accumulators before draining the socket.
    mp->rx_fire_count = 0;
    mp->rx_hits = 0;
    mp->rx_lost = false;

    // Receive first, then time out stale peers, so the live peer count below
    // reflects anyone discovered (or lost) this frame before we pick our rate.
    mp_recv(mp, now);

    int peers = 0;
    for (int i = 0; i < MP_MAX_PEERS; i++) {
        if (!mp->peers[i].used) continue;
        if (now - mp->peers[i].last_seen > MP_PEER_TIMEOUT) {
            printf("[mp] peer left: %s (session %08x)\n",
                   mp->peers[i].name, mp->peers[i].session_id);
            mp->peers[i].used = false;
        } else {
            peers++;
            // Refresh this peer's receive-rate once per ~1s window.
            mp_peer_t *pe = &mp->peers[i];
            if (pe->rx_t0 > 0.0 && now - pe->rx_t0 >= 1.0) {
                pe->rx_hz = (float)(pe->rx_count / (now - pe->rx_t0));
                pe->rx_count = 0;
                pe->rx_t0 = now;
            }
        }
    }

    // Refresh our send-rate once per ~1s window.
    if (mp->tx_t0 == 0.0) mp->tx_t0 = now;
    if (now - mp->tx_t0 >= 1.0) {
        mp->tx_hz = (float)(mp->tx_count / (now - mp->tx_t0));
        mp->tx_count = 0;
        mp->tx_t0 = now;
    }

    // Low-rate discovery beacon (broadcast) so newly-joined peers can find us,
    // even once we're already unicasting to others.
    if (now - mp->last_beacon >= MP_BEACON_INTERVAL) {
        mp_packet_t pkt;
        mp_fill_state(mp, &pkt);
        mp_send_broadcast(mp, &pkt);
        mp->last_beacon = now;
        if (peers == 0) mp->tx_count++;   // when alone, the beacon is our send rate
    }

    // High-rate state to each known peer via unicast (one copy per peer).
    if (peers > 0 && now - mp->last_send >= MP_ACTIVE_INTERVAL) {
        mp_packet_t pkt;
        mp_fill_state(mp, &pkt);
        mp_send_peers(mp, &pkt);
        mp->tx_count++;                   // count per update tick, not per peer
        mp->last_send = now;
    }
}

// Build an event packet and unicast it to every known peer. Falls back to
// broadcast if we don't have any peer addresses yet.
static void mp_send_event(mp_t *mp, uint16_t type,
                          const float origin[3], const float dir[3],
                          uint32_t target_session) {
    if (mp->sock == SOCK_INVALID) return;
    mp_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic          = MP_MAGIC;
    pkt.version        = MP_VERSION;
    pkt.type           = type;
    pkt.session_id     = mp->session_id;
    pkt.target_session = target_session;
    if (origin) memcpy(pkt.fire_origin, origin, sizeof(pkt.fire_origin));
    if (dir)    memcpy(pkt.fire_dir, dir, sizeof(pkt.fire_dir));

    if (mp_send_peers(mp, &pkt) == 0) mp_send_broadcast(mp, &pkt);
}

void mp_send_laser(mp_t *mp, const float origin[3], const float dir[3]) {
    mp_send_event(mp, MP_MSG_LASER, origin, dir, 0);
}

void mp_send_hit(mp_t *mp, uint32_t target_session) {
    mp_send_event(mp, MP_MSG_HIT, NULL, NULL, target_session);
}

void mp_send_lost(mp_t *mp) {
    mp_send_event(mp, MP_MSG_LOST, NULL, NULL, 0);
}

void mp_close(mp_t *mp) {
    if (mp->sock != SOCK_INVALID) {
        SOCK_CLOSE(mp->sock);
        mp->sock = SOCK_INVALID;
    }
#ifdef _WIN32
    WSACleanup();
#endif
}
