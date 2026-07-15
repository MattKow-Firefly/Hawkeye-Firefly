#ifndef NET_MP_H
#define NET_MP_H

// ─────────────────────────────────────────────────────────────────────────
// net_mp — minimal LAN multiplayer for Hawkeye.
//
// Peer discovery + drone-position sharing over UDP broadcast. Every instance
// on the same subnet periodically broadcasts its own drone's state; receiving
// a packet from a new sender registers that sender as a peer. There is no
// host/client role — discovery is symmetric, so launch order does not matter.
//
// The wire format is a fixed, packed struct sent raw. This assumes both peers
// share byte order (fine for the x86/ARM little-endian machines PX4 runs on).
// It is intentionally NOT a hardened protocol — it is a LAN toy to get started.
// ─────────────────────────────────────────────────────────────────────────

#include <stdbool.h>
#include <stdint.h>
#include "mavlink_receiver.h"  // hil_state_t, sock_t

#define MP_MAX_PEERS    15      // leaves room for 1 local vehicle up to MAX_VEHICLES
#define MP_DEFAULT_PORT 14555
#define MP_NAME_LEN     16
#define MP_MAX_EVENTS   64      // received laser-fire events buffered per poll

// A laser-fire event received from a peer (world-space origin + unit direction).
typedef struct {
    float origin[3];
    float dir[3];
} mp_fire_event_t;

// A discovered remote peer and its latest drone state.
typedef struct {
    bool        used;
    uint32_t    session_id;   // sender's unique per-run id
    uint8_t     sysid;        // sender's MAVLink system id (display only)
    uint8_t     mav_type;     // MAV_TYPE, for model selection
    int32_t     score;        // sender's score (from packet)
    int32_t     health;       // sender's health (from packet)
    hil_state_t state;        // latest position/attitude/velocity
    double      last_seen;    // timestamp from the caller's clock (seconds)
    char        name[MP_NAME_LEN];
    // Peer's source address (sockaddr_in bytes), learned from received packets,
    // used to unicast to this peer instead of broadcasting.
    uint8_t     addr[16];
    bool        addr_valid;
    // Receive-rate tracking (packets/sec, averaged over a ~1s window).
    float       rx_hz;
    int         rx_count;
    double      rx_t0;
} mp_peer_t;

typedef struct {
    sock_t      sock;
    uint16_t    port;
    uint32_t    session_id;    // our own id — used to ignore our own broadcasts
    uint8_t     self_sysid;
    uint8_t     self_mav_type;
    int32_t     self_score;    // our score (set via mp_set_local, broadcast to peers)
    int32_t     self_health;   // our health (set via mp_set_local, broadcast to peers)
    char        self_name[MP_NAME_LEN];
    hil_state_t local_state;   // our drone's latest state (sent to peers)
    double      last_send;     // timestamp of last high-rate unicast to peers
    double      last_beacon;   // timestamp of last low-rate discovery broadcast
    // Send-rate tracking (packets/sec, averaged over a ~1s window).
    float       tx_hz;
    int         tx_count;
    double      tx_t0;
    mp_peer_t   peers[MP_MAX_PEERS];

    // Events received during the last mp_poll(), for the caller to drain.
    mp_fire_event_t rx_fires[MP_MAX_EVENTS];  // peers' laser shots to spawn
    int         rx_fire_count;
    int         rx_hits;       // HIT packets that targeted us this poll
    bool        rx_lost;       // a peer reported they lost (we won)
} mp_t;

// Open the broadcast socket bound to `port`. `name` is the player name; pass
// NULL to default to the current user's login name. Returns 0 on success, -1 on failure.
int  mp_init(mp_t *mp, uint16_t port, uint8_t self_sysid, const char *name);

// Update the local drone state (and score/health) that gets broadcast to peers.
void mp_set_local(mp_t *mp, const hil_state_t *state, uint8_t sysid,
                  uint8_t mav_type, int32_t score, int32_t health);

// Send (rate-limited) our state and drain incoming peer packets. Time out
// peers not heard from recently. `now` is a monotonic clock in seconds
// (e.g. raylib's GetTime()). Received events are exposed via rx_* and reset
// at the start of each call.
void mp_poll(mp_t *mp, double now);

// Broadcast a laser-fire event (world-space origin + unit direction).
void mp_send_laser(mp_t *mp, const float origin[3], const float dir[3]);

// Notify a specific peer that one of our lasers hit them.
void mp_send_hit(mp_t *mp, uint32_t target_session);

// Announce that we lost (health reached 0). Peers treat this as their win.
void mp_send_lost(mp_t *mp);

// Close the socket.
void mp_close(mp_t *mp);

#endif
