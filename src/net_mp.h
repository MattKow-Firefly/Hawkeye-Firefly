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

// A discovered remote peer and its latest drone state.
typedef struct {
    bool        used;
    uint32_t    session_id;   // sender's unique per-run id
    uint8_t     sysid;        // sender's MAVLink system id (display only)
    uint8_t     mav_type;     // MAV_TYPE, for model selection
    int32_t     score;        // sender's score (from packet)
    hil_state_t state;        // latest position/attitude/velocity
    double      last_seen;    // timestamp from the caller's clock (seconds)
    char        name[MP_NAME_LEN];
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
    char        self_name[MP_NAME_LEN];
    hil_state_t local_state;   // our drone's latest state (broadcast to peers)
    double      last_send;     // timestamp of last outgoing broadcast
    // Send-rate tracking (packets/sec, averaged over a ~1s window).
    float       tx_hz;
    int         tx_count;
    double      tx_t0;
    mp_peer_t   peers[MP_MAX_PEERS];
} mp_t;

// Open the broadcast socket bound to `port`. `name` is the player name; pass
// NULL to default to the current user's login name. Returns 0 on success, -1 on failure.
int  mp_init(mp_t *mp, uint16_t port, uint8_t self_sysid, const char *name);

// Update the local drone state (and score) that gets broadcast to peers.
void mp_set_local(mp_t *mp, const hil_state_t *state, uint8_t sysid,
                  uint8_t mav_type, int32_t score);

// Send (rate-limited) our state and drain incoming peer packets. Time out
// peers not heard from recently. `now` is a monotonic clock in seconds
// (e.g. raylib's GetTime()).
void mp_poll(mp_t *mp, double now);

// Close the socket.
void mp_close(mp_t *mp);

#endif
