#ifndef LASERS_H
#define LASERS_H

// ─────────────────────────────────────────────────────────────────────────
// lasers — projectiles fired from a drone's legs.
//
// Each laser is a capsule (pill): a segment cap_a→cap_b with a radius. It flies
// straight along the body-forward direction at a fixed speed until it hits the
// ground or exceeds its max range. The capsule + AABB are recomputed every
// frame and are the collision volume — object collision (against a future
// dynamic set of targets) will test against these; only ground/range is handled
// here for now.
// ─────────────────────────────────────────────────────────────────────────

#include <stdbool.h>
#include "raylib.h"

#define LASERS_MAX        128
#define LASER_SPEED_MPS   75.0f    // medium speed
#define LASER_MAX_RANGE_M 500.0f   // despawn distance
#define LASER_LENGTH_M    2.0f     // pill core length
#define LASER_RADIUS_M    0.10f    // pill radius (collision + visual)
#define LASER_FIRE_INTERVAL_S 0.20f  // gap between shots while the trigger is held

typedef struct {
    bool    active;
    Vector3 pos;        // center of the pill, advances each frame
    Vector3 dir;        // unit travel direction
    float   traveled;   // meters flown since spawn

    // Collision volume, refreshed every update():
    Vector3 cap_a;      // rear cap center
    Vector3 cap_b;      // front cap center (leading tip)
    float   radius;
    BoundingBox bbox;   // broadphase AABB around the capsule
} laser_t;

typedef struct {
    laser_t items[LASERS_MAX];
} lasers_t;

// Zero all slots.
void lasers_init(lasers_t *L);

// Spawn a single laser at origin travelling along dir (need not be normalized).
void lasers_spawn(lasers_t *L, Vector3 origin, Vector3 dir);

// Fire the two leg lasers from a drone at body_pos with the given attitude.
// scale is the vehicle model scale (used to place the legs). Forward is derived
// from heading/pitch in the app's world frame (North=-Z, East=+X, Up=+Y).
void lasers_fire_from(lasers_t *L, Vector3 body_pos,
                      float heading_deg, float pitch_deg, float scale);

// Advance all lasers; despawn on ground contact (tip.y <= ground_y) or range.
void lasers_update(lasers_t *L, float dt, float ground_y);

// Draw all active lasers (call inside BeginMode3D).
void lasers_draw(const lasers_t *L);

#endif
