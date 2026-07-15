#include "lasers.h"

#include <math.h>
#include "raymath.h"

void lasers_init(lasers_t *L) {
    for (int i = 0; i < LASERS_MAX; i++) L->items[i].active = false;
}

// Recompute the capsule endpoints and AABB from the current center/direction.
static void laser_refresh_volume(laser_t *l) {
    Vector3 half = Vector3Scale(l->dir, LASER_LENGTH_M * 0.5f);
    l->cap_a = Vector3Subtract(l->pos, half);  // rear
    l->cap_b = Vector3Add(l->pos, half);        // front (leading tip)
    l->radius = LASER_RADIUS_M;

    Vector3 lo = {
        fminf(l->cap_a.x, l->cap_b.x) - l->radius,
        fminf(l->cap_a.y, l->cap_b.y) - l->radius,
        fminf(l->cap_a.z, l->cap_b.z) - l->radius,
    };
    Vector3 hi = {
        fmaxf(l->cap_a.x, l->cap_b.x) + l->radius,
        fmaxf(l->cap_a.y, l->cap_b.y) + l->radius,
        fmaxf(l->cap_a.z, l->cap_b.z) + l->radius,
    };
    l->bbox = (BoundingBox){ lo, hi };
}

void lasers_spawn(lasers_t *L, Vector3 origin, Vector3 dir) {
    float len = Vector3Length(dir);
    if (len < 1e-6f) return;  // no direction, nothing to fire
    for (int i = 0; i < LASERS_MAX; i++) {
        laser_t *l = &L->items[i];
        if (l->active) continue;
        l->active = true;
        l->pos = origin;
        l->dir = Vector3Scale(dir, 1.0f / len);
        l->traveled = 0.0f;
        laser_refresh_volume(l);
        return;
    }
    // Pool full: silently drop (rare at these limits).
}

int lasers_fire_from(lasers_t *L, Vector3 body_pos,
                     float heading_deg, float pitch_deg, float scale,
                     laser_shot_t *out, int max_out) {
    float h = heading_deg * DEG2RAD;
    float p = pitch_deg * DEG2RAD;

    // Body forward in world space. Heading is measured from North (-Z) toward
    // East (+X); pitch tilts the nose up (+Y). Roll doesn't change where the
    // nose points, so it's ignored.
    Vector3 fwd = Vector3Normalize((Vector3){
        cosf(p) * sinf(h),
        sinf(p),
        cosf(p) * -cosf(h),
    });

    // Right vector for placing the two legs. Fall back to +X if forward is
    // nearly vertical (degenerate cross product).
    Vector3 up = { 0.0f, 1.0f, 0.0f };
    Vector3 right = Vector3CrossProduct(fwd, up);
    if (Vector3Length(right) < 1e-4f) right = (Vector3){ 1.0f, 0.0f, 0.0f };
    right = Vector3Normalize(right);

    // Leg spawn points: a bit below the body, offset left/right, nudged forward.
    float half_w   = scale * 0.30f;
    float drop     = scale * -0.02f;
    float fwd_off  = scale * 0.15f;
    Vector3 base = Vector3Add(body_pos, Vector3Scale(fwd, fwd_off));
    base.y -= drop;
    Vector3 leg_l = Vector3Subtract(base, Vector3Scale(right, half_w));
    Vector3 leg_r = Vector3Add(base, Vector3Scale(right, half_w));

    lasers_spawn(L, leg_l, fwd);
    lasers_spawn(L, leg_r, fwd);

    // Report the shots so the caller can forward them to peers.
    int n = 0;
    if (out && max_out > 0) { out[n].origin = leg_l; out[n].dir = fwd; n++; }
    if (out && max_out > 1) { out[n].origin = leg_r; out[n].dir = fwd; n++; }
    return n;
}

void lasers_update(lasers_t *L, float dt, float ground_y) {
    float step = LASER_SPEED_MPS * dt;
    for (int i = 0; i < LASERS_MAX; i++) {
        laser_t *l = &L->items[i];
        if (!l->active) continue;

        l->pos = Vector3Add(l->pos, Vector3Scale(l->dir, step));
        l->traveled += step;
        laser_refresh_volume(l);

        // Despawn conditions: reached max range, or the leading tip hit ground.
        if (l->traveled >= LASER_MAX_RANGE_M || l->cap_b.y <= ground_y) {
            l->active = false;
            continue;
        }

    }
}

// Squared distance from point p to segment ab.
static float point_seg_dist2(Vector3 p, Vector3 a, Vector3 b) {
    Vector3 ab = Vector3Subtract(b, a);
    Vector3 ap = Vector3Subtract(p, a);
    float denom = Vector3DotProduct(ab, ab);
    float t = (denom > 1e-9f) ? Vector3DotProduct(ap, ab) / denom : 0.0f;
    if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
    Vector3 closest = Vector3Add(a, Vector3Scale(ab, t));
    Vector3 d = Vector3Subtract(p, closest);
    return Vector3DotProduct(d, d);
}

bool lasers_take_hit(lasers_t *L, Vector3 center, float radius) {
    float r = radius + LASER_RADIUS_M;
    float r2 = r * r;
    for (int i = 0; i < LASERS_MAX; i++) {
        laser_t *l = &L->items[i];
        if (!l->active) continue;
        if (point_seg_dist2(center, l->cap_a, l->cap_b) <= r2) {
            l->active = false;  // consume the laser on impact
            return true;
        }
    }
    return false;
}

void lasers_draw(const lasers_t *L, Color color) {
    for (int i = 0; i < LASERS_MAX; i++) {
        const laser_t *l = &L->items[i];
        if (!l->active) continue;

        // Solid bright core with rounded caps (the beam itself).
        DrawCylinderEx(l->cap_a, l->cap_b, l->radius, l->radius, 10, color);
        DrawSphere(l->cap_a, l->radius, color);
        DrawSphere(l->cap_b, l->radius, color);
    }
}
