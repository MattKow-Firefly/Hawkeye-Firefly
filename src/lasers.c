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

void lasers_fire_from(lasers_t *L, Vector3 body_pos,
                      float heading_deg, float pitch_deg, float scale) {
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

        // TODO: object collision goes here — test l->bbox / capsule (l->cap_a,
        // l->cap_b, l->radius) against the dynamic target set once it exists.
    }
}

void lasers_draw(const lasers_t *L) {
    const Color core  = (Color){ 255, 60, 50, 255 };
    const Color glow  = (Color){ 255, 90, 80, 90 };

    for (int i = 0; i < LASERS_MAX; i++) {
        const laser_t *l = &L->items[i];
        if (!l->active) continue;

        // Translucent outer shell for a bit of glow, solid bright core, rounded
        // caps to complete the pill.
        // Nah, disable that, only show the actual laser beam
        //DrawCylinderEx(l->cap_a, l->cap_b, l->radius * 1.5f, l->radius * 1.5f, 8, glow);
        DrawCylinderEx(l->cap_a, l->cap_b, l->radius, l->radius, 10, core);
        DrawSphere(l->cap_a, l->radius, core);
        DrawSphere(l->cap_b, l->radius, core);
    }
}
