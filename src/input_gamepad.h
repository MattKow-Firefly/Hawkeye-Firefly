#ifndef INPUT_GAMEPAD_H
#define INPUT_GAMEPAD_H

// ─────────────────────────────────────────────────────────────────────────
// input_gamepad — read an Xbox-style gamepad for Hawkeye's own UI actions.
//
// Flight control stays with QGC (sticks + arm go QGC → PX4 as before). This
// module only reads the pad for viewer actions. Reading here does NOT conflict
// with QGC: on Linux both read the device non-exclusively via evdev/SDL, and
// on Windows XInput allows multiple readers.
//
// Add a new control by extending gp_action_t and the GP_BINDINGS table in
// input_gamepad.c — nothing else needs to change.
// ─────────────────────────────────────────────────────────────────────────

#include <stdbool.h>

// Semantic actions Hawkeye responds to. Keep GP_ACTION_COUNT last.
// NOTE: the A button is intentionally left unbound — QGC uses it to arm.
typedef enum {
    GP_ACTION_SHOOT,        // right trigger
    GP_ACTION_PAUSE,        // Menu/Start button — toggle pause
    GP_ACTION_CAMERA_MODE,  // Y button — cycle camera mode
    GP_ACTION_OVERLAY,      // View/Back button — toggle the gamepad overlay
    GP_ACTION_COUNT
} gp_action_t;

typedef struct {
    int         gamepad;    // active slot chosen each frame (-1 if none)
    bool        connected;
    const char *name;       // driver-reported name (NULL if disconnected)
    const char *profile;    // active mapping-profile label (NULL until resolved)

    // Per-action state, refreshed every input_gamepad_update():
    bool pressed[GP_ACTION_COUNT];   // transitioned to down this frame (edge)
    bool released[GP_ACTION_COUNT];  // transitioned to up this frame (edge)
    bool down[GP_ACTION_COUNT];      // currently held (level)

    // Raw analog values, handy for overlays or future controls:
    float left_x, left_y;            // -1..1 (up is negative)
    float right_x, right_y;          // -1..1
    float left_trigger, right_trigger; // -1 (released) .. 1 (fully pressed)
} input_gamepad_t;

// Initialize state. `gamepad` is a starting hint only — update() re-selects
// the active slot each frame, skipping filtered-out devices (see .c).
void input_gamepad_init(input_gamepad_t *ig, int gamepad);

// Poll the pad and recompute action state. Auto-selects the active gamepad
// slot each call. Call once per frame, before you query any actions.
void input_gamepad_update(input_gamepad_t *ig);

// True only on the frame the action's button went down.
static inline bool input_gamepad_pressed(const input_gamepad_t *ig, gp_action_t a) {
    return ig->connected && ig->pressed[a];
}
// True while the action's button is held.
static inline bool input_gamepad_down(const input_gamepad_t *ig, gp_action_t a) {
    return ig->connected && ig->down[a];
}

// Draw a compact live overlay of button/stick/trigger state (top-right).
// Uses raylib's default font so the module has no theme dependency.
void input_gamepad_draw_debug(const input_gamepad_t *ig, int screen_w, int screen_h);

#endif
