#include "input_gamepad.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "raylib.h"

// We read the controller directly through GLFW's raw joystick API (those
// symbols are linked in via raylib's bundled GLFW). This avoids needing an SDL
// gamepad-mapping database: raylib's own IsGamepadButtonDown() only reports
// controllers that already have a mapping, whereas glfwGetJoystickButtons/Axes
// always return raw state. The trade-off is that raw indices are specific to
// the controller and connection; the defaults below are for an Xbox Series pad
// on Linux over USB. Build with -DGP_SHOW_RAW_INPUT=1 to see live indices and
// adjust GP_BINDINGS if yours differs.
extern int                  glfwJoystickPresent(int jid);
extern const char          *glfwGetJoystickName(int jid);
extern const unsigned char *glfwGetJoystickButtons(int jid, int *count);
extern const float         *glfwGetJoystickAxes(int jid, int *count);

#define GP_GLFW_PRESS 1
#define GP_MAX_SCAN   16      // GLFW supports joystick ids 0..15
#define GP_TRIGGER_ON (-0.5f) // trigger axis rests at -1, +1 fully pulled

// Set to 1 to add a raw button/axis diagnostic to the overlay. Off by default.
#ifndef GP_SHOW_RAW_INPUT
#define GP_SHOW_RAW_INPUT 0
#endif

// Some HID devices (e.g. a Keychron keyboard's "System Control" collection)
// enumerate as joysticks. Skip any whose name contains this substring.
#define GP_IGNORE_SUBSTR "Keychron"

// Raw index binding per action. Use button >= 0 for a digital button, or
// axis >= 0 to treat an analog axis (e.g. a trigger) as a button.
typedef struct { int button; int axis; } gp_binding_t;

// Which OS a mapping is for (raw indices differ per platform driver).
typedef enum { GP_PLAT_ANY, GP_PLAT_LINUX, GP_PLAT_WINDOWS, GP_PLAT_MACOS } gp_platform_t;

#if defined(_WIN32)
#define GP_THIS_PLATFORM GP_PLAT_WINDOWS
#elif defined(__APPLE__)
#define GP_THIS_PLATFORM GP_PLAT_MACOS
#else
#define GP_THIS_PLATFORM GP_PLAT_LINUX
#endif

// A named mapping profile: matched against the controller name (substring) and
// the build platform. To support a new controller/OS, add an entry here — build
// with -DGP_SHOW_RAW_INPUT=1, read the live indices off the overlay, and fill
// in the bindings. The first entry is the default used when nothing matches.
typedef struct {
    const char   *label;       // human-readable description
    const char   *name_match;  // substring of the controller name to match
    gp_platform_t platform;    // OS this mapping targets
    gp_binding_t  bindings[GP_ACTION_COUNT];
} gp_profile_t;

static const gp_profile_t GP_PROFILES[] = {
    // Xbox controller on Linux (xpad/xone, USB):
    //   A=b0 B=b1 X=b2 Y=b3 LB=b4 RB=b5 Back=b6 Start=b7 Guide=b8
    //   LStick=b9 RStick=b10; axes LX=a0 LY=a1 LT=a2 RX=a3 RY=a4 RT=a5.
    {
        .label = "Xbox controller (Linux)",
        .name_match = "Xbox",
        .platform = GP_PLAT_LINUX,
        .bindings = {
            [GP_ACTION_SHOOT]       = { .button = -1, .axis = 5 },  // right trigger
            [GP_ACTION_MENU]        = { .button = 7,  .axis = -1 }, // Start / Menu
            [GP_ACTION_CAMERA_MODE] = { .button = 3,  .axis = -1 }, // Y
        },
    },
    // Add more profiles here (other controllers, Windows, macOS, …).
};
#define GP_PROFILE_COUNT   ((int)(sizeof(GP_PROFILES) / sizeof(GP_PROFILES[0])))
#define GP_DEFAULT_PROFILE 0   // fallback profile index when no name/platform matches

// Active bindings, resolved from GP_PROFILES when the controller changes.
// One gamepad instance is in play, so a file-static selection is sufficient.
static const gp_binding_t *s_bindings = GP_PROFILES[GP_DEFAULT_PROFILE].bindings;

// Resolve the mapping profile for a controller name, logging the outcome. Only
// called when the active controller changes (not every frame).
static void gp_resolve_profile(input_gamepad_t *ig, const char *name) {
    for (int p = 0; p < GP_PROFILE_COUNT; p++) {
        const gp_profile_t *pr = &GP_PROFILES[p];
        if (pr->platform != GP_PLAT_ANY && pr->platform != GP_THIS_PLATFORM) continue;
        if (pr->name_match && name[0] && strstr(name, pr->name_match)) {
            s_bindings = pr->bindings;
            ig->profile = pr->label;
            TraceLog(LOG_INFO, "GAMEPAD: '%s' matched mapping profile '%s'",
                     name, pr->label);
            return;
        }
    }
    // No match — keep working with the default, but make it loud so the user
    // knows to add a profile for this controller/platform.
    s_bindings = GP_PROFILES[GP_DEFAULT_PROFILE].bindings;
    ig->profile = "default (no match)";
    TraceLog(LOG_WARNING, "GAMEPAD: no mapping profile matched '%s' on this platform; "
             "using default mapping ('%s'). Add a profile in input_gamepad.c "
             "(GP_PROFILES) to map this controller correctly.",
             name, GP_PROFILES[GP_DEFAULT_PROFILE].label);
}

void input_gamepad_init(input_gamepad_t *ig, int gamepad) {
    for (int i = 0; i < GP_ACTION_COUNT; i++) {
        ig->pressed[i] = ig->released[i] = ig->down[i] = false;
    }
    ig->gamepad = gamepad;
    ig->connected = false;
    ig->name = NULL;
    ig->profile = NULL;
    ig->left_x = ig->left_y = ig->right_x = ig->right_y = 0.0f;
    ig->left_trigger = ig->right_trigger = -1.0f;
}

void input_gamepad_update(input_gamepad_t *ig) {
    // Clear per-frame edges up front so an early return leaves them false.
    for (int i = 0; i < GP_ACTION_COUNT; i++) {
        ig->pressed[i] = false;
        ig->released[i] = false;
    }

    // Select the active joystick: first present slot whose name isn't filtered
    // out. Re-scanned every frame so hotplugging just works.
    int gp = -1;
    for (int i = 0; i < GP_MAX_SCAN; i++) {
        if (!glfwJoystickPresent(i)) continue;
        const char *nm = glfwGetJoystickName(i);
        if (nm && strstr(nm, GP_IGNORE_SUBSTR)) continue;  // e.g. the keyboard
        gp = i;
        break;
    }

    // Track the last-resolved controller so we only re-match (and log) on change.
    static char s_resolved[128] = {0};

    ig->gamepad = gp;
    ig->connected = (gp >= 0);
    if (!ig->connected) {
        for (int i = 0; i < GP_ACTION_COUNT; i++) ig->down[i] = false;
        ig->name = NULL;
        ig->profile = NULL;
        ig->left_x = ig->left_y = ig->right_x = ig->right_y = 0.0f;
        ig->left_trigger = ig->right_trigger = -1.0f;
        s_resolved[0] = '\0';   // force re-resolve on reconnect
        return;
    }

    ig->name = glfwGetJoystickName(gp);

    // Resolve the mapping profile whenever the active controller changes.
    const char *cur_name = ig->name ? ig->name : "";
    if (strncmp(s_resolved, cur_name, sizeof(s_resolved)) != 0) {
        gp_resolve_profile(ig, cur_name);
        snprintf(s_resolved, sizeof(s_resolved), "%s", cur_name);
    }

    int nbtn = 0, nax = 0;
    const unsigned char *btn = glfwGetJoystickButtons(gp, &nbtn);
    const float        *ax  = glfwGetJoystickAxes(gp, &nax);

    ig->left_x        = (nax > 0) ? ax[0] : 0.0f;
    ig->left_y        = (nax > 1) ? ax[1] : 0.0f;
    ig->left_trigger  = (nax > 2) ? ax[2] : -1.0f;
    ig->right_x       = (nax > 3) ? ax[3] : 0.0f;
    ig->right_y       = (nax > 4) ? ax[4] : 0.0f;
    ig->right_trigger = (nax > 5) ? ax[5] : -1.0f;

    for (int i = 0; i < GP_ACTION_COUNT; i++) {
        const gp_binding_t *b = &s_bindings[i];
        bool cur = false;
        if (b->axis >= 0) {
            cur = (nax > b->axis) && (ax[b->axis] > GP_TRIGGER_ON);
        } else if (b->button >= 0) {
            cur = btn && (nbtn > b->button) && (btn[b->button] == GP_GLFW_PRESS);
        }
        ig->pressed[i]  = cur && !ig->down[i];
        ig->released[i] = !cur && ig->down[i];
        ig->down[i]     = cur;
    }
}

// ── Debug overlay ────────────────────────────────────────────────────────

static void draw_dot(int x, int y, bool on, const char *label) {
    Color c = on ? (Color){ 90, 220, 120, 255 } : (Color){ 90, 90, 100, 255 };
    DrawCircle(x, y, 6, c);
    DrawText(label, x + 12, y - 7, 14, (Color){ 210, 210, 220, 255 });
}

// Horizontal bar for a -1..1 axis (triggers/sticks).
static void draw_bar(int x, int y, int w, float v, const char *label) {
    DrawText(label, x, y - 2, 14, (Color){ 210, 210, 220, 255 });
    int bx = x + 46, bw = w - 46, bh = 10;
    DrawRectangle(bx, y, bw, bh, (Color){ 40, 40, 48, 255 });
    float t = (v + 1.0f) * 0.5f;           // -1..1 → 0..1
    if (t < 0) t = 0; if (t > 1) t = 1;
    DrawRectangle(bx, y, (int)(bw * t), bh, (Color){ 80, 160, 230, 255 });
}

void input_gamepad_draw_debug(const input_gamepad_t *ig, int screen_w, int screen_h) {
    (void)screen_h;
#if GP_SHOW_RAW_INPUT
    const int w = 240;
    const int h = ig->connected ? 374 : 52;
#else
    const int w = 226;
    const int h = ig->connected ? 228 : 52;
#endif
    const int x = screen_w - w - 12, y = 12;
    DrawRectangle(x, y, w, h, (Color){ 0, 0, 0, 170 });
    DrawRectangleLines(x, y, w, h, (Color){ 90, 90, 110, 200 });

    int cx = x + 14, cy = y + 12;
    DrawText("GAMEPAD", cx, cy, 16, (Color){ 255, 255, 255, 255 });
    cy += 20;

    if (!ig->connected) {
        DrawText("not connected", cx, cy, 14, (Color){ 220, 140, 140, 255 });
        return;
    }

    const char *nm = ig->name ? ig->name : "unknown";
    DrawText(TextFormat("%.30s", nm), cx, cy, 12, (Color){ 170, 170, 190, 255 });
    cy += 16;

    const char *pf = ig->profile ? ig->profile : "-";
    bool matched = ig->profile && strcmp(ig->profile, "default (no match)") != 0;
    DrawText(TextFormat("map: %.24s", pf), cx, cy, 12,
             matched ? (Color){ 150, 200, 150, 255 } : (Color){ 235, 200, 120, 255 });
    cy += 22;

    // Semantic actions (what Hawkeye reacts to)
    draw_dot(cx + 4, cy, ig->down[GP_ACTION_SHOOT],       "RT  Shoot");     cy += 22;
    draw_dot(cx + 4, cy, ig->down[GP_ACTION_MENU],        "Menu");          cy += 22;
    draw_dot(cx + 4, cy, ig->down[GP_ACTION_CAMERA_MODE], "Y   Camera");    cy += 24;

#if GP_SHOW_RAW_INPUT
    // ── Raw diagnostic — press a button and note which index lights up ──
    DrawText("RAW INPUT (GLFW joystick)", cx, cy, 12, (Color){ 235, 200, 120, 255 });
    cy += 18;

    int nbtn = 0, nax = 0;
    const unsigned char *btn = glfwGetJoystickButtons(ig->gamepad, &nbtn);
    const float *ax = glfwGetJoystickAxes(ig->gamepad, &nax);

    char held[96] = {0};
    for (int b = 0; b < nbtn; b++) {
        if (btn && btn[b] == GP_GLFW_PRESS) {
            char t[8];
            snprintf(t, sizeof(t), "%d ", b);
            strncat(held, t, sizeof(held) - strlen(held) - 1);
        }
    }
    DrawText(TextFormat("held btn: %s", held[0] ? held : "-"), cx, cy, 13,
             (Color){ 120, 220, 150, 255 });
    cy += 20;

    for (int a = 0; a < nax && a < 8; a++) {
        draw_bar(cx, cy, w - 28, ax[a], TextFormat("a%d", a));
        cy += 19;
    }
#else
    // Analog reference for the sticks and triggers we care about.
    draw_bar(cx, cy, w - 28, ig->left_trigger,  "LT");  cy += 20;
    draw_bar(cx, cy, w - 28, ig->right_trigger, "RT");  cy += 20;
    draw_bar(cx, cy, w - 28, ig->left_x,        "LX");  cy += 20;
    draw_bar(cx, cy, w - 28, ig->left_y,        "LY");
#endif
}
