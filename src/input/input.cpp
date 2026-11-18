#include "input.h"
#include <3ds.h>

static InputState g_input;

void input_init(void) {
    for (int i = 0; i < ACT_COUNT; i++) {
        g_input.buttons[i] = false;
    }
    g_input.stick_x = 0.0f;
    g_input.stick_y = 0.0f;
}

void input_update(void) {
    hidScanInput();
    u32 kHeld = hidKeysHeld();

    g_input.buttons[ACT_ACCELERATE] = (kHeld & KEY_A) != 0;
    g_input.buttons[ACT_BRAKE]      = (kHeld & KEY_B) != 0;
    g_input.buttons[ACT_NITROUS]    = (kHeld & KEY_X) != 0;
    g_input.buttons[ACT_HANDBRAKE]  = (kHeld & KEY_Y) != 0;
    g_input.buttons[ACT_SHIFT_UP]   = (kHeld & KEY_ZR) != 0;
    g_input.buttons[ACT_SHIFT_DOWN] = (kHeld & KEY_ZL) != 0;
    g_input.buttons[ACT_CAMERA]     = (kHeld & KEY_L) != 0;
    g_input.buttons[ACT_LOOK_BACK]  = (kHeld & KEY_R) != 0;
    g_input.buttons[ACT_RESET_CAR]  = (kHeld & KEY_SELECT) != 0;
    g_input.buttons[ACT_EASY_DRIVE] = (kHeld & KEY_START) != 0;
    g_input.buttons[ACT_WORLD_MAP]  = (kHeld & KEY_DLEFT) != 0;

    circlePosition pos;
    hidCircleRead(&pos);
    g_input.stick_x = (float)pos.dx / 150.0f;
    g_input.stick_y = (float)pos.dy / 150.0f;
    if (g_input.stick_x >  1.0f) g_input.stick_x =  1.0f;
    if (g_input.stick_x < -1.0f) g_input.stick_x = -1.0f;
    if (g_input.stick_y >  1.0f) g_input.stick_y =  1.0f;
    if (g_input.stick_y < -1.0f) g_input.stick_y = -1.0f;
}

bool input_is_pressed(ActionID action) {
    if (action < 0 || action >= ACT_COUNT) return false;
    return g_input.buttons[action];
}

float input_get_stick_x(void) {
    return g_input.stick_x;
}

float input_get_stick_y(void) {
    return g_input.stick_y;
}
