#ifndef INPUT_H
#define INPUT_H

#include <3ds.h>
#include <stdint.h>
#include <stdbool.h>

// Action IDs for NFS Most Wanted 2012
typedef enum {
    ACT_NONE = 0,
    ACT_ACCELERATE,
    ACT_BRAKE,
    ACT_NITROUS,
    ACT_HANDBRAKE,
    ACT_SHIFT_UP,
    ACT_SHIFT_DOWN,
    ACT_CAMERA,
    ACT_LOOK_BACK,
    ACT_RESET_CAR,
    ACT_EASY_DRIVE,
    ACT_WORLD_MAP,
    ACT_COUNT
} ActionID;

// Input state structure
typedef struct {
    bool buttons[ACT_COUNT];
    float stick_x;
    float stick_y;
} InputState;

// Function prototypes
void input_init(void);
void input_update(void);
bool input_is_pressed(ActionID action);
float input_get_stick_x(void);
float input_get_stick_y(void);

#endif // INPUT_H
