#ifndef WORLD_H
#define WORLD_H

#include <3ds.h>
#include <citro3d.h>
#include <stdbool.h>

struct WorldVertex {
    float position[3];
    float color[3];
};

void world_init(void);
void world_render(const C3D_Mtx* projection, const C3D_Mtx* view, float car_z);
void world_exit(void);

#endif // WORLD_H
