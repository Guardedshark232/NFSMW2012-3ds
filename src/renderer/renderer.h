#pragma once

#include <3ds.h>
#include <citro3d.h>
#include <citro2d.h>
#include "physics.h"
#include <stdint.h>

void renderer_init(void);
void renderer_exit(void);
void renderer_begin_frame(void);
void renderer_render_world_and_car(float dt);
void renderer_render_hud(bool easyDriveOpen, int easyDriveSelection, bool bndl_opened, uint32_t bndl_entries, float fps);
void renderer_reset_view(void);
float renderer_debug_car_y(void);
float renderer_debug_cam_y(void);
void renderer_end_frame(void);
