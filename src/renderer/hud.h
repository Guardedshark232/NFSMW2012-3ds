#pragma once
#include <3ds.h>
#include <citro3d.h>
#include <citro2d.h>
#include "physics.h"
#include <stdint.h>

void hud_init(void);
void hud_exit(void);
void hud_render(C3D_RenderTarget* target, const CarPhysicsState* car, bool easyDriveOpen, int easyDriveSelection, bool bndl_opened, uint32_t bndl_entries, float fps);
