#pragma once
#include <3ds.h>
#include <citro3d.h>
#include <citro2d.h>

void assets_init(void);
void assets_exit(void);

C2D_SpriteSheet assets_get_sheet(const char* name);
C2D_Image assets_get_image(const char* sheetName, size_t index);
C3D_Tex* assets_world_atlas(void);
