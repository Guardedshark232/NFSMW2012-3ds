#ifndef FB_DUMP_H
#define FB_DUMP_H

#include <3ds.h>
#include <citro3d.h>

void fbdump_init(void);
void fbdump_tick(C3D_RenderTarget* top, C3D_RenderTarget* bottom);

#endif
