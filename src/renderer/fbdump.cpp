#include "fbdump.h"
#include <3ds.h>
#include <citro3d.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Enabled while diagnosing the GPU world-render mismatch.  Writes both raw
// framebuffers to the SD card so the rendered image can be inspected without
// a capture device.  The host tool does the tiling/channel decode.
// Off by default; build with -DFB_DUMP_ENABLE=1 to re-enable.
#ifndef FB_DUMP_ENABLE
#define FB_DUMP_ENABLE 0
#endif

// Frames at which the dump is taken.  The first is late enough that world
// loading is done; the rest sample the car while it drives.
static const int kDumpFrames[] = { 60, 300, 540, 780, 900 };
static const int kNumDumpFrames = (int)(sizeof(kDumpFrames) / sizeof(kDumpFrames[0]));

static int g_frame = 0;

#if FB_DUMP_ENABLE
static void dump_raw(C3D_RenderTarget* target, const char* path) {
    if (!target || !target->frameBuf.colorBuf) return;
    const size_t bytes = (size_t)target->frameBuf.width * target->frameBuf.height * 4;

    // The GPU wrote this buffer; drop any stale CPU cache lines before the
    // CPU reads it back.
    GSPGPU_InvalidateDataCache(target->frameBuf.colorBuf, (u32)bytes);
    gspWaitForVBlank();

    FILE* f = fopen(path, "wb");
    if (f) {
        fwrite(target->frameBuf.colorBuf, 1, bytes, f);
        fclose(f);
    }
}
#endif

void fbdump_init(void) {
    g_frame = 0;
}

void fbdump_tick(C3D_RenderTarget* top, C3D_RenderTarget* bottom) {
#if FB_DUMP_ENABLE
    g_frame++;
    for (int i = 0; i < kNumDumpFrames; i++) {
        if (g_frame != kDumpFrames[i]) continue;
        char path[64];
        snprintf(path, sizeof(path), "sdmc:/fbraw_top_%d.bin", kDumpFrames[i]);
        dump_raw(top, path);
        snprintf(path, sizeof(path), "sdmc:/fbraw_bottom_%d.bin", kDumpFrames[i]);
        dump_raw(bottom, path);
    }
#endif
}
