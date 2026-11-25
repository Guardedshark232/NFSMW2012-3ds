#include "hud.h"
#include "assets.h"
#include "renderer.h"
#include "world_mesh.h"
#include <3ds/font.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Citro2D's C2D_TextBufNew() always maps the console's shared system font,
// even when every glyph comes from a bundled .bcfnt. On systems where
// APT_GetSharedFont() is unavailable that path leaves a NULL font and crashes
// inside C2D_TextBufNew(). Installing the bundled font as libctru's shared
// font first makes fontEnsureMapped() a no-op and keeps the HUD independent
// of console fonts.
extern "C" CFNT_s* g_sharedFont;

// Extent of the generated top-down map (romfs:/assets/minimap.t3x):
// 288 texels covering 1440 m, matching tools/gen_minimap.py.
static const float kMapMin = -720.0f;
static const float kMapSize = 1440.0f;
static const float kMapTexels = 288.0f;

// Radar window: 240 m of the map across, drawn at 70x70 px.
static const float kRadarWindowM = 240.0f;
static const float kRadarPx = 70.0f;
static const float kRadarX = 222.5f;
static const float kRadarY = 36.0f;

static C2D_TextBuf g_staticBuf = NULL;
static C2D_TextBuf g_dynamicBuf = NULL;
static C2D_TextBuf g_easyBuf = NULL;
static CFNT_s* g_bundledFont = NULL;
static C2D_Text g_staticText[6];
static C2D_Text g_easyText[6];
static C2D_Text g_dynamicText[6];
static unsigned int g_hudFrame = 0;
static bool g_dynamicReady = false;
static bool g_cachedDrifting = false;
static bool g_bndlTextReady = false;
static bool g_cachedBndlOpened = false;
static uint32_t g_cachedBndlEntries = 0;
static C2D_Text g_bndlText;

enum StaticText {
    STATIC_HEADER,
    STATIC_KMH,
    STATIC_FAIRHAVEN,
    STATIC_RADAR,
    STATIC_CAR_LINE,
    STATIC_CONTROL_LINE,
};

// Load a .bcfnt and hand it to libctru as the mapped "system" font. The data
// must stay in linear memory because Citro2D's glyph cache keeps pointers into
// the font's texture sheets.
static CFNT_s* load_bundled_shared_font(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    long size = ftell(f);
    if (size < (long)sizeof(CFNT_s) || fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return NULL;
    }

    CFNT_s* cfnt = (CFNT_s*)linearAlloc((size_t)size);
    if (!cfnt) {
        fclose(f);
        return NULL;
    }
    size_t got = fread(cfnt, 1, (size_t)size, f);
    fclose(f);

    // Reject anything that is not a little-endian CFNT with sane block count;
    // fontFixPointers() trusts these pointers and would corrupt memory.
    if (got != (size_t)size || cfnt->signature != 0x544E4643 ||
        cfnt->endianness != 0xFEFF || cfnt->fileSize != (u32)size ||
        cfnt->headerSize < 0x14 || cfnt->headerSize > (u32)size ||
        cfnt->nBlocks == 0 || cfnt->nBlocks > 32) {
        linearFree(cfnt);
        return NULL;
    }

    fontFixPointers(cfnt);
    g_sharedFont = cfnt;
    return cfnt;
}

void hud_init(void) {
    g_dynamicReady = false;
    g_hudFrame = 0;

    if (!g_bundledFont) {
        g_bundledFont = load_bundled_shared_font("romfs:/assets/helvetica.bcfnt");
        if (!g_bundledFont) {
            g_bundledFont = load_bundled_shared_font("romfs:/assets/conduit.bcfnt");
        }
    }

    // Without a bundled font, fall back to the console font if it is usable.
    if (!g_sharedFont && R_FAILED(fontEnsureMapped())) {
        printf("HUD: no usable font found; text disabled\n");
        return;
    }

    g_staticBuf = C2D_TextBufNew(4096);
    g_dynamicBuf = C2D_TextBufNew(1024);
    g_easyBuf = C2D_TextBufNew(1024);

    static const char* labels[6] = {
        "NFS: MOST WANTED (2012) - FAIRHAVEN 3DS",
        "km/h",
        "FAIRHAVEN",
        "RADAR MAP",
        "Porsche 911 Carrera S (2012)  |  Fairhaven City",
        "A Drive | B Brake / Reverse | START Menu | SELECT Reset",
    };
    for (int i = 0; g_staticBuf && i < 6; i++) {
        C2D_TextParse(&g_staticText[i], g_staticBuf, labels[i]);
        C2D_TextOptimize(&g_staticText[i]);
    }

    static const char* easyLabels[6] = {
        "EASYDRIVE - QUICK MENU",
        "1. Find Race: Hughes Park Sprint",
        "2. Change Car: Porsche 911 Carrera S",
        "3. Pro Mods: Burnout Nitrous Setup",
        "4. Map: Fairhaven Open World",
        "Press B to navigate | START to close",
    };
    for (int i = 0; g_easyBuf && i < 6; i++) {
        C2D_TextParse(&g_easyText[i], g_easyBuf, easyLabels[i]);
        C2D_TextOptimize(&g_easyText[i]);
    }
    g_bndlTextReady = false;
}

void hud_exit(void) {
    if (g_staticBuf) {
        C2D_TextBufDelete(g_staticBuf);
        g_staticBuf = NULL;
    }
    if (g_dynamicBuf) { C2D_TextBufDelete(g_dynamicBuf); g_dynamicBuf = NULL; }
    if (g_easyBuf) { C2D_TextBufDelete(g_easyBuf); g_easyBuf = NULL; }
    // g_bundledFont is intentionally never freed: Citro2D keeps a static
    // glyph-sheet cache pointing into it, with no public API to invalidate it.
}

static inline void parse_cached(C2D_Text* text, C2D_TextBuf buf, const char* str) {
    C2D_TextParse(text, buf, str);
    C2D_TextOptimize(text);
}

static inline void draw_cached(const C2D_Text* text, float x, float y,
                               float scaleX, float scaleY, u32 color) {
    if (!g_staticBuf || !g_dynamicBuf || !text) return;
    C2D_DrawText(text, C2D_WithColor, x, y, 0.5f, scaleX, scaleY, color);
}

void hud_render(C3D_RenderTarget* target, const CarPhysicsState* car, bool easyDriveOpen, int easyDriveSelection, bool bndl_opened, uint32_t bndl_entries, float fps) {
    if (!target) return;

    C2D_Prepare();
    C2D_TargetClear(target, C2D_Color32(14, 18, 28, 255));
    C2D_SceneBegin(target);

    g_hudFrame++;
    bool refreshText = !g_dynamicReady || ((g_hudFrame & 1u) == 0);
    if (!g_bndlTextReady || g_cachedBndlOpened != bndl_opened || g_cachedBndlEntries != bndl_entries) {
        if (g_staticBuf) {
            char bndlStr[96];
            if (bndl_opened) snprintf(bndlStr, sizeof(bndlStr), "CHAMELEON 2: WORLDLIST.BNDL (%lu entries)", (unsigned long)bndl_entries);
            else snprintf(bndlStr, sizeof(bndlStr), "CHAMELEON 2: SD / RomFS Streaming Ready");
            parse_cached(&g_bndlText, g_staticBuf, bndlStr);
            g_cachedBndlOpened = bndl_opened;
            g_cachedBndlEntries = bndl_entries;
            g_bndlTextReady = true;
        }
    }
    if (refreshText && g_dynamicBuf) {
        C2D_TextBufClear(g_dynamicBuf);
        char text[160];
        snprintf(text, sizeof(text), "%5.1f", car->speed);
        parse_cached(&g_dynamicText[0], g_dynamicBuf, text);
        if (car->reversing) snprintf(text, sizeof(text), "GEAR R   |   %4.0f RPM", car->rpm);
        else snprintf(text, sizeof(text), "GEAR %d   |   %4.0f RPM", car->gear, car->rpm);
        parse_cached(&g_dynamicText[1], g_dynamicBuf, text);
        snprintf(text, sizeof(text), "NITROUS %3.0f%%", car->nitrous_level * 100.0f);
        parse_cached(&g_dynamicText[2], g_dynamicBuf, text);
        g_cachedDrifting = car->is_drifting;
        if (g_cachedDrifting) {
            snprintf(text, sizeof(text), ">> POWER-SLIDE DRIFT ACTIVE! Angle: %4.1f deg <<", car->drift_angle * 57.3f);
        } else {
            snprintf(text, sizeof(text), "Distance: %6.1f m   |   Perf: %4.1f FPS (%4.1f ms)",
                     car->distance, fps, 1000.0f / (fps > 0 ? fps : 60.0f));
        }
        parse_cached(&g_dynamicText[3], g_dynamicBuf, text);
        snprintf(text, sizeof(text), "POS %6.1f %6.1f  GND %6.2f  CAM %6.2f  CH %d/%d",
                 car->pos_x, car->pos_z,
                 renderer_debug_car_y(), renderer_debug_cam_y(),
                 world_mesh_chunk_count(), world_mesh_chunk_total());
        parse_cached(&g_dynamicText[5], g_dynamicBuf, text);
        g_dynamicReady = true;
    }

    // 1. Top Header Banner
    C2D_DrawRectSolid(0, 0, 0, 320, 24, C2D_Color32(20, 25, 38, 255));
    C2D_DrawRectSolid(0, 24, 0, 320, 2, C2D_Color32(0, 200, 255, 255));
    draw_cached(&g_staticText[STATIC_HEADER], 10.f, 4.f, 0.52f, 0.52f, C2D_Color32(255, 255, 255, 255));

    // Porsche crest icon
    C2D_Image crestImg = assets_get_image("crest", 0);
    if (crestImg.tex) {
        C2D_DrawImageAt(crestImg, 282.f, 2.f, 0.5f, NULL, 0.35f, 0.35f);
    }

    // 2. Speedometer & Tachometer Panel
    C2D_DrawRectSolid(10, 34, 0, 185, 74, C2D_Color32(22, 28, 42, 230));
    C2D_DrawRectSolid(10, 34, 0, 185, 2, C2D_Color32(0, 180, 220, 255));

    draw_cached(&g_dynamicText[0], 16.f, 40.f, 0.95f, 0.95f, C2D_Color32(255, 255, 255, 255));
    draw_cached(&g_staticText[STATIC_KMH], 125.f, 52.f, 0.45f, 0.45f, C2D_Color32(0, 200, 255, 255));
    draw_cached(&g_dynamicText[1], 16.f, 80.f, 0.48f, 0.48f, C2D_Color32(0, 220, 255, 255));

    // Minimap / Radar Preview (Top-Right)
    C2D_DrawRectSolid(205, 34, 0, 105, 74, C2D_Color32(22, 28, 42, 230));
    C2D_DrawRectSolid(205, 34, 0, 105, 2, C2D_Color32(0, 180, 220, 255));
    C2D_Image mapImg = assets_get_image("minimap", 0);
    if (mapImg.tex && mapImg.subtex) {
        // Scrolling local radar: show a 240 m window of the top-down
        // Fairhaven map centred on the car, with a heading marker pinned
        // to the middle of the panel.
        float winTex = kRadarWindowM * (kMapTexels / kMapSize);
        float cu = (car->pos_x - kMapMin) / kMapSize;
        float cv = (car->pos_z - kMapMin) / kMapSize;
        float frac = winTex / kMapTexels;

        Tex3DS_SubTexture sub = *mapImg.subtex;
        float du = (sub.right - sub.left) * frac;
        float dv = (sub.bottom - sub.top) * frac;
        float u0 = sub.left + (sub.right - sub.left) * cu - du * 0.5f;
        float v0 = sub.top + (sub.bottom - sub.top) * cv - dv * 0.5f;
        float u_min = fminf(sub.left, sub.right);
        float u_max = fmaxf(sub.left, sub.right) - fabsf(du);
        float v_min = fminf(sub.top, sub.bottom);
        float v_max = fmaxf(sub.top, sub.bottom) - fabsf(dv);
        if (du >= 0.0f) {
            if (u0 < u_min) u0 = u_min;
            if (u0 > u_max) u0 = u_max;
        }
        if (dv >= 0.0f) {
            if (v0 < v_min) v0 = v_min;
            if (v0 > v_max) v0 = v_max;
        } else {
            // Packed textures can have inverted V (top > bottom, dv < 0).
            if (v0 > v_min - dv) v0 = v_min - dv;
            if (v0 < v_max - dv) v0 = v_max - dv;
        }
        sub.width = (u16)winTex;
        sub.height = (u16)winTex;
        sub.left = u0;  sub.right = u0 + du;
        sub.top = v0;   sub.bottom = v0 + dv;

        C2D_Image view = mapImg;
        view.subtex = &sub;
        C2D_DrawImageAt(view, kRadarX, kRadarY, 0.5f, NULL,
                        kRadarPx / winTex, kRadarPx / winTex);

        float cx = kRadarX + kRadarPx * 0.5f;
        float cy = kRadarY + kRadarPx * 0.5f;
        float fx = car->forward_x;
        float fz = car->forward_z;
        u32 marker = C2D_Color32(255, 214, 40, 255);
        C2D_DrawLine(cx, cy, marker, cx + fx * 8.0f, cy + fz * 8.0f, marker, 1.5f, 0.5f);
        C2D_DrawCircleSolid(cx, cy, 0.5f, 2.5f, marker);
    } else {
        draw_cached(&g_staticText[STATIC_FAIRHAVEN], 218.f, 48.f, 0.45f, 0.45f, C2D_Color32(0, 200, 255, 255));
        draw_cached(&g_staticText[STATIC_RADAR], 218.f, 68.f, 0.40f, 0.40f, C2D_Color32(180, 200, 220, 255));
    }

    // 3. Signature Nitrous Bar
    C2D_DrawRectSolid(10, 116, 0, 300, 28, C2D_Color32(22, 28, 42, 230));
    C2D_DrawRectSolid(10, 116, 0, 300, 2, car->nitrous_active ? C2D_Color32(255, 80, 40, 255) : C2D_Color32(0, 180, 220, 255));

    float nitroW = 190.0f * car->nitrous_level;
    C2D_DrawRectSolid(95, 126, 0, 190, 8, C2D_Color32(10, 30, 45, 255));
    if (nitroW > 0.0f) {
        C2D_DrawRectSolid(95, 126, 0, nitroW, 8, car->nitrous_active ? C2D_Color32(255, 90, 40, 255) : C2D_Color32(0, 220, 255, 255));
    }
    draw_cached(&g_dynamicText[2], 16.f, 122.f, 0.42f, 0.42f, car->nitrous_active ? C2D_Color32(255, 100, 50, 255) : C2D_Color32(0, 220, 255, 255));

    // 4. Telemetry & Chameleon BND2 Status Panel
    C2D_DrawRectSolid(10, 152, 0, 300, 78, C2D_Color32(22, 28, 42, 230));

    draw_cached(&g_dynamicText[3], 16.f, 158.f, 0.44f, 0.44f,
                g_cachedDrifting ? C2D_Color32(255, 220, 0, 255) : C2D_Color32(180, 200, 220, 255));
    draw_cached(&g_bndlText, 16.f, 178.f, 0.42f, 0.42f,
                bndl_opened ? C2D_Color32(0, 220, 120, 255) : C2D_Color32(100, 180, 220, 255));
    draw_cached(&g_staticText[STATIC_CAR_LINE], 16.f, 192.f, 0.40f, 0.40f, C2D_Color32(240, 200, 40, 255));
    draw_cached(&g_staticText[STATIC_CONTROL_LINE], 16.f, 206.f, 0.36f, 0.36f, C2D_Color32(140, 160, 180, 255));
    draw_cached(&g_dynamicText[5], 14.f, 222.f, 0.34f, 0.34f, C2D_Color32(120, 210, 160, 255));

    // 5. EasyDrive Overlay
    if (easyDriveOpen) {
        C2D_DrawRectSolid(30, 36, 0, 260, 160, C2D_Color32(15, 20, 32, 245));
        C2D_DrawRectSolid(30, 36, 0, 260, 2, C2D_Color32(0, 220, 255, 255));
        C2D_DrawRectSolid(30, 194, 0, 260, 2, C2D_Color32(0, 220, 255, 255));

        draw_cached(&g_easyText[0], 42.f, 44.f, 0.52f, 0.52f, C2D_Color32(0, 220, 255, 255));

        for (int i = 0; i < 4; i++) {
            bool selected = (i == easyDriveSelection);
            if (selected) {
                C2D_DrawRectSolid(38, 70 + i * 26, 0, 244, 22, C2D_Color32(0, 120, 160, 200));
            }
            draw_cached(&g_easyText[1 + i], 46.f, 74.f + i * 26, 0.46f, 0.46f,
                        selected ? C2D_Color32(255, 255, 255, 255) : C2D_Color32(160, 180, 200, 255));
        }
        draw_cached(&g_easyText[5], 46.f, 178.f, 0.38f, 0.38f, C2D_Color32(140, 160, 180, 255));
    }

    C2D_Flush();
}
