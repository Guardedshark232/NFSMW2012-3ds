#include "hud.h"
#include "assets.h"
#include "renderer.h"
#include "world_mesh.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

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
static C2D_Font g_font = NULL;

void hud_init(void) {
    g_staticBuf = C2D_TextBufNew(4096);

    // Load bundled UI font from RomFS (never crashes if system font is missing)
    g_font = C2D_FontLoad("romfs:/assets/helvetica.bcfnt");
    if (!g_font) {
        g_font = C2D_FontLoad("romfs:/assets/conduit.bcfnt");
    }
    if (!g_font) {
        g_font = C2D_FontLoadSystem(CFG_REGION_USA);
    }
}

void hud_exit(void) {
    if (g_font) {
        C2D_FontFree(g_font);
        g_font = NULL;
    }
    if (g_staticBuf) {
        C2D_TextBufDelete(g_staticBuf);
        g_staticBuf = NULL;
    }
}

static inline void draw_string(const char* str, float x, float y, float scaleX, float scaleY, u32 color) {
    if (!g_staticBuf || !str || !str[0]) return;
    C2D_Text text;
    if (g_font) {
        C2D_TextFontParse(&text, g_font, g_staticBuf, str);
    } else {
        C2D_TextParse(&text, g_staticBuf, str);
    }
    C2D_TextOptimize(&text);
    C2D_DrawText(&text, C2D_WithColor, x, y, 0.5f, scaleX, scaleY, color);
}

void hud_render(C3D_RenderTarget* target, const CarPhysicsState* car, bool easyDriveOpen, int easyDriveSelection, bool bndl_opened, uint32_t bndl_entries, float fps) {
    if (!target) return;

    C2D_Prepare();
    C2D_TargetClear(target, C2D_Color32(14, 18, 28, 255));
    C2D_SceneBegin(target);

    C2D_TextBufClear(g_staticBuf);

    // 1. Top Header Banner
    C2D_DrawRectSolid(0, 0, 0, 320, 24, C2D_Color32(20, 25, 38, 255));
    C2D_DrawRectSolid(0, 24, 0, 320, 2, C2D_Color32(0, 200, 255, 255));
    draw_string("NFS: MOST WANTED (2012) - FAIRHAVEN 3DS", 10.f, 4.f, 0.52f, 0.52f, C2D_Color32(255, 255, 255, 255));

    // Porsche crest icon
    C2D_Image crestImg = assets_get_image("crest", 0);
    if (crestImg.tex) {
        C2D_DrawImageAt(crestImg, 282.f, 2.f, 0.5f, NULL, 0.35f, 0.35f);
    }

    // 2. Speedometer & Tachometer Panel
    C2D_DrawRectSolid(10, 34, 0, 185, 74, C2D_Color32(22, 28, 42, 230));
    C2D_DrawRectSolid(10, 34, 0, 185, 2, C2D_Color32(0, 180, 220, 255));

    char speedBuf[64];
    snprintf(speedBuf, sizeof(speedBuf), "%5.1f", car->speed);
    draw_string(speedBuf, 16.f, 40.f, 0.95f, 0.95f, C2D_Color32(255, 255, 255, 255));
    draw_string("km/h", 125.f, 52.f, 0.45f, 0.45f, C2D_Color32(0, 200, 255, 255));

    char gearBuf[64];
    snprintf(gearBuf, sizeof(gearBuf), "GEAR %d   |   %4.0f RPM", car->gear, car->rpm);
    draw_string(gearBuf, 16.f, 80.f, 0.48f, 0.48f, C2D_Color32(0, 220, 255, 255));

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
        float fx = -sinf(car->heading);
        float fz = -cosf(car->heading);
        u32 marker = C2D_Color32(255, 214, 40, 255);
        C2D_DrawLine(cx, cy, marker, cx + fx * 8.0f, cy + fz * 8.0f, marker, 1.5f, 0.5f);
        C2D_DrawCircleSolid(cx, cy, 0.5f, 2.5f, marker);
    } else {
        draw_string("FAIRHAVEN", 218.f, 48.f, 0.45f, 0.45f, C2D_Color32(0, 200, 255, 255));
        draw_string("RADAR MAP", 218.f, 68.f, 0.40f, 0.40f, C2D_Color32(180, 200, 220, 255));
    }

    // 3. Signature Nitrous Bar
    C2D_DrawRectSolid(10, 116, 0, 300, 28, C2D_Color32(22, 28, 42, 230));
    C2D_DrawRectSolid(10, 116, 0, 300, 2, car->nitrous_active ? C2D_Color32(255, 80, 40, 255) : C2D_Color32(0, 180, 220, 255));

    float nitroW = 190.0f * car->nitrous_level;
    C2D_DrawRectSolid(95, 126, 0, 190, 8, C2D_Color32(10, 30, 45, 255));
    if (nitroW > 0.0f) {
        C2D_DrawRectSolid(95, 126, 0, nitroW, 8, car->nitrous_active ? C2D_Color32(255, 90, 40, 255) : C2D_Color32(0, 220, 255, 255));
    }
    char nitroBuf[32];
    snprintf(nitroBuf, sizeof(nitroBuf), "NITROUS %3.0f%%", car->nitrous_level * 100.0f);
    draw_string(nitroBuf, 16.f, 122.f, 0.42f, 0.42f, car->nitrous_active ? C2D_Color32(255, 100, 50, 255) : C2D_Color32(0, 220, 255, 255));

    // 4. Telemetry & Chameleon BND2 Status Panel
    C2D_DrawRectSolid(10, 152, 0, 300, 78, C2D_Color32(22, 28, 42, 230));

    char driftBuf[128];
    if (car->is_drifting) {
        snprintf(driftBuf, sizeof(driftBuf), ">> POWER-SLIDE DRIFT ACTIVE! Angle: %4.1f deg <<", car->drift_angle * 57.3f);
        draw_string(driftBuf, 16.f, 158.f, 0.44f, 0.44f, C2D_Color32(255, 220, 0, 255));
    } else {
        snprintf(driftBuf, sizeof(driftBuf), "Distance: %6.1f m   |   Perf: %4.1f FPS (%4.1f ms)", car->distance, fps, 1000.0f / (fps > 0 ? fps : 60.0f));
        draw_string(driftBuf, 16.f, 158.f, 0.44f, 0.44f, C2D_Color32(180, 200, 220, 255));
    }

    char bndlBuf[128];
    if (bndl_opened) {
        snprintf(bndlBuf, sizeof(bndlBuf), "CHAMELEON 2: WORLDLIST.BNDL (%lu chunks loaded)", (unsigned long)bndl_entries);
        draw_string(bndlBuf, 16.f, 178.f, 0.42f, 0.42f, C2D_Color32(0, 220, 120, 255));
    } else {
        draw_string("CHAMELEON 2: SD / RomFS Streaming Ready", 16.f, 178.f, 0.42f, 0.42f, C2D_Color32(100, 180, 220, 255));
    }

    draw_string("Porsche 911 Carrera S (2012)  |  Fairhaven City", 16.f, 192.f, 0.40f, 0.40f, C2D_Color32(240, 200, 40, 255));
    draw_string("Press [START] EasyDrive | [SELECT] Reset car", 16.f, 206.f, 0.36f, 0.36f, C2D_Color32(140, 160, 180, 255));

    // Live world telemetry: makes ground/camera faults visible on hardware.
    char dbgBuf[128];
    snprintf(dbgBuf, sizeof(dbgBuf), "POS %6.1f %6.1f  GND %6.2f  CAM %6.2f  CH %d/%d",
             car->pos_x, car->pos_z,
             renderer_debug_car_y(), renderer_debug_cam_y(),
             world_mesh_chunk_count(), world_mesh_chunk_total());
    draw_string(dbgBuf, 14.f, 222.f, 0.34f, 0.34f, C2D_Color32(120, 210, 160, 255));

    // 5. EasyDrive Overlay
    if (easyDriveOpen) {
        C2D_DrawRectSolid(30, 36, 0, 260, 160, C2D_Color32(15, 20, 32, 245));
        C2D_DrawRectSolid(30, 36, 0, 260, 2, C2D_Color32(0, 220, 255, 255));
        C2D_DrawRectSolid(30, 194, 0, 260, 2, C2D_Color32(0, 220, 255, 255));

        draw_string("EASYDRIVE - QUICK MENU", 42.f, 44.f, 0.52f, 0.52f, C2D_Color32(0, 220, 255, 255));

        const char* items[4] = {
            "1. Find Race: Hughes Park Sprint",
            "2. Change Car: Porsche 911 Carrera S",
            "3. Pro Mods: Burnout Nitrous Setup",
            "4. Map: Fairhaven Open World"
        };
        for (int i = 0; i < 4; i++) {
            bool selected = (i == easyDriveSelection);
            if (selected) {
                C2D_DrawRectSolid(38, 70 + i * 26, 0, 244, 22, C2D_Color32(0, 120, 160, 200));
            }
            draw_string(items[i], 46.f, 74.f + i * 26, 0.46f, 0.46f,
                        selected ? C2D_Color32(255, 255, 255, 255) : C2D_Color32(160, 180, 200, 255));
        }
        draw_string("Press B to navigate | START to close", 46.f, 178.f, 0.38f, 0.38f, C2D_Color32(140, 160, 180, 255));
    }

    C2D_Flush();
}
