#include "assets.h"
#include <stdio.h>
#include <string.h>

static C2D_SpriteSheet g_sheetCrest = NULL;
static C2D_SpriteSheet g_sheetEasyDrive = NULL;
static C2D_SpriteSheet g_sheetMinimap = NULL;
static C2D_SpriteSheet g_sheetSpeedo = NULL;
static C2D_SpriteSheet g_sheetWorldAtlas = NULL;
static C2D_SpriteSheet g_sheetCarAtlas = NULL;

void assets_init(void) {
    g_sheetCrest = C2D_SpriteSheetLoad("romfs:/assets/crest.t3x");
    g_sheetEasyDrive = C2D_SpriteSheetLoad("romfs:/assets/easydrive.t3x");
    g_sheetMinimap = C2D_SpriteSheetLoad("romfs:/assets/minimap.t3x");
    g_sheetSpeedo = C2D_SpriteSheetLoad("romfs:/assets/speedo.t3x");
    g_sheetWorldAtlas = C2D_SpriteSheetLoad("romfs:/assets/world_atlas.t3x");
    if (g_sheetWorldAtlas) {
        C3D_Tex* tex = C2D_SpriteSheetGetImage(g_sheetWorldAtlas, 0).tex;
        if (tex) {
            C3D_TexSetFilter(tex, GPU_LINEAR, GPU_LINEAR);
            C3D_TexSetWrap(tex, GPU_CLAMP_TO_EDGE, GPU_CLAMP_TO_EDGE);
        }
    }
    g_sheetCarAtlas = C2D_SpriteSheetLoad("romfs:/assets/car.t3x");
    if (g_sheetCarAtlas) {
        C3D_Tex* tex = C2D_SpriteSheetGetImage(g_sheetCarAtlas, 0).tex;
        if (tex) {
            C3D_TexSetFilter(tex, GPU_LINEAR, GPU_LINEAR);
            C3D_TexSetWrap(tex, GPU_CLAMP_TO_EDGE, GPU_CLAMP_TO_EDGE);
        }
    }
}

void assets_exit(void) {
    if (g_sheetCrest) { C2D_SpriteSheetFree(g_sheetCrest); g_sheetCrest = NULL; }
    if (g_sheetEasyDrive) { C2D_SpriteSheetFree(g_sheetEasyDrive); g_sheetEasyDrive = NULL; }
    if (g_sheetMinimap) { C2D_SpriteSheetFree(g_sheetMinimap); g_sheetMinimap = NULL; }
    if (g_sheetSpeedo) { C2D_SpriteSheetFree(g_sheetSpeedo); g_sheetSpeedo = NULL; }
    if (g_sheetWorldAtlas) { C2D_SpriteSheetFree(g_sheetWorldAtlas); g_sheetWorldAtlas = NULL; }
    if (g_sheetCarAtlas) { C2D_SpriteSheetFree(g_sheetCarAtlas); g_sheetCarAtlas = NULL; }
}

C3D_Tex* assets_world_atlas(void) {
    if (!g_sheetWorldAtlas || C2D_SpriteSheetCount(g_sheetWorldAtlas) == 0) return NULL;
    return C2D_SpriteSheetGetImage(g_sheetWorldAtlas, 0).tex;
}

C3D_Tex* assets_car_atlas(void) {
    if (!g_sheetCarAtlas || C2D_SpriteSheetCount(g_sheetCarAtlas) == 0) return NULL;
    return C2D_SpriteSheetGetImage(g_sheetCarAtlas, 0).tex;
}

C2D_SpriteSheet assets_get_sheet(const char* name) {
    if (!strcmp(name, "crest")) return g_sheetCrest;
    if (!strcmp(name, "easydrive")) return g_sheetEasyDrive;
    if (!strcmp(name, "minimap")) return g_sheetMinimap;
    if (!strcmp(name, "speedo")) return g_sheetSpeedo;
    return NULL;
}

C2D_Image assets_get_image(const char* sheetName, size_t index) {
    C2D_SpriteSheet sheet = assets_get_sheet(sheetName);
    if (sheet && index < C2D_SpriteSheetCount(sheet)) {
        return C2D_SpriteSheetGetImage(sheet, index);
    }
    C2D_Image empty = {0};
    return empty;
}
