#include "renderer.h"
#include "physics.h"
#include "world_mesh.h"
#include "assets.h"
#include "hud.h"
#include "fbdump.h"
#include <citro2d.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <cmath>
#include <vector>

extern "C" {
    extern const u8 vshader_shbin[];
    extern const u32 vshader_shbin_size;
}

#define DISPLAY_TRANSFER_FLAGS \
	(GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) | \
	GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) | \
	GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO))

typedef struct {
    float position[3];
    uint8_t color[4];
} Vertex;

static inline uint8_t color_byte(float v) {
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return 255;
    return (uint8_t)(v * 255.0f + 0.5f);
}

static C3D_RenderTarget* g_topTarget = NULL;
static C3D_RenderTarget* g_botTarget = NULL;

static DVLB_s* g_vshader_dvlb = NULL;
static shaderProgram_s g_program;
int g_uLoc_mvp = -1;

static void* g_car_data = NULL;
static size_t g_car_data_size = 0;
static const uint8_t* g_car_body_verts = NULL;
static const uint16_t* g_car_body_indices = NULL;
static const uint8_t* g_car_wheel_verts = NULL;
static const uint16_t* g_car_wheel_indices = NULL;
static uint32_t g_car_body_icount = 0;
static uint32_t g_car_wheel_icount = 0;
static float g_car_wheel_radius = 0.34f;
static float g_car_wheel_track = 0.78f;
static float g_car_wheel_base = 1.25f;
static float g_car_wheel_y = 0.34f;
static float g_car_wheel_spin = 0.0f;

static void* g_flame_vbo = NULL;
static size_t g_flame_num_verts = 0;

// Full-screen dusk gradient drawn behind the world.
static void* g_sky_vbo = NULL;
static size_t g_sky_num_verts = 0;

// Harbour water plane, fills the sea inlets where the ARENA bundle has no
// geometry.  Baked at the world water level (spawn apron is at y=-13.6).
static void* g_water_vbo = NULL;
static size_t g_water_num_verts = 0;
static const float WATER_LEVEL = -17.0f;
static const float SPAWN_GROUND_Y = -13.6f;

// Camera state with smooth interpolation
static float g_cam_x = 0.0f;
static float g_cam_y = 0.0f;
static float g_cam_z = 0.0f;
static float g_cam_heading = 0.0f;
static float g_cam_fov = 60.0f;
static bool  g_cam_valid = false;

static float g_car_y = 0.0f;
static bool  g_car_y_valid = false;

// Imported 911 body plus one centred wheel block, written by
// tools/extract_vehicle.py.  The runtime instances the wheel four times so
// it can steer and spin without rebuilding vertices per frame.
struct CarHeader {
    char magic[4];
    uint32_t version;
    uint32_t body_verts, body_indices;
    uint32_t wheel_verts, wheel_indices;
    float wheel_radius, wheel_half_track, wheel_halfbase, wheel_centre_y;
    uint32_t reserved;
};
static_assert(sizeof(CarHeader) == 44, "packed car header must be 44 bytes");

static bool load_car_geometry(void) {
    FILE* f = fopen("romfs:/assets/car.mwcar", "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size < (long)sizeof(CarHeader)) { fclose(f); return false; }

    void* data = linearAlloc((size + 0x3F) & ~0x3F);
    if (!data) { fclose(f); return false; }
    size_t got = fread(data, 1, (size_t)size, f);
    fclose(f);

    const CarHeader* h = (const CarHeader*)data;
    size_t need = sizeof(CarHeader)
                + (size_t)h->body_indices * 2 + (size_t)h->body_verts * 20
                + (size_t)h->wheel_indices * 2 + (size_t)h->wheel_verts * 20;
    if (got != (size_t)size || memcmp(h->magic, "MWCR", 4) != 0 ||
        h->version != 2 || need > (size_t)size) {
        linearFree(data);
        return false;
    }

    const uint8_t* p = (const uint8_t*)data + sizeof(CarHeader);
    g_car_body_indices = (const uint16_t*)p;
    p += (size_t)h->body_indices * 2;
    g_car_body_verts = p;
    p += (size_t)h->body_verts * 20;
    g_car_wheel_indices = (const uint16_t*)p;
    p += (size_t)h->wheel_indices * 2;
    g_car_wheel_verts = p;

    g_car_body_icount = h->body_indices;
    g_car_wheel_icount = h->wheel_indices;
    g_car_wheel_radius = h->wheel_radius;
    g_car_wheel_track = h->wheel_half_track;
    g_car_wheel_base = h->wheel_halfbase;
    g_car_wheel_y = h->wheel_centre_y;
    g_car_data = data;
    g_car_data_size = need;

    GSPGPU_FlushDataCache(g_car_data, g_car_data_size);
    return true;
}

static void build_flame_geometry(void) {
    std::vector<Vertex> verts;
    uint8_t fr = color_byte(0.0f), fg = color_byte(0.85f), fb = color_byte(1.0f);
    uint8_t cr = color_byte(0.9f), cg = color_byte(0.98f), cb = color_byte(1.0f);

    // Left Exhaust Flame Jet (Aligned with twin exhaust pipes)
    float lx = -0.45f, y = 0.12f, z = -2.18f;
    verts.push_back({ {lx - 0.10f, y, z}, {fr, fg, fb, 255} });
    verts.push_back({ {lx + 0.10f, y, z}, {fr, fg, fb, 255} });
    verts.push_back({ {lx, y, z - 0.95f}, {cr, cg, cb, 255} });

    // Right Exhaust Flame Jet
    float rx = 0.45f;
    verts.push_back({ {rx - 0.10f, y, z}, {fr, fg, fb, 255} });
    verts.push_back({ {rx + 0.10f, y, z}, {fr, fg, fb, 255} });
    verts.push_back({ {rx, y, z - 0.95f}, {cr, cg, cb, 255} });

    g_flame_num_verts = verts.size();
    g_flame_vbo = linearAlloc(sizeof(Vertex) * g_flame_num_verts);
    if (g_flame_vbo) {
        memcpy(g_flame_vbo, verts.data(), sizeof(Vertex) * g_flame_num_verts);
        GSPGPU_FlushDataCache(g_flame_vbo, sizeof(Vertex) * g_flame_num_verts);
    }
}

static void build_sky_geometry(void) {
    // View-space sky backdrop.  The quad sits 900 m in front of the eye and
    // is drawn with the projection matrix alone: Mtx_PerspTilt already
    // contains the rotation that maps view +Y to screen up on the 3DS's
    // 240x400 tilted framebuffer, so the gradient stays upright.
    std::vector<Vertex> verts;
    uint8_t zen_r = color_byte(0.04f), zen_g = color_byte(0.07f), zen_b = color_byte(0.16f);
    uint8_t mid_r = color_byte(0.32f), mid_g = color_byte(0.24f), mid_b = color_byte(0.42f);
    uint8_t hor_r = color_byte(0.95f), hor_g = color_byte(0.52f), hor_b = color_byte(0.26f);
    uint8_t bot_r = color_byte(0.09f), bot_g = color_byte(0.12f), bot_b = color_byte(0.17f);

    const float z = -900.0f;
    const float x = 3000.0f;

    auto band = [&](float y0, uint8_t r0, uint8_t g0, uint8_t b0,
                    float y1, uint8_t r1, uint8_t g1, uint8_t b1) {
        Vertex a = { {-x, y0, z}, {r0, g0, b0, 255} };
        Vertex b = { { x, y0, z}, {r0, g0, b0, 255} };
        Vertex c = { { x, y1, z}, {r1, g1, b1, 255} };
        Vertex d = { {-x, y1, z}, {r1, g1, b1, 255} };
        verts.push_back(a); verts.push_back(b); verts.push_back(c);
        verts.push_back(a); verts.push_back(c); verts.push_back(d);
    };

    band( 900.0f, zen_r, zen_g, zen_b, 190.0f, mid_r, mid_g, mid_b);
    band( 190.0f, mid_r, mid_g, mid_b,   0.0f, hor_r, hor_g, hor_b);
    band(   0.0f, hor_r, hor_g, hor_b, -900.0f, bot_r, bot_g, bot_b);

    g_sky_num_verts = verts.size();
    g_sky_vbo = linearAlloc(sizeof(Vertex) * g_sky_num_verts);
    if (g_sky_vbo) {
        memcpy(g_sky_vbo, verts.data(), sizeof(Vertex) * g_sky_num_verts);
        GSPGPU_FlushDataCache(g_sky_vbo, sizeof(Vertex) * g_sky_num_verts);
    }
}

static void build_water_geometry(void) {
    // One huge quad at the harbour water level.  Vertex colours warm towards
    // the horizon so the sea reads as dusk water rather than flat paint.
    std::vector<Vertex> verts;
    uint8_t near_r = color_byte(0.05f), near_g = color_byte(0.12f), near_b = color_byte(0.16f);
    uint8_t far_r  = color_byte(0.10f), far_g  = color_byte(0.17f), far_b  = color_byte(0.22f);

    const float s = 1600.0f;
    auto v = [&](float x, float z, uint8_t r, uint8_t g, uint8_t b) {
        Vertex vert = { {x, WATER_LEVEL, z}, {r, g, b, 255} };
        verts.push_back(vert);
    };
    // Two bands: inner (near) dark, outer ring lighter/hazier.
    v(-s,  s, far_r, far_g, far_b);  v( s,  s, far_r, far_g, far_b);
    v( s, -s, near_r, near_g, near_b); v(-s, -s, near_r, near_g, near_b);
    // Triangulate as (0,1,2) + (0,2,3)
    Vertex a = verts[0], b = verts[1], c = verts[2], d = verts[3];
    verts.clear();
    verts.push_back(a); verts.push_back(b); verts.push_back(c);
    verts.push_back(a); verts.push_back(c); verts.push_back(d);

    g_water_num_verts = verts.size();
    g_water_vbo = linearAlloc(sizeof(Vertex) * g_water_num_verts);
    if (g_water_vbo) {
        memcpy(g_water_vbo, verts.data(), sizeof(Vertex) * g_water_num_verts);
        GSPGPU_FlushDataCache(g_water_vbo, sizeof(Vertex) * g_water_num_verts);
    }
}

void renderer_init(void) {
    gfxInitDefault();
    C3D_Init(C3D_DEFAULT_CMDBUF_SIZE);
    C2D_Init(C2D_DEFAULT_MAX_OBJECTS);
    C2D_Prepare();

    g_topTarget = C3D_RenderTargetCreate(240, 400, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
    C3D_RenderTargetSetOutput(g_topTarget, GFX_TOP, GFX_LEFT, DISPLAY_TRANSFER_FLAGS);

    g_botTarget = C3D_RenderTargetCreate(240, 320, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
    C3D_RenderTargetSetOutput(g_botTarget, GFX_BOTTOM, GFX_LEFT, DISPLAY_TRANSFER_FLAGS);

    C3D_CullFace(GPU_CULL_NONE);
    C3D_DepthTest(true, GPU_GREATER, GPU_WRITE_ALL);

    C3D_TexEnv* env = C3D_GetTexEnv(0);
    C3D_TexEnvInit(env);
    C3D_TexEnvSrc(env, C3D_Both, GPU_PRIMARY_COLOR, GPU_PRIMARY_COLOR, (GPU_TEVSRC)0);
    C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);

    g_vshader_dvlb = DVLB_ParseFile((u32*)vshader_shbin, vshader_shbin_size);
    shaderProgramInit(&g_program);
    shaderProgramSetVsh(&g_program, &g_vshader_dvlb->DVLE[0]);
    C3D_BindProgram(&g_program);

    g_uLoc_mvp = shaderInstanceGetUniformLocation(g_program.vertexShader, "mvp");

    C3D_AttrInfo* attrInfo = C3D_GetAttrInfo();
    AttrInfo_Init(attrInfo);
    AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 3); // position
    AttrInfo_AddLoader(attrInfo, 1, GPU_UNSIGNED_BYTE, 4); // normalised color
    AttrInfo_AddLoader(attrInfo, 2, GPU_SHORT, 2); // biased atlas UV

    static_assert(sizeof(Vertex) == 16, "packed vertex must be 16 bytes");

    load_car_geometry();
    build_flame_geometry();
    build_sky_geometry();
    build_water_geometry();
    world_mesh_init();
    // world_mesh_write_diag();  // diagnostic: SD-card world upload hashes
    assets_init();
    hud_init();
    fbdump_init();
}

void renderer_exit(void) {
    hud_exit();
    assets_exit();
    world_mesh_exit();
    if (g_car_data) { linearFree(g_car_data); g_car_data = NULL; }
    if (g_flame_vbo) { linearFree(g_flame_vbo); g_flame_vbo = NULL; }
    if (g_sky_vbo) { linearFree(g_sky_vbo); g_sky_vbo = NULL; }
    if (g_water_vbo) { linearFree(g_water_vbo); g_water_vbo = NULL; }

    shaderProgramFree(&g_program);
    if (g_vshader_dvlb) DVLB_Free(g_vshader_dvlb);

    if (g_topTarget) C3D_RenderTargetDelete(g_topTarget);
    if (g_botTarget) C3D_RenderTargetDelete(g_botTarget);
    C2D_Fini();
    C3D_Fini();
    gfxExit();
}

void renderer_begin_frame(void) {
    C3D_FrameBegin(C3D_FRAME_SYNCDRAW);
}

void renderer_render_world_and_car(float dt) {
    // Restore 3D GPU state (citro2d overwrites shader, attrInfo, texenv, depth/blend)
    C3D_BindProgram(&g_program);

    C3D_AttrInfo* attrInfo = C3D_GetAttrInfo();
    AttrInfo_Init(attrInfo);
    AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 3); // position
    AttrInfo_AddLoader(attrInfo, 1, GPU_UNSIGNED_BYTE, 4); // normalised color
    AttrInfo_AddLoader(attrInfo, 2, GPU_SHORT, 2); // biased atlas UV

    C3D_TexEnv* env = C3D_GetTexEnv(0);
    C3D_TexEnvInit(env);
    C3D_TexEnvSrc(env, C3D_Both, GPU_PRIMARY_COLOR, (GPU_TEVSRC)0, (GPU_TEVSRC)0);
    C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);
    C3D_TexEnvInit(C3D_GetTexEnv(1));

    C3D_CullFace(GPU_CULL_NONE);
    // Reversed-Z: projection maps near to NDC -1, far to 0; DepthMap(-1,0)
    // turns that into 1 at the near plane and 0 at the far plane, matching
    // the 0 depth clear and GPU_GREATER test.  citro2d can leave the depth
    // mapping changed, so set it explicitly every frame.
    C3D_DepthMap(true, -1.0f, 0.0f);
    C3D_DepthTest(true, GPU_GREATER, GPU_WRITE_ALL);
    C3D_AlphaBlend(GPU_BLEND_ADD, GPU_BLEND_ADD, GPU_ONE, GPU_ZERO, GPU_ONE, GPU_ZERO);

    C3D_FrameDrawOn(g_topTarget);
    // Fairhaven dusk sky gradient -- depth clear 0 for GPU_GREATER (near-wins)
    C3D_RenderTargetClear(g_topTarget, C3D_CLEAR_ALL, 0x121A28FF, 0);

    const CarPhysicsState* car = physics_get_state();

    // Sample the ground across a 5-point car footprint (center + 4 inner wheel
    // corners) so a single missing 2 m cell or ramp seam never drops the car
    // through the floor.
    float ref_y = g_car_y_valid ? g_car_y : SPAWN_GROUND_Y;
    float ground = world_mesh_ground_height(car->pos_x, car->pos_z, ref_y, ref_y);
    float fx = car->forward_x * 0.95f;
    float fz = car->forward_z * 0.95f;
    float rx = -car->forward_z * 0.55f;
    float rz = car->forward_x * 0.55f;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            float sx_pos = car->pos_x + sz * fx + sx * rx;
            float sz_pos = car->pos_z + sz * fz + sx * rz;
            float sample = world_mesh_ground_height(sx_pos, sz_pos, ref_y, -1e9f);
            if (sample > -1e8f && sample <= ref_y + 1.35f && sample > ground) {
                ground = sample;
            }
        }
    }
    if (!g_car_y_valid) {
        g_car_y = ground;
        g_car_y_valid = true;
    } else {
        float dy = ground - g_car_y;
        float step = (dt > 0.0f) ? dt : 0.016f;
        if (dy < 0.0f) {
            float max_down = 12.0f * step + 0.02f;  // ~12 m/s descent
            if (dy < -max_down) dy = -max_down;
        } else {
            float max_up = 25.0f * step + 0.05f;    // track fast ramps; roofs rejected by GROUND_STEP_UP
            if (dy > max_up) dy = max_up;
        }
        g_car_y += dy;
    }
    // Never settle below the harbour waterline.
    if (g_car_y < WATER_LEVEL + 0.6f) g_car_y = WATER_LEVEL + 0.6f;
    physics_set_ground_height(g_car_y);

    // Smooth FOV widening under nitrous.
    float fov_k = (dt > 0.0f) ? fminf(1.0f, dt * 6.0f) : 1.0f;
    float target_fov = car->nitrous_active ? 72.0f : 60.0f;
    g_cam_fov += (target_fov - g_cam_fov) * fov_k;

    C3D_Mtx projection, camera;
    Mtx_PerspTilt(&projection, C3D_AngleFromDegrees(g_cam_fov), C3D_AspectRatioTop, 0.05f, 1000.0f, false);

    // Dusk gradient sky: view-space backdrop drawn with the projection
    // matrix only, so it is upright regardless of framebuffer rotation.
    if (g_sky_vbo && g_sky_num_verts > 0) {
        C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, g_uLoc_mvp, &projection);
        C3D_BufInfo* skyInfo = C3D_GetBufInfo();
        BufInfo_Init(skyInfo);
        BufInfo_Add(skyInfo, g_sky_vbo, sizeof(Vertex), 2, 0x10);
        C3D_DepthTest(false, GPU_ALWAYS, GPU_WRITE_COLOR);
        C3D_DrawArrays(GPU_TRIANGLES, 0, g_sky_num_verts);
        C3D_DepthTest(true, GPU_GREATER, GPU_WRITE_ALL);
    }

    // Heading-following chase camera. heading 0 faces -Z.
    float cam_dist = car->nitrous_active ? 6.4f : 5.4f;
    float cam_height = 2.2f;

    // Smooth the camera's heading, not its world position: a positional
    // lerp leaves a speed-proportional gap (the car shrank away at 295 km/h),
    // while a heading lerp keeps a fixed chase distance and still smooths
    // steering changes.
    float step = (dt > 0.0f) ? dt : 0.016f;
    if (!g_cam_valid) {
        g_cam_heading = car->heading;
    } else {
        float dh = car->heading - g_cam_heading;
        while (dh > 3.1415927f) dh -= 6.2831853f;
        while (dh < -3.1415927f) dh += 6.2831853f;
        g_cam_heading += dh * (1.0f - expf(-9.0f * step));
    }

    float csh = sinf(g_cam_heading);
    float cch = cosf(g_cam_heading);
    float eye_x = car->pos_x + csh * cam_dist;
    float eye_y = g_car_y + cam_height;
    float eye_z = car->pos_z + cch * cam_dist;

    if (!g_cam_valid) {
        g_cam_x = eye_x; g_cam_y = eye_y; g_cam_z = eye_z;
        g_cam_valid = true;
    } else {
        // Only the vertical follow is smoothed; x/z stay glued behind the car
        // so the frame does not widen as speed rises.
        g_cam_y += (eye_y - g_cam_y) * (1.0f - expf(-14.0f * step));
        g_cam_x = eye_x;
        g_cam_z = eye_z;
    }

    float tgt_x = car->pos_x - csh * 7.0f;
    float tgt_y = g_car_y + 1.0f;
    float tgt_z = car->pos_z - cch * 7.0f;
    Mtx_LookAt(&camera,
               FVec4_New(g_cam_x, g_cam_y, g_cam_z, 1.0f),
               FVec4_New(tgt_x, tgt_y, tgt_z, 1.0f),
               FVec4_New(0.0f, 1.0f, 0.0f, 0.0f), false);

    // Render Fairhaven world.  The world samples the baked atlas and
    // modulates it by the vertex lighting tint; later draws (water, car)
    // go back to untextured REPLACE.
    C3D_Tex* atlas = assets_world_atlas();
    if (atlas) {
        C3D_TexBind(0, atlas);
        C3D_TexEnvSrc(env, C3D_Both, GPU_TEXTURE0, GPU_PRIMARY_COLOR, GPU_PRIMARY_COLOR);
        C3D_TexEnvFunc(env, C3D_Both, GPU_MODULATE);
        // The sky/water draws already flushed the stage's dirty bit.  The
        // inline TexEnv setters do not re-flag it, so without this the GPU
        // keeps the previous (untextured) combiner for the world draw.
        C3D_DirtyTexEnv(env);
    }
    float stream_x = car->reversing ? -car->forward_x : car->forward_x;
    float stream_z = car->reversing ? -car->forward_z : car->forward_z;
    world_mesh_render(&projection, &camera, car->pos_x, car->pos_z, stream_x, stream_z);
    if (atlas) {
        C3D_TexEnvInit(env);
        C3D_TexEnvSrc(env, C3D_Both, GPU_PRIMARY_COLOR, (GPU_TEVSRC)0, (GPU_TEVSRC)0);
        C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);
        C3D_DirtyTexEnv(env);
    }

    // Harbour water fills the sea inlets where the source bundle has no
    // geometry, so holes in the quays read as water instead of sky.
    if (g_water_vbo && g_water_num_verts > 0) {
        C3D_Mtx waterMVP;
        Mtx_Multiply(&waterMVP, &projection, &camera); // identity model matrix
        C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, g_uLoc_mvp, &waterMVP);
        C3D_BufInfo* waterInfo = C3D_GetBufInfo();
        BufInfo_Init(waterInfo);
        BufInfo_Add(waterInfo, g_water_vbo, sizeof(Vertex), 2, 0x10);
        C3D_DepthTest(true, GPU_GREATER, GPU_WRITE_ALL);
        C3D_DrawArrays(GPU_TRIANGLES, 0, g_water_num_verts);
    }

    // Render the imported Porsche 911 on the harbour apron.
    if (g_car_body_verts && g_car_body_icount > 0) {
        float speed_ms = car->speed / 3.6f;
        if (car->reversing) speed_ms = -speed_ms;
        if (g_car_wheel_radius > 0.01f) {
            g_car_wheel_spin += speed_ms / g_car_wheel_radius * step;
            g_car_wheel_spin = fmodf(g_car_wheel_spin, 6.2831853f);
        }

        C3D_Tex* carTex = assets_car_atlas();
        if (carTex) {
            C3D_TexBind(0, carTex);
            C3D_TexEnvSrc(env, C3D_Both, GPU_TEXTURE0, GPU_PRIMARY_COLOR, GPU_PRIMARY_COLOR);
            C3D_TexEnvFunc(env, C3D_Both, GPU_MODULATE);
            C3D_DirtyTexEnv(env);
        }

        C3D_Mtx carModel, carMV, carMVP;
        Mtx_Identity(&carModel);
        Mtx_Translate(&carModel, car->pos_x, g_car_y, car->pos_z, true);
        Mtx_RotateY(&carModel, C3D_AngleFromDegrees(180.0f) + car->heading + car->drift_angle, true);
        Mtx_Multiply(&carMV, &camera, &carModel);
        Mtx_Multiply(&carMVP, &projection, &carMV);
        C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, g_uLoc_mvp, &carMVP);
        C3D_BufInfo* carInfo = C3D_GetBufInfo();
        BufInfo_Init(carInfo);
        BufInfo_Add(carInfo, g_car_body_verts, 20, 3, 0x210);
        // Disable culling for car (closed mesh, depth test handles occlusion).
        C3D_CullFace(GPU_CULL_NONE);
        C3D_DepthTest(true, GPU_GREATER, GPU_WRITE_ALL);
        C3D_DrawElements(GPU_TRIANGLES, g_car_body_icount, C3D_UNSIGNED_SHORT,
                         g_car_body_indices);

        // Four wheel instances: front pair steers, all four spin.  The right
        // side is the wheel mesh rotated 180 about Y so the rim faces out;
        // that also reverses its local X, hence the negated spin.
        if (g_car_wheel_verts && g_car_wheel_icount > 0) {
            C3D_BufInfo* wheelInfo = C3D_GetBufInfo();
            BufInfo_Init(wheelInfo);
            BufInfo_Add(wheelInfo, g_car_wheel_verts, 20, 3, 0x210);
            float steer = -car->steering_angle * 0.55f;
            for (int side = -1; side <= 1; side += 2) {
                for (int end = -1; end <= 1; end += 2) {
                    C3D_Mtx wheel, wheelMV, wheelMVP;
                    Mtx_Identity(&wheel);
                    Mtx_Translate(&wheel, side * g_car_wheel_track,
                                  g_car_wheel_y, end * g_car_wheel_base, true);
                    if (end > 0) Mtx_RotateY(&wheel, steer, true);
                    if (side < 0) Mtx_RotateY(&wheel, C3D_AngleFromDegrees(180.0f), true);
                    Mtx_RotateX(&wheel, side * g_car_wheel_spin, true);
                    Mtx_Multiply(&wheelMV, &carMV, &wheel);
                    Mtx_Multiply(&wheelMVP, &projection, &wheelMV);
                    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, g_uLoc_mvp, &wheelMVP);
                    C3D_DrawElements(GPU_TRIANGLES, g_car_wheel_icount,
                                     C3D_UNSIGNED_SHORT, g_car_wheel_indices);
                }
            }
            C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, g_uLoc_mvp, &carMVP);
        }

        // Draw cyan nitrous exhaust flames when boost is active
        if (car->nitrous_active && g_flame_vbo && g_flame_num_verts > 0) {
            if (carTex) {
                C3D_TexEnvInit(env);
                C3D_TexEnvSrc(env, C3D_Both, GPU_PRIMARY_COLOR, (GPU_TEVSRC)0, (GPU_TEVSRC)0);
                C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);
                C3D_DirtyTexEnv(env);
            }
            C3D_BufInfo* flameInfo = C3D_GetBufInfo();
            BufInfo_Init(flameInfo);
            BufInfo_Add(flameInfo, g_flame_vbo, sizeof(Vertex), 2, 0x10);
            C3D_DrawArrays(GPU_TRIANGLES, 0, g_flame_num_verts);
        }
    }
}

void renderer_reset_view(void) {
    // Drop cached follow heights so a reset re-seats the car and chase
    // camera on the ground at the spawn instead of keeping a stale (and
    // possibly underground) camera position.
    g_car_y = 0.0f;
    g_car_y_valid = false;
    g_cam_valid = false;
}

float renderer_debug_car_y(void) { return g_car_y; }
float renderer_debug_cam_y(void) { return g_cam_y; }

void renderer_render_hud(bool easyDriveOpen, int easyDriveSelection, bool bndl_opened, uint32_t bndl_entries, float fps) {
    if (!g_botTarget) return;

    // Reset pipeline state for citro2d
    C3D_CullFace(GPU_CULL_NONE);
    C3D_DepthTest(false, GPU_ALWAYS, GPU_WRITE_ALL);
    C3D_AlphaBlend(GPU_BLEND_ADD, GPU_BLEND_ADD,
                   GPU_SRC_ALPHA, GPU_ONE_MINUS_SRC_ALPHA,
                   GPU_SRC_ALPHA, GPU_ONE_MINUS_SRC_ALPHA);

    const CarPhysicsState* car = physics_get_state();
    hud_render(g_botTarget, car, easyDriveOpen, easyDriveSelection, bndl_opened, bndl_entries, fps);
}

void renderer_end_frame(void) {
    C3D_FrameEnd(0);
    fbdump_tick(g_topTarget, g_botTarget);
}
