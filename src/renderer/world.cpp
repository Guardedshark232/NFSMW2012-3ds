#include "world.h"
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <malloc.h>
#include <vector>

static std::vector<WorldVertex> g_segment_verts;
static void* g_world_vbo = NULL;
static size_t g_num_verts = 0;

static const float kSegmentLength = 300.0f; // Repeating module size in meters

static void add_quad(std::vector<WorldVertex>& v,
                     float x1, float y1, float z1,
                     float x2, float y2, float z2,
                     float x3, float y3, float z3,
                     float x4, float y4, float z4,
                     float r, float g, float b) {
    WorldVertex v1 = { {x1, y1, z1}, {r, g, b} };
    WorldVertex v2 = { {x2, y2, z2}, {r, g, b} };
    WorldVertex v3 = { {x3, y3, z3}, {r, g, b} };
    WorldVertex v4 = { {x4, y4, z4}, {r, g, b} };

    v.push_back(v1); v.push_back(v2); v.push_back(v3);
    v.push_back(v1); v.push_back(v3); v.push_back(v4);
}

void world_init(void) {
    g_segment_verts.clear();

    float road_width = 11.0f;
    float step = 5.0f;

    // 1. Build road surface, lane markings, and barriers for a single chunk
    for (float z = -kSegmentLength; z < 0.0f; z += step) {
        float z0 = z;
        float z1 = z + step;

        // Dark asphalt pavement (Criterion Most Wanted style)
        add_quad(g_segment_verts,
                 -road_width, -0.5f, z0,
                  road_width, -0.5f, z0,
                  road_width, -0.5f, z1,
                 -road_width, -0.5f, z1,
                 0.12f, 0.12f, 0.14f);

        // Yellow median divider
        float yw = 0.22f;
        add_quad(g_segment_verts,
                 -yw, -0.49f, z0,
                  yw, -0.49f, z0,
                  yw, -0.49f, z1,
                 -yw, -0.49f, z1,
                 0.95f, 0.82f, 0.05f);

        // Dashed white lane dividers (Left Lane: x = -5.5m, Right Lane: x = +5.5m)
        int dash_idx = (int)(std::abs(z0) / step);
        if (dash_idx % 2 == 0) {
            float dw = 0.15f;
            // Left lane stripe
            add_quad(g_segment_verts,
                     -5.5f - dw, -0.49f, z0,
                     -5.5f + dw, -0.49f, z0,
                     -5.5f + dw, -0.49f, z1,
                     -5.5f - dw, -0.49f, z1,
                     0.90f, 0.90f, 0.92f);
            // Right lane stripe
            add_quad(g_segment_verts,
                      5.5f - dw, -0.49f, z0,
                      5.5f + dw, -0.49f, z0,
                      5.5f + dw, -0.49f, z1,
                      5.5f - dw, -0.49f, z1,
                     0.90f, 0.90f, 0.92f);
        }

        // Left concrete crash barrier
        float lx = -road_width;
        float lr = (dash_idx % 4 == 0) ? 0.8f : 0.6f;
        float lg = (dash_idx % 4 == 0) ? 0.2f : 0.6f;
        float lb = (dash_idx % 4 == 0) ? 0.2f : 0.6f;
        add_quad(g_segment_verts,
                 lx - 0.6f, -0.5f, z0,
                 lx,        0.7f,  z0,
                 lx,        0.7f,  z1,
                 lx - 0.6f, -0.5f, z1,
                 lr, lg, lb);

        // Right concrete crash barrier
        float rx = road_width;
        add_quad(g_segment_verts,
                 rx,        0.7f,  z0,
                 rx + 0.6f, -0.5f, z0,
                 rx + 0.6f, -0.5f, z1,
                 rx,        0.7f,  z1,
                 lr, lg, lb);

        // Streetlights every 50 meters
        if ((int)std::abs(z0) % 50 == 0) {
            // Pole
            float pz = z0;
            add_quad(g_segment_verts,
                     -road_width - 1.2f, -0.5f, pz - 0.15f,
                     -road_width - 1.2f,  7.0f, pz - 0.15f,
                     -road_width - 1.2f,  7.0f, pz + 0.15f,
                     -road_width - 1.2f, -0.5f, pz + 0.15f,
                     0.4f, 0.45f, 0.5f);
            // Light head extending over lane
            add_quad(g_segment_verts,
                     -road_width - 1.2f, 7.0f, pz - 0.15f,
                     -road_width + 1.5f, 6.8f, pz - 0.15f,
                     -road_width + 1.5f, 6.8f, pz + 0.15f,
                     -road_width - 1.2f, 7.0f, pz + 0.15f,
                     0.95f, 0.95f, 0.7f);
        }
    }

    // 2. Overhead Highway Gantry Sign ("Fairhaven - Hughes Park / I-92")
    float sz = -80.0f;
    // Left & Right Pillars
    add_quad(g_segment_verts,
             -road_width - 0.8f, -0.5f, sz - 0.2f,
             -road_width - 0.8f,  7.5f, sz - 0.2f,
             -road_width - 0.8f,  7.5f, sz + 0.2f,
             -road_width - 0.8f, -0.5f, sz + 0.2f,
             0.35f, 0.38f, 0.42f);

    add_quad(g_segment_verts,
              road_width + 0.8f, -0.5f, sz - 0.2f,
              road_width + 0.8f,  7.5f, sz - 0.2f,
              road_width + 0.8f,  7.5f, sz + 0.2f,
              road_width + 0.8f, -0.5f, sz + 0.2f,
              0.35f, 0.38f, 0.42f);

    // Green Signboard
    add_quad(g_segment_verts,
             -7.5f, 4.5f, sz,
              7.5f, 4.5f, sz,
              7.5f, 7.2f, sz,
             -7.5f, 7.2f, sz,
             0.05f, 0.48f, 0.22f);

    g_num_verts = g_segment_verts.size();

    g_world_vbo = linearAlloc(sizeof(WorldVertex) * g_num_verts);
    if (g_world_vbo) {
        memcpy(g_world_vbo, g_segment_verts.data(), sizeof(WorldVertex) * g_num_verts);
    }
}

void world_render(const C3D_Mtx* projection, const C3D_Mtx* view, float car_z) {
    if (!g_world_vbo || g_num_verts == 0) return;

    extern int g_uLoc_mvp;

    C3D_BufInfo* bufInfo = C3D_GetBufInfo();
    BufInfo_Init(bufInfo);
    BufInfo_Add(bufInfo, g_world_vbo, sizeof(WorldVertex), 2, 0x10);

    // Calculate which repeating chunk the car is in
    float base_chunk_z = std::floor(car_z / kSegmentLength) * kSegmentLength;

    C3D_Mtx mvp_base, mv, mvp, model;
    Mtx_Identity(&mvp_base);
    Mtx_Multiply(&mvp_base, projection, view);

    // Render current chunk, next chunk ahead, and one chunk behind
    for (int offset_idx = -1; offset_idx <= 2; offset_idx++) {
        float chunk_offset_z = base_chunk_z + (float)offset_idx * kSegmentLength;

        Mtx_Identity(&model);
        Mtx_Translate(&model, 0.0f, 0.0f, chunk_offset_z, true);

        Mtx_Multiply(&mvp, &mvp_base, &model);

        C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, g_uLoc_mvp, &mvp);
        C3D_DrawArrays(GPU_TRIANGLES, 0, g_num_verts);
    }
}

void world_exit(void) {
    if (g_world_vbo) {
        linearFree(g_world_vbo);
        g_world_vbo = NULL;
    }
    g_num_verts = 0;
}
