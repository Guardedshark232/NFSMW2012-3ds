#include "world_mesh.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <vector>
#include <algorithm>
#include <utility>

// Packed vertex shared with the renderer: float position + lighting tint RGBA
// + uint16 atlas UV.
#pragma pack(push, 1)
struct MWVertex {
    float position[3];
    uint8_t color[4];
    int16_t uv[2];
};

struct MWMSHeader {
    char magic[4];
    uint32_t version;
    uint32_t numChunks;
    uint32_t reserved;
};

struct MWChunkHeader {
    uint32_t numVerts;
    uint32_t numIndices;
    float bboxMin[3];
    float bboxMax[3];
    uint32_t reserved[2];
};

struct MWStreamingHeader {
    char magic[4];       // "MWST"
    uint32_t version;    // 1
    uint32_t numSectors;
    float sectorSize;    // 500.0f
    float origin[3];     // ox, oy, oz
};

struct MWSectorRecord {
    int32_t gx;
    int32_t gz;
    float bbox_min[3];
    float bbox_max[3];
    uint32_t num_chunks;
    uint32_t num_verts;
    uint32_t num_indices;
    char filename[32];
};
#pragma pack(pop)

static_assert(sizeof(MWVertex) == 20, "world vertex must stay 20 bytes");
static_assert(sizeof(MWStreamingHeader) == 28, "MWStreamingHeader must stay 28 bytes");
static_assert(sizeof(MWSectorRecord) == 76, "MWSectorRecord must stay 76 bytes");

struct WorldChunkMesh {
    void* vbo;
    void* ibo;
    uint32_t num_verts;
    uint32_t num_indices;
    float bbox_min[3];
    float bbox_max[3];
};

// ---------------------------------------------------------------------------
// Per-Sector Ground Height Grid (CPU side, local to each loaded 500m sector)
// ---------------------------------------------------------------------------
static const float GROUND_CELL = 2.0f;
static const float SECTOR_SIZE = 500.0f;
static const int   GROUND_DIM  = 252; // 252 * 2 m = 504 m (covers a 500m sector + border)
static const float WALL_CELL = 2.0f;
static const int WALL_DIM = 252;
static const int16_t GROUND_NONE = INT16_MAX;

struct LoadedSector {
    int32_t gx;
    int32_t gz;
    float bbox_min[3];
    float bbox_max[3];
    std::vector<WorldChunkMesh> chunks;
    std::vector<int16_t> ground_lo;
    std::vector<int16_t> ground_hi;
    std::vector<uint8_t> wall_cells;
    std::vector<int16_t> wall_lo;
    std::vector<int16_t> wall_hi;
    std::vector<size_t> ground_cursors;
    size_t ground_scan_index;
    float last_dist_sq;
};

struct ChunkFileRecord {
    long offset;
    MWChunkHeader header;
    float priority;
};

struct SectorLoadJob {
    FILE* file;
    LoadedSector sector;
    std::vector<ChunkFileRecord> chunk_order;
    std::vector<size_t> ground_cursors;
    uint32_t total_chunks;
    uint32_t chunks_loaded;
    bool active;
};

struct StreamCandidate {
    size_t index;
    float priority;
};

// World Streaming State
static bool g_streaming_mode = false;
static std::vector<MWSectorRecord> g_manifest_sectors;
static std::vector<LoadedSector> g_loaded_sectors;
static std::vector<StreamCandidate> g_stream_candidates;
static std::vector<MWVertex> g_load_verts;
static std::vector<uint16_t> g_load_indices;
static SectorLoadJob g_sector_load = {};
static size_t g_ground_sector_index = 0;

// Fallback monolithic chunks (if manifest not found)
static std::vector<WorldChunkMesh> g_fallback_chunks;
static std::vector<int16_t> g_fallback_ground_lo;
static std::vector<int16_t> g_fallback_ground_hi;
static const float FALLBACK_GROUND_MIN = -640.0f;
static const int   FALLBACK_GROUND_DIM = 321;

// Streaming Radii
static const float STREAM_LOAD_DIST   = 650.0f; // Load sectors within 650m
static const float STREAM_UNLOAD_DIST = 900.0f; // Unload sectors beyond 900m (250m hysteresis)
static const float WORLD_DRAW_DIST    = 100.0f;
static const uint32_t WORLD_TRIANGLE_BUDGET = 80000;
static const size_t GROUND_TRIANGLES_PER_FRAME = 1024;
static const u64 GROUND_BUILD_BUDGET_TICKS = (u64)(CPU_TICKS_PER_MSEC * 2.0);
static const size_t MIN_LINEAR_FREE_BUDGET = 2 * 1024 * 1024; // Keep at least 2 MB free

// Forward declarations
static size_t sector_ground_build(LoadedSector& sec,
                                  const MWVertex* verts, size_t num_verts,
                                  const uint16_t* indices, size_t num_indices,
                                  size_t first_triangle, size_t end_triangle);
static bool start_sector_load(const MWSectorRecord& rec,
                              float car_x, float car_z,
                              float forward_x, float forward_z);
static bool continue_sector_load(void);
static void update_resident_ground_build(void);
static void cancel_sector_load(void);
static void unload_sector(size_t index);
static void update_streaming(float car_x, float car_z, float forward_x, float forward_z);

static inline float aabb_distance_sq(float px, float pz, float min_x, float min_z, float max_x, float max_z) {
    float dx = 0.0f;
    if (px < min_x) dx = min_x - px;
    else if (px > max_x) dx = px - max_x;

    float dz = 0.0f;
    if (pz < min_z) dz = min_z - pz;
    else if (pz > max_z) dz = pz - max_z;

    return dx * dx + dz * dz;
}

static float stream_priority(float car_x, float car_z, float forward_x, float forward_z,
                             float distance_sq, float forward_weight,
                             float min_x, float min_z, float max_x, float max_z) {
    float cx = (min_x + max_x) * 0.5f;
    float cz = (min_z + max_z) * 0.5f;
    float dx = cx - car_x;
    float dz = cz - car_z;
    float forward = dx * forward_x + dz * forward_z;
    if (forward > 500.0f) forward = 500.0f;
    if (forward < -500.0f) forward = -500.0f;
    return distance_sq - forward * forward_weight;
}

static size_t sector_gpu_bytes(const MWSectorRecord& rec) {
    size_t vbytes = ((size_t)rec.num_verts * sizeof(MWVertex) + 15u) & ~(size_t)15u;
    size_t ibytes = ((size_t)rec.num_indices * sizeof(uint16_t) + 15u) & ~(size_t)15u;
    return vbytes + ibytes + (size_t)rec.num_chunks * 32u + MIN_LINEAR_FREE_BUDGET;
}

static float aabb_plane_max(const float* axis, float bias, const WorldChunkMesh& ch) {
    float x = (axis[0] >= 0.0f) ? ch.bbox_max[0] : ch.bbox_min[0];
    float y = (axis[1] >= 0.0f) ? ch.bbox_max[1] : ch.bbox_min[1];
    float z = (axis[2] >= 0.0f) ? ch.bbox_max[2] : ch.bbox_min[2];
    return axis[0] * x + axis[1] * y + axis[2] * z + bias;
}

struct FrustumPlane {
    float axis[3];
    float bias;
};

struct FrustumPlanes {
    FrustumPlane plane[5];
};

static void frustum_build(const C3D_Mtx& mvp, FrustumPlanes& planes) {
    const float* p = mvp.m;
    const float a[5][4] = {
        { p[0] + p[12], p[1] + p[13], p[2] + p[14], p[3] + p[15] },
        { p[12] - p[0], p[13] - p[1], p[14] - p[2], p[15] - p[3] },
        { p[4] + p[12], p[5] + p[13], p[6] + p[14], p[7] + p[15] },
        { p[12] - p[4], p[13] - p[5], p[14] - p[6], p[15] - p[7] },
        { p[12], p[13], p[14], p[15] },
    };
    for (int i = 0; i < 5; i++) {
        planes.plane[i].axis[0] = a[i][0];
        planes.plane[i].axis[1] = a[i][1];
        planes.plane[i].axis[2] = a[i][2];
        planes.plane[i].bias = a[i][3];
    }
}

static bool chunk_outside_frustum(const FrustumPlanes& planes, const WorldChunkMesh& ch) {
    for (int i = 0; i < 5; i++) {
        const FrustumPlane& p = planes.plane[i];
        float max_value = aabb_plane_max(p.axis, p.bias, ch);
        if (i == 4 ? max_value <= 0.0f : max_value < 0.0f) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Mesh Loading / Streaming Init
// ---------------------------------------------------------------------------
void world_mesh_init(void) {
    cancel_sector_load();
    g_loaded_sectors.clear();
    g_ground_sector_index = 0;
    g_manifest_sectors.clear();
    g_stream_candidates.clear();
    g_fallback_chunks.clear();
    g_streaming_mode = false;

    // Check for streaming master manifest
    FILE* mf = fopen("romfs:/assets/world_manifest.bin", "rb");
    if (mf) {
        MWStreamingHeader sh;
        if (fread(&sh, 1, sizeof(sh), mf) == sizeof(sh) &&
            memcmp(sh.magic, "MWST", 4) == 0 && sh.version == 1) {
            g_manifest_sectors.resize(sh.numSectors);
            if (fread(g_manifest_sectors.data(), sizeof(MWSectorRecord), sh.numSectors, mf) == sh.numSectors) {
                g_streaming_mode = true;
                g_stream_candidates.reserve(sh.numSectors);
                g_loaded_sectors.reserve(32);
                printf("World Stream: Manifest OK (%lu sectors, linear free %lu B)\n",
                       (unsigned long)sh.numSectors, (unsigned long)linearSpaceFree());
            }
        }
        fclose(mf);
    }

    if (g_streaming_mode) {
        // Match physics_init(): loading around the origin leaves the actual
        // harbour spawn outside the draw radius while that unrelated sector
        // is processed.
        update_streaming(-148.0f, 164.0f, -0.7071068f, -0.7071068f);
        printf("World Stream: Spawn loaded (%lu sectors, %d chunks, free %lu B)\n",
               (unsigned long)g_loaded_sectors.size(), world_mesh_chunk_count(),
               (unsigned long)linearSpaceFree());
        return;
    }

    // Fallback: monolithic fairhaven_block.mwmesh
    printf("World Stream: World manifest missing, loading monolithic fallback\n");
    FILE* f = fopen("romfs:/assets/fairhaven_block.mwmesh", "rb");
    if (!f) {
        printf("Failed to open fairhaven_block.mwmesh\n");
        return;
    }

    MWMSHeader header;
    if (fread(&header, 1, sizeof(header), f) != sizeof(header) ||
        memcmp(header.magic, "MWMS", 4) != 0 || header.version != 3) {
        fclose(f);
        return;
    }

    g_fallback_ground_lo.assign((size_t)FALLBACK_GROUND_DIM * FALLBACK_GROUND_DIM, GROUND_NONE);
    g_fallback_ground_hi.assign((size_t)FALLBACK_GROUND_DIM * FALLBACK_GROUND_DIM, INT16_MIN);

    for (uint32_t i = 0; i < header.numChunks; i++) {
        MWChunkHeader ch;
        if (fread(&ch, 1, sizeof(ch), f) != sizeof(ch)) break;

        std::vector<MWVertex> verts(ch.numVerts);
        if (fread(verts.data(), sizeof(MWVertex), ch.numVerts, f) != ch.numVerts) break;

        std::vector<uint16_t> indices(ch.numIndices);
        if (fread(indices.data(), sizeof(uint16_t), ch.numIndices, f) != ch.numIndices) break;

        if (ch.numIndices & 1) {
            uint16_t pad;
            fread(&pad, 1, sizeof(pad), f);
        }

        // Build fallback height grid
        for (size_t j = 0; j + 2 < indices.size(); j += 3) {
            uint16_t i0 = indices[j], i1 = indices[j + 1], i2 = indices[j + 2];
            if (i0 >= verts.size() || i1 >= verts.size() || i2 >= verts.size()) continue;
            const float* a = verts[i0].position;
            const float* b = verts[i1].position;
            const float* c = verts[i2].position;
            float cy = (a[1] + b[1] + c[1]) / 3.0f;
            if (cy < -24.0f || cy > 10.0f) continue;
            float ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
            float vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
            float nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
            float normal_len_sq = nx * nx + ny * ny + nz * nz;
            if (normal_len_sq < 1e-12f || ny * ny < 0.2025f * normal_len_sq) continue;
            float d = (b[2] - c[2]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[2] - c[2]);
            if (fabsf(d) < 1e-9f) continue;
            float minx = fminf(a[0], fminf(b[0], c[0]));
            float maxx = fmaxf(a[0], fmaxf(b[0], c[0]));
            float minz = fminf(a[2], fminf(b[2], c[2]));
            float maxz = fmaxf(a[2], fmaxf(b[2], c[2]));
            int ix0 = (int)floorf((minx - FALLBACK_GROUND_MIN) / GROUND_CELL);
            int ix1 = (int)floorf((maxx - FALLBACK_GROUND_MIN) / GROUND_CELL);
            int iz0 = (int)floorf((minz - FALLBACK_GROUND_MIN) / GROUND_CELL);
            int iz1 = (int)floorf((maxz - FALLBACK_GROUND_MIN) / GROUND_CELL);
            if (ix0 < 0) ix0 = 0;
            if (iz0 < 0) iz0 = 0;
            if (ix1 >= FALLBACK_GROUND_DIM) ix1 = FALLBACK_GROUND_DIM - 1;
            if (iz1 >= FALLBACK_GROUND_DIM) iz1 = FALLBACK_GROUND_DIM - 1;
            for (int iz = iz0; iz <= iz1; iz++) {
                float pz = FALLBACK_GROUND_MIN + (iz + 0.5f) * GROUND_CELL;
                for (int ix = ix0; ix <= ix1; ix++) {
                    float px = FALLBACK_GROUND_MIN + (ix + 0.5f) * GROUND_CELL;
                    float l1 = ((b[2] - c[2]) * (px - c[0]) + (c[0] - b[0]) * (pz - c[2])) / d;
                    float l2 = ((c[2] - a[2]) * (px - c[0]) + (a[0] - c[0]) * (pz - c[2])) / d;
                    float l3 = 1.0f - l1 - l2;
                    if (l1 < -0.001f || l2 < -0.001f || l3 < -0.001f) continue;
                    float y = l1 * a[1] + l2 * b[1] + l3 * c[1];
                    int16_t h = (int16_t)lrintf(y * 100.0f);
                    size_t idx = (size_t)iz * FALLBACK_GROUND_DIM + ix;
                    if (h < g_fallback_ground_lo[idx]) g_fallback_ground_lo[idx] = h;
                    if (h > g_fallback_ground_hi[idx]) g_fallback_ground_hi[idx] = h;
                }
            }
        }

        WorldChunkMesh cm;
        memset(&cm, 0, sizeof(cm));
        cm.num_verts = ch.numVerts;
        cm.num_indices = ch.numIndices;
        for (int k = 0; k < 3; k++) {
            cm.bbox_min[k] = ch.bboxMin[k];
            cm.bbox_max[k] = ch.bboxMax[k];
        }

        size_t vbytes = (sizeof(MWVertex) * ch.numVerts + 15u) & ~15u;
        size_t ibytes = (sizeof(uint16_t) * ch.numIndices + 15u) & ~15u;
        cm.vbo = linearAlloc(vbytes);
        cm.ibo = linearAlloc(ibytes);
        if (cm.vbo && cm.ibo) {
            memcpy(cm.vbo, verts.data(), sizeof(MWVertex) * ch.numVerts);
            memcpy(cm.ibo, indices.data(), sizeof(uint16_t) * ch.numIndices);
            GSPGPU_FlushDataCache(cm.vbo, vbytes);
            GSPGPU_FlushDataCache(cm.ibo, ibytes);
            g_fallback_chunks.push_back(cm);
        } else {
            if (cm.vbo) linearFree(cm.vbo);
            if (cm.ibo) linearFree(cm.ibo);
        }
    }
    fclose(f);
}

// ---------------------------------------------------------------------------
// Sector Ground Grid Building
// ---------------------------------------------------------------------------
static size_t sector_ground_build(LoadedSector& sec,
                                  const MWVertex* verts, size_t num_verts,
                                  const uint16_t* indices, size_t num_indices,
                                  size_t first_triangle, size_t end_triangle) {
    if (sec.ground_lo.empty()) {
        sec.ground_lo.assign((size_t)GROUND_DIM * GROUND_DIM, GROUND_NONE);
        sec.ground_hi.assign((size_t)GROUND_DIM * GROUND_DIM, INT16_MIN);
        sec.wall_cells.assign((size_t)WALL_DIM * WALL_DIM, 0);
        sec.wall_lo.assign((size_t)WALL_DIM * WALL_DIM, INT16_MAX);
        sec.wall_hi.assign((size_t)WALL_DIM * WALL_DIM, INT16_MIN);
    }

    float sec_min_x = sec.gx * SECTOR_SIZE;
    float sec_min_z = sec.gz * SECTOR_SIZE;

    size_t first_index = first_triangle * 3;
    size_t end_index = end_triangle * 3;
    if (end_index > num_indices) end_index = num_indices;
    u64 start_tick = svcGetSystemTick();
    for (size_t j = first_index; j + 2 < end_index; j += 3) {
        if ((((j / 3) - first_triangle) & 31u) == 0 &&
            svcGetSystemTick() - start_tick >= GROUND_BUILD_BUDGET_TICKS)
            return j / 3;
        uint16_t i0 = indices[j], i1 = indices[j + 1], i2 = indices[j + 2];
        if (i0 >= num_verts || i1 >= num_verts || i2 >= num_verts) continue;

        const float* a = verts[i0].position;
        const float* b = verts[i1].position;
        const float* c = verts[i2].position;

        float cy = (a[1] + b[1] + c[1]) / 3.0f;
        // Accept road and drivable bridge decks
        if (cy < -24.0f || cy > 140.0f) continue;

        float ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
        float vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
        float nx = uy * vz - uz * vy;
        float ny = uz * vx - ux * vz;
        float nz = ux * vy - uy * vx;
        float normal_len_sq = nx * nx + ny * ny + nz * nz;
        if (normal_len_sq < 1e-12f) continue;

        // Keep steep geometry as a lightweight XZ wall footprint. Rasterize
        // triangle edges because vertical faces collapse to lines in XZ.
        float wall_min_x = fminf(a[0], fminf(b[0], c[0]));
        float wall_max_x = fmaxf(a[0], fmaxf(b[0], c[0]));
        float wall_min_z = fminf(a[2], fminf(b[2], c[2]));
        float wall_max_z = fmaxf(a[2], fmaxf(b[2], c[2]));
        float wall_min_y = fminf(a[1], fminf(b[1], c[1]));
        float wall_max_y = fmaxf(a[1], fmaxf(b[1], c[1]));
        float wall_span_x = wall_max_x - wall_min_x;
        float wall_span_z = wall_max_z - wall_min_z;
        if (ny * ny < 0.09f * normal_len_sq && wall_max_y - wall_min_y > 2.0f &&
            fminf(wall_span_x, wall_span_z) < 1.0f) {
            const float* p[3] = { a, b, c };
            for (int e = 0; e < 3; ++e) {
                const float* p0 = p[e];
                const float* p1 = p[(e + 1) % 3];
                float dx = p1[0] - p0[0], dz = p1[2] - p0[2];
                int steps = (int)ceilf(fmaxf(fabsf(dx), fabsf(dz)) / 1.0f);
                if (steps < 1) steps = 1;
                for (int s = 0; s <= steps; ++s) {
                    float t = (float)s / (float)steps;
                    int ix = (int)floorf((p0[0] + dx * t - sec_min_x) / WALL_CELL);
                    int iz = (int)floorf((p0[2] + dz * t - sec_min_z) / WALL_CELL);
                    if (ix >= 0 && ix < WALL_DIM && iz >= 0 && iz < WALL_DIM) {
                        size_t wi = (size_t)iz * WALL_DIM + ix;
                        int16_t wy = (int16_t)lrintf((p0[1] + (p1[1] - p0[1]) * t) * 100.0f);
                        sec.wall_cells[wi] = 1;
                        int16_t low = (int16_t)(wy - 100);
                        int16_t high = (int16_t)(wy + 100);
                        if (low < sec.wall_lo[wi]) sec.wall_lo[wi] = low;
                        if (high > sec.wall_hi[wi]) sec.wall_hi[wi] = high;
                    }
                }
            }
        }
        if (ny * ny < 0.16f * normal_len_sq) continue;

        float d = (b[2] - c[2]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[2] - c[2]);
        if (fabsf(d) < 1e-9f) continue;

        float minx = fminf(a[0], fminf(b[0], c[0]));
        float maxx = fmaxf(a[0], fmaxf(b[0], c[0]));
        float minz = fminf(a[2], fminf(b[2], c[2]));
        float maxz = fmaxf(a[2], fmaxf(b[2], c[2]));

        int ix0 = (int)floorf((minx - sec_min_x) / GROUND_CELL);
        int ix1 = (int)floorf((maxx - sec_min_x) / GROUND_CELL);
        int iz0 = (int)floorf((minz - sec_min_z) / GROUND_CELL);
        int iz1 = (int)floorf((maxz - sec_min_z) / GROUND_CELL);
        if (ix0 < 0) ix0 = 0;
        if (iz0 < 0) iz0 = 0;
        if (ix1 >= GROUND_DIM) ix1 = GROUND_DIM - 1;
        if (iz1 >= GROUND_DIM) iz1 = GROUND_DIM - 1;

        for (int iz = iz0; iz <= iz1; iz++) {
            float pz = sec_min_z + (iz + 0.5f) * GROUND_CELL;
            for (int ix = ix0; ix <= ix1; ix++) {
                float px = sec_min_x + (ix + 0.5f) * GROUND_CELL;
                float l1 = ((b[2] - c[2]) * (px - c[0]) + (c[0] - b[0]) * (pz - c[2])) / d;
                float l2 = ((c[2] - a[2]) * (px - c[0]) + (a[0] - c[0]) * (pz - c[2])) / d;
                float l3 = 1.0f - l1 - l2;
                if (l1 < -0.001f || l2 < -0.001f || l3 < -0.001f) continue;
                float y = l1 * a[1] + l2 * b[1] + l3 * c[1];
                int16_t h = (int16_t)lrintf(y * 100.0f);
                size_t idx = (size_t)iz * GROUND_DIM + ix;
                if (h < sec.ground_lo[idx]) sec.ground_lo[idx] = h;
                if (h > sec.ground_hi[idx]) sec.ground_hi[idx] = h;
            }
        }
    }
    return end_triangle;
}

// ---------------------------------------------------------------------------
// Dynamic Sector Loading & Unloading
// ---------------------------------------------------------------------------
static void cancel_sector_load(void) {
    if (!g_sector_load.active) return;
    if (g_sector_load.file) fclose(g_sector_load.file);
    for (auto& cm : g_sector_load.sector.chunks) {
        if (cm.vbo) linearFree(cm.vbo);
        if (cm.ibo) linearFree(cm.ibo);
    }
    g_sector_load.sector = LoadedSector();
    g_sector_load.chunk_order = std::vector<ChunkFileRecord>();
    g_sector_load.file = NULL;
    g_sector_load.total_chunks = 0;
    g_sector_load.chunks_loaded = 0;
    g_sector_load.ground_cursors.clear();
    g_sector_load.active = false;
}

static bool start_sector_load(const MWSectorRecord& rec,
                              float car_x, float car_z,
                              float forward_x, float forward_z) {
    char path[128];
    snprintf(path, sizeof(path), "romfs:/assets/sectors/%s", rec.filename);
    FILE* f = fopen(path, "rb");
    if (!f) return false;

    MWMSHeader header;
    if (fread(&header, 1, sizeof(header), f) != sizeof(header) ||
        memcmp(header.magic, "MWMS", 4) != 0 || header.version != 3) {
        fclose(f);
        return false;
    }

    std::vector<ChunkFileRecord> chunk_order;
    chunk_order.reserve(header.numChunks);
    bool scan_ok = true;
    for (uint32_t i = 0; i < header.numChunks; i++) {
        long offset = ftell(f);
        MWChunkHeader ch;
        if (offset < 0 || fread(&ch, 1, sizeof(ch), f) != sizeof(ch)) {
            scan_ok = false;
            break;
        }
        float d2 = aabb_distance_sq(car_x, car_z,
                                    ch.bboxMin[0], ch.bboxMin[2],
                                    ch.bboxMax[0], ch.bboxMax[2]);
        float priority = stream_priority(car_x, car_z, forward_x, forward_z, d2, 150.0f,
                                         ch.bboxMin[0], ch.bboxMin[2],
                                         ch.bboxMax[0], ch.bboxMax[2]);
        chunk_order.push_back({offset, ch, priority});
        size_t skip = (size_t)ch.numVerts * sizeof(MWVertex) +
                      (size_t)ch.numIndices * sizeof(uint16_t) +
                      ((ch.numIndices & 1u) ? sizeof(uint16_t) : 0u);
        if (skip > 0x7fffffff || fseek(f, (long)skip, SEEK_CUR) != 0) {
            scan_ok = false;
            break;
        }
    }
    if (!scan_ok) {
        fclose(f);
        return false;
    }
    std::sort(chunk_order.begin(), chunk_order.end(),
              [](const ChunkFileRecord& a, const ChunkFileRecord& b) {
                  return a.priority < b.priority;
              });

    g_sector_load.sector = LoadedSector();
    g_sector_load.chunk_order = std::move(chunk_order);
    g_sector_load.file = f;
    g_sector_load.sector.gx = rec.gx;
    g_sector_load.sector.gz = rec.gz;
    for (int k = 0; k < 3; k++) {
        g_sector_load.sector.bbox_min[k] = rec.bbox_min[k];
        g_sector_load.sector.bbox_max[k] = rec.bbox_max[k];
    }
    g_sector_load.sector.last_dist_sq = 0.0f;
    g_sector_load.total_chunks = header.numChunks;
    g_sector_load.chunks_loaded = 0;
    g_sector_load.sector.chunks.reserve(header.numChunks);
    g_sector_load.ground_cursors.reserve(header.numChunks);
    g_sector_load.active = true;
    return true;
}

static bool continue_sector_load(void) {
    if (!g_sector_load.active) return false;
    // Read and upload one nearby chunk every frame. Collision-grid work runs
    // separately on committed sectors so it cannot stall this file stream.
    if (g_sector_load.chunks_loaded >= g_sector_load.total_chunks) {
        fclose(g_sector_load.file);
        g_sector_load.file = NULL;
        g_sector_load.sector.ground_cursors = std::move(g_sector_load.ground_cursors);
        g_sector_load.sector.ground_scan_index = 0;
        g_loaded_sectors.push_back(std::move(g_sector_load.sector));
        g_sector_load.chunk_order = std::vector<ChunkFileRecord>();
        g_sector_load.ground_cursors.clear();
        g_sector_load.total_chunks = 0;
        g_sector_load.chunks_loaded = 0;
        g_sector_load.active = false;
        return true;
    }

    FILE* f = g_sector_load.file;
    const ChunkFileRecord& file_chunk = g_sector_load.chunk_order[g_sector_load.chunks_loaded];
    MWChunkHeader ch = file_chunk.header;
    if (fseek(f, file_chunk.offset + (long)sizeof(MWChunkHeader), SEEK_SET) != 0) {
        cancel_sector_load();
        return false;
    }

    g_load_verts.resize(ch.numVerts);
    if (fread(g_load_verts.data(), sizeof(MWVertex), ch.numVerts, f) != ch.numVerts) {
        cancel_sector_load();
        return false;
    }

    g_load_indices.resize(ch.numIndices);
    if (fread(g_load_indices.data(), sizeof(uint16_t), ch.numIndices, f) != ch.numIndices) {
        cancel_sector_load();
        return false;
    }

    if (ch.numIndices & 1) {
        uint16_t pad;
        if (fread(&pad, 1, sizeof(pad), f) != sizeof(pad)) {
            cancel_sector_load();
            return false;
        }
    }

    WorldChunkMesh cm;
    memset(&cm, 0, sizeof(cm));
    cm.num_verts = ch.numVerts;
    cm.num_indices = ch.numIndices;
    for (int k = 0; k < 3; k++) {
        cm.bbox_min[k] = ch.bboxMin[k];
        cm.bbox_max[k] = ch.bboxMax[k];
    }

    size_t vbytes = (sizeof(MWVertex) * ch.numVerts + 15u) & ~15u;
    size_t ibytes = (sizeof(uint16_t) * ch.numIndices + 15u) & ~15u;
    cm.vbo = linearAlloc(vbytes);
    cm.ibo = linearAlloc(ibytes);
    if (!cm.vbo || !cm.ibo) {
        if (cm.vbo) linearFree(cm.vbo);
        if (cm.ibo) linearFree(cm.ibo);
        cancel_sector_load();
        return false;
    }

    memcpy(cm.vbo, g_load_verts.data(), sizeof(MWVertex) * ch.numVerts);
    memcpy(cm.ibo, g_load_indices.data(), sizeof(uint16_t) * ch.numIndices);
    GSPGPU_FlushDataCache(cm.vbo, vbytes);
    GSPGPU_FlushDataCache(cm.ibo, ibytes);
    g_sector_load.sector.chunks.push_back(cm);
    g_sector_load.ground_cursors.push_back(0);
    g_sector_load.chunks_loaded++;
    return true;
}

static void update_resident_ground_build(void) {
    if (g_loaded_sectors.empty()) return;
    if (g_ground_sector_index >= g_loaded_sectors.size()) g_ground_sector_index = 0;
    for (size_t n = 0; n < g_loaded_sectors.size(); n++) {
        size_t si = (g_ground_sector_index + n) % g_loaded_sectors.size();
        LoadedSector& sec = g_loaded_sectors[si];
        if (sec.ground_cursors.empty()) continue;
        size_t count = sec.chunks.size();
        for (size_t j = 0; j < count; j++) {
            size_t ci = (sec.ground_scan_index + j) % count;
            WorldChunkMesh& cm = sec.chunks[ci];
            size_t& cursor = sec.ground_cursors[ci];
            size_t triangle_count = cm.num_indices / 3;
            if (cursor >= triangle_count) continue;
            size_t end = cursor + GROUND_TRIANGLES_PER_FRAME;
            if (end > triangle_count) end = triangle_count;
            cursor = sector_ground_build(sec,
                static_cast<const MWVertex*>(cm.vbo), cm.num_verts,
                static_cast<const uint16_t*>(cm.ibo), cm.num_indices,
                cursor, end);
            sec.ground_scan_index = (ci + 1) % count;
            g_ground_sector_index = (si + 1) % g_loaded_sectors.size();
            return;
        }
    }
}

static void unload_sector(size_t index) {
    if (index >= g_loaded_sectors.size()) return;
    LoadedSector& sec = g_loaded_sectors[index];
    for (auto& cm : sec.chunks) {
        if (cm.vbo) { linearFree(cm.vbo); cm.vbo = NULL; }
        if (cm.ibo) { linearFree(cm.ibo); cm.ibo = NULL; }
    }
    sec.chunks.clear();
    sec.ground_lo.clear();
    sec.ground_hi.clear();
    sec.wall_cells.clear();
    sec.wall_lo.clear();
    sec.wall_hi.clear();
    g_loaded_sectors.erase(g_loaded_sectors.begin() + index);
}

static void update_streaming(float car_x, float car_z, float forward_x, float forward_z) {
    if (!g_streaming_mode || g_manifest_sectors.empty()) return;
    update_resident_ground_build();
    if (g_sector_load.active) {
        continue_sector_load();
        return;
    }

    float unload_dist_sq = STREAM_UNLOAD_DIST * STREAM_UNLOAD_DIST;
    float load_dist_sq   = STREAM_LOAD_DIST * STREAM_LOAD_DIST;

    // 1. Unload sectors that are too far from car
    for (int i = (int)g_loaded_sectors.size() - 1; i >= 0; i--) {
        const LoadedSector& sec = g_loaded_sectors[i];
        float d2 = aabb_distance_sq(car_x, car_z,
                                    sec.bbox_min[0], sec.bbox_min[2],
                                    sec.bbox_max[0], sec.bbox_max[2]);
        if (d2 > unload_dist_sq) {
            unload_sector(i);
        }
    }

    // 2. Linear memory guard: proactively unload furthest sector if VRAM is tight
    while (linearSpaceFree() < MIN_LINEAR_FREE_BUDGET && g_loaded_sectors.size() > 2) {
        size_t furthest_idx = 0;
        float worst_priority = -1.0e30f;
        for (size_t i = 0; i < g_loaded_sectors.size(); i++) {
            float d2 = aabb_distance_sq(car_x, car_z,
                                        g_loaded_sectors[i].bbox_min[0], g_loaded_sectors[i].bbox_min[2],
                                        g_loaded_sectors[i].bbox_max[0], g_loaded_sectors[i].bbox_max[2]);
            float priority = stream_priority(car_x, car_z, forward_x, forward_z, d2, 400.0f,
                                             g_loaded_sectors[i].bbox_min[0], g_loaded_sectors[i].bbox_min[2],
                                             g_loaded_sectors[i].bbox_max[0], g_loaded_sectors[i].bbox_max[2]);
            if (priority > worst_priority) {
                worst_priority = priority;
                furthest_idx = i;
            }
        }
        unload_sector(furthest_idx);
    }

    // 3. Find candidates to load within load distance
    g_stream_candidates.clear();

    for (size_t i = 0; i < g_manifest_sectors.size(); i++) {
        const MWSectorRecord& rec = g_manifest_sectors[i];
        // Check if already loaded
        bool loaded = false;
        for (const auto& ls : g_loaded_sectors) {
            if (ls.gx == rec.gx && ls.gz == rec.gz) {
                loaded = true;
                break;
            }
        }
        if (loaded) continue;

        float d2 = aabb_distance_sq(car_x, car_z,
                                    rec.bbox_min[0], rec.bbox_min[2],
                                    rec.bbox_max[0], rec.bbox_max[2]);
        if (d2 <= load_dist_sq) {
            float priority = stream_priority(car_x, car_z, forward_x, forward_z, d2, 400.0f,
                                             rec.bbox_min[0], rec.bbox_min[2],
                                             rec.bbox_max[0], rec.bbox_max[2]);
            g_stream_candidates.push_back({i, priority});
        }
    }

    // Favor sectors along the direction of travel while still considering
    // distance, so the cache prepares the road ahead instead of filling with
    // equally close sectors behind the car.
    std::sort(g_stream_candidates.begin(), g_stream_candidates.end(), [](const StreamCandidate& a, const StreamCandidate& b) {
        return a.priority < b.priority;
    });

    // Budget: load at most one sector per frame. If memory is full, replace
    // a much farther resident sector with the nearest missing candidate. The
    // distance margin prevents the cache from oscillating between neighbours.
    for (const StreamCandidate& candidate : g_stream_candidates) {
        const MWSectorRecord& target = g_manifest_sectors[candidate.index];
        while (linearSpaceFree() < sector_gpu_bytes(target) && g_loaded_sectors.size() > 1) {
            size_t furthest_idx = 0;
            float furthest_priority = -1.0e30f;
            for (size_t i = 0; i < g_loaded_sectors.size(); i++) {
                const LoadedSector& resident = g_loaded_sectors[i];
                float d2 = aabb_distance_sq(car_x, car_z,
                                            resident.bbox_min[0], resident.bbox_min[2],
                                            resident.bbox_max[0], resident.bbox_max[2]);
                float priority = stream_priority(car_x, car_z, forward_x, forward_z, d2, 400.0f,
                                                 resident.bbox_min[0], resident.bbox_min[2],
                                                 resident.bbox_max[0], resident.bbox_max[2]);
                if (priority > furthest_priority) {
                    furthest_priority = priority;
                    furthest_idx = i;
                }
            }
            const float replace_margin = 25000.0f;
            if (candidate.priority + replace_margin >= furthest_priority) break;
            unload_sector(furthest_idx);
        }
        if (linearSpaceFree() < sector_gpu_bytes(target)) continue;
        if (!start_sector_load(target, car_x, car_z, forward_x, forward_z)) {
            printf("World Stream: failed to start %s\n", target.filename);
        }
        break;
    }
}

// ---------------------------------------------------------------------------
// Ground Height Sampling Across Streamed World
// ---------------------------------------------------------------------------
float world_mesh_ground_height(float x, float z, float fallback) {
    if (!g_streaming_mode) {
        if (g_fallback_ground_lo.empty()) return fallback;
        int ix = (int)floorf((x - FALLBACK_GROUND_MIN) / GROUND_CELL);
        int iz = (int)floorf((z - FALLBACK_GROUND_MIN) / GROUND_CELL);
        if (ix < 0 || ix >= FALLBACK_GROUND_DIM || iz < 0 || iz >= FALLBACK_GROUND_DIM) return fallback;
        size_t idx = (size_t)iz * FALLBACK_GROUND_DIM + ix;
        int16_t lo = g_fallback_ground_lo[idx];
        int16_t hi = g_fallback_ground_hi[idx];
        bool has_lo = (lo != GROUND_NONE);
        bool has_hi = (hi != INT16_MIN);
        if (!has_lo && !has_hi) return fallback;
        float ylo = has_lo ? (float)lo / 100.0f : fallback;
        float yhi = has_hi ? (float)hi / 100.0f : fallback;
        return (fabsf(ylo - fallback) <= fabsf(yhi - fallback)) ? ylo : yhi;
    }

    // Streaming mode: find which loaded sector contains (x, z)
    int gx = (int)floorf(x / SECTOR_SIZE);
    int gz = (int)floorf(z / SECTOR_SIZE);

    for (const auto& sec : g_loaded_sectors) {
        if (sec.gx == gx && sec.gz == gz) {
            if (sec.ground_lo.empty()) continue;
            float sec_min_x = sec.gx * SECTOR_SIZE;
            float sec_min_z = sec.gz * SECTOR_SIZE;
            int ix = (int)floorf((x - sec_min_x) / GROUND_CELL);
            int iz = (int)floorf((z - sec_min_z) / GROUND_CELL);
            if (ix < 0 || ix >= GROUND_DIM || iz < 0 || iz >= GROUND_DIM) continue;

            size_t idx = (size_t)iz * GROUND_DIM + ix;
            int16_t lo = sec.ground_lo[idx];
            int16_t hi = sec.ground_hi[idx];
            bool has_lo = (lo != GROUND_NONE);
            bool has_hi = (hi != INT16_MIN);
            if (!has_lo && !has_hi) continue;
            float ylo = has_lo ? (float)lo / 100.0f : fallback;
            float yhi = has_hi ? (float)hi / 100.0f : fallback;
            return (fabsf(ylo - fallback) <= fabsf(yhi - fallback)) ? ylo : yhi;
        }
    }

    // If point is on a seam or slightly outside primary sector, check other loaded sectors
    float best_y = fallback;
    float best_diff = 1e9f;
    for (const auto& sec : g_loaded_sectors) {
        if (sec.ground_lo.empty()) continue;
        float sec_min_x = sec.gx * SECTOR_SIZE;
        float sec_min_z = sec.gz * SECTOR_SIZE;
        int ix = (int)floorf((x - sec_min_x) / GROUND_CELL);
        int iz = (int)floorf((z - sec_min_z) / GROUND_CELL);
        if (ix < 0 || ix >= GROUND_DIM || iz < 0 || iz >= GROUND_DIM) continue;

        size_t idx = (size_t)iz * GROUND_DIM + ix;
        int16_t lo = sec.ground_lo[idx];
        int16_t hi = sec.ground_hi[idx];
        bool has_lo = (lo != GROUND_NONE);
        bool has_hi = (hi != INT16_MIN);
        if (!has_lo && !has_hi) continue;
        float ylo = has_lo ? (float)lo / 100.0f : fallback;
        float yhi = has_hi ? (float)hi / 100.0f : fallback;
        float candidate = (fabsf(ylo - fallback) <= fabsf(yhi - fallback)) ? ylo : yhi;
        float diff = fabsf(candidate - fallback);
        if (diff < best_diff) {
            best_diff = diff;
            best_y = candidate;
        }
    }
    return best_y;
}

static bool wall_cell_at(float x, float z, float y) {
    int gx = (int)floorf(x / SECTOR_SIZE);
    int gz = (int)floorf(z / SECTOR_SIZE);
    for (const auto& sec : g_loaded_sectors) {
        if (sec.gx != gx || sec.gz != gz || sec.wall_cells.empty()) continue;
        float sx = sec.gx * SECTOR_SIZE;
        float sz = sec.gz * SECTOR_SIZE;
        int ix = (int)floorf((x - sx) / WALL_CELL);
        int iz = (int)floorf((z - sz) / WALL_CELL);
        if (ix < 0 || ix >= WALL_DIM || iz < 0 || iz >= WALL_DIM) return false;
        size_t idx = (size_t)iz * WALL_DIM + ix;
        if (!sec.wall_cells[idx]) return false;
        float low = (float)sec.wall_lo[idx] / 100.0f;
        float high = (float)sec.wall_hi[idx] / 100.0f;
        return high >= y && low <= y + 1.8f;
    }
    return false;
}

bool world_mesh_sweep_blocked(float x0, float z0, float y, float x1, float z1) {
    float dx = x1 - x0, dz = z1 - z0;
    int steps = (int)ceilf(fmaxf(fabsf(dx), fabsf(dz)) / 0.75f);
    if (steps < 1) steps = 1;
    for (int i = 1; i <= steps; ++i) {
        float t = (float)i / (float)steps;
        float x = x0 + dx * t, z = z0 + dz * t;
        // Approximate the car footprint with a center and four edge samples.
        if (wall_cell_at(x, z, y)) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// World Rendering & Frustum Draw
// ---------------------------------------------------------------------------
void world_mesh_render(const C3D_Mtx* projection, const C3D_Mtx* view,
                       float car_x, float car_z, float forward_x, float forward_z) {
    if (g_streaming_mode) {
        update_streaming(car_x, car_z, forward_x, forward_z);
        if (g_loaded_sectors.empty() && !g_sector_load.active) return;
    } else {
        if (g_fallback_chunks.empty()) return;
    }

    extern int g_uLoc_mvp;

    C3D_Mtx model, mv, mvp;
    Mtx_Identity(&model);
    Mtx_Multiply(&mv, view, &model);
    Mtx_Multiply(&mvp, projection, &mv);
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, g_uLoc_mvp, &mvp);
    FrustumPlanes frustum;
    frustum_build(mvp, frustum);

    if (g_streaming_mode) {
        struct DrawCandidate {
            const WorldChunkMesh* chunk;
            float priority;
        };
        static const size_t MAX_DRAW_CHUNKS = 64;
        DrawCandidate draw_list[MAX_DRAW_CHUNKS];
        size_t draw_count = 0;
        auto consider_chunk = [&](const WorldChunkMesh& ch) {
            if (!ch.vbo || !ch.ibo || ch.num_indices == 0) return;
            float distance_sq = aabb_distance_sq(car_x, car_z,
                                                 ch.bbox_min[0], ch.bbox_min[2],
                                                 ch.bbox_max[0], ch.bbox_max[2]);
            if (distance_sq > WORLD_DRAW_DIST * WORLD_DRAW_DIST ||
                chunk_outside_frustum(frustum, ch)) return;
            float priority = stream_priority(car_x, car_z, forward_x, forward_z, distance_sq, 150.0f,
                                             ch.bbox_min[0], ch.bbox_min[2],
                                             ch.bbox_max[0], ch.bbox_max[2]);
            size_t at = 0;
            while (at < draw_count && draw_list[at].priority <= priority) at++;
            if (at >= MAX_DRAW_CHUNKS) return;
            if (draw_count < MAX_DRAW_CHUNKS) draw_count++;
            for (size_t j = draw_count - 1; j > at; j--) draw_list[j] = draw_list[j - 1];
            draw_list[at].chunk = &ch;
            draw_list[at].priority = priority;
        };

        for (const auto& sec : g_loaded_sectors) {
            for (const auto& ch : sec.chunks) consider_chunk(ch);
        }
        if (g_sector_load.active) {
            for (const auto& ch : g_sector_load.sector.chunks) consider_chunk(ch);
        }

        uint32_t triangles_left = WORLD_TRIANGLE_BUDGET;
        for (size_t i = 0; i < draw_count; i++) {
            const WorldChunkMesh& ch = *draw_list[i].chunk;
            uint32_t triangles = ch.num_indices / 3;
            if (triangles > triangles_left) continue;

            C3D_BufInfo* bufInfo = C3D_GetBufInfo();
            BufInfo_Init(bufInfo);
            BufInfo_Add(bufInfo, ch.vbo, sizeof(MWVertex), 3, 0x210);
            C3D_DrawElements(GPU_TRIANGLES, ch.num_indices, C3D_UNSIGNED_SHORT, ch.ibo);
            triangles_left -= triangles;
            if (!triangles_left) break;
        }
    } else {
        for (const auto& ch : g_fallback_chunks) {
            if (!ch.vbo || !ch.ibo || ch.num_indices == 0) continue;

            C3D_BufInfo* bufInfo = C3D_GetBufInfo();
            BufInfo_Init(bufInfo);
            BufInfo_Add(bufInfo, ch.vbo, sizeof(MWVertex), 3, 0x210);

            C3D_DrawElements(GPU_TRIANGLES, ch.num_indices, C3D_UNSIGNED_SHORT, ch.ibo);
        }
    }
}

// ---------------------------------------------------------------------------
// Telemetry & Diagnostics
// ---------------------------------------------------------------------------
int world_mesh_chunk_count(void) {
    if (g_streaming_mode) {
        int count = 0;
        for (const auto& sec : g_loaded_sectors) count += (int)sec.chunks.size();
        if (g_sector_load.active) count += (int)g_sector_load.sector.chunks.size();
        return count;
    }
    return (int)g_fallback_chunks.size();
}

int world_mesh_chunk_total(void) {
    if (g_streaming_mode) {
        int tot = 0;
        for (const auto& rec : g_manifest_sectors) tot += (int)rec.num_chunks;
        return tot;
    }
    return (int)g_fallback_chunks.size();
}

int world_mesh_ground_cell_count(void) {
    int n = 0;
    if (g_streaming_mode) {
        for (const auto& sec : g_loaded_sectors) {
            for (size_t i = 0; i < sec.ground_lo.size(); i++) {
                if (sec.ground_lo[i] != GROUND_NONE || sec.ground_hi[i] != INT16_MIN) n++;
            }
        }
    } else {
        for (size_t i = 0; i < g_fallback_ground_lo.size(); i++) {
            if (g_fallback_ground_lo[i] != GROUND_NONE || g_fallback_ground_hi[i] != INT16_MIN) n++;
        }
    }
    return n;
}

void world_mesh_write_diag(void) {
    FILE* f = fopen("sdmc:/world_diag.txt", "w");
    if (!f) return;
    fprintf(f, "mode: %s, loaded_sectors: %lu, total_manifest_sectors: %lu, linear_free: %lu\n",
            g_streaming_mode ? "STREAMING" : "FALLBACK",
            (unsigned long)g_loaded_sectors.size(), (unsigned long)g_manifest_sectors.size(),
            (unsigned long)linearSpaceFree());
    for (size_t s = 0; s < g_loaded_sectors.size(); s++) {
        const auto& sec = g_loaded_sectors[s];
        fprintf(f, "sec (%ld, %ld) chunks %lu\n", (long)sec.gx, (long)sec.gz, (unsigned long)sec.chunks.size());
    }
    fclose(f);
}

void world_mesh_exit(void) {
    for (auto& sec : g_loaded_sectors) {
        for (auto& ch : sec.chunks) {
            if (ch.vbo) { linearFree(ch.vbo); ch.vbo = NULL; }
            if (ch.ibo) { linearFree(ch.ibo); ch.ibo = NULL; }
        }
        sec.chunks.clear();
        sec.ground_lo.clear();
        sec.ground_hi.clear();
    }
    g_loaded_sectors.clear();

    for (auto& ch : g_fallback_chunks) {
        if (ch.vbo) { linearFree(ch.vbo); ch.vbo = NULL; }
        if (ch.ibo) { linearFree(ch.ibo); ch.ibo = NULL; }
    }
    g_fallback_chunks.clear();
    g_fallback_ground_lo.clear();
    g_fallback_ground_hi.clear();
}
