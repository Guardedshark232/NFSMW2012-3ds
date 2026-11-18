#include "world_mesh.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

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
static const float GROUND_CELL = 4.0f;
static const float SECTOR_SIZE = 500.0f;
static const int   GROUND_DIM  = 126; // 126 * 4 m = 504 m (covers a 500m sector + border)
static const int16_t GROUND_NONE = INT16_MAX;

struct LoadedSector {
    int32_t gx;
    int32_t gz;
    float bbox_min[3];
    float bbox_max[3];
    std::vector<WorldChunkMesh> chunks;
    std::vector<int16_t> ground_lo;
    std::vector<int16_t> ground_hi;
    float last_dist_sq;
};

// World Streaming State
static bool g_streaming_mode = false;
static std::vector<MWSectorRecord> g_manifest_sectors;
static std::vector<LoadedSector> g_loaded_sectors;

// Fallback monolithic chunks (if manifest not found)
static std::vector<WorldChunkMesh> g_fallback_chunks;
static std::vector<int16_t> g_fallback_ground_lo;
static std::vector<int16_t> g_fallback_ground_hi;
static const float FALLBACK_GROUND_MIN = -640.0f;
static const int   FALLBACK_GROUND_DIM = 321;

// Streaming Radii
static const float STREAM_LOAD_DIST   = 650.0f; // Load sectors within 650m
static const float STREAM_UNLOAD_DIST = 900.0f; // Unload sectors beyond 900m (250m hysteresis)
static const size_t MIN_LINEAR_FREE_BUDGET = 2 * 1024 * 1024; // Keep at least 2 MB free

// Forward declarations
static void sector_ground_build(LoadedSector& sec,
                                const std::vector<MWVertex>& verts,
                                const std::vector<uint16_t>& indices);
static bool load_sector(const MWSectorRecord& rec);
static void unload_sector(size_t index);
static void update_streaming(float car_x, float car_z);

static inline float aabb_distance_sq(float px, float pz, float min_x, float min_z, float max_x, float max_z) {
    float dx = 0.0f;
    if (px < min_x) dx = min_x - px;
    else if (px > max_x) dx = px - max_x;

    float dz = 0.0f;
    if (pz < min_z) dz = min_z - pz;
    else if (pz > max_z) dz = pz - max_z;

    return dx * dx + dz * dz;
}

// ---------------------------------------------------------------------------
// Mesh Loading / Streaming Init
// ---------------------------------------------------------------------------
void world_mesh_init(void) {
    g_loaded_sectors.clear();
    g_manifest_sectors.clear();
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
                printf("World Stream: Manifest OK (%lu sectors, linear free %lu B)\n",
                       (unsigned long)sh.numSectors, (unsigned long)linearSpaceFree());
            }
        }
        fclose(mf);
    }

    if (g_streaming_mode) {
        // Initial load for spawn area around (0, 0)
        update_streaming(0.0f, 0.0f);
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
            float len = sqrtf(nx * nx + ny * ny + nz * nz);
            if (len < 1e-6f || fabsf(ny) / len < 0.45f) continue;
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
static void sector_ground_build(LoadedSector& sec,
                                const std::vector<MWVertex>& verts,
                                const std::vector<uint16_t>& indices) {
    if (sec.ground_lo.empty()) {
        sec.ground_lo.assign((size_t)GROUND_DIM * GROUND_DIM, GROUND_NONE);
        sec.ground_hi.assign((size_t)GROUND_DIM * GROUND_DIM, INT16_MIN);
    }

    float sec_min_x = sec.gx * SECTOR_SIZE;
    float sec_min_z = sec.gz * SECTOR_SIZE;

    for (size_t j = 0; j + 2 < indices.size(); j += 3) {
        uint16_t i0 = indices[j], i1 = indices[j + 1], i2 = indices[j + 2];
        if (i0 >= verts.size() || i1 >= verts.size() || i2 >= verts.size()) continue;

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
        float len = sqrtf(nx * nx + ny * ny + nz * nz);
        if (len < 1e-6f || fabsf(ny) / len < 0.40f) continue;

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
}

// ---------------------------------------------------------------------------
// Dynamic Sector Loading & Unloading
// ---------------------------------------------------------------------------
static bool load_sector(const MWSectorRecord& rec) {
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

    LoadedSector sec;
    sec.gx = rec.gx;
    sec.gz = rec.gz;
    for (int k = 0; k < 3; k++) {
        sec.bbox_min[k] = rec.bbox_min[k];
        sec.bbox_max[k] = rec.bbox_max[k];
    }
    sec.last_dist_sq = 0.0f;

    bool ok = true;
    for (uint32_t i = 0; i < header.numChunks; i++) {
        MWChunkHeader ch;
        if (fread(&ch, 1, sizeof(ch), f) != sizeof(ch)) { ok = false; break; }

        std::vector<MWVertex> verts(ch.numVerts);
        if (fread(verts.data(), sizeof(MWVertex), ch.numVerts, f) != ch.numVerts) { ok = false; break; }

        std::vector<uint16_t> indices(ch.numIndices);
        if (fread(indices.data(), sizeof(uint16_t), ch.numIndices, f) != ch.numIndices) { ok = false; break; }

        if (ch.numIndices & 1) {
            uint16_t pad;
            fread(&pad, 1, sizeof(pad), f);
        }

        sector_ground_build(sec, verts, indices);

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
            sec.chunks.push_back(cm);
        } else {
            if (cm.vbo) linearFree(cm.vbo);
            if (cm.ibo) linearFree(cm.ibo);
            ok = false;
            break;
        }
    }
    fclose(f);

    if (ok) {
        g_loaded_sectors.push_back(sec);
        return true;
    } else {
        for (auto& cm : sec.chunks) {
            if (cm.vbo) linearFree(cm.vbo);
            if (cm.ibo) linearFree(cm.ibo);
        }
        return false;
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
    g_loaded_sectors.erase(g_loaded_sectors.begin() + index);
}

static void update_streaming(float car_x, float car_z) {
    if (!g_streaming_mode || g_manifest_sectors.empty()) return;

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
        float max_d2 = -1.0f;
        for (size_t i = 0; i < g_loaded_sectors.size(); i++) {
            float d2 = aabb_distance_sq(car_x, car_z,
                                        g_loaded_sectors[i].bbox_min[0], g_loaded_sectors[i].bbox_min[2],
                                        g_loaded_sectors[i].bbox_max[0], g_loaded_sectors[i].bbox_max[2]);
            if (d2 > max_d2) {
                max_d2 = d2;
                furthest_idx = i;
            }
        }
        unload_sector(furthest_idx);
    }

    // 3. Find candidates to load within load distance
    struct Candidate {
        size_t index;
        float d2;
    };
    std::vector<Candidate> to_load;

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
            to_load.push_back({i, d2});
        }
    }

    // Sort by proximity (nearest sector loads first)
    std::sort(to_load.begin(), to_load.end(), [](const Candidate& a, const Candidate& b) {
        return a.d2 < b.d2;
    });

    // Budget: Load at most 1 sector per frame to eliminate stutter while driving fast
    if (!to_load.empty() && linearSpaceFree() >= MIN_LINEAR_FREE_BUDGET) {
        load_sector(g_manifest_sectors[to_load[0].index]);
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

// ---------------------------------------------------------------------------
// World Rendering & Frustum Draw
// ---------------------------------------------------------------------------
void world_mesh_render(const C3D_Mtx* projection, const C3D_Mtx* view, float car_x, float car_z) {
    if (g_streaming_mode) {
        update_streaming(car_x, car_z);
        if (g_loaded_sectors.empty()) return;
    } else {
        if (g_fallback_chunks.empty()) return;
    }

    extern int g_uLoc_mvp;

    C3D_Mtx model, mv, mvp;
    Mtx_Identity(&model);
    Mtx_Multiply(&mv, view, &model);
    Mtx_Multiply(&mvp, projection, &mv);
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, g_uLoc_mvp, &mvp);

    if (g_streaming_mode) {
        for (const auto& sec : g_loaded_sectors) {
            for (const auto& ch : sec.chunks) {
                if (!ch.vbo || !ch.ibo || ch.num_indices == 0) continue;

                C3D_BufInfo* bufInfo = C3D_GetBufInfo();
                BufInfo_Init(bufInfo);
                BufInfo_Add(bufInfo, ch.vbo, sizeof(MWVertex), 3, 0x210);

                C3D_DrawElements(GPU_TRIANGLES, ch.num_indices, C3D_UNSIGNED_SHORT, ch.ibo);
            }
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
