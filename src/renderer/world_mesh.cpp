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
// Baked collision data (tools/build_collision.py)
//
// A .mwcol sidecar contains a 2 m ground grid and 8 m-bucketed wall segments.
// Baking removed the per-triangle rasterizer that used to run on the 3DS and
// left a sector without usable collision for seconds after it streamed in.
// ---------------------------------------------------------------------------
static const float SECTOR_SIZE = 500.0f;
static const float GROUND_CELL = 2.0f;
static const int   GROUND_DIM  = 250; // 250 * 2 m = exactly one sector
static const int16_t GROUND_NONE = INT16_MAX;

static const int   COL_WALL_DIM = 96;      // 96 * 8 m = 768 m
static const float COL_WALL_CELL = 8.0f;
static const float COL_WALL_ORIGIN = -128.0f;
static const float GROUND_STEP_UP = 1.0f;  // highest walkable surface rule

#pragma pack(push, 1)
struct MWColHeader {
    char magic[4];        // "MWCL"
    uint32_t version;     // 2
    int32_t gx;
    int32_t gz;
    uint32_t ground_dim;
    float ground_cell;
    float origin_x;
    float origin_z;
    uint32_t num_segments;
    uint32_t wall_dim;
    float wall_cell;
    float wall_origin_x;
    float wall_origin_z;
    uint32_t num_refs;    // entries in the per-cell segment reference table
    uint32_t reserved[2];
};

struct MWColSeg {
    float x0, z0, x1, z1;
    int16_t y0, y1;       // centimetres
};
#pragma pack(pop)

static_assert(sizeof(MWColHeader) == 64, "MWColHeader must stay 64 bytes");
static_assert(sizeof(MWColSeg) == 20, "MWColSeg must stay 20 bytes");

struct LoadedSector {
    int32_t gx;
    int32_t gz;
    float bbox_min[3];
    float bbox_max[3];
    std::vector<WorldChunkMesh> chunks;
    std::vector<int16_t> ground_lo;
    std::vector<int16_t> ground_hi;
    std::vector<int32_t> wall_start;   // wall_dim^2 + 1 offsets into wall_refs
    std::vector<uint32_t> wall_refs;   // segment ids, grouped by 8 m cell
    std::vector<MWColSeg> wall_segs;
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
    uint32_t total_chunks;
    uint32_t chunks_loaded;
    bool active;

    FILE* col_file;
    std::vector<uint8_t> col_raw;
    size_t col_size;
    size_t col_read;
    bool col_done;
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

// Fallback monolithic chunks (if manifest not found)
static std::vector<WorldChunkMesh> g_fallback_chunks;
static std::vector<int16_t> g_fallback_ground_lo;
static std::vector<int16_t> g_fallback_ground_hi;
static const float FALLBACK_GROUND_MIN = -640.0f;
static const int   FALLBACK_GROUND_DIM = 321;

// Streaming Radii.  Keep roughly a sector of lookahead beyond the draw
// distance instead of filling memory with a 650 m ring of geometry that the
// camera never sees: loading far sectors one chunk per frame kept RomFS busy
// and was the main source of startup/driving hitches.
static const float STREAM_LOAD_DIST   = 400.0f;
static const float STREAM_UNLOAD_DIST = 550.0f;
static float g_draw_dist = 100.0f;
static uint32_t g_triangle_budget = 80000;
static size_t g_max_draw_chunks = 64;
static const size_t MIN_LINEAR_FREE_BUDGET = 2 * 1024 * 1024; // Keep at least 2 MB free

// How many metres of distance credit a BBox centre directly ahead earns when
// ordering streaming work.  Small enough that the sector under the car and
// its immediate neighbours always sort first.
static const float SECTOR_FORWARD_CREDIT = 90.0f;
static const float CHUNK_FORWARD_CREDIT  = 60.0f;

// Forward declarations
static bool start_sector_load(const MWSectorRecord& rec,
                              float car_x, float car_z,
                              float forward_x, float forward_z);
static bool continue_sector_load(void);
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

// Order streaming work by true distance to the bounding box, with a small
// credit for geometry directly ahead.  The previous
// distance_sq - forward * 400 form could give a sector 350 m ahead a lower
// priority number than the sector under the car, so the spawn area stayed
// empty for seconds while far-away chunks streamed in.  forward_weight is
// now in metres of effective distance (0..forward_weight), never enough to
// let something materially farther overtake a nearer sector.
static float stream_priority(float car_x, float car_z, float forward_x, float forward_z,
                             float distance_sq, float forward_weight,
                             float min_x, float min_z, float max_x, float max_z) {
    float dist = sqrtf(distance_sq);
    float cx = (min_x + max_x) * 0.5f;
    float cz = (min_z + max_z) * 0.5f;
    float dx = cx - car_x;
    float dz = cz - car_z;
    float center_dist = sqrtf(dx * dx + dz * dz);
    float flen = sqrtf(forward_x * forward_x + forward_z * forward_z);
    float ahead = 0.0f;
    if (center_dist > 1e-3f && flen > 1e-4f) {
        ahead = (dx * forward_x + dz * forward_z) / (center_dist * flen);
        if (ahead < 0.0f) ahead = 0.0f;
    }
    return dist - ahead * forward_weight;
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

// C3D_FVec stores components in PICA order (w,z,y,x), so matrix.m[] is not a
// plain row-major array: row i starts at m[i*4] with .w and ends with .x.
static void mtx_row_xyz(const float* m, int i, float out[4]) {
    out[0] = m[i * 4 + 3];
    out[1] = m[i * 4 + 2];
    out[2] = m[i * 4 + 1];
    out[3] = m[i * 4 + 0];
}

static void frustum_build(const C3D_Mtx& mvp, FrustumPlanes& planes) {
    float r0[4], r1[4], r2[4], r3[4];
    mtx_row_xyz(mvp.m, 0, r0);
    mtx_row_xyz(mvp.m, 1, r1);
    mtx_row_xyz(mvp.m, 2, r2);
    mtx_row_xyz(mvp.m, 3, r3);

    // Clip space is x,y in [-w,w] and z in [-1,0] (reversed-Z).  Side planes
    // are row3 +/- rowX; the near plane is row3 + row2.  The projection's
    // quarter-turn roll does not change the volume, only the clip axes.
    const float a[5][4] = {
        { r0[0] + r3[0], r0[1] + r3[1], r0[2] + r3[2], r0[3] + r3[3] },
        { r3[0] - r0[0], r3[1] - r0[1], r3[2] - r0[2], r3[3] - r0[3] },
        { r1[0] + r3[0], r1[1] + r3[1], r1[2] + r3[2], r1[3] + r3[3] },
        { r3[0] - r1[0], r3[1] - r1[1], r3[2] - r1[2], r3[3] - r1[3] },
        { r2[0] + r3[0], r2[1] + r3[1], r2[2] + r3[2], r2[3] + r3[3] },
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
        update_streaming(-148.0f, 164.0f, -0.5592f, -0.8290f);
        // Finish the spawn sector before the first frame.  The normal loader
        // commits one chunk per frame, so on hardware the road under the car
        // used to appear only after ~20 frames (and the old priority order
        // could put four far sectors in front of it).  One sector is a few MB
        // and is read before assets/atlas setup, which is already a load screen.
        while (g_sector_load.active) continue_sector_load();
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
// Baked Collision Loading
// ---------------------------------------------------------------------------
static void collider_close(void) {
    if (g_sector_load.col_file) {
        fclose(g_sector_load.col_file);
        g_sector_load.col_file = NULL;
    }
    g_sector_load.col_raw.clear();
    g_sector_load.col_size = 0;
    g_sector_load.col_read = 0;
}

static void collider_parse(void) {
    const uint8_t* base = g_sector_load.col_raw.data();
    MWColHeader hdr;
    memcpy(&hdr, base, sizeof(hdr));

    size_t gcount = (size_t)hdr.ground_dim * hdr.ground_dim;
    size_t wcount = (size_t)hdr.wall_dim * hdr.wall_dim;
    const uint8_t* p = base + sizeof(MWColHeader);

    LoadedSector& sec = g_sector_load.sector;
    sec.ground_lo.resize(gcount);
    memcpy(sec.ground_lo.data(), p, gcount * sizeof(int16_t));
    p += gcount * sizeof(int16_t);
    sec.ground_hi.resize(gcount);
    memcpy(sec.ground_hi.data(), p, gcount * sizeof(int16_t));
    p += gcount * sizeof(int16_t);
    sec.wall_start.resize(wcount + 1);
    memcpy(sec.wall_start.data(), p, (wcount + 1) * sizeof(int32_t));
    p += (wcount + 1) * sizeof(int32_t);
    sec.wall_refs.resize(hdr.num_refs);
    if (hdr.num_refs)
        memcpy(sec.wall_refs.data(), p, (size_t)hdr.num_refs * sizeof(uint32_t));
    p += (size_t)hdr.num_refs * sizeof(uint32_t);
    sec.wall_segs.resize(hdr.num_segments);
    if (hdr.num_segments)
        memcpy(sec.wall_segs.data(), p, (size_t)hdr.num_segments * sizeof(MWColSeg));
}

static bool collider_open(const MWSectorRecord& rec) {
    g_sector_load.col_done = true;
    g_sector_load.col_file = NULL;

    char name[40];
    snprintf(name, sizeof(name), "%s", rec.filename);
    char* dot = strrchr(name, '.');
    if (!dot) return false;
    snprintf(dot, (size_t)(sizeof(name) - (size_t)(dot - name)), ".mwcol");

    char path[128];
    snprintf(path, sizeof(path), "romfs:/assets/sectors/%s", name);
    FILE* f = fopen(path, "rb");
    if (!f) return false;

    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return false; }
    long size = ftell(f);
    if (size < (long)sizeof(MWColHeader) || fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return false;
    }

    MWColHeader hdr;
    if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr) ||
        memcmp(hdr.magic, "MWCL", 4) != 0 || hdr.version != 2 ||
        hdr.gx != rec.gx || hdr.gz != rec.gz ||
        hdr.ground_dim != GROUND_DIM || hdr.ground_cell != GROUND_CELL ||
        hdr.origin_x != (float)rec.gx * SECTOR_SIZE ||
        hdr.origin_z != (float)rec.gz * SECTOR_SIZE ||
        hdr.wall_dim != COL_WALL_DIM || hdr.wall_cell != COL_WALL_CELL ||
        hdr.wall_origin_x != COL_WALL_ORIGIN ||
        hdr.wall_origin_z != COL_WALL_ORIGIN ||
        hdr.num_segments > 250000 || hdr.num_refs > 2000000) {
        fclose(f);
        return false;
    }
    size_t expected = sizeof(MWColHeader) +
                      (size_t)hdr.ground_dim * hdr.ground_dim * 2 * sizeof(int16_t) +
                      ((size_t)hdr.wall_dim * hdr.wall_dim + 1) * sizeof(int32_t) +
                      (size_t)hdr.num_refs * sizeof(uint32_t) +
                      (size_t)hdr.num_segments * sizeof(MWColSeg);
    if ((size_t)size != expected) {
        fclose(f);
        return false;
    }

    // The whole file is read in 64 KB slices from the frame loop; only the
    // header is consumed here so sector selection stays cheap.
    g_sector_load.col_raw.resize((size_t)size);
    memcpy(g_sector_load.col_raw.data(), &hdr, sizeof(hdr));
    g_sector_load.col_size = (size_t)size;
    g_sector_load.col_read = sizeof(hdr);
    g_sector_load.col_file = f;
    g_sector_load.col_done = false;
    return true;
}

static bool collider_load_slice(void) {
    if (g_sector_load.col_done) return true;
    size_t remaining = g_sector_load.col_size - g_sector_load.col_read;
    size_t want = remaining < 64u * 1024u ? remaining : 64u * 1024u;
    if (want) {
        if (fread(g_sector_load.col_raw.data() + g_sector_load.col_read, 1, want,
                  g_sector_load.col_file) != want) {
            collider_close();
            g_sector_load.col_done = true;
            return false;
        }
        g_sector_load.col_read += want;
    }
    if (g_sector_load.col_read >= g_sector_load.col_size) {
        fclose(g_sector_load.col_file);
        g_sector_load.col_file = NULL;
        collider_parse();
        g_sector_load.col_raw.clear();
        g_sector_load.col_done = true;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Dynamic Sector Loading & Unloading
// ---------------------------------------------------------------------------
static void cancel_sector_load(void) {
    if (!g_sector_load.active) return;
    if (g_sector_load.file) fclose(g_sector_load.file);
    collider_close();
    g_sector_load.col_done = true;
    for (auto& cm : g_sector_load.sector.chunks) {
        if (cm.vbo) linearFree(cm.vbo);
        if (cm.ibo) linearFree(cm.ibo);
    }
    g_sector_load.sector = LoadedSector();
    g_sector_load.chunk_order.clear();
    g_sector_load.file = NULL;
    g_sector_load.total_chunks = 0;
    g_sector_load.chunks_loaded = 0;
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
        float priority = stream_priority(car_x, car_z, forward_x, forward_z, d2, CHUNK_FORWARD_CREDIT,
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
    g_sector_load.active = true;

    // Collision is a sidecar read in slices by the frame loop. If it is
    // missing the sector still renders; the car simply gets no wall data.
    collider_open(rec);
    return true;
}

static bool continue_sector_load(void) {
    if (!g_sector_load.active) return false;

    // Spread the collider read across frames as well. The file is small
    // (a few hundred KB) and finishes long before the sector is reached.
    if (!g_sector_load.col_done) collider_load_slice();

    if (g_sector_load.chunks_loaded >= g_sector_load.total_chunks &&
        g_sector_load.col_done) {
        fclose(g_sector_load.file);
        g_sector_load.file = NULL;
        g_loaded_sectors.push_back(std::move(g_sector_load.sector));
        g_sector_load.chunk_order.clear();
        g_sector_load.total_chunks = 0;
        g_sector_load.chunks_loaded = 0;
        g_sector_load.active = false;
        return true;
    }

    if (g_sector_load.chunks_loaded >= g_sector_load.total_chunks)
        return true;

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
    g_sector_load.chunks_loaded++;
    return true;
}

static void unload_sector(size_t index) {
    if (index >= g_loaded_sectors.size()) return;
    LoadedSector& sec = g_loaded_sectors[index];
    for (auto& cm : sec.chunks) {
        if (cm.vbo) { linearFree(cm.vbo); cm.vbo = NULL; }
        if (cm.ibo) { linearFree(cm.ibo); cm.ibo = NULL; }
    }
    sec.chunks.clear();
    // Return the (up to 1.8 MB) collision arrays to the heap while the
    // sector is out of range instead of keeping their capacity cached.
    std::vector<int16_t>().swap(sec.ground_lo);
    std::vector<int16_t>().swap(sec.ground_hi);
    std::vector<int32_t>().swap(sec.wall_start);
    std::vector<uint32_t>().swap(sec.wall_refs);
    std::vector<MWColSeg>().swap(sec.wall_segs);
    g_loaded_sectors.erase(g_loaded_sectors.begin() + index);
}

static void update_streaming(float car_x, float car_z, float forward_x, float forward_z) {
    if (!g_streaming_mode || g_manifest_sectors.empty()) return;
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
            float priority = stream_priority(car_x, car_z, forward_x, forward_z, d2, SECTOR_FORWARD_CREDIT,
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
            float priority = stream_priority(car_x, car_z, forward_x, forward_z, d2, SECTOR_FORWARD_CREDIT,
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
                float priority = stream_priority(car_x, car_z, forward_x, forward_z, d2, SECTOR_FORWARD_CREDIT,
                                                 resident.bbox_min[0], resident.bbox_min[2],
                                                 resident.bbox_max[0], resident.bbox_max[2]);
                if (priority > furthest_priority) {
                    furthest_priority = priority;
                    furthest_idx = i;
                }
            }
            // Priority is now a distance in metres, so a couple of hundred
            // metres of hysteresis replaces the old 25,000 (squared-metre)
            // margin and still prevents neighbour thrash at sector borders.
            const float replace_margin = 150.0f;
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
// A cell stores the lowest and highest drivable surface crossing it, so a
// cell under an overpass has two candidates.  Pick the highest surface the
// car can step up to; only when nothing qualifies (airborne over a drop) fall
// back to the lowest surface so the car still converges on the ground.
static float pick_ground_surface(int16_t lo, int16_t hi, float ref_y, float fallback) {
    float best = -1e9f;
    float lowest = 1e9f;
    bool found = false;
    if (lo != GROUND_NONE) {
        float y = (float)lo / 100.0f;
        lowest = fminf(lowest, y);
        if (y <= ref_y + GROUND_STEP_UP && y > best) { best = y; found = true; }
    }
    if (hi != INT16_MIN) {
        float y = (float)hi / 100.0f;
        lowest = fminf(lowest, y);
        if (y <= ref_y + GROUND_STEP_UP && y > best) { best = y; found = true; }
    }
    if (found) return best;
    if (lowest < 1e8f) return lowest;
    return fallback;
}

static bool sector_ground_at(const LoadedSector& sec, float x, float z,
                             float ref_y, float fallback, float* out_y) {
    if (sec.ground_lo.empty()) return false;
    float sec_min_x = sec.gx * SECTOR_SIZE;
    float sec_min_z = sec.gz * SECTOR_SIZE;
    int ix = (int)floorf((x - sec_min_x) / GROUND_CELL);
    int iz = (int)floorf((z - sec_min_z) / GROUND_CELL);
    if (ix < 0 || ix >= GROUND_DIM || iz < 0 || iz >= GROUND_DIM) return false;

    size_t idx = (size_t)iz * GROUND_DIM + ix;
    *out_y = pick_ground_surface(sec.ground_lo[idx], sec.ground_hi[idx],
                                 ref_y, fallback);
    return true;
}

float world_mesh_ground_height(float x, float z, float ref_y, float fallback) {
    if (!g_streaming_mode) {
        if (g_fallback_ground_lo.empty()) return fallback;
        int ix = (int)floorf((x - FALLBACK_GROUND_MIN) / GROUND_CELL);
        int iz = (int)floorf((z - FALLBACK_GROUND_MIN) / GROUND_CELL);
        if (ix < 0 || ix >= FALLBACK_GROUND_DIM || iz < 0 || iz >= FALLBACK_GROUND_DIM) return fallback;
        size_t idx = (size_t)iz * FALLBACK_GROUND_DIM + ix;
        return pick_ground_surface(g_fallback_ground_lo[idx],
                                   g_fallback_ground_hi[idx], ref_y, fallback);
    }

    // Height grids tile the world exactly, so the containing sector owns the
    // cell.  Scan residents and the sector being streamed: a collider is
    // committed before all of its render chunks, and the car needs the floor
    // as soon as it arrives.
    float y;
    for (const auto& sec : g_loaded_sectors) {
        if (sector_ground_at(sec, x, z, ref_y, fallback, &y)) return y;
    }
    if (g_sector_load.active &&
        sector_ground_at(g_sector_load.sector, x, z, ref_y, fallback, &y))
        return y;
    return fallback;
}

// ---------------------------------------------------------------------------
// Wall Collision (baked 8 m segment buckets)
// ---------------------------------------------------------------------------
// The car is approximated by a vertical cylinder: only walls whose height
// range overlaps the body (not kerbs under the wheels, not signs overhead)
// push it out.
static const float CAR_CLEARANCE_LOW  = 0.20f;
static const float CAR_CLEARANCE_HIGH = 1.50f;
static const int   WALL_SEG_ITERATIONS = 4;

static bool resolve_walls_in_sector(const LoadedSector& sec,
                                    float* x, float* z,
                                    float feet_y, float radius) {
    if (sec.wall_segs.empty() || sec.wall_refs.empty() ||
        sec.wall_start.empty()) return false;
    float lx = *x - sec.gx * SECTOR_SIZE;
    float lz = *z - sec.gz * SECTOR_SIZE;
    int cx = (int)floorf((lx - COL_WALL_ORIGIN) / COL_WALL_CELL);
    int cz = (int)floorf((lz - COL_WALL_ORIGIN) / COL_WALL_CELL);
    if (cx < -1 || cz < -1 || cx > COL_WALL_DIM || cz > COL_WALL_DIM) return false;

    bool moved = false;
    // One bucket ring covers the car radius; the ring is re-read on every
    // iteration because a push can leave the original cell.
    for (int bz = cz - 1; bz <= cz + 1; bz++) {
        if (bz < 0 || bz >= COL_WALL_DIM) continue;
        for (int bx = cx - 1; bx <= cx + 1; bx++) {
            if (bx < 0 || bx >= COL_WALL_DIM) continue;
            size_t cell = (size_t)bz * COL_WALL_DIM + bx;
            int32_t begin = sec.wall_start[cell];
            int32_t end = sec.wall_start[cell + 1];
            if (begin < 0 || end > (int32_t)sec.wall_refs.size()) continue;
            for (int32_t r = begin; r < end; r++) {
                uint32_t si = sec.wall_refs[r];
                if (si >= sec.wall_segs.size()) continue;
                const MWColSeg& s = sec.wall_segs[si];
                float y0 = (float)s.y0 / 100.0f;
                float y1 = (float)s.y1 / 100.0f;
                if (y1 < feet_y + CAR_CLEARANCE_LOW ||
                    y0 > feet_y + CAR_CLEARANCE_HIGH) continue;

                float ex = s.x1 - s.x0;
                float ez = s.z1 - s.z0;
                float len2 = ex * ex + ez * ez;
                if (len2 < 1e-6f) continue;
                float t = ((*x - s.x0) * ex + (*z - s.z0) * ez) / len2;
                t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
                float px = s.x0 + ex * t;
                float pz = s.z0 + ez * t;
                float nx = *x - px;
                float nz = *z - pz;
                float d2 = nx * nx + nz * nz;
                if (d2 >= radius * radius) continue;

                float d = sqrtf(d2);
                if (d < 1e-4f) {
                    // Centre exactly on the wall: use the segment normal so
                    // the push has a defined direction.
                    float inv = 1.0f / sqrtf(len2);
                    nx = -ez * inv;
                    nz = ex * inv;
                    d = 0.0f;
                } else {
                    nx /= d;
                    nz /= d;
                }
                float push = radius - d + 0.002f;
                *x += nx * push;
                *z += nz * push;
                moved = true;
            }
        }
    }
    return moved;
}

static void resolve_walls(float* x, float* z, float feet_y, float radius) {
    for (int iter = 0; iter < WALL_SEG_ITERATIONS; iter++) {
        bool moved = false;
        for (const auto& sec : g_loaded_sectors)
            moved |= resolve_walls_in_sector(sec, x, z, feet_y, radius);
        if (g_sector_load.active)
            moved |= resolve_walls_in_sector(g_sector_load.sector, x, z,
                                             feet_y, radius);
        if (!moved) break;
    }
}

float world_mesh_move(float* x, float* z, float target_x, float target_z,
                      float feet_y, float radius) {
    float ox = *x;
    float oz = *z;
    float dx = target_x - ox;
    float dz = target_z - oz;
    float dist = sqrtf(dx * dx + dz * dz);
    if (!g_streaming_mode || dist < 1e-4f) {
        *x = target_x;
        *z = target_z;
        // Still separate a stationary car that ended up overlapping a wall
        // (reset, teleport, or a collider that finished loading under it).
        if (g_streaming_mode) resolve_walls(x, z, feet_y, radius);
        return 0.0f;
    }

    // Sub-step so fast frames cannot tunnel through a wall.  Every step is
    // resolved immediately, which is what turns a head-on impact into a stop
    // and a glancing impact into a slide.
    int steps = (int)ceilf(dist / 0.75f);
    if (steps < 1) steps = 1;
    if (steps > 12) steps = 12;
    float step_x = dx / (float)steps;
    float step_z = dz / (float)steps;
    float px = ox;
    float pz = oz;
    for (int i = 0; i < steps; i++) {
        px += step_x;
        pz += step_z;
        resolve_walls(&px, &pz, feet_y, radius);
    }

    *x = px;
    *z = pz;
    // Fraction of the requested motion lost to walls.  A glancing hit keeps
    // most of the speed; a perpendicular hit reports ~1.0.
    float progress = ((px - ox) * dx + (pz - oz) * dz) / (dist * dist);
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;
    return 1.0f - progress;
}

// ---------------------------------------------------------------------------
// World Rendering & Frustum Draw
// ---------------------------------------------------------------------------
void world_mesh_set_quality(bool high) {
    if (high) {
        g_draw_dist = 140.0f;
        g_triangle_budget = 120000;
        g_max_draw_chunks = 96;
    } else {
        g_draw_dist = 100.0f;
        g_triangle_budget = 80000;
        g_max_draw_chunks = 64;
    }
}

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
        static const size_t MAX_DRAW_SLOTS = 96;
        DrawCandidate draw_list[MAX_DRAW_SLOTS];
        size_t draw_count = 0;
        const float draw_dist_sq = g_draw_dist * g_draw_dist;
        auto consider_chunk = [&](const WorldChunkMesh& ch) {
            if (!ch.vbo || !ch.ibo || ch.num_indices == 0) return;
            float distance_sq = aabb_distance_sq(car_x, car_z,
                                                 ch.bbox_min[0], ch.bbox_min[2],
                                                 ch.bbox_max[0], ch.bbox_max[2]);
            if (distance_sq > draw_dist_sq ||
                chunk_outside_frustum(frustum, ch)) return;
            // Draw nearest-first.  The old forward-weighted order could put a
            // chunk 90 m ahead before the chunk under the car, and the
            // triangle budget then skipped the near chunk entirely.
            float priority = distance_sq;
            size_t at = 0;
            while (at < draw_count && draw_list[at].priority <= priority) at++;
            if (at >= g_max_draw_chunks) return;
            if (draw_count < g_max_draw_chunks) draw_count++;
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

        // Draw near-to-far and stop at the first chunk that does not fit.
        // Skipping a big near chunk to fill the budget with smaller, farther
        // chunks is what produced missing road under the car.  Chunks within
        // 30 m may overrun the budget slightly so the immediate ground always
        // renders, but the total stays inside a hard 25% cap.
        uint32_t triangles_drawn = 0;
        const uint32_t hard_cap = g_triangle_budget + g_triangle_budget / 4;
        const float near_guard_sq = 30.0f * 30.0f;
        for (size_t i = 0; i < draw_count; i++) {
            const WorldChunkMesh& ch = *draw_list[i].chunk;
            uint32_t triangles = ch.num_indices / 3;
            if (triangles_drawn + triangles > hard_cap) break;
            bool near_chunk = draw_list[i].priority <= near_guard_sq;
            if (triangles_drawn + triangles > g_triangle_budget && !near_chunk) break;

            C3D_BufInfo* bufInfo = C3D_GetBufInfo();
            BufInfo_Init(bufInfo);
            BufInfo_Add(bufInfo, ch.vbo, sizeof(MWVertex), 3, 0x210);
            C3D_DrawElements(GPU_TRIANGLES, ch.num_indices, C3D_UNSIGNED_SHORT, ch.ibo);
            triangles_drawn += triangles;
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
        fprintf(f, "sec (%ld, %ld) chunks %lu\n", (long)sec.gx, (long)sec.gz,
                (unsigned long)sec.chunks.size());
    }
    fclose(f);
}

void world_mesh_exit(void) {
    cancel_sector_load();
    std::vector<MWVertex>().swap(g_load_verts);
    std::vector<uint16_t>().swap(g_load_indices);
    std::vector<StreamCandidate>().swap(g_stream_candidates);
    std::vector<MWSectorRecord>().swap(g_manifest_sectors);
    for (auto& sec : g_loaded_sectors) {
        for (auto& ch : sec.chunks) {
            if (ch.vbo) { linearFree(ch.vbo); ch.vbo = NULL; }
            if (ch.ibo) { linearFree(ch.ibo); ch.ibo = NULL; }
        }
        sec.chunks.clear();
        sec.ground_lo.clear();
        sec.ground_hi.clear();
        sec.wall_start.clear();
        sec.wall_refs.clear();
        sec.wall_segs.clear();
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
