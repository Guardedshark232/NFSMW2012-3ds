#!/usr/bin/env python3
"""Extract real Fairhaven world geometry from MW2012 .BNDL files into .mwmesh.

Format (.mwmesh v1):
  Header (16 bytes):
    magic[4]   = b"MWMS"
    version    = 1 (u32)
    numChunks  = N (u32)
    reserved   = 0 (u32)
  Per chunk (40-byte header + vertices + indices):
    numVerts   (u32)
    numIndices (u32)
    bboxMin[3] (f32 * 3)
    bboxMax[3] (f32 * 3)
    reserved[2](u32 * 2)
    Vertex[numVerts]:   float x, y, z, r, g, b  (24 bytes each)
    Index[numIndices]:  uint16_t                (2 bytes each, padded to 4-byte align)
"""
import os
import sys
import struct
import math
import json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bndl import Bundle
from mwmesh import extract_mesh, MAX_TRI_EDGE
import build_atlas
import yard_props
from game_paths import MW_DIR, require_game_files

# Packed vertex: float3 position + u8x4 tint + u16x2 atlas UV (20 bytes).
VERTEX_SIZE = 20


def face_normal_unnorm(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    return (nx, ny, nz)


def shade_color(nx, ny, nz, y_rel, lx, lz):
    """Neutral lighting tint multiplied over the diffuse texture.

    The runtime now samples real game albedo, so this no longer invents
    material colours: it only bakes a little directional shading and height
    ambient into the vertex colour channel, keeping the low-poly city from
    reading as flat paint.
    """
    l = math.sqrt(nx * nx + ny * ny + nz * nz)
    if l > 1e-8:
        nx, ny, nz = nx / l, ny / l, nz / l
    else:
        nx, ny, nz = 0.0, 1.0, 0.0
    ndotl = max(0.0, nx * 0.45 + ny * 0.72 + nz * 0.40)
    ambient = 0.56 + 0.10 * max(0.0, min(1.0, y_rel))
    lit = min(1.0, ambient + 0.44 * ndotl)
    return (lit, lit * 0.985, lit * 0.955)


def fit_part_uvs(uvs):
    """Fit a part's (possibly tiled) UV set into the [0,1] tile rect.

    Chameleon materials tile their textures with UVs far outside [0,1].
    The atlas holds a single repeat per tile and uses CLAMP_TO_EDGE (a
    REPEAT wrap would bleed into the neighbouring tile), so the part's UVs
    are fitted into its tile here: sub-tile sets are integer-shifted back
    into range, larger sets are squeezed to span exactly one tile, which
    keeps the pattern count visible on big facades and roads.
    """
    if not uvs:
        return uvs
    # Ignore absurd values when measuring the range; some submeshes carry
    # leftover UV attributes that would otherwise collapse the mapping.
    finite = [(u, v) for u, v in uvs
              if abs(u) < 64.0 and abs(v) < 64.0] or uvs
    us = [u for u, _ in finite]
    vs = [v for _, v in finite]
    umin, umax = min(us), max(us)
    vmin, vmax = min(vs), max(vs)
    du = umax - umin
    dv = vmax - vmin
    if du > 1.0:
        base_u, scale_u = umin, 1.0 / du
    else:
        base_u, scale_u = math.floor(umin), 1.0
    if dv > 1.0:
        base_v, scale_v = vmin, 1.0 / dv
    else:
        base_v, scale_v = math.floor(vmin), 1.0
    return [(max(0.0, min(1.0, (u - base_u) * scale_u)),
             max(0.0, min(1.0, (v - base_v) * scale_v)))
            for u, v in uvs]


def build_indexed_chunk(parts, ox, oy, oz, slot_of):
    """Merge a list of decoded parts into a single indexed chunk.

    Each part carries its own UV set and material id; UVs are remapped into
    the material's atlas tile here, then quantised to uint16 so the 3DS
    vertex stays at 20 bytes.
    """
    out_verts = []
    out_indices = []
    mn = [1e30, 1e30, 1e30]
    mx = [-1e30, -1e30, -1e30]

    for part in parts:
        src_verts, src_tris = part["verts"], part["tris"]
        if not src_tris:
            continue
        uvs = fit_part_uvs(part["uvs"])
        has_uv = part.get("has_uv", True)
        origin = slot_of.get(part["material"], build_atlas.FALLBACK)
        # Accumulate smooth normals for used vertices
        used_map = {}
        normals = {}
        valid_tris = []
        for ia, ib, ic in src_tris:
            A, B, C = src_verts[ia], src_verts[ib], src_verts[ic]
            # Drop leftover decoder artifacts: strip padding can weld two
            # distant vertex clusters into enormous flat "sails".
            if max(math.dist(A, B), math.dist(B, C), math.dist(C, A)) > MAX_TRI_EDGE:
                continue
            fn = face_normal_unnorm(A, B, C)
            for idx in (ia, ib, ic):
                if idx not in used_map:
                    used_map[idx] = len(out_verts) + len(used_map)
                    normals[idx] = [0.0, 0.0, 0.0]
                n = normals[idx]
                n[0] += fn[0]
                n[1] += fn[1]
                n[2] += fn[2]
            valid_tris.append((ia, ib, ic))

        if not valid_tris:
            continue

        # Emit remapped vertices in insertion order
        ordered = sorted(used_map.items(), key=lambda kv: kv[1])
        for src_idx, _ in ordered:
            vx, vy, vz = src_verts[src_idx]
            lx, ly, lz = vx - ox, vy - oy, vz - oz
            nx, ny, nz = normals[src_idx]
            y_rel = (ly + 10.0) / 80.0
            cr, cg, cb = shade_color(nx, ny, nz, y_rel, lx, lz)
            # Procedural dressing can bias the baked shading (brighten a dark
            # road-facing container wall, darken gantry steel, ...).
            pc = part.get("color")
            if pc:
                cr = min(1.0, cr * pc[0])
                cg = min(1.0, cg * pc[1])
                cb = min(1.0, cb * pc[2])
            u, v = uvs[src_idx] if has_uv else (0.5, 0.5)
            au, av = build_atlas.uv_remap(u, v, origin)
            # PICA only has signed 16-bit integer attributes, so store
            # atlas UVs biased into [-32768, 32767]; the shader scales by
            # 1/65535 and adds 32768/65535 to recover 0..1.
            iu = max(-32768, min(32767, int(round(au * 65535.0 - 32768.0))))
            iv = max(-32768, min(32767, int(round(av * 65535.0 - 32768.0))))
            out_verts.append((lx, ly, lz, cr, cg, cb, iu, iv))
            mn[0] = min(mn[0], lx); mx[0] = max(mx[0], lx)
            mn[1] = min(mn[1], ly); mx[1] = max(mx[1], ly)
            mn[2] = min(mn[2], lz); mx[2] = max(mx[2], lz)

        for ia, ib, ic in valid_tris:
            out_indices.extend((used_map[ia], used_map[ib], used_map[ic]))

    return dict(verts=out_verts, indices=out_indices, bbox_min=mn, bbox_max=mx)


def _pack_positions(verts, quantum=0.5, max_keys=5000):
    """Quantise vertex positions into int64 keys for set comparisons.

    Subsampled to max_keys unique positions so LOD dedupe stays fast even
    for large meshes (50k+ verts per TRK_UNIT renderable).
    """
    import numpy as np
    q = np.round(np.asarray(verts, dtype=np.float32) / quantum).astype(np.int64)
    q = np.unique(q, axis=0) + 4096
    keys = (q[:, 0] << 28) | (q[:, 1] << 14) | q[:, 2]
    if len(keys) > max_keys:
        keys = keys[::len(keys) // max_keys + 1]
    return keys


def _containment_fraction(small_keys, big_keys, max_sample=2000):
    """Fraction of `small_keys` present in `big_keys` (within one quantum).

    Uses a Python set for O(1) big_keys lookups and subsamples small_keys so
    this stays fast even when both meshes are large (TRK_UNIT geometry).
    """
    if len(small_keys) == 0:
        return 0.0
    shifts = [a * (1 << 28) + b * (1 << 14) + c
              for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]
    big_set = set(int(k) for k in big_keys)
    step = max(1, len(small_keys) // max_sample)
    sample = small_keys[::step]
    hits = sum(1 for k in sample if any((int(k) + s) in big_set for s in shifts))
    return hits / len(sample)


def dedupe_lod_resources(meshes, min_containment=0.6, max_centre_distance=60.0,
                         verbose=True):
    """Keep one renderable per LOD/duplicate cluster.

    Chameleon bundles contain several detail levels (and sometimes exact
    streaming copies) of the same block.  They share a centre and their
    vertex sets are largely contained in the highest-detail member's set,
    so drawing all of them stacks several copies of the world in front of
    the camera.  Cluster by centre proximity + vertex containment and keep
    the member with the most triangles.
    """
    import numpy as np  # noqa: F401  (used by helpers)
    items = []
    for m in meshes:
        verts = [v for p in m["parts"] for v in p["verts"]]
        if not verts:
            continue
        items.append(dict(mesh=m,
                          tris=sum(len(p["tris"]) for p in m["parts"]),
                          keys=_pack_positions(verts)))

    n = len(items)
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for i in range(n):
        a = items[i]
        ax, ay, az = a["mesh"]["centre"]
        for j in range(i + 1, n):
            b = items[j]
            bx, by, bz = b["mesh"]["centre"]
            if math.dist((ax, ay, az), (bx, by, bz)) > max_centre_distance:
                continue
            small, big = (a, b) if len(a["keys"]) <= len(b["keys"]) else (b, a)
            if _containment_fraction(small["keys"], big["keys"]) >= min_containment:
                union(i, j)

    groups = {}
    for i, it in enumerate(items):
        groups.setdefault(find(i), []).append(it)
    winners = [max(g, key=lambda it: it["tris"]) for g in groups.values()]
    kept = [it["mesh"] for it in winners]
    total_tris = sum(it["tris"] for it in items)
    kept_tris = sum(it["tris"] for it in winners)
    if verbose:
        print(f"LOD dedupe: {len(meshes)} renderables -> {len(kept)} "
              f"(dropped {len(meshes) - len(kept)} LOD copies); "
              f"dropped {(total_tris - kept_tris) // 3} duplicate tris")
    return kept


def make_yard_decorator(origin):
    """Return a callback that appends procedural container-yard dressing.

    ``yard_props`` works in the renderer's mesh-local space (the same space
    the car and camera use), while ``collect_world_chunks`` carries parts in
    world coordinates, so convert around the call.
    """
    ox, oy, oz = origin

    def decorate(parts):
        local = [dict(verts=[(x - ox, y - oy, z - oz) for x, y, z in p["verts"]],
                      tris=p["tris"])
                 for p in parts]
        extra = yard_props.build_parts(local)
        for p in extra:
            p["verts"] = [(x + ox, y + oy, z + oz) for x, y, z in p["verts"]]
        return extra

    return decorate


def collect_world_chunks(bundle_names, origin, radius, slot_of, max_verts_per_chunk=32000,
                         decorate=None):
    ox, oy, oz = origin
    seen_ids = set()
    candidates = []

    for bname in bundle_names:
        path = os.path.join(MW_DIR, bname)
        if not os.path.exists(path):
            continue
        b = Bundle(path)
        for r in b.resources:
            if r.type_id != 5 or not r.has_stream1 or r.id in seen_ids:
                continue
            m = extract_mesh(r, b)
            if not m:
                continue
            cx, cy, cz = m["centre"]
            # Skip origin-centered local props/skyboxes
            if abs(cx) < 50.0 and abs(cz) < 50.0:
                continue
            # Skip background shells (sky domes, ocean/terrain rings, distant
            # skyline).  They are several hundred metres across and their
            # giant slabs swallow the chase camera, filling the screen with
            # bands of flat colour.  District/road chunks are long but thin,
            # so a radius test keeps them while dropping the shells.
            if m["radius"] > 500.0:
                continue
            if math.hypot(cx - ox, cz - oz) > radius:
                continue
            seen_ids.add(r.id)
            candidates.append(m)

    selected_parts = []  # decoded parts with verts/uvs/tris/material
    for m in dedupe_lod_resources(candidates):
        for p in m["parts"]:
            if p["tris"]:
                selected_parts.append(p)

    # Optional decorative dressing (container stacks, gantry cranes, ...).
    if decorate is not None:
        selected_parts.extend(decorate(selected_parts))

    # Pack into chunks with <= max_verts_per_chunk vertices
    chunks = []
    cur_parts = []
    cur_verts = 0
    for part in selected_parts:
        verts = part["verts"]
        if cur_parts and cur_verts + len(verts) > max_verts_per_chunk:
            ch = build_indexed_chunk(cur_parts, ox, oy, oz, slot_of)
            if ch["indices"]:
                chunks.append(ch)
            cur_parts = []
            cur_verts = 0
        cur_parts.append(part)
        cur_verts += len(verts)

    if cur_parts:
        ch = build_indexed_chunk(cur_parts, ox, oy, oz, slot_of)
        if ch["indices"]:
            chunks.append(ch)

    return chunks


def clip_chunks(chunks, radius):
    """Drop triangles whose centroid lies outside the (local) XZ radius,
    then compact the vertex/index lists.  Used to keep the GPU-side world
    small enough for the 3DS linear heap."""
    out = []
    r2 = radius * radius
    for c in chunks:
        vs = c["verts"]
        ix = c["indices"]
        kept = []
        for j in range(0, len(ix) - 2, 3):
            i0, i1, i2 = ix[j], ix[j + 1], ix[j + 2]
            cx = (vs[i0][0] + vs[i1][0] + vs[i2][0]) / 3.0
            cz = (vs[i0][2] + vs[i1][2] + vs[i2][2]) / 3.0
            if cx * cx + cz * cz <= r2:
                kept.append((i0, i1, i2))
        if not kept:
            continue
        remap = {}
        nv = []
        ni = []
        mn = [1e30, 1e30, 1e30]
        mx = [-1e30, -1e30, -1e30]
        for tri in kept:
            for i in tri:
                if i not in remap:
                    remap[i] = len(nv)
                    v = vs[i]
                    nv.append(v)
                    for k in range(3):
                        mn[k] = min(mn[k], v[k])
                        mx[k] = max(mx[k], v[k])
                ni.append(remap[i])
        out.append(dict(verts=nv, indices=ni, bbox_min=mn, bbox_max=mx))
    return out


def weld_chunks(chunks, tolerance=0.002):
    """Merge duplicate vertices (same position within tolerance) inside each
    chunk and drop zero-area/duplicate triangles.

    The Chameleon renderables split geometry into many submeshes, so the raw
    extraction repeats each corner position several times.  Welding cuts the
    GPU vertex buffer by roughly 3x; baked colours of coincident corners are
    averaged, which also smooths the low-poly shading a little.
    """
    out = []
    q = 1.0 / tolerance
    for c in chunks:
        vs = c["verts"]
        ix = c["indices"]
        vmap = {}
        pos = []
        col_sum = []
        col_n = []
        ni = []
        for j in range(0, len(ix) - 2, 3):
            tri = (ix[j], ix[j + 1], ix[j + 2])
            a, b, cc = vs[tri[0]], vs[tri[1]], vs[tri[2]]
            # Skip triangles with negligible area (degenerate after merging).
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy, vz = cc[0] - a[0], cc[1] - a[1], cc[2] - a[2]
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            if nx * nx + ny * ny + nz * nz < 1e-8:
                continue
            for i in tri:
                v = vs[i]
                key = (int(round(v[0] * q)), int(round(v[1] * q)), int(round(v[2] * q)), v[6], v[7])
                dst = vmap.get(key)
                if dst is None:
                    dst = len(pos)
                    vmap[key] = dst
                    pos.append((v[0], v[1], v[2], v[6], v[7]))
                    col_sum.append([v[3], v[4], v[5]])
                    col_n.append(1)
                else:
                    s = col_sum[dst]
                    s[0] += v[3]; s[1] += v[4]; s[2] += v[5]
                    col_n[dst] += 1
                ni.append(dst)
        if not ni:
            continue
        nv = []
        mn = [1e30, 1e30, 1e30]
        mx = [-1e30, -1e30, -1e30]
        for i, p in enumerate(pos):
            n = col_n[i]
            v = (p[0], p[1], p[2],
                 col_sum[i][0] / n, col_sum[i][1] / n, col_sum[i][2] / n,
                 p[3], p[4])
            nv.append(v)
            for k in range(3):
                mn[k] = min(mn[k], v[k])
                mx[k] = max(mx[k], v[k])
        out.append(dict(verts=nv, indices=ni, bbox_min=mn, bbox_max=mx))
    return out


def write_mwmesh(chunks, out_path):
    """Write .mwmesh v3.

    Vertex: float32 x, y, z + uint8 r, g, b, a + uint16 u, v  (20 bytes).
    UVs are already mapped into the world atlas; the colour channel holds a
    neutral lighting tint that modulates the sampled texture.
    """
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(struct.pack("<4sIII", b"MWMS", 3, len(chunks), 0))
        for c in chunks:
            nv = len(c["verts"])
            ni = len(c["indices"])
            mn = c["bbox_min"]
            mx = c["bbox_max"]
            f.write(struct.pack("<II6f2I", nv, ni,
                                mn[0], mn[1], mn[2], mx[0], mx[1], mx[2], 0, 0))
            for v in c["verts"]:
                f.write(struct.pack("<3f4B2h", v[0], v[1], v[2],
                                    int(round(v[3] * 255.0)),
                                    int(round(v[4] * 255.0)),
                                    int(round(v[5] * 255.0)), 255,
                                    v[6], v[7]))
            f.write(struct.pack("<%dH" % ni, *c["indices"]))
            if ni & 1:
                f.write(b"\x00\x00")


def render_topdown_preview(chunks, out_png, size=512, span=550.0):
    from PIL import Image, ImageDraw
    img = Image.new("RGB", (size, size), (14, 18, 28))
    draw = ImageDraw.Draw(img)
    tris = []
    for c in chunks:
        vs = c["verts"]
        ix = c["indices"]
        for i in range(0, len(ix), 3):
            v0, v1, v2 = vs[ix[i]], vs[ix[i + 1]], vs[ix[i + 2]]
            ym = (v0[1] + v1[1] + v2[1]) / 3.0
            tris.append((ym, v0, v1, v2))
    tris.sort(key=lambda t: t[0])
    half = size * 0.5
    scale = half / span
    for _, v0, v1, v2 in tris:
        pts = [(half + v[0] * scale, half - v[2] * scale) for v in (v0, v1, v2)]
        col = (int(v0[3] * 255), int(v0[4] * 255), int(v0[5] * 255))
        draw.polygon(pts, fill=col)
    img.save(out_png)


if __name__ == "__main__":
    require_game_files()
    # ARENA_527911 (Fairhaven harbour district).  The chosen spawn is the
    # flat harbour apron at world (-168, 30, 4058); making that the local
    # origin puts the car at (0, 0, 0) on the apron.
    origin = (-168.0, 30.0, 4058.0)
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Pick diffuse textures for every material and pack the downscaled atlas
    # first: the mesh UVs are baked into its tile rectangles.
    atlas_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_atlas.png")
    atlas_info = build_atlas.build(build_atlas.load_map(), atlas_path)
    slot_of = {int(k, 16): v for k, v in atlas_info["slot"].items()}
    print("atlas: %d textures -> %s" % (len(atlas_info["tiles"]), atlas_path))

    chunks = collect_world_chunks(
        ["ARENA_527911.BNDL"],
        origin=origin,
        radius=750.0,
        slot_of=slot_of,
        decorate=make_yard_decorator(origin),
    )
    chunks = weld_chunks(clip_chunks(chunks, 650.0))
    out_mesh = os.path.join(project_root, "romfs", "assets", "fairhaven_block.mwmesh")
    write_mwmesh(chunks, out_mesh)
    tot_v = sum(len(c["verts"]) for c in chunks)
    tot_i = sum(len(c["indices"]) for c in chunks)
    print(f"wrote {out_mesh}: {len(chunks)} chunks, {tot_v} verts, {tot_i // 3} tris, {os.path.getsize(out_mesh)} bytes")
    print(f"GPU vertex memory: {tot_v * VERTEX_SIZE / 1048576.0:.2f} MiB + indices {tot_i * 2 / 1048576.0:.2f} MiB")
    os.makedirs("/tmp/opencode/mw", exist_ok=True)
    render_topdown_preview(chunks, "/tmp/opencode/mw/fairhaven_preview.png", span=620.0)
    print("wrote /tmp/opencode/mw/fairhaven_preview.png")
