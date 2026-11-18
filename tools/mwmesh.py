#!/usr/bin/env python3
"""Decoders for MW2012 (Criterion "Chameleon") renderable resources.

Type-5 resources ("renderable") store:
  stream 0:
    +0   f32 bbox_centre[3]
    +12  f32 bbox_radius
    +16  u16 vertex_decl_id     (12 for most static geometry)
    +18  u16 submesh_count
    +20  u32 stride_hint        (usually 32)
    +24  u32 flags              (0xff00)
    +32  u32 submesh_desc_offset[submesh_count]   (96-byte descriptors)
  submesh descriptor (96 bytes, u32 fields):
    [7]  valid index count (entries, including 0xFFFF restarts)
    [10] offset of index declaration  (relative to stream 0)
    [11] offset of vertex declaration (relative to stream 0)
    [15] index offset  (relative to stream 1)
    [16] index bytes
    [21] vertex offset (relative to stream 1)
    [22] vertex bytes
  index declaration (24 bytes at [10]): [3] offset, [4] size
  vertex declaration (24 bytes at [11]): [3] offset, [4] size

  stream 1: index buffers (u16 triangle strips, 0xFFFF = restart),
            vertex buffers (float3 position first).

Vertex stride is not stored explicitly; it is recovered by testing
candidate strides and scoring the resulting triangles.
"""
import math
import struct

CANDIDATE_STRIDES = (12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72)

# Maximum world-space edge of a triangle we are willing to draw.  Strip
# artefacts that weld two distant vertex clusters into one "sail" exceed it.
MAX_TRI_EDGE = 100.0

# Per-stride byte offset of the two half-float texture coordinates.  The
# attribute layout depends on the material, so it cannot be assumed to sit
# at +16.  Strides without an entry carry no UVs.
UV_OFFSETS = {24: 16, 28: 24, 32: 16, 36: 16, 44: 40, 68: 40}


def parse_renderable(resource, bundle):
    """Return a dict describing a type-5 resource, or None."""
    d0 = bundle.stream0(resource)
    d1 = bundle.stream1(resource)
    if len(d0) < 32:
        return None

    centre = struct.unpack_from("<3f", d0, 0)
    radius = struct.unpack_from("<f", d0, 12)[0]
    decl = struct.unpack_from("<H", d0, 16)[0]
    nsub = struct.unpack_from("<H", d0, 18)[0]
    if nsub == 0 or nsub > 512:
        return None
    if 32 + nsub * 4 > len(d0):
        return None
    table = struct.unpack_from("<%dI" % nsub, d0, 32)

    submeshes = []
    for desc_index, t in enumerate(table):
        if t + 96 > len(d0):
            continue
        w = struct.unpack_from("<24I", d0, t)
        idx_off, idx_sz = w[15], w[16]
        vtx_off, vtx_sz = w[21], w[22]
        if idx_sz == 0 or vtx_sz == 0:
            continue
        if idx_off + idx_sz > len(d1) or vtx_off + vtx_sz > len(d1):
            continue
        # w[7] is the number of valid index entries.  The index buffer is
        # padded with zeroes up to its declared size; decoding those
        # padding entries welds the last strip to distant vertex clusters
        # and produces the multi-hundred-metre "sail" triangles.
        index_count = w[7]
        if index_count == 0 or index_count > idx_sz // 2:
            index_count = idx_sz // 2
        submeshes.append(dict(
            idx_off=idx_off, idx_sz=idx_sz, index_count=index_count,
            vtx_off=vtx_off, vtx_sz=vtx_sz,
            idx_decl=(w[10], w[11]),
            desc_off=t, desc_index=desc_index,
        ))

    if not submeshes:
        return None
    # The submesh->material table follows the last 96-byte descriptor:
    # nsub 16-byte records (u64 material id, u32 offset, u32 reserved).
    trailer = table[-1] + 96
    materials = []
    if trailer + nsub * 16 <= len(d0):
        materials = [struct.unpack_from("<Q", d0, trailer + k * 16)[0]
                     for k in range(nsub)]
    return dict(centre=centre, radius=radius, decl=decl,
                nsub=nsub, subs=submeshes, d0=d0, d1=d1,
                materials=materials)


def read_indices(d1, io, isz, count=None):
    """Read the valid part of a u16 index buffer.

    `count` is the descriptor's valid entry count (field [7]); without it
    the whole declared buffer is read, including its zero padding.
    """
    n = isz // 2
    if count is not None and 0 < count <= n:
        n = count
    return struct.unpack_from("<%dH" % n, d1, io)


def split_strips(indices):
    """Split a u16 index buffer on 0xFFFF restarts into strips."""
    strips, cur = [], []
    for x in indices:
        if x == 0xFFFF:
            if len(cur) >= 3:
                strips.append(cur)
            cur = []
        else:
            cur.append(x)
    if len(cur) >= 3:
        strips.append(cur)
    return strips


def strips_to_triangles(strips):
    tris = []
    for s in strips:
        for i in range(len(s) - 2):
            # alternate winding, keep CCW-ish
            if i & 1:
                tris.append((s[i + 1], s[i], s[i + 2]))
            else:
                tris.append((s[i], s[i + 1], s[i + 2]))
    return tris


def _pos_at(d1, vo, stride, k):
    p = vo + k * stride
    if p + 12 > len(d1):
        return None
    x, y, z = struct.unpack_from("<3f", d1, p)
    return (x, y, z)


def _stride_stats(d1, vo, stride, tris, maxi):
    """Median max-edge length and fraction of absurd triangles."""
    edges = []
    absurd = 0
    step = max(1, len(tris) // 4000)
    for a, b, c in tris[::step]:
        A = _pos_at(d1, vo, stride, a)
        B = _pos_at(d1, vo, stride, b)
        C = _pos_at(d1, vo, stride, c)
        if not (A and B and C):
            absurd += 1
            continue
        if not all(abs(v) < 1e6 for v in A + B + C):
            absurd += 1
            continue
        e = max(
            ((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2 + (A[2] - B[2]) ** 2) ** 0.5,
            ((B[0] - C[0]) ** 2 + (B[1] - C[1]) ** 2 + (B[2] - C[2]) ** 2) ** 0.5,
            ((C[0] - A[0]) ** 2 + (C[1] - A[1]) ** 2 + (C[2] - A[2]) ** 2) ** 0.5,
        )
        edges.append(e)
    if not edges:
        return None
    edges.sort()
    med = edges[len(edges) // 2]
    return med, absurd / max(1, len(tris[::step]))


def _dist2(a, b):
    dx, dy, dz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
    return dx * dx + dy * dy + dz * dz


def _sphere_outlier_fraction(d1, vo, stride, count, centre, radius):
    """Fraction of sampled vertices that fall outside the bounding sphere.

    A correct stride keeps every vertex inside the renderable's declared
    sphere.  Wrong strides smear vertices across kilometres, so this is a
    much sharper test than triangle edge statistics.
    """
    if centre is None or radius <= 0.0:
        return 0.0
    limit2 = (radius * 1.2 + 10.0) ** 2
    cx, cy, cz = centre
    step = max(1, count // 2000)
    out = checked = 0
    for k in range(0, count, step):
        p = _pos_at(d1, vo, stride, k)
        checked += 1
        if p is None:
            out += 1
            continue
        dx, dy, dz = p[0] - cx, p[1] - cy, p[2] - cz
        if dx * dx + dy * dy + dz * dz > limit2:
            out += 1
    return out / max(1, checked)


def guess_stride(d1, vo, vtx_sz, tris, max_index, centre=None, radius=0.0):
    """Pick the vertex stride that yields sane geometry."""
    need = max_index + 1
    best = None
    for s in CANDIDATE_STRIDES:
        n = vtx_sz // s
        pad = vtx_sz - n * s
        if n < need or pad >= 24:
            continue
        outliers = _sphere_outlier_fraction(d1, vo, s, n, centre, radius)
        if outliers > 0.05:
            continue
        stats = _stride_stats(d1, vo, s, tris, max_index)
        if not stats:
            continue
        med, absurd = stats
        if med < 1e-4 or med > 200.0 or absurd > 0.1:
            continue
        # Prefer exact vertex-count match, then the stride that keeps the
        # fewest vertices outside the sphere, then the smallest stride.
        score = (0 if n == need else 1, round(outliers, 3), abs(n - need), s)
        if best is None or score < best[0]:
            best = (score, s, n, med)
    if best is None:
        return None
    return dict(stride=best[1], count=best[2], median_edge=best[3])


def extract_mesh(resource, bundle):
    """Decode a type-5 resource into world-space triangles.

    Returns dict(centre, radius, bbox(min,max), parts=[dict(offset, verts, tris)])
    where `verts` is a list of (x,y,z) and `tris` is a list of index triples
    local to that part.
    """
    info = parse_renderable(resource, bundle)
    if not info:
        return None
    d1 = info["d1"]
    centre = tuple(info["centre"])
    radius = info["radius"]
    limit2 = (radius * 1.2 + 10.0) ** 2

    parts = []
    mn = [1e30] * 3
    mx = [-1e30] * 3

    for sub in info["subs"]:
        indices = read_indices(d1, sub["idx_off"], sub["idx_sz"],
                               sub.get("index_count"))
        real = [x for x in indices if x != 0xFFFF]
        if not real:
            continue
        max_index = max(real)
        strips = split_strips(indices)
        tris = strips_to_triangles(strips)
        if not tris:
            continue
        g = guess_stride(d1, sub["vtx_off"], sub["vtx_sz"], tris, max_index,
                         centre, radius)
        if not g:
            continue
        stride = g["stride"]
        count = g["count"]
        verts = []
        uvs = []
        valid = True
        uv_off = UV_OFFSETS.get(stride)
        if uv_off is not None and uv_off + 4 > stride:
            uv_off = None
        for k in range(count):
            p = _pos_at(d1, sub["vtx_off"], stride, k)
            if p is None or not all(abs(v) < 1e6 for v in p):
                valid = False
                break
            verts.append(p)
            if uv_off is not None:
                u, v = struct.unpack_from("<2e", d1,
                                          sub["vtx_off"] + k * stride + uv_off)
                u, v = float(u), float(v)
                if math.isfinite(u) and math.isfinite(v):
                    uvs.append((u, v))
                else:
                    uvs.append((0.0, 0.0))
            else:
                uvs.append((0.0, 0.0))
        if not valid:
            continue
        # Reject vertices outside the declared sphere and any triangle with
        # an implausible edge.  Both catch strip artefacts that weld two
        # distant vertex clusters into a single "sail" triangle.
        inside = []
        for p in verts:
            dx, dy, dz = p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]
            inside.append(dx * dx + dy * dy + dz * dz <= limit2)
        clean = []
        for a, b, c in tris:
            if max(a, b, c) >= count:
                continue
            if not (inside[a] and inside[b] and inside[c]):
                continue
            A, B, C = verts[a], verts[b], verts[c]
            if max(_dist2(A, B), _dist2(B, C), _dist2(C, A)) > MAX_TRI_EDGE ** 2:
                continue
            clean.append((a, b, c))
        if not clean:
            continue
        for v in verts:
            for a in range(3):
                mn[a] = min(mn[a], v[a])
                mx[a] = max(mx[a], v[a])
        mat = 0
        if info["materials"] and sub["desc_index"] < len(info["materials"]):
            mat = info["materials"][sub["desc_index"]]
        parts.append(dict(verts=verts, uvs=uvs, tris=clean, stride=stride,
                          has_uv=uv_off is not None, material=mat,
                          median_edge=g["median_edge"]))

    if not parts:
        return None
    return dict(centre=info["centre"], radius=info["radius"],
                bbox_min=tuple(mn), bbox_max=tuple(mx), parts=parts)
