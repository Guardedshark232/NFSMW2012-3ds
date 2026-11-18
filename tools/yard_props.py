#!/usr/bin/env python3
"""Procedural dressing for the Fairhaven container yard.

The original game streams the stacked shipping containers and the gantry
cranes as instanced props; ARENA_527911.BNDL only carries the quay, the
concrete apron and a handful of container panels.  This module dresses the
empty apron with container stacks and gantry beams built from geometry we do
have, using the game's own stacked-container photo (atlas material
0x100000000137519) so the yard reads like Callahan Industrial.

Placement is hand-authored: two rows parallel to the pier the car drives
along, plus gantry rails above them.  Every candidate is checked against the
extracted ground height grid, so stacks never float over water or on a slope.
"""
import math

# Material whose albedo is the stacked-container photo (tools/world_atlas.png
# slot (6,6) in the current build).
CONTAINER_MAT = 0x100000000137519
# Bluish-grey ribbed steel (atlas tile (4,7)) for the gantry structure.
STEEL_MAT = 0x1000000000094c02

CONTAINER_LEN = 12.0
CONTAINER_DEPTH = 2.5
ROW_OFFSETS = (-24.0, 12.0)
S_MIN, S_MAX = -20.0, 150.0
S_STEP = 12.8

# Yard centre / pier axis (mesh-local coordinates, the same space the car and
# renderer use).  Direction is the heading the chase camera sees at the yard.
CENTER = (-176.0, -139.0)
AXIS = (0.65, -0.76)
SIDE = (0.76, 0.65)

GRID_CELL = 3.0
VERBOSE = False


def _tri_normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    return (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)


def height_grid(parts, cell=GRID_CELL, y_min=-14.5, y_max=-8.5):
    """Ground height per XZ cell from near-horizontal triangles.

    Only triangles inside the yard deck height band are considered so roofs
    and elevated roads above the apron cannot hide the drivable surface.
    Triangles are rasterised over their whole XZ footprint: the apron is made
    of very large flat triangles, so sampling only their centroids would leave
    most cells empty and reject valid stack positions.
    """
    grid = {}
    for p in parts:
        vs, tris = p["verts"], p["tris"]
        for ia, ib, ic in tris:
            a, b, c = vs[ia], vs[ib], vs[ic]
            nx, ny, nz = _tri_normal(a, b, c)
            n = math.sqrt(nx * nx + ny * ny + nz * nz)
            if n == 0.0 or abs(ny) < 0.62 * n:
                continue
            cy = (a[1] + b[1] + c[1]) / 3.0
            if cy < y_min or cy > y_max:
                continue
            x0 = int(math.floor(min(a[0], b[0], c[0]) / cell))
            x1 = int(math.floor(max(a[0], b[0], c[0]) / cell))
            z0 = int(math.floor(min(a[2], b[2], c[2]) / cell))
            z1 = int(math.floor(max(a[2], b[2], c[2]) / cell))
            if (x1 - x0) * (z1 - z0) > 40000:
                continue
            for gx in range(x0, x1 + 1):
                for gz in range(z0, z1 + 1):
                    key = (gx, gz)
                    if cy > grid.get(key, -1e9):
                        grid[key] = cy
    return grid


def sample(grid, x, z, cell=GRID_CELL):
    """Deck height at a point: median of the valid 3x3 cell neighbourhood.

    Median rather than max, so an isolated roof or ledge cell picked up by
    the rasteriser cannot lift a whole footprint off the apron.
    """
    key = (int(math.floor(x / cell)), int(math.floor(z / cell)))
    vals = []
    for dx in (-1, 0, 1):
        for dz in (-1, 0, 1):
            v = grid.get((key[0] + dx, key[1] + dz))
            if v is not None:
                vals.append(v)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def _flat_ground(grid, x, z, half_l, half_d):
    """Lowest ground height under a footprint, or None if unusable.

    The quay apron slopes gently toward the water, so containers sit at the
    lowest sampled corner: embedding a box slightly is much less noticeable
    than letting it float above a down-slope.
    """
    y0 = sample(grid, x, z)
    if y0 is None:
        return None
    lo = hi = y0
    for lx in (-half_l, 0.0, half_l):
        for lz in (-half_d, 0.0, half_d):
            y = sample(grid, x + lx, z + lz)
            if y is None:
                return None
            lo = min(lo, y)
            hi = max(hi, y)
    if hi - lo > 2.2:
        return None
    return lo


def _quad(verts, uvs, tris, corners, uv_rect):
    """Append a quad (4 corners, CCW) with a source-UV rectangle."""
    base = len(verts)
    u0, v0, u1, v1 = uv_rect
    verts.extend(corners)
    uvs.extend(((u0, v0), (u1, v0), (u1, v1), (u0, v1)))
    tris.extend(((base, base + 1, base + 2), (base, base + 2, base + 3)))


def _box(parts, x, y, z, axis, side, length, depth, height, uv_rect, tint,
         top_tint=(0.42, 0.43, 0.46)):
    """One container block: four photo-mapped sides plus a dark roof."""
    a = (axis[0], axis[1])
    s = (side[0], side[1])
    hl, hd = length * 0.5, depth * 0.5
    corners = []
    for du, dv in ((-hl, -hd), (hl, -hd), (hl, hd), (-hl, hd)):
        corners.append((x + a[0] * du + s[0] * dv,
                        z + a[1] * du + s[1] * dv))

    verts, uvs, tris = [], [], []
    y_top = y + height
    # Long sides (+/- depth), photo mapped.
    for sign in (1.0, -1.0):
        c0 = (corners[0][0] + s[0] * depth * sign, corners[0][1] + s[1] * depth * sign)
        c1 = (corners[1][0] + s[0] * depth * sign, corners[1][1] + s[1] * depth * sign)
        if sign > 0:
            quad = ((c0[0], y, c0[1]), (c1[0], y, c1[1]),
                    (c1[0], y_top, c1[1]), (c0[0], y_top, c0[1]))
        else:
            quad = ((c1[0], y, c1[1]), (c0[0], y, c0[1]),
                    (c0[0], y_top, c0[1]), (c1[0], y_top, c1[1]))
        _quad(verts, uvs, tris, quad, uv_rect)
    # Short ends, darker crop of the same photo.
    e0 = (corners[0][0] + s[0] * 0.0, corners[0][1] + s[1] * 0.0)
    e1 = (corners[3][0], corners[3][1])
    _quad(verts, uvs, tris,
          ((e0[0], y, e0[1]), (e1[0], y, e1[1]),
           (e1[0], y_top, e1[1]), (e0[0], y_top, e0[1])),
          (uv_rect[0], uv_rect[1], uv_rect[0] + 0.18, uv_rect[3]))
    parts.append(dict(verts=verts, uvs=uvs, tris=tris, material=CONTAINER_MAT,
                      has_uv=True, color=tint))

    # Roof: flat dark steel to hide the photo's sky gaps.
    verts, uvs, tris = [], [], []
    quad = ((corners[0][0], y_top, corners[0][1]),
            (corners[1][0], y_top, corners[1][1]),
            (corners[2][0], y_top, corners[2][1]),
            (corners[3][0], y_top, corners[3][1]))
    _quad(verts, uvs, tris, quad, (0.5, 0.5, 0.5, 0.5))
    parts.append(dict(verts=verts, uvs=uvs, tris=tris, material=None,
                      has_uv=True, color=top_tint))


def _beam(parts, x0, z0, x1, z1, y, thickness=0.9, tint=(1.0, 1.0, 1.0)):
    """Square steel beam between two XZ points at height y.

    Split into short spans: the world extractor drops triangles with an edge
    longer than 100 m (a guard against decoder "sails").
    """
    dx, dz = x1 - x0, z1 - z0
    l = math.hypot(dx, dz)
    if l < 0.5:
        return
    ax, az = dx / l, dz / l
    sx, sz = -az * thickness * 0.5, ax * thickness * 0.5
    h = thickness * 0.5
    verts, uvs, tris = [], [], []
    spans = max(1, int(math.ceil(l / 40.0)))
    for i in range(spans):
        t0 = l * i / spans
        t1 = l * (i + 1) / spans
        ax0, az0 = x0 + ax * t0, z0 + az * t0
        ax1, az1 = x0 + ax * t1, z0 + az * t1
        corners = ((ax0 + sx, az0 + sz), (ax1 + sx, az1 + sz),
                   (ax1 - sx, az1 - sz), (ax0 - sx, az0 - sz))
        for (cx0, cz0), (cx1, cz1) in ((corners[0], corners[1]),
                                       (corners[1], corners[2]),
                                       (corners[2], corners[3]),
                                       (corners[3], corners[0])):
            _quad(verts, uvs, tris,
                  ((cx0, y - h, cz0), (cx1, y - h, cz1),
                   (cx1, y + h, cz1), (cx0, y + h, cz0)),
                  (0.5, 0.5, 0.5, 0.5))
    parts.append(dict(verts=verts, uvs=uvs, tris=tris, material=STEEL_MAT,
                      has_uv=True, color=tint))


def _column(parts, x, z, y0, y1, size=1.1, tint=(1.0, 1.0, 1.0)):
    s = size * 0.5
    verts, uvs, tris = [], [], []
    for (cx0, cz0), (cx1, cz1) in (((x - s, z - s), (x + s, z - s)),
                                   ((x + s, z - s), (x + s, z + s)),
                                   ((x + s, z + s), (x - s, z + s)),
                                   ((x - s, z + s), (x - s, z - s))):
        _quad(verts, uvs, tris,
              ((cx0, y0, cz0), (cx1, y0, cz1),
               (cx1, y1, cz1), (cx0, y1, cz0)),
              (0.5, 0.5, 0.5, 0.5))
    parts.append(dict(verts=verts, uvs=uvs, tris=tris, material=STEEL_MAT,
                      has_uv=True, color=tint))


def _uv_window(index, height):
    """Different slice of the stacked-container photo per stack."""
    u0 = (index * 0.23) % 0.62
    v1 = 0.97 if height > 6.0 else 0.72
    return (u0, 0.02, u0 + 0.34, v1)


def build_parts(parts):
    """Return extra world-space parts dressing the container yard."""
    grid = height_grid(parts)
    extra = []
    stacks = 0

    rows = []
    for off in ROW_OFFSETS:
        px = CENTER[0] + SIDE[0] * off
        pz = CENTER[1] + SIDE[1] * off
        rows.append((px, pz))

    for row_i, (rx, rz) in enumerate(rows):
        s = S_MIN
        stack_i = 0
        while s <= S_MAX:
            x = rx + AXIS[0] * s
            z = rz + AXIS[1] * s
            y = _flat_ground(grid, x, z, CONTAINER_LEN * 0.4,
                             CONTAINER_DEPTH * 0.6)
            # The yard deck sits around y = -11.5; anything higher is a roof
            # or an elevated road, anything lower is water.
            if y is None or y < -14.5 or y > -8.5:
                if VERBOSE:
                    print("  row %d s %.0f skip (ground %s)" % (row_i, s, y))
                s += S_STEP
                continue
            if VERBOSE:
                print("  row %d s %.0f at (%.1f, %.1f) y %.1f"
                      % (row_i, s, x, z, y))
            height = 7.8 if (stack_i % 3) else 5.2
            if stack_i % 7 == 6:
                s += S_STEP
                stack_i += 1
                continue
            tint_v = 0.94 + 0.04 * ((stack_i + row_i) % 4)
            _box(extra, x, y, z, AXIS, SIDE,
                 CONTAINER_LEN, CONTAINER_DEPTH, height,
                 _uv_window(stack_i + row_i * 3, height),
                 (1.18 * tint_v, 1.15 * tint_v, 1.10 * tint_v))
            stacks += 1
            s += S_STEP
            stack_i += 1

        # Gantry rails above this row plus a leg every few stacks.
        gx0 = rx + AXIS[0] * S_MIN
        gz0 = rz + AXIS[1] * S_MIN
        gx1 = rx + AXIS[0] * S_MAX
        gz1 = rz + AXIS[1] * S_MAX
        beam_y = None
        for s2 in range(int(S_MIN), int(S_MAX) + 1, 40):
            x = rx + AXIS[0] * s2
            z = rz + AXIS[1] * s2
            y = sample(grid, x, z)
            if y is None or y < -14.5 or y > -8.5:
                continue
            if beam_y is None:
                beam_y = y + 17.0
            _column(extra, x, z, y, beam_y)
        if beam_y is not None:
            _beam(extra, gx0, gz0, gx1, gz1, beam_y)

    print("yard: %d container stacks, %d parts" % (stacks, len(extra)))
    return extra


if __name__ == "__main__":
    # Quick placement preview against the current mesh (no extraction run).
    import struct
    import os
    VERBOSE = True
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(os.path.dirname(here), "romfs", "assets",
                        "fairhaven_block.mwmesh")
    data = open(path, "rb").read()
    magic, ver, n, _ = struct.unpack_from("<4sIII", data, 0)
    off = 16
    all_parts = []
    for _ in range(n):
        nv, ni, *rest = struct.unpack_from("<II6f2I", data, off)
        off += 40
        verts = []
        for k in range(nv):
            x, y, z, r, g, b, a, u, v = struct.unpack_from("<3f4B2h", data, off)
            off += 20
            verts.append((x, y, z))
        idx = struct.unpack_from("<%dH" % ni, data, off)
        off += ni * 2
        if ni & 1:
            off += 2
        tris = [tuple(idx[i:i + 3]) for i in range(0, ni, 3)]
        all_parts.append(dict(verts=verts, tris=tris))
    build_parts(all_parts)
