#!/usr/bin/env python3
"""Bake streamed-world collision data (.mwcol) from the runtime .mwmesh sectors.

The runtime used to rasterize ground and wall cells from the mesh on the 3DS,
one chunk at a time.  That left the car with partial collision for seconds
after a sector appeared and made the result depend on frame timing.  Baking the
same information on the host is cheap, removes thousands of per-frame triangle
tests from the console, and lets the runtime use a better representation:

  * ground: 2 m height grid storing the lowest and highest walkable surface
    per cell (bridges/overpasses give two values)
  * walls: exact 2D segments with a vertical range, bucketed into an 8 m
    spatial hash so a car query only visits a handful of them

Each .mwcol sits next to its .mwmesh in romfs/assets/sectors/.  The format is
shared with src/renderer/world_mesh.cpp (see the constants below).
"""
import glob
import math
import os
import struct
import sys
import time

import numpy as np

GROUND_DIM = 250          # 250 * 2 m = 500 m, exactly one sector
GROUND_CELL = 2.0
WALL_DIM = 96             # 96 * 8 m = 768 m, covers overhanging geometry
WALL_CELL = 8.0
WALL_ORIGIN = -128.0
GROUND_NONE = 32767       # int16 max: no floor surface
GROUND_NONE_HI = -32768   # int16 min: no ceiling surface

MWCL_MAGIC = b"MWCL"
MWCL_VERSION = 2
HEADER_FORMAT = "<4sIiiIfffIIfff3I"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
SEG_FORMAT = "<4f2h"
SEG_SIZE = struct.calcsize(SEG_FORMAT)
assert HEADER_SIZE == 64, HEADER_SIZE
assert SEG_SIZE == 20, SEG_SIZE

MIN_GROUND_NY = 0.45       # |normal.y| / |normal|, matches the old runtime
MIN_WALL_NY = 0.45         # steeper than this is a wall, not drivable ground
MIN_WALL_YSPAN = 1.2       # ignore kerbs and other low steps
MIN_WALL_HLEN = 0.5        # ignore posts/thin trim
MAX_WALL_EDGE_DY = 0.35    # only outline edges, not face diagonals
MIN_GROUND_Y = -24.0       # road/terrace band; rejects seabed and sky shells
MAX_GROUND_Y = 140.0
MAX_TRI_EDGE = 100.0       # decoder-artifact guard, matches the extractor

SECTOR_SIZE = 500.0


def read_mwmesh(path):
    """Read a .mwmesh v3 file into one vertex array and one triangle array."""
    with open(path, "rb") as f:
        data = f.read()
    magic, version, nchunks, _ = struct.unpack_from("<4sIII", data, 0)
    if magic != b"MWMS" or version != 3:
        raise ValueError(f"{path}: not a v3 .mwmesh")
    off = 16
    verts = []
    tris = []
    base = 0
    for _ in range(nchunks):
        nv, ni, *rest = struct.unpack_from("<II6f2I", data, off)
        off += 40
        v = np.frombuffer(data, dtype=np.dtype([("p", "<3f4"), ("c", "4u1"), ("uv", "<2i2")]),
                          count=nv, offset=off)["p"].copy()
        off += nv * 20
        ix = np.frombuffer(data, dtype="<u2", count=ni, offset=off).reshape(-1, 3).astype(np.int32) + base
        off += ni * 2
        if ni & 1:
            off += 2
        verts.append(v)
        tris.append(ix)
        base += nv
    return np.vstack(verts), np.vstack(tris)


class GroundGrids:
    """All sector height grids, keyed by (gx, gz)."""

    def __init__(self):
        self.grids = {}

    def get(self, gx, gz):
        key = (gx, gz)
        grid = self.grids.get(key)
        if grid is None:
            lo = np.full(GROUND_DIM * GROUND_DIM, GROUND_NONE, dtype=np.int16)
            hi = np.full(GROUND_DIM * GROUND_DIM, GROUND_NONE_HI, dtype=np.int16)
            grid = (lo, hi)
            self.grids[key] = grid
        return grid

    def raster(self, a, b, c):
        """Rasterize one near-horizontal triangle into the sector grids it covers."""
        ax, ay, az = float(a[0]), float(a[1]), float(a[2])
        bx, by, bz = float(b[0]), float(b[1]), float(b[2])
        cx, cy, cz = float(c[0]), float(c[1]), float(c[2])

        # Same barycentric formulation as the old runtime rasterizer.
        d = (bz - cz) * (ax - cx) + (cx - bx) * (az - cz)
        if abs(d) < 1e-9:
            return

        minx = min(ax, bx, cx)
        maxx = max(ax, bx, cx)
        minz = min(az, bz, cz)
        maxz = max(az, bz, cz)

        ix0 = int(math.floor(minx / GROUND_CELL))
        ix1 = int(math.floor(maxx / GROUND_CELL))
        iz0 = int(math.floor(minz / GROUND_CELL))
        iz1 = int(math.floor(maxz / GROUND_CELL))

        # Fast path: the whole triangle is inside one 500 m sector.
        gx0 = int(math.floor(minx / SECTOR_SIZE))
        gx1 = int(math.floor(maxx / SECTOR_SIZE))
        gz0 = int(math.floor(minz / SECTOR_SIZE))
        gz1 = int(math.floor(maxz / SECTOR_SIZE))
        if gx0 == gx1 and gz0 == gz1:
            lo, hi = self.get(gx0, gz0)
            ox = gx0 * SECTOR_SIZE
            oz = gz0 * SECTOR_SIZE
            self._cells(lo, hi, ox, oz, ax, ay, az, bx, by, bz, cx, cy, cz, d,
                        ix0 - int(ox / GROUND_CELL), ix1 - int(ox / GROUND_CELL),
                        iz0 - int(oz / GROUND_CELL), iz1 - int(oz / GROUND_CELL))
            return

        # Boundary-crossing triangle: test cells in world space and update
        # every sector grid that covers the cell centre.
        for iz in range(iz0, iz1 + 1):
            pz = (iz + 0.5) * GROUND_CELL
            for ix in range(ix0, ix1 + 1):
                px = (ix + 0.5) * GROUND_CELL
                l1 = ((bz - cz) * (px - cx) + (cx - bx) * (pz - cz)) / d
                l2 = ((cz - az) * (px - cx) + (ax - cx) * (pz - cz)) / d
                l3 = 1.0 - l1 - l2
                if l1 < -0.001 or l2 < -0.001 or l3 < -0.001:
                    continue
                y = l1 * ay + l2 * by + l3 * cy
                h = max(-32767, min(32767, int(round(y * 100.0))))
                gx = int(math.floor(px / SECTOR_SIZE))
                gz = int(math.floor(pz / SECTOR_SIZE))
                lx = int(math.floor((px - gx * SECTOR_SIZE) / GROUND_CELL))
                lz = int(math.floor((pz - gz * SECTOR_SIZE) / GROUND_CELL))
                lo, hi = self.get(gx, gz)
                if 0 <= lx < GROUND_DIM and 0 <= lz < GROUND_DIM:
                    idx = lz * GROUND_DIM + lx
                    if h < lo[idx]:
                        lo[idx] = h
                    if h > hi[idx]:
                        hi[idx] = h

    @staticmethod
    def _cells(lo, hi, ox, oz, ax, ay, az, bx, by, bz, cx, cy, cz, d,
               ix0, ix1, iz0, iz1):
        ix0 = max(0, ix0)
        iz0 = max(0, iz0)
        ix1 = min(GROUND_DIM - 1, ix1)
        iz1 = min(GROUND_DIM - 1, iz1)
        for iz in range(iz0, iz1 + 1):
            pz = oz + (iz + 0.5) * GROUND_CELL
            for ix in range(ix0, ix1 + 1):
                px = ox + (ix + 0.5) * GROUND_CELL
                l1 = ((bz - cz) * (px - cx) + (cx - bx) * (pz - cz)) / d
                l2 = ((cz - az) * (px - cx) + (ax - cx) * (pz - cz)) / d
                l3 = 1.0 - l1 - l2
                if l1 < -0.001 or l2 < -0.001 or l3 < -0.001:
                    continue
                y = l1 * ay + l2 * by + l3 * cy
                h = max(-32767, min(32767, int(round(y * 100.0))))
                idx = iz * GROUND_DIM + ix
                if h < lo[idx]:
                    lo[idx] = h
                if h > hi[idx]:
                    hi[idx] = h


def build_ground(verts, tris, grids):
    a = verts[tris[:, 0]]
    b = verts[tris[:, 1]]
    c = verts[tris[:, 2]]
    n = np.cross(b - a, c - a)
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-5
    a, b, c, n, ln = a[ok], b[ok], c[ok], n[ok], ln[ok]
    ny = np.abs(n[:, 1]) / ln
    cy = (a[:, 1] + b[:, 1] + c[:, 1]) / 3.0

    edge = np.maximum(np.linalg.norm(b - a, axis=1),
                      np.maximum(np.linalg.norm(c - b, axis=1),
                                 np.linalg.norm(a - c, axis=1)))
    keep = (ny >= MIN_GROUND_NY) & (cy >= MIN_GROUND_Y) & (cy <= MAX_GROUND_Y) & (edge <= MAX_TRI_EDGE)
    idxs = np.where(keep)[0]
    for i in idxs:
        grids.raster(a[i], b[i], c[i])
    return len(idxs)


def build_walls(verts, tris):
    """Return unique wall segments as (x0, z0, x1, z1, y0, y1)."""
    a = verts[tris[:, 0]]
    b = verts[tris[:, 1]]
    c = verts[tris[:, 2]]
    n = np.cross(b - a, c - a)
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-5
    a, b, c, n, ln = a[ok], b[ok], c[ok], n[ok], ln[ok]
    ny = np.abs(n[:, 1]) / ln
    cy = (a[:, 1] + b[:, 1] + c[:, 1]) / 3.0
    ymin = np.minimum(a[:, 1], np.minimum(b[:, 1], c[:, 1]))
    ymax = np.maximum(a[:, 1], np.maximum(b[:, 1], c[:, 1]))
    yspan = ymax - ymin
    edge = np.maximum(np.linalg.norm(b - a, axis=1),
                      np.maximum(np.linalg.norm(c - b, axis=1),
                                 np.linalg.norm(a - c, axis=1)))
    steep = ((ny < MIN_WALL_NY) & (cy >= MIN_GROUND_Y) & (cy <= MAX_GROUND_Y) &
             (yspan >= MIN_WALL_YSPAN) & (edge <= MAX_TRI_EDGE))

    segs = {}
    q = 4.0  # 0.25 m dedupe grid

    # A vertical wall quad is triangulated into two triangles; its vertical
    # edges collapse to points in XZ and its diagonals cut across the face.
    # Emitting every edge therefore leaves the wall "fenceless" except for a
    # useless diagonal.  Keep only the horizontal (constant-Y) outline edges:
    # the bottom and top edge of a wall have identical XZ endpoints, so they
    # dedupe into one segment carrying the full vertical range.
    for p, r in ((a, b), (b, c), (c, a)):
        hlen = np.hypot(r[:, 0] - p[:, 0], r[:, 2] - p[:, 2])
        dy = np.abs(r[:, 1] - p[:, 1])
        mask = steep & (hlen >= MIN_WALL_HLEN) & (dy <= MAX_WALL_EDGE_DY)
        for i in np.where(mask)[0]:
            x0, z0 = p[i, 0], p[i, 2]
            x1, z1 = r[i, 0], r[i, 2]
            k0 = (int(round(x0 * q)), int(round(z0 * q)))
            k1 = (int(round(x1 * q)), int(round(z1 * q)))
            if k0 == k1:
                continue
            if k0 > k1:
                k0, k1 = k1, k0
            key = (k0, k1)
            y0 = float(ymin[i])
            y1 = float(ymax[i])
            old = segs.get(key)
            if old is None:
                segs[key] = [x0, z0, x1, z1, y0, y1]
            else:
                old[4] = min(old[4], y0)
                old[5] = max(old[5], y1)

    return [(s[4], s[5], s[0], s[1], s[2], s[3]) for s in segs.values()]


def build_index(segments, gx, gz):
    """Build a cell -> segment-reference table for the 8 m wall hash.

    A single ``next`` link per segment cannot represent membership in several
    cells (long walls cross many), so the index is a compressed adjacency:
    ``cell_start[c]..cell_start[c+1]`` indexes into a flat array of segment
    ids.  Runtime traversal is a simple contiguous loop per cell.

    Segment coordinates stay in mesh-local world space for the runtime's
    closest-point math, but the hash cells are sector-local (0 = sector
    origin) with an extra margin for geometry overhanging the 500 m cell.
    """
    ox, oz = gx * SECTOR_SIZE, gz * SECTOR_SIZE
    buckets = {}

    for si, (_y0, _y1, x0, z0, x1, z1) in enumerate(segments):
        lx0 = (x0 - ox - WALL_ORIGIN) / WALL_CELL
        lz0 = (z0 - oz - WALL_ORIGIN) / WALL_CELL
        lx1 = (x1 - ox - WALL_ORIGIN) / WALL_CELL
        lz1 = (z1 - oz - WALL_ORIGIN) / WALL_CELL
        steps = max(1, int(math.ceil(max(abs(lx1 - lx0), abs(lz1 - lz0)) / 0.5)))
        touched = set()
        for s in range(steps + 1):
            t = s / steps
            cx = int(math.floor(lx0 + (lx1 - lx0) * t))
            cz = int(math.floor(lz0 + (lz1 - lz0) * t))
            if 0 <= cx < WALL_DIM and 0 <= cz < WALL_DIM:
                touched.add(cz * WALL_DIM + cx)
        for cell in touched:
            buckets.setdefault(cell, []).append(si)

    cell_start = np.zeros(WALL_DIM * WALL_DIM + 1, dtype=np.int32)
    refs = []
    for cell in range(WALL_DIM * WALL_DIM):
        refs.extend(buckets.get(cell, ()))
        cell_start[cell + 1] = len(refs)
    return cell_start, np.asarray(refs, dtype=np.uint32)


def write_collider(path, gx, gz, grids, segments):
    lo, hi = grids.get(gx, gz)
    cell_start, refs = build_index(segments, gx, gz)

    with open(path, "wb") as f:
        f.write(struct.pack(HEADER_FORMAT,
                            MWCL_MAGIC, MWCL_VERSION, gx, gz,
                            GROUND_DIM, GROUND_CELL,
                            gx * SECTOR_SIZE, gz * SECTOR_SIZE,
                            len(segments),
                            WALL_DIM, WALL_CELL, WALL_ORIGIN, WALL_ORIGIN,
                            len(refs), 0, 0))
        f.write(lo.astype("<i2").tobytes())
        f.write(hi.astype("<i2").tobytes())
        f.write(cell_start.astype("<i4").tobytes())
        f.write(refs.astype("<u4").tobytes())
        for y0, y1, x0, z0, x1, z1 in segments:
            f.write(struct.pack(SEG_FORMAT, x0, z0, x1, z1,
                                max(-32767, min(32767, int(round(y0 * 100.0)))),
                                max(-32767, min(32767, int(round(y1 * 100.0))))))


def read_manifest(root):
    path = os.path.join(root, "romfs", "assets", "world_manifest.bin")
    with open(path, "rb") as f:
        data = f.read()
    magic, version, count, sector_size, ox, oy, oz = struct.unpack_from("<4sIIf3f", data, 0)
    if magic != b"MWST" or version != 1:
        raise ValueError("world_manifest.bin is not MWST v1")
    records = []
    off = 28
    for _ in range(count):
        gx, gz, b0, b1, b2, b3, b4, b5, nch, nv, ni, fn = struct.unpack_from("<ii6f3I32s", data, off)
        off += 76
        name = fn.split(b"\x00", 1)[0].decode("ascii")
        records.append((gx, gz, name))
    return records


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sector_dir = os.path.join(root, "romfs", "assets", "sectors")
    records = read_manifest(root)
    only = None
    if len(sys.argv) >= 4 and sys.argv[1] == "--only":
        only = (int(sys.argv[2]), int(sys.argv[3]))
        records = [r for r in records if (r[0], r[1]) == only]

    grids = GroundGrids()
    total_segments = 0
    total_ground = 0
    total_bytes = 0
    t0 = time.time()

    for n, (gx, gz, name) in enumerate(records):
        path = os.path.join(sector_dir, name)
        verts, tris = read_mwmesh(path)
        total_ground += build_ground(verts, tris, grids)
        segments = build_walls(verts, tris)
        out = os.path.join(sector_dir, name.replace(".mwmesh", ".mwcol"))
        # Only sectors that actually own walls or ground need a file, but
        # keeping one per manifest sector makes runtime loading predictable.
        write_collider(out, gx, gz, grids, segments)
        total_segments += len(segments)
        total_bytes += os.path.getsize(out)
        if (n + 1) % 10 == 0 or (n + 1) == len(records):
            print(f"  [{n+1:2d}/{len(records):2d}] ({gx:3d},{gz:3d}) {name:18s} "
                  f"ground={total_ground:7d} wall={len(segments):6d} "
                  f"col={os.path.getsize(out)/1024:6.0f} KB [{time.time()-t0:.1f}s]")

    # Remove colliders whose sector no longer exists.
    referenced = {r[2].replace(".mwmesh", ".mwcol") for r in read_manifest(root)}
    for path in glob.glob(os.path.join(sector_dir, "*.mwcol")):
        if os.path.basename(path) not in referenced:
            os.remove(path)

    print(f"\nWrote {len(records)} colliders: {total_segments} wall segments, "
          f"{total_bytes/(1024*1024):.2f} MB, {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
