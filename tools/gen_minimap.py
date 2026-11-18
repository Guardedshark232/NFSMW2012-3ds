#!/usr/bin/env python3
"""Render a top-down Fairhaven minimap from the converted world mesh.

The HUD draws a 240 m window of this map centred on the car, so the image
covers a slightly larger area than the drivable radius.  Roads (near-
horizontal surfaces between the waterline and roof level) are painted mid
grey; roofs and walls are painted light grey-blue; everything without
geometry stays deep sea blue.  The PNG is converted to .t3x by tex3ds in the
Makefile.
"""
import math
import struct
import sys

from PIL import Image, ImageDraw

MIN = -720.0          # metres, west/south edge of the map
MAX = 720.0
RES = 288             # texels (5 m per texel)
SCALE = RES / (MAX - MIN)

ROAD = (58, 70, 86)
BUILDING = (150, 164, 184)
WATER = (14, 26, 36)


def load(path):
    data = open(path, "rb").read()
    magic, version, nch, _ = struct.unpack_from("<4sIII", data, 0)
    assert magic == b"MWMS" and version == 3
    off = 16
    for _ in range(nch):
        nv, ni = struct.unpack_from("<II", data, off)
        vstart = off + 40
        verts = [struct.unpack_from("<3f", data, vstart + k * 20) for k in range(nv)]
        istart = vstart + nv * 20
        idx = struct.unpack_from("<%dH" % ni, data, istart)
        yield verts, idx
        off = istart + ni * 2 + (2 if ni & 1 else 0)


def main(mesh_path, out_png):
    road_mask = Image.new("L", (RES, RES), 0)
    bld_mask = Image.new("L", (RES, RES), 0)
    dr = ImageDraw.Draw(road_mask)
    db = ImageDraw.Draw(bld_mask)

    def px(p):
        return ((p[0] - MIN) * SCALE, (p[2] - MIN) * SCALE)

    for verts, idx in load(mesh_path):
        for j in range(0, len(idx) - 2, 3):
            a = verts[idx[j]]
            b = verts[idx[j + 1]]
            c = verts[idx[j + 2]]
            cy = (a[1] + b[1] + c[1]) / 3.0
            if cy <= -17.5:
                continue  # beneath the waterline: not part of the city
            # Projected normal, y component ratio
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            ln = math.sqrt(nx * nx + ny * ny + nz * nz)
            if ln < 1e-6:
                continue
            horiz = abs(ny) / ln
            poly = [px(a), px(b), px(c)]
            if horiz >= 0.5 and cy <= -4.5:
                dr.polygon(poly, fill=255)
            else:
                db.polygon(poly, fill=255)

    im = Image.new("RGB", (RES, RES), WATER)
    im.paste(Image.new("RGB", (RES, RES), ROAD), (0, 0), road_mask)
    # Buildings and roofs last: lit blocks over dark streets read like a
    # conventional city map at radar size.
    im.paste(Image.new("RGB", (RES, RES), BUILDING), (0, 0), bld_mask)
    im.save(out_png)
    print(f"wrote {out_png} ({RES}x{RES}, {MAX - MIN:.0f} m across)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "romfs/assets/fairhaven_block.mwmesh",
         sys.argv[2] if len(sys.argv) > 2 else "/tmp/opencode/mw/minimap.png")
