#!/usr/bin/env python3
"""Extract the player Porsche 911 from the retail vehicle bundles.

The PC game stores vehicles as type-5 renderables inside
``VEH_<id>_LO.BNDL`` / ``VEH_<id>_HI.BNDL``.  Vehicle geometry uses packed
int16 positions (``xyz`` at +0, scale 1/8192) instead of the float3 layout
the world extractor expects, and the LO body LOD at resource 53 carries the
whole shell at a third of the HI triangle count.

The extractor writes two runtime assets:

* ``tools/car_atlas.png`` - a 512x256 RGBA4444 source atlas containing the
  paint, plate, badge, carbon and interior maps.  ``tex3ds`` packs it into
  ``romfs/assets/car.t3x``.
* ``romfs/assets/car.mwcar`` - the body and one centred wheel block in
  metres, using the same 20-byte vertex layout as the world:
  ``float3 position, u8x4 tint, s16x2 atlas UV``.  The runtime instances the
  wheel four times so it can steer and spin.

Wheel placement transforms are not present in the LO bundle (only type-81
wrappers reference the wheel mesh), so the track and wheelbase below come
from measuring the body bounds.  The front of the source model is +Z.
"""
import math
import os
import struct
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bndl import Bundle
from mwmesh import parse_renderable, read_indices, split_strips, strips_to_triangles
import mwtex
from game_paths import MW_ROOT

BODY_BUNDLE = "VEH_1085007_LO.BNDL"
TEXTURE_BUNDLES = ("VEH_1085007_LO.BNDL", "VEH_1085007_HI.BNDL", "VEHICLETEX.BNDL")

BODY_RES = 53          # LO shell: 6,522 triangles after dropping the bad strip
WHEEL_RES = 8          # complete blue-rim wheel, 428 triangles
SKIP_BODY_SUBS = {8}   # stride-45 strip with 7 m bounds: decoder artefact

MODEL_SCALE = 2.5      # source model units -> metres (911 is 4.49 m long)
WHEEL_SCALE = 1.25     # source rim reads small; tyres need to fill the arches
WHEEL_X = 0.270        # model units, half track (0.675 m, tucked inside arches)
WHEEL_Z = 0.500        # model units, half wheelbase
WHEEL_Y = 0.1354       # model units, wheel centre / radius

ATLAS_W, ATLAS_H = 512, 256
INSET = 2              # texels, keeps bilinear filtering inside each tile
VERTEX_SIZE = 20

# tile -> (x, y, w, h, source texture id).  Texture ids are the low 32 bits;
# vehicle-local resources carry the 0x1_0000_0000 high word.  The body paint
# material references only a normal map and a dirt mask: the game colours it
# from shader constants, so the paint shells sample the flat white tile.
TILES = {
    "engine":    (0,   0,   256, 256, 0x0426d6),
    "badge":     (256, 0,   128, 128, 0x102725),
    "carbon":    (384, 0,   128, 128, 0x11077f),
    "plate":     (256, 128, 128, 64,  0x0d8cd4),
    "trim":      (384, 128, 64,  64,  0x102736),
    "interior":  (448, 128, 64,  64,  0x10272f),
    "lettering": (256, 192, 128, 64,  0x138805),
    "dash":      (384, 192, 64,  64,  0x10272b),
    "white":     (448, 192, 64,  64,  None),
}

# submesh -> tile.  Submeshes 1/4/7/13 share the paint material family; the
# material constants pick body paint, glass and black trim, which we bake as
# vertex tints instead.
TILE_BY_SUB = {
    0: "engine", 1: "white", 4: "white", 7: "white", 13: "white",
    2: "badge", 3: "carbon", 5: "white", 6: "trim",
    9: "white", 10: "plate", 11: "lettering", 12: "interior", 14: "dash",
}
PAINT = (0.105, 0.275, 0.62)      # Guards-blue metallic
GLASS = (0.035, 0.045, 0.060)
TRIM = (0.085, 0.085, 0.090)
TINT_BY_SUB = {
    1: PAINT, 13: PAINT,          # main shell, front bumper cover
    4: GLASS,                     # windows and lamp covers
    7: TRIM,                      # sills / arch liners
    5: (0.55, 0.48, 0.38),        # small interior trim strip
    9: (0.90, 0.12, 0.08),        # rear light bar glows red
}
WHEEL_COLOURS = {
    0: (0.045, 0.045, 0.050),  # tyre
    1: (0.720, 0.740, 0.780),  # rim face
}

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
ATLAS_OUT = os.path.join(HERE, "car_atlas.png")
CAR_OUT = os.path.join(PROJECT, "romfs", "assets", "car.mwcar")


def full_id(low):
    return 0x100000000000000 | low


def open_vehicles():
    path = os.path.join(str(MW_ROOT), "VEHICLES")
    if not os.path.isdir(path):
        raise SystemExit(
            "vehicle bundles not found under %s; set NFS_MW_DIR to the game folder"
            % path
        )
    bundles = {}
    for name in TEXTURE_BUNDLES:
        bundles[name] = Bundle(os.path.join(path, name))
    return bundles


def find_texture(bundles, low_id):
    """Resolve a texture id across the vehicle bundles (not HAWAII)."""
    rid = full_id(low_id)
    for name in TEXTURE_BUNDLES:
        b = bundles[name]
        for r in b.resources:
            if r.id == rid and r.type_id == 1:
                return b, r
    return None, None


def decode_submesh(d1, sub, uv_floor):
    """Decode one type-5 submesh into positions, UVs and triangles."""
    indices = read_indices(d1, sub["idx_off"], sub["idx_sz"], sub.get("index_count"))
    real = [i for i in indices if i != 0xFFFF]
    if not real:
        return None
    count = max(real) + 1
    stride, rem = divmod(sub["vtx_sz"], count)
    if stride < 20 or stride > 80 or rem >= 24:
        return None

    pos = np.array(
        [struct.unpack_from("<3h", d1, sub["vtx_off"] + k * stride) for k in range(count)],
        dtype=np.float64,
    ) / 8192.0

    # UV0 sits on a 4-byte boundary after the 8-byte packed position.  Scan
    # for the offset whose half-floats form a mostly in-range, non-constant
    # pair; some submeshes (small badges) legitimately have none.
    uv_off = None
    for off in range(uv_floor, stride - 3, 4):
        vals = []
        ok = True
        for k in range(0, count, max(1, count // 3000)):
            u, v = struct.unpack_from("<2e", d1, sub["vtx_off"] + k * stride + off)
            if not (math.isfinite(u) and math.isfinite(v)):
                ok = False
                break
            vals.append((u, v))
        if not ok or len(vals) < 3:
            continue
        in01 = sum(1 for u, v in vals if -0.05 <= u <= 1.05 and -0.05 <= v <= 1.05)
        uniq = len(set((round(u, 2), round(v, 2)) for u, v in vals))
        if in01 / len(vals) > 0.985 and uniq > 8:
            uv_off = off
            break

    if uv_off is None:
        uvs = np.zeros((count, 2), dtype=np.float64)
    else:
        uvs = np.array(
            [struct.unpack_from("<2e", d1, sub["vtx_off"] + k * stride + uv_off)
             for k in range(count)],
            dtype=np.float64,
        )

    tris = np.array(strips_to_triangles(split_strips(indices)), dtype=np.int32)
    if not len(tris):
        return None
    return pos, uvs, tris, uv_off is not None


def smooth_normals(verts, tris):
    normals = np.zeros_like(verts)
    for a, b, c in tris:
        n = np.cross(verts[b] - verts[a], verts[c] - verts[a])
        normals[a] += n
        normals[b] += n
        normals[c] += n
    length = np.linalg.norm(normals, axis=1)
    length[length < 1e-12] = 1.0
    return normals / length[:, None]


def shade(normals):
    """Gentle directional tint, matching the world mesh's lighting bake."""
    ndotl = np.clip(normals[:, 0] * 0.45 + normals[:, 1] * 0.72 + normals[:, 2] * 0.40,
                    0.0, 1.0)
    lit = 0.78 + 0.22 * ndotl
    return np.stack([lit, lit * 0.985, lit * 0.955], axis=1)


def uv_in_tile(u, v, tile):
    """Source [0,1] UV -> atlas UV, with tex3ds' vertical flip applied."""
    x, y, w, h = tile[:4]
    u = u - math.floor(u)
    v = v - math.floor(v)
    u = min(1.0, max(0.0, u))
    v = min(1.0, max(0.0, v))
    a = (x + INSET + u * (w - 2.0 * INSET)) / ATLAS_W
    b = 1.0 - (y + INSET + v * (h - 2.0 * INSET)) / ATLAS_H
    return a, b


def build_atlas(bundles):
    atlas = Image.new("RGBA", (ATLAS_W, ATLAS_H), (0, 0, 0, 255))
    for name, (x, y, w, h, tid) in TILES.items():
        if tid is None:
            atlas.paste((255, 255, 255, 255), (x, y, x + w, y + h))
            continue
        b, r = find_texture(bundles, tid)
        if r is None:
            print("  missing texture %#x (%s), using grey" % (tid, name))
            atlas.paste((96, 96, 96, 255), (x, y, x + w, y + h))
            continue
        arr = mwtex.decode_texture(b, r)
        if arr is None:
            print("  undecodable texture %#x (%s), using grey" % (tid, name))
            atlas.paste((96, 96, 96, 255), (x, y, x + w, y + h))
            continue
        im = Image.fromarray(arr, "RGBA").convert("RGB")
        im = im.resize((w, h), Image.LANCZOS).convert("RGBA")
        a = np.array(im)
        a[..., 3] = 255
        atlas.paste(Image.fromarray(a, "RGBA"), (x, y))
    atlas.save(ATLAS_OUT)
    return atlas


def mesh_block(parts):
    """Build one vertex/index block from decoded parts."""
    vertices = []
    indices = []

    for part in parts:
        base = len(vertices)
        for tri in part["tris"]:
            indices.extend((int(tri[0]) + base, int(tri[1]) + base, int(tri[2]) + base))
        for k, pos in enumerate(part["verts"]):
            r, g, b = part["colour"][k]
            u, v = part["uv"][k]
            # Atlas UVs are biased into [-32768, 32767]; the shader scales
            # by 1/65535 and adds 32768/65535 to recover 0..1.
            u16 = max(-32768, min(32767, int(round(u * 65535.0 - 32768.0))))
            v16 = max(-32768, min(32767, int(round(v * 65535.0 - 32768.0))))
            vertices.append((float(pos[0]), float(pos[1]), float(pos[2]),
                             int(round(r * 255)), int(round(g * 255)),
                             int(round(b * 255)), 255, u16, v16))

    if len(indices) & 1:
        indices.append(0)  # keep the vertex block 4-byte aligned

    out = bytearray()
    out += struct.pack("<%dH" % len(indices), *indices)
    for p in vertices:
        out += struct.pack("<3f4B2h", p[0], p[1], p[2],
                           p[3], p[4], p[5], p[6], p[7], p[8])
    return bytes(out), len(vertices), len(indices)


def pack_car(body_parts, wheel_parts, wheel_radius):
    """Write .mwcar v2: body block plus one centred wheel block.

    The runtime instances the wheel block four times so it can steer and
    spin; baking the transforms would freeze them like the retail-imported
    static preview.
    """
    body_blob, body_v, body_i = mesh_block(body_parts)
    wheel_blob, wheel_v, wheel_i = mesh_block(wheel_parts)
    if body_v > 0xFFFF or wheel_v > 0xFFFF:
        raise SystemExit("car vertex count exceeds uint16 indices")

    header = struct.pack(
        "<4sIIIII4fI",
        b"MWCR", 2, body_v, body_i, wheel_v, wheel_i,
        wheel_radius, WHEEL_X * MODEL_SCALE, WHEEL_Z * MODEL_SCALE,
        WHEEL_Y * MODEL_SCALE, 0)
    return header + body_blob + wheel_blob, body_v, body_i, wheel_v, wheel_i


def body_parts(bundle):
    resource = next(r for r in bundle.resources
                    if r.type_id == 5 and r.index == BODY_RES)
    parsed = parse_renderable(resource, bundle)
    if parsed is None:
        raise SystemExit("body resource %d did not parse" % BODY_RES)

    d1 = parsed["d1"]
    parts = []
    for sub_index, sub in enumerate(parsed["subs"]):
        if sub_index in SKIP_BODY_SUBS:
            continue
        decoded = decode_submesh(d1, sub, uv_floor=20)
        if decoded is None:
            continue
        pos, uvs, tris, has_uv = decoded
        pos = pos * MODEL_SCALE

        normals = smooth_normals(pos, tris)
        if sub_index == 4:
            tints = np.tile(np.array([0.35, 0.40, 0.48]), (len(pos), 1))
            taillight_mask = (pos[:, 2] < -1.5) & (pos[:, 1] < 0.8)
            tints[taillight_mask] = [0.90, 0.15, 0.10]
            colour = shade(normals) * tints
        else:
            tint = np.array(TINT_BY_SUB.get(sub_index, (1.0, 1.0, 1.0)))
            colour = shade(normals) * tint

        tile = TILES[TILE_BY_SUB.get(sub_index, "white")]
        if has_uv:
            remapped = np.array([uv_in_tile(u, v, tile) for u, v in uvs])
        else:
            cx = (tile[0] + tile[2] * 0.5) / ATLAS_W
            cy = 1.0 - (tile[1] + tile[3] * 0.5) / ATLAS_H
            remapped = np.tile((cx, cy), (len(pos), 1))

        parts.append(dict(sub=sub_index, verts=pos, tris=tris, uv=remapped, colour=colour))
        print("  body sub %2d: %4d verts %4d tris  tile %-9s uv %s"
              % (sub_index, len(pos), len(tris), TILE_BY_SUB.get(sub_index, "white"),
                 "yes" if has_uv else "none"))
    return parts


def wheel_parts(bundle):
    resource = next(r for r in bundle.resources
                    if r.type_id == 5 and r.index == WHEEL_RES)
    parsed = parse_renderable(resource, bundle)
    d1 = parsed["d1"]

    wheel = []
    for sub_index, sub in enumerate(parsed["subs"]):
        decoded = decode_submesh(d1, sub, uv_floor=8)
        if decoded is None:
            continue
        pos, uvs, tris, _ = decoded
        pos = pos * (MODEL_SCALE * WHEEL_SCALE)
        normals = smooth_normals(pos, tris)
        colour = shade(normals) * np.array(WHEEL_COLOURS.get(sub_index, (0.5, 0.5, 0.5)))
        wheel.append(dict(sub=sub_index, verts=pos, tris=tris, colour=colour))
        print("  wheel sub %d: %4d verts %4d tris" % (sub_index, len(pos), len(tris)))

    tile = TILES["white"]
    cx = (tile[0] + tile[2] * 0.5) / ATLAS_W
    cy = 1.0 - (tile[1] + tile[3] * 0.5) / ATLAS_H

    # One wheel centred on the origin, axle along X, rim facing +X.  The
    # right side is rotated 180 degrees about Y at runtime.
    parts = []
    radius = 0.0
    for w in wheel:
        v = w["verts"].copy()
        radius = max(radius, float(np.abs(v[:, 1:3]).max()))
        parts.append(dict(sub=w["sub"], verts=v, tris=w["tris"],
                          uv=np.tile((cx, cy), (len(v), 1)),
                          colour=w["colour"]))
    return parts, radius


def main():
    bundles = open_vehicles()
    lo = bundles[BODY_BUNDLE]

    print("building car atlas")
    build_atlas(bundles)

    print("decoding body (%s res %d)" % (BODY_BUNDLE, BODY_RES))
    body = body_parts(lo)
    print("decoding wheels (%s res %d)" % (BODY_BUNDLE, WHEEL_RES))
    wheels, radius = wheel_parts(lo)

    data, bv, bi, wv, wi = pack_car(body, wheels, radius)
    os.makedirs(os.path.dirname(CAR_OUT), exist_ok=True)
    with open(CAR_OUT, "wb") as f:
        f.write(data)

    print("wrote %s: body %d verts/%d tris, wheel %d verts/%d tris (r %.3f m), %.1f KiB"
          % (CAR_OUT, bv, bi // 3, wv, wi // 3, radius, len(data) / 1024.0))
    print("wrote %s" % ATLAS_OUT)


if __name__ == "__main__":
    main()
