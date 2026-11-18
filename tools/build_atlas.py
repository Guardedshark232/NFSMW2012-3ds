#!/usr/bin/env python3
"""Build a downscaled texture atlas for the Fairhaven world mesh.

The 3DS has no use for the PC game's 512-2048 px maps, so every referenced
diffuse texture is reduced to a 128x128 tile and packed into a single
1024x1024 RGBA4444 page (64 tile slots).  A one-texel inset around each tile
keeps bilinear filtering from bleeding into the neighbour.

The world extractor bakes the atlas rectangle into the mesh's UVs, so the
runtime only needs one texture bind and no per-material bookkeeping.
"""
import json
import math
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mwtex

ATLAS = 1024
TILE = 128
GRID = ATLAS // TILE          # 8x8 = 64 slots
INSET = 1.0                   # texels of padding on each tile edge
FALLBACK = (-1, -1)           # grey tile for materials with no texture

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)


def load_map(path=None):
    path = path or os.path.join(HERE, "world_texture_map.json")
    with open(path) as f:
        data = json.load(f)
    return data


def build(map_data, out_png):
    mats = map_data["materials"]
    # Most-used textures get the first (cache-friendly) tiles.
    usage = {}
    for mid, c in mats.items():
        usage[int(c["tex"], 16)] = usage.get(int(c["tex"], 16), 0) + 1

    order = [t for t, _ in sorted(usage.items(), key=lambda kv: (-kv[1], kv[0]))]
    if len(order) > GRID * GRID - 1:
        raise SystemExit("atlas full: %d textures for %d slots" % (len(order), GRID * GRID - 1))

    tiles = {}
    for n, tid in enumerate(order):
        # Slot (0,0) is reserved for the flat grey fallback.
        n += 1
        tiles[tid] = ((n % GRID) * TILE, (n // GRID) * TILE)

    atlas = Image.new("RGBA", (ATLAS, ATLAS), (96, 96, 96, 255))
    for tid, (px, py) in tiles.items():
        b, r = mwtex.resolve(tid, 1)
        if not r:
            raise SystemExit("texture 0x%x not found" % tid)
        arr = mwtex.decode_texture(b, r)
        im = Image.fromarray(arr, "RGBA").convert("RGB")
        im = im.resize((TILE, TILE), Image.LANCZOS)
        im = im.convert("RGBA")
        a = np.array(im)
        a[..., 3] = 255
        atlas.paste(Image.fromarray(a, "RGBA"), (px, py))

    os.makedirs(os.path.dirname(out_png) or ".", exist_ok=True)
    atlas.save(out_png)

    # Map every material to the atlas origin of its texture; missing
    # materials fall back to the untouched grey reserved at slot 0,0.
    slot = {}
    for mid, c in mats.items():
        slot[mid] = tiles.get(int(c["tex"], 16), FALLBACK)
    for mid in map_data.get("missing", []):
        slot[mid] = FALLBACK
    return dict(tiles=tiles, slot=slot, atlas=out_png)


def uv_remap(u, v, origin):
    """Source [0,1] UV -> atlas UV inside the tile at `origin` (pixels).

    tex3ds flips images vertically when it packs the .t3x (PNG row 0 ends
    up at V=1), so the atlas V coordinate is mirrored.  Flipping it here
    keeps the source texture's orientation (v=0 = top row).
    """
    px, py = origin
    if px < 0:
        # Point at the middle of the reserved grey fallback cell (0,0).
        return (TILE * 0.5) / ATLAS, 1.0 - (TILE * 0.5) / ATLAS
    a = (px + INSET + max(0.0, min(1.0, u)) * (TILE - 2.0 * INSET)) / ATLAS
    b = 1.0 - (py + INSET + max(0.0, min(1.0, v)) * (TILE - 2.0 * INSET)) / ATLAS
    return a, b


def main():
    out = os.path.join(HERE, "world_atlas.png")
    info = build(load_map(), out)
    print("wrote %s: %d textures, %d material slots" % (out, len(info["tiles"]), len(info["slot"])))


if __name__ == "__main__":
    main()
