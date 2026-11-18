#!/usr/bin/env python3
"""Pick a diffuse/albedo texture for every world material.

Walks the same renderables as extract_world.py (centre/radius selection),
resolves each submesh's material id across all bundles, decodes every texture
the material references and keeps the one that looks most like an albedo
(see mwtex.texture_albedo_rank).

Output: tools/world_texture_map.json
    { "materials": { "0x...": {"tex": "0x...", "bundle": "...", "index": N} },
      "missing": ["0x..."] }
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mwtex
from bndl import Bundle
from mwmesh import parse_renderable

MW_DIR = mwtex.MW_DIR
MAIN_BUNDLE = "ARENA_527911.BNDL"
ORIGIN = (-168.0, 30.0, 4058.0)
RADIUS = 750.0
MAX_RENDERABLE_RADIUS = 500.0

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_JSON = os.path.join(HERE, "world_texture_map.json")

# Manual overrides: material id -> texture id (or None to force a flat tile).
# Filled in after visual inspection of a build.
OVERRIDES = {
    # Black-and-white static noise and a 128px reflection-cubemap face win
    # the generic score but are not albedo; use the wall textures that the
    # same material graphs reference instead.
    0x10000000010f457: 0x1000000000f3a8b,
    0x1000000000887d1: 0x10000000006175b,
    0x1000000000f69bd: 0x1000000000f69b3,
    0x100000000094c02: 0x1000000001e1dc9,
}


def renderable_materials():
    """Return {material_id: submesh_count} for the selected world renderables."""
    b = Bundle(os.path.join(MW_DIR, MAIN_BUNDLE))
    counts = {}
    for r in b.resources:
        if r.type_id != 5 or not r.has_stream1:
            continue
        info = parse_renderable(r, b)
        if not info:
            continue
        cx, cy, cz = info["centre"]
        if abs(cx) < 50.0 and abs(cz) < 50.0:
            continue
        if info["radius"] > MAX_RENDERABLE_RADIUS:
            continue
        if math.hypot(cx - ORIGIN[0], cz - ORIGIN[2]) > RADIUS:
            continue
        start = info["subs"][-1]["desc_off"] + 96
        d0 = info["d0"]
        for k in range(info["nsub"]):
            off = start + k * 16
            if off + 16 > len(d0):
                break
            mid = int.from_bytes(d0[off:off + 8], "little")
            if mid:
                counts[mid] = counts.get(mid, 0) + 1
    return counts


def choose_textures(counts):
    from PIL import Image
    chosen = {}
    missing = []
    for mid, nsub in sorted(counts.items(), key=lambda kv: -kv[1]):
        if mid in OVERRIDES:
            tid = OVERRIDES[mid]
            if tid is None:
                missing.append(mid)
                continue
            b, r = mwtex.resolve(tid, 1)
            if not r:
                missing.append(mid)
                continue
            chosen[mid] = dict(tex=tid, bundle=os.path.basename(b.path), index=r.index)
            continue

        best = None
        for tid in mwtex.material_texture_ids(resolve_material_d0(mid)):
            b, r = mwtex.resolve(tid, 1)
            if not r:
                continue
            info = b.texture_info(r)
            if not info or info["format"] not in (71, 72, 77, 78):
                continue
            px = mwtex.decode_texture(b, r)
            if px is None:
                continue
            rank = mwtex.texture_albedo_rank(Image.fromarray(px, "RGBA"))
            if rank <= 0.0:
                continue
            # Mild preference for higher resolution when the scores are close.
            res_bonus = 0.06 * math.log2(max(1, info["width"] * info["height"]) / 65536.0)
            score = rank + res_bonus
            if best is None or score > best[0]:
                best = (score, tid, os.path.basename(b.path), r.index)
        if best is None:
            missing.append(mid)
        else:
            chosen[mid] = dict(tex=best[1], bundle=best[2], index=best[3])

    return chosen, missing


_D0 = {}


def resolve_material_d0(mid):
    if mid in _D0:
        return _D0[mid]
    b, r = mwtex.resolve(mid, 2)
    d0 = b.stream0(r) if r else b""
    _D0[mid] = d0
    return d0


def main():
    counts = renderable_materials()
    chosen, missing = choose_textures(counts)
    uniq = {}
    for mid, c in chosen.items():
        uniq.setdefault(c["tex"], []).append(mid)
    print("world materials: %d, with albedo: %d, unique textures: %d"
          % (len(counts), len(chosen), len(uniq)))
    if missing:
        print("no albedo (%d): %s" % (len(missing), ", ".join(hex(m)[-7:] for m in missing)))
    with open(OUT_JSON, "w") as f:
        json.dump({
            "materials": {hex(k): dict(v, tex=hex(v["tex"])) for k, v in chosen.items()},
            "missing": [hex(m) for m in missing],
        }, f, indent=1)
    print("wrote", OUT_JSON)


if __name__ == "__main__":
    main()
