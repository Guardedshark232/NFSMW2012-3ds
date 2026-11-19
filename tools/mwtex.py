#!/usr/bin/env python3
"""Texture/material resolution helpers for NFS MW2012 world extraction.

The world renderables only reference materials by id; the material resources
in turn reference textures by id.  Both lookups can cross bundle boundaries
(the ARENA stream references DISTRICT/TRK/GLOBALRESOURCES assets), so this
module keeps a lazily-populated, process-wide cache of bundle readers and of
the id -> (bundle, resource) table built from the whole HAWAII folder.

Also contains a vectorised BC1/BC3 decoder, ~50x faster than the pure-Python
reference implementation in bndl.py, so the atlas builder can decode a few
hundred 512x512 textures in seconds.
"""
import os
import struct
import numpy as np

from bndl import Bundle
from game_paths import MW_DIR, require_game_files

# id -> (bundle filename, resource index, type_id)
_INDEX = None
_BUNDLES = {}


def bundle(name):
    """Open (and cache) a bundle by file name."""
    if name not in _BUNDLES:
        _BUNDLES[name] = Bundle(os.path.join(MW_DIR, name))
    return _BUNDLES[name]


_CACHE = "/tmp/opencode/mw/global_ids.json"


def global_index(progress=False):
    """Scan every .BNDL in HAWAII once and index type-1/type-2 resources."""
    global _INDEX
    require_game_files()
    if _INDEX is not None:
        return _INDEX
    if os.path.exists(_CACHE):
        import json
        with open(_CACHE) as f:
            raw = json.load(f)
        _INDEX = {int(k, 16): [(fn, i, t) for fn, ts in v for t, i in ts]
                  for k, v in raw.items()}
        return _INDEX
    idx = {}
    names = sorted(n for n in os.listdir(MW_DIR) if n.upper().endswith(".BNDL"))
    for n, name in enumerate(names):
        if progress:
            print("  scan %3d/%d %s" % (n + 1, len(names), name), flush=True)
        try:
            with open(os.path.join(MW_DIR, name), "rb") as f:
                hdr = f.read(32)
                magic, _ver, _fs, nent, entoff, _doff = struct.unpack_from("<4sIIIII", hdr, 0)
                if magic != b"bnd2":
                    continue
                f.seek(entoff)
                table = f.read(nent * 72)
        except OSError:
            continue
        for i in range(nent):
            o = i * 72
            rid = struct.unpack_from("<Q", table, o)[0]
            typ = struct.unpack_from("<I", table, o + 60)[0]
            if typ in (1, 2):
                idx.setdefault(rid, []).append((name, i, typ))
    _INDEX = idx
    return idx


def resolve(rid, want_type=None):
    """Return (bundle, Resource) for a resource id, or (None, None)."""
    for name, index, typ in global_index().get(rid, ()):
        if want_type is not None and typ != want_type:
            continue
        b = bundle(name)
        for r in b.resources:
            if r.index == index and r.type_id == typ:
                return b, r
    return None, None


# ---------------------------------------------------------------------------
# Material parsing
# ---------------------------------------------------------------------------
# A type-2 material ends with a table of 16-byte records:
#   u32 bid (low 32 bits of the referenced resource id)
#   u8  subtype, u8 0, u8 0, u8 type      (type 1 = texture, 0 = misc, 0x53 shader)
#   u32 offset
#   u32 0
# Texture records appear in slot order.  The first record is always the
# material shader (subtype 0x53), not a texture.
def material_texture_ids(d0):
    ids = []
    seen = set()
    for off in range(0, len(d0) - 15, 4):
        u64 = struct.unpack_from("<Q", d0, off)[0]
        typ = u64 >> 56
        sub = struct.unpack_from("<H", d0, off + 4)[0]
        if typ != 1 or sub != 0:
            continue
        if u64 in seen:
            continue
        seen.add(u64)
        ids.append(u64)
    return ids


def texture_albedo_rank(img):
    """Heuristic 0..100 score: how likely is `img` a diffuse/albedo map?

    NFS MW2012 material graphs reference normal maps (purple/blue), gloss and
    window masks (flat or neon cyan/green/red) alongside the diffuse map.
    Neon mask channels are near-clipped and very saturated, whereas city
    albedo (asphalt, brick, concrete, glass) sits in a natural mid-tone band.
    """
    a = np.asarray(img.convert("RGB"), dtype=np.float32) / 255.0
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    mx = a.max(axis=2)
    mn = a.min(axis=2)
    mr, mg, mb = float(r.mean()), float(g.mean()), float(b.mean())
    luma = 0.299 * r + 0.587 * g + 0.114 * b
    std = float(luma.std())
    bright = float(mx.mean())
    sat = float(((mx - mn) / np.maximum(mx, 1e-4)).mean())
    clipped = float(((mx > 0.82) & (mn < 0.18)).mean())
    white_frac = float((luma > 0.85).mean())
    # Complementary red/cyan blobs are blend/gloss masks (the game packs
    # colour-channel masks this way), never city albedo.
    red_frac = float(((r > g + 0.08) & (r > b + 0.08)).mean())
    cyan_frac = float(((g > r + 0.08) & (b > r + 0.08)).mean())

    if mb > mr + 0.06 and mb > mg + 0.06 and mb > 0.35:
        return 0.0                       # normal map (purple/blue)
    if mg > mr * 1.30 and mb > mr * 1.15:
        return 0.0                       # cyan/teal window or gloss mask
    if mg > mr * 1.40 and mg > mb * 1.25:
        return 0.0                       # green mask
    if mb > mr * 1.40 and mb > mg * 1.15:
        return 0.0                       # blue mask
    if clipped > 0.12:
        return 0.0                       # neon/clipped mask channels
    if sat > 0.85:
        return 0.0                       # near-primary neon mask
    if red_frac > 0.10 and cyan_frac > 0.10:
        return 0.0                       # red/cyan blend mask
    if sat < 0.10 and std > 0.18:
        return 0.0                       # black-on-white decal/sign sheet
    if std < 0.035:
        return 0.0                       # constant mask
    if bright < 0.05:
        return 0.0                       # black mask
    if sat > 0.80 and std < 0.10:
        return 0.0                       # flat saturated mask
    if bright > 0.90:
        return 0.0                       # white grunge/alpha sheet
    if white_frac > 0.18 and sat < 0.12:
        return 0.0                       # black-on-white decal/sign atlas

    score = 14.0 + std * 120.0 + (1.0 - abs(bright - 0.42)) * 18.0 + sat * 22.0
    return max(0.0, min(100.0, score))


# ---------------------------------------------------------------------------
# Fast BC decoding
# ---------------------------------------------------------------------------
def _rgb565(c):
    r = ((c >> 11) & 0x1F).astype(np.uint16) * 255 // 31
    g = ((c >> 5) & 0x3F).astype(np.uint16) * 255 // 63
    b = (c & 0x1F).astype(np.uint16) * 255 // 31
    return r, g, b


def _bits16(block, off):
    """Little-endian 32-bit field from 4 bytes, vectorised."""
    return (block[:, off].astype(np.uint32)
            | (block[:, off + 1].astype(np.uint32) << 8)
            | (block[:, off + 2].astype(np.uint32) << 16)
            | (block[:, off + 3].astype(np.uint32) << 24))


def decode_bc_np(data, w, h, alpha):
    """Decode BC1 (alpha=False) or BC3 (alpha=True) to an (h,w,4) uint8 array."""
    bw, bh = max(1, (w + 3) // 4), max(1, (h + 3) // 4)
    n = bw * bh
    block = 16 if alpha else 8
    raw = np.frombuffer(data[:n * block], dtype=np.uint8)
    if raw.size < n * block:
        raw = np.concatenate([raw, np.zeros(n * block - raw.size, np.uint8)])
    blk = raw.reshape(n, block)

    if alpha:
        c0 = blk[:, 8].astype(np.uint32) | (blk[:, 9].astype(np.uint32) << 8)
        c1 = blk[:, 10].astype(np.uint32) | (blk[:, 11].astype(np.uint32) << 8)
        bits = _bits16(blk, 12)
    else:
        c0 = blk[:, 0].astype(np.uint32) | (blk[:, 1].astype(np.uint32) << 8)
        c1 = blk[:, 2].astype(np.uint32) | (blk[:, 3].astype(np.uint32) << 8)
        bits = _bits16(blk, 4)

    r0, g0, b0 = _rgb565(c0)
    r1, g1, b1 = _rgb565(c1)
    pal = np.empty((n, 4, 4), dtype=np.uint16)
    pal[:, 0] = np.stack([r0, g0, b0, np.full(n, 255, np.uint16)], axis=1)
    pal[:, 1] = np.stack([r1, g1, b1, np.full(n, 255, np.uint16)], axis=1)
    if alpha:
        pal[:, 2] = np.stack([(2 * r0 + r1) // 3, (2 * g0 + g1) // 3,
                              (2 * b0 + b1) // 3, np.full(n, 255, np.uint16)], axis=1)
        pal[:, 3] = np.stack([(r0 + 2 * r1) // 3, (g0 + 2 * g1) // 3,
                              (b0 + 2 * b1) // 3, np.full(n, 255, np.uint16)], axis=1)
    else:
        pal[:, 2] = np.stack([(r0 + r1) // 2, (g0 + g1) // 2,
                              (b0 + b1) // 2, np.full(n, 255, np.uint16)], axis=1)
        black = (c0 <= c1)
        pal[:, 3] = np.where(black[:, None],
                             np.zeros((n, 4), np.uint16),
                             pal[:, 2])

    idx = (bits[:, None] >> (2 * np.arange(16, dtype=np.uint32))[None, :]) & 3
    pix = pal[np.arange(n)[:, None], idx]          # (n,16,4)

    if alpha:
        a0 = blk[:, 0].astype(np.uint16)
        a1 = blk[:, 1].astype(np.uint16)
        abits = np.zeros(n, dtype=np.uint64)
        for k in range(6):
            abits |= blk[:, 2 + k].astype(np.uint64) << (8 * k)
        aidx = (abits[:, None] >> (3 * np.arange(16, dtype=np.uint64))[None, :]) & 7
        apal = np.empty((n, 8), dtype=np.uint16)
        apal[:, 0] = a0
        apal[:, 1] = a1
        interp8 = np.stack([((6 - i) * a0 + (1 + i) * a1) // 7 for i in range(6)], axis=1)
        interp6 = np.stack([((4 - i) * a0 + (1 + i) * a1) // 5 for i in range(4)], axis=1)
        gt = (a0 > a1)
        apal[:, 2:] = np.where(gt[:, None], interp8,
                               np.concatenate([interp6,
                                               np.zeros((n, 2), np.uint16)], axis=1))
        pix[..., 3] = apal[np.arange(n)[:, None], aidx.astype(np.int64)]

    pix = pix.reshape(bh, bw, 4, 4, 4).transpose(0, 2, 1, 3, 4).reshape(bh * 4, bw * 4, 4)
    return pix[:h, :w].astype(np.uint8)


def decode_texture(b, res):
    """Decode a type-1 resource to (h,w,4) uint8, or None."""
    info = b.texture_info(res)
    if not info or info["format"] not in (71, 72, 77, 78):
        return None
    w, h = info["width"], info["height"]
    alpha = info["format"] in (77, 78)
    return decode_bc_np(b.stream1(res), w, h, alpha)
