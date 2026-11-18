#!/usr/bin/env python3
"""Criterion 'bnd2' bundle reader (Need for Speed: Most Wanted 2012 retail PC).

Verified against retail bundles:
  * header +0x18 = base offset of the bulk (stream-1) data region
  * 72-byte entry records, two streams per entry:
      stream 0: payload, offset relative to header.data_offset
      stream 1: bulk data, offset relative to header.bulk_offset
  * texture resources (type 1) carry a 48-byte DXGI descriptor in stream 0
    and BC1/BC3 blocks (plus mips) in stream 1

Usage:
    from bndl import Bundle
    b = Bundle("TRK_UNIT19.BNDL")
    for res in b.resources: ...
    tex = b.texture(res)      # -> PIL.Image or None
"""
import struct
import zlib
from dataclasses import dataclass
from typing import Optional, List

SIZE_MASK = 0x0FFFFFFF
FLAG_MASK = 0xF0000000

DXGI_BC1 = 71
DXGI_BC1_SRGB = 72
DXGI_BC3 = 77
DXGI_BC3_SRGB = 78


@dataclass
class Resource:
    index: int
    id: int
    type_id: int
    size0: int
    comp0: int
    offset0: int         # relative to data_offset
    size1: int
    comp1: int
    offset1: int         # relative to bulk_offset
    payload_size: int
    raw0: bool = False   # stream stored uncompressed
    raw1: bool = False
    data: Optional[bytes] = None   # stream 0 (lazy)
    data1: Optional[bytes] = None  # stream 1 (lazy)

    @property
    def has_stream1(self) -> bool:
        return self.comp1 != 0


class Bundle:
    def __init__(self, path: str):
        self.path = path
        with open(path, "rb") as f:
            self.raw = f.read()

        (magic, self.version, self.file_size, self.num_entries,
         self.entry_offset, self.data_offset) = struct.unpack_from("<4sIIIII", self.raw, 0)
        if magic != b"bnd2":
            raise ValueError(f"{path}: not a bnd2 bundle ({magic!r})")
        self.bulk_offset = struct.unpack_from("<I", self.raw, 24)[0]

        self.resources: List[Resource] = []
        for i in range(self.num_entries):
            o = self.entry_offset + i * 72
            (rid, size0, size1, _r0, _r1, comp0, comp1,
             _r2, _r3, offset0, offset1, _r4, _r5, payload, type_id,
             _r7, _r8) = struct.unpack_from("<Q16I", self.raw, o)
            self.resources.append(Resource(
                index=i, id=rid, type_id=type_id,
                size0=size0 & SIZE_MASK, comp0=comp0 & SIZE_MASK,
                offset0=offset0, size1=size1 & SIZE_MASK, comp1=comp1 & SIZE_MASK,
                offset1=offset1, payload_size=payload,
                raw0=bool(comp0 & FLAG_MASK), raw1=bool(comp1 & FLAG_MASK)))

    # -- low level -----------------------------------------------------
    def chunk(self, offset: int, comp_size: int, raw: bool = False) -> bytes:
        """Read and inflate one stream. Falls back to raw deflate."""
        blob = self.raw[offset:offset + comp_size]
        if not blob:
            return b""
        if raw:
            return blob
        if blob[:1] == b"\x78":
            return zlib.decompress(blob)
        try:
            return zlib.decompress(blob)
        except zlib.error:
            return zlib.decompress(blob, -15)

    def stream0(self, res: Resource) -> bytes:
        if res.data is None:
            res.data = self.chunk(self.data_offset + res.offset0, res.comp0, res.raw0)
        return res.data

    def stream1(self, res: Resource) -> bytes:
        if not res.has_stream1:
            return b""
        if res.data1 is None:
            res.data1 = self.chunk(self.bulk_offset + res.offset1, res.comp1, res.raw1)
        return res.data1

    # -- textures ------------------------------------------------------
    def texture_info(self, res: Resource):
        d = self.stream0(res)
        if len(d) < 48:
            return None
        fmt, mips, w, h = struct.unpack_from("<IIHH", d, 28)
        return {"format": fmt, "mips": mips, "width": w, "height": h}

    def texture(self, res: Resource):
        """Decode a type-1 texture resource to a PIL image (RGBA)."""
        from PIL import Image
        info = self.texture_info(res)
        if not info:
            return None
        w, h, fmt = info["width"], info["height"], info["format"]
        data = self.stream1(res)
        if fmt in (DXGI_BC1, DXGI_BC1_SRGB):
            pixels = decode_bc(data, w, h, alpha=False)
        elif fmt in (DXGI_BC3, DXGI_BC3_SRGB):
            pixels = decode_bc(data, w, h, alpha=True)
        else:
            return None
        return Image.frombytes("RGBA", (w, h), pixels)


# ----------------------------------------------------------------------
# DXT / BCn decoding (small, dependency-free apart from PIL/numpy)
# ----------------------------------------------------------------------
def _rgb565(c):
    return (((c >> 11) & 0x1F) * 255 // 31,
            ((c >> 5) & 0x3F) * 255 // 63,
            (c & 0x1F) * 255 // 31)


def decode_bc(data: bytes, w: int, h: int, alpha: bool) -> bytes:
    out = bytearray(w * h * 4)
    bw, bh = max(1, w // 4), max(1, h // 4)
    block = 16 if alpha else 8
    p = 0
    for by in range(bh):
        for bx in range(bw):
            if p + block > len(data):
                return bytes(out)
            b = data[p:p + block]
            p += block

            if alpha:
                a0, a1 = b[0], b[1]
                abits = int.from_bytes(b[2:8], "little")
                alphas = [a0, a1]
                if a0 > a1:
                    alphas += [((6 - i) * a0 + (1 + i) * a1) // 7 for i in range(6)]
                else:
                    alphas += [((4 - i) * a0 + (1 + i) * a1) // 5 for i in range(4)]
                    alphas += [0, 255]
                c0, c1 = struct.unpack_from("<HH", b, 8)
                cbits = struct.unpack_from("<I", b, 12)[0]
            else:
                c0, c1 = struct.unpack_from("<HH", b, 0)
                cbits = struct.unpack_from("<I", b, 4)[0]

            r0, g0, b0 = _rgb565(c0)
            r1, g1, b1 = _rgb565(c1)
            cols = [(r0, g0, b0, 255), (r1, g1, b1, 255)]
            if c0 > c1 or alpha:
                cols += [((2 * r0 + r1) // 3, (2 * g0 + g1) // 3, (2 * b0 + b1) // 3, 255),
                         ((r0 + 2 * r1) // 3, (g0 + 2 * g1) // 3, (b0 + 2 * b1) // 3, 255)]
            else:
                cols += [((r0 + r1) // 2, (g0 + g1) // 2, (b0 + b1) // 2, 255),
                         (0, 0, 0, 0)]

            for py in range(4):
                y = by * 4 + py
                if y >= h:
                    break
                for px in range(4):
                    x = bx * 4 + px
                    if x >= w:
                        break
                    r, g, bb, a = cols[(cbits >> (2 * (py * 4 + px))) & 3]
                    if alpha:
                        a = alphas[(abits >> (3 * (py * 4 + px))) & 7]
                    o = (y * w + x) * 4
                    out[o:o + 4] = bytes((r, g, bb, a))
    return bytes(out)
