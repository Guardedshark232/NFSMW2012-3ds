#!/usr/bin/env python3
"""Build contact sheets from MW2012 UI texture bundles so real HUD art can be found."""
import os
import sys
import math
import glob

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bndl import Bundle
from PIL import Image, ImageDraw
from game_paths import MW_ROOT


def collect(folder, pattern="*.BNDL"):
    entries = []
    for path in sorted(glob.glob(os.path.join(folder, pattern))):
        try:
            b = Bundle(path)
        except Exception:
            continue
        for r in b.resources:
            if r.type_id != 1 or not r.has_stream1:
                continue
            try:
                img = b.texture(r)
            except Exception:
                continue
            if img:
                entries.append((os.path.basename(path)[:-5], r.id, r.type_id, img))
    return entries


def sheet(entries, out, cols=8, thumb=112, per_sheet=160):
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    pages = math.ceil(len(entries) / per_sheet)
    for page in range(pages):
        subset = entries[page * per_sheet:(page + 1) * per_sheet]
        rows = math.ceil(len(subset) / cols)
        label_h = 14
        W = cols * (thumb + 6)
        H = rows * (thumb + label_h + 6)
        canvas = Image.new("RGBA", (W, H), (18, 18, 22, 255))
        d = ImageDraw.Draw(canvas)
        for i, (name, rid, typ, img) in enumerate(subset):
            cx = (i % cols) * (thumb + 6) + 3
            cy = (i // cols) * (thumb + label_h + 6) + 3
            t = img.copy()
            t.thumbnail((thumb, thumb))
            bg = Image.new("RGBA", (thumb, thumb), (40, 40, 48, 255))
            bg.alpha_composite(t, ((thumb - t.width) // 2, (thumb - t.height) // 2))
            canvas.paste(bg, (cx, cy))
            d.text((cx + 1, cy + thumb + 1), f"{page*per_sheet+i}:{name}", fill=(255, 220, 120, 255))
        path = out if pages == 1 else out.replace(".png", f"_{page}.png")
        canvas.convert("RGB").save(path)
        print("wrote", path, len(subset), "tiles")


if __name__ == "__main__":
    folder = sys.argv[1] if len(sys.argv) > 1 else os.path.join(MW_ROOT, "UI/IMAGES/STREAMED")
    out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/opencode/mw_tex/sheet.png"
    entries = collect(folder)
    print("textures:", len(entries))
    sheet(entries, out)
