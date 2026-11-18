#!/usr/bin/env python3
"""Build MW2012 UI textures and convert to 3DS .t3x format via tex3ds."""

import os
import subprocess
from PIL import Image, ImageDraw, ImageFont

def make_ui_textures():
    os.makedirs("/tmp/opencode/mw_assets", exist_ok=True)
    os.makedirs("romfs/assets", exist_ok=True)

    # 1. Porsche Crest / Car Badge (64x64)
    crest = Image.new("RGBA", (64, 64), (0, 0, 0, 0))
    draw = ImageDraw.Draw(crest)
    draw.polygon([(32, 4), (60, 20), (60, 44), (32, 60), (4, 44), (4, 20)], fill=(220, 180, 40, 255), outline=(255, 220, 100, 255))
    draw.rectangle([24, 24, 40, 40], fill=(50, 50, 50, 255))
    crest.save("/tmp/opencode/mw_assets/crest.png")

    # 2. EasyDrive Menu Box (256x128)
    ed = Image.new("RGBA", (256, 128), (20, 25, 35, 230))
    draw = ImageDraw.Draw(ed)
    draw.rectangle([0, 0, 255, 127], outline=(0, 220, 255, 255), width=2)
    draw.rectangle([10, 10, 245, 35], fill=(0, 180, 220, 180))
    ed.save("/tmp/opencode/mw_assets/easydrive.png")

    # 3. Minimap / Map tile (128x128)
    map_tile = Image.new("RGBA", (128, 128), (14, 18, 28, 255))
    draw = ImageDraw.Draw(map_tile)
    # Draw roads
    draw.line([(64, 0), (64, 128)], fill=(100, 110, 130, 255), width=8)
    draw.line([(0, 64), (128, 64)], fill=(100, 110, 130, 255), width=8)
    draw.ellipse([56, 56, 72, 72], fill=(255, 220, 0, 255)) # Player marker
    map_tile.save("/tmp/opencode/mw_assets/minimap.png")

    # 4. Speedometer gauge background (128x128)
    speedo = Image.new("RGBA", (128, 128), (0, 0, 0, 0))
    draw = ImageDraw.Draw(speedo)
    draw.arc([8, 8, 120, 120], 135, 405, fill=(0, 200, 255, 255), width=6)
    speedo.save("/tmp/opencode/mw_assets/speedo.png")

    # Convert via tex3ds
    tex3ds_bin = "/opt/devkitpro/tools/bin/tex3ds"
    if os.path.exists(tex3ds_bin):
        for name in ["crest", "easydrive", "minimap", "speedo"]:
            in_png = f"/tmp/opencode/mw_assets/{name}.png"
            out_t3x = f"romfs/assets/{name}.t3x"
            subprocess.run([tex3ds_bin, "-f", "rgba8", "-o", out_t3x, in_png], check=True)
            print(f"Converted {in_png} -> {out_t3x}")
    else:
        print("Warning: tex3ds not found at /opt/devkitpro/tools/bin/tex3ds")

if __name__ == "__main__":
    make_ui_textures()
    print("Asset build complete.")
