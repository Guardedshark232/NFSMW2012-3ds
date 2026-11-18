# NFS: Most Wanted (2012) — Fairhaven 3DS

A Nintendo 3DS re-implementation of the Fairhaven open-world harbour district
from *Need for Speed: Most Wanted (2012)*, built with devkitPro, libctru,
citro3d and citro2d.

## Build

Requires devkitPro (`DEVKITPRO`, default `/opt/devkitpro`) with
`3ds-dev`, `citro3d`, `citro2d`, and the `tex3ds` / `picasso` tools.

```sh
make -j8        # nfs_3ds.3dsx
make cia        # nfs_3ds.cia (uses makerom/bannertool via wine)
```

`make` embeds `romfs/` into both outputs. The 3DSX can be launched from the
Homebrew Launcher or with Azahar/Citra; the CIA can be installed with
FBI/GodMode9 on hardware.

## Controls

| Input      | Action              |
|------------|---------------------|
| Circle Pad | Steer               |
| A          | Accelerate          |
| B          | Brake               |
| X          | Nitrous             |
| Y          | Handbrake / drift   |
| START      | EasyDrive menu      |
| SELECT     | Reset car           |

The top screen shows the 3D chase camera; the bottom screen is the EasyDrive
HUD with speed, gear/RPM, nitrous, telemetry and a scrolling radar map.

## World data

`romfs/assets/fairhaven_block.mwmesh` contains the converted Fairhaven block
as six chunks of indexed geometry (v3 format: `MWMS` header, per-chunk 20-byte
packed vertices `float3 position + uint8x4 lighting tint + int16x2 atlas UV`
and `uint16` indices).

- `tools/extract_world.py` / `tools/mwmesh.py` convert the original Chameleon
  bundles into `.mwmesh`.
- `tools/gen_minimap.py` rasterises a top-down city/street map from the mesh;
  `make minimap` regenerates `romfs/assets/minimap.t3x` with `tex3ds`.

## World textures

The map uses the game's own diffuse maps, downscaled for the 3DS rather than
redrawn.  The pipeline runs in three steps:

```sh
python3 tools/world_textures.py   # pick an albedo per material
python3 tools/extract_world.py    # rebuild world_atlas.png + fairhaven_block.mwmesh
make -j8                          # tex3ds-compress the atlas into the romfs
```

- `tools/mwtex.py` resolves material ids across the bundles, decodes BC1/BC3
  textures and scores each reference as a likely albedo (normal maps, gloss /
  window masks, red-cyan blend masks and black-on-white decal sheets are
  rejected).
- `tools/world_textures.py` picks the best-scoring texture per material and
  writes `tools/world_texture_map.json`; `OVERRIDES` fixes up individual
  materials after visual inspection.
- `tools/build_atlas.py` packs every selected texture into one 1024×1024
  RGBA4444 page (`romfs/assets/world_atlas.t3x`), 128×128 tiles with a one
  texel inset, and bakes the atlas rectangle into the mesh UVs so the runtime
  only binds a single texture per frame.

A few materials tile their source UVs far outside `[0,1]`; the extractor fits
each part's UV range into its tile (repeating the tile would bleed into
neighbouring atlas slots under `CLAMP_TO_EDGE`), so very large surfaces can
look softer than the rest of the city.

## Rendering notes

- The world is drawn with `C3D_DrawElements`; the VBO/IBO binding is re-fetched
  with `C3D_GetBufInfo()` for every chunk draw. Holding one `C3D_BufInfo*`
  across multiple draws left the first chunk's VBO bound and produced the
  original giant-slab artifact.
- A CPU-side `lo`/`hi` height grid samples quays, roads and decks so the car
  and camera follow the surface nearest their current height and never drop
  to the seabed.
- Reversed-Z depth (`C3D_DepthMap(true, -1.0f, 0.0f)`) is re-applied each
  frame because citro2d changes the depth mapping.
- The radar window is a 240 m sub-rectangle of the generated 288×288 map,
  centred on the car, with a heading marker pinned to the panel centre.
- TEV state is only re-uploaded when a stage is marked dirty. `C3D_GetTexEnv()`
  does that, but the inline `C3D_TexEnvSrc/Func` setters do not — so after the
  sky draw flushes the dirty flag, the world's texture combine must be
  re-flagged with `C3D_DirtyTexEnv(env)` or the map renders untextured.
- Diagnostic framebuffer dumps (`src/renderer/fbdump.cpp`) and the automated
  driving telemetry in `src/core/main.cpp` are compiled out by default; enable
  `FB_DUMP_ENABLE`/`DIAG_AUTO_DRIVE` when investigating render or physics
  regressions.
