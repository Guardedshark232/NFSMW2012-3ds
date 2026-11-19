# Need for Speed: Most Wanted (2012) — Nintendo 3DS

An open-world driving port of the Fairhaven map from *Need for Speed: Most
Wanted (2012)* for the Nintendo 3DS. It runs on original 3DS, 2DS and 3DS XL
hardware and uses the New 3DS CPU/L2 speedup automatically when available.

The port renders the retail PC world — roads, the harbour, the container
terminal, city blocks and the highway network — streamed from the console's
SD card. It is built with devkitPro, libctru, citro3d and citro2d.


## Controls

| Input | Action |
| --- | --- |
| Circle Pad | Steer |
| A | Accelerate / brake reverse motion |
| B | Brake; hold at a stop to reverse |
| X | Nitrous |
| Y | Handbrake / drift |
| SELECT | Reset the car to spawn |

## Building

### Requirements

- devkitPro with `devkitARM`, `libctru`, `citro3d`, `citro2d`
- `tex3ds` and `picasso` from devkitPro tools
- For `make cia`: `makerom` and `bannertool` (a Wine prefix and the binaries
  under `NFSU2-3DS/tools/3ds-bin/` are already wired up in the Makefile)

```sh
make		          # builds nfs_3ds.3dsx
make cia          # builds installable nfs_3ds.cia
make clean
```

The 3DSX embeds the whole `romfs/` directory, so the output is large (about
400 MB with world geometry and collision). The CIA is the better option for
installing on a 3DS.

### Running

- **3DS (CFW):** copy `nfs_3ds.cia` to the SD card and install it with FBI.
- **Homebrew Launcher:** copy `nfs_3ds.3dsx` and launch it from the Homebrew
  Launcher. Expect a long first copy/launch because of the embedded world.

## World asset pipeline

The generated `romfs/assets/` data is checked in, so a normal build does not
need the retail game. Rebuilding the world requires the PC game files and
Python 3 with Pillow and numpy. The tools find common Steam install locations
automatically; for a custom location, set `NFS_MW_DIR` to either the game's
`HAWAII` folder or its parent folder before running the asset tools.

```sh
export NFS_MW_DIR="/path/to/Need for Speed(TM) Most Wanted/HAWAII"
```

```sh
python3 tools/world_textures.py         # material -> albedo map (JSON)
python3 tools/extract_world_sectors.py  # mesh world + world_manifest.bin
python3 tools/build_collision.py        # per-sector .mwcol colliders
python3 tools/build_atlas.py            # rebuild the atlas PNG alone
make -j8                                # compress atlas, embed RomFS, link
```

- `tools/extract_world_sectors.py` reads the eight arenas, connecting
  districts and placed `TRK_UNIT` set pieces, dedupes LOD copies, splits long
  roads by triangle centroid, and writes 500 m sectors plus the manifest.
- `tools/build_collision.py` bakes the ground grid and wall segments from the
  resulting `.mwmesh` files. Run it after every extraction.
- `tools/extract_world.py` is the single-arena reference extractor;
  `tools/yard_props.py` holds the hand-authored container-yard layout.

## Project layout

```
src/core/       entry point, BNDL reader, tinf decompressor
src/renderer/   world streaming, collision, HUD, car and water rendering
src/physics/    vehicle simulation
src/input/      button mapping
tools/          host-side extraction, atlas and collision pipelines
romfs/          generated runtime assets (world, textures, sectors)
meta/           CIA metadata and New 3DS resource request
```

## Status and limitations

- The world geometry, textures, streaming and collision are in place. The car
  is still procedural geometry: the retail Porsche BNDL mesh is not decoded.
- Container stacks and gantry cranes are hand-placed approximations, because
  the original harbour prop transforms are not part of the extracted arena.
- The wall index covers geometry within about 128 m outside each sector's
  500 m cell; extreme overhangs can fall outside it.
- Ground height is a 2 m grid with the lowest and highest surface per cell, so
  a cell crossed by more than two stacked surfaces only stores the extremes.
- Street-name HUD labels are not wired up: the localized name bundle was not
  present in the available game install.
- Collision was validated offline by linking the real vehicle physics against
  all 252 baked colliders: the spawn lane is clear for 142 m, a full-throttle
  wall stop settles at the 1.1 m body radius, and reverse pulls the car back
  out.
- Verified in Azahar (`nfs_3ds.3dsx`): the spawn road, city blocks, and HUD
  render immediately at ~60 FPS (`59.9 FPS`). Real hardware frame rate and
  New 3DS 804 MHz speedup behavior remain to be tested on console.
