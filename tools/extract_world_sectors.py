#!/usr/bin/env python3
"""Extract the COMPLETE Fairhaven open world from MW2012 .BNDL files
(all 8 ARENAs + all 169 TRK_UNITs + all DISTRICTs) into streamable spatial
sectors (.mwmesh v3) and a master manifest.

Each sector is an NxN metre spatial block in mesh-local coordinates (centred on
origin = (-168.0, 30.0, 4058.0)).  At runtime on the 3DS, sectors are
dynamically loaded when the car approaches and unloaded when out of range.
"""
import os
import sys
import struct
import math
import glob
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bndl import Bundle
from mwmesh import parse_renderable, extract_mesh
import build_atlas
from extract_world import (
    MW_DIR, VERTEX_SIZE, dedupe_lod_resources, build_indexed_chunk,
    weld_chunks, write_mwmesh, make_yard_decorator
)

ORIGIN = (-168.0, 30.0, 4058.0)
SECTOR_SIZE = 500.0
MAX_VERTS_PER_CHUNK = 30000


def collect_sector_parts(bundle_paths, origin, slot_of):
    """Collect renderables from all bundles and assign to grid sectors."""
    ox, oy, oz = origin
    seen_ids = set()

    # Step 1: Scan and group mesh candidates by spatial sector
    print(f"Scanning {len(bundle_paths)} bundles across the full open world...")
    sector_meshes = {}
    total_scanned = 0
    t0 = time.time()

    for bi, bpath in enumerate(bundle_paths):
        bname = os.path.basename(bpath)
        b = Bundle(bpath)
        rends = [r for r in b.resources if r.type_id == 5 and r.has_stream1]
        b_accepted = 0

        for r in rends:
            if r.id in seen_ids:
                continue
            p = parse_renderable(r, b)
            if not p:
                continue
            cx, cy, cz = p["centre"]
            rad = p["radius"]
            # Skip origin-centered local props and extreme sky/ocean shells.
            # Long highway/interchange renderables can have 600-1500 m radii;
            # keep those and let triangle edge filtering remove decoder sails.
            if (abs(cx) <= 50.0 and abs(cz) <= 50.0) or rad >= 5000.0:
                continue

            m = extract_mesh(r, b)
            if not m:
                continue

            seen_ids.add(r.id)
            total_scanned += 1
            b_accepted += 1

            lx = cx - ox
            lz = cz - oz
            gx = int(math.floor(lx / SECTOR_SIZE))
            gz = int(math.floor(lz / SECTOR_SIZE))
            key = (gx, gz)
            sector_meshes.setdefault(key, []).append(m)

        if (bi + 1) % 25 == 0 or (bi + 1) == len(bundle_paths):
            print(f"  [{bi+1:3d}/{len(bundle_paths):3d}] bundles scanned ({total_scanned} world meshes in {len(sector_meshes)} sectors, {time.time()-t0:.1f}s)")

    print(f"\nDeduplicating LOD clusters across {len(sector_meshes)} spatial sectors...")
    sectors = {}
    total_parts = 0
    total_deduped_meshes = 0

    for key, m_list in sector_meshes.items():
        kept = dedupe_lod_resources(m_list)
        total_deduped_meshes += len(kept)
        for m in kept:
            for p in m["parts"]:
                if p["tris"]:
                    sectors.setdefault(key, []).append(p)
                    total_parts += 1

    print(f"LOD deduplication complete: {total_scanned} -> {total_deduped_meshes} meshes (kept {total_parts} parts across {len(sectors)} sectors)")
    return sectors


def build_and_save_sectors(sectors, origin, slot_of, out_dir):
    """Build indexed chunks for each sector, write .mwmesh files and master manifest."""
    os.makedirs(out_dir, exist_ok=True)
    ox, oy, oz = origin
    yard_dec = make_yard_decorator(origin)

    sector_records = []
    tot_sectors = len(sectors)
    total_verts = 0
    total_indices = 0
    total_bytes = 0

    print(f"\nProcessing and welding geometry for {tot_sectors} sectors...")
    sorted_keys = sorted(sectors.keys(), key=lambda k: (k[0], k[1]))

    for idx, (gx, gz) in enumerate(sorted_keys):
        parts = sectors[(gx, gz)]
        # If this sector covers the container harbour yard around (-176, -139), apply yard decorator
        if gx == -1 and gz == -1:
            print(f"  Applying yard dressing to Harbour Sector ({gx}, {gz})...")
            extra = yard_dec(parts)
            parts = list(parts) + extra

        # Pack into chunks with <= MAX_VERTS_PER_CHUNK
        chunks = []
        cur_parts = []
        cur_verts = 0
        for p in parts:
            nv = len(p["verts"])
            if cur_parts and cur_verts + nv > MAX_VERTS_PER_CHUNK:
                ch = build_indexed_chunk(cur_parts, ox, oy, oz, slot_of)
                if ch["indices"]:
                    chunks.append(ch)
                cur_parts = []
                cur_verts = 0
            cur_parts.append(p)
            cur_verts += nv

        if cur_parts:
            ch = build_indexed_chunk(cur_parts, ox, oy, oz, slot_of)
            if ch["indices"]:
                chunks.append(ch)

        if not chunks:
            continue

        # Weld vertices and drop degenerate triangles
        chunks = weld_chunks(chunks)
        if not chunks:
            continue

        fname = f"sec_{gx}_{gz}.mwmesh"
        fpath = os.path.join(out_dir, fname)
        write_mwmesh(chunks, fpath)

        s_verts = sum(len(c["verts"]) for c in chunks)
        s_indices = sum(len(c["indices"]) for c in chunks)
        s_bytes = os.path.getsize(fpath)
        total_verts += s_verts
        total_indices += s_indices
        total_bytes += s_bytes

        s_bbox_min = [min(c["bbox_min"][k] for c in chunks) for k in range(3)]
        s_bbox_max = [max(c["bbox_max"][k] for c in chunks) for k in range(3)]

        sector_records.append({
            "gx": gx,
            "gz": gz,
            "bbox_min": s_bbox_min,
            "bbox_max": s_bbox_max,
            "num_chunks": len(chunks),
            "num_verts": s_verts,
            "num_indices": s_indices,
            "filename": fname,
            "filesize": s_bytes,
        })
        if (idx + 1) % 10 == 0 or (idx + 1) == tot_sectors:
            print(f"  [{idx+1:2d}/{tot_sectors:2d}] Sector ({gx:2d}, {gz:2d}) -> {fname:18s} ({len(chunks)} ch, {s_verts:5d} v, {s_indices//3:5d} tri, {s_bytes/1024:.1f} KB)")

    # Write Master Manifest (binary)
    manifest_path = os.path.join(os.path.dirname(out_dir), "world_manifest.bin")
    with open(manifest_path, "wb") as mf:
        mf.write(struct.pack("<4sIIf3f", b"MWST", 1, len(sector_records), SECTOR_SIZE, ox, oy, oz))
        for r in sector_records:
            fn_bytes = r["filename"].encode("ascii").ljust(32, b"\x00")[:32]
            mf.write(struct.pack("<ii6f3I32s",
                                 r["gx"], r["gz"],
                                 r["bbox_min"][0], r["bbox_min"][1], r["bbox_min"][2],
                                 r["bbox_max"][0], r["bbox_max"][1], r["bbox_max"][2],
                                 r["num_chunks"], r["num_verts"], r["num_indices"],
                                 fn_bytes))

    print(f"\nWrote World Manifest: {manifest_path} ({len(sector_records)} sectors, {os.path.getsize(manifest_path)} bytes)")
    print(f"Summary: {len(sector_records)} sectors, {total_verts} verts, {total_indices//3} triangles, {total_bytes / (1024*1024):.2f} MB on disk")
    return sector_records


def main():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(project_root, "romfs", "assets", "sectors")

    atlas_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_atlas.png")
    atlas_info = build_atlas.build(build_atlas.load_map(), atlas_path)
    slot_of = {int(k, 16): v for k, v in atlas_info["slot"].items()}

    # All ARENAs + all DISTRICTs + all TRK_UNITs
    all_bundles = sorted(
        glob.glob(os.path.join(MW_DIR, "ARENA_*.BNDL")) +
        glob.glob(os.path.join(MW_DIR, "DISTRICT_*.BNDL")) +
        glob.glob(os.path.join(MW_DIR, "TRK_UNIT*.BNDL"))
    )

    sectors = collect_sector_parts(all_bundles, ORIGIN, slot_of)
    records = build_and_save_sectors(sectors, ORIGIN, slot_of, out_dir)


if __name__ == "__main__":
    main()
