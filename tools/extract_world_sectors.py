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
import pickle
import gc
import numpy as np
from collections import OrderedDict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bndl import Bundle
from mwmesh import parse_renderable, extract_mesh
import build_atlas
from game_paths import require_game_files
from extract_world import (
    MW_DIR, VERTEX_SIZE, dedupe_lod_resources, build_indexed_chunk,
    weld_chunks, write_mwmesh, make_yard_decorator
)

ORIGIN = (-168.0, 30.0, 4058.0)
SECTOR_SIZE = 500.0
MAX_VERTS_PER_CHUNK = 30000
CHUNK_SPATIAL_SIZE = 125.0


def collect_sector_parts(bundle_paths, origin, slot_of, spool_dir=None):
    """Collect direct and placed renderables, emitting compact sector parts."""
    ox, oy, oz = origin
    seen_ids = set()
    seen_instances = set()

    # Step 1: Scan and group mesh candidates by spatial sector
    print(f"Scanning {len(bundle_paths)} bundles across the full open world...")
    sectors = {} if spool_dir is None else None
    spool_handles = {}
    sector_keys = set()
    total_scanned = 0
    total_placed = 0
    total_kept = 0
    total_parts = 0
    t0 = time.time()

    for bi, bpath in enumerate(bundle_paths):
        bname = os.path.basename(bpath)
        b = Bundle(bpath)
        bundle_sector_meshes = {}
        by_id = {r.id: r for r in b.resources}
        renderables = {r.id: r for r in b.resources
                       if r.type_id == 5 and r.has_stream1}
        wrappers = {r.id: r for r in b.resources if r.type_id == 81}
        wrapper_geometries = {}
        placed_geometry_ids = set()

        # Type-81 placement wrappers reference one or more type-5 mesh
        # resources. Most TRK_UNIT geometry is stored around (0, 0, 0) and
        # only becomes world geometry after its type-80 transform is applied.
        for wid, wrapper in wrappers.items():
            d0 = b.stream0(wrapper)
            refs = []
            for off in range(0, len(d0) - 7, 8):
                rid = struct.unpack_from("<Q", d0, off)[0]
                if rid in renderables and rid not in refs:
                    refs.append(rid)
            if refs:
                wrapper_geometries[wid] = refs

        mesh_cache = OrderedDict()
        for parent in (r for r in b.resources if r.type_id == 80):
            d0 = b.stream0(parent)
            for off in range(0, len(d0) - 15, 8):
                wid = struct.unpack_from("<Q", d0, off)[0]
                geometry_ids = wrapper_geometries.get(wid)
                if not geometry_ids:
                    continue
                transform_off = struct.unpack_from("<Q", d0, off + 8)[0]
                if (transform_off < 16 or (transform_off - 16) % 96 != 0
                        or transform_off + 96 > len(d0)):
                    continue

                # Type-80 stores three basis vectors and a translation as
                # four float4s beginning 32 bytes into each 96-byte record.
                basis = [struct.unpack_from("<3f", d0, transform_off + k)
                         for k in (32, 48, 64)]
                translation = struct.unpack_from("<3f", d0, transform_off + 80)
                if not all(math.isfinite(v) and abs(v) < 100000.0
                           for row in basis for v in row) or not all(
                               math.isfinite(v) and abs(v) < 100000.0
                               for v in translation):
                    continue
                # Reject corrupt/false placement records. The audited game
                # data has only two transforms above 20x; both expand small
                # prop meshes into giant road-crossing geometry.
                if max(math.sqrt(sum(v * v for v in axis)) for axis in basis) > 20.0:
                    continue

                # Bundles repeat identical placements in streaming copies.
                # Keep one exact geometry/transform combination, while still
                # allowing the same local mesh at different world positions.
                key = (tuple(geometry_ids),
                       tuple(round(v, 4) for row in basis for v in row),
                       tuple(round(v, 3) for v in translation))
                if key in seen_instances:
                    continue
                seen_instances.add(key)
                placed_geometry_ids.update(geometry_ids)

                for rid in geometry_ids:
                    if rid in mesh_cache:
                        template = mesh_cache.pop(rid)
                        mesh_cache[rid] = template
                    else:
                        template = extract_mesh(renderables[rid], b)
                        mesh_cache[rid] = template
                        if len(mesh_cache) > 128:
                            mesh_cache.popitem(last=False)
                    if not template:
                        continue
                    m = transform_mesh(template, basis, translation)
                    if not m:
                        continue
                    cx, _, cz = m["centre"]
                    gx = int(math.floor((cx - ox) / SECTOR_SIZE))
                    gz = int(math.floor((cz - oz) / SECTOR_SIZE))
                    bundle_sector_meshes.setdefault((gx, gz), []).append(m)
                    total_placed += 1

        # Some bundles also contain complete world-space renderables which
        # need no instance transform.
        for r in renderables.values():
            if r.id in placed_geometry_ids:
                continue
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

            lx = cx - ox
            lz = cz - oz
            gx = int(math.floor(lx / SECTOR_SIZE))
            gz = int(math.floor(lz / SECTOR_SIZE))
            key = (gx, gz)
            bundle_sector_meshes.setdefault(key, []).append(m)

        # LOD copies mostly live in the same bundle. Deduplicate them before
        # writing compact triangle partitions to disk, keeping RAM bounded as
        # thousands of instanced set pieces are expanded.
        def emit(sector_key, part):
            sector_keys.add(sector_key)
            if sectors is not None:
                sectors.setdefault(sector_key, []).append(part)
                return
            handle = spool_handles.get(sector_key)
            if handle is None:
                gx, gz = sector_key
                path = os.path.join(spool_dir, f"parts_{gx}_{gz}.pkl")
                handle = open(path, "wb")
                spool_handles[sector_key] = handle
            pickle.dump(part, handle, protocol=4)

        for m_list in bundle_sector_meshes.values():
            kept = dedupe_lod_resources(m_list, verbose=False)
            total_kept += len(kept)
            for mesh in kept:
                for part in mesh["parts"]:
                    if not part["tris"]:
                        continue
                    for target_key, target_part in split_part_by_sector(part, origin):
                        emit(target_key, target_part)
                        total_parts += 1

        del bundle_sector_meshes, mesh_cache, b, by_id, renderables, wrappers
        if (bi + 1) % 25 == 0:
            gc.collect()

        if (bi + 1) % 25 == 0 or (bi + 1) == len(bundle_paths):
            print(f"  [{bi+1:3d}/{len(bundle_paths):3d}] bundles scanned ({total_scanned} direct meshes, {total_placed} placed meshes -> {total_kept} after LOD filter, {len(sector_keys)} output sectors, {time.time()-t0:.1f}s)")

    for handle in spool_handles.values():
        handle.close()
    print(f"Mesh collection complete: {total_scanned} direct meshes, {total_placed} placed meshes -> {total_kept} after LOD filtering, {total_parts} sector parts, {len(sector_keys)} output sectors")
    return sectors


def transform_mesh(mesh, basis, translation):
    """Transform one decoded local mesh into its placed world coordinates using NumPy."""
    # Convert basis to a 3x3 matrix (rows are x, y, z basis vectors)
    basis_mat = np.array(basis, dtype=np.float32)
    trans_vec = np.array(translation, dtype=np.float32)

    parts = []
    all_verts = []
    for part in mesh["parts"]:
        # Convert vertices to a (N, 3) array
        v_arr = np.array(part["verts"], dtype=np.float32)
        # Transform: v' = basis * v + translation
        transformed_v = v_arr @ basis_mat.T + trans_vec
        
        # Convert back to list of tuples for compatibility with the rest of the pipeline
        new_verts = [tuple(v) for v in transformed_v]
        all_verts.extend(new_verts)
        
        transformed_part = dict(part)
        transformed_part["verts"] = new_verts
        parts.append(transformed_part)

    if not all_verts:
        return None

    # Transform centre
    centre_vec = np.array(mesh["centre"], dtype=np.float32)
    new_centre = tuple((centre_vec @ basis_mat.T + trans_vec).tolist())

    # Calculate basis scale
    basis_scale = np.max(np.linalg.norm(basis_mat, axis=1))
    
    all_verts_arr = np.array(all_verts, dtype=np.float32)
    bbox_min = tuple(np.min(all_verts_arr, axis=0).tolist())
    bbox_max = tuple(np.max(all_verts_arr, axis=0).tolist())

    return {
        "centre": new_centre,
        "radius": mesh["radius"] * basis_scale,
        "bbox_min": bbox_min,
        "bbox_max": bbox_max,
        "parts": parts
    }



def split_part_by_sector(part, origin, cell_size=SECTOR_SIZE):
    """Partition a world-space part by triangle centroid into spatial cells using NumPy."""
    ox, _, oz = origin
    verts = np.array(part["verts"], dtype=np.float32)
    uvs = np.array(part["uvs"], dtype=np.float32)
    tris = np.array(part["tris"], dtype=np.int32)

    # Calculate centroids of all triangles: (T, 3)
    # tris is (T, 3), verts is (V, 3). We want the average of vertices for each triangle.
    tri_verts = verts[tris] # (T, 3, 3)
    centroids = np.mean(tri_verts, axis=1) # (T, 3)

    # Calculate grid indices: (T, 2)
    gx = np.floor((centroids[:, 0] - ox) / cell_size).astype(np.int32)
    gz = np.floor((centroids[:, 2] - oz) / cell_size).astype(np.int32)

    grouped = {}
    for i, (g_x, g_z) in enumerate(zip(gx, gz)):
        key = (int(g_x), int(g_z))
        if key not in grouped:
            grouped[key] = {"verts": [], "uvs": [], "tris": [], "vertex_map": {}}
        dst = grouped[key]
        remapped = []
        for src in tris[i]:
            vi = dst["vertex_map"].get(src)
            if vi is None:
                vi = len(dst["verts"])
                dst["vertex_map"][src] = vi
                dst["verts"].append(tuple(verts[src]))
                dst["uvs"].append(tuple(uvs[src]))
            remapped.append(vi)
        dst["tris"].append(tuple(remapped))

    result = []
    for key, data in grouped.items():
        del data["vertex_map"]
        split_part = dict(part)
        split_part.update(data)
        result.append((key, split_part))
    return result


def build_and_save_sectors(sectors, origin, slot_of, out_dir):
    """Build indexed chunks for each sector, write .mwmesh files and master manifest."""
    os.makedirs(out_dir, exist_ok=True)
    ox, oy, oz = origin
    yard_dec = make_yard_decorator(origin)

    sector_records = []
    is_spooled = isinstance(sectors, (str, bytes, os.PathLike))
    if is_spooled:
        spool_dir = os.fspath(sectors)
        spool_paths = glob.glob(os.path.join(spool_dir, "parts_*.pkl"))
        sorted_keys = []
        for path in spool_paths:
            name = os.path.basename(path)[6:-4]
            gx, gz = (int(v) for v in name.split("_", 1))
            sorted_keys.append((gx, gz))
        sorted_keys.sort()
    else:
        sorted_keys = sorted(sectors.keys())
    tot_sectors = len(sorted_keys)
    total_verts = 0
    total_indices = 0
    total_bytes = 0

    print(f"\nProcessing and welding geometry for {tot_sectors} sectors...")
    for idx, (gx, gz) in enumerate(sorted_keys):
        if is_spooled:
            path = os.path.join(spool_dir, f"parts_{gx}_{gz}.pkl")
            parts = []
            with open(path, "rb") as pf:
                while True:
                    try:
                        parts.append(pickle.load(pf))
                    except EOFError:
                        break
        else:
            parts = sectors[(gx, gz)]
        # If this sector covers the container harbour yard around (-176, -139), apply yard decorator
        if gx == -1 and gz == -1:
            print(f"  Applying yard dressing to Harbour Sector ({gx}, {gz})...")
            extra = yard_dec(parts)
            parts = list(parts) + extra

        # Keep each runtime chunk spatially compact. Without this ordering,
        # bundle-order parts from opposite sides of a 500m sector can share a
        # chunk; its broad AABB then defeats runtime distance/frustum culling.
        spatial_parts = []
        for part in parts:
            spatial_parts.extend(split_part_by_sector(part, origin, CHUNK_SPATIAL_SIZE))
        spatial_parts.sort(key=lambda item: item[0])

        # Pack one spatial cell at a time into chunks with <= MAX_VERTS_PER_CHUNK.
        chunks = []
        cur_parts = []
        cur_verts = 0
        cur_cell = None
        for cell, p in spatial_parts:
            nv = len(p["verts"])
            if cur_parts and (cell != cur_cell or cur_verts + nv > MAX_VERTS_PER_CHUNK):
                ch = build_indexed_chunk(cur_parts, ox, oy, oz, slot_of)
                if ch["indices"]:
                    chunks.append(ch)
                cur_parts = []
                cur_verts = 0
            cur_cell = cell
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

    # Old generated sectors are not referenced by the new manifest. Remove
    # them so the embedded RomFS does not carry obsolete duplicate world data.
    referenced = {r["filename"] for r in sector_records}
    for path in glob.glob(os.path.join(out_dir, "sec_*.mwmesh")):
        if os.path.basename(path) not in referenced:
            os.remove(path)

    print(f"\nWrote World Manifest: {manifest_path} ({len(sector_records)} sectors, {os.path.getsize(manifest_path)} bytes)")
    print(f"Summary: {len(sector_records)} sectors, {total_verts} verts, {total_indices//3} triangles, {total_bytes / (1024*1024):.2f} MB on disk")
    return sector_records


def main():
    require_game_files()
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

    import tempfile
    with tempfile.TemporaryDirectory(prefix="mw_world_parts_") as spool_dir:
        collect_sector_parts(all_bundles, ORIGIN, slot_of, spool_dir)
        records = build_and_save_sectors(spool_dir, ORIGIN, slot_of, out_dir)


if __name__ == "__main__":
    main()
