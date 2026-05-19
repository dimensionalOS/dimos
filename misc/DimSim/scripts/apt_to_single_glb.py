#!/usr/bin/env python3
"""
Merge apt.json (the original 97MB blob) into a single GLB so you can open it in
Blender / any GLB viewer for A/B comparison against the decomposed version.

Reuses make_geom / apply_transform / apply_material from decompose_objects_to_glb.

Usage:  python3 scripts/apt_to_single_glb.py <input apt.json> <output.glb>
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))
from decompose_objects_to_glb import (
    apply_material,
    apply_transform,
    make_geom,
)


def main(apt_path: str, out_path: str) -> None:
    with open(apt_path) as f:
        doc = json.load(f)

    scene = trimesh.Scene()
    skipped = 0

    # Top-level primitives (structure: walls/floor)
    for prim in doc.get("primitives", []):
        g = make_geom(prim)
        if g is None:
            skipped += 1
            continue
        apply_material(g, prim.get("material", {}))
        apply_transform(g, prim.get("transform", {}))
        scene.add_geometry(g)

    # Assets (furniture).  Each non-_deltaOnly asset's state has nested primitives.
    # Match engine's instantiateAsset: build the asset's geometry, recenter around its
    # bbox center, then apply the outer asset transform.
    for asset in doc.get("assets", []):
        if asset.get("_deltaOnly"):
            continue
        states = asset.get("states", [])
        if not states:
            continue
        current_id = asset.get("currentStateId", "state-default")
        state = next((s for s in states if s.get("id") == current_id), states[0])
        prims = state.get("scene", {}).get("primitives", [])
        outer_tr = asset.get("transform", {})

        built = []
        for prim in prims:
            g = make_geom(prim)
            if g is None:
                skipped += 1
                continue
            apply_material(g, prim.get("material", {}))
            apply_transform(g, prim.get("transform", {}))
            built.append(g)

        if not built:
            continue

        all_verts = np.concatenate([m.vertices for m in built])
        bbox_center = (all_verts.min(axis=0) + all_verts.max(axis=0)) / 2.0
        recenter = trimesh.transformations.translation_matrix(-bbox_center)
        for m in built:
            m.apply_transform(recenter)
            apply_transform(m, outer_tr)
            scene.add_geometry(m)

    n = len(scene.geometry)
    print(f"Composed {n} meshes ({skipped} skipped)")
    scene.export(out_path)
    size = Path(out_path).stat().st_size
    print(f"Wrote {out_path} ({size / 1024:.1f} KB)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("apt_path")
    ap.add_argument("out_path")
    args = ap.parse_args()
    main(args.apt_path, args.out_path)
