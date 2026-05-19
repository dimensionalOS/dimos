#!/usr/bin/env python3
"""
Convert apt.json's 87 furniture assets into individual GLB files + a placements list.

Each apt asset = a Group of primitive sub-meshes + an outer transform.  This script:
  * Builds the asset's local geometry (sub-primitives merged) as a trimesh Scene
  * Exports to scenes/apartment/objects/<slug>.glb
  * Dedupes by (sorted) primitive-hash so repeats share one GLB
  * Emits scenes/apartment/objects.js — a placements list calling loadGLTF

Run:  python3 scripts/decompose_objects_to_glb.py /tmp/apt.json
"""

import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "scenes" / "apartment"
GLB_DIR = OUT / "objects"


# ── Primitive constructors ───────────────────────────────────────────────────


def _srgb_to_linear(c: float) -> float:
    """Convert one sRGB channel (0..1) to linear-light (0..1).  glTF baseColorFactor
    is in linear space; apt.json colors are sRGB hex."""
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def hex_to_linear_rgba(color: str | None, opacity: float = 1.0) -> tuple[float, float, float, float]:
    if not color or not isinstance(color, str) or not color.startswith("#"):
        return (_srgb_to_linear(0.78), _srgb_to_linear(0.78), _srgb_to_linear(0.78), opacity)
    h = color[1:]
    if len(h) == 3:
        h = "".join(c * 2 for c in h)
    r = _srgb_to_linear(int(h[0:2], 16) / 255)
    g = _srgb_to_linear(int(h[2:4], 16) / 255)
    b = _srgb_to_linear(int(h[4:6], 16) / 255)
    return (r, g, b, opacity)


# trimesh creates cylinders/cones along the +Z axis; threejs's CylinderGeometry
# and ConeGeometry are along +Y.  Rotate -π/2 around X to map +Z → +Y so the
# baked geometry matches threejs convention.
_Z_TO_Y = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])


def make_geom(prim: dict) -> trimesh.Trimesh | None:
    t = prim.get("type", "box")
    dims = prim.get("dimensions", {})
    try:
        if t == "box":
            w = dims.get("width", 1)
            h = dims.get("height", 1)
            d = dims.get("depth", 1)
            return trimesh.creation.box(extents=[w, h, d])
        if t == "sphere":
            r = dims.get("radius", 0.5)
            return trimesh.creation.uv_sphere(radius=r, count=[16, 16])
        if t == "cylinder":
            rt = dims.get("radiusTop", dims.get("radius", 0.5))
            rb = dims.get("radiusBottom", dims.get("radius", 0.5))
            h = dims.get("height", 1)
            # trimesh has no native frustum — use average radius for tapered cylinders.
            mesh = trimesh.creation.cylinder(radius=(rt + rb) / 2, height=h, sections=24)
            mesh.apply_transform(_Z_TO_Y)
            return mesh
        if t == "cone":
            r = dims.get("radius", 0.5)
            h = dims.get("height", 1)
            mesh = trimesh.creation.cone(radius=r, height=h, sections=24)
            # trimesh cone: base at z=0, apex at z=h.  threejs ConeGeometry is
            # centered (base at y=-h/2, apex at y=h/2).  Recenter along z first,
            # then rotate +Z → +Y.
            mesh.apply_translation([0, 0, -h / 2])
            mesh.apply_transform(_Z_TO_Y)
            return mesh
        if t == "torus":
            r = dims.get("radius", 0.5)
            tube = dims.get("tube", 0.2)
            # Both trimesh and threejs put torus in the XY plane with axis +Z — no rotation needed.
            return trimesh.creation.torus(major_radius=r, minor_radius=tube, major_sections=24, minor_sections=12)
        if t == "plane":
            w = dims.get("width", 1)
            h = dims.get("height", 1)
            # threejs PlaneGeometry lies in the XY plane (axis +Z).
            verts = np.array([
                [-w / 2, -h / 2, 0],
                [w / 2, -h / 2, 0],
                [w / 2, h / 2, 0],
                [-w / 2, h / 2, 0],
            ])
            faces = np.array([[0, 1, 2], [0, 2, 3]])
            return trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    except Exception as e:
        print(f"  WARN: failed to build {t}: {e}")
        return None
    return None


def apply_transform(mesh: trimesh.Trimesh, tr: dict) -> trimesh.Trimesh:
    pos = tr.get("position", {})
    rot = tr.get("rotation", {})
    scale = tr.get("scale", {})

    sx = scale.get("x", 1)
    sy = scale.get("y", 1)
    sz = scale.get("z", 1)
    if sx != 1 or sy != 1 or sz != 1:
        S = np.diag([sx, sy, sz, 1.0])
        mesh.apply_transform(S)

    rx = rot.get("x", 0)
    ry = rot.get("y", 0)
    rz = rot.get("z", 0)
    if rx or ry or rz:
        # threejs default Euler order is 'XYZ' → matrix M = Rx * Ry * Rz
        Rx = trimesh.transformations.rotation_matrix(rx, [1, 0, 0])
        Ry = trimesh.transformations.rotation_matrix(ry, [0, 1, 0])
        Rz = trimesh.transformations.rotation_matrix(rz, [0, 0, 1])
        mesh.apply_transform(Rx @ Ry @ Rz)

    px = pos.get("x", 0)
    py = pos.get("y", 0)
    pz = pos.get("z", 0)
    if px or py or pz:
        T = trimesh.transformations.translation_matrix([px, py, pz])
        mesh.apply_transform(T)

    return mesh


def apply_material(mesh: trimesh.Trimesh, mat: dict) -> None:
    opacity = float(mat.get("opacity", 1))
    color = hex_to_linear_rgba(mat.get("color"), opacity)
    rough = float(mat.get("roughness", 0.5))
    metal = float(mat.get("metalness", 0))
    em = mat.get("emissive")
    emissive_factor = [0, 0, 0]
    if em and em != "#000000":
        ei = float(mat.get("emissiveIntensity", 1))
        er, eg, eb, _ = hex_to_linear_rgba(em)
        emissive_factor = [er * ei, eg * ei, eb * ei]

    pbr = PBRMaterial(
        baseColorFactor=list(color),
        metallicFactor=metal,
        roughnessFactor=rough,
        emissiveFactor=emissive_factor,
        alphaMode="BLEND" if opacity < 1 else "OPAQUE",
        name="mat",
    )
    mesh.visual = trimesh.visual.TextureVisuals(material=pbr)


def slug(s: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", s.lower()).strip("-") or "object"


# ── Main ─────────────────────────────────────────────────────────────────────


def asset_geometry_hash(prims: list) -> str:
    """Hash the geometry-defining parts of an asset's primitives (no top-level transform).
    Two assets with identical sub-primitives share one GLB."""
    canonical = []
    for p in prims:
        canonical.append({
            "type": p.get("type"),
            "dimensions": p.get("dimensions"),
            "transform": p.get("transform"),
            "material": p.get("material", {}).get("color"),  # color only — ignore textures
        })
    return hashlib.sha256(json.dumps(canonical, sort_keys=True).encode()).hexdigest()[:12]


def main(apt_path: str) -> None:
    with open(apt_path) as f:
        doc = json.load(f)

    GLB_DIR.mkdir(parents=True, exist_ok=True)

    # Group assets by geometry hash so duplicates share one GLB
    seen_hashes: dict[str, str] = {}  # hash → glb_filename
    placements: list[dict] = []
    title_counts: dict[str, int] = {}

    assets = [a for a in doc["assets"] if not a.get("_deltaOnly") and a.get("states")]
    print(f"Processing {len(assets)} assets...")

    for asset in assets:
        current_id = asset.get("currentStateId", "state-default")
        state = next(
            (s for s in asset["states"] if s.get("id") == current_id),
            asset["states"][0],
        )
        prims = state.get("scene", {}).get("primitives", [])
        if not prims:
            continue

        ghash = asset_geometry_hash(prims)
        if ghash not in seen_hashes:
            title = asset.get("title") or asset.get("libraryAssetId") or "object"
            base_slug = slug(title)
            count = title_counts.get(base_slug, 0)
            title_counts[base_slug] = count + 1
            slug_name = base_slug if count == 0 else f"{base_slug}-{count + 1}"
            glb_name = f"{slug_name}.glb"

            built = []
            for prim in prims:
                geom = make_geom(prim)
                if geom is None:
                    continue
                apply_material(geom, prim.get("material", {}))
                apply_transform(geom, prim.get("transform", {}))
                built.append(geom)

            if not built:
                print(f"  SKIP {title} — no geometry")
                continue

            # Match engine's instantiateAsset: recenter around the bbox center.
            # asset.transform.position is where this center lands in world space.
            all_verts = np.concatenate([g.vertices for g in built])
            bbox_center = (all_verts.min(axis=0) + all_verts.max(axis=0)) / 2.0
            recenter = trimesh.transformations.translation_matrix(-bbox_center)
            scene = trimesh.Scene()
            for g in built:
                g.apply_transform(recenter)
                scene.add_geometry(g)

            out_path = GLB_DIR / glb_name
            scene.export(str(out_path))
            seen_hashes[ghash] = glb_name
            print(f"  WROTE {glb_name} ({out_path.stat().st_size / 1024:.1f} KB, {len(built)} parts, recenter={bbox_center.round(3).tolist()})")

        placements.append({
            "file": seen_hashes[ghash],
            "transform": asset.get("transform", {}),
            "title": asset.get("title") or "object",
        })

    # Emit placements file
    emit_placements_js(placements, OUT / "objects.js")
    print(f"\n{len(seen_hashes)} unique GLBs, {len(placements)} placements")
    total_glb_bytes = sum(p.stat().st_size for p in GLB_DIR.iterdir())
    print(f"objects/ total: {total_glb_bytes / 1024:.1f} KB")


def emit_placements_js(placements: list, out_path: Path) -> None:
    """Emit objects.js — placements list that index.js consumes via loadGLTF."""
    def jnum(x: float, ndigits: int = 4) -> str:
        if x == int(x):
            return str(int(x))
        s = f"{x:.{ndigits}f}".rstrip("0").rstrip(".")
        return s if s else "0"

    def emit_transform(tr: dict) -> str:
        pos = tr.get("position", {})
        rot = tr.get("rotation", {})
        scale = tr.get("scale", {})
        parts = [f"p:[{jnum(pos.get(k, 0))}" for k in "xyz"]
        p_arr = "[" + ",".join(jnum(pos.get(k, 0)) for k in "xyz") + "]"
        result = [f"p:{p_arr}"]
        if any(rot.get(k, 0) != 0 for k in "xyz"):
            r_arr = "[" + ",".join(jnum(rot.get(k, 0)) for k in "xyz") + "]"
            result.append(f"r:{r_arr}")
        if any(scale.get(k, 1) != 1 for k in "xyz"):
            s_arr = "[" + ",".join(jnum(scale.get(k, 1)) for k in "xyz") + "]"
            result.append(f"s:{s_arr}")
        return "{" + ", ".join(result) + "}"

    lines = [
        "// Asset placements — pairs each loadable GLB in ./objects/ with a world transform.",
        "// To add furniture: drop a .glb into objects/, append an entry here.",
        "",
        "export const OBJECTS = [",
    ]
    for p in placements:
        title = json.dumps(p["title"])
        lines.append(f"  {{ file: '{p['file']}', t: {emit_transform(p['transform'])}, title: {title} }},")
    lines += ["];", ""]
    out_path.write_text("\n".join(lines))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("apt_path", help="path to apt.json")
    args = ap.parse_args()
    main(args.apt_path)
