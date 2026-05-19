#!/usr/bin/env python3
"""
One-shot decomposer: scenes/apartment/apt.json → JS skeleton + texture files.

Run from DimSim repo root:  python3 scripts/decompose_apt.py

Produces:
  scenes/apartment/index.js       — entry: applies sky, calls shell/furniture/lights
  scenes/apartment/shell.js       — building shell (80 top-level primitives + colliders)
  scenes/apartment/furniture.js   — 87 asset instances (each with nested primitives)
  scenes/apartment/lights.js      — 11 point lights
  scenes/apartment/textures/      — 16 unique extracted images (avif/jpg/png)

apt.json afterwards is dead and should be deleted.
"""

import base64
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "scenes" / "apartment"
TEX_DIR = OUT / "textures"


def sniff_format(raw: bytes) -> str:
    if raw[:4] == b"\x89PNG":
        return "png"
    if raw[:2] == b"\xff\xd8":
        return "jpg"
    if b"ftypavif" in raw[:32] or b"ftypmif1" in raw[:32]:
        return "avif"
    if b"ftypheic" in raw[:32] or b"ftypheix" in raw[:32]:
        return "heic"
    if raw[:6] in (b"GIF87a", b"GIF89a"):
        return "gif"
    if raw[:4] == b"RIFF" and raw[8:12] == b"WEBP":
        return "webp"
    return "bin"


def extract_textures(doc: dict) -> dict[str, str]:
    """Walk all materials, write unique textures to disk, return data-url → filename map."""
    TEX_DIR.mkdir(parents=True, exist_ok=True)
    seen: dict[str, str] = {}  # raw bytes hash → filename

    def collect(node):
        if isinstance(node, dict):
            mat = node.get("material")
            if isinstance(mat, dict):
                url = mat.get("textureDataUrl")
                if url:
                    m = re.match(r"^data:([^;]+);base64,(.+)$", url)
                    if m:
                        raw = base64.b64decode(m.group(2))
                        h = hashlib.sha256(raw).hexdigest()[:12]
                        if h not in seen:
                            ext = sniff_format(raw)
                            name = f"{h}.{ext}"
                            (TEX_DIR / name).write_bytes(raw)
                            seen[h] = name
                        # replace inline url with filename ref for emit step
                        mat["_texFile"] = seen[h]
                    mat.pop("textureDataUrl", None)
            for v in node.values():
                collect(v)
        elif isinstance(node, list):
            for v in node:
                collect(v)

    collect(doc)
    print(f"  Extracted {len(seen)} unique textures to {TEX_DIR.relative_to(ROOT)}/")
    return seen


# ── JS emit ──────────────────────────────────────────────────────────────────


def js_hex(color: str | None) -> str:
    if not color or not color.startswith("#"):
        return "0xffffff"
    return f"0x{color[1:].lower()}"


def js_num(x: float, ndigits: int = 4) -> str:
    """Compact number repr — strip trailing zeros."""
    if x == int(x):
        return str(int(x))
    s = f"{x:.{ndigits}f}".rstrip("0").rstrip(".")
    return s if s else "0"


def emit_material(mat: dict) -> str:
    """Emit a JS object literal for material opts passed to the primitive helper.
    Mirrors the property mapping in engine.js createPrimitiveMaterial."""
    parts = []
    parts.append(f"color:{js_hex(mat.get('color'))}")
    # engine: roughness ← softness (fallback to roughness, default 0.7)
    rough = mat.get("softness", mat.get("roughness", 0.7))
    if rough != 0.7:
        parts.append(f"r:{js_num(rough, 3)}")
    if (m := mat.get("metalness", 0)) != 0:
        parts.append(f"m:{js_num(m, 3)}")
    if (s := mat.get("specularIntensity", 1)) != 1:
        parts.append(f"si:{js_num(s, 3)}")
    if (sc := mat.get("specularColor")) and sc != "#ffffff":
        parts.append(f"sc:{js_hex(sc)}")
    if (ei_v := mat.get("envMapIntensity", 1)) != 1:
        parts.append(f"emi:{js_num(ei_v, 3)}")
    # engine: clearcoat ← max(clearcoat, hardness*0.85)
    hard = mat.get("hardness", 0)
    cc = max(mat.get("clearcoat", 0), hard * 0.85)
    if cc > 0:
        parts.append(f"cc:{js_num(cc, 3)}")
        ccr = min(mat.get("clearcoatRoughness", 0), 1 - hard * 0.8)
        if ccr > 0:
            parts.append(f"ccr:{js_num(ccr, 3)}")
    # engine: sheen ← fluffiness
    fluff = mat.get("fluffiness", 0)
    if fluff > 0:
        parts.append(f"sh:{js_num(fluff, 3)}")
    if (tr := mat.get("transmission", 0)) > 0:
        parts.append(f"tr:{js_num(tr, 3)}")
        if (ior := mat.get("ior", 1.45)) != 1.45:
            parts.append(f"ior:{js_num(ior, 3)}")
        if (thk := mat.get("thickness", 0)) > 0:
            parts.append(f"thk:{js_num(thk, 3)}")
    if (iri := mat.get("iridescence", 0)) > 0:
        parts.append(f"iri:{js_num(iri, 3)}")
    if (em := mat.get("emissive")) and em != "#000000":
        parts.append(f"e:{js_hex(em)}")
        if (ei := mat.get("emissiveIntensity", 1)) != 1:
            parts.append(f"ei:{js_num(ei, 3)}")
    if (op := mat.get("opacity", 1)) != 1:
        parts.append(f"op:{js_num(op, 3)}")
    if mat.get("doubleSided"):
        parts.append("ds:1")
    if (tex := mat.get("_texFile")):
        parts.append(f"t:'{tex}'")
        uv = mat.get("uvTransform") or {}
        uv_parts = []
        for k_in, k_out in [("repeatX", "rx"), ("repeatY", "ry"),
                            ("offsetX", "ox"), ("offsetY", "oy"),
                            ("rotationDeg", "rot")]:
            v = uv.get(k_in)
            if v not in (None, 0, 1):
                # repeat defaults to 1; offset/rotation default to 0 — only emit non-default
                if (k_in.startswith("repeat") and v != 1) or (not k_in.startswith("repeat") and v != 0):
                    uv_parts.append(f"{k_out}:{js_num(v, 3)}")
        if uv_parts:
            parts.append("uv:{" + ",".join(uv_parts) + "}")
    return "{" + ", ".join(parts) + "}"


def emit_transform(tr: dict) -> str:
    """Position/rotation/scale as compact array tuples."""
    pos = tr.get("position", {})
    rot = tr.get("rotation", {})
    scale = tr.get("scale", {})
    p = [js_num(pos.get(k, 0)) for k in "xyz"]
    parts = [f"p:[{','.join(p)}]"]
    if any(rot.get(k, 0) != 0 for k in "xyz"):
        r = [js_num(rot.get(k, 0)) for k in "xyz"]
        parts.append(f"r:[{','.join(r)}]")
    if any(scale.get(k, 1) != 1 for k in "xyz"):
        s = [js_num(scale.get(k, 1)) for k in "xyz"]
        parts.append(f"s:[{','.join(s)}]")
    return "{" + ", ".join(parts) + "}"


def emit_primitive(prim: dict) -> str:
    """Emit one primitive() call as a JS data tuple."""
    t = prim.get("type", "box")
    dims = prim.get("dimensions", {})
    if t == "box":
        d = [js_num(dims.get(k, 1)) for k in ("width", "height", "depth")]
        dim_str = f"[{','.join(d)}]"
    elif t == "sphere":
        dim_str = f"[{js_num(dims.get('radius', 0.5))}]"
    elif t == "cylinder":
        dim_str = f"[{js_num(dims.get('radiusTop', dims.get('radius', 0.5)))},{js_num(dims.get('radiusBottom', dims.get('radius', 0.5)))},{js_num(dims.get('height', 1))}]"
    elif t == "plane":
        d = [js_num(dims.get(k, 1)) for k in ("width", "height")]
        dim_str = f"[{','.join(d)}]"
    elif t == "cone":
        dim_str = f"[{js_num(dims.get('radius', 0.5))},{js_num(dims.get('height', 1))}]"
    elif t == "torus":
        dim_str = f"[{js_num(dims.get('radius', 0.5))},{js_num(dims.get('tube', 0.2))}]"
    else:
        dim_str = "[]"
    tr = emit_transform(prim.get("transform", {}))
    mat = emit_material(prim.get("material", {}))
    return f"['{t}',{dim_str},{tr},{mat}]"


def emit_structure(primitives: list, out_path: Path) -> None:
    lines = [
        "// Static geometry — walls, floor, ceiling, fixtures.",
        "",
        "import { addPrimitives } from './_prim.js';",
        "",
        "const STRUCTURE = [",
    ]
    for p in primitives:
        lines.append(f"  {emit_primitive(p)},")
    lines += [
        "];",
        "",
        "export function buildStructure(scene, THREE, physics) {",
        "  addPrimitives(scene, THREE, physics, STRUCTURE, { staticColliders: true });",
        "}",
        "",
    ]
    out_path.write_text("\n".join(lines))


def emit_objects(assets: list, out_path: Path) -> None:
    """Each asset = a Group at the asset's transform, containing its state's primitives."""
    items = []
    for a in assets:
        if a.get("_deltaOnly"):
            continue
        states = a.get("states", [])
        if not states:
            continue
        current_id = a.get("currentStateId", "state-default")
        state = next((s for s in states if s.get("id") == current_id), states[0])
        scene_data = state.get("scene", {})
        prims = scene_data.get("primitives", [])
        if not prims:
            continue
        items.append((a.get("title", "object"), a.get("transform", {}), prims))

    lines = [
        "// Code-built placed items.  Each entry is a Group of primitives + a transform.",
        "// Future direction: migrate to GLB files loaded via loadGLTF().",
        "",
        "import { addObjectGroup } from './_prim.js';",
        "",
        "const OBJECTS = [",
    ]
    for title, tr, prims in items:
        title_js = json.dumps(title)
        lines.append(f"  {{ title: {title_js}, t: {emit_transform(tr)}, prims: [")
        for p in prims:
            lines.append(f"    {emit_primitive(p)},")
        lines.append("  ] },")
    lines += [
        "];",
        "",
        "export function buildObjects(scene, THREE, physics) {",
        "  for (const item of OBJECTS) {",
        "    addObjectGroup(scene, THREE, physics, item);",
        "  }",
        "}",
        "",
    ]
    out_path.write_text("\n".join(lines))


def emit_lights(lights: list, out_path: Path) -> None:
    lines = [
        "// Auto-generated from apt.json — lights.",
        "",
        "export function buildLights(scene, THREE) {",
    ]
    for i, light in enumerate(lights):
        t = light.get("type", "point")
        color = js_hex(light.get("color"))
        intensity = js_num(light.get("intensity", 1))
        pos = light.get("position", {})
        if t == "point":
            dist = js_num(light.get("distance", 0))
            lines.append(
                f"  {{ const l = new THREE.PointLight({color}, {intensity}, {dist});"
                f" l.position.set({js_num(pos.get('x', 0))}, {js_num(pos.get('y', 0))}, {js_num(pos.get('z', 0))});"
                f" l.castShadow = {str(bool(light.get('castShadow'))).lower()}; scene.add(l); }}"
            )
        elif t == "directional":
            lines.append(
                f"  {{ const l = new THREE.DirectionalLight({color}, {intensity});"
                f" l.position.set({js_num(pos.get('x', 0))}, {js_num(pos.get('y', 0))}, {js_num(pos.get('z', 0))});"
                f" l.castShadow = {str(bool(light.get('castShadow'))).lower()}; scene.add(l); }}"
            )
        elif t == "ambient":
            lines.append(f"  scene.add(new THREE.AmbientLight({color}, {intensity}));")
        elif t == "hemisphere":
            ground = js_hex(light.get("groundColor"))
            lines.append(
                f"  {{ const l = new THREE.HemisphereLight({color}, {ground}, {intensity});"
                f" l.position.set({js_num(pos.get('x', 0))}, {js_num(pos.get('y', 0))}, {js_num(pos.get('z', 0))}); scene.add(l); }}"
            )
    lines += ["}", ""]
    out_path.write_text("\n".join(lines))


def emit_prim_helper(out_path: Path) -> None:
    """The shared primitive-construction helper used by shell.js + furniture.js."""
    content = """// Shared primitive constructor + texture cache.  Used by shell.js + furniture.js.
//
// Data tuple shape:  ['box'|'sphere'|'cylinder'|'plane'|'cone'|'torus', dims[], transform, material]
//   dims: box [w,h,d], sphere [r], cylinder [rTop,rBot,h], plane [w,h], cone [r,h], torus [r,tube]
//   transform: { p:[x,y,z], r?:[rx,ry,rz], s?:[sx,sy,sz] }
//   material: { color, r?, m?, e?, ei?, t?, op? }   // t = texture filename (./textures/<t>)

const _texCache = new Map();
const _texBase = new URL('./textures/', import.meta.url).toString();

function loadTex(name, THREE) {
  if (!_texCache.has(name)) {
    const tex = new THREE.TextureLoader().load(_texBase + name);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    _texCache.set(name, tex);
  }
  return _texCache.get(name);
}

function makeMaterial(THREE, mat) {
  const opts = { color: mat.color ?? 0xffffff };
  if (mat.r != null) opts.roughness = mat.r;
  if (mat.m != null) opts.metalness = mat.m;
  if (mat.e != null) opts.emissive = mat.e;
  if (mat.ei != null) opts.emissiveIntensity = mat.ei;
  if (mat.op != null) { opts.opacity = mat.op; opts.transparent = mat.op < 1; }
  if (mat.t) opts.map = loadTex(mat.t, THREE);
  return new THREE.MeshStandardMaterial(opts);
}

function makeGeometry(THREE, type, dims) {
  switch (type) {
    case 'box':      return new THREE.BoxGeometry(dims[0], dims[1], dims[2]);
    case 'sphere':   return new THREE.SphereGeometry(dims[0], 24, 24);
    case 'cylinder': return new THREE.CylinderGeometry(dims[0], dims[1], dims[2], 24);
    case 'plane':    return new THREE.PlaneGeometry(dims[0], dims[1]);
    case 'cone':     return new THREE.ConeGeometry(dims[0], dims[1], 24);
    case 'torus':    return new THREE.TorusGeometry(dims[0], dims[1], 12, 24);
    default:         return new THREE.BoxGeometry(1, 1, 1);
  }
}

function applyTransform(obj, tr) {
  if (tr.p) obj.position.set(tr.p[0], tr.p[1], tr.p[2]);
  if (tr.r) obj.rotation.set(tr.r[0], tr.r[1], tr.r[2]);
  if (tr.s) obj.scale.set(tr.s[0], tr.s[1], tr.s[2]);
}

export function makePrimitive(THREE, tuple) {
  const [type, dims, tr, mat] = tuple;
  const mesh = new THREE.Mesh(makeGeometry(THREE, type, dims), makeMaterial(THREE, mat));
  applyTransform(mesh, tr);
  mesh.castShadow = mesh.receiveShadow = true;
  return mesh;
}

export function addPrimitives(scene, THREE, physics, tuples, opts = {}) {
  for (const t of tuples) {
    const m = makePrimitive(THREE, t);
    scene.add(m);
    if (opts.staticColliders) {
      const shape = t[0] === 'sphere' ? 'sphere' : (t[0] === 'plane' ? 'box' : t[0] === 'box' ? 'box' : 'trimesh');
      physics.staticCollider(m, shape);
    }
  }
}

export function addObjectGroup(scene, THREE, physics, item) {
  const g = new THREE.Group();
  applyTransform(g, item.t);
  for (const t of item.prims) g.add(makePrimitive(THREE, t));
  scene.add(g);
  physics.staticCollider(g, 'trimesh');
}
"""
    out_path.write_text(content)


def emit_index(scene_settings: dict, out_path: Path, spawn=(2, 0.5, 3)) -> None:
    sky = scene_settings.get("sky", {})
    sky_obj = {
        k: v
        for k, v in sky.items()
        if k in ("topColor", "horizonColor", "bottomColor", "brightness", "softness", "sunStrength", "sunHeight")
    }
    sky_js = json.dumps(sky_obj, indent=2).replace("\n", "\n  ")
    content = f"""// scenes/apartment/index.js — entry point for the apartment scene.

const SKY = {sky_js};

export default async function build({{ scene, THREE, physics, setSky }}) {{
  const ts = `?t=${{Date.now()}}`;
  const [structure, objects, lights] = await Promise.all([
    import(/* @vite-ignore */ './structure.js' + ts),
    import(/* @vite-ignore */ './objects.js' + ts),
    import(/* @vite-ignore */ './lights.js' + ts),
  ]);

  if (setSky) setSky(SKY);
  structure.buildStructure(scene, THREE, physics);
  objects.buildObjects(scene, THREE, physics);
  lights.buildLights(scene, THREE);

  return {{
    embodiment: null,
    spawnPoint: {{ x: {js_num(spawn[0])}, y: {js_num(spawn[1])}, z: {js_num(spawn[2])} }},
  }};
}}
"""
    out_path.write_text(content)


# ── Main ─────────────────────────────────────────────────────────────────────


def main(apt_path: str = None) -> None:
    src = Path(apt_path) if apt_path else (OUT / "apt.json")
    if not src.exists():
        print(f"FATAL: {src} not found")
        return

    print(f"Reading {src} ({src.stat().st_size / 1e6:.1f} MB)...")
    with src.open() as f:
        doc = json.load(f)

    print("Extracting textures...")
    extract_textures(doc)

    print(f"Emitting structure.js ({len(doc['primitives'])} primitives)...")
    emit_structure(doc["primitives"], OUT / "structure.js")

    print(f"Emitting objects.js ({len(doc['assets'])} assets, filtering _deltaOnly)...")
    emit_objects(doc["assets"], OUT / "objects.js")

    print(f"Emitting lights.js ({len(doc['lights'])} lights)...")
    emit_lights(doc["lights"], OUT / "lights.js")

    emit_prim_helper(OUT / "_prim.js")
    emit_index(doc.get("sceneSettings", {}), OUT / "index.js")

    print("\nDone.  Files in scenes/apartment/:")
    for p in sorted(OUT.iterdir()):
        if p.is_file():
            print(f"  {p.name:30s}  {p.stat().st_size / 1024:8.1f} KB")
        elif p.is_dir():
            tot = sum(f.stat().st_size for f in p.iterdir())
            n = sum(1 for _ in p.iterdir())
            print(f"  {p.name}/  ({n} files, {tot / 1024:.1f} KB)")


if __name__ == "__main__":
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else None)
