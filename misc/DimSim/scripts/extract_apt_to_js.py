#!/usr/bin/env python3
"""
Extract apt.json → JS modules of apt-shape data (full parity with loadJson).

Run from DimSim repo root:  python3 scripts/extract_apt_to_js.py

Reads:   scenes/apartment/apt.json
Writes:  scenes/apartment/textures/<hash>.<ext>
         scenes/apartment/data/sky.js
         scenes/apartment/data/tags.js
         scenes/apartment/data/groups.js
         scenes/apartment/data/lights.js
         scenes/apartment/data/structure.js  (top-level primitives)
         scenes/apartment/data/objects.js    (assets with full state lists)

Then scenes/apartment/index.js assembles these + sceneApi.loadLevel() to feed
the engine's importLevelFromJSON — same code path as loadJson, so pickables,
multi-state objects, TV toggle etc. work exactly as before.

Texture data URLs are extracted to disk and the materials are rewritten with
`texturePath: '<hash>.<ext>'`.  loadLevel resolves these to absolute URLs
against the scene base before import.
"""

import base64
import hashlib
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE_DIR = ROOT / "scenes" / "apartment"
TEX_DIR = SCENE_DIR / "textures"
DATA_DIR = SCENE_DIR / "data"


def sniff_format(raw: bytes) -> str:
    if raw[:4] == b"\x89PNG": return "png"
    if raw[:2] == b"\xff\xd8": return "jpg"
    if b"ftypavif" in raw[:32] or b"ftypmif1" in raw[:32]: return "avif"
    if b"ftypheic" in raw[:32] or b"ftypheix" in raw[:32]: return "heic"
    if raw[:6] in (b"GIF87a", b"GIF89a"): return "gif"
    if raw[:4] == b"RIFF" and raw[8:12] == b"WEBP": return "webp"
    return "bin"


def extract_textures(node):
    """Walk every dict.  Replace material.textureDataUrl (data: URL) with
    material.texturePath (filename only).  Write unique textures to disk."""
    if isinstance(node, dict):
        mat = node.get("material")
        if isinstance(mat, dict):
            url = mat.get("textureDataUrl")
            if isinstance(url, str):
                m = re.match(r"^data:([^;]+);base64,(.+)$", url)
                if m:
                    raw = base64.b64decode(m.group(2))
                    h = hashlib.sha256(raw).hexdigest()[:12]
                    ext = sniff_format(raw)
                    name = f"{h}.{ext}"
                    path = TEX_DIR / name
                    if not path.exists():
                        path.write_bytes(raw)
                    mat["texturePath"] = f"textures/{name}"
                    del mat["textureDataUrl"]
        for v in node.values():
            extract_textures(v)
    elif isinstance(node, list):
        for v in node:
            extract_textures(v)


# ── JS emit ──────────────────────────────────────────────────────────────────


def to_js_literal(obj, indent=0):
    """Convert Python obj → JS object/array literal source.  Booleans/null
    map to JS spellings; everything else passes through json.dumps."""
    # json.dumps already produces valid JS for objects/arrays/strings/numbers.
    # Booleans (true/false) and null are spelled the same in JSON and JS.
    return json.dumps(obj, indent=2 if indent else None, separators=(", ", ": "))


def emit_module(name: str, value, out_path: Path, note: str = "") -> None:
    src = to_js_literal(value, indent=2)
    header = f"// {note}\n" if note else ""
    out_path.write_text(f"{header}export const {name} = {src};\n")


def emit_index(spawn=(2, 0.5, 3)) -> None:
    content = f"""// scenes/apartment/index.js — entry for the apartment scene.
//
// JS-authored level data (apt-shape) is fed through importLevelFromJSON via
// scene-api.loadLevel().  That registers all assets in the engine's
// interaction registry, so E-key pickups, door states, and the TV toggle
// work exactly as they did when the scene loaded from apt.json directly.

import {{ SKY }} from './data/sky.js';
import {{ TAGS }} from './data/tags.js';
import {{ GROUPS }} from './data/groups.js';
import {{ LIGHTS }} from './data/lights.js';
import {{ PRIMITIVES }} from './data/structure.js';
import {{ ASSETS }} from './data/objects.js';

export default async function build({{ loadLevel }}) {{
  await loadLevel({{
    version: '2.0',
    worldKey: 'default',
    tags: TAGS,
    primitives: PRIMITIVES,
    assets: ASSETS,
    lights: LIGHTS,
    groups: GROUPS,
    sceneSettings: {{ sky: SKY }},
  }});

  return {{
    embodiment: null,
    spawnPoint: {{ x: {spawn[0]}, y: {spawn[1]}, z: {spawn[2]} }},
  }};
}}
"""
    (SCENE_DIR / "index.js").write_text(content)


# ── Main ─────────────────────────────────────────────────────────────────────


def main(apt_path: str | None = None) -> None:
    src = Path(apt_path) if apt_path else (SCENE_DIR / "apt.json")
    if not src.exists():
        print(f"FATAL: {src} not found")
        sys.exit(1)

    TEX_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Reading {src} ({src.stat().st_size / 1e6:.1f} MB)...")
    doc = json.loads(src.read_text())

    print("Extracting textures + rewriting material.textureDataUrl → texturePath...")
    extract_textures(doc)
    n_tex = sum(1 for _ in TEX_DIR.iterdir())
    print(f"  {n_tex} files in textures/")

    sky = (doc.get("sceneSettings") or {}).get("sky", {})
    tags = doc.get("tags", [])
    groups = doc.get("groups", [])
    lights = doc.get("lights", [])
    prims = doc.get("primitives", [])
    assets = doc.get("assets", [])

    print(f"Emitting data/sky.js, data/tags.js, data/groups.js...")
    emit_module("SKY", sky, DATA_DIR / "sky.js", note="Sky / atmosphere settings.")
    emit_module("TAGS", tags, DATA_DIR / "tags.js", note="World tag list (string filter tags).")
    emit_module("GROUPS", groups, DATA_DIR / "groups.js", note="Editor groupings.")
    print(f"Emitting data/lights.js ({len(lights)} lights)...")
    emit_module("LIGHTS", lights, DATA_DIR / "lights.js", note="Scene lights.")
    print(f"Emitting data/structure.js ({len(prims)} top-level primitives)...")
    emit_module("PRIMITIVES", prims, DATA_DIR / "structure.js",
                note="Static geometry (walls, floor, ceiling, fixtures).")
    print(f"Emitting data/objects.js ({len(assets)} assets, incl. _deltaOnly)...")
    emit_module("ASSETS", assets, DATA_DIR / "objects.js",
                note="Placed assets with full state lists (interactive parity with apt.json).")

    print("Emitting index.js...")
    emit_index()

    print("\nDone.  scenes/apartment/:")
    for p in sorted(SCENE_DIR.iterdir()):
        if p.is_file():
            print(f"  {p.name:30s}  {p.stat().st_size / 1024:8.1f} KB")
        elif p.is_dir():
            sub = sorted(p.iterdir())
            tot = sum(f.stat().st_size for f in sub if f.is_file())
            print(f"  {p.name}/  ({len(sub)} files, {tot / 1024:.1f} KB)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
