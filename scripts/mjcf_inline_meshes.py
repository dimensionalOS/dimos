#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Write an MJCF's inline meshes out as files, so the USD converter will take it.

Run once by hand, before scripts/mjcf_to_usd.py; see that script's docstring for the rest of
the pipeline.

MuJoCo lets a `<mesh>` carry its geometry inline, as flat `vertex` and `face` arrays with no
`file` at all. Google DeepMind's converter does not: it stops at the first one with

    Python runtime error: Mesh beanbag_1_m0 has no file

and writes nothing. The office_555 scene has 79 of those against 424 that name a file, so the
whole scene is unconvertible until they are on disk.

This writes each one as an .obj beside the others and points the MJCF at it. Nothing is
resampled or decimated -- the vertices and faces go out exactly as MuJoCo would have read them
in, so the converted scene is the same geometry, just reachable.

    python scripts/mjcf_inline_meshes.py --mjcf scene.xml --meshes meshes/ --out ready.xml
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def write_obj(path: Path, vertices: list[float], faces: list[int]) -> None:
    """``vertices`` and ``faces`` as a Wavefront OBJ. OBJ indexes from 1, MuJoCo from 0."""
    lines = [f"# {path.stem}, written from an MJCF's inline mesh by scripts/mjcf_inline_meshes.py"]
    lines += [
        f"v {vertices[i]:.6g} {vertices[i + 1]:.6g} {vertices[i + 2]:.6g}"
        for i in range(0, len(vertices), 3)
    ]
    lines += [
        f"f {faces[i] + 1} {faces[i + 1] + 1} {faces[i + 2] + 1}" for i in range(0, len(faces), 3)
    ]
    path.write_text("\n".join(lines) + "\n")


def materialize(root: ET.Element, meshes: Path, prefix: str) -> int:
    """Write every inline mesh under ``root`` to ``meshes`` and repoint it. Returns how many."""
    written = 0
    for mesh in root.iter("mesh"):
        vertex, face = mesh.get("vertex"), mesh.get("face")
        if mesh.get("file") or not vertex:
            continue
        name = mesh.get("name")
        if not name:
            logger.warning("skipping an inline mesh with no name; nothing could reference it")
            continue
        if not face:
            # MuJoCo would hull it. Saying so beats writing a point cloud an importer reads as
            # an empty mesh, which is a hole in the scene with nothing to explain it.
            logger.warning(f"skipping inline mesh {name!r}: it has vertices but no faces")
            continue
        vertices = [float(v) for v in vertex.split()]
        faces = [int(i) for i in face.split()]
        file = f"{prefix}{name}.obj"
        write_obj(meshes / file, vertices, faces)
        mesh.set("file", file)
        del mesh.attrib["vertex"]
        del mesh.attrib["face"]
        written += 1
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mjcf", required=True, type=Path, help="the MJCF to read")
    parser.add_argument(
        "--meshes", required=True, type=Path, help="where its mesh files live, and these go"
    )
    parser.add_argument("--out", required=True, type=Path, help="the converter-ready MJCF to write")
    parser.add_argument(
        "--prefix", default="inline_", help="prepended to each written file, to keep them apart"
    )
    args = parser.parse_args()

    if not args.mjcf.is_file():
        print(f"ERROR: no file at {args.mjcf}", file=sys.stderr)
        return 1
    if not args.meshes.is_dir():
        print(f"ERROR: no mesh directory at {args.meshes}", file=sys.stderr)
        return 1

    tree = ET.parse(args.mjcf)
    root = tree.getroot()
    written = materialize(root, args.meshes, args.prefix)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.out, encoding="unicode")
    remaining = sum(1 for mesh in root.iter("mesh") if not mesh.get("file"))
    print(f"{written} inline mesh(es) written to {args.meshes}")
    print(f"wrote {args.out}")
    if remaining:
        print(f"note: {remaining} mesh(es) still have no file; the converter will refuse them")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
