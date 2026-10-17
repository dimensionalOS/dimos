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

"""Smoke-export LIBERO-PRO / RoboCasa textures into DimOS ``data/`` MJCF packs.

    python -m dimos.evals.environments.lib.export_benchmark_scenes \\
        --libero-pro /path/to/LIBERO-PRO --robocasa /path/to/robocasa
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
from typing import Any
import xml.etree.ElementTree as ET

from dimos.evals.environments.lib.benchmark_scenes import write_manifest
from dimos.utils.data import get_data, get_data_dir

# Three free bodies so included xarm7.xml home keyframe nq still matches.
# Attribute values are filled via ElementTree (not str.format).
_SCENE = """\
<mujoco>
  <include file="xarm7.xml"/>
  <statistic center="0.3 0 0.4" extent=".65"/>
  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.4 0.4 0.4" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="150" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="floor_tex" file=""/>
    <texture type="2d" name="table_tex" file=""/>
    <material name="groundplane" texture="floor_tex" texuniform="true" texrepeat="5 5" reflectance="0"/>
    <material name="table_top" texture="table_tex" texuniform="true" texrepeat="2 2" reflectance="0.1"/>
    <material name="table_leg" rgba="0.4 0.35 0.3 1"/>
    <material name="object" rgba="1 1 1 1"/>
  </asset>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <geom type="cylinder" size=".06 .06" pos="0 0 .06" rgba="1 1 1 1"/>
    <geom name="table_top" type="box" size="0.15 0.20 0.01" pos="0.45 0.0 0.12"
          material="table_top" mass="5.0"/>
    <geom name="table_leg1" type="cylinder" size="0.02 0.055" pos="0.32 -0.18 0.055" material="table_leg"/>
    <geom name="table_leg2" type="cylinder" size="0.02 0.055" pos="0.58 -0.18 0.055" material="table_leg"/>
    <geom name="table_leg3" type="cylinder" size="0.02 0.055" pos="0.32  0.18 0.055" material="table_leg"/>
    <geom name="table_leg4" type="cylinder" size="0.02 0.055" pos="0.58  0.18 0.055" material="table_leg"/>
    <body name="object" pos="0 0 0">
      <freejoint/>
      <geom type="sphere" size="0.04" material="object" mass="0.15"/>
    </body>
    <body name="filler_a" pos="0.40 0.10 0.16">
      <freejoint/>
      <geom type="sphere" size="0.03" rgba="0.6 0.6 0.6 1" mass="0.1"/>
    </body>
    <body name="filler_b" pos="0.50 -0.10 0.16">
      <freejoint/>
      <geom type="sphere" size="0.03" rgba="0.6 0.6 0.6 1" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""


def _copy(src: Path, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)


def _xarm_root(data_dir: Path | None) -> Path:
    if data_dir is not None:
        xarm = data_dir / "xarm7"
        if not xarm.is_dir():
            raise FileNotFoundError(f"missing {xarm}")
        return xarm
    return get_data("xarm7")


def _link_xarm(pack: Path, data_dir: Path | None) -> None:
    xarm = _xarm_root(data_dir)
    for name in ("xarm7.xml", "assets"):
        dest = pack / name
        if dest.exists() or dest.is_symlink():
            dest.unlink()
        dest.symlink_to(Path(os.path.relpath(xarm / name, start=pack)))


def _find_png(root: Path, names: list[str]) -> Path:
    for name in names:
        hits = list(root.rglob(name))
        if hits:
            return hits[0]
    pngs = list(root.rglob("*.png"))
    if not pngs:
        raise FileNotFoundError(f"no png under {root}")
    return pngs[0]


def _require(elem: ET.Element | None, path: str) -> ET.Element:
    if elem is None:
        raise ValueError(f"scene template missing {path}")
    return elem


def _write_scene(
    pack: Path,
    *,
    model: str,
    floor_name: str,
    table_name: str,
    body: str,
    body_pos: str,
    geom: str,
    size: str,
    rgba: str,
) -> None:
    root = ET.fromstring(_SCENE)
    root.set("model", model)
    _require(root.find("asset/texture[@name='floor_tex']"), "floor_tex").set(
        "file", f"textures/{floor_name}"
    )
    _require(root.find("asset/texture[@name='table_tex']"), "table_tex").set(
        "file", f"textures/{table_name}"
    )
    _require(root.find("asset/material[@name='object']"), "object material").set("rgba", rgba)
    body_el = _require(root.find("worldbody/body[@name='object']"), "object body")
    body_el.set("name", body)
    body_el.set("pos", body_pos)
    geom_el = _require(body_el.find("geom"), "object geom")
    geom_el.set("type", geom)
    geom_el.set("size", size)
    (pack / "scene.xml").write_text(ET.tostring(root, encoding="unicode") + "\n")


def _write_pack(
    out_root: Path,
    *,
    data_dir: Path | None,
    model: str,
    floor_src: Path,
    table_src: Path,
    floor_name: str,
    table_name: str,
    body: str,
    body_pos: str,
    geom: str,
    size: str,
    rgba: str,
    source: str,
    root: str,
    case: dict[str, Any],
) -> Path:
    pack = out_root / "smoke"
    pack.mkdir(parents=True, exist_ok=True)
    _link_xarm(pack, data_dir)
    _copy(floor_src, pack / "textures" / floor_name)
    _copy(table_src, pack / "textures" / table_name)
    _write_scene(
        pack,
        model=model,
        floor_name=floor_name,
        table_name=table_name,
        body=body,
        body_pos=body_pos,
        geom=geom,
        size=size,
        rgba=rgba,
    )
    write_manifest(
        out_root / "manifest.json",
        {"source": source, "root": root, "cases": [case]},
    )
    return out_root


def export_libero_pro(source: Path, out_root: Path, *, data_dir: Path | None = None) -> Path:
    textures = source / "libero" / "libero" / "assets" / "textures"
    if not textures.is_dir():
        raise FileNotFoundError(textures)
    floor = textures / "tile_grigia_caldera_porcelain_floor.png"
    table = textures / "table_light_wood.png"
    return _write_pack(
        out_root,
        data_dir=data_dir,
        model="libero_pro_smoke",
        floor_src=floor,
        table_src=table,
        floor_name=floor.name,
        table_name=table.name,
        body="akita_black_bowl",
        body_pos="0.45 0.0 0.16",
        geom="cylinder",
        size="0.04 0.025",
        rgba="0.12 0.12 0.12 1",
        source="LIBERO-PRO",
        root="libero_pro",
        case={
            "id": "libero_pro_smoke_lift_akita_black_bowl",
            "language": (
                "Pick up the akita black bowl from the table and hold it in the air. "
                "The table top is at z=0.13 m; find the bowl through the wrist camera."
            ),
            "scene": "smoke/scene.xml",
            "tracked_bodies": ["akita_black_bowl"],
            "grade": {"type": "lifted", "body": "akita_black_bowl", "by_m": 0.05},
            "tags": ["mujoco", "manipulation", "smoke", "libero_pro"],
        },
    )


def export_robocasa(source: Path, out_root: Path, *, data_dir: Path | None = None) -> Path:
    assets = source / "robocasa" / "models" / "assets"
    table = _find_png(assets, ["marble.png", "light_wood.png", "wood.png", "ceramic.png"])
    # Reuse one texture for floor + counter in the smoke pack.
    return _write_pack(
        out_root,
        data_dir=data_dir,
        model="robocasa_smoke",
        floor_src=table,
        table_src=table,
        floor_name="counter.png",
        table_name="counter.png",
        body="apple",
        body_pos="0.45 0.05 0.17",
        geom="sphere",
        size="0.04",
        rgba="0.85 0.1 0.1 1",
        source="RoboCasa",
        root="robocasa",
        case={
            "id": "robocasa_smoke_lift_apple",
            "language": (
                "Pick up the apple from the counter and hold it in the air. "
                "The counter top is at z=0.13 m; find the apple through the wrist camera."
            ),
            "scene": "smoke/scene.xml",
            "tracked_bodies": ["apple"],
            "grade": {"type": "lifted", "body": "apple", "by_m": 0.05},
            "tags": ["mujoco", "manipulation", "smoke", "robocasa"],
        },
    )


def main(argv: list[str] | None = None) -> int:
    # Always write under project data/ — smoke suites load via get_data().
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--libero-pro", type=Path)
    p.add_argument("--robocasa", type=Path)
    args = p.parse_args(argv)
    if not args.libero_pro and not args.robocasa:
        p.error("pass --libero-pro and/or --robocasa")
    data = get_data_dir()
    if args.libero_pro:
        print("wrote", export_libero_pro(args.libero_pro, data / "libero_pro"))
    if args.robocasa:
        print("wrote", export_robocasa(args.robocasa, data / "robocasa"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
