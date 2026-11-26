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

"""Compose the USD served by the blueprint's HardwareModules."""

from collections.abc import Sequence
from hashlib import sha256
from pathlib import Path
from typing import Any

from dimos.constants import CACHE_DIR
from dimos.core.hardware_file_client import fetch_hardware_files
from dimos.core.hardware_module import HardwareSpec
from dimos.core.module import ModuleConfig
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.assets import publish_usd_assets, resolve_usd_assets
from dimos.robot.assets.usd_utils.layers import overlay_usds
from dimos.robot.assets.usd_utils.pxr import frame_path, layer_from, load_pxr
from dimos.spec.utils import get_protocol_method_signatures
from dimos.utils.safe_thread_map import safe_thread_map


class SceneConfig(ModuleConfig):
    # A supplied complete USD is useful for a physics test. Blueprints normally
    # leave this empty and contribute HardwareModules instead.
    scene: UsdModel | Path | None = None


def compose_scene(modules: Sequence[Any]) -> UsdModel:
    required = set(get_protocol_method_signatures(HardwareSpec))
    hardware = [module for module in modules if required <= set(module.rpcs)]

    def fetch(module: Any) -> tuple[str, str | None]:
        namespace = module.instance_name.rpartition("/")[0]
        local = fetch_hardware_files(namespace)
        return resolve_usd_assets(local.read_text(), str(local.parent)), module.attached_to()

    parts = safe_thread_map(hardware, fetch)
    roots = [usd for usd, mount in parts if mount is None]
    if len(roots) != 1:
        raise ValueError("simulation requires exactly one unmounted scene HardwareModule")
    return UsdModel.from_layer(
        overlay_usds(roots[0], [(usd, mount) for usd, mount in parts if mount is not None])
    )


def scene_revision(scene: UsdModel) -> str:
    # Physics and a renderer may fetch the same files into different host caches.
    # Reuse the file server's public asset names so cache paths are not identity.
    root = scene.source_path.parent if scene.source_path else None
    canonical, _ = publish_usd_assets(scene.usd, root)
    return sha256(canonical.encode()).hexdigest()


def materialize_scene(scene: UsdModel) -> UsdModel:
    root = str(scene.source_path.parent) if scene.source_path else ""
    usd = resolve_usd_assets(scene.usd, root)
    path = CACHE_DIR / "sim2" / "scenes" / (sha256(usd.encode()).hexdigest() + ".usda")
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_text(usd)
    return UsdModel.from_file(path)


def joint_paths(scene: UsdModel) -> dict[str, str]:
    """Map a device's declared joint names to its USD prim paths, before namespace."""
    pxr = load_pxr()
    root = scene.source_path.parent if scene.source_path else None
    stage = pxr.Usd.Stage.Open(layer_from(pxr, scene.usd, "joint names", root))
    return {
        str(prim.GetDisplayName() or prim.GetName()): frame_path(prim)
        for prim in stage.Traverse()
        if prim.IsA(pxr.UsdPhysics.Joint)
    }
