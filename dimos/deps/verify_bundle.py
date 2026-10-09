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

"""Exercise a prepared bundle inside the interpreter it was installed into.

    python -m dimos.deps.verify_bundle <bundle> [--backend cpu|cuda]

Imports every registry entry assigned to the bundle (and the bundles it includes), then
the late dependency paths a blueprint reaches only after import: adapter and task
registries, planning backends, model classes, the spatial-memory store, the MuJoCo
subprocess script, agents, web and relay. Finally it checks the inference providers.
``dimos/deps/test_prepare_integration.py`` runs this in fresh virtualenvs; the module
itself needs no test packages. Any failure propagates; nothing is skipped silently.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import os
import platform
import sys

# Importing blueprints builds LCM handles; keep them off the network.
os.environ.setdefault("LCM_DEFAULT_URL", "memq://")

from dimos.core.global_config import global_config
from dimos.deps.bundles import load_assignments
from dimos.deps.install import PROVIDERS
from dimos.robot.get_all_blueprints import get_by_name

# Mirrors the self-references in pyproject.toml (checked by test_backend.py).
INCLUDES = {
    "runtime-common": (),
    "runtime-unitree": ("runtime-common",),
    "runtime-manipulation": ("runtime-unitree",),
    "runtime-unitree-dds": ("runtime-manipulation",),
    "runtime-drone": ("runtime-common",),
    "runtime-spot": ("runtime-common",),
}
# Entries whose import needs a vendor SDK or system library that no bundle can install:
# registry name -> (import that must exist, description). They are only imported when the
# prerequisite is present; gstreamer_camera.py otherwise adds the system dist-packages to
# sys.path before failing, which would leak unrelated system packages into this check.
HOST_PREREQUISITES = {
    "gstreamer-camera-module": ("gi", "GStreamer with PyGObject (python3-gi)"),
    "zed-camera": ("pyzed", "the Stereolabs ZED SDK"),
    "demo-object-scene-registration": ("pyzed", "the Stereolabs ZED SDK"),
}
LATE_IMPORTS: dict[str, tuple[str, ...]] = {
    "runtime-common": (
        "langgraph",
        "langchain_openai",
        "langchain_core.tools",
        "faster_whisper",
        "openai",
        "ollama",
        "openevals",
        "dimos.agents.mcp.mcp_client",
        "fastapi",
        "uvicorn",
        "aioquic",
        "socketio",
        "sse_starlette",
        "ffmpeg",
        "soundfile",
        "dimos.web.relay_bridge.relay_bridge_module",
        "dimos.web.websocket_vis.websocket_vis_module",
        "torch",
        "torchvision",
        "transformers",
        "ultralytics",
        "chromadb",
        "timm",
        "einops",
        "sentencepiece",
        "tokenizers",
        "hydra",
        "omegaconf",
        "sam2",
        "moondream",
        "huggingface_hub",
        "safetensors",
        "dimos.models.segmentation.edge_tam",
        "dimos.models.vl.moondream",
        "dimos.models.vl.florence",
        "dimos.models.embedding.clip",
        "dimos.models.embedding.siglip",
        "dimos.models.embedding.mobileclip",
        "dimos.models.embedding.treid",
        "dimos.perception.detection.detectors.yolo",
        "dimos.perception.detection.detectors.owlv2",
        "dimos.perception.experimental.image_embedding",
        "open_clip",
        "torchreid",
        "googlemaps",
        "portal",
        "gdown",
        "tensorboard",
        "aiortc",
        "aiohttp",
        "av",
        "reportlab",
        "manifold3d",
        "trimesh",
        "mcap.reader",
        "dimos.memory.store.mcap",
        "dimos.visualization.rerun.bridge",
        "dimos.visualization.rerun.urdf_robot",
    ),
    "runtime-unitree": (
        "unitree_webrtc_connect",
        "pygame",
        "mujoco",
        "mujoco_playground",
        "dimos.robot.unitree.keyboard_teleop",
        "dimos.robot.unitree.mujoco_connection",
        "dimos.robot.unitree.dimsim_connection",
        # The MuJoCo connection runs this script as a separate interpreter.
        "dimos.simulation.mujoco.mujoco_process",
        "dimos.simulation.mujoco.policy",
        "dimos.simulation.mujoco.menagerie",
    ),
    "runtime-manipulation": (
        "xarm",
        "piper_sdk",
        "can_motor_control",
        "can",
        "xacro",
        "roboplan",
        "viser",
        "yourdfpy",
        "collada",
        "h5py",
        "pyarrow",
        "pandas",
        "dimos.hardware.manipulators.xarm.adapter",
        "dimos.hardware.manipulators.piper.adapter",
        "dimos.hardware.manipulators.a750.adapter",
        "dimos.hardware.manipulators.sim.adapter",
        "dimos.hardware.whole_body.damiao.adapter",
        "dimos.hardware.whole_body.openarm_damiao.adapter",
        "dimos.hardware.whole_body.openyam_damiao.adapter",
        "dimos.hardware.whole_body.dual_openyam_damiao.adapter",
        "dimos.hardware.drive_trains.flowbase.adapter",
        "dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task",
        "dimos.manipulation.planning.factory",
        "dimos.manipulation.planning.world.roboplan_world",
        "dimos.manipulation.planning.planners.roboplan_planner",
        "dimos.manipulation.planning.trajectory_generator.roboplan_toppra_parametrizer",
        "dimos.manipulation.planning.utils.mesh_utils",
        "dimos.manipulation.visualization.viser.visualizer",
        "dimos.imitation.dataprep.formats.hdf5.writer",
        "dimos.imitation.dataprep.formats.lerobot.writer",
    ),
    "runtime-unitree-dds": (
        "cyclonedds",
        "unitree_sdk2py",
        "dimos.hardware.drive_trains.unitree_go2.adapter",
        "dimos.robot.unitree.g1.effectors.high_level.dds_sdk",
    ),
    "runtime-drone": ("pymavlink", "dimos.robot.drone.mavlink_connection"),
    "runtime-spot": ("bosdyn.client", "bosdyn.api", "dimos.experimental.robot.bosdyn.spot.utils"),
}


def _host_limited_imports(bundle: str) -> list[str]:
    """Packages a marker leaves out on some hosts (documented limitations, not failures)."""
    linux_aarch64 = sys.platform == "linux" and platform.machine() == "aarch64"
    apple_silicon = sys.platform == "darwin" and platform.machine() == "arm64"
    python = f"{sys.version_info[0]}.{sys.version_info[1]}"
    names: list[str] = []
    if bundle == "runtime-common":
        if python != "3.10" or apple_silicon:
            names.append("gtsam")
    if bundle == "runtime-manipulation":
        if not linux_aarch64:
            names.append("pydrake")
        if sys.platform == "linux" and platform.machine() == "x86_64" and python == "3.12":
            names.append("a750_control")
    return names


def closure(bundle: str) -> list[str]:
    bundles = [bundle]
    for included in INCLUDES[bundle]:
        bundles += closure(included)
    return bundles


def exercise(bundle: str) -> None:
    """Late paths that need more than an import."""
    if bundle == "runtime-common":
        from dimos.perception.experimental.spatial_vector_db import SpatialVectorDB

        # chromadb client plus its sqlite store, in memory.
        SpatialVectorDB(collection_name="verify_bundle")
    if bundle == "runtime-manipulation":
        from dimos.control.tasks.registry import control_task_registry
        from dimos.hardware.manipulators.registry import adapter_registry
        from dimos.hardware.whole_body.registry import whole_body_adapter_registry

        assert {"xarm", "piper", "a750", "mock"} <= set(adapter_registry.available())
        assert {"sim_mujoco_g1", "transport_lcm"} <= set(whole_body_adapter_registry.available())
        assert {"cartesian_ik", "g1_groot_wbc", "trajectory"} <= set(
            control_task_registry.available()
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", choices=sorted(INCLUDES))
    parser.add_argument("--backend", choices=sorted(PROVIDERS), default="cpu")
    args = parser.parse_args(argv)

    bundles = closure(args.bundle)
    assignments = load_assignments()
    names = sorted(name for name, bundle in assignments.items() if bundle in bundles)
    # The multi-robot blueprints read the fleet addresses at import time.
    global_config.robot_ips = "192.0.2.10,192.0.2.11"
    imported = 0
    for name in names:
        if name in HOST_PREREQUISITES:
            module, description = HOST_PREREQUISITES[name]
            if importlib.util.find_spec(module) is None:
                print(f"{name}: skipped, needs {description} ({module} is not installed)")
                continue
        get_by_name(name)
        imported += 1

    modules = [module for bundle in bundles for module in LATE_IMPORTS[bundle]]
    modules += [module for bundle in bundles for module in _host_limited_imports(bundle)]
    for module in modules:
        importlib.import_module(module)
    for bundle in bundles:
        exercise(bundle)

    import cv2
    import onnxruntime

    assert hasattr(cv2, "legacy"), "opencv-contrib-python was overwritten by another OpenCV build"
    providers = onnxruntime.get_available_providers()
    assert PROVIDERS[args.backend] in providers, f"onnxruntime providers: {providers}"
    print(
        f"verified {args.bundle}: {imported} registry entries, {len(modules)} late imports, "
        f"onnxruntime {PROVIDERS[args.backend]}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
