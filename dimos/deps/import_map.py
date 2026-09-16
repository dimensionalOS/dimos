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

"""Reviewed mapping from top-level import names to what provides them.

Import names are not distribution names (``cv2`` is ``opencv-contrib-python``,
``pinocchio`` is ``pin``), namespace packages can have several providers
(``open3d`` on aarch64 Linux is ``open3d-unofficial-arm``) and some imports are
provided by the host rather than by pip (ROS, the ZED SDK, PyGObject). This
table is the single source of truth the catalog generator uses; it never
consults the installed environment, so generation stays deterministic.

When a new third-party import appears, the catalog generation test fails and
names the import. Add it here, in the section that fits.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import sys
from typing import Literal

ProviderKind = Literal["dist", "system", "native", "undeclared", "dev"]


@dataclass(frozen=True)
class Provider:
    kind: ProviderKind
    names: tuple[str, ...] = ()
    """Canonical distribution names (``dist``), or import names otherwise."""


# Import name -> canonical (PEP 503) distribution names. A tuple lists every
# distribution that provides the import; any one of them satisfies it.
IMPORT_TO_DISTRIBUTION: Mapping[str, tuple[str, ...]] = {
    "IPython": ("ipython",),
    "PIL": ("pillow",),
    "a750_control": ("a750-control",),
    "aiohttp": ("aiohttp",),
    "aioquic": ("aioquic",),
    "aiortc": ("aiortc",),
    "attrs": ("attrs",),
    "av": ("av",),
    "bleak": ("bleak",),
    "bosdyn": ("bosdyn-client",),
    "can": ("python-can",),
    "can_motor_control": ("can-motor-control",),
    "chromadb": ("chromadb",),
    "coacd": ("coacd",),
    "cryptography": ("cryptography",),
    "cv2": ("opencv-contrib-python",),
    "cyclonedds": ("cyclonedds",),
    "dimos_lcm": ("dimos-lcm",),
    "dotenv": ("python-dotenv",),
    "etils": ("etils",),
    "fastapi": ("fastapi",),
    "faster_whisper": ("faster-whisper",),
    "ffmpeg": ("ffmpeg-python",),
    "filelock": ("filelock",),
    "git": ("gitpython",),
    "googlemaps": ("googlemaps",),
    "gtsam": ("gtsam-extended",),
    "h5py": ("h5py",),
    "httpx": ("httpx",),
    "huggingface_hub": ("huggingface-hub",),
    "hydra": ("hydra-core",),
    "imagecodecs": ("imagecodecs",),
    "jsonlines": ("jsonlines",),
    "keyring": ("keyring",),
    "langchain": ("langchain",),
    "langchain_core": ("langchain-core",),
    "langchain_openai": ("langchain-openai",),
    "langgraph": ("langgraph",),
    "lcm": ("lcm-dimos-fork",),
    "lz4": ("lz4",),
    "matplotlib": ("matplotlib",),
    "mcap": ("mcap",),
    "moondream": ("moondream",),
    "mujoco": ("mujoco",),
    "mujoco_playground": ("playground",),
    "numba": ("numba",),
    "numpy": ("numpy",),
    "ollama": ("ollama",),
    "omegaconf": ("omegaconf",),
    "onnxruntime": ("onnxruntime", "onnxruntime-gpu"),
    "open3d": ("open3d", "open3d-unofficial-arm"),
    "open_clip": ("open-clip-torch",),
    "openai": ("openai",),
    "openevals": ("openevals",),
    "optuna": ("optuna",),
    "packaging": ("packaging",),
    "pandas": ("pandas",),
    "pink": ("pin-pink",),
    "pinocchio": ("pin",),
    "piper_sdk": ("piper-sdk",),
    "plotext": ("plotext",),
    "plum": ("plum-dispatch",),
    "portal": ("portal",),
    "psutil": ("psutil",),
    "pxr": ("usd-core",),
    "pyarrow": ("pyarrow",),
    "pydantic": ("pydantic",),
    "pydantic_core": ("pydantic-core",),
    "pydantic_settings": ("pydantic-settings",),
    "pydrake": ("drake",),
    "pygame": ("pygame",),
    "pymavlink": ("pymavlink",),
    "pytest": ("pytest",),
    "qpsolvers": ("qpsolvers",),
    "reactivex": ("reactivex",),
    "reportlab": ("reportlab",),
    "requests": ("requests",),
    "rerun": ("rerun-sdk",),
    "rich": ("rich",),
    "roboplan": ("roboplan",),
    "safetensors": ("safetensors",),
    "sam2": ("edgetam-dimos",),
    "scipy": ("scipy",),
    "socketio": ("python-socketio",),
    "sortedcontainers": ("sortedcontainers",),
    "sounddevice": ("sounddevice",),
    "soundfile": ("soundfile",),
    "sqlite_vec": ("sqlite-vec",),
    "sse_starlette": ("sse-starlette",),
    "starlette": ("starlette",),
    "structlog": ("structlog",),
    "terminaltexteffects": ("terminaltexteffects",),
    "textual": ("textual",),
    "textual_serve": ("textual-serve",),
    "tokenizers": ("tokenizers",),
    "tomli": ("tomli",),
    "toolz": ("toolz",),
    "torch": ("torch",),
    "torchreid": ("torchreid",),
    "torchvision": ("torchvision",),
    "transformers": ("transformers",),
    "trimesh": ("trimesh",),
    "turbojpeg": ("pyturbojpeg",),
    "typer": ("typer",),
    "typing_extensions": ("typing-extensions",),
    "ultralytics": ("ultralytics",),
    "unitree_sdk2py": ("unitree-sdk2py-dimos",),
    "unitree_webrtc_connect": ("unitree-webrtc-connect",),
    "uvicorn": ("uvicorn",),
    "viser": ("viser",),
    "watchdog": ("watchdog",),
    "websocket": ("websocket-client",),
    "websockets": ("websockets",),
    "whisper": ("openai-whisper",),
    "xacro": ("xacro",),
    "xarm": ("xarm-python-sdk",),
    "yaml": ("pyyaml",),
    "yourdfpy": ("yourdfpy",),
    "zenoh": ("eclipse-zenoh",),
}

# Provided by the host installation (ROS 2, vendor SDKs, desktop libraries),
# never by a pip requirement of this project.
SYSTEM_PROVIDED: frozenset[str] = frozenset(
    {
        "a1z",
        "ament_index_python",
        "builtin_interfaces",
        "genesis",
        "geometry_msgs",
        "gi",
        "gs_usb",
        "habitat_sim",
        "isaacsim",
        "lcm_msgs",
        "nav_msgs",
        "omni",
        "pyzed",
        "rclpy",
        "sensor_msgs",
        "std_msgs",
        "tf2_msgs",
        "usb",
    }
)

# Built from this repository (maturin, pybind11), not installed from an index.
IN_TREE_NATIVE: frozenset[str] = frozenset(
    {
        "dimos_local_planner",
        "dimos_mls_planner",
        "dimos_trajectory_follower",
        "dimos_voxel_ray_tracing",
        "dimos.navigation.go2.replanning_a_star.min_cost_astar_ext",
    }
)

# Distributions no extra provides. Files importing them eagerly are reported
# rather than failed; they are candidates for deletion or a new extra.
UNDECLARED: frozenset[str] = frozenset(
    {"datasets", "gymnasium", "jsonref", "mbodied", "pyttsx3", "redis"}
)

# Only legitimate in test/tool files, which the checks skip.
DEV_ONLY: frozenset[str] = frozenset({"optuna", "pytest", "watchdog"})

# Abstract backend requirements resolved by the hardware profile.
BACKENDS: Mapping[str, Mapping[str, tuple[str, ...]]] = {
    "onnxruntime": {"cpu": ("cpu",), "cuda": ("cuda",)},
}
BACKEND_DISTRIBUTIONS: Mapping[str, str] = {
    "onnxruntime": "onnxruntime",
    "onnxruntime-gpu": "onnxruntime",
}

STDLIB_NAMES: frozenset[str] = frozenset(sys.stdlib_module_names) | {"__future__"}


def is_stdlib(top_level: str) -> bool:
    return top_level in STDLIB_NAMES


def provider_for(top_level: str) -> Provider | None:
    """What provides an import name; ``None`` when the name is unknown."""
    if top_level in IN_TREE_NATIVE:
        return Provider("native", (top_level,))
    if top_level in SYSTEM_PROVIDED:
        return Provider("system", (top_level,))
    if top_level in DEV_ONLY:
        return Provider("dev", (top_level,))
    if top_level in UNDECLARED:
        return Provider("undeclared", (top_level,))
    distributions = IMPORT_TO_DISTRIBUTION.get(top_level)
    if distributions is None:
        return None
    return Provider("dist", distributions)


def suggest(top_level: str) -> str:
    """Hint for an unknown import name, from the installed environment if possible."""
    try:
        from importlib.metadata import packages_distributions

        found = packages_distributions().get(top_level)
    except Exception:
        found = None
    if found:
        return f"installed environment maps it to {', '.join(sorted(set(found)))}"
    return "not installed here; check the project that publishes it"
