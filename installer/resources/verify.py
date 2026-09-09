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

"""Dependency checks only: do not deploy modules, connect robots, or fetch models."""

import ctypes
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from turbojpeg import TurboJPEG

from dimos.core.native_module import NativeModule

REFERENCES = {
    "navigation": (
        "dimos.robot.unitree.go2.blueprints.smart.unitree_go2",
        "unitree_go2",
        ["dimos.navigation.replanning_a_star.min_cost_astar_ext", "unitree_webrtc_connect"],
    ),
    "manipulation": (
        "dimos.robot.manipulators.xarm.blueprints.basic",
        "xarm7_planner_coordinator",
        ["roboplan", "pinocchio", "xarm"],
    ),
}


def main() -> None:
    profile, project_path = sys.argv[1:]
    project = Path(project_path).resolve()
    if Path(sys.prefix).resolve() != project / ".venv":
        raise RuntimeError("Python is not running from the managed uv environment")
    prefix = Path(os.environ["CONDA_PREFIX"])
    if (prefix / "bin/python").exists():
        raise RuntimeError("Pixi must not own Python")
    for library in (
        "libturbojpeg.so",
        "libportaudio.so",
        "libsndfile.so",
        "libGL.so.1",
        "libEGL.so.1",
    ):
        ctypes.CDLL(str(prefix / "lib" / library))
    for command in (["git", "lfs", "version"], ["ffmpeg", "-version"]):
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL, timeout=30)
    for name in ("zenoh", "lcm", "cv2", "open3d"):
        importlib.import_module(name)
    codec = TurboJPEG()
    pixels = np.zeros((8, 8, 3), dtype=np.uint8)
    decoded = codec.decode(codec.encode(pixels))
    if decoded.shape != pixels.shape:
        raise RuntimeError("TurboJPEG round trip failed")
    module, name, native_imports = REFERENCES[profile]
    for dependency in native_imports:
        importlib.import_module(dependency)
    blueprint = getattr(importlib.import_module(module), name)
    # These starter blueprints currently require wheel extensions, not custom
    # NativeModule executables. Fail if that changes, rather than deferring a
    # newly introduced build to the user's first robot run.
    for atom in blueprint.active_blueprints:
        if issubclass(atom.module, NativeModule):
            raise RuntimeError(f"{atom.module.__name__} needs explicit setup-time Nix preparation")
    print(json.dumps({"profile": profile, "reference": name, "dependency_checks": "passed"}))


if __name__ == "__main__":
    main()
