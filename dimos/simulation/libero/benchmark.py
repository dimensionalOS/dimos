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


"""The pinned LIBERO-PRO checkout: its BDDL task sets, its perturbator and its runtime.

LIBERO-PRO is cloned once into the dimos cache at ``LIBERO_PRO_COMMIT``, with a few
fixes for files the pinned commit gets wrong. Scenes are never built here: the LIBERO
native (``server.py``) builds them inside LIBERO itself.
"""

from __future__ import annotations

import contextlib
import functools
import importlib.util
import io
from pathlib import Path
import random
import re
import subprocess
from typing import TYPE_CHECKING

from dimos.constants import CACHE_DIR

if TYPE_CHECKING:
    from types import ModuleType

LIBERO_PRO_REPO = "https://github.com/Zxy-MLlab/LIBERO-PRO.git"
LIBERO_PRO_COMMIT = "eafdb809426b13153aa1e4c42d6601844217dfec"
LIBERO_ROOT = CACHE_DIR / "libero-pro"
# LIBERO's own runtime, provisioned by ``uv run``; nothing is installed into dimos.
LIBERO_PYTHON = "3.10"
LIBERO_DEPS = (
    "robosuite==1.4.0",
    "mujoco==2.3.7",
    "bddl==1.0.1",
    "numpy==1.26.4",
    "gym==0.25.2",
    "easydict==1.13",
    "future==1.0.0",
    "matplotlib==3.10.9",
    "termcolor==3.3.0",
    "pyyaml==6.0.3",
    # The native publishes dimos messages on zenoh; same versions as dimos's lock.
    "dimos-lcm==0.1.4",
    "eclipse-zenoh==1.10.1",
)
# Upstream objects whose XML lives elsewhere in the repo than the path LIBERO-PRO expects.
_ASSET_FALLBACKS = {"red_sticker": "libero/libero/assets/stable_scanned_objects/red_sticker"}
# Object types LIBERO-PRO registers but whose assets it never published (at the pinned commit).
UNAVAILABLE_OBJECTS = frozenset({"red_box", "blue_red_sticker", "libero_mug_green"})
# (file, broken, fixed): typos in the pinned commit, applied once to the cached checkout.
_UPSTREAM_FIXES = (
    (
        "libero_ood/ood_object.yaml",  # an empty key where wooden_cabinet belongs
        "      - white_bottle\r\n    :\r\n      - yellow_cabinet",
        "      - white_bottle\r\n    wooden_cabinet:\r\n      - yellow_cabinet",
    ),
)
# LIBERO-PRO's YAML-driven perturbations: case suffix -> (PerturbFlags field, config).
PERTURBATIONS = {
    "env": ("use_environment", "environment", "ood_environment.yaml"),
    "swap": ("use_swap", "swap", "ood_spatial_relation.yaml"),
    "object": ("use_object", "object", "ood_object.yaml"),
    "lan": ("use_language", "language", "ood_language.yaml"),
    "task": ("use_task", "task", "ood_task.yaml"),
}


def libero_root() -> Path:
    """The pinned LIBERO-PRO checkout, cloned on first use."""
    if not (LIBERO_ROOT / ".git").exists():
        LIBERO_ROOT.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "init", "-q", str(LIBERO_ROOT)], check=True)
        subprocess.run(
            [
                "git",
                "-C",
                str(LIBERO_ROOT),
                "fetch",
                "-q",
                "--depth",
                "1",
                LIBERO_PRO_REPO,
                LIBERO_PRO_COMMIT,
            ],
            check=True,
        )
        subprocess.run(["git", "-C", str(LIBERO_ROOT), "checkout", "-q", "FETCH_HEAD"], check=True)
    head = subprocess.run(
        ["git", "-C", str(LIBERO_ROOT), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if head != LIBERO_PRO_COMMIT:
        raise RuntimeError(
            f"{LIBERO_ROOT} is at {head}, expected {LIBERO_PRO_COMMIT}; delete it to re-clone"
        )
    custom = LIBERO_ROOT / "notebooks" / "custom_assets"
    for name, source in _ASSET_FALLBACKS.items():
        if not (custom / name).exists():
            custom.mkdir(parents=True, exist_ok=True)
            (custom / name).symlink_to(LIBERO_ROOT / source, target_is_directory=True)
    for file, broken, fixed in _UPSTREAM_FIXES:
        path = LIBERO_ROOT / file
        content = path.read_bytes().decode()
        if broken in content:
            path.write_bytes(content.replace(broken, fixed).encode())
        elif fixed not in content:
            raise RuntimeError(f"Unexpected content in {path}; the upstream fix no longer applies")
    return LIBERO_ROOT


def bddl_root() -> Path:
    return libero_root() / "libero" / "libero" / "bddl_files"


def bddl_language(bddl: Path) -> str:
    match = re.search(r"\(:language\s*(.*?)\)", bddl.read_text(), flags=re.S)
    return " ".join(match.group(1).split()) if match else ""


def bddl_object_types(bddl: Path) -> set[str]:
    """Every object and fixture type the task declares (``name - type``)."""
    types: set[str] = set()
    for section in re.finditer(r"\(:(?:objects|fixtures)\s(.*?)\)", bddl.read_text(), flags=re.S):
        types |= {m.group(1) for m in re.finditer(r"-\s*(\w+)", section.group(1))}
    return types


def perturbed_bddl(bddl: Path, suite: str, kind: str, seed: int = 0) -> Path:
    """``bddl`` rewritten by LIBERO-PRO's own perturbator; ``kind`` is a key of PERTURBATIONS.

    Written as ``<cache>/libero-bddl/<suite>_<kind>/<task>.bddl``, mirroring LIBERO-PRO's
    folder names, and rewritten only when the content changes.
    """
    module = _perturbation_module()
    flags = module.PerturbFlags()
    configs = {}
    for name, (flag, config_key, file) in PERTURBATIONS.items():
        configs[config_key] = str(libero_root() / "libero_ood" / file)
        setattr(flags, flag, name == kind)
    state = random.getstate()
    random.seed(seed)  # swap draws from the global RNG without taking a seed
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            content = module.BDDLCombinedPerturbator(configs).perturb_content(
                bddl.read_text(), suite, bddl.stem, flags, seed=seed
            )
    finally:
        random.setstate(state)
    out = CACHE_DIR / "libero-bddl" / f"{suite}_{kind}" / bddl.name
    if not out.exists() or out.read_text() != content:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(content)
    return out


@functools.cache
def _perturbation_module() -> ModuleType:
    spec = importlib.util.spec_from_file_location(
        "libero_pro_perturbation", libero_root() / "perturbation.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def libero_config_dir() -> Path:
    """A LIBERO config pointing at the checkout; LIBERO prompts on stdin without one."""
    package = libero_root() / "libero" / "libero"
    config = CACHE_DIR / "libero-config"
    config.mkdir(parents=True, exist_ok=True)
    (config / "config.yaml").write_text(
        "".join(
            f"{key}: {package / sub}\n"
            for key, sub in (
                ("benchmark_root", "."),
                ("bddl_files", "bddl_files"),
                ("init_states", "init_files"),
                ("datasets", "../datasets"),
                ("assets", "assets"),
            )
        )
    )
    return config
