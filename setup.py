# Copyright 2025-2026 Dimensional Inc.
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

import fnmatch
import os
from pathlib import Path
import struct
import sys

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import find_packages, setup
from setuptools.command.build_py import build_py as _build_py


def python_is_macos_universal_binary(executable: str | None = None) -> bool:
    """
    Returns True if the given executable is a macOS universal (fat) binary.
    """
    FAT_MAGIC = 0xCAFEBABE  # big-endian fat
    FAT_CIGAM = 0xBEBAFECA  # little-endian fat
    FAT_MAGIC_64 = 0xCAFEBABF  # big-endian fat 64
    FAT_CIGAM_64 = 0xBFBAFECA  # little-endian fat 64

    if executable is None:
        executable = sys.executable

    path = Path(executable)
    if not path.exists():
        return False

    try:
        with path.open("rb") as f:
            header = f.read(4)
            if len(header) < 4:
                return False

            magic = struct.unpack(">I", header)[0]
            return magic in {
                FAT_MAGIC,
                FAT_CIGAM,
                FAT_MAGIC_64,
                FAT_CIGAM_64,
            }
    except OSError:
        return False


TEST_MODULE_PATTERNS = ("test_*.py", "conftest.py")

# The Deno relay (repo-root web/) ships inside the wheel so a pip-installed
# dimos can run it without a checkout. Copied into build_lib below; editable
# installs skip the copy and locate.find_web_dir() resolves the checkout.
# MANIFEST.in grafts web/ so sdist->wheel builds can reproduce this.
RELAY_DIST_SOURCES = ("deno.json", "deno.lock", "relay", "shared")
RELAY_DIST_TARGET = os.path.join("dimos", "web", "relay_bridge", "_relay_dist")


class build_py(_build_py):
    def find_package_modules(self, package, package_dir):
        return [
            (pkg, mod, filepath)
            for pkg, mod, filepath in super().find_package_modules(package, package_dir)
            if not any(
                fnmatch.fnmatch(os.path.basename(filepath), pat) for pat in TEST_MODULE_PATTERNS
            )
        ]

    def run(self):
        super().run()
        if not getattr(self, "editable_mode", False):
            self._copy_relay_dist()

    def _copy_relay_dist(self):
        src = Path(__file__).parent / "web"
        if not (src / "relay" / "main.ts").is_file():
            raise RuntimeError(f"relay sources missing at {src}; refusing to build the wheel")
        dst = Path(self.build_lib) / RELAY_DIST_TARGET
        for name in RELAY_DIST_SOURCES:
            for path in sorted((src / name).rglob("*")) if (src / name).is_dir() else [src / name]:
                if path.is_dir() or path.name.endswith("_test.ts"):
                    continue
                target = dst / path.relative_to(src)
                self.mkpath(str(target.parent))
                self.copy_file(str(path), str(target))


extra_compile_args = [
    "-O3",  # Maximum optimization
    "-ffast-math",  # Fast floating point
]
# when the python exe is a universal binary, this option fails because the compiler
# call tries to build a matching (e.g. universal) binary, clang doesn't support this option for universal binaries
# if the user is using an arm64 specific binary (ex: nix build) then the optimization exists and is useful
# CIBUILDWHEEL=1 marks the release-wheel build; drop -march=native there so wheels are portable across customer CPUs
if not python_is_macos_universal_binary() and os.environ.get("CIBUILDWHEEL") != "1":
    extra_compile_args.append("-march=native")

# C++ extensions
ext_modules = [
    Pybind11Extension(
        "dimos.navigation.replanning_a_star.min_cost_astar_ext",
        [os.path.join("dimos", "navigation", "replanning_a_star", "min_cost_astar_cpp.cpp")],
        extra_compile_args=extra_compile_args,
        define_macros=[
            ("NDEBUG", "1"),
        ],
    ),
]

setup(
    packages=find_packages(),
    package_dir={"": "."},
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext, "build_py": build_py},
)
