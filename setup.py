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

import os
from pathlib import Path
import struct
import sys

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import find_packages, setup


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


extra_compile_args = [
    "-O3",  # Maximum optimization
    "-ffast-math",  # Fast floating point
]
# when the python exe is a universal binary, this option fails because the compiler
# call tries to build a matching (e.g. universal) binary, clang doesn't support this option for universal binaries
# if the user is using an arm64 specific binary (ex: nix build) then the optimization exists and is useful
if not python_is_macos_universal_binary():
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
    cmdclass={"build_ext": build_ext},
)
