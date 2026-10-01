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

"""Build generated extensions using the project's setuptools native build path."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any

from pybind11.setup_helpers import build_ext
from setuptools import Extension


class MessageExtension(Extension):
    def __init__(self, name: str, source_dir: Path, prefix: Path | None = None) -> None:
        super().__init__(name, sources=[])
        self.cmake_source_dir = source_dir.resolve()
        self.cmake_prefix = prefix


class MessageBuildExt(build_ext):
    def build_extension(self, extension: Any) -> None:
        if not isinstance(extension, MessageExtension):
            super().build_extension(extension)
            return
        destination = Path(self.get_ext_fullpath(extension.name)).resolve().parent
        build = Path(self.build_temp).resolve() / extension.name
        extra = []
        if extension.cmake_prefix is not None:
            prefixes = [
                str(extension.cmake_prefix.resolve()),
                os.environ.get("CMAKE_PREFIX_PATH", ""),
            ]
            extra.append("-DCMAKE_PREFIX_PATH=" + ";".join(prefixes))
        subprocess.run(
            [
                "cmake",
                "-S",
                str(extension.cmake_source_dir),
                "-B",
                str(build),
                "-DCMAKE_BUILD_TYPE=Release",
                f"-DPython_EXECUTABLE={sys.executable}",
                f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={destination}",
                *extra,
            ],
            check=True,
        )
        subprocess.run(["cmake", "--build", str(build), "--parallel", "2"], check=True)
        stubs = destination / f"{extension.name}-stubs"
        if stubs.exists():
            shutil.rmtree(stubs)
        shutil.copytree(extension.cmake_source_dir.parent / "typing" / extension.name, stubs)
