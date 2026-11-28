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

"""Lets ctypes find the libraries shipped in the AppImage's usr/lib.

Python imports this file at startup. ctypes.util.find_library only knows the
ldconfig cache and what `ld` reports, so sounddevice does not find the bundled
PortAudio even with LD_LIBRARY_PATH set.
"""

import ctypes.util
import glob
import os
import sys

_LIB_DIR = os.path.join(sys.prefix, "..", "..", "usr", "lib")
_find_library = ctypes.util.find_library


def find_library(name: str) -> str | None:
    bundled = sorted(glob.glob(os.path.join(_LIB_DIR, f"lib{name}.so*")))
    return bundled[0] if bundled else _find_library(name)


ctypes.util.find_library = find_library
