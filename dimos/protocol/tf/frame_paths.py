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

"""tf frame ids as paths: ``go2_A/go2/base`` is the prim ``/go2/base`` under the namespace ``go2_A``.

Each "/"-separated segment is a USD prim name, so a frame id is a prim path without its leading "/".
On the wire every frame a HardwareModule publishes has a leading "/" (``/go2_A/go2/base``), which
the tf buffer drops.
"""

import re

# A USD prim name (Tf identifier): an ASCII letter or underscore, then letters, digits or underscores.
_SEGMENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_FIRST = re.compile(r"[A-Za-z_]")
_REST = re.compile(r"[A-Za-z0-9_]")


def frame_path_error(path: str) -> str | None:
    """Why ``path`` is not a frame path (which segment, which character), or None if it is one."""
    if not path:
        return "it is empty"
    for segment in path.split("/"):
        if not segment:
            return f"{path!r} has an empty segment (a leading, trailing or doubled '/')"
        if _SEGMENT.fullmatch(segment):
            continue
        bad = next(
            (index, char)
            for index, char in enumerate(segment)
            if not (_FIRST if index == 0 else _REST).fullmatch(char)
        )
        where = "starts with" if bad[0] == 0 else f"has at position {bad[0]}"
        return (
            f"its segment {segment!r} {where} {bad[1]!r}; a segment is a USD prim name: a letter or "
            f"'_', then letters, digits or '_'"
        )
    return None


def check_frame_path(path: str, what: str) -> None:
    """Raise if ``path`` is not a frame path, naming ``what`` it is (a link, a tf frame, ...)."""
    error = frame_path_error(path)
    if error is not None:
        raise ValueError(f"{what} {path!r} cannot be a tf frame / USD prim path: {error}")


def is_hardware_edge(parent: str, child: str) -> bool:
    """Whether a wire edge (raw frame ids) is a hardware edge, not an imaginary one (an estimate, a localization)."""
    # A HardwareModule puts one leading "/" on every frame it publishes; no other module puts one on any.
    return parent.startswith("/") and child.startswith("/")
