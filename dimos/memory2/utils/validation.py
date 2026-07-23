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

from __future__ import annotations

import re

# Dot-separated segments: each segment is a safe SQL identifier, the whole is a
# T15 module-path stream name (``nav_stack.voxel_mapper2.decisions``). No ``/``,
# no leading/trailing/doubled dot, no empty segment; every SQL site that names a
# stream quotes the identifier, so a dotted name is one atom, never a schema ref.
_IDENT_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z0-9_]+)*$")


def validate_identifier(name: str) -> None:
    """Reject stream names that aren't safe (dot-separated) SQL identifiers."""
    if not _IDENT_RE.match(name):
        raise ValueError(f"Invalid stream name: {name!r}")
