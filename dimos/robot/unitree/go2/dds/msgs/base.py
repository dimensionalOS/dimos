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

"""Pretty multi-line ``__repr__`` for the Go2 message dataclasses.

Mixin for top-level messages; the formatter recurses through nested dataclasses,
lists, and numpy arrays, so nested types need not inherit it.
"""

from __future__ import annotations

from dataclasses import fields, is_dataclass
from typing import Any

import numpy as np


def _fmt(v: Any, indent: int) -> str:
    pad, close = "  " * (indent + 1), "  " * indent
    if is_dataclass(v) and not isinstance(v, type):
        body = "".join(
            f"\n{pad}{f.name}={_fmt(getattr(v, f.name), indent + 1)}," for f in fields(v)
        )
        return f"{type(v).__name__}({body}\n{close})"
    if isinstance(v, np.ndarray):
        a = np.round(v, 4) if v.dtype.kind == "f" else v
        return np.array2string(a, separator=", ", max_line_width=100)
    if isinstance(v, list):
        if v and is_dataclass(v[0]):
            return "[" + "".join(f"\n{pad}{_fmt(x, indent + 1)}," for x in v) + f"\n{close}]"
        return repr(v)
    if isinstance(v, float):
        return f"{v:.4f}"
    return repr(v)


class PrettyMsg:
    """Mixin giving a readable multi-line ``__repr__``. Use with ``@dataclass(repr=False)``."""

    def __repr__(self) -> str:
        return _fmt(self, 0)
