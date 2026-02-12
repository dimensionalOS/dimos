from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import Any

def attach(
    package_name: str,
    *,
    submodules: Sequence[str] | None = None,
    submod_attrs: Mapping[str, Sequence[str]] | None = None,
) -> tuple[Callable[[str], Any], Callable[[], list[str]], list[str]]: ...
