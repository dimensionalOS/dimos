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

"""Resolve recorded stream roles without depending on robot-specific names."""

from collections.abc import Mapping

import typer


def select_stream(
    streams: Mapping[str, type],
    compatible: type | tuple[type, ...],
    selected: str | None,
    option: str,
    *,
    required: bool = True,
) -> str | None:
    """Accept an explicit compatible stream or the sole compatible candidate."""
    types = compatible if isinstance(compatible, tuple) else (compatible,)
    candidates = sorted(name for name, kind in streams.items() if issubclass(kind, types))
    if selected is not None:
        if selected not in candidates:
            raise typer.BadParameter(
                f"{selected!r} is not a compatible stream. Candidates: {', '.join(candidates) or 'none'}",
                param_hint=option,
            )
        return selected
    if len(candidates) == 1:
        return candidates[0]
    if len(candidates) > 1:
        raise typer.BadParameter(
            f"Multiple compatible streams: {', '.join(candidates)}. Select one with {option}.",
            param_hint=option,
        )
    if required:
        raise typer.BadParameter("No compatible stream found", param_hint=option)
    return None
