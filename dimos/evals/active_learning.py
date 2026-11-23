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

"""Optional, named eval prompts. Not part of the agent system prompt or skills.

Pass ``active_learning=<name>`` or a prompt-file path on ``DimosAgentAdapter``.
Empty means off. Packs here are task heuristics learned from a suite; keep tool
mechanics in ``MEMORY_INDEX`` / ``MEMORY_RULES`` / the memory skill.

The adapter writes the pack to ``active-learning.md`` in the case workspace and
points the user message at that file's absolute path (consult-on-demand memory).
Memory protocol (``MEMORY_GUIDANCE``) is inlined in the same user message.
"""

from __future__ import annotations

from pathlib import Path

HANDPICK_MEMORY = """\
A shop name in objects_and_text is not a walkable path and not "the store is open". Inspect whether the robot actually went there, and whether the door or gate is open in the frame.

A walkable path is blocked if fences, construction barriers, stanchions, closed gates, or similar obstacles sit on the route. An open-looking mall floor is not enough — inspect the frames between the two places and look specifically for those barriers.

Do not judge a store open from lights, signs, or merchandise. Look at whether door leaves or a roller shutter block the entrance; a shutter or closed door means closed even if the interior is lit.
"""

PACKS: dict[str, str] = {
    "handpick-memory": HANDPICK_MEMORY,
}


def load_active_learning(spec: str) -> str:
    """Return a named pack, or the text of a prompt file.

    Args:
        spec: Pack name in ``PACKS``, or a filesystem path.

    Raises:
        ValueError: Empty spec, unknown name, or missing file.
    """
    key = spec.strip()
    if not key:
        raise ValueError("active_learning is empty")
    pack = PACKS.get(key)
    if pack is not None:
        return pack.strip() + "\n"
    path = Path(key).expanduser()
    if path.is_file():
        return path.read_text(encoding="utf-8").rstrip() + "\n"
    known = ", ".join(sorted(PACKS))
    raise ValueError(
        f"unknown active_learning {spec!r}; use a named pack ({known}) or a prompt file"
    )
