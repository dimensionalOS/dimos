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

"""Finding a thing in a model that came from USD, by the name a description calls it.

MuJoCo reads USD, and names everything in it by its full prim path: the body a URDF called
``pelvis`` arrives as ``/g1/Joints/pelvis``, and 448 of the office scene's 449 bodies carry a
``/``. A description should not have to know that -- ``root_body="pelvis"`` is what a person
writes, and what the same robot's MJCF would have been called -- so the path is resolved here and
stays an implementation detail of loading.

Exact match wins, because a model that really does name something ``/g1/Joints/pelvis`` means it.
Otherwise the last segment of the path is what a short name refers to. Ambiguity is an error
rather than a guess: two bodies ending ``pelvis`` means the description has to say which.
"""

from collections.abc import Iterable


def resolve(names: Iterable[str], wanted: str) -> str:
    """The full name `wanted` refers to, among `names`.

    Raises KeyError naming the candidates when nothing matches or more than one does.
    """
    available = list(names)
    if wanted in available:
        return wanted
    tail = [name for name in available if name.rsplit("/", 1)[-1] == wanted]
    if len(tail) == 1:
        return tail[0]
    if not tail:
        raise KeyError(f"no {wanted!r} among {_sample(available)}")
    raise KeyError(f"{wanted!r} is ambiguous: {sorted(tail)}")


def resolved(names: Iterable[str], wanted: str) -> str | None:
    """`resolve`, or None where it would raise: for a lookup that has somewhere else to go."""
    try:
        return resolve(names, wanted)
    except KeyError:
        return None


def _sample(available: list[str], most: int = 6) -> str:
    """Enough of what there was to see the shape of the names, without printing a whole scene."""
    shown = sorted(available)[:most]
    rest = len(available) - len(shown)
    return ", ".join(shown) + (f", ... ({rest} more)" if rest > 0 else "")
