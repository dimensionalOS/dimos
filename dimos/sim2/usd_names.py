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

Resolving a lookup is not enough for a robot, because its names leave the simulation: attaching
one under its id makes every body ``g1/<whatever MuJoCo called it>``, and that string is the
entity id actions address, the tf frame the viewer places, and what a sensor's view asks for. A
robot's elements are therefore renamed to what the description calls them before it is attached
(``shorten``); the scene keeps its own names, since nothing addresses a room by short name.
"""

from collections import Counter
from collections.abc import Iterable, Mapping
from typing import Any


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


def shorten(spec: Any) -> dict[str, str]:
    """Rename a loaded model's parts to the names a description uses. Returns what was renamed.

    What a robot's parts are called does not stay inside the engine: attaching a robot under its
    id names every body ``g1/<engine's name>``, and that is the entity id an action addresses and
    the tf frame a viewer places a model at. Off a USD layer the engine's name is the prim path,
    so the world came out publishing ``dog//toy/Geometry/base`` -- which nobody can write down,
    and a description that said ``root_body="base"`` could not find its own robot in the world it
    had just built.

    **References move with the names.** A model refers to its own parts by name, as strings --
    an actuator names the joint it drives, a sensor the site it reads -- and renaming an element
    does not touch them. Leaving them behind is worse than not renaming at all, and silent: the
    toy robot's motor kept pointing at the joint's old path, so the compiled world came out with
    no actuators and nothing to tell you why.

    Geoms are deliberately left alone. Nothing addresses one by short name, and a model is far
    more likely to have two of them ending the same way (``base_0``, ``base_1``) than two bodies.

    Ambiguity is left alone rather than guessed at: two elements *of one kind* whose paths end
    the same way, or a short name another of that kind already holds, keep the paths they came
    with. ``resolve`` still finds those, and a name that cannot be made unique is better long
    than quietly wrong.
    """
    # A kind at a time, because MuJoCo's namespaces are per kind: the toy robot's hinge and the
    # motor driving it are both called "neck", and judging them together made each the other's
    # ambiguity, so neither was shortened and the actuator was left pointing at a path.
    renames: dict[str, str] = {}
    for kind in ("bodies", "joints", "sites", "cameras", "actuators"):
        renames.update(_shortened(getattr(spec, kind, ())))
    if renames:
        _follow(spec, renames)
    return renames


# Where a model names one of its own parts: the fields that have to follow a rename.
_REFERENCES = {
    "actuators": ("target", "refsite", "slidersite"),
    "sensors": ("objname", "refname"),
    "pairs": ("geomname1", "geomname2"),
    "equalities": ("name1", "name2"),
    "excludes": ("bodyname1", "bodyname2"),
}


def _shortened(elements: Iterable[Any]) -> dict[str, str]:
    """Each element renamed to the tail of its path where that is unambiguous; old name -> new."""
    named = [element for element in elements if element.name]
    tails = Counter(element.name.rsplit("/", 1)[-1] for element in named)
    taken = {element.name for element in named}
    renames: dict[str, str] = {}
    for element in named:
        old = element.name
        tail = old.rsplit("/", 1)[-1]
        if tail == old or tails[tail] > 1 or tail in taken:
            continue
        taken.discard(old)
        taken.add(tail)
        element.name = tail
        renames[old] = tail
    return renames


def _follow(spec: Any, renames: Mapping[str, str]) -> None:
    for kind, fields in _REFERENCES.items():
        for element in getattr(spec, kind, ()):
            for field in fields:
                referred = getattr(element, field, None)
                if referred in renames:
                    setattr(element, field, renames[referred])


def _sample(available: list[str], most: int = 6) -> str:
    """Enough of what there was to see the shape of the names, without printing a whole scene."""
    shown = sorted(available)[:most]
    rest = len(available) - len(shown)
    return ", ".join(shown) + (f", ... ({rest} more)" if rest > 0 else "")
