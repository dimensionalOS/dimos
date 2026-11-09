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

MuJoCo names everything by its full prim path: the body a URDF called ``pelvis`` arrives as
``/g1/Joints/pelvis``. Exact match wins; otherwise the last path segment is what a short name
refers to, and ambiguity is an error rather than a guess.

That is not enough for a robot, because its names leave the simulation as entity ids and tf
frames, so a robot's elements are renamed before it is attached (``shorten``). The scene keeps
its own names: nothing addresses a room by short name.
"""

from collections import Counter
from collections.abc import Iterable, Mapping, Sequence
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


def short_names(names: Iterable[str]) -> dict[str, str]:
    """Each name mapped to its last path segment, where that segment names only one of them.

    For the handful of things in a *scene* that are addressed rather than merely drawn: a crate
    with a free joint arrives called ``/warehouse/Geometry/crate``. The room's own walls are not
    renamed -- hundreds of them, nothing addresses one, and two ending alike is likely -- so this
    is applied to the entities and not to the model.
    """
    full = list(names)
    tails = Counter(name.rsplit("/", 1)[-1] for name in full)
    return {name: tail if tails[tail := name.rsplit("/", 1)[-1]] == 1 else name for name in full}


def shorten(spec: Any, ids: Iterable[str] = ()) -> dict[str, str]:
    """Rename a loaded model's parts to the names a description uses. Returns what was renamed.

    What a robot's parts are called leaves the engine: attaching a robot under its id names every
    body ``g1/<engine's name>``, which is the entity id an action addresses and the tf frame a
    viewer places. Off a USD layer that name is the prim path, so the world published
    ``dog//toy/Geometry/base`` and ``root_body="base"`` found nothing.

    **References move with the names**: a model names its own parts as strings, and renaming an
    element does not touch them. The toy's motor kept pointing at the joint's old path, so the
    compiled world came out with no actuators and nothing to say why.

    Geoms are left alone -- nothing addresses one by short name, and two ending alike is likely.

    ``ids`` names the robots a *composed* world holds, which keeps three of one robot apart: a
    grafted subtree is nested under a prim named after its mount frame, so the id is in the path,
    and an element under a named robot becomes ``<id>/<tail>``. Given ids, only what belongs to one
    is renamed; given none, everything is, which is a robot's own layer before attaching.

    Ambiguity is left alone rather than guessed at: two elements *of one kind* whose paths end
    alike keep what they came with, and ``resolve`` still finds those.
    """
    # A kind at a time, because MuJoCo's namespaces are per kind: the toy robot's hinge and the
    # motor driving it are both called "neck", and judging them together made each the other's
    # ambiguity, so neither was shortened and the actuator was left pointing at a path.
    owners = list(ids)
    renames: dict[str, str] = {}
    for kind in ("bodies", "joints", "sites", "cameras", "actuators"):
        renames.update(_shortened(getattr(spec, kind, ()), owners))
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


def _shortened(elements: Iterable[Any], owners: Sequence[str] = ()) -> dict[str, str]:
    """Each element renamed to the name its description uses, where that is unambiguous. See
    ``shorten``; ``owners`` are the robot ids a composed world holds.
    """
    named = [element for element in elements if element.name]
    tails = Counter(_wanted(element.name, owners) for element in named)
    taken = {element.name for element in named}
    renames: dict[str, str] = {}
    for element in named:
        old = element.name
        tail = _wanted(old, owners)
        if tail == old or tails[tail] > 1 or tail in taken:
            continue
        taken.discard(old)
        taken.add(tail)
        element.name = tail
        renames[old] = tail
    return renames


def _wanted(name: str, owners: Sequence[str]) -> str:
    """The name a description would use for the element the engine calls ``name``, or ``name``
    itself where it belongs to no named robot in a world that has some -- which ``_shortened``
    reads as "leave it alone".
    """
    tail = name.rsplit("/", 1)[-1]
    for owner in owners:
        if f"/{owner}/" in name or name.startswith(f"{owner}/"):
            return f"{owner}/{tail}"
    return name if owners else tail


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
