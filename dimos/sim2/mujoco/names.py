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


from collections import Counter
from collections.abc import Iterable, Mapping, Sequence
from typing import Any


def resolve(names: Iterable[str], wanted: str) -> str:
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
    try:
        return resolve(names, wanted)
    except KeyError:
        return None


def short_names(names: Iterable[str]) -> dict[str, str]:
    full = list(names)
    tails = Counter(name.rsplit("/", 1)[-1] for name in full)
    return {name: tail if tails[tail := name.rsplit("/", 1)[-1]] == 1 else name for name in full}


def shorten(spec: Any, ids: Iterable[str] = ()) -> dict[str, str]:
    owners = list(ids)
    renames: dict[str, str] = {}
    for kind in ("bodies", "joints", "sites", "cameras", "actuators"):
        renames.update(_shortened(getattr(spec, kind, ()), owners))
    if renames:
        _follow(spec, renames)
    return renames


_REFERENCES = {
    "actuators": ("target", "refsite", "slidersite"),
    "sensors": ("objname", "refname"),
    "pairs": ("geomname1", "geomname2"),
    "equalities": ("name1", "name2"),
    "excludes": ("bodyname1", "bodyname2"),
}


def _shortened(elements: Iterable[Any], owners: Sequence[str] = ()) -> dict[str, str]:
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
                if isinstance(referred, str) and referred in renames:
                    setattr(element, field, renames[referred])


def _sample(available: list[str], most: int = 6) -> str:
    shown = sorted(available)[:most]
    rest = len(available) - len(shown)
    return ", ".join(shown) + (f", ... ({rest} more)" if rest > 0 else "")
